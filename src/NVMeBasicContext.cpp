#include "NVMeBasicContext.hpp"

#include <endian.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <boost/asio/buffer.hpp>
#include <boost/asio/experimental/concurrent_channel.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/beast/core/file_posix.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/linux_error.hpp>

#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <format>
#include <system_error>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

/*
 * NVMe-MI Basic Management Command
 *
 * https://nvmexpress.org/wp-content/uploads/NVMe_Management_-_Technical_Note_on_Basic_Management_Command.pdf
 */

struct BasicQueryResponse
{
    boost::system::error_code ec;
    MessageResponse resp;
};

static BasicQueryResponse execBasicQuery(int bus, uint8_t addr, uint8_t cmd)
{
    BasicQueryResponse ret;

    boost::beast::file_posix fileHandle;
    boost::system::error_code ec;
    {
        std::string devpath = std::format("/dev/i2c-{}", bus);
        fileHandle.open(devpath.c_str(), boost::beast::file_mode::read, ec);
        if (ec)
        {
            std::cerr << "Failed to create file handle for bus " << std::dec
                      << bus << ": " << ec << "\n";
            return ret;
        }
    }
    using boost::system::linux_error::linux_errno;
    /* Select the target device */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    if (::ioctl(fileHandle.native_handle(), I2C_SLAVE, addr) == -1)
    {
        std::cerr << "Failed to configure device address 0x" << std::hex
                  << (int)addr << " for bus " << std::dec << bus << ": "
                  << strerror(errno) << "\n";
        ret.ec = boost::system::linux_error::make_error_code(
            static_cast<linux_errno>(errno));
        return ret;
    }

    /* Issue the NVMe MI basic command */
    int32_t size = i2c_smbus_read_block_data(fileHandle.native_handle(), cmd,
                                             ret.resp.buffer.data());
    if (size < 0)
    {
        std::cerr << "Failed to read block data from device 0x" << std::hex
                  << (int)addr << " on bus " << std::dec << bus << ": "
                  << strerror(errno) << "\n";
        ret.ec = boost::system::errc::make_error_code(
            boost::system::errc::invalid_argument);
        return ret;
    }
    if (size > UINT8_MAX + 1)
    {
        std::cerr << "Unexpected message length from device 0x" << std::hex
                  << (int)addr << " on bus " << std::dec << bus << ": " << size
                  << " (" << UINT8_MAX << ")\n";
        ret.ec = boost::system::errc::make_error_code(
            boost::system::errc::invalid_argument);
        return ret;
    }

    ret.resp.validData = std::span(ret.resp.buffer.data(), size);

    return ret;
}

static void
    processMessage(MessageRequest& req,
                   boost::asio::experimental::concurrent_channel<void(
                       boost::system::error_code ec, MessageResponse)>& out)
{
    /* Execute the query */
    BasicQueryResponse resp = execBasicQuery(req.bus, req.device, req.offset);

    out.async_send(resp.ec, resp.resp, [](boost::system::error_code ec) {
        if (ec)
        {
            std::cout << "sending error with message " << ec << "\n";
            return;
        }
    });
}

/* Throws std::error_code on failure */
/* FIXME: Probably shouldn't do fallible stuff in a constructor */
NVMeBasicContext::NVMeBasicContext(boost::asio::io_context& io, int rootBus) :
    NVMeContext::NVMeContext(io, rootBus), io(io), toThread(io, 1),
    fromThread(threadIo, 1)
{
    thread = std::jthread([this]() {
        startThreadReceive();

        threadIo.run();
    });
}

void NVMeBasicContext::startThreadReceive()
{
    toThread.async_receive(
        [this](boost::system::error_code ec, MessageRequest msg) {
        if (ec)
        {
            std::cout << "Failed receiving " << ec << "\n";
            return;
        }
        processMessage(msg, fromThread);
        startThreadReceive();
    });
}

void NVMeBasicContext::readAndProcessNVMeSensor()
{
    if (pollCursor == sensors.end())
    {
        this->pollNVMeDevices();
        return;
    }

    std::shared_ptr<NVMeSensor> sensor = *pollCursor++;

    if (!sensor->readingStateGood())
    {
        sensor->markAvailable(false);
        sensor->updateValue(std::numeric_limits<double>::quiet_NaN());
        readAndProcessNVMeSensor();
        return;
    }

    /* Potentially defer sampling the sensor sensor if it is in error */
    if (!sensor->sample())
    {
        readAndProcessNVMeSensor();
        return;
    }

    MessageRequest command{
        .bus = sensor->bus,
        .device = sensor->address,
        .offset = 0x00,
        .len = 0, // Is this right?
    };

    /* Issue the request */
    toThread.async_send(boost::system::error_code{}, command,
                        [](boost::system::error_code ec) {
        if (ec)
        {
            std::cerr << "Got error writing basic query: " << ec << "\n";
        }
    });

    fromThread.async_receive([weakSelf = weak_from_this(),
                              sensor](boost::system::error_code ec,
                                      MessageResponse response) mutable {
        if (ec)
        {
            std::cerr << "Got error reading basic query: " << ec << "\n";
            return;
        }
        if (auto self = weakSelf.lock())
        {
            /* Update the sensor */
            self->processResponse(sensor, response.validData);

            /* Enqueue processing of the next sensor */
            self->readAndProcessNVMeSensor();
        }
    });
}

void NVMeBasicContext::pollNVMeDevices()
{
    pollCursor = sensors.begin();

    scanTimer.expires_after(std::chrono::seconds(1));
    scanTimer.async_wait([weakSelf{weak_from_this()}](
                             const boost::system::error_code errorCode) {
        if (errorCode == boost::asio::error::operation_aborted)
        {
            return;
        }

        if (errorCode)
        {
            std::cerr << errorCode.message() << "\n";
            return;
        }

        if (auto self = weakSelf.lock())
        {
            self->readAndProcessNVMeSensor();
        }
    });
}

static double getTemperatureReading(int8_t reading)
{
    if (reading == static_cast<int8_t>(0x80) ||
        reading == static_cast<int8_t>(0x81))
    {
        // 0x80 = No temperature data or temperature data is more the 5 s
        // old 0x81 = Temperature sensor failure
        return std::numeric_limits<double>::quiet_NaN();
    }

    return reading;
}

void NVMeBasicContext::processResponse(std::shared_ptr<NVMeSensor>& sensor,
                                       std::span<uint8_t> messageData)
{
    if (messageData.size() < 6)
    {
        sensor->incrementError();
        return;
    }

    uint8_t status = messageData[0];
    if (((status & NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY) != 0) ||
        ((status & NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL) == 0))
    {
        sensor->markFunctional(false);
        return;
    }

    double value = getTemperatureReading(messageData[2]);
    if (!std::isfinite(value))
    {
        sensor->incrementError();
        return;
    }

    sensor->updateValue(value);
}
