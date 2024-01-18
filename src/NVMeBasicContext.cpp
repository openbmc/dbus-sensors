#include "NVMeBasicContext.hpp"

#include <endian.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <FileHandle.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
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

static std::shared_ptr<std::array<uint8_t, 6>>
    encodeBasicQuery(int bus, uint8_t device, uint8_t offset)
{
    if (bus < 0)
    {
        throw std::domain_error("Invalid bus argument");
    }

    /* bus + address + command */
    uint32_t busle = htole32(static_cast<uint32_t>(bus));
    auto command =
        std::make_shared<std::array<uint8_t, sizeof(busle) + 1 + 1>>();
    memcpy(command->data(), &busle, sizeof(busle));
    (*command)[sizeof(busle) + 0] = device;
    (*command)[sizeof(busle) + 1] = offset;

    return command;
}

static void decodeBasicQuery(const std::array<uint8_t, 6>& req, int& bus,
                             uint8_t& device, uint8_t& offset)
{
    uint32_t busle = 0;

    memcpy(&busle, req.data(), sizeof(busle));
    bus = le32toh(busle);
    device = req[sizeof(busle) + 0];
    offset = req[sizeof(busle) + 1];
}

static void execBasicQuery(int bus, uint8_t addr, uint8_t cmd,
                           std::vector<uint8_t>& resp)
{
    int32_t size = 0;
    std::filesystem::path devpath = "/dev/i2c-" + std::to_string(bus);

    try
    {
        FileHandle fileHandle(devpath);

        /* Select the target device */
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if (::ioctl(fileHandle.handle(), I2C_SLAVE, addr) == -1)
        {
            std::cerr << "Failed to configure device address 0x" << std::hex
                      << (int)addr << " for bus " << std::dec << bus << ": "
                      << strerror(errno) << "\n";
            resp.resize(0);
            return;
        }

        resp.resize(UINT8_MAX + 1);

        /* Issue the NVMe MI basic command */
        size = i2c_smbus_read_block_data(fileHandle.handle(), cmd, resp.data());
        if (size < 0)
        {
            std::cerr << "Failed to read block data from device 0x" << std::hex
                      << (int)addr << " on bus " << std::dec << bus << ": "
                      << strerror(errno) << "\n";
            resp.resize(0);
        }
        else if (size > UINT8_MAX + 1)
        {
            std::cerr << "Unexpected message length from device 0x" << std::hex
                      << (int)addr << " on bus " << std::dec << bus << ": "
                      << size << " (" << UINT8_MAX << ")\n";
            resp.resize(0);
        }
        else
        {
            resp.resize(size);
        }
    }
    catch (const std::out_of_range& e)
    {
        std::cerr << "Failed to create file handle for bus " << std::dec << bus
                  << ": " << e.what() << "\n";
        resp.resize(0);
    }
}

static ssize_t processBasicQueryStream(FileHandle& in, FileHandle& out)
{
    std::vector<uint8_t> resp{};
    ssize_t rc = 0;

    while (true)
    {
        uint8_t device = 0;
        uint8_t offset = 0;
        uint8_t len = 0;
        int bus = 0;

        /* bus + address + command */
        std::array<uint8_t, sizeof(uint32_t) + 1 + 1> req{};

        /* Read the command parameters */
        ssize_t rc = ::read(in.handle(), req.data(), req.size());
        if (rc != static_cast<ssize_t>(req.size()))
        {
            std::cerr << "Failed to read request from in descriptor "
                      << strerror(errno) << "\n";
            if (rc != 0)
            {
                return -errno;
            }
            return -EIO;
        }

        decodeBasicQuery(req, bus, device, offset);

        /* Execute the query */
        execBasicQuery(bus, device, offset, resp);

        /* Write out the response length */
        len = resp.size();
        rc = ::write(out.handle(), &len, sizeof(len));
        if (rc != sizeof(len))
        {
            std::cerr << "Failed to write block (" << std::dec << len
                      << ") length to out descriptor: "
                      << strerror(static_cast<int>(-rc)) << "\n";
            if (rc != 0)
            {
                return -errno;
            }
            return -EIO;
        }

        /* Write out the response data */
        std::vector<uint8_t>::iterator cursor = resp.begin();
        while (cursor != resp.end())
        {
            size_t lenRemaining = std::distance(cursor, resp.end());
            ssize_t egress = ::write(out.handle(), &(*cursor), lenRemaining);
            if (egress == -1)
            {
                std::cerr << "Failed to write block data of length " << std::dec
                          << lenRemaining << " to out pipe: " << strerror(errno)
                          << "\n";
                if (rc != 0)
                {
                    return -errno;
                }
                return -EIO;
            }

            cursor += egress;
        }
    }

    return rc;
}

/* Throws std::error_code on failure */
/* FIXME: Probably shouldn't do fallible stuff in a constructor */
NVMeBasicContext::NVMeBasicContext(boost::asio::io_context& io, int rootBus) :
    NVMeContext::NVMeContext(io, rootBus), io(io), reqStream(io), respStream(io)
{
    std::array<int, 2> responsePipe{};
    std::array<int, 2> requestPipe{};

    /* Set up inter-thread communication */
    if (::pipe(requestPipe.data()) == -1)
    {
        std::cerr << "Failed to create request pipe: " << strerror(errno)
                  << "\n";
        throw std::error_code(errno, std::system_category());
    }

    if (::pipe(responsePipe.data()) == -1)
    {
        std::cerr << "Failed to create response pipe: " << strerror(errno)
                  << "\n";

        if (::close(requestPipe[0]) == -1)
        {
            std::cerr << "Failed to close write fd of request pipe: "
                      << strerror(errno) << "\n";
        }

        if (::close(requestPipe[1]) == -1)
        {
            std::cerr << "Failed to close read fd of request pipe: "
                      << strerror(errno) << "\n";
        }

        throw std::error_code(errno, std::system_category());
    }

    reqStream.assign(requestPipe[1]);
    FileHandle streamIn(requestPipe[0]);
    FileHandle streamOut(responsePipe[1]);
    respStream.assign(responsePipe[0]);

    thread = std::jthread([streamIn{std::move(streamIn)},
                           streamOut{std::move(streamOut)}]() mutable {
        ssize_t rc = 0;

        if ((rc = processBasicQueryStream(streamIn, streamOut)) < 0)
        {
            std::cerr << "Failure while processing query stream: "
                      << strerror(static_cast<int>(-rc)) << "\n";
        }

        std::cerr << "Terminating basic query thread\n";
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

    auto command = encodeBasicQuery(sensor->bus, sensor->address, 0x00);

    /* Issue the request */
    boost::asio::async_write(
        reqStream, boost::asio::buffer(command->data(), command->size()),
        [command](boost::system::error_code ec, std::size_t) {
        if (ec)
        {
            std::cerr << "Got error writing basic query: " << ec << "\n";
        }
        });

    auto response = std::make_shared<boost::asio::streambuf>();
    response->prepare(1);

    /* Gather the response and dispatch for parsing */
    boost::asio::async_read(
        respStream, *response,
        [response](const boost::system::error_code& ec, std::size_t n) {
        if (ec)
        {
            std::cerr << "Got error completing basic query: " << ec << "\n";
            return static_cast<std::size_t>(0);
        }

        if (n == 0)
        {
            return static_cast<std::size_t>(1);
        }

        std::istream is(response.get());
        size_t len = static_cast<std::size_t>(is.peek());

        if (n > len + 1)
        {
            std::cerr << "Query stream has become unsynchronised: "
                      << "n: " << n << ", "
                      << "len: " << len << "\n";
            return static_cast<std::size_t>(0);
        }

        if (n == len + 1)
        {
            return static_cast<std::size_t>(0);
        }

        if (n > 1)
        {
            return len + 1 - n;
        }

        response->prepare(len);
        return len;
        },
        [weakSelf{weak_from_this()}, sensor, response](
            const boost::system::error_code& ec, std::size_t length) mutable {
        if (ec)
        {
            std::cerr << "Got error reading basic query: " << ec << "\n";
            return;
        }

        if (length == 0)
        {
            std::cerr << "Invalid message length: " << length << "\n";
            return;
        }

        if (auto self = weakSelf.lock())
        {
            /* Deserialise the response */
            response->consume(1); /* Drop the length byte */
            std::istream is(response.get());
            std::vector<char> data(response->size());
            is.read(data.data(), response->size());

            /* Update the sensor */
            self->processResponse(sensor, data.data(), data.size());

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
                                       void* msg, size_t len)
{
    if (msg == nullptr || len < 6)
    {
        sensor->incrementError();
        return;
    }

    uint8_t* messageData = static_cast<uint8_t*>(msg);

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
