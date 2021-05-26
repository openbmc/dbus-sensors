#include "NVMeBasicContext.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <thread>

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

/*
 * Terrible implementation of IO threads.
 *
 * Bit of a straw-person to flush out better ideas.
 */

static int nvmeMiBasicQueryExec(int dev, int in, int out)
{
    /* len + buf + PEC */
    std::array<uint8_t, 1 + UINT8_MAX + 1> buf{};
    std::array<uint8_t, 3> req{};
    uint8_t addr, cmd, len;
    uint8_t* cursor;
    ssize_t egress;
    int32_t size;
    ssize_t rc;

    /* Read the command parameters: (address, command, length) */
    if ((rc = ::read(in, req.data(), req.size())) !=
        static_cast<ssize_t>(req.size()))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        if (errno)
        {
            std::cerr << "Failed to read request: " << strerror(errno) << "\n";
        }
        goto cleanup_fds;
    }

    /* Deserialise */
    addr = req[0];
    cmd = req[1];
    len = req[2];

    /* Select the target device */
    if (::ioctl(dev, I2C_SLAVE, addr) == -1)
    {
        rc = -errno;
        std::cerr << "Failed to configure device address: " << strerror(errno)
                  << "\n";
        goto cleanup_fds;
    }

    /* Issue the NVMe MI basic command */
    if ((size = i2c_smbus_read_block_data(dev, cmd, buf.data())) < 0)
    {
        rc = size;
        std::cerr << "Failed to read block data: " << strerror(errno) << "\n";
        goto cleanup_fds;
    }
    else if (size > UINT8_MAX)
    {
        /* Shouldn't be possible */
        rc = -EBADMSG;
        std::cerr << "Unexpected message length: " << size << " (" << UINT8_MAX
                  << ")\n";
        goto cleanup_fds;
    }

    len = size;

    /* Write the response out */
    if ((rc = ::write(out, &len, sizeof(len))) != sizeof(len))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        std::cerr << "Failed to write block length to out pipe: "
                  << strerror(errno) << "\n";
        goto cleanup_fds;
    }

    cursor = buf.data();
    while (len > 0)
    {
        if ((egress = ::write(out, cursor, len)) == -1)
        {
            rc = -errno;
            std::cerr << "Failed to write block data to out pipe: "
                      << strerror(errno) << "\n";
            goto cleanup_fds;
        }

        cursor += egress;
        len -= egress;
    }

    rc = 0;

cleanup_fds:
    if (::close(dev) == -1)
    {
        std::cerr << "Failed to close device descriptor: " << strerror(errno)
                  << "\n";
    }

    return rc;
}

static int
    nvmeMiBasicQuery(boost::asio::io_service& io, int dev,
                     const std::shared_ptr<std::array<uint8_t, 3>>& command,
                     const std::shared_ptr<std::array<uint8_t, 7>>& response,
                     const std::shared_ptr<NVMeContext>& ctx)
{
    std::array<int, 2> requestPipe;
    std::array<int, 2> responsePipe;

    /* Set up inter-thread communication */
    if (::pipe(requestPipe.data()) == -1)
    {
        return -errno;
    }

    if (::pipe(responsePipe.data()) == -1)
    {
        int rc = -errno;

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

        return rc;
    }

    /* Give the pipe endpoints some sensible names */
    int cmdOut = requestPipe[1];
    int execIn = requestPipe[0];
    int execOut = responsePipe[1];
    int cmdIn = responsePipe[0];

    auto reqStream =
        std::make_shared<boost::asio::posix::stream_descriptor>(io, cmdOut);
    boost::asio::async_write(
        *reqStream, boost::asio::buffer(command->data(), command->size()),
        [](boost::system::error_code, std::size_t) {});

    /* Start the IO thread */
    /* XXX: Use a thread pool? */
    std::thread thread([dev, execIn, execOut, reqStream]() {
        int rc;

        if ((rc = nvmeMiBasicQueryExec(dev, execIn, execOut)) < 0)
        {
            std::cerr << "Failed to process NVMe MI basic query: " << rc
                      << "\n";
        }

        /* Clean up requestPipe */
        reqStream->close();
        if (::close(execIn) == -1)
        {
            std::cerr << "Failed to close execIn\n";
        }
    });
    thread.detach();

    /* Dispatch the response for parsing */
    auto respStream =
        std::make_shared<boost::asio::posix::stream_descriptor>(io, cmdIn);
    boost::asio::async_read(
        *respStream, boost::asio::buffer(response->data(), response->size()),
        [ctx, response, execOut, respStream](boost::system::error_code ec,
                                             std::size_t length) {
            /* Clean up responsePipe */
            respStream->close();
            if (::close(execOut) == -1)
            {
                std::cerr << "Failed to close execOut: " << strerror(errno)
                          << "\n";
            }

            if (ec)
            {
                std::cerr << "Got error code\n";
                return;
            }

            if (length < 2)
            {
                std::cerr << "Invalid message length: " << length << "\n";
                return;
            }

            ctx->processResponse(&response->data()[1], response->data()[0]);
        });

    return 0;
}

NVMeBasicContext::NVMeBasicContext(boost::asio::io_service& io, int rootBus) :
    NVMeContext::NVMeContext(io, rootBus), io(io)
{}

void NVMeBasicContext::pollNVMeDevices()
{
    scanTimer.expires_from_now(boost::posix_time::seconds(1));
    scanTimer.async_wait(
        [self{shared_from_this()}](const boost::system::error_code errorCode) {
            if (errorCode == boost::asio::error::operation_aborted)
            {
                return;
            }

            if (errorCode)
            {
                std::cerr << errorCode.message() << "\n";
                return;
            }

            self->readAndProcessNVMeSensor();
            self->pollNVMeDevices();
        });
}

void NVMeBasicContext::readAndProcessNVMeSensor()
{
    char devpath[PATH_MAX]{};
    int dev;
    int rc;

    if (sensors.empty())
    {
        return;
    }

    std::shared_ptr<NVMeSensor>& sensor = sensors.front();

    rc = snprintf(devpath, sizeof(devpath), "/dev/i2c-%d", sensor->bus);
    if ((size_t)rc > sizeof(devpath))
    {
        std::cerr << "Failed to format device path for bus " << sensor->bus
                  << "\n";
        return;
    }

    if ((dev = ::open(devpath, O_RDWR)) == -1)
    {
        std::cerr << "Failed to open bus device " << devpath << ": "
                  << strerror(errno) << "\n";
        return;
    }

    /* Serialise the command parameters for the IO thread */
    std::shared_ptr<std::array<uint8_t, 3>> command =
        std::make_shared<std::array<uint8_t, 3>>();

    (*command)[0] = 0x6a; /* I2C address of the device */
    (*command)[1] = 0x00; /* Offset for the start of the block read */
    (*command)[2] = 0x06; /* Length of the block read */

    /* Response buffer */
    std::shared_ptr<std::array<uint8_t, 7>> response =
        std::make_shared<std::array<uint8_t, 7>>();

    rc = nvmeMiBasicQuery(io, dev, command, response, shared_from_this());
    if (rc < 0)
    {
        std::cerr << "Failed to query NVMe device on bus " << sensor->bus
                  << "\n";
    }
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

void NVMeBasicContext::processResponse(void* msg, size_t len)
{
    if (msg == nullptr)
    {
        std::cerr << "Bad message received\n";
        return;
    }

    if (len < 3)
    {
        std::cerr << "Invalid message length: " << len << "\n";
        return;
    }

    uint8_t* messageData = static_cast<uint8_t*>(msg);

    std::shared_ptr<NVMeSensor> sensor = sensors.front();
    double value = getTemperatureReading(messageData[2]);

    if (std::isfinite(value))
    {
        sensor->updateValue(value);
    }
    else
    {
        sensor->markAvailable(false);
        sensor->incrementError();
    }

    sensors.pop_front();
    sensors.emplace_back(sensor);
}
