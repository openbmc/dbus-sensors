#include "NVMeBasicContext.hpp"

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/thread/thread.hpp>

#include <cassert>
#include <cerrno>
#include <cstdio>

extern "C"
{
#include <err.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

/*
 * Terrible implementation of IO threads.
 *
 * Bit of a straw-person to flush out better ideas.
 */

static int nvme_mi_basic_query_exec(int dev, int in, int out)
{
    uint8_t buf[1 + 256 + 1]; /* len + buf + PEC */
    uint8_t addr, cmd, len;
    uint8_t* cursor;
    ssize_t egress;
    int32_t size;
    ssize_t rc;

    /* Read the command parameters: (address, command, length) */
    if ((rc = ::read(in, &addr, sizeof(addr))) != sizeof(addr))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        if (errno)
            warn("Failed to read device address");
        else
            warnx("In pipe %d is closed", in);
        goto cleanup_fds;
    }

    if ((rc = ::read(in, &cmd, sizeof(cmd))) != sizeof(cmd))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        if (errno)
            warn("Failed to read command");
        else
            warnx("In pipe %d is closed", in);
        goto cleanup_fds;
    }

    if ((rc = ::read(in, &len, sizeof(len))) != sizeof(len))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        if (errno)
            warn("Failed to read block length");
        else
            warnx("In pipe %d is closed", in);
        goto cleanup_fds;
    }

    /* Select the target device */
    if (::ioctl(dev, I2C_SLAVE, addr) == -1)
    {
        rc = -errno;
        warn("Failed to configure device address");
        goto cleanup_fds;
    }

    /* Issue the NVMe MI basic command */
    memset(buf, 0, sizeof(buf));
    if ((size = i2c_smbus_read_block_data(dev, cmd, buf)) < 0)
    {
        rc = size;
        warn("Failed to read block data");
        goto cleanup_fds;
    }

    len = size;

    /* Write the response out */
    if ((rc = ::write(out, &len, sizeof(len))) != sizeof(len))
    {
        assert(rc < 1);
        rc = rc ? -errno : -EIO;
        warn("Failed to write block length to out pipe");
        goto cleanup_fds;
    }

    cursor = buf;
    while (len > 0)
    {
        if ((egress = ::write(out, cursor, len)) == -1)
        {
            rc = -errno;
            warn("Failed to write block data to out pipe");
            goto cleanup_fds;
        }

        cursor += egress;
        len -= egress;
    }

    rc = 0;

cleanup_fds:
    if (::close(dev) == -1)
    {
        warn("Failed to close device descriptor");
    }

    return rc;
}

static int nvme_mi_basic_query(boost::asio::io_service& io, int dev,
                               std::shared_ptr<uint8_t[3]> command,
                               std::shared_ptr<uint8_t[7]> response,
                               std::shared_ptr<NVMeContext> ctx)
{
    int request_pipe[2], response_pipe[2];
    int rc;

    /* Set up inter-thread communication */
    if ((rc = ::pipe(request_pipe)) == -1)
        return -errno;

    if ((rc = ::pipe(response_pipe)) == -1)
    {
        rc = -errno;
        if (::close(request_pipe[0]) == -1)
        {
            warn("Failed to close write fd of request pipe");
        }
        if (::close(request_pipe[1]) == -1)
        {
            warn("Failed to close read fd of request pipe");
        }
        return rc;
    }

    /* Give the pipe endpoints some sensible names */
    int cmd_out = request_pipe[1];
    int exec_in = request_pipe[0];
    int exec_out = response_pipe[1];
    int cmd_in = response_pipe[0];

    /* Set up to issue the NVMe MI basic command */
    command[0] = 0x6a;
    command[1] = 0x00;
    command[2] = 0x06;
    auto req_stream =
        std::make_shared<boost::asio::posix::stream_descriptor>(io, cmd_out);
    boost::asio::async_write(*req_stream, boost::asio::buffer(command.get(), 3),
                             [](boost::system::error_code, std::size_t) {});

    /* Start the IO thread */
    /* XXX: Use a thread pool? */
    boost::thread thread([dev, exec_in, exec_out, req_stream]() {
        int rc;

        rc = nvme_mi_basic_query_exec(dev, exec_in, exec_out);
        if (rc < 0)
        {
            warnx("Failed to process NVMe MI basic query: %d", rc);
        }

        req_stream->close();
    });
    thread.detach();

    /* Dispatch the response for parsing */
    auto resp_stream =
        std::make_shared<boost::asio::posix::stream_descriptor>(io, cmd_in);
    boost::asio::async_read(
        *resp_stream, boost::asio::buffer(response.get(), 7),
        [ctx, response, resp_stream](boost::system::error_code ec,
                                     std::size_t length) {
            resp_stream->close();

            if (ec)
            {
                warnx("Got error code");
                return;
            }

            if (length < 2)
            {
                warnx("Invalid message length: %zu", length);
                return;
            }

            ctx->processResponse(&response[1], response[0]);
        });

    return 0;
}

NVMeBasicContext::NVMeBasicContext(boost::asio::io_service& io, int rootBus) :
    NVMeContext::NVMeContext(io, rootBus), io(io)
{}

NVMeBasicContext::~NVMeBasicContext()
{
    close();
}

void NVMeBasicContext::close()
{
    this->NVMeContext::close();
}

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
                warnx("%s\n", errorCode.message().c_str());
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
        warnx("Failed to format device path for bus %d", sensor->bus);
        return;
    }

    if ((dev = ::open(devpath, O_RDWR)) == -1)
    {
        warn("Failed to open bus device %s", devpath);
        return;
    }

    std::shared_ptr<uint8_t[3]> command = std::make_shared<uint8_t[3]>();
    std::shared_ptr<uint8_t[7]> response = std::make_shared<uint8_t[7]>();

    rc = nvme_mi_basic_query(io, dev, command, response, shared_from_this());
    if (rc < 0)
    {
        warn("Failed to query NVMe device on bus %d", sensor->bus);
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
        warn("Bad message received");
        return;
    }

    if (len < 3)
    {
        warn("Invalid message length: %zu", len);
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
