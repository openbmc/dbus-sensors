#include "NVMeBasicContext.hpp"

#include <endian.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <boost/asio/read.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <system_error>
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
    uint32_t busle;

    memcpy(&busle, req.data(), sizeof(busle));
    bus = le32toh(busle);
    device = req[sizeof(busle) + 0];
    offset = req[sizeof(busle) + 1];
}

static ssize_t execBasicQuery(int bus, uint8_t addr, uint8_t cmd,
                              std::vector<uint8_t>& resp)
{
    char devpath[PATH_MAX]{};
    int32_t size;

    ssize_t rc = snprintf(devpath, sizeof(devpath), "/dev/i2c-%" PRIu32, bus);
    if ((size_t)rc > sizeof(devpath))
    {
        std::cerr << "Failed to format device path for bus " << bus << "\n";
        return -EINVAL;
    }

    int dev = ::open(devpath, O_RDWR);
    if (dev == -1)
    {
        std::cerr << "Failed to open bus device " << devpath << ": "
                  << strerror(errno) << "\n";
        return -errno;
    }

    /* Select the target device */
    if (::ioctl(dev, I2C_SLAVE, addr) == -1)
    {
        rc = -errno;
        std::cerr << "Failed to configure device address: " << strerror(errno)
                  << "\n";
        goto cleanup_fds;
    }

    resp.reserve(UINT8_MAX + 1);

    /* Issue the NVMe MI basic command */
    size = i2c_smbus_read_block_data(dev, cmd, resp.data());
    if (size < 0)
    {
        rc = size;
        std::cerr << "Failed to read block data: " << strerror(errno) << "\n";
        goto cleanup_fds;
    }
    else if (size > UINT8_MAX + 1)
    {
        rc = -EBADMSG;
        std::cerr << "Unexpected message length: " << size << " (" << UINT8_MAX
                  << ")\n";
        goto cleanup_fds;
    }

    rc = size;

cleanup_fds:
    if (::close(dev) == -1)
    {
        std::cerr << "Failed to close device descriptor: " << strerror(errno)
                  << "\n";
    }

    return rc;
}

static ssize_t processBasicQueryStream(int in, int out)
{
    std::vector<uint8_t> resp{};
    ssize_t rc;

    while (true)
    {
        uint8_t device;
        uint8_t offset;
        uint8_t len;
        int bus;

        /* bus + address + command */
        std::array<uint8_t, sizeof(uint32_t) + 1 + 1> req{};

        /* Read the command parameters */
        if ((rc = ::read(in, req.data(), req.size())) !=
            static_cast<ssize_t>(req.size()))
        {
            assert(rc < 1);
            rc = rc ? -errno : -EIO;
            if (errno)
            {
                std::cerr << "Failed to read request: " << strerror(errno)
                          << "\n";
            }
            goto done;
        }

        decodeBasicQuery(req, bus, device, offset);

        /* Execute the query */
        rc = execBasicQuery(bus, device, offset, resp);

        /* Bounds check the response */
        if (rc < 0)
        {
            len = 0;
        }
        else if (rc > UINT8_MAX)
        {
            assert(rc == UINT8_MAX + 1);

            /* YOLO: Lop off the PEC */
            len = UINT8_MAX;
        }
        else
        {
            len = rc;
        }

        /* Write out the response length */
        if ((rc = ::write(out, &len, sizeof(len))) != sizeof(len))
        {
            assert(rc < 1);
            rc = rc ? -errno : -EIO;
            std::cerr << "Failed to write block length to out pipe: "
                      << strerror(-rc) << "\n";
            goto done;
        }

        /* Write out the response data */
        uint8_t* cursor = resp.data();
        while (len > 0)
        {
            ssize_t egress;

            if ((egress = ::write(out, cursor, len)) == -1)
            {
                rc = -errno;
                std::cerr << "Failed to write block data to out pipe: "
                          << strerror(errno) << "\n";
                goto done;
            }

            cursor += egress;
            len -= egress;
        }
    }

done:
    if (::close(in) == -1)
    {
        std::cerr << "Failed to close in descriptor: " << strerror(errno)
                  << "\n";
    }

    if (::close(out) == -1)
    {
        std::cerr << "Failed to close out descriptor: " << strerror(errno)
                  << "\n";
    }

    return rc;
}

/* Throws std::error_code on failure */
/* FIXME: Probably shouldn't do fallible stuff in a constructor */
NVMeBasicContext::NVMeBasicContext(boost::asio::io_service& io, int rootBus) :
    NVMeContext::NVMeContext(io, rootBus), io(io), reqStream(io), respStream(io)
{
    std::array<int, 2> responsePipe;
    std::array<int, 2> requestPipe;

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
    int streamIn = requestPipe[0];
    int streamOut = responsePipe[1];
    respStream.assign(responsePipe[0]);

    std::thread thread([streamIn, streamOut]() {
        ssize_t rc;

        if ((rc = processBasicQueryStream(streamIn, streamOut)) < 0)
        {
            std::cerr << "Failure while processing query stream: "
                      << strerror(-rc) << "\n";
        }

        if (::close(streamIn) == -1)
        {
            std::cerr << "Failed to close streamIn descriptor: "
                      << strerror(errno) << "\n";
        }

        if (::close(streamOut) == -1)
        {
            std::cerr << "Failed to close streamOut descriptor: "
                      << strerror(errno) << "\n";
        }

        std::cerr << "Terminating basic query thread\n";
    });
    thread.detach();
}

void NVMeBasicContext::readAndProcessNVMeSensor()
{
    auto response = std::make_shared<boost::asio::streambuf>();

    if (sensors.empty())
    {
        return;
    }

    std::shared_ptr<NVMeSensor>& sensor = sensors.front();

    if (!sensor->readingStateGood())
    {
        sensor->markAvailable(false);
        sensor->updateValue(std::numeric_limits<double>::quiet_NaN());
        return;
    }

    /* Ensure sensor query parameters are sensible */
    if (sensor->bus < 0)
    {
        std::cerr << "Bus index cannot be negative: " << sensor->bus << "\n";

        sensors.pop_front();
        sensors.emplace_back(sensor);

        return;
    }

    auto command = encodeBasicQuery(sensor->bus, 0x6a, 0x00);

    /* Issue the request */
    boost::asio::async_write(
        reqStream, boost::asio::buffer(command->data(), command->size()),
        [&command](boost::system::error_code ec, std::size_t) {
            if (ec)
            {
                std::cerr << "Got error writing basic query: " << ec << "\n";
            }
        });

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
        [self{shared_from_this()},
         response](const boost::system::error_code& ec, std::size_t length) {
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

            if (length == 1)
            {
                std::cerr << "Basic query failed\n";
                return;
            }

            /* Deserialise the response */
            response->consume(1); /* Drop the length byte */
            std::istream is(response.get());
            std::vector<char> data(response->size());
            is.read(data.data(), response->size());

            self->processResponse(data.data(), data.size());
        });
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
                std::cerr << errorCode.message() << "\n";
                return;
            }

            self->readAndProcessNVMeSensor();
            self->pollNVMeDevices();
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

void NVMeBasicContext::processResponse(void* msg, size_t len)
{
    if (msg == nullptr)
    {
        std::cerr << "Bad message received\n";
        return;
    }

    if (len < 6)
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
