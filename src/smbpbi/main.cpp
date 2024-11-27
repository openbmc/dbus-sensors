/// SMBPBI: SMBus Post-Box Interface. It's a "virtual EEPROM" I2C endpoint.
/// NVIDIA uses this on their HMCs to give lower latency access to important
/// sensors, to avoid the whole TCP/IP-over-USB stack when using Redfish.

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <FileHandle.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <xyz/openbmc_project/Configuration/SmbusPostBox/client.hpp>
#include <xyz/openbmc_project/ObjectMapper/client.hpp>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

using ObjectMapper = sdbusplus::client::xyz::openbmc_project::ObjectMapper<>;
using SmbusPostBox =
    sdbusplus::client::xyz::openbmc_project::configuration::SmbusPostBox<>;

struct LinuxI2CBus : FileHandle
{
    void setSlaveAddress(int address) const
    {
        if (::ioctl(handle(), I2C_SLAVE, address) < 0)
        {
            auto error = errno;
            lg2::error("Unable to set slave address @{ADDRESS}: {MSG}",
                       "ADDRESS", address, "MSG", strerror(error));
            throw std::system_error(error, std::generic_category(),
                                    "Unable to set i2c slave address");
        }
    }
};

struct LinuxI2CDevice : LinuxI2CBus
{
    static LinuxI2CDevice open(std::string_view path, int address)
    {
        // NOTE: FileHandle opens with O_RDONLY for some reason.
        // [pid 29649] openat(AT_FDCWD, "/dev/i2c-13",
        // O_RDONLY|O_LARGEFILE|0x18) = 6
        auto fd = ::open(path.data(), O_RDWR | O_CLOEXEC);
        if (fd == -1)
        {
            auto error = errno;
            lg2::error("Unable to open {PATH}: {MSG}", "PATH", path, "MSG",
                       strerror(error));
            throw std::system_error(error, std::generic_category(),
                                    "Unable to open i2c chardev file");
        }
        auto i2c = LinuxI2CDevice(LinuxI2CBus(FileHandle(fd)));
        i2c.setSlaveAddress(address);
        return i2c;
    }
};

static void readSmbusPostBox(const LinuxI2CDevice& i2c, int offset,
                             std::span<std::byte> data)
{
    std::array<std::byte, sizeof(uint16_t)> buf{};
    std::span<std::byte> offsetBytes;
    if (offset <= std::numeric_limits<uint8_t>::max())
    {
        buf[0] = std::byte(offset);
        offsetBytes = std::span(buf.begin(), 1);
    }
    else
    {
        buf[0] = std::byte((offset >> 8) & 0xff);
        buf[1] = std::byte(offset & 0xff);
        offsetBytes = std::span(buf.begin(), 2);
    }
    ssize_t n = 0;
    if ((n = write(i2c.handle(), offsetBytes.data(), offsetBytes.size())) !=
        ssize_t(offsetBytes.size()))
    {
        auto error = errno;
        lg2::error("Wrote {N}/{M} bytes of the virtual EEPROM offset: {MSG}",
                   "N", n, "M", offsetBytes.size(), "MSG", strerror(error));
        throw std::system_error(error, std::generic_category(),
                                "Unable to write smbus post-box offset");
    }
    if ((n = read(i2c.handle(), data.data(), data.size())) !=
        ssize_t(data.size()))
    {
        auto error = errno;
        lg2::error("Read {N}/{M} bytes from +{OFFSET}: {MSG}", "N", n, "M",
                   offsetBytes.size(), "OFFSET", offset, "MSG",
                   strerror(error));
        throw std::system_error(error, std::generic_category(),
                                "Unable to read smbus post-box data");
    }
}

static auto postBoxSize(SmbusPostBox::Format format) -> size_t
{
    switch (format)
    {
        case SmbusPostBox::Format::I24F8:
            return sizeof(uint32_t);
    }
    throw std::logic_error("Unimplemented SmbusPostBox format");
}

static auto parseI24F8(std::span<const std::byte> bytes) -> double
{
    if (bytes.size() < sizeof(uint32_t))
    {
        throw std::invalid_argument("I24F8 requires 32 bits");
    }

    auto byte0 = int32_t(bytes[0]);
    auto byte1 = int32_t(bytes[1]);
    auto byte2 = int32_t(bytes[2]);
    auto byte3 = int32_t(bytes[3]);

    auto integer = double((byte3 << 24 | byte2 << 16 | byte1 << 8) >> 8);
    auto fraction = (integer < 0 ? -1.0 : 1.0) * double(byte0) / 256.0;
    return integer + fraction;
}

auto asyncMain(sdbusplus::async::context& ctx) -> sdbusplus::async::task<>
{
    // Find all of the smbus post-box interface configuration objects within
    // entity-manager inventory.
    auto bigBuf = std::array<std::byte, sizeof(uint64_t)>{};
    auto mapper = ObjectMapper(ctx)
                      .service(ObjectMapper::default_service)
                      .path(ObjectMapper::instance_path);
    auto configPaths = co_await mapper.get_sub_tree(
        "/xyz/openbmc_project/inventory", 0, {SmbusPostBox::interface});
    for (const auto& [configPath, services] : configPaths)
    {
        // NOTE: Only one service should be exporting each sensor.
        if (services.size() != 1)
        {
            lg2::error(
                "Expected exactly one service to provide {PATH}, {INTERFACE}",
                "PATH", configPath, "INTERFACE", SmbusPostBox::interface);
            throw std::runtime_error(
                "Multiple services providing the same SmbusPostBox configuration object path");
        }
        auto service = services.begin()->first;
        auto postBox = SmbusPostBox(ctx).service(service).path(configPath);
        auto bus = co_await postBox.bus();
        auto address = co_await postBox.address();
        auto offset = co_await postBox.offset();
        auto format = co_await postBox.format();
        auto name = co_await postBox.name();
        auto size = postBoxSize(format);
        auto i2c =
            LinuxI2CDevice::open("/dev/i2c-" + std::to_string(bus), address);
        auto buf = std::span(bigBuf.begin(), size);
        readSmbusPostBox(i2c, offset, buf);

        auto value = -1.0;
        switch (format)
        {
            case SmbusPostBox::Format::I24F8:
                value = parseI24F8(buf);
                break;
        }

        lg2::debug("/dev/i2c-{BUS} @{ADDRESS} +{OFFSET} {NAME}={VALUE}", "BUS",
                   bus, "ADDRESS", address, "OFFSET", offset, "NAME", name,
                   "VALUE", value);
    }

    co_return;
}

int main()
{
    sdbusplus::async::context ctx;
    ctx.spawn(asyncMain(ctx));
    ctx.run();
    return 0;
}
