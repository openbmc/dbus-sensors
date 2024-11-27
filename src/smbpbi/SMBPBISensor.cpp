/// SMBPBI: SMBus Post-Box Interface. It's a "virtual EEPROM" I2C endpoint.
/// NVIDIA uses this on their HMCs to give lower latency access to important
/// sensors, to avoid the whole TCP/IP-over-USB stack when using Redfish.

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/server/manager.hpp>

int main()
{
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, "xyz/openbmc_project/sensors"};
    ctx.request_name("xyz.openbmc_project.SMBPBISensor");
    ctx.run();
    return 0;
}
