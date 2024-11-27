/// SMBPBI: SMBus Post-Box Interface. It's a "virtual EEPROM" I2C endpoint.
/// NVIDIA uses this on their HMCs to give low-er latency access to important
/// sensors, to avoid the whole TCP/IP-over-USB stack when using Redfish.

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>

int main()
{
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx,
                                         "xyz/openbmc_project/smbpbisensor"};
    ctx.request_name("xyz.openbmc_project.SMBPBISensor");
    ctx.run();
    return 0;
}
