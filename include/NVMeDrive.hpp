#include <xyz/openbmc_project/Inventory/Item/Drive/server.hpp>

using DriveBase =
    sdbusplus::xyz::openbmc_project::Inventory::Item::server::Drive;
class NVMeDrive : public DriveBase
{
  public:
    NVMeDrive(sdbusplus::bus_t& bus, const char* path) : DriveBase(bus, path)
    {}
};