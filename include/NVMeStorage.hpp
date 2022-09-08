#include <xyz/openbmc_project/Inventory/Item/Storage/server.hpp>

using StorageBase =
    sdbusplus::xyz::openbmc_project::Inventory::Item::server::Storage;
class NVMeStorage : public StorageBase
{
  public:
    NVMeStorage(sdbusplus::bus_t& bus, const char* path) :
        StorageBase(bus, path)
    {
        emit_added();
    }
    ~NVMeStorage() override
    {
        emit_removed();
    }
};
