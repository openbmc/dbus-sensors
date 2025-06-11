#include "Inventory.hpp"

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>

static constexpr const char* inventoryPrefix =
    "/xyz/openbmc_project/inventory/";
static constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, DeviceType deviceType) :
    name(escapeName(inventoryName))
{
    if (deviceType == DeviceType::GPU)
    {
        std::string path = std::string(inventoryPrefix) + name;
        try
        {
            acceleratorInterface =
                objectServer.add_interface(path, acceleratorIfaceName);
            acceleratorInterface->register_property("Type", std::string("GPU"));
            acceleratorInterface->initialize();
        }
        catch (const std::exception& e)
        {
            lg2::error(
                "Failed to add accelerator interface. path='{PATH}', error='{ERROR}'",
                "PATH", path, "ERROR", e.what());
        }
    }
}
