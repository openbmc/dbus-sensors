#include "Inventory.hpp"

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <exception>
#include <memory>
#include <string>

constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName,
    const gpu::NvidiaVDMDeviceType deviceType) : name(escapeName(inventoryName))
{
    if (deviceType == gpu::NvidiaVDMDeviceType::GPU)
    {
        std::string path = inventoryPrefix + name;
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
