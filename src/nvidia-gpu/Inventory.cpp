#include "Inventory.hpp"

constexpr int maxAttempts = 3;
static constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
static constexpr const char* acceleratorIfaceName = "xyz.openbmc_project.Inventory.Item.Accelerator";
static constexpr const char* assetIfaceName = "xyz.openbmc_project.Inventory.Decorator.Asset";

Inventory::Inventory(const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
                     sdbusplus::asio::object_server& objectServer,
                     const std::string& inventoryName,
                     mctp::MctpRequester& mctpRequester,
                     DeviceType deviceType,
                     uint8_t eid)
    : path(std::string(inventoryPrefix) + inventoryName),
      mctpRequester(mctpRequester),
      deviceType(deviceType),
      eid(eid)
{
    if (deviceType == DeviceType::GPU)
    {
        acceleratorInterface = objectServer.add_interface(
            path, acceleratorIfaceName);
        acceleratorInterface->register_property("Type", std::string("GPU"));
        acceleratorInterface->initialize();
    }
}