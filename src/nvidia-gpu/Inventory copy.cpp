#include "Inventory.hpp"
#include <phosphor-logging/lg2.hpp>

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

    assetIface = objectServer.add_interface(
        path, assetIfaceName);
    assetIface->register_property("PartNumber", std::string{});
    assetIface->register_property("SerialNumber", std::string{});
    assetIface->initialize();
}

void Inventory::update()
{
    static int partNumberAttempts = 0;
    static int serialNumberAttempts = 0;
    requestInventoryProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER, "PartNumber", partNumberAttempts);
    requestInventoryProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, "SerialNumber", serialNumberAttempts);
}

void Inventory::requestInventoryProperty(gpu::InventoryPropertyId propertyId, const std::string& dbusProperty, int& attempts)
{
    int rc = gpu::encodeGetInventoryInformationRequest(0, static_cast<uint8_t>(propertyId), requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode {PROP} request: rc={RC}", "PROP", dbusProperty, "RC", rc);
        return;
    }
    mctpRequester.sendRecvMsg(
        eid, requestBuffer, responseBuffer,
        [this, propertyId, dbusProperty, &attempts](int sendRecvMsgResult) mutable {
            handleInventoryPropertyResponse(propertyId, dbusProperty, attempts, sendRecvMsgResult);
        });
}

void Inventory::handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId, const std::string& dbusProperty, int& attempts, int sendRecvMsgResult)
{
    bool success = false;
    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryInfo info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            responseBuffer, cc, reasonCode, propertyId, info);
        if (rc == 0 && cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            std::string value = std::get<std::string>(info);
            if (assetIface)
            {
                assetIface->set_property(dbusProperty, value);
            }
            success = true;
        }
    }

    if (!success)
    {
        attempts++;
        lg2::error("Failed to update {PROP} (attempt {ATTEMPT}/{MAX})", "PROP", dbusProperty, "ATTEMPT", attempts, "MAX", maxAttempts);
        if (attempts < maxAttempts)
        {
            requestInventoryProperty(propertyId, dbusProperty, attempts);
        }
        else
        {
            attempts = 0;
        }
    }
    else
    {
        attempts = 0;
    }
} 
