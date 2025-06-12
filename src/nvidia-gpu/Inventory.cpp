#include "Inventory.hpp"

#include <phosphor-logging/lg2.hpp>

#include <optional>
#include <string_view>
#include <utility>

constexpr int maxAttempts = 3;
static constexpr const char* inventoryPrefix =
    "/xyz/openbmc_project/inventory/";
static constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";
static constexpr const char* assetIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Asset";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, mctp::MctpRequester& mctpRequester,
    DeviceType deviceType, uint8_t eid) :
    path(std::string(inventoryPrefix) + inventoryName),
    mctpRequester(mctpRequester), deviceType(deviceType), eid(eid)
{
    if (deviceType == DeviceType::GPU)
    {
        acceleratorInterface =
            objectServer.add_interface(path, acceleratorIfaceName);
        acceleratorInterface->register_property("Type", std::string("GPU"));
        acceleratorInterface->initialize();
    }

    assetIface = objectServer.add_interface(path, assetIfaceName);
    assetIface->register_property("PartNumber", std::string{});
    assetIface->register_property("SerialNumber", std::string{});
    assetIface->initialize();
}

void Inventory::fetchPartNumber()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER, req,
                             resp);
}

void Inventory::fetchSerialNumber()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, req,
                             resp);
}

void Inventory::requestInventoryProperty(
    gpu::InventoryPropertyId propertyId,
    std::shared_ptr<InventoryRequestBuffer> requestBuffer,
    std::shared_ptr<InventoryResponseBuffer> responseBuffer)
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(propertyId), *requestBuffer);
    if (rc != 0)
    {
        auto dbusPropertyOpt = dbusPropertyNameForId(propertyId);
        std::string propName =
            dbusPropertyOpt ? std::string(*dbusPropertyOpt) : "Unknown";
        lg2::error("Failed to encode {PROP} request: rc={RC}", "PROP", propName,
                   "RC", rc);
        return;
    }
    mctpRequester.sendRecvMsg(
        eid, *requestBuffer, *responseBuffer,
        [this, propertyId, requestBuffer,
         responseBuffer](int sendRecvMsgResult) mutable {
            handleInventoryPropertyResponse(propertyId, sendRecvMsgResult,
                                            requestBuffer, responseBuffer);
        });
}

void Inventory::handleInventoryPropertyResponse(
    gpu::InventoryPropertyId propertyId, int sendRecvMsgResult,
    std::shared_ptr<InventoryRequestBuffer> requestBuffer,
    std::shared_ptr<InventoryResponseBuffer> responseBuffer)
{
    static int attempts = 0;
    bool success = false;
    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryInfo info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            *responseBuffer, cc, reasonCode, propertyId, info);
        if (rc == 0 &&
            cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            std::string value = std::get<std::string>(info);
            if (assetIface)
            {
                auto dbusPropertyOpt = dbusPropertyNameForId(propertyId);
                if (dbusPropertyOpt)
                {
                    assetIface->set_property(std::string(*dbusPropertyOpt),
                                             value);
                    success = true;
                }
            }
        }
    }

    if (!success)
    {
        attempts++;
        lg2::error("Failed to update property (attempt {ATTEMPT}/{MAX})",
                   "ATTEMPT", attempts, "MAX", maxAttempts);
        if (attempts < maxAttempts)
        {
            requestInventoryProperty(propertyId, requestBuffer, responseBuffer);
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

void Inventory::update()
{
    fetchPartNumber();
    fetchSerialNumber();
}

std::optional<std::string_view> Inventory::dbusPropertyNameForId(
    gpu::InventoryPropertyId propertyId)
{
    switch (propertyId)
    {
        case gpu::InventoryPropertyId::BOARD_PART_NUMBER:
            return "BoardPartNumber";
        case gpu::InventoryPropertyId::SERIAL_NUMBER:
            return "SerialNumber";
        case gpu::InventoryPropertyId::MARKETING_NAME:
            return "MarketingName";
        case gpu::InventoryPropertyId::DEVICE_PART_NUMBER:
            return "PartNumber";
        // Add more as needed
        default:
            return std::nullopt;
    }
}
