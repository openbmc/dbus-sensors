#include "Inventory.hpp"

#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>

#include <optional>
#include <string_view>
#include <utility>

constexpr int maxRetryAttempts = 3;
static constexpr const char* inventoryPrefix =
    "/xyz/openbmc_project/inventory/";
static constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";
static constexpr const char* assetIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Asset";
static constexpr const char* revisionIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Revision";
static constexpr const char* locationCodeIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.LocationCode";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, mctp::MctpRequester& mctpRequester,
    DeviceType deviceType, uint8_t eid, boost::asio::io_context& io) :
    path(std::string(inventoryPrefix) + inventoryName),
    mctpRequester(mctpRequester), deviceType(deviceType), eid(eid),
    retryTimer(io), objectServer(objectServer)
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
    assetIface->register_property("Model", std::string{});
    assetIface->initialize();

    revisionIface = objectServer.add_interface(path, revisionIfaceName);
    revisionIface->register_property("Version", std::string{});
    revisionIface->initialize();
}

void Inventory::setLocationCode(const std::string& locationCode)
{
    if (!locationCodeIface)
    {
        locationCodeIface =
            objectServer.add_interface(path, locationCodeIfaceName);
    }
    locationCodeIface->register_property("LocationCode", locationCode);
    locationCodeIface->initialize();
}

void Inventory::fetchBoardPartNumber()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER, req,
                             resp);
}

void Inventory::fetchSerialNumber()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, req,
                             resp);
}

void Inventory::fetchMarketingName()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::MARKETING_NAME, req,
                             resp);
}

void Inventory::fetchDevicePartNumber()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    requestInventoryProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER, req,
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
        lg2::error("Failed to encode property ID {PROP_ID} request: rc={RC}",
                   "PROP_ID", static_cast<uint8_t>(propertyId), "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *requestBuffer, *responseBuffer,
        [self = weak_from_this(), propertyId, requestBuffer,
         responseBuffer](int sendRecvMsgResult) {
            if (auto inventory = self.lock())
            {
                inventory->handleInventoryPropertyResponse(
                    propertyId, sendRecvMsgResult, requestBuffer,
                    responseBuffer);
            }
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
                    attempts = 0;
                }
            }
        }
    }

    if (!success)
    {
        attempts++;
        lg2::error(
            "Failed to update property ID {PROP_ID} (attempt {ATTEMPT}/{MAX})",
            "PROP_ID", static_cast<uint8_t>(propertyId), "ATTEMPT", attempts,
            "MAX", maxRetryAttempts);
        if (attempts < maxRetryAttempts)
        {
            retryRequest(propertyId, requestBuffer, responseBuffer);
        }
        else
        {
            attempts = 0;
        }
    }
}

void Inventory::retryRequest(
    gpu::InventoryPropertyId propertyId,
    std::shared_ptr<InventoryRequestBuffer> requestBuffer,
    std::shared_ptr<InventoryResponseBuffer> responseBuffer)
{
    retryTimer.expires_after(retryDelay);
    retryTimer.async_wait(
        [self = weak_from_this(), propertyId, requestBuffer,
         responseBuffer](const boost::system::error_code& ec) {
            if (ec)
            {
                lg2::error("Retry timer error: {ERROR}", "ERROR", ec.message());
                return;
            }
            if (auto inventory = self.lock())
            {
                inventory->requestInventoryProperty(propertyId, requestBuffer,
                                                    responseBuffer);
            }
        });
}

void Inventory::update()
{
    fetchBoardPartNumber();
    fetchSerialNumber();
    fetchMarketingName();
    fetchDevicePartNumber();
}

std::optional<std::string_view> Inventory::dbusPropertyNameForId(
    gpu::InventoryPropertyId propertyId)
{
    switch (propertyId)
    {
        case gpu::InventoryPropertyId::BOARD_PART_NUMBER:
            return "PartNumber";
        case gpu::InventoryPropertyId::SERIAL_NUMBER:
            return "SerialNumber";
        case gpu::InventoryPropertyId::MARKETING_NAME:
            return "Model";
        case gpu::InventoryPropertyId::DEVICE_PART_NUMBER:
            return "Version";
        // Add more as needed
        default:
            return std::nullopt;
    }
}
