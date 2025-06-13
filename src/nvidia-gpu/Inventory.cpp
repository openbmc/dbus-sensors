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
static constexpr const char* uuidIfaceName =
    "xyz.openbmc_project.Common.UUID";
static constexpr const char* revisionIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Revision";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, mctp::MctpRequester& mctpRequester,
    DeviceType deviceType, uint8_t eid, boost::asio::io_context& io) :
    path(std::string(inventoryPrefix) + inventoryName),
    mctpRequester(mctpRequester), deviceType(deviceType), eid(eid),
    retryTimer(io)
{
    requestBuffer = std::make_shared<InventoryRequestBuffer>();
    responseBuffer = std::make_shared<InventoryResponseBuffer>();

    initializeInterfaces(objectServer);
}

void Inventory::initializeInterfaces(sdbusplus::asio::object_server& objectServer)
{
    if (deviceType == DeviceType::GPU)
    {
        acceleratorInterface =
            objectServer.add_interface(path, acceleratorIfaceName);
        acceleratorInterface->register_property("Type", std::string("GPU"));
        acceleratorInterface->initialize();
    }

    assetIface = objectServer.add_interface(path, assetIfaceName);
    assetIface->initialize();

    uuidInterface = objectServer.add_interface(path, uuidIfaceName);
    uuidInterface->initialize();
    revisionIface = objectServer.add_interface(path, revisionIfaceName);
    revisionIface->initialize();

    // Register properties with their respective interfaces
    registerProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER,
                    assetIfaceName, "PartNumber");
    registerProperty(gpu::InventoryPropertyId::SERIAL_NUMBER,
                    assetIfaceName, "SerialNumber");
    registerProperty(gpu::InventoryPropertyId::MARKETING_NAME,
                    assetIfaceName, "Model");
    registerProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER,
                    assetIfaceName, "Version");
    registerProperty(gpu::InventoryPropertyId::DEVICE_GUID,
                    uuidIfaceName, "UUID");
}

void Inventory::registerProperty(gpu::InventoryPropertyId propertyId,
                               const std::string& interfaceName,
                               const std::string& propertyName)
{
    std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
    if (interfaceName == assetIfaceName)
    {
        interface = assetIface;
    }
    else if (interfaceName == acceleratorIfaceName)
    {
        interface = acceleratorInterface;
    }
    else if (interfaceName == uuidIfaceName)
    {
        interface = uuidInterface;
    }
    else if (interfaceName == revisionIfaceName)
    {
        interface = revisionIface;
    }

    if (interface)
    {
        interface->register_property(propertyName, std::string{});
        properties[propertyId] = {interface, propertyName, 0, false};
    }
}

void Inventory::fetchBoardPartNumber()
{
    fetchInventoryProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER);
}

void Inventory::fetchSerialNumber()
{
    fetchInventoryProperty(gpu::InventoryPropertyId::SERIAL_NUMBER);
}

void Inventory::fetchUUID()
{
    fetchInventoryProperty(gpu::InventoryPropertyId::DEVICE_GUID);
}

void Inventory::fetchMarketingName()
{
    fetchInventoryProperty(gpu::InventoryPropertyId::MARKETING_NAME);
}

void Inventory::fetchDevicePartNumber()
{
    fetchInventoryProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER);
}

void Inventory::fetchInventoryProperty(gpu::InventoryPropertyId propertyId)
{
    auto it = properties.find(propertyId);
    if (it != properties.end())
    {
        markPropertyPending(propertyId);
        if (auto nextProperty = getNextPendingProperty(); nextProperty && *nextProperty == propertyId)
        {
            // If this is the first pending property, start processing immediately
            processNextProperty();
        }
    }
}

void Inventory::markPropertyPending(gpu::InventoryPropertyId propertyId)
{
    auto it = properties.find(propertyId);
    if (it != properties.end())
    {
        it->second.isPending = true;
        it->second.retryCount = 0;
    }
}

void Inventory::markPropertyProcessed(gpu::InventoryPropertyId propertyId)
{
    auto it = properties.find(propertyId);
    if (it != properties.end())
    {
        it->second.isPending = false;
    }
}

std::optional<gpu::InventoryPropertyId> Inventory::getNextPendingProperty() const
{
    for (const auto& [propertyId, info] : properties)
    {
        if (info.isPending)
        {
            return propertyId;
        }
    }
    return std::nullopt;
}

void Inventory::requestInventoryProperty(gpu::InventoryPropertyId propertyId)
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
        [self = weak_from_this(), propertyId](int sendRecvMsgResult) {
            if (auto inventory = self.lock())
            {
                inventory->handleInventoryPropertyResponse(propertyId,
                                                          sendRecvMsgResult);
            }
        });
}

void Inventory::handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId,
                                              int sendRecvMsgResult)
{
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
            auto it = properties.find(propertyId);
            if (it != properties.end())
            {
                it->second.interface->set_property(it->second.propertyName, value);
                success = true;
            }
        }
    }

    if (!success)
    {
        // Property failed, increment retry count
        auto it = properties.find(propertyId);
        if (it != properties.end())
        {
            it->second.retryCount++;
            if (it->second.retryCount >= maxRetryAttempts)
            {
                lg2::error("Property ID {PROP_ID} failed after {ATTEMPTS} attempts",
                          "PROP_ID", static_cast<uint8_t>(propertyId), "ATTEMPTS",
                          maxRetryAttempts);
                markPropertyProcessed(propertyId);
            }
            else
            {
                // Schedule retry
                retryTimer.expires_after(retryDelay);
                retryTimer.async_wait(
                    [self = weak_from_this()](const boost::system::error_code& ec) {
                        if (ec)
                        {
                            lg2::error("Retry timer error: {ERROR}", "ERROR",
                                      ec.message());
                            return;
                        }
                        if (auto inventory = self.lock())
                        {
                            inventory->processNextProperty();
                        }
                    });
                return;
            }
        }
    }
    else
    {
        // Property succeeded
        markPropertyProcessed(propertyId);
    }

    // Process next property if any
    if (auto nextProperty = getNextPendingProperty())
    {
        processNextProperty();
    }
}

void Inventory::processNextProperty()
{
    if (auto nextProperty = getNextPendingProperty())
    {
        requestInventoryProperty(*nextProperty);
    }
}

void Inventory::update()
{
    fetchBoardPartNumber();
    fetchSerialNumber();
    fetchUUID();
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
        case gpu::InventoryPropertyId::DEVICE_GUID:
            return "UUID";
        // Add more as needed
        default:
            return std::nullopt;
    }
}
