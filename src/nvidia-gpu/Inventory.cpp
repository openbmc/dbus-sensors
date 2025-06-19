#include "Inventory.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

constexpr const char* inventoryPrefix = "/xyz/openbmc_project/inventory/";
constexpr const char* acceleratorIfaceName =
    "xyz.openbmc_project.Inventory.Item.Accelerator";
static constexpr const char* assetIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Asset";
static constexpr const char* uuidIfaceName = "xyz.openbmc_project.Common.UUID";
static constexpr const char* revisionIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.Revision";
static constexpr const char* locationCodeIfaceName =
    "xyz.openbmc_project.Inventory.Decorator.LocationCode";
static constexpr const char* associationIfaceName =
    "xyz.openbmc_project.Association.Definitions";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, mctp::MctpRequester& mctpRequester,
    const gpu::NvidiaVDMDeviceType deviceTypeIn, const uint8_t eid,
    boost::asio::io_context& io) :
    name(escapeName(inventoryName)), mctpRequester(mctpRequester),
    deviceType(deviceTypeIn), eid(eid), retryTimer(io),
    objectServer(objectServer),
    inventoryPath(std::string(inventoryPrefix) + escapeName(inventoryName))
{
    requestBuffer = std::make_shared<InventoryRequestBuffer>();
    responseBuffer = std::make_shared<InventoryResponseBuffer>();

    acceleratorInterface =
        objectServer.add_interface(inventoryPath, acceleratorIfaceName);
    assetIface = objectServer.add_interface(inventoryPath, assetIfaceName);
    uuidInterface = objectServer.add_interface(inventoryPath, uuidIfaceName);
    revisionIface =
        objectServer.add_interface(inventoryPath, revisionIfaceName);

    // Static properties
    if (deviceType == gpu::NvidiaVDMDeviceType::GPU)
    {
        acceleratorInterface->register_property("Type", std::string("GPU"));
    }
    assetIface->register_property("Manufacturer", std::string("NVIDIA"));

    // Register properties which need to be fetched from the device
    registerProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, assetIface,
                     "SerialNumber");
    registerProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER, assetIface,
                     "PartNumber");
    registerProperty(gpu::InventoryPropertyId::DEVICE_GUID, uuidInterface,
                     "UUID");
    registerProperty(gpu::InventoryPropertyId::MARKETING_NAME, assetIface,
                     "Model");
    registerProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER,
                     revisionIface, "Version");

    acceleratorInterface->initialize();
    assetIface->initialize();
    uuidInterface->initialize();
    revisionIface->initialize();
    processNextProperty();
}

void Inventory::registerProperty(
    gpu::InventoryPropertyId propertyId,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
    const std::string& propertyName)
{
    if (interface)
    {
        interface->register_property(propertyName, std::string{});
        properties[propertyId] = {interface, propertyName, 0, true};
    }
}

void Inventory::setLocationCode(const std::string& locationCode)
{
    if (!locationCodeIface)
    {
        locationCodeIface =
            objectServer.add_interface(inventoryPath, locationCodeIfaceName);
    }
    locationCodeIface->register_property("LocationCode", locationCode);
    locationCodeIface->initialize();
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
        markPropertyPending(it);
        std::optional<gpu::InventoryPropertyId> nextProperty =
            getNextPendingProperty();
        if (nextProperty && *nextProperty == propertyId)
        {
            processNextProperty();
        }
    }
}

void Inventory::markPropertyPending(
    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo>::iterator it)
{
    it->second.isPending = true;
    it->second.retryCount = 0;
}

void Inventory::markPropertyProcessed(
    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo>::iterator it)
{
    it->second.isPending = false;
}

std::optional<gpu::InventoryPropertyId> Inventory::getNextPendingProperty()
    const
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

void Inventory::sendInventoryPropertyRequest(
    gpu::InventoryPropertyId propertyId)
{
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(propertyId), *requestBuffer);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode property ID {PROP_ID} request for {NAME}: rc={RC}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name, "RC",
            rc);
        return;
    }

    lg2::info(
        "Sending inventory request for property ID {PROP_ID} to EID {EID} for {NAME}",
        "PROP_ID", static_cast<uint8_t>(propertyId), "EID", eid, "NAME", name);

    mctpRequester.sendRecvMsg(eid, *requestBuffer, *responseBuffer,
                              [this, propertyId](int sendRecvMsgResult) {
                                  this->handleInventoryPropertyResponse(
                                      propertyId, sendRecvMsgResult);
                              });
}

void Inventory::handleInventoryPropertyResponse(
    gpu::InventoryPropertyId propertyId, int sendRecvMsgResult)
{
    auto it = properties.find(propertyId);
    if (it == properties.end())
    {
        lg2::error("Property ID {PROP_ID} for {NAME} not found", "PROP_ID",
                   static_cast<uint8_t>(propertyId), "NAME", name);
        processNextProperty();
        return;
    }

    bool success = false;
    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryValue info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            *responseBuffer, cc, reasonCode, propertyId, info);

        lg2::info(
            "Response for property ID {PROP_ID} from {NAME}, sendRecvMsgResult: {RESULT}, decode_rc: {RC}, completion_code: {CC}, reason_code: {REASON}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name, "RESULT",
            sendRecvMsgResult, "RC", rc, "CC", static_cast<uint8_t>(cc),
            "REASON", reasonCode);

        if (rc == 0 &&
            cc == ocp::accelerator_management::CompletionCode::SUCCESS)
        {
            std::string value;

            // Handle different property types based on property ID
            switch (propertyId)
            {
                case gpu::InventoryPropertyId::BOARD_PART_NUMBER:
                case gpu::InventoryPropertyId::SERIAL_NUMBER:
                case gpu::InventoryPropertyId::MARKETING_NAME:
                case gpu::InventoryPropertyId::DEVICE_PART_NUMBER:
                    if (std::holds_alternative<std::string>(info))
                    {
                        value = std::get<std::string>(info);
                    }
                    else
                    {
                        lg2::error(
                            "Property ID {PROP_ID} for {NAME} expected string but got different type",
                            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME",
                            name);
                        break;
                    }
                    break;

                case gpu::InventoryPropertyId::DEVICE_GUID:
                    if (std::holds_alternative<std::vector<uint8_t>>(info))
                    {
                        const auto& guidBytes =
                            std::get<std::vector<uint8_t>>(info);
                        value = formatUuid(guidBytes);
                    }
                    else
                    {
                        lg2::error(
                            "Property ID {PROP_ID} for {NAME} expected vector<uint8_t> but got different type",
                            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME",
                            name);
                        break;
                    }
                    break;

                default:
                    lg2::error("Unsupported property ID {PROP_ID} for {NAME}",
                               "PROP_ID", static_cast<uint8_t>(propertyId),
                               "NAME", name);
                    break;
            }

            if (!value.empty())
            {
                it->second.interface->set_property(it->second.propertyName,
                                                   value);
                lg2::info(
                    "Successfully received property ID {PROP_ID} for {NAME} with value: {VALUE}",
                    "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name,
                    "VALUE", value);
                success = true;
            }
        }
    }

    if (!success)
    {
        it->second.retryCount++;
        if (it->second.retryCount >= maxRetryAttempts)
        {
            lg2::error(
                "Property ID {PROP_ID} for {NAME} failed after {ATTEMPTS} attempts",
                "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name,
                "ATTEMPTS", maxRetryAttempts);
            markPropertyProcessed(it);
        }
        else
        {
            retryTimer.expires_after(retryDelay);
            retryTimer.async_wait([this](const boost::system::error_code& ec) {
                if (ec)
                {
                    lg2::error("Retry timer error for {NAME}: {ERROR}", "NAME",
                               name, "ERROR", ec.message());
                    return;
                }
                this->processNextProperty();
            });
            return;
        }
    }
    else
    {
        markPropertyProcessed(it);
    }

    processNextProperty();
}

void Inventory::processNextProperty()
{
    std::optional<gpu::InventoryPropertyId> nextProperty =
        getNextPendingProperty();
    if (nextProperty)
    {
        sendInventoryPropertyRequest(*nextProperty);
    }
    else
    {
        lg2::info("No pending properties found to process for {NAME}", "NAME",
                  name);
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

std::string Inventory::getInventoryPath() const
{
    return inventoryPath;
}

void Inventory::setAssociation(const std::string& chassisPath)
{
    std::string path = std::string(inventoryPrefix) + name;
    associationInterface =
        objectServer.add_interface(path, associationIfaceName);
    associations.emplace_back("parent_chassis", "all_processor", chassisPath);
    associationInterface->register_property("Associations", associations);
    associationInterface->initialize();
}
