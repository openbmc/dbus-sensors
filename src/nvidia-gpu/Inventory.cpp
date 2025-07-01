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
    const DeviceType deviceTypeIn, const uint8_t eid,
    boost::asio::io_context& io) :
    name(escapeName(inventoryName)), mctpRequester(mctpRequester),
    deviceType(deviceTypeIn), eid(eid), retryTimer(io)
{
    requestBuffer = std::make_shared<InventoryRequestBuffer>();
    responseBuffer = std::make_shared<InventoryResponseBuffer>();

    std::string path = std::string(inventoryPrefix) + name;
    acceleratorInterface =
        objectServer.add_interface(path, acceleratorIfaceName);
    assetIface = objectServer.add_interface(path, assetIfaceName);

    // Static properties
    if (deviceType == DeviceType::GPU)
    {
        acceleratorInterface->register_property("Type", std::string("GPU"));
    }
    assetIface->register_property("Manufacturer", std::string("NVIDIA"));

    // Register properties which need to be fetched from the device
    registerProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, assetIface,
                     "SerialNumber");
    registerProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER, assetIface,
                     "PartNumber");

    acceleratorInterface->initialize();
    assetIface->initialize();
    processNextProperty();
}

void Inventory::registerProperty(
    gpu::InventoryPropertyId propertyId,
    std::shared_ptr<sdbusplus::asio::dbus_interface> interface,
    const std::string& propertyName)
{
    if (interface)
    {
        interface->register_property(propertyName, std::string{});
        properties[propertyId] = {interface, propertyName, 0, true};
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

void Inventory::fetchInventoryProperty(gpu::InventoryPropertyId propertyId)
{
    auto it = properties.find(propertyId);
    if (it != properties.end())
    {
        markPropertyPending(propertyId);
        std::optional<gpu::InventoryPropertyId> nextProperty =
            getNextPendingProperty();
        if (nextProperty && *nextProperty == propertyId)
        {
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

void Inventory::requestInventoryProperty(gpu::InventoryPropertyId propertyId)
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
    bool success = false;
    if (sendRecvMsgResult == 0)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryInfo info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            *responseBuffer, cc, reasonCode, propertyId, info);

        lg2::info(
            "Response for property ID {PROP_ID} from {NAME}, sendRecvMsgResult: {RESULT}, decode_rc: {RC}, completion_code: {CC}, reason_code: {REASON}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name, "RESULT",
            sendRecvMsgResult, "RC", rc, "CC", static_cast<uint8_t>(cc),
            "REASON", reasonCode);

        if (rc == 0 &&
            cc == ocp::accelerator_management::CompletionCode::SUCCESS &&
            std::holds_alternative<std::string>(info))
        {
            std::string value = std::get<std::string>(info);
            auto it = properties.find(propertyId);
            if (it != properties.end())
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
        auto it = properties.find(propertyId);
        if (it != properties.end())
        {
            it->second.retryCount++;
            if (it->second.retryCount >= maxRetryAttempts)
            {
                lg2::error(
                    "Property ID {PROP_ID} for {NAME} failed after {ATTEMPTS} attempts",
                    "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name,
                    "ATTEMPTS", maxRetryAttempts);
                markPropertyProcessed(propertyId);
            }
            else
            {
                retryTimer.expires_after(retryDelay);
                retryTimer.async_wait(
                    [this](const boost::system::error_code& ec) {
                        if (ec)
                        {
                            lg2::error("Retry timer error for {NAME}: {ERROR}",
                                       "NAME", name, "ERROR", ec.message());
                            return;
                        }
                        this->processNextProperty();
                    });
                return;
            }
        }
    }
    else
    {
        markPropertyProcessed(propertyId);
    }

    processNextProperty();
}

void Inventory::processNextProperty()
{
    std::optional<gpu::InventoryPropertyId> nextProperty =
        getNextPendingProperty();
    if (nextProperty)
    {
        requestInventoryProperty(*nextProperty);
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
}
