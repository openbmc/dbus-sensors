#include "Inventory.hpp"

#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <unordered_map>
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
static constexpr const char* operatingConfigIfaceName =
    "xyz.openbmc_project.Inventory.Item.Cpu.OperatingConfig";

Inventory::Inventory(
    const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
    sdbusplus::asio::object_server& objectServer,
    const std::string& inventoryName, mctp::MctpRequester& mctpRequester,
    const gpu::DeviceIdentification deviceTypeIn, const uint8_t eid,
    boost::asio::io_context& io) :
    name(escapeName(inventoryName)), mctpRequester(mctpRequester),
    deviceType(deviceTypeIn), eid(eid), retryTimer(io)
{
    std::string path = inventoryPrefix + name;

    assetIface = objectServer.add_interface(path, assetIfaceName);
    assetIface->register_property("Manufacturer", std::string("NVIDIA"));
    // Register properties which need to be fetched from the device
    registerProperty(gpu::InventoryPropertyId::SERIAL_NUMBER, assetIface,
                     "SerialNumber");
    registerProperty(gpu::InventoryPropertyId::BOARD_PART_NUMBER, assetIface,
                     "PartNumber");
    registerProperty(gpu::InventoryPropertyId::MARKETING_NAME, assetIface,
                     "Model");
    assetIface->initialize();

    uuidInterface = objectServer.add_interface(path, uuidIfaceName);
    registerProperty(gpu::InventoryPropertyId::DEVICE_GUID, uuidInterface,
                     "UUID");
    uuidInterface->initialize();

    revisionIface = objectServer.add_interface(path, revisionIfaceName);
    registerProperty(gpu::InventoryPropertyId::DEVICE_PART_NUMBER,
                     revisionIface, "Version");
    revisionIface->initialize();

    operatingConfigInterface =
        objectServer.add_interface(path, operatingConfigIfaceName);
    registerUint32Property(gpu::InventoryPropertyId::DEFAULT_BASE_CLOCKS,
                           operatingConfigInterface, "BaseSpeed");
    registerUint32Property(gpu::InventoryPropertyId::MAX_GRAPHICS_CLOCK,
                           operatingConfigInterface, "MaxSpeed");
    registerUint32Property(gpu::InventoryPropertyId::MIN_GRAPHICS_CLOCK,
                           operatingConfigInterface, "MinSpeed");
    // Polling properties - updated via update()
    operatingConfigInterface->register_property(
        "OperatingSpeed", operatingSpeed,
        sdbusplus::asio::PropertyPermission::readOnly);
    operatingConfigInterface->register_property("SpeedLimit", speedLimit);
    operatingConfigInterface->register_property("SpeedLocked", speedLocked);
    operatingConfigInterface->register_property("RequestedSpeedLimitMin",
                                                requestedSpeedLimitMin);
    operatingConfigInterface->register_property("RequestedSpeedLimitMax",
                                                requestedSpeedLimitMax);
    operatingConfigInterface->initialize();

    // Static properties
    if (deviceType == gpu::DeviceIdentification::DEVICE_GPU)
    {
        acceleratorInterface =
            objectServer.add_interface(path, acceleratorIfaceName);
        acceleratorInterface->register_property("Type", std::string("GPU"));

        // Register BoostClockFrequency property
        acceleratorInterface->register_property(
            "BoostClockFrequency", std::numeric_limits<uint64_t>::max());

        acceleratorInterface->initialize();

        // Add to query queue (manually since registerProperty is for strings
        // only)
        properties[gpu::InventoryPropertyId::DEFAULT_BOOST_CLOCKS] = {
            acceleratorInterface, "BoostClockFrequency", 0, true};
    }
}

void Inventory::init()
{
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

void Inventory::registerUint32Property(
    gpu::InventoryPropertyId propertyId,
    const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
    const std::string& propertyName)
{
    if (interface)
    {
        interface->register_property(propertyName, uint32_t{0});
        properties[propertyId] = {interface, propertyName, 0, true};
    }
}

void Inventory::processInventoryProperty(gpu::InventoryPropertyId propertyId)
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
        0, static_cast<uint8_t>(propertyId), requestBuffer);
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

    mctpRequester.sendRecvMsg(
        eid, requestBuffer,
        [weak{weak_from_this()}, propertyId](const std::error_code& ec,
                                             std::span<const uint8_t> buffer) {
            std::shared_ptr<Inventory> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid Inventory reference");
                return;
            }
            self->handleInventoryPropertyResponse(propertyId, ec, buffer);
        });
}

void Inventory::handleInventoryPropertyResponse(
    gpu::InventoryPropertyId propertyId, const std::error_code& ec,
    std::span<const uint8_t> buffer)
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
    if (!ec)
    {
        ocp::accelerator_management::CompletionCode cc{};
        uint16_t reasonCode = 0;
        gpu::InventoryValue info;
        int rc = gpu::decodeGetInventoryInformationResponse(
            buffer, cc, reasonCode, propertyId, info);

        lg2::info(
            "Response for property ID {PROP_ID} from {NAME}, sendRecvMsgResult: {RESULT}, decode_rc: {RC}, completion_code: {CC}, reason_code: {REASON}",
            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME", name, "RESULT",
            ec.message(), "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);

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
                        if (guidBytes.size() >= 16)
                        {
                            boost::uuids::uuid uuid;
                            std::copy(guidBytes.begin(), guidBytes.begin() + 16,
                                      uuid.begin());
                            value = boost::uuids::to_string(uuid);
                        }
                        else
                        {
                            lg2::error(
                                "Property ID {PROP_ID} for {NAME} GUID size {SIZE} is less than 16 bytes",
                                "PROP_ID", static_cast<uint8_t>(propertyId),
                                "NAME", name, "SIZE", guidBytes.size());
                            break;
                        }
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

                case gpu::InventoryPropertyId::DEFAULT_BOOST_CLOCKS:
                    if (std::holds_alternative<uint32_t>(info))
                    {
                        const uint32_t clockSpeed = std::get<uint32_t>(info);
                        // Convert to uint64_t for D-Bus interface requirement
                        const uint64_t clockSpeed64 =
                            static_cast<uint64_t>(clockSpeed);
                        it->second.interface->set_property(
                            it->second.propertyName, clockSpeed64);
                        success = true;
                    }
                    else
                    {
                        lg2::error(
                            "Property ID {PROP_ID} for {NAME} expected uint32_t but got different type",
                            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME",
                            name);
                    }
                    break;

                case gpu::InventoryPropertyId::DEFAULT_BASE_CLOCKS:
                case gpu::InventoryPropertyId::MIN_GRAPHICS_CLOCK:
                case gpu::InventoryPropertyId::MAX_GRAPHICS_CLOCK:
                    if (std::holds_alternative<uint32_t>(info))
                    {
                        uint32_t clockValue = std::get<uint32_t>(info);
                        it->second.interface->set_property(
                            it->second.propertyName, clockValue);
                        lg2::info(
                            "Successfully received property ID {PROP_ID} for {NAME} with value: {VALUE}",
                            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME",
                            name, "VALUE", clockValue);
                        success = true;
                    }
                    else
                    {
                        lg2::error(
                            "Property ID {PROP_ID} for {NAME} expected uint32_t but got different type",
                            "PROP_ID", static_cast<uint8_t>(propertyId), "NAME",
                            name);
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
            retryTimer.async_wait(
                [weak{weak_from_this()}](const boost::system::error_code& ec) {
                    std::shared_ptr<Inventory> self = weak.lock();
                    if (!self)
                    {
                        lg2::error("Invalid reference to Inventory");
                        return;
                    }
                    if (ec)
                    {
                        lg2::error("Retry timer error for {NAME}: {ERROR}",
                                   "NAME", self->name, "ERROR", ec.message());
                        return;
                    }
                    self->processNextProperty();
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
    sendClockFrequencyRequest();
    sendClockLimitRequest();
}

void Inventory::sendClockFrequencyRequest()
{
    int rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, gpu::ClockType::GRAPHICS_CLOCK, clockFrequencyRequestBuffer);
    if (rc != 0)
    {
        lg2::error(
            "Failed to encode clock frequency request for {NAME}: rc={RC}",
            "NAME", name, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, clockFrequencyRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<Inventory> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid Inventory reference");
                return;
            }
            self->handleClockFrequencyResponse(ec, buffer);
        });
}

void Inventory::handleClockFrequencyResponse(const std::error_code& ec,
                                             std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error reading clock frequency for {NAME}: MCTP failed, rc={RC}",
            "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;

    int rc = gpu::decodeGetCurrentClockFrequencyResponse(buffer, cc, reasonCode,
                                                         operatingSpeed);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock frequency for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    operatingConfigInterface->set_property("OperatingSpeed", operatingSpeed);
}

void Inventory::sendClockLimitRequest()
{
    int rc = gpu::encodeGetClockLimitRequest(0, gpu::ClockType::GRAPHICS_CLOCK,
                                             clockLimitRequestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode clock limit request for {NAME}: rc={RC}",
                   "NAME", name, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, clockLimitRequestBuffer,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<Inventory> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid Inventory reference");
                return;
            }
            self->handleClockLimitResponse(ec, buffer);
        });
}

void Inventory::handleClockLimitResponse(const std::error_code& ec,
                                         std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error("Error reading clock limit for {NAME}: MCTP failed, rc={RC}",
                   "NAME", name, "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t requestedLimitMin = 0;
    uint32_t requestedLimitMax = 0;
    uint32_t presentLimitMin = 0;
    uint32_t presentLimitMax = 0;

    int rc = gpu::decodeGetClockLimitResponse(
        buffer, cc, reasonCode, requestedLimitMin, requestedLimitMax,
        presentLimitMin, presentLimitMax);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error decoding clock limit for {NAME}: rc={RC}, cc={CC}, reasonCode={REASON}",
            "NAME", name, "RC", rc, "CC", static_cast<uint8_t>(cc), "REASON",
            reasonCode);
        return;
    }

    speedLimit = presentLimitMax;
    speedLocked = (presentLimitMax == presentLimitMin);
    requestedSpeedLimitMin = requestedLimitMin;
    requestedSpeedLimitMax = requestedLimitMax;

    operatingConfigInterface->set_property("SpeedLimit", speedLimit);
    operatingConfigInterface->set_property("SpeedLocked", speedLocked);
    operatingConfigInterface->set_property("RequestedSpeedLimitMin",
                                           requestedSpeedLimitMin);
    operatingConfigInterface->set_property("RequestedSpeedLimitMax",
                                           requestedSpeedLimitMax);
}
