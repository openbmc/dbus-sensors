/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <DeviceInterface.hpp>
#include <EndpointState.hpp>
#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaSensorConfig.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

static constexpr auto sensorPollRateMs = 1000;

// Bound on how many times a transient D-Bus failure during a single endpoint's
// config-resolution chain will re-trigger a discovery sweep before giving up.
static constexpr unsigned maxDiscoveryRetries = 3;

// EntityManager escapes inventory object paths by replacing every character
// that is not [A-Za-z0-9_] with '_'; escapeName() only replaces spaces, so a
// board name with e.g. a hyphen would not match its inventory path segment.
static std::string escapeForInventoryPath(const std::string& name)
{
    std::string out = name;
    std::ranges::replace_if(
        out,
        [](char c) {
            return (c < '0' || c > '9') && (c < 'A' || c > 'Z') &&
                   (c < 'a' || c > 'z') && c != '_';
        },
        '_');
    return out;
}

// Helper function to extract bridge pool information from properties
static std::optional<std::pair<uint8_t, uint8_t>> extractBridgePool(
    const SensorBaseConfigMap& properties)
{
    auto poolStartIt = properties.find("PoolStart");
    auto poolEndIt = properties.find("PoolEnd");

    const auto* poolStartPtr = (poolStartIt != properties.end())
                                   ? std::get_if<uint8_t>(&poolStartIt->second)
                                   : nullptr;
    const auto* poolEndPtr = (poolEndIt != properties.end())
                                 ? std::get_if<uint8_t>(&poolEndIt->second)
                                 : nullptr;

    if ((poolStartPtr != nullptr) && (poolEndPtr != nullptr))
    {
        // Reject a reversed range: downstream code computes the pool size as
        // PoolEnd - PoolStart + 1 in uint8_t, which would underflow to a huge
        // count if PoolEnd < PoolStart.
        if (*poolEndPtr < *poolStartPtr)
        {
            lg2::error(
                "Ignoring invalid bridge pool: PoolEnd {END} < PoolStart {START}",
                "END", *poolEndPtr, "START", *poolStartPtr);
            return std::nullopt;
        }

        auto bridgePool = std::make_pair(*poolStartPtr, *poolEndPtr);
        lg2::info("EID Bridge found: PoolStart={START}, PoolEnd={END}", "START",
                  bridgePool.first, "END", bridgePool.second);
        return bridgePool;
    }

    return std::nullopt;
}

DeviceManager::DeviceManager(boost::asio::io_context& io,
                             sdbusplus::asio::object_server& objectServer,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             mctp::MctpRequester& mctpRequester) :
    io(io), objectServer(objectServer), conn(std::move(conn)),
    mctpRequester(mctpRequester), configTimer(io)
{}

// Debounce window to coalesce a burst of entity-manager config property
// changes / mctpd connectivity events into a single discovery sweep.
static constexpr std::chrono::seconds configSettleInterval{1};

void DeviceManager::scheduleRescan()
{
    // Coalesce bursts of entity-manager config property changes into a single
    // discovery sweep (multiple properties typically change together).
    configTimer.expires_after(configSettleInterval);
    configTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        createSensors();
    });
}

bool DeviceManager::retryDiscovery(const std::string& endpointPath, uint8_t eid)
{
    unsigned& count = discoveryRetries[endpointPath];
    if (count >= maxDiscoveryRetries)
    {
        // Keep the entry at the cap (do not erase) so a persistently failing
        // endpoint stops re-triggering sweeps; a normal event-driven rescan
        // will still process it if its config later appears.
        lg2::error(
            "EID {EID}: giving up discovery for {PATH} after {MAX} transient retries",
            "EID", eid, "PATH", endpointPath, "MAX", maxDiscoveryRetries);
        return false;
    }
    ++count;
    lg2::warning(
        "EID {EID}: transient discovery error for {PATH}, scheduling retry {N}/{MAX}",
        "EID", eid, "PATH", endpointPath, "N", count, "MAX",
        maxDiscoveryRetries);
    scheduleRescan();
    return true;
}
void DeviceManager::processQueryDeviceIdResponse(
    const SensorConfigs& configs, const std::string& path,
    const std::string& endpointPath, uint8_t eid,
    const std::string& chassisPath, const std::string& deviceName,
    const std::error_code& sendRecvMsgResult,
    std::span<const uint8_t> queryDeviceIdentificationResponse)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "RC", sendRecvMsgResult.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t responseDeviceType = 0;
    uint8_t responseInstanceId = 0;

    auto rc = gpu::decodeQueryDeviceIdentificationResponse(
        queryDeviceIdentificationResponse, cc, reasonCode, responseDeviceType,
        responseInstanceId);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    switch (static_cast<gpu::DeviceIdentification>(responseDeviceType))
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
        {
            lg2::info(
                "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            const std::string gpuName =
                deviceName.empty() ? std::format("Nvidia_GPU_{}", eid)
                                   : deviceName;

            std::shared_ptr<GpuDevice>& gpu = gpuDevices[gpuName];

            if (gpu == nullptr)
            {
                gpu = std::make_shared<GpuDevice>(configs, gpuName, path,
                                                  chassisPath, conn, eid, io,
                                                  mctpRequester, objectServer);

                gpu->init();
            }
            else
            {
                lg2::info(
                    "GPU Device with name {NAME} already exists. Skipping creating a new device.",
                    "NAME", gpuName);
            }

            break;
        }

        case gpu::DeviceIdentification::DEVICE_SMA:
        {
            lg2::info(
                "Found the SMA Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            const std::string smaName =
                deviceName.empty() ? std::format("Nvidia_SMA_{}", eid)
                                   : deviceName;

            std::shared_ptr<SmaDevice>& sma = smaDevices[smaName];

            if (sma == nullptr)
            {
                sma = std::make_shared<SmaDevice>(configs, smaName, path, conn,
                                                  eid, io, mctpRequester,
                                                  objectServer);

                sma->init();

                // Only the endpoint itself (queried with an empty name) owns
                // the mctpd endpoint object; bridged pool devices share the
                // bridge's endpointPath and must not register / recover under
                // it, which would clobber the bridge's own endpoint record.
                if (deviceName.empty())
                {
                    registerEndpoint(
                        endpointPath, eid,
                        std::static_pointer_cast<DeviceInterface>(sma));

                    // init() only builds the D-Bus objects; the Init -> Online
                    // transition starts polling via setOnline().
                    applyEvent(endpointPath, EndpointEvent::InitComplete);
                }
            }
            else
            {
                lg2::info(
                    "SMA Device with name {NAME} already exists. Skipping creating a new device.",
                    "NAME", smaName);
            }

            break;
        }

        case gpu::DeviceIdentification::DEVICE_PCIE:
        {
            lg2::info(
                "Found the PCIe Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            const std::string pcieName =
                deviceName.empty() ? std::format("Nvidia_ConnectX_{}", eid)
                                   : deviceName;

            std::shared_ptr<PcieDevice>& pcie = pcieDevices[pcieName];

            if (pcie == nullptr)
            {
                pcie = std::make_shared<PcieDevice>(
                    configs, pcieName, path, conn, eid, io, mctpRequester,
                    objectServer);

                pcie->init();
            }
            else
            {
                lg2::info(
                    "PCIe Device with name {NAME} already exists. Skipping creating a new device.",
                    "NAME", pcieName);
            }

            break;
        }

        default:
            lg2::error("Unknown device type {TYPE} for EID {EID}", "TYPE",
                       responseDeviceType, "EID", eid);
            break;
    }
}

void DeviceManager::queryDeviceIdentification(
    const SensorConfigs& configs, const std::string& path,
    const std::string& endpointPath, uint8_t eid,
    const std::string& chassisPath, const std::string& deviceName)
{
    // Reaching here means the config-resolution chain succeeded for this
    // endpoint, so clear any transient-error retry budget accrued for it.
    discoveryRetries.erase(endpointPath);

    auto queryDeviceIdentificationRequest = std::make_shared<
        std::array<uint8_t, gpu::queryDeviceIdentificationRequestSize>>();

    auto rc = gpu::encodeQueryDeviceIdentificationRequest(
        0, *queryDeviceIdentificationRequest);
    if (rc != 0)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : encode failed, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *queryDeviceIdentificationRequest,
        [this, configs, path, endpointPath, eid, chassisPath, deviceName,
         queryDeviceIdentificationRequest](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(configs, path, endpointPath, eid,
                                         chassisPath, deviceName, ec, response);
        });
}

void DeviceManager::queryDevicesForEndpoint(
    const SensorConfigs& configs, const std::string& configPath,
    const std::string& endpointPath, uint8_t eid,
    const std::string& chassisPath,
    const std::optional<std::pair<uint8_t, uint8_t>>& bridgePool,
    const std::optional<std::vector<std::string>>& bridgedEndpoints)
{
    // Query the SMA (the endpoint itself) with an empty name to keep the
    // eid-based naming and the recovery wiring intact.
    queryDeviceIdentification(configs, configPath, endpointPath, eid,
                              chassisPath, "");

    // If this endpoint is a bridge, also create the bridged devices behind it
    // by walking the bridge's EID pool alongside the BridgedEndpoints names.
    if (bridgePool && bridgedEndpoints)
    {
        uint8_t index = 0;
        for (const auto& name : *bridgedEndpoints)
        {
            uint8_t bridgedEid = bridgePool->first + index;
            queryDeviceIdentification(configs, configPath, endpointPath,
                                      bridgedEid, chassisPath, name);
            ++index;
        }
    }
}

void DeviceManager::checkAssociationAndQueryDevice(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    const std::string associationPath = endpointPath + "/configured_by";

    conn->async_method_call(
        [this, configs, endpointPath, eid, associationPath, bridgePool](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            if (ec || ret.empty())
            {
                lg2::error(
                    "EID {EID}: No association found at {PATH}, skipping endpoint",
                    "EID", eid, "PATH", associationPath);
                return;
            }
            getAssociationEndpoints(configs, endpointPath, eid, associationPath,
                                    ret[0].first, bridgePool);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", associationPath,
        std::vector<std::string>{"xyz.openbmc_project.Association"});
}

void DeviceManager::getAssociationEndpoints(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& associationPath, const std::string& associationService,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    conn->async_method_call(
        [this, configs, endpointPath, eid,
         bridgePool](const boost::system::error_code& ec,
                     const std::variant<std::vector<std::string>>& value) {
            processAssociationEndpointsResult(configs, endpointPath, eid, ec,
                                              value, bridgePool);
        },
        associationService, associationPath, "org.freedesktop.DBus.Properties",
        "Get", "xyz.openbmc_project.Association", "endpoints");
}

void DeviceManager::processAssociationEndpointsResult(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const boost::system::error_code& ec,
    const std::variant<std::vector<std::string>>& value,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    if (ec)
    {
        lg2::error("EID {EID}: Failed to get endpoints property: {ERROR}",
                   "EID", eid, "ERROR", ec.message());
        retryDiscovery(endpointPath, eid);
        return;
    }

    const auto* endpointsPtr = std::get_if<std::vector<std::string>>(&value);
    if ((endpointsPtr == nullptr) || endpointsPtr->empty())
    {
        lg2::error("EID {EID}: endpoints property is empty, skipping", "EID",
                   eid);
        return;
    }

    getConfigService(configs, endpointPath, eid, (*endpointsPtr)[0],
                     bridgePool);
}

void DeviceManager::getConfigService(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& configPath,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    conn->async_method_call(
        [this, configs, endpointPath, eid, configPath, bridgePool](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            if (ec || ret.empty())
            {
                lg2::error(
                    "EID {EID}: Failed to get service for config path {PATH}: {ERROR}",
                    "EID", eid, "PATH", configPath, "ERROR", ec.message());
                retryDiscovery(endpointPath, eid);
                return;
            }
            getConfigProperties(configs, endpointPath, eid, configPath,
                                ret[0].first, bridgePool);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", configPath,
        std::vector<std::string>{});
}

void DeviceManager::getConfigProperties(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& configPath, const std::string& configService,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    conn->async_method_call(
        [this, configs, endpointPath, eid, configPath,
         bridgePool](const boost::system::error_code& ec,
                     const SensorBaseConfigMap& configProps) {
            processConfigPropertiesResult(configs, endpointPath, eid,
                                          configPath, ec, configProps,
                                          bridgePool);
        },
        configService, configPath, "org.freedesktop.DBus.Properties", "GetAll",
        "");
}

void DeviceManager::processConfigPropertiesResult(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& configPath, const boost::system::error_code& ec,
    const SensorBaseConfigMap& configProps,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    if (ec)
    {
        lg2::error("EID {EID}: Failed to get config properties: {ERROR}", "EID",
                   eid, "ERROR", ec.message());
        retryDiscovery(endpointPath, eid);
        return;
    }

    auto nameIt = configProps.find("Name");
    if (nameIt == configProps.end())
    {
        lg2::error("EID {EID}: Name property not found in config, skipping",
                   "EID", eid);
        return;
    }
    const auto* namePtr = std::get_if<std::string>(&nameIt->second);
    if (namePtr == nullptr)
    {
        lg2::error("EID {EID}: Name property has invalid type, skipping", "EID",
                   eid);
        return;
    }
    const std::string& deviceName = *namePtr;
    lg2::info("EID {EID}: Found device name {NAME}", "EID", eid, "NAME",
              deviceName);

    // If this endpoint is a bridge, resolve the list of names for the devices
    // living behind it (one per EID in the bridge's pool).
    std::optional<std::vector<std::string>> bridgedEndpoints;
    if (bridgePool && bridgePool->first != 0 && bridgePool->second != 0)
    {
        auto bridgedIt = configProps.find("BridgedEndpoints");
        const auto* bridgedEndpointsPtr =
            (bridgedIt != configProps.end())
                ? std::get_if<std::vector<std::string>>(&bridgedIt->second)
                : nullptr;
        uint8_t expectedCount = bridgePool->second - bridgePool->first + 1;

        if (bridgedEndpointsPtr == nullptr)
        {
            // No (or malformed) BridgedEndpoints: without names we cannot
            // create the pool devices, so warn rather than dropping them
            // silently.
            lg2::warning(
                "EID {EID} is a bridge with pool range {START}-{END} but its config has no valid BridgedEndpoints; {COUNT} bridged device(s) will not be created",
                "EID", eid, "START", bridgePool->first, "END",
                bridgePool->second, "COUNT", expectedCount);
        }
        else if (bridgedEndpointsPtr->size() != expectedCount)
        {
            lg2::error(
                "EID {EID}: BridgedEndpoints array size mismatch. Expected {EXPECTED}, got {ACTUAL}",
                "EID", eid, "EXPECTED", expectedCount, "ACTUAL",
                bridgedEndpointsPtr->size());
        }
        else
        {
            bridgedEndpoints = *bridgedEndpointsPtr;
            lg2::info("EID {EID} is a bridge with pool range {START}-{END}",
                      "EID", eid, "START", bridgePool->first, "END",
                      bridgePool->second);
        }
    }

    auto boardIt = configProps.find("Board");
    if (boardIt != configProps.end())
    {
        const auto* boardPtr = std::get_if<std::string>(&boardIt->second);
        if ((boardPtr != nullptr) && !boardPtr->empty())
        {
            findBoardInventoryPath(configs, endpointPath, eid,
                                   escapeForInventoryPath(*boardPtr),
                                   configPath, bridgePool, bridgedEndpoints);
            return;
        }
    }

    lg2::info("EID {EID}: No Board property found, using config path {PATH}",
              "EID", eid, "PATH", configPath);
    queryDevicesForEndpoint(configs, configPath, endpointPath, eid, "",
                            bridgePool, bridgedEndpoints);
}

void DeviceManager::findBoardInventoryPath(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& boardName, const std::string& configPath,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
    const std::optional<std::vector<std::string>>& bridgedEndpoints)
{
    const std::string searchPath{"/xyz/openbmc_project/inventory"};
    const std::vector<std::string> ifaceList{
        "xyz.openbmc_project.Inventory.Item.Chassis",
        "xyz.openbmc_project.Inventory.Item.Board"};

    conn->async_method_call(
        [this, configs, endpointPath, eid, boardName, configPath, bridgePool,
         bridgedEndpoints](const boost::system::error_code& ec,
                           const GetSubTreeType& ret) {
            processBoardInventoryResult(configs, endpointPath, eid, boardName,
                                        configPath, ec, ret, bridgePool,
                                        bridgedEndpoints);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void DeviceManager::processBoardInventoryResult(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& boardName, const std::string& configPath,
    const boost::system::error_code& ec, const GetSubTreeType& ret,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
    const std::optional<std::vector<std::string>>& bridgedEndpoints)
{
    std::string inventoryPath = configPath;

    if (!ec && !ret.empty())
    {
        for (const auto& [objPath, services] : ret)
        {
            if (objPath.ends_with("/" + boardName))
            {
                inventoryPath = objPath;
                lg2::info(
                    "EID {EID}: Found board inventory path {PATH} for board {BOARD}",
                    "EID", eid, "PATH", inventoryPath, "BOARD", boardName);
                break;
            }
        }
    }

    if (inventoryPath == configPath)
    {
        lg2::info(
            "EID {EID}: Board {BOARD} not found in inventory, using config path {PATH}",
            "EID", eid, "BOARD", boardName, "PATH", configPath);
        queryDevicesForEndpoint(configs, configPath, endpointPath, eid, "",
                                bridgePool, bridgedEndpoints);
        return;
    }

    conn->async_method_call(
        [this, configs, endpointPath, eid, inventoryPath, configPath,
         bridgePool, bridgedEndpoints](const boost::system::error_code& ec2,
                                       const GetSubTreeType& ret2) {
            processNvidiaMctpVdmConfigSearch(
                configs, endpointPath, eid, inventoryPath, configPath, ec2,
                ret2, bridgePool, bridgedEndpoints);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", inventoryPath, 0,
        std::vector<std::string>{
            "xyz.openbmc_project.Configuration.NvidiaMctpVdm"});
}

void DeviceManager::processNvidiaMctpVdmConfigSearch(
    const SensorConfigs& configs, const std::string& endpointPath, uint8_t eid,
    const std::string& inventoryPath, const std::string& configPath,
    const boost::system::error_code& ec, const GetSubTreeType& ret,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool,
    const std::optional<std::vector<std::string>>& bridgedEndpoints)
{
    std::string finalConfigPath = configPath;

    if (!ec && !ret.empty())
    {
        const std::string& objPath = ret[0].first;
        if (objPath.find(inventoryPath) != std::string::npos)
        {
            finalConfigPath = objPath;
        }
    }
    else
    {
        lg2::info(
            "EID {EID}: NvidiaMctpVdm config not found under board, using original {PATH}",
            "EID", eid, "PATH", configPath);
    }

    // inventoryPath is the Entity-Manager board object (Item.Chassis /
    // Item.Board) resolved via the endpoint's configured_by association; pass
    // it as the chassis path so the device can publish its chassis interfaces
    // there.
    queryDevicesForEndpoint(configs, finalConfigPath, endpointPath, eid,
                            inventoryPath, bridgePool, bridgedEndpoints);
}

void DeviceManager::processEndpoint(
    const SensorConfigs& configs, const std::string& endpointPath,
    const boost::system::error_code& ec, const SensorBaseConfigMap& endpoint,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoint: Error:{ERROR}", "ERROR",
                   ec.message());
        return;
    }

    auto hasEid = endpoint.find("EID");
    uint8_t eid{};

    if (hasEid != endpoint.end())
    {
        const auto* eidPtr = std::get_if<uint8_t>(&hasEid->second);
        if (eidPtr != nullptr)
        {
            eid = *eidPtr;
        }
        else
        {
            lg2::error(
                "Error processing MCTP endpoint: Property EID does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "Error processing MCTP endpoint: Property EID not found in the configuration.");
        return;
    }

    auto hasMctpTypes = endpoint.find("SupportedMessageTypes");
    std::vector<uint8_t> mctpTypes{};

    if (hasMctpTypes != endpoint.end())
    {
        const auto* mctpTypePtr =
            std::get_if<std::vector<uint8_t>>(&hasMctpTypes->second);
        if (mctpTypePtr != nullptr)
        {
            mctpTypes = *mctpTypePtr;
        }
        else
        {
            lg2::error(
                "Error processing MCTP endpoint with eid {EID} : Property SupportedMessageTypes does not have valid type.",
                "EID", eid);
            return;
        }
    }
    else
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : Property SupportedMessageTypes not found in the configuration.",
            "EID", eid);
        return;
    }

    if (std::find(mctpTypes.begin(), mctpTypes.end(),
                  ocp::accelerator_management::messageType) != mctpTypes.end())
    {
        lg2::info("Found OCP MCTP VDM Endpoint with ID {EID}", "EID", eid);
        checkAssociationAndQueryDevice(configs, endpointPath, eid, bridgePool);
    }
}

void DeviceManager::queryEndpoints(const SensorConfigs& configs,
                                   const boost::system::error_code& ec,
                                   const GetSubTreeType& ret)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoints: {ERROR}", "ERROR",
                   ec.message());
        return;
    }

    if (ret.empty())
    {
        return;
    }

    for (const auto& [objPath, services] : ret)
    {
        for (const auto& [service, ifaces] : services)
        {
            for (const auto& iface : ifaces)
            {
                if (iface == "xyz.openbmc_project.MCTP.Endpoint")
                {
                    // GetAll with an empty interface returns properties from
                    // all interfaces on the object, so a bridge endpoint's
                    // PoolStart/PoolEnd (on the Bridge1 interface) are visible.
                    conn->async_method_call(
                        [this, configs, endpointPath{objPath}](
                            const boost::system::error_code& ec,
                            const SensorBaseConfigMap& endpoint) {
                            auto bridgePool = extractBridgePool(endpoint);
                            processEndpoint(configs, endpointPath, ec, endpoint,
                                            bridgePool);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", "");
                }
            }
        }
    }
}

void DeviceManager::discoverDevices(const SensorConfigs& configs)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this, configs](const boost::system::error_code& ec,
                        const GetSubTreeType& ret) {
            queryEndpoints(configs, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void DeviceManager::createSensors()
{
    if (!conn)
    {
        lg2::error("Connection not created");
        return;
    }

    SensorConfigs configs;
    configs.pollRate = sensorPollRateMs;
    discoverDevices(configs);
}

void DeviceManager::onConfigInterfaceRemoved(sdbusplus::message_t& message)
{
    sdbusplus::object_path removedPath;
    std::vector<std::string> interfaces;

    message.read(removedPath, interfaces);

    // If the xyz.openbmc_project.Configuration.X interface was removed
    // for one or more sensors, delete those sensor objects.
    auto sensorIt = gpuDevices.begin();
    while (sensorIt != gpuDevices.end())
    {
        if ((sensorIt->second->getPath() == removedPath) &&
            (std::ranges::any_of(interfaces, [](const std::string& i) {
                return i.starts_with(configInterfacePrefix);
            })))
        {
            sensorIt = gpuDevices.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }

    auto smaSensorIt = smaDevices.begin();
    while (smaSensorIt != smaDevices.end())
    {
        if ((smaSensorIt->second->getPath() == removedPath) &&
            (std::ranges::any_of(interfaces, [](const std::string& i) {
                return i.starts_with(configInterfacePrefix);
            })))
        {
            smaSensorIt = smaDevices.erase(smaSensorIt);
        }
        else
        {
            smaSensorIt++;
        }
    }

    auto pcieSensorIt = pcieDevices.begin();
    while (pcieSensorIt != pcieDevices.end())
    {
        if ((pcieSensorIt->second->getPath() == removedPath) &&
            (std::ranges::any_of(interfaces, [](const std::string& i) {
                return i.starts_with(configInterfacePrefix);
            })))
        {
            pcieSensorIt = pcieDevices.erase(pcieSensorIt);
        }
        else
        {
            pcieSensorIt++;
        }
    }
}

void DeviceManager::registerEndpoint(
    const std::string& endpointPath, uint8_t eid,
    const std::shared_ptr<DeviceInterface>& device)
{
    endpoints[endpointPath] =
        EndpointRecord{device, eid, EndpointState::Init, {}};
    fetchEndpointUuid(endpointPath);
}

void DeviceManager::fetchEndpointUuid(const std::string& endpointPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", endpointPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, endpointPath](const boost::system::error_code& ec,
                             const std::string& uuid) {
            // UUID is an optional interface on the endpoint; absence is fine.
            if (ec || uuid.empty())
            {
                return;
            }
            auto it = endpoints.find(endpointPath);
            if (it == endpoints.end())
            {
                return;
            }
            it->second.uuid = uuid;
            uuidToDevice[uuid] = it->second.device;
        });
}

void DeviceManager::onEndpointRemoved(sdbusplus::message_t& msg)
{
    sdbusplus::object_path objPath;
    std::vector<std::string> removedInterfaces;
    msg.read(objPath, removedInterfaces);

    // Only react when the MCTP endpoint interface itself goes away.
    if (std::ranges::find(removedInterfaces,
                          "au.com.codeconstruct.MCTP.Endpoint1") ==
        removedInterfaces.end())
    {
        return;
    }

    // applyEvent is a no-op if we do not track this endpoint.
    applyEvent(objPath.str, EndpointEvent::EndpointRemoved);
}

void DeviceManager::onEndpointAdded(sdbusplus::message_t& msg)
{
    sdbusplus::object_path objPath;
    // Read only the object path; the interface/property dictionary carries
    // many typed properties whose variant types we do not want to depend on.
    msg.read(objPath);

    auto it = endpoints.find(objPath.str);
    if (it == endpoints.end())
    {
        // New endpoint path. It may be a device we already manage that was
        // re-enumerated with a different EID (path changes with the EID), so
        // try to re-attach it by UUID. Guard against churn from non-endpoint
        // mctp objects (networks, interfaces).
        if (objPath.str.find("/endpoints/") != std::string::npos)
        {
            reattachByUuid(objPath.str);
        }
        return;
    }

    if (it->second.state != EndpointState::Offline)
    {
        return;
    }

    verifyAndReadd(objPath.str);
}

void DeviceManager::reattachByUuid(const std::string& endpointPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", endpointPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, endpointPath](const boost::system::error_code& ec,
                             const std::string& uuid) {
            std::shared_ptr<DeviceInterface> device;
            if (!ec && !uuid.empty())
            {
                auto uit = uuidToDevice.find(uuid);
                if (uit != uuidToDevice.end())
                {
                    device = uit->second.lock();
                }
            }
            if (!device)
            {
                // Unknown device (or no UUID) -> let discovery create it.
                scheduleRescan();
                return;
            }

            // Known device re-enumerated at a new EID: read the new EID and
            // re-bind the existing device object in place (no rebuild).
            sdbusplus::asio::getProperty<uint8_t>(
                *conn, "au.com.codeconstruct.MCTP1", endpointPath,
                "xyz.openbmc_project.MCTP.Endpoint", "EID",
                [this, endpointPath, uuid, device](
                    const boost::system::error_code& eidEc, uint8_t newEid) {
                    if (eidEc)
                    {
                        lg2::error(
                            "Failed to read EID for re-added endpoint {PATH}",
                            "PATH", endpointPath);
                        return;
                    }
                    // Drop the stale (Offline) entry still tracking this device
                    // under its previous endpoint path.
                    std::erase_if(endpoints, [&device](const auto& kv) {
                        return kv.second.device.lock() == device;
                    });
                    device->setEid(newEid);
                    endpoints[endpointPath] = EndpointRecord{
                        device, newEid, EndpointState::Offline, uuid};
                    uuidToDevice[uuid] = device;
                    applyEvent(endpointPath, EndpointEvent::EndpointReadded);
                });
        });
}

void DeviceManager::verifyAndReadd(const std::string& endpointPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", endpointPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, endpointPath](const boost::system::error_code& ec,
                             const std::string& uuid) {
            auto it = endpoints.find(endpointPath);
            if (it == endpoints.end() ||
                it->second.state != EndpointState::Offline)
            {
                return;
            }

            const std::string& expected = it->second.uuid;
            if (!ec && !uuid.empty() && !expected.empty() && uuid != expected)
            {
                // Same path/EID but a different device took it over; treat as
                // a fresh device rather than re-attaching the old one.
                lg2::warning(
                    "MCTP endpoint {PATH} reappeared with different UUID; rescanning",
                    "PATH", endpointPath);
                endpoints.erase(it);
                scheduleRescan();
                return;
            }

            applyEvent(endpointPath, EndpointEvent::EndpointReadded);
        });
}

void DeviceManager::applyEvent(const std::string& endpointPath,
                               EndpointEvent event)
{
    auto it = endpoints.find(endpointPath);
    if (it == endpoints.end())
    {
        return;
    }
    auto& rec = it->second;
    auto [next, action] = nextState(rec.state, event);
    rec.state = next;

    auto device = rec.device.lock();
    switch (action)
    {
        case EndpointAction::GoOffline:
            lg2::info("MCTP endpoint {PATH} (eid {EID}) offline; stopping poll",
                      "PATH", endpointPath, "EID", rec.eid);
            if (device)
            {
                device->setOffline();
            }
            break;
        case EndpointAction::GoOnline:
            lg2::info("MCTP endpoint {PATH} (eid {EID}) online; starting poll",
                      "PATH", endpointPath, "EID", rec.eid);
            if (device)
            {
                device->setOnline();
            }
            break;
        case EndpointAction::None:
            break;
    }
}

void DeviceManager::onConnectivityChanged(sdbusplus::message_t& msg)
{
    std::string iface;
    boost::container::flat_map<std::string, std::variant<std::string>> props;
    std::vector<std::string> invalidated;
    msg.read(iface, props, invalidated);

    auto it = props.find("Connectivity");
    if (it == props.end())
    {
        return;
    }

    const auto* value = std::get_if<std::string>(&it->second);
    if (value == nullptr)
    {
        return;
    }

    const std::string endpointPath{msg.get_path()};

    if (*value == "Degraded")
    {
        applyEvent(endpointPath, EndpointEvent::ConnectivityDegraded);
    }
    else if (*value == "Available")
    {
        applyEvent(endpointPath, EndpointEvent::ConnectivityAvailable);
    }
}
