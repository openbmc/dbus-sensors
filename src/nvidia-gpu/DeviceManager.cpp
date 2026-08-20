/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSmaDevice.hpp"
#include "NvidiaUtils.hpp"
#include "Utils.hpp"

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
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

// Bound on how many times a transient D-Bus failure during a single endpoint's
// config-resolution chain will re-trigger a discovery sweep before giving up.
static constexpr unsigned maxDiscoveryRetries = 3;

// EntityManager exports an array-of-objects config property as one indexed
// interface per element, e.g. Configuration.MCTPUSBDevice.BridgedEndpoints0.
// Returns the element index, or nullopt when the interface is not one of them.
static std::optional<size_t> bridgedEndpointIndex(const std::string& iface)
{
    static constexpr std::string_view marker = ".BridgedEndpoints";

    const size_t pos = iface.rfind(marker);
    if (pos == std::string::npos)
    {
        return std::nullopt;
    }

    const std::string_view digits{iface};
    const std::string_view suffix = digits.substr(pos + marker.size());
    if (suffix.empty())
    {
        return std::nullopt;
    }

    size_t index{};
    const auto* end = suffix.data() + suffix.size();
    const auto [ptr, ec] = std::from_chars(suffix.data(), end, index);
    if (ec != std::errc{} || ptr != end)
    {
        return std::nullopt;
    }

    return index;
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

// The devices behind a bridge are described by the config's BridgedEndpoints
// records, one per EID in the bridge's pool. Returns the records to use, or
// nothing when the endpoint is not a bridge or the records do not describe
// the pool it reported.
static std::vector<DeviceManager::BridgedEndpoint> selectBridgedEndpoints(
    const std::optional<std::pair<uint8_t, uint8_t>>& bridgePool,
    const std::vector<DeviceManager::BridgedEndpoint>& records, uint8_t eid)
{
    if (!bridgePool || bridgePool->first == 0 || bridgePool->second == 0)
    {
        return {};
    }

    const uint8_t expectedCount = bridgePool->second - bridgePool->first + 1;

    if (records.empty())
    {
        // Without the records we cannot name or place the pool devices, so
        // warn rather than dropping them silently.
        lg2::warning(
            "EID {EID} is a bridge with pool range {START}-{END} but its config has no usable BridgedEndpoints; {COUNT} bridged device(s) will not be created",
            "EID", eid, "START", bridgePool->first, "END", bridgePool->second,
            "COUNT", expectedCount);
        return {};
    }

    if (records.size() != expectedCount)
    {
        lg2::error(
            "EID {EID}: BridgedEndpoints count mismatch. Expected {EXPECTED}, got {ACTUAL}",
            "EID", eid, "EXPECTED", expectedCount, "ACTUAL", records.size());
        return {};
    }

    lg2::info("EID {EID} is a bridge with pool range {START}-{END}", "EID", eid,
              "START", bridgePool->first, "END", bridgePool->second);

    if (expectedCount > 1)
    {
        // Only the count can be checked, so say when the order is load
        // bearing rather than leaving the assumption to the schema.
        lg2::info(
            "EID {EID}: pairing {COUNT} pool EIDs with BridgedEndpoints by position, which assumes the bridge hands them out in the order the records are listed",
            "EID", eid, "COUNT", expectedCount);
    }

    return records;
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

// Entity manager publishes the name a configuration was matched under here,
// unescaped, so a board can be found by the name a platform record refers to
// it by rather than by rebuilding the object path that name was exported at.
static constexpr std::string_view probeIface =
    "xyz.openbmc_project.Configuration.Probe";

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

bool DeviceManager::retryDiscovery(const sdbusplus::object_path& mctpObjectPath,
                                   uint8_t eid)
{
    unsigned& count = discoveryRetries[mctpObjectPath];
    if (count >= maxDiscoveryRetries)
    {
        // Keep the entry at the cap (do not erase) so a persistently failing
        // endpoint stops re-triggering sweeps; a normal event-driven rescan
        // will still process it if its config later appears.
        lg2::error(
            "EID {EID}: giving up discovery for {PATH} after {MAX} transient retries",
            "EID", eid, "PATH", mctpObjectPath, "MAX", maxDiscoveryRetries);
        return false;
    }
    ++count;
    lg2::warning(
        "EID {EID}: transient discovery error for {PATH}, scheduling retry {N}/{MAX}",
        "EID", eid, "PATH", mctpObjectPath, "N", count, "MAX",
        maxDiscoveryRetries);
    scheduleRescan();
    return true;
}
void DeviceManager::processQueryDeviceIdResponse(
    const SensorConfigs& configs, const DeviceConfig& deviceConfig,
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
    bool isEndpointItself, const std::error_code& sendRecvMsgResult,
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

    const sdbusplus::object_path& path = deviceConfig.path;
    const std::string& deviceName = deviceConfig.name;

    switch (static_cast<gpu::DeviceIdentification>(responseDeviceType))
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
        {
            lg2::info(
                "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            const std::string& gpuName = deviceName;

            std::shared_ptr<GpuDevice>& gpu = gpuDevices[gpuName];

            if (gpu == nullptr)
            {
                gpu = std::make_shared<GpuDevice>(configs, gpuName, path, conn,
                                                  eid, io, mctpRequester,
                                                  objectServer);

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

            const std::string& smaName = deviceName;

            auto existing =
                std::ranges::find(smaDevices, smaName, &SmaDeviceRecord::name);

            if (existing == smaDevices.end())
            {
                auto sma = std::make_shared<SmaDevice>(
                    configs, smaName, path, conn, eid, io, mctpRequester,
                    objectServer);

                sma->init();

                // Only the endpoint itself owns the mctpd endpoint object.
                // Bridged pool devices share the bridge's path, so leaving
                // theirs empty keeps a Connectivity signal from matching more
                // than the bridge's own record.
                smaDevices.emplace_back(SmaDeviceRecord{
                    .device = std::move(sma),
                    .name = smaName,
                    .mctpObjectPath = isEndpointItself
                                          ? mctpObjectPath
                                          : sdbusplus::object_path{},
                    .uuid = {},
                    .eid = eid,
                    .state = EndpointState::Init});

                if (isEndpointItself)
                {
                    fetchEndpointUuid(mctpObjectPath);
                    applyEvent(mctpObjectPath, EndpointEvent::InitComplete);
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

            const std::string& pcieName = deviceName;

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
    const SensorConfigs& configs, const DeviceConfig& deviceConfig,
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
    bool isEndpointItself)
{
    // Reaching here means the config-resolution chain succeeded for this
    // endpoint, so clear any transient-error retry budget accrued for it.
    discoveryRetries.erase(mctpObjectPath);

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
        [this, configs, deviceConfig, mctpObjectPath, eid, isEndpointItself,
         queryDeviceIdentificationRequest](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(configs, deviceConfig, mctpObjectPath,
                                         eid, isEndpointItself, ec, response);
        });
}

void DeviceManager::queryDevicesForEndpoint(
    const SensorConfigs& configs, const DeviceConfig& deviceConfig,
    const std::string& boardName, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const std::optional<std::pair<uint8_t, uint8_t>>& bridgePool,
    const std::vector<BridgedEndpoint>& bridgedEndpoints)
{
    queryDeviceIdentification(configs, deviceConfig, mctpObjectPath, eid, true);

    if (!bridgePool)
    {
        return;
    }

    // Walk the bridge's EID pool alongside the BridgedEndpoints records. Each
    // record is paired with a configuration of its own rather than reusing the
    // bridge's: an SMA and the device behind it can sit on different boards.
    uint8_t index = 0;
    for (const BridgedEndpoint& endpoint : bridgedEndpoints)
    {
        const uint8_t bridgedEid = bridgePool->first + index;
        ++index;

        // A record that names no board of its own is on the bridge's.
        const std::string& board =
            endpoint.board.empty() ? boardName : endpoint.board;

        findBoardInventoryPath(
            board, endpoint.name, bridgedEid,
            [this, configs, mctpObjectPath,
             bridgedEid](const DeviceConfig& resolved) {
                queryDeviceIdentification(configs, resolved, mctpObjectPath,
                                          bridgedEid, false);
            });
    }
}

void DeviceManager::checkAssociationAndQueryDevice(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    const sdbusplus::object_path associationPath =
        mctpObjectPath / "configured_by";

    conn->async_method_call(
        [this, configs, mctpObjectPath, eid, associationPath, bridgePool](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            if (ec || ret.empty())
            {
                // Only an endpoint the reactor set up from a configuration
                // has this association. The devices behind a bridge get
                // their own endpoints without one, so this is the normal
                // case for them rather than a failure.
                lg2::debug(
                    "EID {EID}: No association found at {PATH}, skipping endpoint",
                    "EID", eid, "PATH", associationPath);
                return;
            }
            getAssociationEndpoints(configs, mctpObjectPath, eid,
                                    associationPath, ret[0].first, bridgePool);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject",
        associationPath.string(),
        std::vector<std::string>{"xyz.openbmc_project.Association"});
}

void DeviceManager::getAssociationEndpoints(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const sdbusplus::object_path& associationPath,
    const std::string& associationService,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    conn->async_method_call(
        [this, configs, mctpObjectPath, eid,
         bridgePool](const boost::system::error_code& ec,
                     const std::variant<std::vector<std::string>>& value) {
            processAssociationEndpointsResult(configs, mctpObjectPath, eid, ec,
                                              value, bridgePool);
        },
        associationService, associationPath, "org.freedesktop.DBus.Properties",
        "Get", "xyz.openbmc_project.Association", "endpoints");
}

void DeviceManager::processAssociationEndpointsResult(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const boost::system::error_code& ec,
    const std::variant<std::vector<std::string>>& value,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    if (ec)
    {
        lg2::error("EID {EID}: Failed to get endpoints property: {ERROR}",
                   "EID", eid, "ERROR", ec.message());
        retryDiscovery(mctpObjectPath, eid);
        return;
    }

    const auto* endpointsPtr = std::get_if<std::vector<std::string>>(&value);
    if ((endpointsPtr == nullptr) || endpointsPtr->empty())
    {
        lg2::error("EID {EID}: endpoints property is empty, skipping", "EID",
                   eid);
        return;
    }

    getConfigService(configs, mctpObjectPath, eid, (*endpointsPtr)[0],
                     bridgePool);
}

void DeviceManager::getConfigService(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const sdbusplus::object_path& configPath,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    resolveObjectService(
        conn, configPath,
        [this, configs, mctpObjectPath, eid, configPath,
         bridgePool](const std::string& service,
                     const std::vector<std::string>& interfaces) {
            if (service.empty())
            {
                lg2::error("EID {EID}: no service owns config path {PATH}",
                           "EID", eid, "PATH", configPath);
                retryDiscovery(mctpObjectPath, eid);
                return;
            }
            getConfigProperties(configs, mctpObjectPath, eid, configPath,
                                service, interfaces, bridgePool);
        });
}

void DeviceManager::getConfigProperties(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const sdbusplus::object_path& configPath,
    const std::string& configService,
    const std::vector<std::string>& interfaces,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
    std::vector<std::string> baseIfaces;
    std::vector<std::pair<size_t, std::string>> bridgedIfaces;
    for (const std::string& iface : interfaces)
    {
        if (!iface.starts_with(configInterfacePrefix))
        {
            continue;
        }
        const std::optional<size_t> index = bridgedEndpointIndex(iface);
        if (index)
        {
            bridgedIfaces.emplace_back(*index, iface);
        }
        else
        {
            baseIfaces.push_back(iface);
        }
    }

    if (baseIfaces.empty())
    {
        lg2::error("EID {EID}: No configuration interface on {PATH}", "EID",
                   eid, "PATH", configPath);
        return;
    }

    // Keyed by element index so the bridged devices stay in pool order
    // whatever order the replies arrive in.
    struct Fetch
    {
        SensorBaseConfigMap base;
        std::map<size_t, BridgedEndpoint> bridged;
        size_t pending{};
        bool failed{false};
    };
    auto fetch = std::make_shared<Fetch>();
    fetch->pending = baseIfaces.size() + bridgedIfaces.size();

    std::function<void()> arrived = [this, configs, mctpObjectPath, eid,
                                     configPath, bridgePool, fetch]() {
        if (--fetch->pending != 0)
        {
            return;
        }
        if (fetch->failed)
        {
            retryDiscovery(mctpObjectPath, eid);
            return;
        }

        std::vector<BridgedEndpoint> bridged;
        bridged.reserve(fetch->bridged.size());
        for (auto& [index, endpoint] : fetch->bridged)
        {
            bridged.push_back(std::move(endpoint));
        }

        processConfigPropertiesResult(configs, mctpObjectPath, eid, configPath,
                                      fetch->base, bridged, bridgePool);
    };

    for (const std::string& iface : baseIfaces)
    {
        conn->async_method_call(
            [fetch, arrived](const boost::system::error_code& ec,
                             const SensorBaseConfigMap& props) {
                if (ec)
                {
                    fetch->failed = true;
                }
                else
                {
                    fetch->base.insert(props.begin(), props.end());
                }
                arrived();
            },
            configService, configPath, "org.freedesktop.DBus.Properties",
            "GetAll", iface);
    }

    for (const auto& [index, iface] : bridgedIfaces)
    {
        conn->async_method_call(
            [fetch, arrived, index, eid](const boost::system::error_code& ec,
                                         const SensorBaseConfigMap& props) {
                if (ec)
                {
                    fetch->failed = true;
                    arrived();
                    return;
                }

                BridgedEndpoint endpoint;
                const auto nameIt = props.find("Name");
                if (nameIt != props.end())
                {
                    const auto* name =
                        std::get_if<std::string>(&nameIt->second);
                    if (name != nullptr)
                    {
                        endpoint.name = *name;
                    }
                }
                const auto boardIt = props.find("Board");
                if (boardIt != props.end())
                {
                    const auto* board =
                        std::get_if<std::string>(&boardIt->second);
                    if (board != nullptr)
                    {
                        endpoint.board = *board;
                    }
                }

                if (endpoint.name.empty())
                {
                    lg2::error(
                        "EID {EID}: bridged endpoint {INDEX} has no usable Name, skipping",
                        "EID", eid, "INDEX", index);
                }
                else
                {
                    fetch->bridged.emplace(index, std::move(endpoint));
                }
                arrived();
            },
            configService, configPath, "org.freedesktop.DBus.Properties",
            "GetAll", iface);
    }
}

void DeviceManager::processConfigPropertiesResult(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const sdbusplus::object_path& configPath,
    const SensorBaseConfigMap& configProps,
    const std::vector<BridgedEndpoint>& bridgedEndpoints,
    std::optional<std::pair<uint8_t, uint8_t>> bridgePool)
{
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

    const std::vector<BridgedEndpoint> bridged =
        selectBridgedEndpoints(bridgePool, bridgedEndpoints, eid);

    std::string boardName;
    auto boardIt = configProps.find("Board");
    if (boardIt != configProps.end())
    {
        const auto* boardPtr = std::get_if<std::string>(&boardIt->second);
        if (boardPtr != nullptr)
        {
            boardName = *boardPtr;
        }
    }

    if (boardName.empty())
    {
        // The board is what holds the record naming this device, so without
        // one there is nothing to pair the device with.
        lg2::error(
            "EID {EID}: config {PATH} names no board, not creating {NAME}",
            "EID", eid, "PATH", configPath, "NAME", deviceName);
        return;
    }

    findBoardInventoryPath(
        boardName, deviceName, eid,
        [this, configs, boardName, mctpObjectPath, eid, bridgePool,
         bridged](const DeviceConfig& resolved) {
            queryDevicesForEndpoint(configs, resolved, boardName,
                                    mctpObjectPath, eid, bridgePool, bridged);
        });
}

void DeviceManager::collectBoardPaths(std::function<void()> done)
{
    boardPaths.clear();

    conn->async_method_call(
        [this, done{std::move(done)}](const boost::system::error_code& ec,
                                      const GetSubTreeType& ret) {
            if (ec || ret.empty())
            {
                done();
                return;
            }

            // The names arrive one reply at a time, so the sweep can only
            // start once the last of them has.
            struct Collect
            {
                size_t pending{};
                std::function<void()> done;
            };
            auto collect = std::make_shared<Collect>();
            collect->pending = ret.size();
            collect->done = done;

            for (const auto& [objPath, services] : ret)
            {
                if (services.empty())
                {
                    if (--collect->pending == 0)
                    {
                        collect->done();
                    }
                    continue;
                }

                conn->async_method_call(
                    [this, collect,
                     objPath](const boost::system::error_code& propEc,
                              const SensorBaseConfigMap& props) {
                        if (!propEc)
                        {
                            const auto nameIt = props.find("Name");
                            if (nameIt != props.end())
                            {
                                const auto* name =
                                    std::get_if<std::string>(&nameIt->second);
                                if (name != nullptr)
                                {
                                    boardPaths[*name] = objPath;
                                }
                            }
                        }
                        if (--collect->pending == 0)
                        {
                            collect->done();
                        }
                    },
                    services.front().first, objPath,
                    "org.freedesktop.DBus.Properties", "GetAll", probeIface);
            }
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree",
        "/xyz/openbmc_project/inventory", 0,
        std::vector<std::string>{std::string(probeIface)});
}

// The interfaces a board exposes an MCTP VDM configuration on, one per kind
// of device.
static constexpr std::string_view mctpVdmGpuIface =
    "xyz.openbmc_project.Configuration.NvidiaMctpVdmGpu";
static constexpr std::string_view mctpVdmSmaIface =
    "xyz.openbmc_project.Configuration.NvidiaMctpVdmSma";
static constexpr std::string_view mctpVdmCxIface =
    "xyz.openbmc_project.Configuration.NvidiaMctpVdmCx";

static bool isMctpVdmIface(std::string_view iface)
{
    return iface == mctpVdmGpuIface || iface == mctpVdmSmaIface ||
           iface == mctpVdmCxIface;
}

// Pair the device with the record its board exposes for it. The platform
// named the device by that record's PlatformConfigName, which is the only
// thing that tells apart several records a board carries for devices of one
// kind. The record's DeviceName is what the device is then called.
static void selectMctpVdmConfig(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    const sdbusplus::object_path& inventoryPath,
    const std::string& inventoryName, uint8_t eid,
    const boost::system::error_code& ec, const GetSubTreeType& ret,
    const std::function<void(const DeviceManager::DeviceConfig&)>& done)
{
    // The subtree reply already names the service holding each record, so the
    // records can be read without asking the mapper for their owner again.
    struct Candidate
    {
        std::string path;
        std::string service;
        std::string iface;
    };
    std::vector<Candidate> candidates;

    if (!ec)
    {
        for (const auto& [objPath, services] : ret)
        {
            if (!objPath.starts_with(inventoryPath.string()))
            {
                continue;
            }
            for (const auto& [service, ifaces] : services)
            {
                for (const std::string& iface : ifaces)
                {
                    if (isMctpVdmIface(iface))
                    {
                        candidates.emplace_back(objPath, service, iface);
                    }
                }
            }
        }
    }

    if (candidates.empty())
    {
        lg2::error(
            "EID {EID}: board {PATH} exposes no MCTP VDM configuration, not creating {NAME}",
            "EID", eid, "PATH", inventoryPath, "NAME", inventoryName);
        return;
    }

    // Only one record can carry the name, but the replies arrive in any
    // order, so what has been seen so far is shared between them.
    struct Search
    {
        size_t pending{};
        bool matched{false};
    };
    auto search = std::make_shared<Search>();
    search->pending = candidates.size();

    for (const Candidate& candidate : candidates)
    {
        conn->async_method_call(
            [search, path{candidate.path}, inventoryPath, inventoryName, eid,
             done](const boost::system::error_code& propEc,
                   const SensorBaseConfigMap& props) {
                --search->pending;

                if (!propEc && !search->matched)
                {
                    const auto nameIt = props.find("PlatformConfigName");
                    if (nameIt != props.end())
                    {
                        const auto* name =
                            std::get_if<std::string>(&nameIt->second);
                        if (name != nullptr && *name == inventoryName)
                        {
                            // The record says what the device is called on
                            // D-Bus; the platform's name only paired them up.
                            const auto deviceIt = props.find("DeviceName");
                            const auto* device = (deviceIt != props.end())
                                                     ? std::get_if<std::string>(
                                                           &deviceIt->second)
                                                     : nullptr;
                            search->matched = true;
                            done({.path = path,
                                  .name = (device != nullptr) ? *device
                                                              : inventoryName});
                        }
                    }
                }

                if (search->pending == 0 && !search->matched)
                {
                    lg2::error(
                        "EID {EID}: board {PATH} exposes no configuration named {NAME}, not creating the device",
                        "EID", eid, "PATH", inventoryPath, "NAME",
                        inventoryName);
                }
            },
            candidate.service, candidate.path,
            "org.freedesktop.DBus.Properties", "GetAll", candidate.iface);
    }
}

void DeviceManager::findBoardInventoryPath(
    const std::string& boardName, const std::string& inventoryName, uint8_t eid,
    const ConfigPathHandler& done)
{
    auto board = boardPaths.find(boardName);
    if (board == boardPaths.end())
    {
        lg2::error(
            "EID {EID}: board {BOARD} not found in inventory, not creating {NAME}",
            "EID", eid, "BOARD", boardName, "NAME", inventoryName);
        return;
    }

    const sdbusplus::object_path& inventoryPath = board->second;
    lg2::info("EID {EID}: Found board inventory path {PATH} for board {BOARD}",
              "EID", eid, "PATH", inventoryPath, "BOARD", boardName);

    conn->async_method_call(
        [conn{conn}, eid, inventoryPath, inventoryName, done](
            const boost::system::error_code& ec2, const GetSubTreeType& ret2) {
            selectMctpVdmConfig(conn, inventoryPath, inventoryName, eid, ec2,
                                ret2, done);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree",
        inventoryPath.string(), 0,
        std::vector<std::string>{std::string(mctpVdmGpuIface),
                                 std::string(mctpVdmSmaIface),
                                 std::string(mctpVdmCxIface)});
}

void DeviceManager::processEndpoint(
    const SensorConfigs& configs, const sdbusplus::object_path& mctpObjectPath,
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
        checkAssociationAndQueryDevice(configs, mctpObjectPath, eid,
                                       bridgePool);
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
                        [this, configs, mctpObjectPath{objPath}](
                            const boost::system::error_code& ec,
                            const SensorBaseConfigMap& endpoint) {
                            auto bridgePool = extractBridgePool(endpoint);
                            processEndpoint(configs, mctpObjectPath, ec,
                                            endpoint, bridgePool);
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
    collectBoardPaths([this, configs]() { discoverDevices(configs); });
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

    std::erase_if(smaDevices, [&](const SmaDeviceRecord& rec) {
        return (rec.device->getPath() == removedPath) &&
               std::ranges::any_of(interfaces, [](const std::string& i) {
                   return i.starts_with(configInterfacePrefix);
               });
    });
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

void DeviceManager::fetchEndpointUuid(
    const sdbusplus::object_path& mctpObjectPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", mctpObjectPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, mctpObjectPath](const boost::system::error_code& ec,
                               const std::string& uuid) {
            // UUID is an optional interface on the endpoint; absence is fine.
            if (ec || uuid.empty())
            {
                return;
            }
            auto it = std::ranges::find(smaDevices, mctpObjectPath,
                                        &SmaDeviceRecord::mctpObjectPath);
            if (it == smaDevices.end())
            {
                return;
            }
            it->uuid = uuid;
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
    applyEvent(objPath, EndpointEvent::EndpointRemoved);
}

void DeviceManager::onEndpointAdded(sdbusplus::message_t& msg)
{
    sdbusplus::object_path objPath;
    // Read only the object path; the interface/property dictionary carries
    // many typed properties whose variant types we do not want to depend on.
    msg.read(objPath);

    auto it = std::ranges::find(smaDevices, objPath,
                                &SmaDeviceRecord::mctpObjectPath);
    if (it == smaDevices.end())
    {
        // New endpoint path. It may be a device we already manage that was
        // re-enumerated with a different EID (path changes with the EID), so
        // try to re-attach it by UUID. Guard against churn from non-endpoint
        // mctp objects (networks, interfaces).
        if (objPath.string().find("/endpoints/") != std::string::npos)
        {
            reattachByUuid(objPath);
        }
        return;
    }

    if (it->state != EndpointState::Offline)
    {
        return;
    }

    verifyAndReadd(objPath);
}

void DeviceManager::reattachByUuid(const sdbusplus::object_path& mctpObjectPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", mctpObjectPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, mctpObjectPath](const boost::system::error_code& ec,
                               const std::string& uuid) {
            if (ec || uuid.empty() ||
                std::ranges::find(smaDevices, uuid, &SmaDeviceRecord::uuid) ==
                    smaDevices.end())
            {
                // Unknown device (or no UUID) -> let discovery create it.
                scheduleRescan();
                return;
            }

            // Known device re-enumerated at a new EID: read the new EID and
            // re-bind the existing device object in place (no rebuild).
            sdbusplus::asio::getProperty<uint8_t>(
                *conn, "au.com.codeconstruct.MCTP1", mctpObjectPath,
                "xyz.openbmc_project.MCTP.Endpoint", "EID",
                [this, mctpObjectPath,
                 uuid](const boost::system::error_code& eidEc, uint8_t newEid) {
                    if (eidEc)
                    {
                        lg2::error(
                            "Failed to read EID for re-added endpoint {PATH}",
                            "PATH", mctpObjectPath);
                        return;
                    }
                    auto it = std::ranges::find(smaDevices, uuid,
                                                &SmaDeviceRecord::uuid);
                    if (it == smaDevices.end())
                    {
                        return;
                    }
                    // The record moves with the device, so its previous
                    // endpoint path leaves nothing behind to clean up.
                    it->device->setEid(newEid);
                    it->mctpObjectPath = mctpObjectPath;
                    it->eid = newEid;
                    applyEvent(mctpObjectPath, EndpointEvent::EndpointReadded);
                });
        });
}

void DeviceManager::verifyAndReadd(const sdbusplus::object_path& mctpObjectPath)
{
    sdbusplus::asio::getProperty<std::string>(
        *conn, "au.com.codeconstruct.MCTP1", mctpObjectPath,
        "xyz.openbmc_project.Common.UUID", "UUID",
        [this, mctpObjectPath](const boost::system::error_code& ec,
                               const std::string& uuid) {
            auto it = std::ranges::find(smaDevices, mctpObjectPath,
                                        &SmaDeviceRecord::mctpObjectPath);
            if (it == smaDevices.end() || it->state != EndpointState::Offline)
            {
                return;
            }

            const std::string& expected = it->uuid;
            if (!ec && !uuid.empty() && !expected.empty() && uuid != expected)
            {
                // Same path/EID but a different device took it over; treat as
                // a fresh device rather than re-attaching the old one.
                lg2::warning(
                    "MCTP endpoint {PATH} reappeared with different UUID; rescanning",
                    "PATH", mctpObjectPath);
                smaDevices.erase(it);
                scheduleRescan();
                return;
            }

            applyEvent(mctpObjectPath, EndpointEvent::EndpointReadded);
        });
}

void DeviceManager::applyEvent(const sdbusplus::object_path& mctpObjectPath,
                               EndpointEvent event)
{
    auto it = std::ranges::find(smaDevices, mctpObjectPath,
                                &SmaDeviceRecord::mctpObjectPath);
    if (it == smaDevices.end())
    {
        return;
    }

    SmaDeviceRecord& rec = *it;
    const auto [next, action] = nextState(rec.state, event);
    rec.state = next;

    switch (action)
    {
        case EndpointAction::GoOffline:
            lg2::info("MCTP endpoint {PATH} (eid {EID}) went offline", "PATH",
                      mctpObjectPath, "EID", rec.eid);
            rec.device->setOffline();
            break;
        case EndpointAction::GoOnline:
            lg2::info("MCTP endpoint {PATH} (eid {EID}) came online", "PATH",
                      mctpObjectPath, "EID", rec.eid);
            rec.device->setOnline();
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

    const sdbusplus::object_path mctpObjectPath{msg.get_path()};

    if (*value == "Degraded")
    {
        applyEvent(mctpObjectPath, EndpointEvent::ConnectivityDegraded);
    }
    else if (*value == "Available")
    {
        applyEvent(mctpObjectPath, EndpointEvent::ConnectivityAvailable);
    }
}
