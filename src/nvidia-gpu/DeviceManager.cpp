/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSmaDevice.hpp"
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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

static constexpr unsigned maxDiscoveryRetries = 3;

DeviceManager::DeviceManager(boost::asio::io_context& io,
                             sdbusplus::asio::object_server& objectServer,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             mctp::MctpRequester& mctpRequester) :
    io(io), objectServer(objectServer), conn(std::move(conn)),
    mctpRequester(mctpRequester), configTimer(io)
{}

// A configuration usually arrives as several properties changing together,
// so wait for them to settle rather than sweeping once per property.
static constexpr std::chrono::seconds configSettleInterval{1};

static constexpr std::string_view probeIface =
    "xyz.openbmc_project.Configuration.Probe";

// Entity manager names an object after its configuration's Name, with every
// character other than a letter or a digit replaced.
static std::string entityPathSegment(std::string_view name)
{
    std::string segment(name);
    std::ranges::replace_if(
        segment,
        [](char c) {
            return (c < '0' || c > '9') && (c < 'A' || c > 'Z') &&
                   (c < 'a' || c > 'z');
        },
        '_');
    return segment;
}

void DeviceManager::scheduleRescan()
{
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
// The settings an NvidiaMctpVdm record carries for any kind of device.
static NvidiaMctpVdmConfig readNvidiaMctpVdmConfig(const MctpVdmRecord& record)
{
    NvidiaMctpVdmConfig config{.path = record.path};

    try
    {
        config.pollRate = loadVariant<uint64_t>(record.properties, "PollRate");
        if (config.pollRate == 0)
        {
            config.pollRate = sensorPollRate.count();
        }
    }
    catch (const std::invalid_argument&)
    {
        // PollRate is an optional config
        config.pollRate = sensorPollRate.count();
    }

    return config;
}

// Only a ConnectX device has network ports, so the record is only asked for
// a count of them once the device has turned out to be one.
static PcieDeviceConfigs readPcieDeviceConfigs(const MctpVdmRecord& record)
{
    PcieDeviceConfigs pcieConfig;

    try
    {
        pcieConfig.networkPortCount =
            loadVariant<uint64_t>(record.properties, "NicNetworkPortCount");
    }
    catch (const std::invalid_argument&)
    {
        // NicNetworkPortCount is an optional config
        pcieConfig.networkPortCount = 0;
    }

    return pcieConfig;
}

void DeviceManager::processQueryDeviceIdResponse(
    const MctpVdmRecord& record, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid, const std::error_code& sendRecvMsgResult,
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

    const NvidiaMctpVdmConfig config = readNvidiaMctpVdmConfig(record);

    switch (static_cast<gpu::DeviceIdentification>(responseDeviceType))
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
        {
            lg2::info(
                "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            const std::string gpuName = std::format("Nvidia_GPU_{}", eid);

            std::shared_ptr<GpuDevice>& gpu = gpuDevices[gpuName];

            if (gpu == nullptr)
            {
                gpu =
                    std::make_shared<GpuDevice>(config, gpuName, conn, eid, io,
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

            const std::string smaName = std::format("Nvidia_SMA_{}", eid);

            auto existing =
                std::ranges::find(smaDevices, smaName, &SmaDeviceRecord::name);

            if (existing == smaDevices.end())
            {
                auto sma =
                    std::make_shared<SmaDevice>(config, smaName, conn, eid, io,
                                                mctpRequester, objectServer);

                sma->init();

                smaDevices.emplace_back(SmaDeviceRecord{
                    .device = std::move(sma),
                    .name = smaName,
                    .mctpObjectPath = mctpObjectPath,
                    .uuid = {},
                    .eid = eid,
                    .state = EndpointState::Init});

                fetchEndpointUuid(mctpObjectPath);
                applyEvent(mctpObjectPath, EndpointEvent::InitComplete);
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

            const std::string pcieName = std::format("Nvidia_ConnectX_{}", eid);

            std::shared_ptr<PcieDevice>& pcie = pcieDevices[pcieName];

            if (pcie == nullptr)
            {
                pcie = std::make_shared<PcieDevice>(
                    config, readPcieDeviceConfigs(record), pcieName, conn, eid,
                    io, mctpRequester, objectServer);

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
    const MctpVdmRecord& record, const sdbusplus::object_path& mctpObjectPath,
    uint8_t eid)
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
        [this, record, mctpObjectPath, eid, queryDeviceIdentificationRequest](
            const std::error_code& ec, std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(record, mctpObjectPath, eid, ec,
                                         response);
        });
}

void DeviceManager::checkAssociationAndQueryDevice(
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid)
{
    const sdbusplus::object_path associationPath =
        mctpObjectPath / "configured_by";

    conn->async_method_call(
        [this, mctpObjectPath, eid, associationPath](
            const boost::system::error_code& ec, const GetSubTreeType& ret) {
            if (ec)
            {
                lg2::error("EID {EID}: Failed to look up {PATH}: {ERROR}",
                           "EID", eid, "PATH", associationPath, "ERROR",
                           ec.message());
                retryDiscovery(mctpObjectPath, eid);
                return;
            }

            // The mapper is listed as an owner too, with no interfaces.
            for (const auto& [configPath, services] : ret)
            {
                const auto owner = std::ranges::find_if(
                    services, [](const auto& s) { return !s.second.empty(); });
                if (owner != services.end())
                {
                    getConfigProperties(mctpObjectPath, eid, configPath,
                                        owner->first);
                    return;
                }
            }

            // Devices behind a bridge have no association of their own.
            lg2::debug(
                "EID {EID}: No association found at {PATH}, skipping endpoint",
                "EID", eid, "PATH", associationPath);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetAssociatedSubTree",
        associationPath, sdbusplus::object_path("/"), 0,
        std::vector<std::string>{});
}

void DeviceManager::getConfigProperties(
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
    const sdbusplus::object_path& configPath, const std::string& configService)
{
    conn->async_method_call(
        [this, mctpObjectPath, eid,
         configPath](const boost::system::error_code& ec,
                     const SensorBaseConfigMap& configProps) {
            processConfigPropertiesResult(mctpObjectPath, eid, configPath, ec,
                                          configProps);
        },
        configService, configPath, "org.freedesktop.DBus.Properties", "GetAll",
        "");
}

void DeviceManager::processConfigPropertiesResult(
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
    const sdbusplus::object_path& configPath,
    const boost::system::error_code& ec, const SensorBaseConfigMap& configProps)
{
    if (ec)
    {
        lg2::error("EID {EID}: Failed to get config properties: {ERROR}", "EID",
                   eid, "ERROR", ec.message());
        retryDiscovery(mctpObjectPath, eid);
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

    // The object the endpoint was configured from says which board the device
    // is on, not how the device is read. Its own record is the board's to
    // answer for and is not read here, so the device is created from a record
    // with nothing in it, which reads as the defaults, until it is.
    const MctpVdmRecord record{.path = configPath, .properties = {}};

    auto boardIt = configProps.find("Board");
    if (boardIt != configProps.end())
    {
        const auto* boardPtr = std::get_if<std::string>(&boardIt->second);
        if ((boardPtr != nullptr) && !boardPtr->empty())
        {
            findBoardInventoryPath(
                *boardPtr, configPath, eid,
                [this, mctpObjectPath,
                 eid](const sdbusplus::object_path& entityObjectPath) {
                    queryDeviceIdentification(
                        MctpVdmRecord{.path = entityObjectPath,
                                      .properties = {}},
                        mctpObjectPath, eid);
                });
            return;
        }
    }

    lg2::info("EID {EID}: No Board property found, using config path {PATH}",
              "EID", eid, "PATH", configPath);
    queryDeviceIdentification(record, mctpObjectPath, eid);
}

// Pick the NvidiaMctpVdm configuration found under the board, falling back to
// fallbackPath when the search turned nothing up.
static void selectMctpVdmConfig(
    const sdbusplus::object_path& inventoryPath,
    const sdbusplus::object_path& fallbackPath, uint8_t eid,
    const boost::system::error_code& ec, const GetSubTreeType& ret,
    const std::function<void(const std::string&)>& done)
{
    std::string finalConfigPath = fallbackPath;

    if (!ec && !ret.empty())
    {
        const std::string& objPath = ret[0].first;
        if (objPath.contains(inventoryPath.string()))
        {
            finalConfigPath = objPath;
        }
    }
    else
    {
        lg2::error(
            "EID {EID}: NvidiaMctpVdm config not found under board, using original {PATH}",
            "EID", eid, "PATH", fallbackPath);
    }

    done(finalConfigPath);
}

void DeviceManager::lookupBoard(const std::string& boardName,
                                BoardPathHandler done)
{
    auto board = boardPaths.find(boardName);
    if (board != boardPaths.end())
    {
        done(board->second);
        return;
    }

    auto [waiters, first] = boardLookups.try_emplace(boardName);
    waiters->second.push_back(std::move(done));
    if (!first)
    {
        return;
    }

    conn->async_method_call(
        [this, boardName](const boost::system::error_code& ec,
                          const GetSubTreeType& ret) {
            std::optional<sdbusplus::object_path> path;
            const std::string segment = "/" + entityPathSegment(boardName);
            const auto found = std::ranges::find_if(ret, [&](const auto& obj) {
                return obj.first.ends_with(segment);
            });
            if (!ec && found != ret.end())
            {
                path.emplace(found->first);
                boardPaths.emplace(boardName, *path);
            }

            auto waiting = boardLookups.extract(boardName);
            for (const BoardPathHandler& waiter : waiting.mapped())
            {
                waiter(path);
            }
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree",
        "/xyz/openbmc_project/inventory", 0,
        std::vector<std::string>{std::string(probeIface)});
}

void DeviceManager::findBoardInventoryPath(
    const std::string& boardName, const sdbusplus::object_path& fallbackPath,
    uint8_t eid, const ConfigPathHandler& done)
{
    lookupBoard(boardName, [this, boardName, fallbackPath, eid,
                            done](const std::optional<sdbusplus::object_path>&
                                      path) {
        if (!path)
        {
            lg2::error(
                "EID {EID}: Board {BOARD} not found in inventory, using config path {PATH}",
                "EID", eid, "BOARD", boardName, "PATH", fallbackPath);
            done(fallbackPath);
            return;
        }
        findBoardConfig(*path, fallbackPath, eid, done);
    });
}

void DeviceManager::findBoardConfig(const sdbusplus::object_path& inventoryPath,
                                    const sdbusplus::object_path& fallbackPath,
                                    uint8_t eid, const ConfigPathHandler& done)
{
    lg2::info("EID {EID}: Found board inventory path {PATH}", "EID", eid,
              "PATH", inventoryPath);

    conn->async_method_call(
        [eid, inventoryPath, fallbackPath, done](
            const boost::system::error_code& ec2, const GetSubTreeType& ret2) {
            selectMctpVdmConfig(inventoryPath, fallbackPath, eid, ec2, ret2,
                                done);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree",
        inventoryPath.string(), 0,
        std::vector<std::string>{
            "xyz.openbmc_project.Configuration.NvidiaMctpVdm"});
}

void DeviceManager::processEndpoint(
    const sdbusplus::object_path& mctpObjectPath,
    const boost::system::error_code& ec, const SensorBaseConfigMap& endpoint)
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
        checkAssociationAndQueryDevice(mctpObjectPath, eid);
    }
}

void DeviceManager::queryEndpoints(const boost::system::error_code& ec,
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
                    conn->async_method_call(
                        [this, mctpObjectPath{objPath}](
                            const boost::system::error_code& ec,
                            const SensorBaseConfigMap& endpoint) {
                            processEndpoint(mctpObjectPath, ec, endpoint);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void DeviceManager::discoverDevices()
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this](const boost::system::error_code& ec, const GetSubTreeType& ret) {
            queryEndpoints(ec, ret);
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

    boardPaths.clear();
    discoverDevices();
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

    if (std::ranges::find(removedInterfaces,
                          "au.com.codeconstruct.MCTP.Endpoint1") ==
        removedInterfaces.end())
    {
        return;
    }

    // applyEvent is a no-op if we do not track this endpoint.
    applyEvent(objPath, EndpointEvent::EndpointRemoved);
}

namespace
{

// Every other property of the endpoint has a type of its own, and is skipped
// rather than failing the read.
using EndpointProperties =
    boost::container::flat_map<std::string, std::variant<std::string, uint8_t>>;
using EndpointInterfaces =
    boost::container::flat_map<std::string, EndpointProperties>;

template <typename T>
const T* findEndpointProperty(const EndpointInterfaces& interfaces,
                              const std::string& interface,
                              const std::string& property)
{
    auto iface = interfaces.find(interface);
    if (iface == interfaces.end())
    {
        return nullptr;
    }
    auto prop = iface->second.find(property);
    if (prop == iface->second.end())
    {
        return nullptr;
    }
    return std::get_if<T>(&prop->second);
}

// What mctpd reports for a device that does not give a UUID of its own.
constexpr std::string_view nilUuid = "00000000-0000-0000-0000-000000000000";

} // namespace

void DeviceManager::onEndpointAdded(sdbusplus::message_t& msg)
{
    sdbusplus::object_path objPath;
    msg.read(objPath);

    // mctpd also adds networks and interfaces under the same namespace.
    if (!objPath.str.contains("/endpoints/"))
    {
        return;
    }

    EndpointInterfaces interfaces;
    msg.read(interfaces);

    const auto* uuid = findEndpointProperty<std::string>(
        interfaces, "xyz.openbmc_project.Common.UUID", "UUID");
    const auto* eid = findEndpointProperty<uint8_t>(
        interfaces, "xyz.openbmc_project.MCTP.Endpoint", "EID");
    const bool identified = uuid != nullptr && !uuid->empty() &&
                            *uuid != nilUuid && eid != nullptr;

    // The EID was given up before mctpd handed it out again, so a device known
    // at this path is gone unless it is the one that came back.
    std::erase_if(smaDevices, [&](const SmaDeviceRecord& rec) {
        return rec.mctpObjectPath == objPath &&
               (!identified || rec.uuid != *uuid);
    });

    if (!identified)
    {
        scheduleRescan();
        return;
    }

    auto it = std::ranges::find(smaDevices, *uuid, &SmaDeviceRecord::uuid);
    std::optional<EndpointState> known;
    if (it != smaDevices.end())
    {
        known = it->state;
    }

    switch (matchAddedEndpoint(known, known && it->mctpObjectPath == objPath))
    {
        case EndpointMatch::Rescan:
            scheduleRescan();
            break;
        case EndpointMatch::Ignore:
            lg2::warning(
                "MCTP endpoint {PATH} has the UUID of a device still reached through {KNOWN}",
                "PATH", objPath, "KNOWN", it->mctpObjectPath);
            break;
        case EndpointMatch::Rebind:
            lg2::info("MCTP endpoint {KNOWN} moved to {PATH}", "KNOWN",
                      it->mctpObjectPath, "PATH", objPath);
            it->device->setEid(*eid);
            it->mctpObjectPath = objPath;
            it->eid = *eid;
            applyEvent(objPath, EndpointEvent::EndpointReadded);
            break;
        case EndpointMatch::Readd:
            applyEvent(objPath, EndpointEvent::EndpointReadded);
            break;
    }
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

// mctpd reports an endpoint as Degraded once it stops answering, and as
// Available when it is reachable again.
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

    const sdbusplus::object_path mctpObjectPath{msg.get_path()};

    const auto* value = std::get_if<std::string>(&it->second);
    if (value == nullptr)
    {
        lg2::error(
            "MCTP endpoint {PATH} reported a Connectivity that is not a string",
            "PATH", mctpObjectPath);
        return;
    }

    if (*value == "Degraded")
    {
        applyEvent(mctpObjectPath, EndpointEvent::ConnectivityDegraded);
    }
    else if (*value == "Available")
    {
        applyEvent(mctpObjectPath, EndpointEvent::ConnectivityAvailable);
    }
    else
    {
        lg2::error(
            "MCTP endpoint {PATH} reported an unknown Connectivity {VALUE}",
            "PATH", mctpObjectPath, "VALUE", *value);
    }
}
