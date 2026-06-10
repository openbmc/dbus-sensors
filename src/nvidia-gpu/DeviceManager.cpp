/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DeviceManager.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaSensorConfig.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr auto sensorPollRateMs = 1000;

// Debounce window to coalesce a burst of entity-manager config property
// changes (which normally fire several at once) into one rescan.
static constexpr std::chrono::seconds configSettleInterval{1};

DeviceManager::DeviceManager(boost::asio::io_context& io,
                             sdbusplus::asio::object_server& objectServer,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             mctp::MctpRequester& mctpRequester) :
    io(io), objectServer(objectServer), conn(std::move(conn)),
    mctpRequester(mctpRequester), configTimer(io)
{}

void DeviceManager::start()
{
    boost::asio::post(io, [this]() { createSensors(); });

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [this](sdbusplus::message_t&) {
            configTimer.expires_after(configSettleInterval);
            // create a timer because normally multiple properties change
            configTimer.async_wait([this](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                createSensors();
            });
        };

    std::array<std::string_view, 1> deviceTypes({deviceType});
    configMatches =
        setupPropertiesChangedMatches(*conn, deviceTypes, eventHandler);

    // Watch for entity-manager to remove configuration interfaces
    // so the corresponding sensors can be removed.
    configIfaceRemovedMatch = std::make_unique<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*conn),
        sdbusplus::bus::match::rules::interfacesRemovedAtPath(
            std::string(inventoryPath)),
        [this](sdbusplus::message_t& msg) { onConfigInterfaceRemoved(msg); });
}

void DeviceManager::processQueryDeviceIdResponse(
    const SensorConfigs& configs, const std::string& path, uint8_t eid,
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

            const std::string gpuName = std::format("Nvidia_GPU_{}", eid);

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

            const std::string smaName = std::format("Nvidia_SMA_{}", eid);

            std::shared_ptr<SmaDevice>& sma = smaDevices[smaName];

            if (sma == nullptr)
            {
                sma = std::make_shared<SmaDevice>(configs, smaName, path, conn,
                                                  eid, io, mctpRequester,
                                                  objectServer);

                sma->init();
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
    const SensorConfigs& configs, const std::string& path, uint8_t eid)
{
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
        [this, configs, path, eid, queryDeviceIdentificationRequest](
            const std::error_code& ec, std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(configs, path, eid, ec, response);
        });
}

void DeviceManager::processEndpoint(
    const SensorConfigs& configs, const std::string& path,
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
        queryDeviceIdentification(configs, path, eid);
    }
}

void DeviceManager::queryEndpoints(
    const SensorConfigs& configs, const std::string& path,
    const boost::system::error_code& ec, const GetSubTreeType& ret)
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
                        [this, configs,
                         path](const boost::system::error_code& ec,
                               const SensorBaseConfigMap& endpoint) {
                            processEndpoint(configs, path, ec, endpoint);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void DeviceManager::discoverDevices(const SensorConfigs& configs,
                                    const std::string& path)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this, configs,
         path](const boost::system::error_code& ec, const GetSubTreeType& ret) {
            queryEndpoints(configs, path, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void DeviceManager::processSensorConfigs(const ManagedObjectType& resp)
{
    for (const auto& [path, interfaces] : resp)
    {
        for (const auto& [intf, cfg] : interfaces)
        {
            if (intf != configInterfaceName(deviceType))
            {
                continue;
            }

            SensorConfigs configs;

            configs.name = loadVariant<std::string>(cfg, "Name");

            try
            {
                configs.pollRate = loadVariant<uint64_t>(cfg, "PollRate");
            }
            catch (const std::invalid_argument&)
            {
                // PollRate is an optional config
                configs.pollRate = sensorPollRateMs;
            }

            try
            {
                configs.nicNetworkPortCount =
                    loadVariant<uint64_t>(cfg, "NicNetworkPortCount");
            }
            catch (const std::invalid_argument&)
            {
                // NicNetworkPortCount is an optional config
                configs.nicNetworkPortCount = 0;
            }

            discoverDevices(configs, path);

            lg2::info(
                "Detected configuration {NAME} of type {TYPE} at path: {PATH}.",
                "NAME", configs.name, "TYPE", deviceType, "PATH", path);
        }
    }
}

void DeviceManager::createSensors()
{
    if (!conn)
    {
        lg2::error("Connection not created");
        return;
    }
    conn->async_method_call(
        [this](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }

            processSensorConfigs(resp);
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
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
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(deviceType)) != interfaces.end()))
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
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(deviceType)) != interfaces.end()))
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
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(deviceType)) != interfaces.end()))
        {
            pcieSensorIt = pcieDevices.erase(pcieSensorIt);
        }
        else
        {
            pcieSensorIt++;
        }
    }
}
