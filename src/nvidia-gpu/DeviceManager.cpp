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
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

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
void DeviceManager::processQueryDeviceIdResponse(
    const SensorConfigs& configs,
    const sdbusplus::object_path& entityObjectPath,
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid,
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
                gpu = std::make_shared<GpuDevice>(
                    configs, gpuName, entityObjectPath, conn, eid, io,
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
                auto sma = std::make_shared<SmaDevice>(
                    configs, smaName, entityObjectPath, conn, eid, io,
                    mctpRequester, objectServer);

                sma->init();

                smaDevices.emplace_back(SmaDeviceRecord{
                    .device = std::move(sma),
                    .name = smaName,
                    .mctpObjectPath = mctpObjectPath,
                    .eid = eid,
                    .state = EndpointState::Init});

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
                    configs, pcieName, entityObjectPath, conn, eid, io,
                    mctpRequester, objectServer);

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
    const SensorConfigs& configs,
    const sdbusplus::object_path& entityObjectPath,
    const sdbusplus::object_path& mctpObjectPath, uint8_t eid)
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
        [this, configs, entityObjectPath, mctpObjectPath, eid,
         queryDeviceIdentificationRequest](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(configs, entityObjectPath,
                                         mctpObjectPath, eid, ec, response);
        });
}

void DeviceManager::processEndpoint(
    const SensorConfigs& configs,
    const sdbusplus::object_path& entityObjectPath,
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
        queryDeviceIdentification(configs, entityObjectPath, mctpObjectPath,
                                  eid);
    }
}

void DeviceManager::queryEndpoints(
    const SensorConfigs& configs,
    const sdbusplus::object_path& entityObjectPath,
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
                        [this, configs, entityObjectPath,
                         mctpObjectPath{objPath}](
                            const boost::system::error_code& ec,
                            const SensorBaseConfigMap& endpoint) {
                            processEndpoint(configs, entityObjectPath,
                                            mctpObjectPath, ec, endpoint);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void DeviceManager::discoverDevices(
    const SensorConfigs& configs,
    const sdbusplus::object_path& entityObjectPath)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this, configs, entityObjectPath](const boost::system::error_code& ec,
                                          const GetSubTreeType& ret) {
            queryEndpoints(configs, entityObjectPath, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void DeviceManager::processSensorConfigs(const ManagedObjectType& resp)
{
    for (const auto& [entityObjectPath, interfaces] : resp)
    {
        for (const auto& [intf, cfg] : interfaces)
        {
            if (intf != configInterfaceName(sensorType))
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

            discoverDevices(configs, entityObjectPath);

            lg2::info(
                "Detected configuration {NAME} of type {TYPE} at path: {PATH}.",
                "NAME", configs.name, "TYPE", sensorType, "PATH",
                entityObjectPath);
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
                       configInterfaceName(sensorType)) != interfaces.end()))
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
               (std::find(interfaces.begin(), interfaces.end(),
                          configInterfaceName(sensorType)) != interfaces.end());
    });

    auto pcieSensorIt = pcieDevices.begin();
    while (pcieSensorIt != pcieDevices.end())
    {
        if ((pcieSensorIt->second->getPath() == removedPath) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(sensorType)) != interfaces.end()))
        {
            pcieSensorIt = pcieDevices.erase(pcieSensorIt);
        }
        else
        {
            pcieSensorIt++;
        }
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
