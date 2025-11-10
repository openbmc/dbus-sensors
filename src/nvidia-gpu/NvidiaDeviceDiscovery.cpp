/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaDeviceDiscovery.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr auto sensorPollRateMs = 1000;

void processQueryDeviceIdResponse(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const SensorConfigs& configs,
    const std::string& path, uint8_t eid,
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

    try
    {
        switch (static_cast<gpu::DeviceIdentification>(responseDeviceType))
        {
            case gpu::DeviceIdentification::DEVICE_GPU:
            {
                lg2::info(
                    "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                    "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                    responseInstanceId);

                auto gpuName = configs.name + '_' +
                               std::to_string(responseInstanceId);

                auto gpu =
                    gpuDevices
                        .insert(std::make_pair(
                            gpuName, std::make_shared<GpuDevice>(
                                         configs, gpuName, path, conn, eid, io,
                                         mctpRequester, objectServer)))
                        .first;
                (*gpu).second->init();
                break;
            }

            case gpu::DeviceIdentification::DEVICE_SMA:
            {
                lg2::info(
                    "Found the SMA Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                    "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                    responseInstanceId);

                auto smaName = configs.name + "_SMA_" +
                               std::to_string(responseInstanceId);

                auto sma =
                    smaDevices
                        .insert(std::make_pair(
                            smaName, std::make_shared<SmaDevice>(
                                         configs, smaName, path, conn, eid, io,
                                         mctpRequester, objectServer)))
                        .first;
                (*sma).second->init();
                break;
            }
        }
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "Exception processing MCTP endpoint with eid {EID}, DeviceType {DEVTYPE}, "
            "and InstanceId {IID} : {ERROR}",
            "EID", eid, "DEVTYPE", responseDeviceType, "IID",
            responseInstanceId, "ERROR", e.what());
    }
}

void queryDeviceIdentification(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const SensorConfigs& configs,
    const std::string& path, uint8_t eid)
{
    auto queryDeviceIdentificationRequest = std::make_shared<
        std::array<uint8_t, sizeof(gpu::QueryDeviceIdentificationRequest)>>();

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
        [&io, &objectServer, &gpuDevices, &smaDevices, conn, &mctpRequester,
         configs, path, eid, queryDeviceIdentificationRequest](
            const std::error_code& ec, std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(io, objectServer, gpuDevices,
                                         smaDevices, conn, mctpRequester,
                                         configs, path, eid, ec, response);
        });
}

void processEndpoint(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const SensorConfigs& configs,
    const std::string& path, const boost::system::error_code& ec,
    const SensorBaseConfigMap& endpoint)
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
        queryDeviceIdentification(io, objectServer, gpuDevices, smaDevices,
                                  conn, mctpRequester, configs, path, eid);
    }
}

void queryEndpoints(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const SensorConfigs& configs,
    const std::string& path, const boost::system::error_code& ec,
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
                        [&io, &objectServer, &gpuDevices, &smaDevices, conn,
                         &mctpRequester, configs,
                         path](const boost::system::error_code& ec,
                               const SensorBaseConfigMap& endpoint) {
                            processEndpoint(io, objectServer, gpuDevices,
                                            smaDevices, conn, mctpRequester,
                                            configs, path, ec, endpoint);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void discoverDevices(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const SensorConfigs& configs,
    const std::string& path)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, conn, &mctpRequester,
         configs,
         path](const boost::system::error_code& ec, const GetSubTreeType& ret) {
            queryEndpoints(io, objectServer, gpuDevices, smaDevices, conn,
                           mctpRequester, configs, path, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void processSensorConfigs(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester, const ManagedObjectType& resp)
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

            discoverDevices(io, objectServer, gpuDevices, smaDevices,
                            dbusConnection, mctpRequester, configs, path);

            lg2::info(
                "Detected configuration {NAME} of type {TYPE} at path: {PATH}.",
                "NAME", configs.name, "TYPE", deviceType, "PATH", path);
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }
    dbusConnection->async_method_call(
        [&gpuDevices, &smaDevices, &mctpRequester, dbusConnection, &io,
         &objectServer](boost::system::error_code ec,
                        const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }

            processSensorConfigs(io, objectServer, gpuDevices, smaDevices,
                                 dbusConnection, mctpRequester, resp);
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices)
{
    if (message.is_method_error())
    {
        lg2::error("interfacesRemoved callback method error");
        return;
    }

    sdbusplus::message::object_path removedPath;
    std::vector<std::string> interfaces;

    message.read(removedPath, interfaces);

    // If the xyz.openbmc_project.Confguration.X interface was removed
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
}
