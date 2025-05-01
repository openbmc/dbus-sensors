/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuDevice.hpp"

#include "GpuSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "mctp/Requester.hpp"

#include <bits/basic_string.h>

#include <GpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds samplingInterval{1000ms};

std::unique_ptr<GpuDevice> gpuDevice;

GpuDevice::GpuDevice(const std::string& name, const std::string& path,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    sensorPollMs(samplingInterval),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    name(escapeName(name)), path(path)
{
    discoverGpus();
}

void GpuDevice::makeSensors()
{
    sensors.push_back(std::make_shared<GpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, objectServer,
        std::vector<thresholds::Threshold>{}));

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);
}

void GpuDevice::read()
{
    for (const auto& sensor : sensors)
    {
        sensor->update();
    }

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}

void GpuDevice::processGpuEndpoint(uint8_t eid)
{
    std::vector<uint8_t> reqMsg;

    auto rc = gpu::encodeQueryDeviceIdentificationRequest(0, reqMsg);
    if (rc != 0)
    {
        lg2::error("Error processing GPU endpoint: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this,
         eid](int sendRecvMsgResult, std::optional<std::vector<uint8_t>> resp) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "Error processing GPU endpoint: sending message over MCTP failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (!resp.has_value())
            {
                lg2::error("Error updating Temperature Sensor: empty response");
                return;
            }

            const auto& respMsg = resp.value();

            if (respMsg.empty())
            {
                lg2::error("Error processing GPU endpoint: empty response");
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            uint8_t responseDeviceType = 0;
            uint8_t responseInstanceId = 0;

            auto rc = gpu::decodeQueryDeviceIdentificationResponse(
                respMsg, cc, reasonCode, responseDeviceType,
                responseInstanceId);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error processing GPU endpoint: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "RC", rc, "CC", cc, "RESC", reasonCode);
                return;
            }

            if (responseDeviceType ==
                static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU))
            {
                lg2::info(
                    "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                    "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                    responseInstanceId);

                this->eid = eid;
                this->makeSensors();
                this->read();
            }
        });
}

void GpuDevice::queryEndpoints(const boost::system::error_code& ec,
                               const getSubTreeRet& ret)
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
                        [this](const boost::system::error_code& ec,
                               const GpuDeviceConfigMap& configs) {
                            this->processEndpoint(ec, configs);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void GpuDevice::processEndpoint(const boost::system::error_code& ec,
                                const GpuDeviceConfigMap& endpoint)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoint: Error:{ERROR}", "ERROR",
                   ec.message());
        return;
    }

    uint8_t eid{};
    std::vector<uint8_t> mctpTypes{};

    auto hasEid = endpoint.find("EID");
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
                "Error processing MCTP endpoint: Property SupportedMessageTypes does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "Error processing MCTP endpoint: Property SupportedMessageTypes not found in the configuration.");
        return;
    }

    if (std::find(mctpTypes.begin(), mctpTypes.end(),
                  ocp::accelerator_management::messageType) != mctpTypes.end())
    {
        lg2::info("Found OCP MCTP VDM Endpoint with ID {EID}", "EID", eid);
        this->processGpuEndpoint(eid);
    }
}

void GpuDevice::discoverGpus()
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this](const boost::system::error_code& ec, const getSubTreeRet& ret) {
            queryEndpoints(ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void processSensorConfigs(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
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

            std::string name = loadVariant<std::string>(cfg, "Name");

            gpuDevice[name] = std::make_shared<GpuDevice>(
                name, path, dbusConnection, io, mctpRequester, objectServer);
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }
    dbusConnection->async_method_call(
        [&gpuDevice, &mctpRequester, &dbusConnection, &io, &objectServer](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }

            processSensorConfigs(io, objectServer, gpuDevice, dbusConnection,
                                 mctpRequester, resp);
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevice)
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
    auto sensorIt = gpuDevice.begin();
    while (sensorIt != gpuDevice.end())
    {
        if ((sensorIt->second->getPath() == removedPath) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(deviceType)) != interfaces.end()))
        {
            sensorIt = gpuDevice.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}
