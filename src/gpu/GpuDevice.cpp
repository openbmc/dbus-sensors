/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuDevice.hpp"

#include "GpuSensor.hpp"
#include "GpuTLimitSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <GpuMctpVdm.hpp>
#include <MctpRequester.hpp>
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

void GpuDevice::createSensors()
{
    sensors.push_back(std::make_shared<GpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, objectServer,
        std::vector<thresholds::Threshold>{}));

    sensors.push_back(std::make_shared<GpuTLimitSensor>(
        conn, mctpRequester, name + "_TEMP_1", path, eid, objectServer,
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
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::QueryDeviceIdentificationRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeQueryDeviceIdentificationRequest(0, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "GpuDevice::processGpuEndPoint(): gpuEncodeQueryDeviceIdentificationRequest failed, rc={RC}",
            "RC", static_cast<int>(rc));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this, eid](int sendRecvMsgResult, std::vector<uint8_t> respMsg) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "GpuDevice::processGpuEndPoint(): MctpRequester::sendRecvMsg() failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (respMsg.empty())
            {
                lg2::error(
                    "GpuDevice::processGpuEndPoint(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
                return;
            }

            uint8_t cc = 0;
            uint16_t reasonCode = 0;
            uint8_t responseDeviceType = 0;
            uint8_t responseInstanceId = 0;

            auto rc = gpu::decodeQueryDeviceIdentificationResponse(
                *new (respMsg.data()) ocp::accelerator_management::Message,
                respMsg.size(), cc, reasonCode, responseDeviceType,
                responseInstanceId);

            if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
                cc != static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::SUCCESS))
            {
                lg2::error(
                    "GpuDevice::processGpuEndPoint(): gpuDecodeQueryDeviceIdentificationResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            if (responseDeviceType ==
                static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU))
            {
                lg2::info(
                    "GpuDevice::processGpuEndPoint(): found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                    "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                    responseInstanceId);

                this->eid = eid;
                this->createSensors();
                this->read();
            }
        });
}

void GpuDevice::processMctpEndpoints(const boost::system::error_code& ec,
                                     const getSubTreeRet& ret)
{
    if (ec)
    {
        lg2::error("GpuDevice::discoverGpus(): Error:{ERROR}", "ERROR",
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
                               const GpuSensorConfigMap& configs) {
                            this->processEndpointConfigs(ec, configs);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void GpuDevice::processEndpointConfigs(const boost::system::error_code& ec,
                                       const GpuSensorConfigMap& configs)
{
    if (ec)
    {
        lg2::error("GpuDevice::discoverGpus(): Error:{ERROR}", "ERROR",
                   ec.message());
        return;
    }

    uint8_t eid{};
    std::vector<uint8_t> mctpTypes{};

    auto hasEid = configs.find("EID");
    if (hasEid != configs.end())
    {
        const auto* eidPtr = std::get_if<uint8_t>(&hasEid->second);
        if (eidPtr != nullptr)
        {
            eid = *eidPtr;
        }
        else
        {
            lg2::error(
                "GpuDevice::discoverGpus(): Property EID does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "GpuDevice::discoverGpus(): Property EID not found in the configuration.");
        return;
    }

    auto hasMctpTypes = configs.find("SupportedMessageTypes");
    if (hasMctpTypes != configs.end())
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
                "GpuDevice::discoverGpus(): Property SupportedMessageTypes does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "GpuDevice::discoverGpus(): Property SupportedMessageTypes not found in the configuration.");
        return;
    }

    if (std::find(mctpTypes.begin(), mctpTypes.end(),
                  ocp::accelerator_management::messageType) != mctpTypes.end())
    {
        lg2::info(
            "GpuDevice::discoverGpus(): Found OCP MCTP VDM Endpoint with ID {EID}",
            "EID", eid);
        this->processGpuEndpoint(eid);
    }
}

void GpuDevice::discoverGpus()
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this](const boost::system::error_code& ec, const getSubTreeRet& ret) {
            processMctpEndpoints(ec, ret);
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
            if (intf != configInterfaceName(sensorType))
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
                       configInterfaceName(sensorType)) != interfaces.end()))
        {
            sensorIt = gpuDevice.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}
