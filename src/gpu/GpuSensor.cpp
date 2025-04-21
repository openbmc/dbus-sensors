/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace std::literals;

constexpr uint8_t gpuTempSensorId{0};
constexpr std::chrono::milliseconds samplingInterval{1000ms};
static constexpr double gpuTempSensorMaxReading = 127;
static constexpr double gpuTempSensorMinReading = -128;

GpuTempSensor::GpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
    const std::string& name, const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    std::chrono::milliseconds pollRate) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "temperature", false, true, gpuTempSensorMaxReading,
           gpuTempSensorMinReading, conn),
    sensorId{gpuTempSensorId}, sensorPollMs(pollRate),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer)
{
    std::string dbusPath =
        sensorPathPrefix + "temperature/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    init();
}

GpuTempSensor::~GpuTempSensor()
{
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void GpuTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void GpuTempSensor::init()
{
    discoverGpus();
}

void GpuTempSensor::read()
{
    update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}

void GpuTempSensor::update()
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::GetTemperatureReadingRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeGetTemperatureReadingRequest(0, sensorId, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "GpuTempSensor::update(): gpuEncodeGetTemperatureReadingRequest failed, rc={RC}",
            "RC", static_cast<int>(rc));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this](int sendRecvMsgResult, std::vector<uint8_t> respMsg) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "GpuTempSensor::update(): MctpRequester::sendRecvMsg() failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (respMsg.empty())
            {
                lg2::error(
                    "GpuTempSensor::update(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
                return;
            }

            uint8_t cc = 0;
            uint16_t reasonCode = 0;
            double tempValue = 0;

            auto rc = gpu::decodeGetTemperatureReadingResponse(
                *new (respMsg.data()) ocp::accelerator_management::Message,
                respMsg.size(), cc, reasonCode, tempValue);

            if (rc != ocp::accelerator_management::CompletionCode::SUCCESS ||
                cc != static_cast<uint8_t>(
                          ocp::accelerator_management::CompletionCode::SUCCESS))
            {
                lg2::error(
                    "GpuTempSensor::update(): gpuDecodeGetTemperatureReadingResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            updateValue(tempValue);
        });
}

void GpuTempSensor::processGpuEndpoint(uint8_t eid)
{
    std::vector<uint8_t> reqMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(gpu::QueryDeviceIdentificationRequest));

    auto* msg = new (reqMsg.data()) ocp::accelerator_management::Message;

    auto rc = gpu::encodeQueryDeviceIdentificationRequest(0, *msg);
    if (rc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "GpuTempSensor::processGpuEndPoint(): gpuEncodeQueryDeviceIdentificationRequest failed, rc={RC}",
            "RC", static_cast<int>(rc));
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, reqMsg,
        [this, eid](int sendRecvMsgResult, std::vector<uint8_t> respMsg) {
            if (sendRecvMsgResult != 0)
            {
                lg2::error(
                    "GpuTempSensor::processGpuEndPoint(): MctpRequester::sendRecvMsg() failed, rc={RC}",
                    "RC", sendRecvMsgResult);
                return;
            }

            if (respMsg.empty())
            {
                lg2::error(
                    "GpuTempSensor::processGpuEndPoint(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
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
                    "GpuTempSensor::processGpuEndPoint(): gpuDecodeQueryDeviceIdentificationResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            if (responseDeviceType ==
                static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU))
            {
                lg2::info(
                    "GpuTempSensor::processGpuEndPoint(): found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                    "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                    responseInstanceId);

                this->eid = eid;
                setInitialProperties(sensor_paths::unitDegreesC);
                this->read();
            }
        });
}

void GpuTempSensor::processMctpEndpoints(const boost::system::error_code& ec,
                                         const getSubTreeRet& ret)
{
    if (ec)
    {
        lg2::error("GpuTempSensor::discoverGpus(): Error:{ERROR}", "ERROR",
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

void GpuTempSensor::processEndpointConfigs(const boost::system::error_code& ec,
                                           const GpuSensorConfigMap& configs)
{
    if (ec)
    {
        lg2::error("GpuTempSensor::discoverGpus(): Error:{ERROR}", "ERROR",
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
                "GpuTempSensor::discoverGpus(): Property EID does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "GpuTempSensor::discoverGpus(): Property EID not found in the configuration.");
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
                "GpuTempSensor::discoverGpus(): Property SupportedMessageTypes does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "GpuTempSensor::discoverGpus(): Property SupportedMessageTypes not found in the configuration.");
        return;
    }

    if (std::find(mctpTypes.begin(), mctpTypes.end(),
                  ocp::accelerator_management::messageType) != mctpTypes.end())
    {
        lg2::info(
            "GpuTempSensor::discoverGpus(): Found OCP MCTP VDM Endpoint with ID {EID}",
            "EID", eid);
        this->processGpuEndpoint(eid);
    }
}

void GpuTempSensor::discoverGpus()
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
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors,
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

            sensors[name] = std::make_shared<GpuTempSensor>(
                dbusConnection, io, mctpRequester, name, path, objectServer,
                std::vector<thresholds::Threshold>{}, samplingInterval);

            lg2::info(
                "Added GPU Temperature Sensor {NAME} with chassis path: {PATH}.",
                "NAME", name, "PATH", path);
        }
    }
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }
    dbusConnection->async_method_call(
        [&sensors, &mctpRequester, &dbusConnection, &io, &objectServer](
            boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }

            processSensorConfigs(io, objectServer, sensors, dbusConnection,
                                 mctpRequester, resp);
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<GpuTempSensor>>&
        sensors)
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
    auto sensorIt = sensors.begin();
    while (sensorIt != sensors.end())
    {
        if ((sensorIt->second->configurationPath == removedPath) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       configInterfaceName(sensorType)) != interfaces.end()))
        {
            sensorIt = sensors.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}
