/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace std::literals;

constexpr uint8_t gpuTempSensorId{0};
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

    discoverGpus();
}

GpuTempSensor::~GpuTempSensor()
{
    waitTimer.cancel();
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(association);
    objectServer.remove_interface(sensorInterface);
}

void GpuTempSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void GpuTempSensor::queryEndpoints(const boost::system::error_code& ec,
                                   const GetSubTreeType& ret)
{
    if (ec)
    {
        lg2::error("Error querying endoints :{ERROR}", "ERROR", ec.message());
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
                               const SensorBaseConfigMap& configs) {
                            this->processEndpoint(ec, configs);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void GpuTempSensor::read()
{
    update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weakPtrToThis = std::weak_ptr<GpuTempSensor>{shared_from_this()}](
            const boost::system::error_code& ec) {
            if (ec)
            {
                return;
            }
            if (auto ptr = weakPtrToThis.lock())
            {
                ptr->read();
            }
        });
}

void GpuTempSensor::processResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating Temperature Sensor: sending message over MCTP failed, rc={RC}",
            "RC", sendRecvMsgResult);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    double tempValue = 0;

    auto rc = gpu::decodeGetTemperatureReadingResponse(
        getTemperatureReadingResponse, cc, reasonCode, tempValue);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Temperature Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    updateValue(tempValue);
}

void GpuTempSensor::update()
{
    auto rc = gpu::encodeGetTemperatureReadingRequest(
        0, sensorId, getTemperatureReadingRequest);
    if (rc != 0)
    {
        lg2::error("Error updating Temperature Sensor: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, getTemperatureReadingRequest, getTemperatureReadingResponse,
        [this](int sendRecvMsgResult) { processResponse(sendRecvMsgResult); });
}

void GpuTempSensor::processQueryDeviceIdResponse(uint8_t eid,
                                                 int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error processing GPU endpoint: sending message over MCTP failed, rc={RC}",
            "RC", sendRecvMsgResult);
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
        setInitialProperties(sensor_paths::unitDegreesC);
        read();
    }
}

void GpuTempSensor::processGpuEndpoint(uint8_t eid)
{
    auto rc = gpu::encodeQueryDeviceIdentificationRequest(
        0, queryDeviceIdentificationRequest);
    if (rc != 0)
    {
        lg2::error("Error processing GPU endpoint: encode failed, rc={RC}",
                   "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, queryDeviceIdentificationRequest,
        queryDeviceIdentificationResponse, [this, eid](int sendRecvMsgResult) {
            processQueryDeviceIdResponse(eid, sendRecvMsgResult);
        });
}

void GpuTempSensor::processEndpoint(const boost::system::error_code& ec,
                                    const SensorBaseConfigMap& endpoint)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoint: {ERROR}", "ERROR",
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
        [this](const boost::system::error_code& ec, const GetSubTreeType& ret) {
            queryEndpoints(ec, ret);
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

            uint64_t pollRate = loadVariant<uint64_t>(cfg, "PollRate");

            sensors[name] = std::make_shared<GpuTempSensor>(
                dbusConnection, io, mctpRequester, name, path, objectServer,
                std::vector<thresholds::Threshold>{},
                std::chrono::milliseconds{pollRate});

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
        [&sensors, &mctpRequester, &dbusConnection, &io,
         &objectServer](const boost::system::error_code& ec,
                        const ManagedObjectType& resp) {
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
