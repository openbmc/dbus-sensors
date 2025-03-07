/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuTempSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <GpuSensor.hpp>
#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

using namespace std::literals;

static constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";
static constexpr const char* sensorType = "NvidiaMctpVdm";

constexpr uint8_t gpuTempSensorId{0};
constexpr std::chrono::milliseconds samplingInterval{1000ms};
static constexpr double gpuTempSensorMaxReading =
    std::numeric_limits<float>::max();
static constexpr double gpuTempSensorMinReading =
    std::numeric_limits<float>::min();

using getSubTreeRet = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using GpuSensorConfigMap =
    std::map<std::string, std::variant<std::string, bool, uint32_t, uint8_t,
                                       int64_t, std::vector<uint8_t>>>;

GpuTempSensor::GpuTempSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, mctp::MctpRequester& mctpRequester,
    const std::string& name, const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    std::chrono::milliseconds pollRate, const DeviceInfo& deviceInfo,
    bool verbose) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "temperature", false, true, gpuTempSensorMinReading,
           gpuTempSensorMaxReading, conn),
    sensorId{gpuTempSensorId}, sensorPollMs(pollRate), verbose(verbose),
    deviceInfo(deviceInfo),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer)
{
    std::string dbusPath =
        sensorPathPrefix + "temperature"s + "/" + escapeName(name);

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
    if (verbose)
    {
        lg2::info(
            "GpuTempSensor::init(): starting initialization for eid={EID}, instanceId={IID}",
            "EID", eid, "IID", deviceInfo.instanceId);
    }

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
    if (verbose)
    {
        lg2::info("GpuTempSensor::update(): eid={EID} sensorId={SID}", "EID",
                  eid, "SID", sensorId);
    }

    std::vector<uint8_t> reqMsg(
        sizeof(OcpAmiBindingPciVid) + sizeof(GpuGetTemperatureReadingRequest));

    auto* msg = new (reqMsg.data()) OcpAmiMessage;

    auto rc =
        gpuEncodeGetTemperatureReadingRequest(ocpAmiInstanceMin, sensorId, msg);
    if (rc != OCP_AMI_SUCCESS)
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

            auto rc = gpuDecodeGetTemperatureReadingResponse(
                new (respMsg.data()) OcpAmiMessage, respMsg.size(), &cc,
                &reasonCode, &tempValue);

            if (rc != OCP_AMI_SUCCESS || cc != OCP_AMI_SUCCESS)
            {
                lg2::error(
                    "GpuTempSensor::update(): gpuDecodeGetTemperatureReadingResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            updateValue(tempValue);

            if (verbose)
            {
                lg2::info(
                    "GpuTempSensor::update(): eid={EID} sensorId={SID} temp={TEMP}",
                    "EID", eid, "SID", sensorId, "TEMP", tempValue);
            }
        });
}

void GpuTempSensor::processGpuEndpoint(uint8_t eid)
{
    if (verbose)
    {
        lg2::info("GpuTempSensor::processGpuEndPoint(): eid={EID}", "EID", eid);
    }

    std::vector<uint8_t> reqMsg(sizeof(OcpAmiBindingPciVid) +
                                sizeof(GpuQueryDeviceIdentificationRequest));

    auto* msg = new (reqMsg.data()) OcpAmiMessage;

    auto rc = gpuEncodeQueryDeviceIdentificationRequest(ocpAmiInstanceMin, msg);
    if (rc != OCP_AMI_SUCCESS)
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

            auto rc = gpuDecodeQueryDeviceIdentificationResponse(
                new (respMsg.data()) OcpAmiMessage, respMsg.size(), &cc,
                &reasonCode, &responseDeviceType, &responseInstanceId);

            if (rc != OCP_AMI_SUCCESS || cc != OCP_AMI_SUCCESS)
            {
                lg2::error(
                    "GpuTempSensor::processGpuEndPoint(): gpuDecodeQueryDeviceIdentificationResponse() failed, rc={RC} cc={CC} reasonCode={RESC}",
                    "RC", static_cast<int>(rc), "CC", cc, "RESC", reasonCode);
                return;
            }

            if (responseDeviceType == deviceInfo.deviceType &&
                responseInstanceId == deviceInfo.instanceId)
            {
                if (verbose)
                {
                    lg2::info(
                        "GpuTempSensor::processGpuEndPoint(): found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                        "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                        responseInstanceId);
                }

                this->eid = eid;
                setInitialProperties(sensor_paths::unitDegreesC);
                this->read();
            }
        });
}

void GpuTempSensor::discoverGpus()
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [this](const boost::system::error_code& ec, const getSubTreeRet& ret) {
            if (ec)
            {
                lg2::error("GpuTempSensor::discoverGpus(): Error:{ERROR}",
                           "ERROR", ec.message());
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
                                [this, service, objPath](
                                    const boost::system::error_code& configEc,
                                    const GpuSensorConfigMap& configs) {
                                    if (!configEc)
                                    {
                                        try
                                        {
                                            auto eid = std::get<uint8_t>(
                                                configs.at("EID"));
                                            auto mctpTypes = std::get<
                                                std::vector<uint8_t>>(
                                                configs.at(
                                                    "SupportedMessageTypes"));

                                            if (std::find(mctpTypes.begin(),
                                                          mctpTypes.end(),
                                                          ocpAmiMessageType) !=
                                                mctpTypes.end())
                                            {
                                                if (verbose)
                                                {
                                                    lg2::info(
                                                        "GpuTempSensor::discoverGpus(): Found OCPAMI Endpoint with ID {EID}",
                                                        "EID", eid);
                                                }
                                                this->processGpuEndpoint(eid);
                                            }
                                        }
                                        catch (const std::exception& e)
                                        {
                                            lg2::error(
                                                "GpuTempSensor::discoverGpus(): Error:{ERROR}",
                                                "ERROR", e.what());
                                        }
                                    }
                                    else
                                    {
                                        lg2::error(
                                            "GpuTempSensor::discoverGpus(): Error:{ERROR}",
                                            "ERROR", configEc.message());
                                    }
                                },
                                service, objPath,
                                "org.freedesktop.DBus.Properties", "GetAll",
                                iface);
                        }
                    }
                }
            }
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
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
        [&](boost::system::error_code ec, const ManagedObjectType& resp) {
            if (ec)
            {
                lg2::error("Error contacting entity manager");
                return;
            }
            for (const auto& [path, interfaces] : resp)
            {
                for (const auto& [intf, cfg] : interfaces)
                {
                    if (intf != configInterfaceName(sensorType))
                    {
                        continue;
                    }

                    std::string name = loadVariant<std::string>(cfg, "Name");

                    uint8_t entityId = loadVariant<uint64_t>(cfg, "EntityId");

                    uint8_t entityInstance =
                        loadVariant<uint64_t>(cfg, "EntityInstance");

                    DeviceInfo deviceInfo = {.deviceType = entityId,
                                             .instanceId = entityInstance};

                    sensors[name] = std::make_shared<GpuTempSensor>(
                        dbusConnection, io, mctpRequester, name, path,
                        objectServer, std::vector<thresholds::Threshold>{},
                        samplingInterval, deviceInfo, true);

                    lg2::info(
                        "Added GPU Temperature Sensor {NAME} with chassis path: {PATH}.",
                        "NAME", name, "PATH", path);
                }
            }
        },
        entityManagerName, "/xyz/openbmc_project/inventory",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
}
