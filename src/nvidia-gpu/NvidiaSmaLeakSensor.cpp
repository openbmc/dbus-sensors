/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaLeakSensor.hpp"

#include "NvidiaSensorUtils.hpp"
#include "NvidiaUtils.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace std::literals;

static constexpr double smaLeakSensorMaxReading = 5;
static constexpr double smaLeakSensorMinReading = 0;

NvidiaSmaLeakSensor::NvidiaSmaLeakSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn, const std::string& name,
    const std::string& sensorConfiguration,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "voltage", false, true, smaLeakSensorMaxReading,
           smaLeakSensorMinReading, conn),
    conn(conn), objectServer(objectServer)
{
    std::string dbusPath = sensorPathPrefix + "voltage/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(dbusPath, interface);
    }

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitVolts);

    const std::optional<std::string> physicalContext =
        nvidia_sensor_utils::deviceTypeToPhysicalContext(deviceType);

    if (physicalContext)
    {
        commonPhysicalContextInterface = objectServer.add_interface(
            dbusPath, "xyz.openbmc_project.Common.PhysicalContext");

        commonPhysicalContextInterface->register_property("Type",
                                                          *physicalContext);

        if (!commonPhysicalContextInterface->initialize())
        {
            lg2::error(
                "Error initializing PhysicalContext Interface for Leak Sensor for sensor {NAME}",
                "NAME", name);
        }
    }

    std::string leakDetectorPath =
        "/xyz/openbmc_project/state/leak/detector/" + escapeName(name);
    leakDetectorInterface = objectServer.add_interface(
        leakDetectorPath, "xyz.openbmc_project.State.Leak.Detector");

    leakDetectorInterface->register_property("PrettyName", name);
    leakDetectorInterface->register_property(
        "State",
        std::string(
            "xyz.openbmc_project.State.Leak.Detector.DetectorState.Normal"));
    leakDetectorInterface->register_property(
        "Type",
        std::string(
            "xyz.openbmc_project.State.Leak.Detector.DetectorType.Unknown"));

    if (!leakDetectorInterface->initialize())
    {
        lg2::error(
            "Error initializing Leak Detector Interface for Leak Sensor for {NAME}",
            "NAME", name);
    }

    std::string leakFaultPath = "/xyz/openbmc_project/state/leak/detector/" +
                                escapeName(name) + "_Fault";
    leakFaultInterface = objectServer.add_interface(
        leakFaultPath, "xyz.openbmc_project.State.Leak.Detector");

    leakFaultInterface->register_property("PrettyName", name + "_Fault");
    leakFaultInterface->register_property(
        "State",
        std::string(
            "xyz.openbmc_project.State.Leak.Detector.DetectorState.Normal"));
    leakFaultInterface->register_property(
        "Type",
        std::string(
            "xyz.openbmc_project.State.Leak.Detector.DetectorType.Unknown"));

    if (!leakFaultInterface->initialize())
    {
        lg2::error(
            "Error initializing Leak Fault Interface for Leak Sensor for {NAME}",
            "NAME", name + "_Fault");
    }
}

NvidiaSmaLeakSensor::~NvidiaSmaLeakSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
    if (commonPhysicalContextInterface)
    {
        objectServer.remove_interface(commonPhysicalContextInterface);
    }
    if (leakDetectorInterface)
    {
        objectServer.remove_interface(leakDetectorInterface);
    }
    if (leakFaultInterface)
    {
        objectServer.remove_interface(leakFaultInterface);
    }
}

void NvidiaSmaLeakSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaSmaLeakSensor::updateState(uint8_t value)
{
    LeakState newState =
        ((value & 0x01) != 0) ? LeakState::Abnormal : LeakState::Normal;
    LeakState newFault =
        ((value & 0x02) != 0) ? LeakState::Abnormal : LeakState::Normal;

    auto triggerSystemdService = [this](const std::string& type,
                                        LeakState state,
                                        const std::string& targetName) {
        std::string action =
            (state == LeakState::Normal) ? "deassert" : "assert";
        std::string target = "xyz.openbmc_project.leakdetector." + type + "." +
                             action + "@" + targetName + ".service";

        lg2::info("Starting systemd target {TARGET} for leak state change",
                  "TARGET", target);

        conn->async_method_call(
            [target](const boost::system::error_code& errc,
                     const sdbusplus::object_path&) {
                if (errc)
                {
                    lg2::error("Failed to start systemd unit {TARGET}: {ERROR}",
                               "TARGET", target, "ERROR", errc.message());
                }
            },
            "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
            "org.freedesktop.systemd1.Manager", "StartUnit", target, "replace");
    };

    if (newState != lastLeakState)
    {
        lastLeakState = newState;
        leakDetectorInterface->set_property(
            "State",
            std::string(
                (lastLeakState == LeakState::Normal)
                    ? "xyz.openbmc_project.State.Leak.Detector.DetectorState.Normal"
                    : "xyz.openbmc_project.State.Leak.Detector.DetectorState.Abnormal"));

        triggerSystemdService("critical", lastLeakState, escapeName(name));

        std::string detectorPath =
            "/xyz/openbmc_project/state/leak/detector/" + escapeName(name);
        if (lastLeakState == LeakState::Abnormal)
        {
            using LeakDetectedCritical = sdbusplus::error::xyz::
                openbmc_project::state::leak::Detector::LeakDetectedCritical;
            lg2::commit(LeakDetectedCritical("DETECTOR_NAME", detectorPath));
        }
        else
        {
            using LeakDetectedNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            lg2::commit(LeakDetectedNormal("DETECTOR_NAME", detectorPath));
        }
    }

    if (newFault != lastLeakFault)
    {
        lastLeakFault = newFault;
        leakFaultInterface->set_property(
            "State",
            std::string(
                (lastLeakFault == LeakState::Normal)
                    ? "xyz.openbmc_project.State.Leak.Detector.DetectorState.Normal"
                    : "xyz.openbmc_project.State.Leak.Detector.DetectorState.Abnormal"));
        // Fault uses "critical" as the service type as well, but with _Fault
        // suffix in the name
        triggerSystemdService("critical", lastLeakFault,
                              escapeName(name) + "_Fault");

        std::string faultPath = "/xyz/openbmc_project/state/leak/detector/" +
                                escapeName(name) + "_Fault";
        if (lastLeakFault == LeakState::Abnormal)
        {
            using LeakDetectedCritical = sdbusplus::error::xyz::
                openbmc_project::state::leak::Detector::LeakDetectedCritical;
            lg2::commit(LeakDetectedCritical("DETECTOR_NAME", faultPath));
        }
        else
        {
            using LeakDetectedNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            lg2::commit(LeakDetectedNormal("DETECTOR_NAME", faultPath));
        }
    }
}

NvidiaSmaLeakSensorCarrier::NvidiaSmaLeakSensorCarrier(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid,
    sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    conn(conn), mctpRequester(mctpRequester), name(name),
    sensorConfiguration(sensorConfiguration), eid(eid),
    objectServer(objectServer), thresholdData(std::move(thresholdData)),
    deviceType(deviceType)
{}

void NvidiaSmaLeakSensorCarrier::init()
{
    auto leakReq = std::make_shared<
        std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize>>();
    gpu::encodeGetLeakDetectionInfoRequest(0, *leakReq);
    mctpRequester.sendRecvMsg(
        eid, *leakReq,
        [weak{weak_from_this()}, leakReq](const std::error_code& ec,
                                          std::span<const uint8_t> response) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaSmaLeakSensorCarrier reference");
                return;
            }
            if (ec)
            {
                lg2::error(
                    "Transport error probing Leak Sensor {NAME}: ec={EC}",
                    "NAME", self->name, "EC", ec.message());
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            self->parsedSensors.clear();

            auto rc = gpu::decodeGetLeakDetectionInfoResponse(
                response, cc, reasonCode, self->parsedSensors);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error creating Leak Sensor: decoding GetLeakDetectionInfo response for {NAME} failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "NAME", self->name, "RC", rc, "CC", cc, "RESC", reasonCode);
                return;
            }

            if (self->parsedSensors.empty())
            {
                lg2::error(
                    "Error creating Leak Sensor: decode success but no sensors returned for {NAME}",
                    "NAME", self->name);
                return;
            }

            for (size_t i = 0; i < self->parsedSensors.size(); ++i)
            {
                uint8_t sensorId = self->parsedSensors[i].sensorId;
                std::string sensorName =
                    self->name + "_" + std::to_string(sensorId);

                // Use dynamic thresholds from hardware if exactly 3 are
                // provided
                std::vector<thresholds::Threshold> sensorThresholds =
                    self->thresholdData;
                if (self->parsedSensors[i].thresholds.size() == 3)
                {
                    sensorThresholds.clear();
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL, thresholds::Direction::LOW,
                        self->parsedSensors[i].thresholds[0] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::WARNING, thresholds::Direction::LOW,
                        self->parsedSensors[i].thresholds[1] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL,
                        thresholds::Direction::HIGH,
                        self->parsedSensors[i].thresholds[2] / 1000.0);
                }

                auto newSensor = std::make_shared<NvidiaSmaLeakSensor>(
                    self->conn, sensorName, self->sensorConfiguration,
                    self->objectServer, std::move(sensorThresholds),
                    self->deviceType);
                newSensor->updateValue(
                    self->parsedSensors[i].adcReadingMv / 1000.0);
                self->leakSensors[sensorId] = newSensor;
            }
            lg2::info("Added SMA Leak Sensors with chassis path: {PATH}.",
                      "PATH", self->sensorConfiguration);
        });
}

void NvidiaSmaLeakSensorCarrier::processResponse(
    const std::error_code& ec, std::span<const uint8_t> buffer)
{
    if (ec)
    {
        lg2::error(
            "Error updating Leak Sensor: sending message over MCTP failed, rc={RC}",
            "RC", ec.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    parsedSensors.clear();

    auto rc = gpu::decodeGetLeakDetectionInfoResponse(buffer, cc, reasonCode,
                                                      parsedSensors);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating Leak Sensor: decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    if (parsedSensors.empty())
    {
        lg2::error(
            "Error updating Leak Sensor: decode success but no sensors returned");
        return;
    }

    for (const auto& sensorData : parsedSensors)
    {
        auto it = leakSensors.find(sensorData.sensorId);
        if (it != leakSensors.end())
        {
            // Reading from the device is in millivolts and unit set on the dbus
            // is volts.
            it->second->updateValue(sensorData.adcReadingMv / 1000.0);
            it->second->updateState(sensorData.leakState);
        }
    }
}

void NvidiaSmaLeakSensorCarrier::update()
{
    if (leakSensors.empty())
    {
        return;
    }

    auto rc = gpu::encodeGetLeakDetectionInfoRequest(0, request);

    if (rc != 0)
    {
        lg2::error("Error updating Leak Sensor: encode failed, rc={RC}", "RC",
                   rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaSmaLeakSensorCarrier> self = weak.lock();
            if (!self)
            {
                lg2::error("invalid reference to NvidiaSmaLeakSensorCarrier");
                return;
            }
            self->processResponse(ec, buffer);
        });
}
