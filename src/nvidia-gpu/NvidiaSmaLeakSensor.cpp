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
#include <boost/asio/error.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
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

static constexpr double millivoltsPerVolt = 1000.0;

static constexpr std::chrono::milliseconds setThresholdDebounce{100};

static constexpr size_t minLeakSlot = 0;
static constexpr size_t maxLeakSlot = 1;
static constexpr size_t maxNormalSlot = 2;

static std::optional<size_t> thresholdIndex(
    const thresholds::Threshold& threshold)
{
    if (threshold.level == thresholds::Level::CRITICAL)
    {
        return threshold.direction == thresholds::Direction::LOW
                   ? minLeakSlot
                   : maxNormalSlot;
    }

    if (threshold.level == thresholds::Level::WARNING &&
        threshold.direction == thresholds::Direction::LOW)
    {
        return maxLeakSlot;
    }

    return std::nullopt;
}

NvidiaSmaLeakSensor::NvidiaSmaLeakSensor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& sensorConfiguration, const uint8_t eid,
    const uint8_t sensorId, sdbusplus::asio::object_server& objectServer,
    std::vector<thresholds::Threshold>&& thresholdData,
    const gpu::DeviceIdentification deviceType) :
    Sensor(escapeName(name), std::move(thresholdData), sensorConfiguration,
           "voltage", false, true, smaLeakSensorMaxReading,
           smaLeakSensorMinReading, conn),
    conn(conn), mctpRequester(mctpRequester), eid(eid), sensorId(sensorId),
    objectServer(objectServer), setThresholdTimer(conn->get_io_context())
{
    std::string dbusPath = sensorPathPrefix + "voltage/"s + escapeName(name);

    sensorInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Sensor.Value");

    association = objectServer.add_interface(dbusPath, association::interface);

    setInitialProperties(sensor_paths::unitVolts);

    registerThresholds(dbusPath);

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

    const std::string monitoredPath =
        sdbusplus::object_path(sensorConfiguration).parent_path();

    addMonitoringAssociation(leakDetectorAssociation, leakDetectorPath,
                             monitoredPath, name);

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

    addMonitoringAssociation(leakFaultAssociation, leakFaultPath, monitoredPath,
                             name + "_Fault");
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
    if (leakDetectorAssociation)
    {
        objectServer.remove_interface(leakDetectorAssociation);
    }
    if (leakFaultAssociation)
    {
        objectServer.remove_interface(leakFaultAssociation);
    }
}

void NvidiaSmaLeakSensor::addMonitoringAssociation(
    std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
    const std::string& path, const std::string& monitoredPath,
    const std::string& detectorName)
{
    interface = objectServer.add_interface(path, association::interface);

    interface->register_property(
        "Associations",
        std::vector<Association>{
            Association{"monitoring", "monitored_by", monitoredPath}});

    if (!interface->initialize())
    {
        lg2::error("Error initializing Association Interface for {NAME}",
                   "NAME", detectorName);
        objectServer.remove_interface(interface);
        interface = nullptr;
    }
}

void NvidiaSmaLeakSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void NvidiaSmaLeakSensor::registerThresholds(const std::string& dbusPath)
{
    for (const auto& threshold : thresholds)
    {
        const auto level = static_cast<size_t>(threshold.level);

        auto& iface = thresholdInterfaces[level];
        if (!iface)
        {
            iface = objectServer.add_interface(
                dbusPath, thresholds::getInterface(threshold.level));
        }

        const std::string property =
            propertyLevel(threshold.level, threshold.direction);
        const std::string alarm =
            propertyAlarm(threshold.level, threshold.direction);

        if (property.empty() || alarm.empty())
        {
            continue;
        }

        const std::optional<size_t> index = thresholdIndex(threshold);

        if (!index)
        {
            iface->register_property(property, threshold.value);
            iface->register_property(alarm, false);
            continue;
        }

        const size_t slot = *index;

        iface->register_property<double>(
            property, threshold.value,
            [this, slot](const double& newValue, double& /*current*/) {
                return handleThresholdSet(slot, newValue);
            },
            [this, slot, configured = threshold.value](double&) {
                return thresholdsKnown
                           ? deviceThresholds[slot] / millivoltsPerVolt
                           : configured;
            });

        iface->register_property(alarm, false);

        publishedThresholds.emplace_back(iface, property, slot);
    }

    for (auto& iface : thresholdInterfaces)
    {
        if (iface && !iface->initialize())
        {
            lg2::error("Error initializing threshold interface for {NAME}",
                       "NAME", name);
        }
    }
}

int NvidiaSmaLeakSensor::handleThresholdSet(const size_t index,
                                            const double& newValue)
{
    if (!thresholdsKnown)
    {
        lg2::error(
            "Leak threshold set rejected for detector {ID}: the device has not reported its thresholds",
            "ID", sensorId);
        throw sdbusplus::error::xyz::openbmc_project::common::Unavailable();
    }

    const double millivolts = newValue * millivoltsPerVolt;

    if (!std::isfinite(millivolts) || millivolts < 0 ||
        millivolts > std::numeric_limits<uint16_t>::max())
    {
        lg2::error(
            "Leak threshold set rejected for detector {ID}: {VAL} is not a reading the device can hold",
            "ID", sensorId, "VAL", millivolts);
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }

    std::array<uint16_t, gpu::leakDetectorThresholdCount> candidate{};
    for (size_t slot = 0; slot < candidate.size(); ++slot)
    {
        candidate[slot] =
            pendingThresholds[slot].value_or(deviceThresholds[slot]);
    }
    candidate[index] = static_cast<uint16_t>(millivolts);

    // The device refuses a request that crosses its thresholds over.
    if (candidate[minLeakSlot] >= candidate[maxLeakSlot] ||
        candidate[maxLeakSlot] >= candidate[maxNormalSlot])
    {
        lg2::error(
            "Leak threshold set rejected for detector {ID}: {VAL} would cross the thresholds the device holds",
            "ID", sensorId, "VAL", millivolts);
        throw sdbusplus::error::xyz::openbmc_project::common::InvalidArgument();
    }

    pendingThresholds[index] = candidate[index];

    armSetThresholdTimer();

    return 1;
}

void NvidiaSmaLeakSensor::armSetThresholdTimer()
{
    setThresholdTimer.expires_after(setThresholdDebounce);
    setThresholdTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }

            std::shared_ptr<NvidiaSmaLeakSensor> self = weak.lock();
            if (!self)
            {
                return;
            }

            self->applyRequestedThresholds();
        });
}

void NvidiaSmaLeakSensor::applyRequestedThresholds()
{
    if (setInflight)
    {
        armSetThresholdTimer();
        return;
    }

    std::array<uint16_t, gpu::leakDetectorThresholdCount> sent{};
    for (size_t slot = 0; slot < sent.size(); ++slot)
    {
        sent[slot] = pendingThresholds[slot].value_or(deviceThresholds[slot]);
        pendingThresholds[slot].reset();
    }

    const int rc = gpu::encodeSetLeakDetectionThresholdsRequest(
        0, sensorId, sent[0], sent[1], sent[2], setRequest);

    if (rc != 0)
    {
        lg2::error(
            "Error setting leak thresholds for detector {ID}: encode failed, rc={RC}",
            "ID", sensorId, "RC", rc);
        return;
    }

    setInflight = true;

    mctpRequester.sendRecvMsg(
        eid, setRequest,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaSmaLeakSensor> self = weak.lock();
            if (!self)
            {
                return;
            }

            if (ec)
            {
                lg2::error(
                    "Error setting leak thresholds for detector {ID}: sending message over MCTP failed, rc={RC}",
                    "ID", self->sensorId, "RC", ec.message());
                self->setInflight = false;
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;

            const int rc = gpu::decodeSetLeakDetectionThresholdsResponse(
                buffer, cc, reasonCode);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error setting leak thresholds for detector {ID}: rc={RC}, cc={CC}, reasonCode={RESC}",
                    "ID", self->sensorId, "RC", rc, "CC", cc, "RESC",
                    reasonCode);
                self->setInflight = false;
                return;
            }

            self->awaitingReadback = true;

            if (self->readbackHandler)
            {
                self->readbackHandler();
            }
        });
}

void NvidiaSmaLeakSensor::setReadbackHandler(std::function<void()> handler)
{
    readbackHandler = std::move(handler);
}

void NvidiaSmaLeakSensor::readingAttempted()
{
    if (awaitingReadback)
    {
        awaitingReadback = false;
        setInflight = false;
    }
}

void NvidiaSmaLeakSensor::updateThresholds(
    const std::vector<uint16_t>& reported)
{
    if (reported.size() != gpu::leakDetectorThresholdCount)
    {
        return;
    }

    const auto previous = deviceThresholds;

    std::ranges::copy(reported, deviceThresholds.begin());

    thresholdsKnown = true;

    for (auto& threshold : thresholds)
    {
        const std::optional<size_t> index = thresholdIndex(threshold);

        if (index)
        {
            threshold.value = deviceThresholds[*index] / millivoltsPerVolt;
        }
    }

    for (const auto& published : publishedThresholds)
    {
        if (deviceThresholds[published.index] != previous[published.index])
        {
            published.interface->signal_property(published.property);
        }
    }
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
                    self->conn, self->mctpRequester, sensorName,
                    self->sensorConfiguration, self->eid, sensorId,
                    self->objectServer, std::move(sensorThresholds),
                    self->deviceType);
                newSensor->setReadbackHandler([weak{self->weak_from_this()}] {
                    std::shared_ptr<NvidiaSmaLeakSensorCarrier> carrier =
                        weak.lock();
                    if (carrier)
                    {
                        carrier->update();
                    }
                });
                newSensor->updateThresholds(self->parsedSensors[i].thresholds);
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
    for (const auto& [sensorId, sensor] : leakSensors)
    {
        sensor->readingAttempted();
    }

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
            it->second->updateThresholds(sensorData.thresholds);
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
