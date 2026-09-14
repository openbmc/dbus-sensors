/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct NvidiaSmaLeakSensor :
    public Sensor,
    public std::enable_shared_from_this<NvidiaSmaLeakSensor>
{
  public:
    NvidiaSmaLeakSensor(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid, uint8_t sensorId,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData,
        gpu::DeviceIdentification deviceType);

    ~NvidiaSmaLeakSensor() override;

    void checkThresholds() override;

    void update();

    void updateState(uint8_t value);

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    void addMonitoringAssociation(
        std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        const sdbusplus::object_path& path,
        const sdbusplus::object_path& monitoredPath,
        const std::string& detectorName);

    void registerThresholds(const sdbusplus::object_path& dbusPath);

    int handleThresholdSet(size_t index, const double& newValue);

    void armSetThresholdTimer();

    void applyRequestedThresholds();

    void updateThresholds(const std::vector<uint16_t>& reported);

    uint8_t eid{};

    uint8_t sensorId{};

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    sdbusplus::asio::object_server& objectServer;

    std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize> request{};

    std::array<uint8_t, gpu::setLeakDetectionThresholdsRequestSize>
        setRequest{};

    std::array<uint16_t, gpu::leakDetectorThresholdCount> deviceThresholds{};

    // The device holds a threshold a millivolt below the value written, so a
    // threshold a request does not name is sent as last requested, not as
    // last reported.
    std::array<uint16_t, gpu::leakDetectorThresholdCount> requestedThresholds{};

    std::array<std::optional<uint16_t>, gpu::leakDetectorThresholdCount>
        pendingThresholds;

    bool thresholdsKnown{false};

    bool setInflight{false};

    bool awaitingReadback{false};

    struct PublishedThreshold
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string property;
        size_t index;
    };

    std::vector<PublishedThreshold> publishedThresholds;

    boost::asio::steady_timer setThresholdTimer;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        commonPhysicalContextInterface;

    std::vector<gpu::LeakSensorData> parsedSensors;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakDetectorInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakFaultInterface;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakDetectorAssociation;

    std::shared_ptr<sdbusplus::asio::dbus_interface> leakFaultAssociation;

    enum class LeakState
    {
        Normal,
        Abnormal
    };

    LeakState lastLeakState = LeakState::Normal;

    LeakState lastLeakFault = LeakState::Normal;
};
