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

#include <array>
#include <cstdint>
#include <functional>
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

    void updateState(uint8_t value);

    void updateThresholds(const std::vector<uint16_t>& reported);

    void setReadbackHandler(std::function<void()> handler);

    // Reopens the gate; a device that stops answering must not wedge it.
    void readingAttempted();

  private:
    void addMonitoringAssociation(
        std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        const std::string& path, const std::string& monitoredPath,
        const std::string& detectorName);

    void registerThresholds(const std::string& dbusPath);

    int handleThresholdSet(size_t index, const double& newValue);

    void armSetThresholdTimer();

    void applyRequestedThresholds();

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    uint8_t eid;

    uint8_t sensorId;

    std::array<uint16_t, gpu::leakDetectorThresholdCount> deviceThresholds{};

    std::array<std::optional<uint16_t>, gpu::leakDetectorThresholdCount>
        pendingThresholds;

    bool setInflight{false};

    // A second burst before this lands would undo the first, because a
    // threshold it does not name is sent as the device last reported it.
    bool awaitingReadback{false};

    std::function<void()> readbackHandler;

    struct PublishedThreshold
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string property;
        size_t index;
    };

    std::vector<PublishedThreshold> publishedThresholds;

    bool thresholdsKnown{false};

    std::array<uint8_t, gpu::setLeakDetectionThresholdsRequestSize>
        setRequest{};

    sdbusplus::asio::object_server& objectServer;

    boost::asio::steady_timer setThresholdTimer;

    std::shared_ptr<sdbusplus::asio::dbus_interface>
        commonPhysicalContextInterface;

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

struct NvidiaSmaLeakSensorCarrier :
    public std::enable_shared_from_this<NvidiaSmaLeakSensorCarrier>
{
  public:
    NvidiaSmaLeakSensorCarrier(
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        mctp::MctpRequester& mctpRequester, const std::string& name,
        const std::string& sensorConfiguration, uint8_t eid,
        sdbusplus::asio::object_server& objectServer,
        std::vector<thresholds::Threshold>&& thresholdData,
        gpu::DeviceIdentification deviceType);

    ~NvidiaSmaLeakSensorCarrier() = default;

    void init();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::connection> conn;

    mctp::MctpRequester& mctpRequester;

    std::string name;

    std::string sensorConfiguration;

    uint8_t eid;

    sdbusplus::asio::object_server& objectServer;

    std::vector<thresholds::Threshold> thresholdData;

    gpu::DeviceIdentification deviceType;

    std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize> request{};

    std::map<uint8_t, std::shared_ptr<NvidiaSmaLeakSensor>> leakSensors;

    std::vector<gpu::LeakSensorData> parsedSensors;
};
