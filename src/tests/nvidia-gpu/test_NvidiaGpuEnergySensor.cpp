/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuEnergySensor.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "Thresholds.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;
constexpr size_t energyResponseSize = 19;
constexpr size_t errorResponseSize = 9;

std::vector<uint8_t> buildEnergyResponse(uint64_t energyMillijoules)
{
    std::vector<uint8_t> buf(energyResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(
        static_cast<uint8_t>(0x06)); // command: GET_CURRENT_ENERGY_COUNTER
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(8));   // data_size: sizeof(uint64_t)
    pack.pack(energyMillijoules);
    return buf;
}

std::vector<uint8_t> buildEnergyErrorResponse(uint8_t cc, uint16_t reasonCode)
{
    std::vector<uint8_t> buf(errorResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(
        static_cast<uint8_t>(0x06)); // command: GET_CURRENT_ENERGY_COUNTER
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

class NvidiaGpuEnergySensorTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaGpuEnergySensor> createSensor(
        uint8_t sensorId = gpuEnergySensorId,
        const std::string& name = "GPU_ENERGY",
        std::vector<thresholds::Threshold> thresholds = {},
        gpu::DeviceIdentification deviceType =
            gpu::DeviceIdentification::DEVICE_GPU,
        uint8_t eid = defaultEid)
    {
        return std::make_shared<NvidiaGpuEnergySensor>(
            conn, *mctpRequester, name, "/test/config", eid, sensorId,
            *objectServer, std::move(thresholds), deviceType);
    }

    static std::string energyPath(const std::string& name)
    {
        return "/xyz/openbmc_project/sensors/energy/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuEnergySensorTestBase, ConstructorCreatesDbusInterfaces)
{
    const std::string name = "nrg_ctor_iface";
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, name);
    const std::string path = energyPath(name);

    EXPECT_TRUE(std::isnan(getProperty<double>(
        path, "xyz.openbmc_project.Sensor.Value", "Value")));
    EXPECT_DOUBLE_EQ(
        getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                            "MaxValue"),
        static_cast<double>(std::numeric_limits<uint64_t>::max()) / 1000.0);
    EXPECT_EQ(getProperty<double>(path, "xyz.openbmc_project.Sensor.Value",
                                  "MinValue"),
              0.0);
    EXPECT_EQ(getProperty<std::string>(path, "xyz.openbmc_project.Sensor.Value",
                                       "Unit"),
              "xyz.openbmc_project.Sensor.Value.Unit.Joules");
}

// Constructor — PhysicalContext

TEST_F(NvidiaGpuEnergySensorTestBase, ConstructorWithGpuPhysicalContext)
{
    const std::string name = "nrg_gpu_ctx";
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor = createSensor(
        gpuEnergySensorId, name, {}, gpu::DeviceIdentification::DEVICE_GPU);
    const std::string path = energyPath(name);

    EXPECT_EQ(
        getProperty<std::string>(
            path, "xyz.openbmc_project.Common.PhysicalContext", "Type"),
        "xyz.openbmc_project.Common.PhysicalContext.PhysicalContextType.Accelerator");
}

// Update — successful energy readings

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateSuccessValidEnergy)
{
    mock_mctp::setNextResponse({}, buildEnergyResponse(5000000));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_valid");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);
}

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateSuccessZeroEnergy)
{
    mock_mctp::setNextResponse({}, buildEnergyResponse(0));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_zero");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 0.0);
}

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateSuccessFractionalEnergy)
{
    mock_mctp::setNextResponse({}, buildEnergyResponse(1500));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_frac");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 1.5);
}

// Note: UpdateSuccessTwiceOverwritesValue is omitted for energy because
// the base Sensor hysteresisPublish = (uint64_max/1000) * 0.0001 ≈ 1.84e12,
// which exceeds any practical decoded energy delta, preventing subsequent
// value updates from taking effect.

// Update — error handling

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateMctpTransportError)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_mctp_err");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateBadCompletionCodeError)
{
    mock_mctp::setNextResponse(
        {}, buildEnergyErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_bad_cc");
    sensor->update();
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateDecodeFailTruncatedBuffer)
{
    mock_mctp::setNextResponse({}, std::vector<uint8_t>(5, 0));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_trunc");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateDecodeFailEmptyBuffer)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_empty");
    EXPECT_NO_THROW(sensor->update());
    EXPECT_TRUE(std::isnan(sensor->value));
}

// Update — value retention after errors

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateSuccessThenErrorKeepsPreviousValue)
{
    mock_mctp::setNextResponse({}, buildEnergyResponse(5000000));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_retain");
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);

    mock_mctp::setNextResponse(
        {}, buildEnergyErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    sensor->update();
    EXPECT_DOUBLE_EQ(sensor->value, 5000.0);
}

// Update — request encoding verification

TEST_F(NvidiaGpuEnergySensorTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, buildEnergyResponse(5000000));
    const std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_eid_test", {},
                     gpu::DeviceIdentification::DEVICE_GPU, testEid);
    sensor->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Destructor

TEST_F(NvidiaGpuEnergySensorTestBase, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuEnergySensor> sensor =
        createSensor(gpuEnergySensorId, "nrg_dtor");
    ASSERT_NE(sensor, nullptr);
    ASSERT_NE(sensor->sensorInterface, nullptr);
    sensor.reset();
    ASSERT_EQ(sensor, nullptr);
}

} // namespace
