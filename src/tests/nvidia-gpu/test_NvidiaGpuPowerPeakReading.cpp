/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerPeakReading.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

std::vector<uint8_t> buildPeakPowerResponse(uint32_t powerMilliwatts)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_MAX_OBSERVED_POWER,
        powerMilliwatts);
}

std::vector<uint8_t> buildPeakPowerErrorResponse(uint8_t cc,
                                                 uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_MAX_OBSERVED_POWER, cc,
        reasonCode);
}

class NvidiaGpuPowerPeakReadingTest : public MctpMockTestBase
{
  protected:
    // shared_ptr ownership is required: update() resolves weak_from_this().
    static std::shared_ptr<NvidiaGpuPowerPeakReading> createPeakReading(
        const std::string& name = "GPU_PEAK_POWER",
        uint8_t eid = test_utils::defaultEid,
        uint8_t sensorId = gpuPeakPowerSensorId)
    {
        return std::make_shared<NvidiaGpuPowerPeakReading>(
            requester(), name, eid, sensorId, objects());
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuPowerPeakReadingTest, ConstructorCreatesTelemetryInterface)
{
    const std::string name = "peak_ctor";
    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading(name);
    const std::string path = "/xyz/openbmc_project/sensors/power/" + name;

    EXPECT_EQ(getProperty<bool>(path, "xyz.openbmc_project.Telemetry.Report",
                                "Persistency"),
              false);
    EXPECT_EQ(getProperty<bool>(path, "xyz.openbmc_project.Telemetry.Report",
                                "Enabled"),
              true);
}

// Update — sends request

TEST_F(NvidiaGpuPowerPeakReadingTest, UpdateSendsRequest)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith({}, buildPeakPowerResponse(100000)));

    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_sends");
    peak->update();
}

TEST_F(NvidiaGpuPowerPeakReadingTest, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    EXPECT_CALL(mctpMock, sendRecvMsg(testEid, testing::_, testing::_))
        .WillOnce(mock_mctp::respondWith({}, buildPeakPowerResponse(100000)));

    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_eid_test", testEid);
    peak->update();
}

// Error handling

TEST_F(NvidiaGpuPowerPeakReadingTest, UpdateMctpTransportErrorNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_mctp_err");
    EXPECT_NO_THROW(peak->update());
}

TEST_F(NvidiaGpuPowerPeakReadingTest, UpdateBadCompletionCodeNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .WillOnce(mock_mctp::respondWith(
            {}, buildPeakPowerErrorResponse(
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_bad_cc");
    EXPECT_NO_THROW(peak->update());
}

TEST_F(NvidiaGpuPowerPeakReadingTest, UpdateEmptyBufferNoCrash)
{
    EXPECT_CALL(mctpMock, sendRecvMsg).WillOnce(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_empty");
    EXPECT_NO_THROW(peak->update());
}

// Destructor

TEST_F(NvidiaGpuPowerPeakReadingTest, DestructorDoesNotCrash)
{
    std::shared_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_dtor");
    ASSERT_NE(peak, nullptr);
    EXPECT_NO_THROW(peak.reset());
}

} // namespace
