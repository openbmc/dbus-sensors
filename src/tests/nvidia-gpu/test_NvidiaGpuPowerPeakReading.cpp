/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaGpuPowerPeakReading.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 10;
constexpr size_t peakPowerResponseSize = 15;

std::vector<uint8_t> buildPeakPowerResponse(uint32_t powerMilliwatts)
{
    std::vector<uint8_t> buf(peakPowerResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x04)); // command: GET_MAX_OBSERVED_POWER
    pack.pack(static_cast<uint8_t>(0x00)); // cc: SUCCESS
    pack.pack(static_cast<uint16_t>(0));   // reserved
    pack.pack(static_cast<uint16_t>(4));   // data_size: sizeof(uint32_t)
    pack.pack(powerMilliwatts);
    return buf;
}

std::vector<uint8_t> buildPeakPowerErrorResponse(uint8_t cc,
                                                 uint16_t reasonCode)
{
    std::vector<uint8_t> buf(9);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(0x04)); // command: GET_MAX_OBSERVED_POWER
    pack.pack(cc);
    pack.pack(reasonCode);
    return buf;
}

class NvidiaGpuPowerPeakReadingTestBase : public DbusMockTestBase
{
  protected:
    static std::unique_ptr<NvidiaGpuPowerPeakReading> createPeakReading(
        const std::string& name = "GPU_PEAK_POWER", uint8_t eid = defaultEid,
        uint8_t sensorId = gpuPeakPowerSensorId)
    {
        return std::make_unique<NvidiaGpuPowerPeakReading>(
            *mctpRequester, name, eid, sensorId, *objectServer);
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuPowerPeakReadingTestBase, ConstructorCreatesTelemetryInterface)
{
    const std::string name = "peak_ctor";
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
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

TEST_F(NvidiaGpuPowerPeakReadingTestBase, UpdateSendsRequest)
{
    mock_mctp::setNextResponse({}, buildPeakPowerResponse(100000));
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_sends");
    peak->update();

    EXPECT_GE(mock_mctp::getRequestCount(), 1U);
}

TEST_F(NvidiaGpuPowerPeakReadingTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse({}, buildPeakPowerResponse(100000));
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_eid_test", testEid);
    peak->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Error handling

TEST_F(NvidiaGpuPowerPeakReadingTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_mctp_err");
    EXPECT_NO_THROW(peak->update());
}

TEST_F(NvidiaGpuPowerPeakReadingTestBase, UpdateBadCompletionCodeNoCrash)
{
    mock_mctp::setNextResponse(
        {}, buildPeakPowerErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_bad_cc");
    EXPECT_NO_THROW(peak->update());
}

TEST_F(NvidiaGpuPowerPeakReadingTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_empty");
    EXPECT_NO_THROW(peak->update());
}

// Destructor

TEST_F(NvidiaGpuPowerPeakReadingTestBase, DestructorDoesNotCrash)
{
    std::unique_ptr<NvidiaGpuPowerPeakReading> peak =
        createPeakReading("peak_dtor");
    ASSERT_NE(peak, nullptr);
    EXPECT_NO_THROW(peak.reset());
}

} // namespace
