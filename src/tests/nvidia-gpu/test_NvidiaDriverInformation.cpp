/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MockDbusHelper.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaDriverInformation.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

namespace
{

// GetDriverInformationResponse layout:
// CommonResponse (11 bytes) = Header(5) + command(1) + cc(1) + reserved(2) +
//   data_size(2) + DriverState(1) + driverVersion(1+)
// Buffer must be 13 + version.size() so the decoder reads version.size() chars
// (decoder computes versionSize = buf.size() - 13 + 1 from the trailing field).
std::vector<uint8_t> buildDriverInfoResponse(uint8_t driverState,
                                             const std::string& version)
{
    const size_t versionSize = version.size();
    const size_t responseSize =
        ocp::accelerator_management::commonResponseSize + 1 + versionSize + 1;
    std::vector<uint8_t> buf(responseSize, 0);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(2 + versionSize));
    pack.pack(driverState);
    for (const char c : version)
    {
        pack.pack(static_cast<uint8_t>(c));
    }
    return buf;
}

std::vector<uint8_t> buildDriverInfoErrorResponse(uint8_t cc,
                                                  uint16_t reasonCode)
{
    return test_utils::buildPlatformEnvErrorResponse(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION, cc,
        reasonCode);
}

class NvidiaDriverInformationTestBase : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaDriverInformation> createDriverInfo(
        const std::string& name = "GPU_DRIVER",
        uint8_t eid = test_utils::defaultEid)
    {
        const sdbusplus::message::object_path path =
            "/xyz/openbmc_project/software/" + name;
        return std::make_shared<NvidiaDriverInformation>(
            conn, *mctpRequester, name, path, eid, *objectServer);
    }

    static std::string driverPath(const std::string& name)
    {
        return "/xyz/openbmc_project/software/" + name;
    }
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaDriverInformationTestBase, ConstructorCreatesVersionInterface)
{
    const std::string name = "drv_ctor";
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo(name);
    const std::string path = driverPath(name);

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Software.Version", "Version"),
              "");
    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Software.Version", "Purpose"),
              "xyz.openbmc_project.Software.Version.VersionPurpose.Other");
}

// Update — successful version update

TEST_F(NvidiaDriverInformationTestBase, UpdateSuccessUpdatesVersion)
{
    const std::string name = "drv_update";
    mock_mctp::setNextResponse(
        {}, buildDriverInfoResponse(
                static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED),
                "535.129.03"));
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo(name);
    driverInfo->update();
    const std::string path = driverPath(name);

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Software.Version", "Version"),
              "535.129.03");
}

TEST_F(NvidiaDriverInformationTestBase, UpdateSuccessTwiceOverwritesVersion)
{
    const std::string name = "drv_update2";
    mock_mctp::setNextResponse(
        {}, buildDriverInfoResponse(
                static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED),
                "535"));
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo(name);
    driverInfo->update();
    const std::string path = driverPath(name);

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Software.Version", "Version"),
              "535");

    mock_mctp::setNextResponse(
        {}, buildDriverInfoResponse(
                static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED),
                "540"));
    driverInfo->update();

    EXPECT_EQ(getProperty<std::string>(
                  path, "xyz.openbmc_project.Software.Version", "Version"),
              "540");
}

// Update — request encoding verification

TEST_F(NvidiaDriverInformationTestBase, UpdateVerifiesRequestEncoding)
{
    mock_mctp::setNextResponse(
        {}, buildDriverInfoResponse(
                static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED),
                "535"));
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo("drv_req_enc");
    driverInfo->update();

    const std::span<const uint8_t> req = mock_mctp::getLastRequest();
    ASSERT_FALSE(req.empty());

    UnpackBuffer unpack(req);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    const int rc = ocp::accelerator_management::unpackHeader(
        unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(ocpMsgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    unpack.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION));

    EXPECT_EQ(unpack.getError(), 0);
}

TEST_F(NvidiaDriverInformationTestBase, UpdateRequestContainsCorrectEid)
{
    constexpr uint8_t testEid = 42;
    mock_mctp::setNextResponse(
        {}, buildDriverInfoResponse(
                static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED),
                "535"));
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo("drv_eid_test", testEid);
    driverInfo->update();
    EXPECT_EQ(mock_mctp::getLastEid(), testEid);
}

// Error handling

TEST_F(NvidiaDriverInformationTestBase, UpdateMctpTransportErrorNoCrash)
{
    mock_mctp::setNextResponse(std::make_error_code(std::errc::timed_out), {});
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo("drv_mctp_err");
    EXPECT_NO_THROW(driverInfo->update());
}

TEST_F(NvidiaDriverInformationTestBase, UpdateBadCompletionCodeNoCrash)
{
    mock_mctp::setNextResponse(
        {}, buildDriverInfoErrorResponse(
                static_cast<uint8_t>(
                    ocp::accelerator_management::CompletionCode::ERROR),
                0));
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo("drv_bad_cc");
    EXPECT_NO_THROW(driverInfo->update());
}

TEST_F(NvidiaDriverInformationTestBase, UpdateEmptyBufferNoCrash)
{
    mock_mctp::setNextResponse({}, {});
    const std::shared_ptr<NvidiaDriverInformation> driverInfo =
        createDriverInfo("drv_empty");
    EXPECT_NO_THROW(driverInfo->update());
}

} // namespace
