/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaLldpConfiguration.hpp"
#include "OcpMctpVdm.hpp"

// NOLINTNEXTLINE(misc-include-cleaner): sd_bus_error lives here
#include <systemd/sd-bus.h>

#include <boost/system/error_code.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <variant>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* lldpIface =
    "xyz.openbmc_project.Network.LLDP.Configuration";
constexpr const char* modePrefix =
    "xyz.openbmc_project.Network.LLDP.Configuration.Mode.";

// An interface can only be registered on a path once per process, so each
// test works on a path of its own rather than inheriting whatever the test
// before it left registered.
std::string deviceNameFor()
{
    return std::string("CX_") +
           testing::UnitTest::GetInstance()->current_test_info()->name();
}

constexpr uint8_t testEid = 42;

// Long enough to cover the control's 100 ms debounce plus D-Bus round trips.
constexpr std::chrono::seconds pumpTimeout{5};

// The mode byte packs transmit in bits 0:1, receive in bits 2:3 and the
// neighboring protocol in bit 4.
constexpr uint8_t modeReceiveAll = 0b0'10'00;
constexpr uint8_t modeTransmitAll = 0b0'00'10;
constexpr uint8_t modeDcbxOnBothAll = 0b1'10'10;

std::vector<uint8_t> buildGetModeResponse(uint8_t modeData)
{
    // Data size, then the length of the mode in force, the length of the one
    // a reset would bring in, and the two values.
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             (2 * sizeof(uint16_t)) + (2 * sizeof(uint8_t)));
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CONFIGURATION));
    pack.pack(static_cast<uint8_t>(
        gpu::DeviceConfigurationCommands::GET_DEVICE_MODE_SETTINGS_V2));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(6)); // data size
    pack.pack(static_cast<uint16_t>(1)); // current length
    pack.pack(static_cast<uint16_t>(1)); // pending length
    pack.pack(modeData);                 // current
    pack.pack(modeData);                 // pending
    return buf;
}

std::vector<uint8_t> buildSetModeResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CONFIGURATION));
    pack.pack(static_cast<uint8_t>(
        gpu::DeviceConfigurationCommands::SET_DEVICE_MODE_SETTINGS_V2));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(0)); // data size
    return buf;
}

// A device that does not hold the mode answers every read this way.
std::vector<uint8_t> buildUnsupportedResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CONFIGURATION));
    pack.pack(static_cast<uint8_t>(
        gpu::DeviceConfigurationCommands::GET_DEVICE_MODE_SETTINGS_V2));
    pack.pack(static_cast<uint8_t>(ocp::accelerator_management::CompletionCode::
                                       ERR_UNSUPPORTED_COMMAND_CODE));
    pack.pack(static_cast<uint16_t>(0));
    pack.pack(static_cast<uint16_t>(0));
    return buf;
}

uint8_t commandOf(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::messageHeaderSize];
}

// The mode byte of a write, as it sits after the common request header and
// the four byte index that selects which mode is being written.
uint8_t modeDataOf(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::commonRequestSize +
                   sizeof(uint32_t)];
}

class NvidiaLldpConfigurationTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(
                [this](uint8_t /*eid*/, std::span<const uint8_t> request,
                       auto callback) {
                    sentRequests.emplace_back(request.begin(), request.end());
                    const std::vector<uint8_t> response = buildReply(request);
                    callback(std::error_code{}, response);
                });
    }

    std::vector<uint8_t> buildReply(std::span<const uint8_t> request)
    {
        const auto command =
            static_cast<gpu::DeviceConfigurationCommands>(commandOf(request));

        if (command ==
            gpu::DeviceConfigurationCommands::SET_DEVICE_MODE_SETTINGS_V2)
        {
            // A device that takes a write reports it from then on.
            deviceMode = modeDataOf(request);
            return buildSetModeResponse();
        }

        return supported ? buildGetModeResponse(deviceMode)
                         : buildUnsupportedResponse();
    }

    static std::string lldpPath()
    {
        return "/xyz/openbmc_project/network/lldp/" + deviceNameFor();
    }

    static std::string adapterPath()
    {
        return "/xyz/openbmc_project/inventory/" + deviceNameFor();
    }

    static std::shared_ptr<NvidiaLldpConfiguration> makeConfiguration()
    {
        auto configuration = std::make_shared<NvidiaLldpConfiguration>(
            objects(), requester(), deviceNameFor(),
            sdbusplus::object_path(adapterPath()), testEid, ioContext());
        configuration->update();
        return configuration;
    }

    // The outcome of a client write: whether the bus accepted it and, when it
    // did not, which error the daemon raised.
    struct SetResult
    {
        bool succeeded{};
        std::string errorName;
    };

    // A write has to go through the bus rather than straight at the object: a
    // synchronous call to oneself deadlocks.
    static SetResult setMode(const std::string& property,
                             const std::string& value)
    {
        auto result = std::make_shared<std::optional<SetResult>>();

        bus()->async_method_call(
            [result](const boost::system::error_code& ec,
                     sdbusplus::message_t& reply) {
                SetResult outcome;
                outcome.succeeded = !ec;
                // NOLINTNEXTLINE(misc-include-cleaner): from sd-bus.h
                const sd_bus_error* error = reply.get_error();
                if (error != nullptr && error->name != nullptr)
                {
                    outcome.errorName = error->name;
                }
                *result = outcome;
            },
            bus()->get_unique_name(), lldpPath(),
            "org.freedesktop.DBus.Properties", "Set", lldpIface, property,
            std::variant<std::string>(value));

        pumpIoUntil([result] { return result->has_value(); }, pumpTimeout);
        return result->value_or(SetResult{false, "timeout"});
    }

    size_t countWrites() const
    {
        size_t writes = 0;
        for (const std::vector<uint8_t>& request : sentRequests)
        {
            if (static_cast<gpu::DeviceConfigurationCommands>(
                    commandOf(request)) ==
                gpu::DeviceConfigurationCommands::SET_DEVICE_MODE_SETTINGS_V2)
            {
                ++writes;
            }
        }
        return writes;
    }

    std::vector<std::vector<uint8_t>> sentRequests;
    uint8_t deviceMode{0};
    bool supported{true};
};

TEST_F(NvidiaLldpConfigurationTest, ReportsWhatTheDeviceHolds)
{
    deviceMode = modeReceiveAll;
    auto configuration = makeConfiguration();

    EXPECT_EQ(getProperty<std::string>(lldpPath(), lldpIface, "ReceiveMode"),
              std::string(modePrefix) + "All");
    EXPECT_EQ(getProperty<std::string>(lldpPath(), lldpIface, "TransmitMode"),
              std::string(modePrefix) + "Disabled");
}

TEST_F(NvidiaLldpConfigurationTest, PointsAtTheAdapterItConfigures)
{
    auto configuration = makeConfiguration();

    const auto associations = getProperty<
        std::vector<std::tuple<std::string, std::string, std::string>>>(
        lldpPath(), "xyz.openbmc_project.Association.Definitions",
        "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations.front()), "controlling");
    EXPECT_EQ(std::get<1>(associations.front()), "controlled_by");
    EXPECT_EQ(std::get<2>(associations.front()), adapterPath());
}

// A device that cannot be configured must not be given a control that goes
// nowhere.
TEST_F(NvidiaLldpConfigurationTest, PublishesNothingWhenTheDeviceHasNoMode)
{
    supported = false;
    auto configuration = makeConfiguration();

    EXPECT_ANY_THROW(
        getProperty<std::string>(lldpPath(), lldpIface, "ReceiveMode"));
}

TEST_F(NvidiaLldpConfigurationTest, FollowsTheDeviceWhenItChangesUnderneath)
{
    auto configuration = makeConfiguration();
    ASSERT_EQ(getProperty<std::string>(lldpPath(), lldpIface, "TransmitMode"),
              std::string(modePrefix) + "Disabled");

    deviceMode = modeTransmitAll;
    configuration->update();

    EXPECT_TRUE(pumpIoUntil(
        [] {
            return getProperty<std::string>(lldpPath(), lldpIface,
                                            "TransmitMode") ==
                   std::string(modePrefix) + "All";
        },
        pumpTimeout));
}

TEST_F(NvidiaLldpConfigurationTest, LeavesTheOtherDirectionAlone)
{
    deviceMode = modeTransmitAll;
    auto configuration = makeConfiguration();

    ASSERT_TRUE(
        setMode("ReceiveMode", std::string(modePrefix) + "All").succeeded);

    ASSERT_TRUE(pumpIoUntil([this] { return countWrites() > 0; }, pumpTimeout));
    EXPECT_EQ(deviceMode, modeTransmitAll | modeReceiveAll);
}

// The neighboring protocol needs every field of a frame in both directions,
// so narrowing either while it is on has to be refused.
TEST_F(NvidiaLldpConfigurationTest, RefusesAWriteThatWouldBreakTheNeighbor)
{
    deviceMode = modeDcbxOnBothAll;
    auto configuration = makeConfiguration();

    const SetResult result =
        setMode("ReceiveMode", std::string(modePrefix) + "Mandatory");

    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName, "xyz.openbmc_project.Common.Error.Unavailable");
    EXPECT_EQ(countWrites(), 0U);
}

} // namespace
