/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Inventory.hpp"
#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuClockSpeedControl.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"
#include "TestUtils.hpp"

// NOLINTNEXTLINE(misc-include-cleaner): sd_bus_error lives here
#include <systemd/sd-bus.h>

#include <boost/system/error_code.hpp>
#include <sdbusplus/message.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* clockSpeedIface =
    "xyz.openbmc_project.Control.OperatingClockSpeed";
constexpr const char* clockSpeedPathPrefix =
    "/xyz/openbmc_project/control/operatingclockspeed/";

constexpr uint64_t mhzToHz = 1'000'000;

// Bounds reported by the mock GPU's inventory; every in-range test value is
// picked from inside [minClockMHz, maxClockMHz].
constexpr uint32_t minClockMHz = 300;
constexpr uint32_t maxClockMHz = 2100;

// Long enough to cover the control's 100 ms debounce plus D-Bus round trips.
constexpr std::chrono::seconds pumpTimeout{5};

std::vector<uint8_t> buildGetClockLimitResponse(
    uint32_t requestedMin, uint32_t requestedMax, uint32_t presentMin,
    uint32_t presentMax)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             (4 * sizeof(uint32_t)));
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(4 * sizeof(uint32_t)));
    pack.pack(requestedMin);
    pack.pack(requestedMax);
    pack.pack(presentMin);
    pack.pack(presentMax);
    return buf;
}

// SetClockLimit carries no payload, so its success reply is the common
// response header with a zero data size.
std::vector<uint8_t> buildSetClockLimitResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(0)); // data_size
    return buf;
}

// GetInventoryInformation reply carrying a uint32, the encoding the graphics
// clock bounds use.
std::vector<uint8_t> buildInventoryUint32Response(uint32_t value)
{
    return test_utils::buildPlatformEnvSuccessResponse(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION, value);
}

// A 16-byte payload decodes cleanly for every non-integral inventory property
// (string fields and the 16-byte GUID alike).
std::vector<uint8_t> buildInventoryStringResponse()
{
    const std::string value = "NVIDIA-CLK-TEST0";
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + value.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pack.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0));            // reserved
    pack.pack(static_cast<uint16_t>(value.size())); // data_size
    for (const char c : value)
    {
        pack.pack(static_cast<uint8_t>(c));
    }
    return buf;
}

uint8_t commandOf(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::messageHeaderSize];
}

// GetInventoryInformation appends the property id after the common header.
uint8_t inventoryPropertyOf(std::span<const uint8_t> request)
{
    return request[ocp::accelerator_management::commonRequestSize];
}

// Payload of a SetClockLimit request, as it sits on the wire after the common
// request header.
struct SetClockLimitFields
{
    uint8_t clockType{};
    uint8_t flag{};
    uint32_t limitMinMHz{};
    uint32_t limitMaxMHz{};
};

SetClockLimitFields decodeSetClockLimitRequest(std::span<const uint8_t> request)
{
    UnpackBuffer buffer{request};
    ocp::accelerator_management::MessageType msgType{};
    uint8_t instanceId = 0;
    uint8_t nsmMsgType = 0;
    ocp::accelerator_management::unpackHeader(buffer, gpu::nvidiaPciVendorId,
                                              msgType, instanceId, nsmMsgType);

    uint8_t command = 0;
    uint8_t dataSize = 0;
    SetClockLimitFields fields;
    buffer.unpack(command);
    buffer.unpack(dataSize);
    buffer.unpack(fields.clockType);
    buffer.unpack(fields.flag);
    buffer.unpack(fields.limitMinMHz);
    buffer.unpack(fields.limitMaxMHz);
    return fields;
}

class NvidiaGpuClockSpeedControlTest : public MctpMockTestBase
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

    // Answers a request the way a healthy GPU would.
    std::vector<uint8_t> buildReply(std::span<const uint8_t> request) const
    {
        switch (
            static_cast<gpu::PlatformEnvironmentalCommands>(commandOf(request)))
        {
            case gpu::PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION:
            {
                const auto propertyId = static_cast<gpu::InventoryPropertyId>(
                    inventoryPropertyOf(request));
                if (propertyId == gpu::InventoryPropertyId::MIN_GRAPHICS_CLOCK)
                {
                    return buildInventoryUint32Response(minClockMHz);
                }
                if (propertyId == gpu::InventoryPropertyId::MAX_GRAPHICS_CLOCK)
                {
                    return buildInventoryUint32Response(maxClockMHz);
                }
                return buildInventoryStringResponse();
            }
            case gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT:
                return buildGetClockLimitResponse(
                    clockLimit.requestedMin, clockLimit.requestedMax,
                    clockLimit.presentMin, clockLimit.presentMax);
            case gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT:
                return buildSetClockLimitResponse();
            default:
                return {};
        }
    }

    // Inventory drives the range check in the setters, so it must have
    // answered its MIN/MAX_GRAPHICS_CLOCK queries before the control is used.
    static std::shared_ptr<Inventory> createInventory(
        const std::string& name, uint8_t eid = test_utils::defaultEid)
    {
        auto inventory = std::make_shared<Inventory>(
            bus(), objects(), name, requester(),
            gpu::DeviceIdentification::DEVICE_GPU, eid, ioContext(), nullptr,
            nullptr, nullptr, nullptr);
        inventory->init();
        return inventory;
    }

    // shared_ptr ownership is required: the async handlers resolve
    // weak_from_this().
    static std::shared_ptr<NvidiaGpuClockSpeedControl> createControl(
        const std::string& name, const std::shared_ptr<Inventory>& inventory,
        uint8_t eid = test_utils::defaultEid)
    {
        return std::make_shared<NvidiaGpuClockSpeedControl>(
            objects(), name, requester(), eid, ioContext(), inventory);
    }

    static std::string pathFor(const std::string& name)
    {
        return clockSpeedPathPrefix + name;
    }

    size_t countRequests(gpu::PlatformEnvironmentalCommands command) const
    {
        return static_cast<size_t>(
            std::ranges::count_if(sentRequests, [command](const auto& request) {
                return commandOf(request) == static_cast<uint8_t>(command);
            }));
    }

    // The most recent request for `command`, or an empty vector if the
    // command was never sent.
    std::vector<uint8_t> lastRequest(
        gpu::PlatformEnvironmentalCommands command) const
    {
        for (const auto& request : sentRequests | std::views::reverse)
        {
            if (commandOf(request) == static_cast<uint8_t>(command))
            {
                return request;
            }
        }
        return {};
    }

    // Result of a D-Bus property write: the error name lets a rejected write
    // be told apart from a transport failure.
    struct SetResult
    {
        bool succeeded{false};
        std::string errorName;
    };

    // Writes a property on this connection's own object server. Async because
    // sd-bus rejects self-directed synchronous calls with ELOOP.
    static SetResult setSpeedLimit(const std::string& path,
                                   const std::string& property, uint64_t value)
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
            bus()->get_unique_name(), path, "org.freedesktop.DBus.Properties",
            "Set", clockSpeedIface, property, std::variant<uint64_t>(value));

        pumpIoUntil([result] { return result->has_value(); }, pumpTimeout);
        return result->value_or(SetResult{false, "timeout"});
    }

    // Values the mock GPU reports for GetClockLimit, in MHz.
    struct ClockLimit
    {
        uint32_t requestedMin{minClockMHz};
        uint32_t requestedMax{maxClockMHz};
        uint32_t presentMin{minClockMHz};
        uint32_t presentMax{maxClockMHz};
    };
    ClockLimit clockLimit;

    std::vector<std::vector<uint8_t>> sentRequests;
};

// Constructor — D-Bus interface creation

TEST_F(NvidiaGpuClockSpeedControlTest, ConstructorCreatesClockSpeedInterface)
{
    const std::string name = "clk_ctor";
    const auto control = createControl(name, createInventory(name));

    // Present* start at their registered defaults until the first poll.
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMinHz"),
              0U);
    // Requested* are backed by the class members, which start at zero.
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "RequestedSpeedLimitMinHz"),
              0U);
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "RequestedSpeedLimitMaxHz"),
              0U);
}

// Update — GetClockLimit publishes both the present and the requested limits

TEST_F(NvidiaGpuClockSpeedControlTest, UpdatePublishesClockLimits)
{
    const std::string name = "clk_update";
    clockLimit = {.requestedMin = 600,
                  .requestedMax = 1800,
                  .presentMin = 700,
                  .presentMax = 1700};

    const auto control = createControl(name, createInventory(name));
    control->update();

    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMinHz"),
              700U * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMaxHz"),
              1700U * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "RequestedSpeedLimitMinHz"),
              600U * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "RequestedSpeedLimitMaxHz"),
              1800U * mhzToHz);
}

// Set — a write inside the device's range reaches the device as SetClockLimit

TEST_F(NvidiaGpuClockSpeedControlTest, SetRequestedMaxIssuesSetClockLimit)
{
    const std::string name = "clk_set_max";
    const auto control = createControl(name, createInventory(name));
    sentRequests.clear();

    const uint64_t newMaxHz = 1500ULL * mhzToHz;
    const SetResult result =
        setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz", newMaxHz);
    ASSERT_TRUE(result.succeeded) << result.errorName;

    ASSERT_TRUE(pumpIoUntil(
        [this] {
            return countRequests(
                       gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT) > 0;
        },
        pumpTimeout));

    const std::vector<uint8_t> request =
        lastRequest(gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT);
    ASSERT_FALSE(request.empty());

    const SetClockLimitFields fields = decodeSetClockLimitRequest(request);
    EXPECT_EQ(fields.clockType,
              static_cast<uint8_t>(gpu::ClockType::GRAPHICS_CLOCK));
    EXPECT_EQ(fields.flag,
              static_cast<uint8_t>(gpu::ClockLimitFlag::PERSISTENCE));
    EXPECT_EQ(fields.limitMaxMHz, 1500U);
}

// Set — Min and Max writes of one PATCH coalesce into a single SetClockLimit

TEST_F(NvidiaGpuClockSpeedControlTest, SetMinAndMaxCoalesceIntoOneRequest)
{
    const std::string name = "clk_coalesce";
    const auto control = createControl(name, createInventory(name));
    sentRequests.clear();

    ASSERT_TRUE(setSpeedLimit(pathFor(name), "RequestedSpeedLimitMinHz",
                              1200ULL * mhzToHz)
                    .succeeded);
    ASSERT_TRUE(setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz",
                              1400ULL * mhzToHz)
                    .succeeded);

    ASSERT_TRUE(pumpIoUntil(
        [this] {
            return countRequests(
                       gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT) > 0;
        },
        pumpTimeout));

    // Let any second debounce expiry that would follow land before counting.
    pumpIoUntil([] { return false; }, std::chrono::seconds{1});

    EXPECT_EQ(
        countRequests(gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT), 1U);

    const std::vector<uint8_t> request =
        lastRequest(gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT);
    ASSERT_FALSE(request.empty());

    const SetClockLimitFields fields = decodeSetClockLimitRequest(request);
    EXPECT_EQ(fields.limitMinMHz, 1200U);
    EXPECT_EQ(fields.limitMaxMHz, 1400U);
}

// Set — a successful SetClockLimit is followed by a GetClockLimit read-back,
// so D-Bus reflects the device without waiting for the next poll.

TEST_F(NvidiaGpuClockSpeedControlTest, SetSuccessReadsBackFromDevice)
{
    const std::string name = "clk_readback";
    const auto control = createControl(name, createInventory(name));
    sentRequests.clear();

    clockLimit = {.requestedMin = 900,
                  .requestedMax = 1600,
                  .presentMin = 900,
                  .presentMax = 1600};

    ASSERT_TRUE(setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz",
                              1600ULL * mhzToHz)
                    .succeeded);

    ASSERT_TRUE(pumpIoUntil(
        [this] {
            return countRequests(
                       gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT) > 0;
        },
        pumpTimeout));

    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "RequestedSpeedLimitMaxHz"),
              1600U * mhzToHz);
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMaxHz"),
              1600U * mhzToHz);
}

// Set — rejected writes

TEST_F(NvidiaGpuClockSpeedControlTest, SetAboveHardwareMaxRejected)
{
    const std::string name = "clk_too_high";
    const auto control = createControl(name, createInventory(name));
    sentRequests.clear();

    const SetResult result =
        setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz",
                      (maxClockMHz + 1ULL) * mhzToHz);

    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName,
              "xyz.openbmc_project.Common.Error.InvalidArgument");
    EXPECT_EQ(
        countRequests(gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT), 0U);
}

TEST_F(NvidiaGpuClockSpeedControlTest, SetBelowHardwareMinRejected)
{
    const std::string name = "clk_too_low";
    const auto control = createControl(name, createInventory(name));
    sentRequests.clear();

    const SetResult result =
        setSpeedLimit(pathFor(name), "RequestedSpeedLimitMinHz",
                      (minClockMHz - 1ULL) * mhzToHz);

    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName,
              "xyz.openbmc_project.Common.Error.InvalidArgument");
    EXPECT_EQ(
        countRequests(gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT), 0U);
}

TEST_F(NvidiaGpuClockSpeedControlTest, SetWithoutInventoryRejected)
{
    const std::string name = "clk_no_inventory";
    const auto control = createControl(name, nullptr);

    const SetResult result = setSpeedLimit(
        pathFor(name), "RequestedSpeedLimitMaxHz", 1500ULL * mhzToHz);

    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName, "xyz.openbmc_project.Common.Error.Unavailable");
}

TEST_F(NvidiaGpuClockSpeedControlTest, SetBeforeInventoryKnownRejected)
{
    const std::string name = "clk_inventory_pending";
    // No init(): the graphics clock bounds have not been read from the device.
    auto inventory = std::make_shared<Inventory>(
        bus(), objects(), name, requester(),
        gpu::DeviceIdentification::DEVICE_GPU, test_utils::defaultEid,
        ioContext(), nullptr, nullptr, nullptr, nullptr);
    const auto control = createControl(name, inventory);

    const SetResult result = setSpeedLimit(
        pathFor(name), "RequestedSpeedLimitMaxHz", 1500ULL * mhzToHz);

    EXPECT_FALSE(result.succeeded);
    EXPECT_EQ(result.errorName, "xyz.openbmc_project.Common.Error.Unavailable");
}

// Error handling — a failed exchange must neither crash nor publish values

TEST_F(NvidiaGpuClockSpeedControlTest, UpdateMctpTransportErrorNoCrash)
{
    const std::string name = "clk_mctp_err";
    const auto inventory = createInventory(name);
    const auto control = createControl(name, inventory);

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            std::make_error_code(std::errc::timed_out), {}));

    EXPECT_NO_THROW(control->update());
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMinHz"),
              0U);
}

TEST_F(NvidiaGpuClockSpeedControlTest, UpdateBadCompletionCodeNoCrash)
{
    const std::string name = "clk_bad_cc";
    const auto inventory = createInventory(name);
    const auto control = createControl(name, inventory);

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith(
            {}, test_utils::buildPlatformEnvErrorResponse(
                    gpu::PlatformEnvironmentalCommands::GET_CLOCK_LIMIT,
                    static_cast<uint8_t>(
                        ocp::accelerator_management::CompletionCode::ERROR),
                    0)));

    EXPECT_NO_THROW(control->update());
    EXPECT_EQ(getProperty<uint64_t>(pathFor(name), clockSpeedIface,
                                    "PresentSpeedLimitMinHz"),
              0U);
}

TEST_F(NvidiaGpuClockSpeedControlTest, UpdateEmptyBufferNoCrash)
{
    const std::string name = "clk_empty";
    const auto inventory = createInventory(name);
    const auto control = createControl(name, inventory);

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(mock_mctp::respondWith({}, {}));

    EXPECT_NO_THROW(control->update());
}

// A SetClockLimit that fails must release the coalesce gate, so a later write
// still reaches the device.

TEST_F(NvidiaGpuClockSpeedControlTest, SetFailureDoesNotBlockLaterSet)
{
    const std::string name = "clk_set_err";
    const auto control = createControl(name, createInventory(name));

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([this](uint8_t /*eid*/, std::span<const uint8_t> request,
                              auto callback) {
            sentRequests.emplace_back(request.begin(), request.end());
            if (commandOf(request) ==
                static_cast<uint8_t>(
                    gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT))
            {
                callback(std::make_error_code(std::errc::timed_out), {});
                return;
            }
            const std::vector<uint8_t> response = buildReply(request);
            callback(std::error_code{}, response);
        });

    sentRequests.clear();
    ASSERT_TRUE(setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz",
                              1500ULL * mhzToHz)
                    .succeeded);
    ASSERT_TRUE(pumpIoUntil(
        [this] {
            return countRequests(
                       gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT) > 0;
        },
        pumpTimeout));

    ASSERT_TRUE(setSpeedLimit(pathFor(name), "RequestedSpeedLimitMaxHz",
                              1600ULL * mhzToHz)
                    .succeeded);
    EXPECT_TRUE(pumpIoUntil(
        [this] {
            return countRequests(
                       gpu::PlatformEnvironmentalCommands::SET_CLOCK_LIMIT) > 1;
        },
        pumpTimeout));
}

// Destructor

TEST_F(NvidiaGpuClockSpeedControlTest, DestructorDoesNotCrash)
{
    const std::string name = "clk_dtor";
    auto control = createControl(name, createInventory(name));
    ASSERT_NE(control, nullptr);
    EXPECT_NO_THROW(control.reset());
    drainPendingAsync();
}

} // namespace
