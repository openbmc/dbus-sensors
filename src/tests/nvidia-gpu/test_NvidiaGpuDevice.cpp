/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaEventReporting.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "OcpMctpVdm.hpp"

#include <MessagePackUnpackUtils.hpp>
#include <sdbusplus/exception.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr uint8_t defaultEid = 20;

constexpr const char* dimmIface = "xyz.openbmc_project.Inventory.Item.Dimm";

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

using DiscoveryCommands = gpu::DeviceCapabilityDiscoveryCommands;

struct DecodedRequest
{
    uint8_t messageType{};
    uint8_t command{};
};

DecodedRequest decodeRequest(std::span<const uint8_t> request)
{
    UnpackBuffer buffer(request);
    ocp::accelerator_management::MessageType messageType{};
    uint8_t instanceId = 0;
    uint8_t nvidiaMessageType = 0;
    ocp::accelerator_management::unpackHeader(
        buffer, gpu::nvidiaPciVendorId, messageType, instanceId,
        nvidiaMessageType);

    DecodedRequest decoded{};
    decoded.messageType = nvidiaMessageType;
    buffer.unpack(decoded.command);
    return decoded;
}

std::array<uint8_t, gpu::supportedListBitfieldSize> bitsOf(
    std::initializer_list<uint8_t> codes)
{
    std::array<uint8_t, gpu::supportedListBitfieldSize> bits{};
    for (uint8_t code : codes)
    {
        bits[code / 8U] |= static_cast<uint8_t>(1U << (code % 8U));
    }
    return bits;
}

std::vector<uint8_t> buildSupportedListResponse(
    DiscoveryCommands command,
    const std::array<uint8_t, gpu::supportedListBitfieldSize>& bits)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             gpu::supportedListBitfieldSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pack.pack(static_cast<uint8_t>(command));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(gpu::supportedListBitfieldSize));
    for (uint8_t byte : bits)
    {
        pack.pack(byte);
    }
    return buf;
}

std::vector<uint8_t> buildRediscoveryEvent()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::eventHeaderSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::REQUEST, 0,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    constexpr uint8_t eventVersion = 1;
    pack.pack(static_cast<uint8_t>(
        eventVersion & ocp::accelerator_management::eventVersionBitMask));
    pack.pack(static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryEvents::REDISCOVERY));
    pack.pack(static_cast<uint8_t>(0));  // event class
    pack.pack(static_cast<uint16_t>(0)); // event state
    pack.pack(static_cast<uint8_t>(0));  // no trailing event data
    return buf;
}

class NvidiaGpuDeviceTest : public MctpMockTestBase
{
  protected:
    static std::shared_ptr<GpuDevice> createDevice(
        const std::string& name = "GPU_DEV", uint8_t eid = defaultEid,
        uint64_t pollRate = sensorPollRateMs)
    {
        const std::string path = "/test/gpu/" + name;
        const SensorConfigs configs{.name = name, .pollRate = pollRate};
        return std::make_shared<GpuDevice>(configs, name, path, bus(), eid,
                                           ioContext(), requester(), objects());
    }

    static std::string dramPath(const std::string& name)
    {
        return "/xyz/openbmc_project/inventory/" + name + "_DRAM_0";
    }
};

// Constructor

TEST_F(NvidiaGpuDeviceTest, ConstructorDoesNotCrash)
{
    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_ctor");
    EXPECT_NE(device, nullptr);
}

TEST_F(NvidiaGpuDeviceTest, GetPathReturnsConfiguredPath)
{
    const std::string name = "gpudev_path";
    const std::shared_ptr<GpuDevice> device = createDevice(name);
    EXPECT_EQ(device->getPath(), "/test/gpu/" + name);
}

// Init

TEST_F(NvidiaGpuDeviceTest, InitSendsAtLeastOneRequest)
{
    // init() sends one request per sensor/metric; complete each with an
    // empty response so any chained requests still make progress.
    EXPECT_CALL(mctpMock, sendRecvMsg)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(mock_mctp::respondWith({}, {}));

    const std::shared_ptr<GpuDevice> device = createDevice("gpudev_init");
    device->init();
}

// ReadLoop

TEST_F(NvidiaGpuDeviceTest, ReadLoopStopsAfterDeviceIsDestroyed)
{
    int requests = 0;
    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault(
            [&requests](uint8_t /*eid*/, std::span<const uint8_t> /*reqMsg*/,
                        auto callback) {
                ++requests;
                callback(std::error_code{}, std::span<const uint8_t>{});
            });

    {
        const std::shared_ptr<GpuDevice> device =
            createDevice("gpudev_readloop", defaultEid, fastPollMs);
        device->init();

        // While the device is alive the poll timer keeps re-arming the loop,
        // so a later round has to produce more requests.
        const int afterInit = requests;
        ASSERT_TRUE(
            pumpIoUntil([&] { return requests > afterInit; }, pollTimeout));
    }

    // Once the device is gone the loop must stop: no further request may be
    // issued even after several more poll intervals have elapsed.
    const int afterDestroy = requests;
    EXPECT_FALSE(
        pumpIoUntil([&] { return requests > afterDestroy; }, quietWindow));
}

// Rediscovery

TEST_F(NvidiaGpuDeviceTest, RediscoveryEventRequeriesSupportedCommandCodes)
{
    int commandCodeQueries = 0;

    ON_CALL(mctpMock, sendRecvMsg)
        .WillByDefault([&commandCodeQueries](uint8_t /*eid*/,
                                             std::span<const uint8_t> request,
                                             auto callback) {
            const DecodedRequest decoded = decodeRequest(request);
            if (decoded.messageType !=
                static_cast<uint8_t>(
                    gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY))
            {
                callback(std::error_code{}, std::span<const uint8_t>{});
                return;
            }
            if (decoded.command ==
                static_cast<uint8_t>(
                    DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES))
            {
                callback(
                    std::error_code{},
                    buildSupportedListResponse(
                        DiscoveryCommands::GET_SUPPORTED_MESSAGE_TYPES,
                        bitsOf({static_cast<uint8_t>(
                            gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY)})));
                return;
            }
            ++commandCodeQueries;
            callback(std::error_code{},
                     buildSupportedListResponse(
                         DiscoveryCommands::GET_SUPPORTED_COMMAND_CODES,
                         bitsOf({})));
        });

    const std::shared_ptr<GpuDevice> device =
        createDevice("gpudev_requery", defaultEid, fastPollMs);
    device->init();

    // The rediscovery handler is only registered once the initial read of the
    // supported command codes has completed.
    ASSERT_TRUE(
        pumpIoUntil([&] { return commandCodeQueries > 0; }, pollTimeout));
    const int afterInit = commandCodeQueries;

    NvidiaEventHandler::handleEvent(defaultEid, buildRediscoveryEvent());

    EXPECT_TRUE(pumpIoUntil([&] { return commandCodeQueries > afterInit; },
                            pollTimeout));
}

// Destructor

TEST_F(NvidiaGpuDeviceTest, DestructorRemovesInterfaces)
{
    const std::string name = "gpudev_dtor";
    {
        const std::shared_ptr<GpuDevice> device = createDevice(name);
        ASSERT_NE(device, nullptr);
        // The DRAM Item.Dimm interface is published by the constructor.
        EXPECT_NO_THROW(
            getProperty<std::string>(dramPath(name), dimmIface, "MemoryType"));
    }
    drainPendingAsync();
    EXPECT_THROW(
        getProperty<std::string>(dramPath(name), dimmIface, "MemoryType"),
        sdbusplus::exception_t);
}

} // namespace
