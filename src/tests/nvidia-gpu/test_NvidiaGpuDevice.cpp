/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "MockMctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaSensorConfig.hpp"
#include "OcpMctpVdm.hpp"

#include <sdbusplus/exception.hpp>

#include <chrono>
#include <cstdint>
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

constexpr const char* portIface =
    "xyz.openbmc_project.Inventory.Connector.Port";

// How many NVLink ports the mocked device reports as available.
constexpr uint8_t nvLinkPortCount = 2;

// Short enough that a second poll round lands well inside pollTimeout.
constexpr uint64_t fastPollMs = 10;

// Upper bound on how long the read loop may take to poll again.
constexpr std::chrono::seconds pollTimeout{5};

// Several fastPollMs intervals, so a loop that kept running would be caught.
constexpr std::chrono::seconds quietWindow{1};

// Query Ports Available (Nvidia MCTP VDM 0x41): the number of NVLink ports.
std::vector<uint8_t> buildPortsAvailableResponse(uint8_t numberNvPorts)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize +
                             sizeof(numberNvPorts));
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(
        static_cast<uint8_t>(gpu::NetworkPortCommands::QueryPortsAvailable));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(sizeof(numberNvPorts)));
    pack.pack(numberNvPorts);
    return buf;
}

// True when the request is the NETWORK_PORT Query Ports Available command.
bool isQueryPortsAvailable(std::span<const uint8_t> request)
{
    UnpackBuffer unpack(request);
    ocp::accelerator_management::MessageType ocpMsgType{};
    uint8_t instanceId = 0;
    uint8_t msgType = 0;
    if (ocp::accelerator_management::unpackHeader(
            unpack, gpu::nvidiaPciVendorId, ocpMsgType, instanceId, msgType) !=
        0)
    {
        return false;
    }
    uint8_t command = 0;
    unpack.unpack(command);
    return msgType == static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT) &&
           command == static_cast<uint8_t>(
                          gpu::NetworkPortCommands::QueryPortsAvailable);
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

    static std::string nvLinkPortPath(const std::string& name,
                                      uint8_t portIndex)
    {
        return "/xyz/openbmc_project/inventory/" + name + "/NVLink_" +
               std::to_string(portIndex);
    }

    // Report nvLinkPortCount NVLink ports, and complete every other request
    // with an empty response so the rest of init() still makes progress.
    void expectNvLinkPortCount()
    {
        EXPECT_CALL(mctpMock, sendRecvMsg)
            .Times(testing::AtLeast(1))
            .WillRepeatedly([](uint8_t /*eid*/, std::span<const uint8_t> reqMsg,
                               auto callback) {
                if (isQueryPortsAvailable(reqMsg))
                {
                    const std::vector<uint8_t> response =
                        buildPortsAvailableResponse(nvLinkPortCount);
                    callback(std::error_code{}, response);
                    return;
                }
                callback(std::error_code{}, std::span<const uint8_t>{});
            });
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

// NVLink ports

TEST_F(NvidiaGpuDeviceTest, InitPublishesNvLinkPortObjects)
{
    expectNvLinkPortCount();

    const std::string name = "gpudev_nvlink";
    const std::shared_ptr<GpuDevice> device = createDevice(name);
    device->init();

    // The device owns the port object: the properties no telemetry command
    // reports are published with the port, seeded with their defaults.
    for (uint8_t i = 0; i < nvLinkPortCount; ++i)
    {
        const std::string path = nvLinkPortPath(name, i);

        EXPECT_EQ(getProperty<std::string>(path, portIface, "PortProtocol"),
                  "xyz.openbmc_project.Inventory.Connector.Port.PortProtocol."
                  "NVLink");
        EXPECT_EQ(getProperty<std::string>(path, portIface, "PortType"),
                  "xyz.openbmc_project.Inventory.Connector.Port.PortType."
                  "Bidirectional");
        EXPECT_EQ(
            getProperty<std::string>(path, portIface, "LinkStatus"),
            "xyz.openbmc_project.Inventory.Connector.Port.LinkStatus.NoLink");
        EXPECT_EQ(
            getProperty<std::string>(path, portIface, "LinkState"),
            "xyz.openbmc_project.Inventory.Connector.Port.LinkState.Unknown");
    }
}

// Destructor

TEST_F(NvidiaGpuDeviceTest, DestructorRemovesNvLinkPortInterfaces)
{
    const std::string name = "gpudev_nvlink_dtor";
    {
        expectNvLinkPortCount();

        const std::shared_ptr<GpuDevice> device = createDevice(name);
        device->init();
        EXPECT_NO_THROW(getProperty<std::string>(nvLinkPortPath(name, 0),
                                                 portIface, "PortProtocol"));
    }
    drainPendingAsync();
    EXPECT_THROW(getProperty<std::string>(nvLinkPortPath(name, 0), portIface,
                                          "PortProtocol"),
                 sdbusplus::exception_t);
}

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
