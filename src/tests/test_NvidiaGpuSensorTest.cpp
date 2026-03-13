/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MessagePackUnpackUtils.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace ocp_mctp_tests
{

class OcpMctpVdmTests : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        // Initialize common test data here
    }
};

// Tests for OcpMctpVdm::packHeader function (PackBuffer-based)
TEST_F(OcpMctpVdmTests, PackHeaderRequestSuccess)
{
    const uint16_t pciVendorId = 0x1234;
    std::array<uint8_t, 16> buf{};
    PackBuffer pbuf(buf);

    int result = ocp::accelerator_management::packHeader(
        pbuf, pciVendorId, ocp::accelerator_management::MessageType::REQUEST, 5,
        0x7E);
    EXPECT_EQ(result, 0);
    EXPECT_EQ(pbuf.getError(), 0);

    // Verify with unpackHeader
    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t instanceId = 0;
    uint8_t msgTypeField = 0;
    int rc = ocp::accelerator_management::unpackHeader(
        ubuf, pciVendorId, msgType, instanceId, msgTypeField);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(instanceId, 5);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(msgTypeField, 0x7E);
}

TEST_F(OcpMctpVdmTests, PackHeaderResponseSuccess)
{
    const uint16_t pciVendorId = 0x1234;
    std::array<uint8_t, 16> buf{};
    PackBuffer pbuf(buf);

    int result = ocp::accelerator_management::packHeader(
        pbuf, pciVendorId, ocp::accelerator_management::MessageType::RESPONSE,
        10, 0x7E);
    EXPECT_EQ(result, 0);
    EXPECT_EQ(pbuf.getError(), 0);

    // Verify with unpackHeader
    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t instanceId = 0;
    uint8_t msgTypeField = 0;
    int rc = ocp::accelerator_management::unpackHeader(
        ubuf, pciVendorId, msgType, instanceId, msgTypeField);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(instanceId, 10);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::RESPONSE);
    EXPECT_EQ(msgTypeField, 0x7E);
}

TEST_F(OcpMctpVdmTests, PackHeaderInvalidInstanceId)
{
    const uint16_t pciVendorId = 0x1234;
    std::array<uint8_t, 16> buf{};
    PackBuffer pbuf(buf);

    int result = ocp::accelerator_management::packHeader(
        pbuf, pciVendorId, ocp::accelerator_management::MessageType::REQUEST,
        32, 0x7E);

    EXPECT_EQ(result, EINVAL);
}

// Tests for OcpMctpVdm::unpackReasonCodeAndCC function
TEST_F(OcpMctpVdmTests, UnpackReasonCodeAndCCSuccessCase)
{
    std::array<uint8_t, 16> buf{};
    PackBuffer pbuf(buf);
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(static_cast<uint16_t>(0x1234));
    ASSERT_EQ(pbuf.getError(), 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = ocp::accelerator_management::unpackReasonCodeAndCC(
        ubuf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0); // Should be 0 for SUCCESS
}

TEST_F(OcpMctpVdmTests, UnpackReasonCodeAndCCErrorCase)
{
    std::array<uint8_t, 16> buf{};
    PackBuffer pbuf(buf);
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR));
    pbuf.pack(static_cast<uint16_t>(0x5678));
    ASSERT_EQ(pbuf.getError(), 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = ocp::accelerator_management::unpackReasonCodeAndCC(
        ubuf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x5678);
}

// Helper: pack a BindingPciVid header into a PackBuffer for response messages
void packResponseHeader(PackBuffer& buffer, uint8_t msgType)
{
    // pci_vendor_id (2 bytes, big-endian)
    buffer.pack(static_cast<uint16_t>(htobe16(0x10de)));
    // instance_id: response bit clear, instance=0
    buffer.pack(static_cast<uint8_t>(0x00));
    // ocp_version: type=8 (upper nibble), version=9 (lower nibble)
    buffer.pack(static_cast<uint8_t>(0x89));
    // ocp_accelerator_management_msg_type
    buffer.pack(msgType);
}

// Tests for ocp::accelerator_management::decodeEvent

TEST_F(OcpMctpVdmTests, DecodeEventSuccess)
{
    // Event struct: BindingPciVid(5) + version(1) + eventId(1) +
    //              eventClass(1) + eventState(2) + size(1) = 11 bytes
    std::vector<uint8_t> buf(11);
    PackBuffer packer(buf);

    // Pack BindingPciVid header
    packResponseHeader(packer, 0x03);

    // version: ack not required (bit 4 = 0), version = 1
    packer.pack(static_cast<uint8_t>(0x01));
    // eventId
    packer.pack(static_cast<uint8_t>(0x42));
    // eventClass
    packer.pack(static_cast<uint8_t>(0x03));
    // eventState (little-endian)
    packer.pack(static_cast<uint16_t>(0xBEEF));
    // size
    packer.pack(static_cast<uint8_t>(0));

    ASSERT_EQ(packer.getError(), 0);

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(messageType, 0x03);
    EXPECT_FALSE(ackRequired);
    EXPECT_EQ(version, 1);
    EXPECT_EQ(eventId, 0x42);
    EXPECT_EQ(eventClass, 0x03);
    EXPECT_EQ(eventState, 0xBEEF);
    EXPECT_EQ(size, 0);
    EXPECT_TRUE(eventData.empty());
}

TEST_F(OcpMctpVdmTests, DecodeEventWithEventData)
{
    // 11 bytes header + 4 bytes event data
    std::vector<uint8_t> buf(15);
    PackBuffer packer(buf);

    packResponseHeader(packer, 0x03);

    packer.pack(static_cast<uint8_t>(0x02));    // version=2, no ack
    packer.pack(static_cast<uint8_t>(0x10));    // eventId
    packer.pack(static_cast<uint8_t>(0x05));    // eventClass
    packer.pack(static_cast<uint16_t>(0x1234)); // eventState
    packer.pack(static_cast<uint8_t>(4));       // size = 4 bytes of event data

    ASSERT_EQ(packer.getError(), 0);

    // Fill event data bytes
    buf[11] = 0xAA;
    buf[12] = 0xBB;
    buf[13] = 0xCC;
    buf[14] = 0xDD;

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(messageType, 0x03);
    EXPECT_FALSE(ackRequired);
    EXPECT_EQ(version, 2);
    EXPECT_EQ(eventId, 0x10);
    EXPECT_EQ(eventClass, 0x05);
    EXPECT_EQ(eventState, 0x1234);
    EXPECT_EQ(size, 4);
    ASSERT_EQ(eventData.size(), 4);
    EXPECT_EQ(eventData[0], 0xAA);
    EXPECT_EQ(eventData[1], 0xBB);
    EXPECT_EQ(eventData[2], 0xCC);
    EXPECT_EQ(eventData[3], 0xDD);
}

TEST_F(OcpMctpVdmTests, DecodeEventAckRequired)
{
    std::vector<uint8_t> buf(11);
    PackBuffer packer(buf);

    packResponseHeader(packer, 0x03);

    // version byte: bit 4 set (ack required) + version=1 => 0x11
    packer.pack(static_cast<uint8_t>(0x11));
    packer.pack(static_cast<uint8_t>(0x01));    // eventId
    packer.pack(static_cast<uint8_t>(0x02));    // eventClass
    packer.pack(static_cast<uint16_t>(0x0001)); // eventState
    packer.pack(static_cast<uint8_t>(0));       // size

    ASSERT_EQ(packer.getError(), 0);

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, 0);
    EXPECT_TRUE(ackRequired);
    EXPECT_EQ(version, 1);
}

TEST_F(OcpMctpVdmTests, DecodeEventAckNotRequired)
{
    std::vector<uint8_t> buf(11);
    PackBuffer packer(buf);

    packResponseHeader(packer, 0x03);

    // version byte: bit 4 clear, version=3 => 0x03
    packer.pack(static_cast<uint8_t>(0x03));
    packer.pack(static_cast<uint8_t>(0x01));    // eventId
    packer.pack(static_cast<uint8_t>(0x02));    // eventClass
    packer.pack(static_cast<uint16_t>(0x0001)); // eventState
    packer.pack(static_cast<uint8_t>(0));       // size

    ASSERT_EQ(packer.getError(), 0);

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, 0);
    EXPECT_FALSE(ackRequired);
    EXPECT_EQ(version, 3);
}

TEST_F(OcpMctpVdmTests, DecodeEventBufferTooSmallForHeader)
{
    // Event header is 11 bytes; provide only 6
    std::vector<uint8_t> buf(6, 0);

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(OcpMctpVdmTests, DecodeEventBufferTooSmallForEventData)
{
    // 11 byte header, size says 5 bytes of data, but no trailing data
    std::vector<uint8_t> buf(11);
    PackBuffer packer(buf);

    packResponseHeader(packer, 0x03);

    packer.pack(static_cast<uint8_t>(0x01));    // version
    packer.pack(static_cast<uint8_t>(0x01));    // eventId
    packer.pack(static_cast<uint8_t>(0x02));    // eventClass
    packer.pack(static_cast<uint16_t>(0x0001)); // eventState
    packer.pack(static_cast<uint8_t>(5));       // size = 5, but no data follows

    ASSERT_EQ(packer.getError(), 0);

    uint8_t messageType{};
    bool ackRequired{};
    uint8_t version{};
    uint8_t eventId{};
    uint8_t eventClass{};
    uint16_t eventState{};
    uint8_t size{};
    std::span<const uint8_t> eventData;

    int result = ocp::accelerator_management::decodeEvent(
        buf, gpu::nvidiaPciVendorId, messageType, ackRequired, version, eventId,
        eventClass, eventState, size, eventData);

    EXPECT_EQ(result, EINVAL);
}

} // namespace ocp_mctp_tests

namespace gpu_mctp_tests
{

class GpuMctpVdmTests : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        // Initialize common test data here
    }
};

// Tests for GpuMctpVdm::encodeQueryDeviceIdentificationRequest function
TEST_F(GpuMctpVdmTests, EncodeQueryDeviceIdentificationRequestSuccess)
{
    const uint8_t instanceId = 3;
    std::vector<uint8_t> buf(256);

    int result = gpu::encodeQueryDeviceIdentificationRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(
        unpackedMsgType,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeQueryDeviceIdentificationResponse function
TEST_F(GpuMctpVdmTests, DecodeQueryDeviceIdentificationResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 3,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pbuf.pack(static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0));         // reserved
    pbuf.pack(static_cast<uint16_t>(2));         // data_size
    pbuf.pack(static_cast<uint8_t>(
        gpu::DeviceIdentification::DEVICE_GPU)); // device_identification
    pbuf.pack(static_cast<uint8_t>(7));          // instance_id
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t deviceIdentification{};
    uint8_t deviceInstance{};

    int result = gpu::decodeQueryDeviceIdentificationResponse(
        buf, cc, reasonCode, deviceIdentification, deviceInstance);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(deviceIdentification,
              static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU));
    EXPECT_EQ(deviceInstance, 7);
}

TEST_F(GpuMctpVdmTests, DecodeQueryDeviceIdentificationResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 3,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    pbuf.pack(static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR));         // CC
    pbuf.pack(static_cast<uint16_t>(0x1234)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t deviceIdentification{};
    uint8_t deviceInstance{};

    int result = gpu::decodeQueryDeviceIdentificationResponse(
        buf, cc, reasonCode, deviceIdentification, deviceInstance);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeQueryDeviceIdentificationResponseInvalidSize)
{
    std::vector<uint8_t> buf(7);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 3,
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t deviceIdentification{};
    uint8_t deviceInstance{};

    int result = gpu::decodeQueryDeviceIdentificationResponse(
        buf, cc, reasonCode, deviceIdentification, deviceInstance);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid size
}

// Tests for GpuMctpVdm::encodeGetTemperatureReadingRequest function
TEST_F(GpuMctpVdmTests, EncodeGetTemperatureReadingRequestSuccess)
{
    const uint8_t instanceId = 4;
    const uint8_t sensorId = 0;
    std::vector<uint8_t> buf(256);

    int result =
        gpu::encodeGetTemperatureReadingRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(sensorId));
    uint8_t unpackedSensorId = 0;
    ubuf.unpack(unpackedSensorId);
    EXPECT_EQ(unpackedSensorId, sensorId);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeGetTemperatureReadingResponse function
TEST_F(GpuMctpVdmTests, DecodeGetTemperatureReadingResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 4,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_TEMPERATURE_READING)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));   // CC
    pbuf.pack(static_cast<uint16_t>(0));                          // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(int32_t)));            // data_size
    // Set a temperature value of 75.5C (75.5 * 256 = 19328)
    pbuf.pack(static_cast<int32_t>(19328));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    double temperatureReading{};

    int result = gpu::decodeGetTemperatureReadingResponse(
        buf, cc, reasonCode, temperatureReading);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_NEAR(temperatureReading, 75.5, 0.01);
}

TEST_F(GpuMctpVdmTests, DecodeGetTemperatureReadingResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 3,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_TEMPERATURE_READING));     // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x4321)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    double temperatureReading{};

    int result = gpu::decodeGetTemperatureReadingResponse(
        buf, cc, reasonCode, temperatureReading);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x4321);
}

TEST_F(GpuMctpVdmTests, DecodeGetTemperatureReadingResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 4,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_TEMPERATURE_READING)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));   // CC
    pbuf.pack(static_cast<uint16_t>(0));                          // reserved
    pbuf.pack(static_cast<uint16_t>(1)); // Invalid data_size
    pbuf.pack(static_cast<int32_t>(19328));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    double temperatureReading{};

    int result = gpu::decodeGetTemperatureReadingResponse(
        buf, cc, reasonCode, temperatureReading);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeReadThermalParametersRequest function
TEST_F(GpuMctpVdmTests, EncodeReadThermalParametersRequestSuccess)
{
    const uint8_t instanceId = 5;
    const uint8_t sensorId = 1;
    std::array<uint8_t, gpu::readThermalParametersRequestSize> buf{};

    int result =
        gpu::encodeReadThermalParametersRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(sensorId));
    uint8_t unpackedSensorId = 0;
    ubuf.unpack(unpackedSensorId);
    EXPECT_EQ(unpackedSensorId, sensorId);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeReadThermalParametersResponse function
TEST_F(GpuMctpVdmTests, DecodeReadThermalParametersResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       READ_THERMAL_PARAMETERS)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));   // CC
    pbuf.pack(static_cast<uint16_t>(0));                          // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(int32_t)));            // data_size
    // Set a threshold value of 85C (85 * 256 = 21760)
    pbuf.pack(static_cast<int32_t>(21760));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    int32_t threshold{};

    int result = gpu::decodeReadThermalParametersResponse(
        buf, cc, reasonCode, threshold);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(threshold, 21760);
}

TEST_F(GpuMctpVdmTests, DecodeReadThermalParametersResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       READ_THERMAL_PARAMETERS));     // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x5678)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    int32_t threshold{};

    int result = gpu::decodeReadThermalParametersResponse(
        buf, cc, reasonCode, threshold);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeReadThermalParametersResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       READ_THERMAL_PARAMETERS)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));   // CC
    pbuf.pack(static_cast<uint16_t>(0));                          // reserved
    pbuf.pack(static_cast<uint16_t>(2)); // Invalid data_size
    pbuf.pack(static_cast<int32_t>(21760));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    int32_t threshold{};

    int result = gpu::decodeReadThermalParametersResponse(
        buf, cc, reasonCode, threshold);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeGetCurrentPowerDrawRequest function
TEST_F(GpuMctpVdmTests, EncodeGetCurrentPowerDrawRequestSuccess)
{
    const uint8_t instanceId = 6;
    const uint8_t sensorId = 2;
    const uint8_t averagingInterval = 10;
    gpu::PlatformEnvironmentalCommands commandCode =
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW;
    std::array<uint8_t, gpu::getPowerDrawRequestSize> buf{};

    int result = gpu::encodeGetPowerDrawRequest(
        commandCode, instanceId, sensorId, averagingInterval, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(commandCode));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(sensorId) + sizeof(averagingInterval));
    uint8_t unpackedSensorId = 0;
    ubuf.unpack(unpackedSensorId);
    EXPECT_EQ(unpackedSensorId, sensorId);
    uint8_t unpackedAvgInterval = 0;
    ubuf.unpack(unpackedAvgInterval);
    EXPECT_EQ(unpackedAvgInterval, averagingInterval);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeGetCurrentPowerDrawResponse function
TEST_F(GpuMctpVdmTests, DecodeGetCurrentPowerDrawResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 6,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0));                // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(uint32_t))); // data_size
    pbuf.pack(static_cast<uint32_t>(250));              // power = 250W
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t power{};

    int result = gpu::decodeGetPowerDrawResponse(buf, cc, reasonCode, power);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(power, 250U);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentPowerDrawResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 6,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x9ABC)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t power{};

    int result = gpu::decodeGetPowerDrawResponse(buf, cc, reasonCode, power);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x9ABC);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentPowerDrawResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 6,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0)); // reserved
    pbuf.pack(static_cast<uint16_t>(2)); // Invalid data_size
    pbuf.pack(static_cast<uint32_t>(250));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t power{};

    int result = gpu::decodeGetPowerDrawResponse(buf, cc, reasonCode, power);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeGetCurrentEnergyCounterRequest function
TEST_F(GpuMctpVdmTests, EncodeGetCurrentEnergyCounterRequestSuccess)
{
    const uint8_t instanceId = 7;
    const uint8_t sensorId = 3;
    std::array<uint8_t, gpu::getCurrentEnergyCounterRequestSize> buf{};

    int result =
        gpu::encodeGetCurrentEnergyCounterRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(sensorId));
    uint8_t unpackedSensorId = 0;
    ubuf.unpack(unpackedSensorId);
    EXPECT_EQ(unpackedSensorId, sensorId);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeGetCurrentEnergyCounterResponse function
TEST_F(GpuMctpVdmTests, DecodeGetCurrentEnergyCounterResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 7,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_CURRENT_ENERGY_COUNTER)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));      // CC
    pbuf.pack(static_cast<uint16_t>(0));                             // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(uint64_t))); // data_size
    pbuf.pack(static_cast<uint64_t>(3600000));          // energy
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint64_t energy{};

    int result =
        gpu::decodeGetCurrentEnergyCounterResponse(buf, cc, reasonCode, energy);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(energy, 3600000U);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentEnergyCounterResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 7,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_CURRENT_ENERGY_COUNTER));  // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0xDEF0)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint64_t energy{};

    int result =
        gpu::decodeGetCurrentEnergyCounterResponse(buf, cc, reasonCode, energy);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0xDEF0);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentEnergyCounterResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 7,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::
                                       GET_CURRENT_ENERGY_COUNTER)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));      // CC
    pbuf.pack(static_cast<uint16_t>(0));                             // reserved
    pbuf.pack(static_cast<uint16_t>(4)); // Invalid - should be sizeof(uint64_t)
    pbuf.pack(static_cast<uint64_t>(3600000));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint64_t energy{};

    int result =
        gpu::decodeGetCurrentEnergyCounterResponse(buf, cc, reasonCode, energy);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeGetDriverInformationRequest function
TEST_F(GpuMctpVdmTests, EncodeGetDriverInformationRequestSuccess)
{
    const uint8_t instanceId = 9;
    std::array<uint8_t, ocp::accelerator_management::commonRequestSize> buf{};

    int result = gpu::encodeGetDriverInformationRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetDriverInformationRequestBufferTooSmall)
{
    const uint8_t instanceId = 9;
    std::array<uint8_t, 1> buf{}; // Too small buffer

    int result = gpu::encodeGetDriverInformationRequest(instanceId, buf);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeGetDriverInformationResponse function
TEST_F(GpuMctpVdmTests, DecodeGetDriverInformationResponseSuccess)
{
    const std::string expectedVersion = "535.104.05";
    const size_t dataSize =
        sizeof(gpu::DriverState) + expectedVersion.size() + 1;
    // Size the buffer exactly to match what decoder expects
    const size_t bufSize =
        gpu::getDriverInformationResponseMinSize + expectedVersion.size();
    std::vector<uint8_t> buf(bufSize);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 9,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0));         // reserved
    pbuf.pack(static_cast<uint16_t>(dataSize));  // data_size
    pbuf.pack(static_cast<uint8_t>(
        gpu::DriverState::DRIVER_STATE_LOADED)); // driverState
    // Pack driver version string including null terminator
    for (size_t i = 0; i <= expectedVersion.size(); i++)
    {
        pbuf.pack(static_cast<uint8_t>(expectedVersion.c_str()[i]));
    }
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::DriverState driverState{};
    std::string driverVersion{};

    int result = gpu::decodeGetDriverInformationResponse(
        buf, cc, reasonCode, driverState, driverVersion);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(driverState, gpu::DriverState::DRIVER_STATE_LOADED);
    EXPECT_EQ(driverVersion, expectedVersion);
}

TEST_F(GpuMctpVdmTests, DecodeGetDriverInformationResponseDriverNotLoaded)
{
    const size_t dataSize =
        sizeof(gpu::DriverState) + 1; // Minimum version size
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 9,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0));             // reserved
    pbuf.pack(static_cast<uint16_t>(dataSize));      // data_size
    pbuf.pack(static_cast<uint8_t>(
        gpu::DriverState::DRIVER_STATE_NOT_LOADED)); // driverState
    pbuf.pack(static_cast<uint8_t>('\0'));           // null terminator
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::DriverState driverState{};
    std::string driverVersion{};

    int result = gpu::decodeGetDriverInformationResponse(
        buf, cc, reasonCode, driverState, driverVersion);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(driverState, gpu::DriverState::DRIVER_STATE_NOT_LOADED);
}

TEST_F(GpuMctpVdmTests, DecodeGetDriverInformationResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 9,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0xABCD)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::DriverState driverState{};
    std::string driverVersion{};

    int result = gpu::decodeGetDriverInformationResponse(
        buf, cc, reasonCode, driverState, driverVersion);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0xABCD);
}

TEST_F(GpuMctpVdmTests, DecodeGetDriverInformationResponseInvalidSize)
{
    // Create a buffer that's too small - just the header + CC fields
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 9,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0)); // reserved
    pbuf.pack(static_cast<uint16_t>(2)); // Valid data size but buffer too small
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::DriverState driverState{};
    std::string driverVersion{};

    int result = gpu::decodeGetDriverInformationResponse(
        buf, cc, reasonCode, driverState, driverVersion);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid size
}

TEST_F(GpuMctpVdmTests, DecodeGetDriverInformationResponseInvalidDataSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 9,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));       // CC
    pbuf.pack(static_cast<uint16_t>(0)); // reserved
    // data_size = 1 is invalid (needs at least sizeof(DriverState) + 1 = 2)
    pbuf.pack(static_cast<uint16_t>(1)); // data_size
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::DriverState driverState{};
    std::string driverVersion{};

    int result = gpu::decodeGetDriverInformationResponse(
        buf, cc, reasonCode, driverState, driverVersion);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeGetVoltageRequest function
TEST_F(GpuMctpVdmTests, EncodeGetVoltageRequestSuccess)
{
    const uint8_t instanceId = 8;
    const uint8_t sensorId = 4;
    std::array<uint8_t, gpu::getVoltageRequestSize> buf{};

    int result = gpu::encodeGetVoltageRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(sensorId));
    uint8_t unpackedSensorId = 0;
    ubuf.unpack(unpackedSensorId);
    EXPECT_EQ(unpackedSensorId, sensorId);
    EXPECT_EQ(ubuf.getError(), 0);
}

// Tests for GpuMctpVdm::decodeGetVoltageResponse function
TEST_F(GpuMctpVdmTests, DecodeGetVoltageResponseSuccess)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 8,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));      // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // CC
    pbuf.pack(static_cast<uint16_t>(0));                        // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(uint32_t)));         // data_size
    pbuf.pack(static_cast<uint32_t>(12500)); // voltage = 12.5V
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t voltage{};

    int result = gpu::decodeGetVoltageResponse(buf, cc, reasonCode, voltage);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(voltage, 12500U);
}

TEST_F(GpuMctpVdmTests, DecodeGetVoltageResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 8,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));            // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x1234)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t voltage{};

    int result = gpu::decodeGetVoltageResponse(buf, cc, reasonCode, voltage);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeGetVoltageResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 8,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));      // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // CC
    pbuf.pack(static_cast<uint16_t>(0));                        // reserved
    pbuf.pack(static_cast<uint16_t>(2)); // Invalid data_size
    pbuf.pack(static_cast<uint32_t>(12500));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t voltage{};

    int result = gpu::decodeGetVoltageResponse(buf, cc, reasonCode, voltage);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

// Tests for GpuMctpVdm::encodeGetPowerLimitsRequest function
TEST_F(GpuMctpVdmTests, EncodeGetPowerLimitsRequestSuccess)
{
    const uint8_t instanceId = 1;
    const uint32_t powerLimitId = 0;
    std::array<uint8_t, gpu::getPowerLimitsRequestSize> buf{};

    int result =
        gpu::encodeGetPowerLimitsRequest(instanceId, powerLimitId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(powerLimitId));
    uint32_t unpackedPowerLimitId = 0;
    ubuf.unpack(unpackedPowerLimitId);
    EXPECT_EQ(unpackedPowerLimitId, powerLimitId);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetPowerLimitsRequestBufferTooSmall)
{
    const uint8_t instanceId = 1;
    const uint32_t powerLimitId = 0;
    std::array<uint8_t, 1> buf{};

    int result =
        gpu::encodeGetPowerLimitsRequest(instanceId, powerLimitId, buf);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeGetPowerLimitsResponse function
TEST_F(GpuMctpVdmTests, DecodeGetPowerLimitsResponseSuccess)
{
    const uint32_t expectedPersistent = 300;
    const uint32_t expectedOneshot = 400;
    const uint32_t expectedEnforced = 250;

    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // CC
    pbuf.pack(static_cast<uint16_t>(0));                        // reserved
    pbuf.pack(static_cast<uint16_t>(sizeof(uint32_t) * 3));     // data_size
    pbuf.pack(expectedPersistent);
    pbuf.pack(expectedOneshot);
    pbuf.pack(expectedEnforced);
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t persistentPowerLimit{};
    uint32_t oneshotPowerLimit{};
    uint32_t powerLimitEnforced{};

    int result = gpu::decodeGetPowerLimitsResponse(
        buf, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        powerLimitEnforced);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(persistentPowerLimit, expectedPersistent);
    EXPECT_EQ(oneshotPowerLimit, expectedOneshot);
    EXPECT_EQ(powerLimitEnforced, expectedEnforced);
}

TEST_F(GpuMctpVdmTests, DecodeGetPowerLimitsResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS));       // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x5678)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t persistentPowerLimit{};
    uint32_t oneshotPowerLimit{};
    uint32_t powerLimitEnforced{};

    int result = gpu::decodeGetPowerLimitsResponse(
        buf, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        powerLimitEnforced);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeGetPowerLimitsResponseBufferTooSmall)
{
    std::array<uint8_t, 4> buf{};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t persistentPowerLimit{};
    uint32_t oneshotPowerLimit{};
    uint32_t powerLimitEnforced{};

    int result = gpu::decodeGetPowerLimitsResponse(
        buf, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        powerLimitEnforced);

    EXPECT_NE(result, 0);
}

TEST_F(GpuMctpVdmTests, DecodeGetPowerLimitsResponseInvalidDataSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_POWER_LIMITS)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // CC
    pbuf.pack(static_cast<uint16_t>(0));                        // reserved
    pbuf.pack(static_cast<uint16_t>(4)); // data_size (wrong)
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t persistentPowerLimit{};
    uint32_t oneshotPowerLimit{};
    uint32_t powerLimitEnforced{};

    int result = gpu::decodeGetPowerLimitsResponse(
        buf, cc, reasonCode, persistentPowerLimit, oneshotPowerLimit,
        powerLimitEnforced);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, EncodeQueryScalarGroupTelemetryV1RequestSuccess)
{
    const uint8_t instanceId = 10;
    const uint8_t deviceIndex = 0;
    const gpu::PcieScalarGroupId groupId =
        gpu::PcieScalarGroupId::LinkSpeedWidth;
    std::array<uint8_t, gpu::queryScalarGroupTelemetryV1RequestSize> buf{};

    int result = gpu::encodeQueryScalarGroupTelemetryV1Request(
        instanceId, deviceIndex, groupId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 2);
    uint8_t unpackedDeviceIndex = 0;
    ubuf.unpack(unpackedDeviceIndex);
    EXPECT_EQ(unpackedDeviceIndex, deviceIndex);
    uint8_t unpackedGroupId = 0;
    ubuf.unpack(unpackedGroupId);
    EXPECT_EQ(unpackedGroupId, static_cast<uint8_t>(groupId));
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeQueryScalarGroupTelemetryV1RequestInvalidSize)
{
    std::array<uint8_t, 1> buf{};

    int result = gpu::encodeQueryScalarGroupTelemetryV1Request(
        0, 0, gpu::PcieScalarGroupId::LinkSpeedWidth, buf);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, EncodeQueryScalarGroupTelemetryV2RequestSuccess)
{
    const uint8_t instanceId = 10;
    const gpu::PciePortType portType = gpu::PciePortType::DOWNSTREAM;
    const uint8_t upstreamPortNumber = 3;
    const uint8_t portNumber = 5;
    const gpu::PcieScalarGroupId groupId =
        gpu::PcieScalarGroupId::AerErrorCounters;
    std::array<uint8_t, gpu::queryScalarGroupTelemetryV2RequestSize> buf{};

    int result = gpu::encodeQueryScalarGroupTelemetryV2Request(
        instanceId, portType, upstreamPortNumber, portNumber, groupId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 3);
    uint8_t unpackedUpstream = 0;
    ubuf.unpack(unpackedUpstream);
    EXPECT_EQ(unpackedUpstream, (static_cast<uint8_t>(portType) << 7) |
                                    (upstreamPortNumber & 0x7F));
    uint8_t unpackedPortNumber = 0;
    ubuf.unpack(unpackedPortNumber);
    EXPECT_EQ(unpackedPortNumber, portNumber);
    uint8_t unpackedGroupId = 0;
    ubuf.unpack(unpackedGroupId);
    EXPECT_EQ(unpackedGroupId, static_cast<uint8_t>(groupId));
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeQueryScalarGroupTelemetryV2RequestInvalidSize)
{
    std::array<uint8_t, 1> buf{};

    int result = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, {}, 0, 0, gpu::PcieScalarGroupId::LinkSpeedWidth, buf);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV1ResponseSuccess)
{
    const size_t numValues = 4;
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));       // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));     // CC
    pbuf.pack(static_cast<uint16_t>(0));                            // reserved
    pbuf.pack(static_cast<uint16_t>(numValues * sizeof(uint32_t))); // data_size
    for (size_t i = 0; i < numValues; ++i)
    {
        pbuf.pack(static_cast<uint32_t>((i + 1) * 100));
    }
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV1Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(numTelemetryValues, numValues);
    ASSERT_EQ(telemetryValues.size(), numValues);
    EXPECT_EQ(telemetryValues[0], 100U);
    EXPECT_EQ(telemetryValues[1], 200U);
    EXPECT_EQ(telemetryValues[2], 300U);
    EXPECT_EQ(telemetryValues[3], 400U);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV1ResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV1));         // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x5678)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV1Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV1ResponseInvalidSize)
{
    std::vector<uint8_t> buf(7);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV1Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV2ResponseSuccess)
{
    const size_t numValues = 4;
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));       // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));     // CC
    pbuf.pack(static_cast<uint16_t>(0));                            // reserved
    pbuf.pack(static_cast<uint16_t>(numValues * sizeof(uint32_t))); // data_size

    for (size_t i = 0; i < numValues; ++i)
    {
        pbuf.pack(static_cast<uint32_t>((i + 1) * 100));
    }
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV2Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(numTelemetryValues, numValues);
    ASSERT_EQ(telemetryValues.size(), numValues);
    EXPECT_EQ(telemetryValues[0], 100U);
    EXPECT_EQ(telemetryValues[1], 200U);
    EXPECT_EQ(telemetryValues[2], 300U);
    EXPECT_EQ(telemetryValues[3], 400U);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV2ResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));         // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0x5678)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV2Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV2ResponseInvalidSize)
{
    std::vector<uint8_t> buf(7);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 10,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    size_t numTelemetryValues{};
    std::vector<uint32_t> telemetryValues{};

    int result = gpu::decodeQueryScalarGroupTelemetryV2Response(
        buf, cc, reasonCode, numTelemetryValues, telemetryValues);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, EncodeListPciePortsRequestSuccess)
{
    const uint8_t instanceId = 11;
    std::array<uint8_t, ocp::accelerator_management::commonRequestSize> buf{};

    int result = gpu::encodeListPciePortsRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, DecodeListPciePortsResponseSuccess)
{
    const size_t totalUpstreamPorts = 2;
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 11,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));      // CC
    pbuf.pack(static_cast<uint16_t>(0));                             // reserved
    pbuf.pack(static_cast<uint16_t>(
        sizeof(uint16_t) +
        totalUpstreamPorts * 2 * sizeof(uint8_t))); // data_size
    pbuf.pack(static_cast<uint16_t>(totalUpstreamPorts));
    // Port 0: external, 4 downstream
    pbuf.pack(static_cast<uint8_t>(0)); // isInternal
    pbuf.pack(static_cast<uint8_t>(4)); // count
    // Port 1: internal, 2 downstream
    pbuf.pack(static_cast<uint8_t>(1)); // isInternal
    pbuf.pack(static_cast<uint8_t>(2)); // count
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t numUpstreamPorts{};
    std::vector<uint8_t> numDownstreamPorts{};

    int result = gpu::decodeListPciePortsResponse(
        buf, cc, reasonCode, numUpstreamPorts, numDownstreamPorts);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(numUpstreamPorts, 1);
    ASSERT_EQ(numDownstreamPorts.size(), 1);
    EXPECT_EQ(numDownstreamPorts[0], 4);
}

TEST_F(GpuMctpVdmTests, DecodeListPciePortsResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 11,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts));  // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0xBEEF)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t numUpstreamPorts{};
    std::vector<uint8_t> numDownstreamPorts{};

    int result = gpu::decodeListPciePortsResponse(
        buf, cc, reasonCode, numUpstreamPorts, numDownstreamPorts);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0xBEEF);
}

TEST_F(GpuMctpVdmTests, DecodeListPciePortsResponseInvalidSize)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 11,
        static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));
    pbuf.pack(
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));      // CC
    pbuf.pack(static_cast<uint16_t>(0));                             // reserved
    pbuf.pack(static_cast<uint16_t>(1)); // Invalid data_size
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t numUpstreamPorts{};
    std::vector<uint8_t> numDownstreamPorts{};

    int result = gpu::decodeListPciePortsResponse(
        buf, cc, reasonCode, numUpstreamPorts, numDownstreamPorts);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeAggregateResponse function
TEST_F(GpuMctpVdmTests, DecodeAggregateResponsePowerOf2LengthEncodingValid)
{
    // Test with power-of-2 length encoding and valid bit set
    std::vector<uint8_t> buf = {
        0x03, 0x00, // telemetryCount = 3 (little-endian)
        // Item 0: tag=0x01, tagInfo=0x05 (power-of-2, valueLength=2, valid=1)
        // length = 1 << 2 = 4 bytes
        0x01, 0x05, 0x11, 0x22, 0x33, 0x44,
        // Item 1: tag=0x02, tagInfo=0x07 (power-of-2, valueLength=3, valid=1)
        // length = 1 << 3 = 8 bytes
        0x02, 0x07, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
        // Item 2: tag=0x03, tagInfo=0x01 (power-of-2, valueLength=0, valid=1)
        // length = 1 << 0 = 1 byte
        0x03, 0x01, 0x55};

    UnpackBuffer buffer(buf);

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
        std::vector<std::vector<uint8_t>> values;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  UnpackBuffer& buffer) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        std::vector<uint8_t> value(length);
        for (uint8_t i = 0; i < length; ++i)
        {
            uint8_t byte = 0;
            int rc = buffer.unpack(byte);
            if (rc != 0)
            {
                return rc;
            }
            value[i] = byte;
        }
        handlerData.values.push_back(std::move(value));
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, 0);

    // Verify 3 items were processed
    EXPECT_EQ(handlerData.tags.size(), 3U);

    // Verify item 0: tag=0x01, length=4
    EXPECT_EQ(handlerData.tags[0], 0x01);
    EXPECT_EQ(handlerData.lengths[0], 4);
    EXPECT_EQ(handlerData.values[0].size(), 4U);
    EXPECT_EQ(handlerData.values[0][0], 0x11);
    EXPECT_EQ(handlerData.values[0][1], 0x22);
    EXPECT_EQ(handlerData.values[0][2], 0x33);
    EXPECT_EQ(handlerData.values[0][3], 0x44);

    // Verify item 1: tag=0x02, length=8
    EXPECT_EQ(handlerData.tags[1], 0x02);
    EXPECT_EQ(handlerData.lengths[1], 8);
    EXPECT_EQ(handlerData.values[1].size(), 8U);
    EXPECT_EQ(handlerData.values[1][0], 0xAA);
    EXPECT_EQ(handlerData.values[1][7], 0x11);

    // Verify item 2: tag=0x03, length=1
    EXPECT_EQ(handlerData.tags[2], 0x03);
    EXPECT_EQ(handlerData.lengths[2], 1);
    EXPECT_EQ(handlerData.values[2].size(), 1U);
    EXPECT_EQ(handlerData.values[2][0], 0x55);
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseByteLengthEncodingValid)
{
    // Test with byte length encoding and valid bit set
    std::vector<uint8_t> buf = {
        0x02, 0x00, // telemetryCount = 2 (little-endian)
        // Item 0: tag=0x10, tagInfo=0x85 (byte-length, valueLength=2, valid=1)
        // bit 7=1 (byte-length), bits 3-1=010 (valueLength=2), bit 0=1 (valid)
        // length = 2 bytes
        0x10, 0x85, 0xAA, 0xBB,
        // Item 1: tag=0x11, tagInfo=0x8B (byte-length, valueLength=5, valid=1)
        // bit 7=1 (byte-length), bits 3-1=101 (valueLength=5), bit 0=1 (valid)
        // length = 5 bytes
        0x11, 0x8B, 0x01, 0x02, 0x03, 0x04, 0x05};

    UnpackBuffer buffer(buf);

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
        std::vector<std::vector<uint8_t>> values;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  UnpackBuffer& buffer) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        std::vector<uint8_t> value(length);
        for (uint8_t i = 0; i < length; ++i)
        {
            uint8_t byte = 0;
            int rc = buffer.unpack(byte);
            if (rc != 0)
            {
                return rc;
            }
            value[i] = byte;
        }
        handlerData.values.push_back(std::move(value));
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, 0);

    // Verify 2 items were processed
    EXPECT_EQ(handlerData.tags.size(), 2U);

    // Verify item 0: tag=0x10, length=2
    EXPECT_EQ(handlerData.tags[0], 0x10);
    EXPECT_EQ(handlerData.lengths[0], 2);
    EXPECT_EQ(handlerData.values[0].size(), 2U);
    EXPECT_EQ(handlerData.values[0][0], 0xAA);
    EXPECT_EQ(handlerData.values[0][1], 0xBB);

    // Verify item 1: tag=0x11, length=5
    EXPECT_EQ(handlerData.tags[1], 0x11);
    EXPECT_EQ(handlerData.lengths[1], 5);
    EXPECT_EQ(handlerData.values[1].size(), 5U);
    EXPECT_EQ(handlerData.values[1][0], 0x01);
    EXPECT_EQ(handlerData.values[1][4], 0x05);
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseInvalidBitNotSet)
{
    // Test with valid bit NOT set - items should be skipped
    std::vector<uint8_t> buf = {
        0x03, 0x00, // telemetryCount = 3
        // Item 0: tag=0x01, tagInfo=0x05 (valid bit set)
        0x01, 0x05, 0x11, 0x22, 0x33, 0x44,
        // Item 1: tag=0x02, tagInfo=0x06 (valid bit NOT set - bit 0 = 0)
        // This should be skipped
        0x02, 0x06, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
        // Item 2: tag=0x03, tagInfo=0x01 (valid bit set)
        0x03, 0x01, 0x55};

    UnpackBuffer buffer(buf);

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  UnpackBuffer& buffer) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        buffer.skip(length);
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, 0);

    // Only 2 items should be processed (item 1 skipped because valid bit not
    // set)
    EXPECT_EQ(handlerData.tags.size(), 2U);
    EXPECT_EQ(handlerData.tags[0], 0x01);
    EXPECT_EQ(handlerData.tags[1], 0x03);
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseMixedEncodingAndValidity)
{
    // Test with mixed length encoding types and validity
    std::vector<uint8_t> buf = {
        0x04, 0x00, // telemetryCount = 4
        // Item 0: power-of-2, valid
        0x01, 0x05, 0x11, 0x22, 0x33, 0x44,
        // Item 1: byte-length, valid
        0x02, 0x83, 0xAA,
        // Item 2: power-of-2, invalid
        0x03, 0x04, 0xCC, 0xDD, 0xEE, 0xFF,
        // Item 3: byte-length, valid
        0x04, 0x89, 0x55, 0x66, 0x77, 0x88};

    UnpackBuffer buffer(buf);

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  UnpackBuffer& buffer) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        buffer.skip(length);
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, 0);

    // 3 items should be processed (item 2 skipped)
    EXPECT_EQ(handlerData.tags.size(), 3U);
    EXPECT_EQ(handlerData.tags[0], 0x01);
    EXPECT_EQ(handlerData.lengths[0], 4); // power-of-2: 1 << 2 = 4
    EXPECT_EQ(handlerData.tags[1], 0x02);
    EXPECT_EQ(handlerData.lengths[1], 1); // byte-length: 1
    EXPECT_EQ(handlerData.tags[2], 0x04);
    EXPECT_EQ(handlerData.lengths[2], 4); // byte-length: 4
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseHandlerError)
{
    // Test that handler returning error stops processing
    std::vector<uint8_t> buf = {
        0x02, 0x00, // telemetryCount = 2
                    // Item 0: tag=0x01, tagInfo=0x01 (power-of-2,
                    // valueLength=0, valid=1) length = 1 byte
        0x01, 0x01, 0xAA,
        // Item 1: tag=0x02, tagInfo=0x01
        0x02, 0x01, 0xBB};

    UnpackBuffer buffer(buf);

    int handlerCallCount = 0;
    auto handler = [&handlerCallCount](const uint8_t, const uint8_t length,
                                       UnpackBuffer& buffer) -> int {
        handlerCallCount++;
        buffer.skip(length);
        return ENOTSUP; // Return error on first call
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, ENOTSUP);
    EXPECT_EQ(handlerCallCount, 1); // Only first item processed before error
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseInvalidBufferSize)
{
    // Test with buffer too small - missing second byte of telemetryCount
    std::vector<uint8_t> buf = {
        0x01 // Truncated - missing second byte of telemetryCount
    };

    UnpackBuffer buffer(buf);

    auto handler = [](const uint8_t, const uint8_t, UnpackBuffer&) -> int {
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseEmptyItems)
{
    // Test with zero items
    std::vector<uint8_t> buf = {
        0x00, 0x00 // telemetryCount = 0
    };

    UnpackBuffer buffer(buf);

    int handlerCallCount = 0;
    auto handler = [&handlerCallCount](const uint8_t, const uint8_t,
                                       UnpackBuffer&) -> int {
        handlerCallCount++;
        return 0;
    };

    int result = ocp::accelerator_management::unpackAggregateResponse(
        buffer, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(handlerCallCount, 0); // No items to process
}

// Tests for GpuMctpVdm::encodeGetPortNetworkAddressesRequest function
TEST_F(GpuMctpVdmTests, EncodeGetPortNetworkAddressesRequestSuccess)
{
    const uint8_t instanceId = 12;
    const uint16_t portNumber = 5;
    std::array<uint8_t, gpu::getPortNetworkAddressesRequestSize> buf{};

    int result =
        gpu::encodeGetPortNetworkAddressesRequest(instanceId, portNumber, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command, static_cast<uint8_t>(
                           gpu::NetworkPortCommands::GetPortNetworkAddresses));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 2);
    uint16_t unpackedPortNumber = 0;
    ubuf.unpack(unpackedPortNumber);
    EXPECT_EQ(unpackedPortNumber, portNumber);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetPortNetworkAddressesRequestInvalidBufferSize)
{
    const uint8_t instanceId = 12;
    const uint16_t portNumber = 5;
    std::array<uint8_t, gpu::getPortNetworkAddressesRequestSize - 1> buf{};

    int result =
        gpu::encodeGetPortNetworkAddressesRequest(instanceId, portNumber, buf);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeGetPortNetworkAddressesResponse function
TEST_F(GpuMctpVdmTests, DecodeGetPortNetworkAddressesResponseSuccess)
{
    // Test with linkType and two MAC addresses
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x11, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x03, 0x00,                         // telemetryCount = 3
        // Item 0: tag=0x00, tagInfo=0x01 (power-of-2, valueLength=0, valid=1)
        // length = 1 << 0 = 1 byte
        0x00, 0x01, 0x00, // linkType = ETHERNET (0)
        // Item 1: tag=0x01, tagInfo=0x07 (power-of-2, valueLength=3, valid=1)
        // length = 1 << 3 = 8 bytes (uint64_t MAC address)
        0x01, 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        // Item 2: tag=0x02, tagInfo=0x07 (power-of-2, valueLength=3, valid=1)
        // length = 8 bytes (uint64_t MAC address)
        0x02, 0x07, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::NetworkPortLinkType linkType{};
    std::vector<std::pair<uint8_t, uint64_t>> addresses{};

    int result = gpu::decodeGetPortNetworkAddressesResponse(
        buf, cc, reasonCode, linkType, addresses);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(linkType, gpu::NetworkPortLinkType::ETHERNET);
    EXPECT_EQ(addresses.size(), 2U);

    // Verify first address (tag=0x01)
    EXPECT_EQ(addresses[0].first, 0x01);
    EXPECT_EQ(addresses[0].second, 0x0807060504030201ULL);

    // Verify second address (tag=0x02)
    EXPECT_EQ(addresses[1].first, 0x02);
    EXPECT_EQ(addresses[1].second, 0x11100F0E0D0C0B0AULL);
}

TEST_F(GpuMctpVdmTests, DecodeGetPortNetworkAddressesResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 12,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pbuf.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetPortNetworkAddresses));          // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0xABCD)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::NetworkPortLinkType linkType{};
    std::vector<std::pair<uint8_t, uint64_t>> addresses{};

    int result = gpu::decodeGetPortNetworkAddressesResponse(
        buf, cc, reasonCode, linkType, addresses);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0xABCD);
    EXPECT_EQ(addresses.size(), 0U); // Should be empty on error
}

TEST_F(GpuMctpVdmTests, DecodeGetPortNetworkAddressesResponseInvalidBufferSize)
{
    // Buffer too small - missing telemetryCount
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code
        0x01 // Truncated - missing second byte of telemetryCount
    };

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::NetworkPortLinkType linkType{};
    std::vector<std::pair<uint8_t, uint64_t>> addresses{};

    int result = gpu::decodeGetPortNetworkAddressesResponse(
        buf, cc, reasonCode, linkType, addresses);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::encodeGetEthernetPortTelemetryCountersRequest function
TEST_F(GpuMctpVdmTests, EncodeGetEthernetPortTelemetryCountersRequestSuccess)
{
    const uint8_t instanceId = 13;
    const uint16_t portNumber = 7;
    std::array<uint8_t, gpu::getEthernetPortTelemetryCountersRequestSize> buf{};

    int result = gpu::encodeGetEthernetPortTelemetryCountersRequest(
        instanceId, portNumber, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t unpackedInstanceId = 0;
    uint8_t unpackedMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, unpackedInstanceId,
                  unpackedMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(unpackedInstanceId,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(unpackedMsgType,
              static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 2);
    uint16_t unpackedPortNumber = 0;
    ubuf.unpack(unpackedPortNumber);
    EXPECT_EQ(unpackedPortNumber, portNumber);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests,
       EncodeGetEthernetPortTelemetryCountersRequestInvalidBufferSize)
{
    const uint8_t instanceId = 13;
    const uint16_t portNumber = 7;
    std::array<uint8_t, gpu::getEthernetPortTelemetryCountersRequestSize - 1>
        buf{};

    int result = gpu::encodeGetEthernetPortTelemetryCountersRequest(
        instanceId, portNumber, buf);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeGetEthernetPortTelemetryCountersResponse
// function
TEST_F(GpuMctpVdmTests, DecodeGetEthernetPortTelemetryCountersResponseSuccess)
{
    // Test with mixed uint32_t and uint64_t telemetry values
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x03, 0x00,                         // telemetryCount = 3
        // Item 0: tag=0x01, tagInfo=0x05 (power-of-2, valueLength=2, valid=1)
        // length = 1 << 2 = 4 bytes (uint32_t)
        0x01, 0x05, 0x64, 0x00, 0x00, 0x00, // value = 100
        // Item 1: tag=0x02, tagInfo=0x07 (power-of-2, valueLength=3, valid=1)
        // length = 1 << 3 = 8 bytes (uint64_t)
        0x02, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, // value = 1000
        // Item 2: tag=0x03, tagInfo=0x05 (power-of-2, valueLength=2, valid=1)
        // length = 4 bytes (uint32_t)
        0x03, 0x05, 0x10, 0x27, 0x00, 0x00 // value = 10000
    };

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues{};

    int result = gpu::decodeGetEthernetPortTelemetryCountersResponse(
        buf, cc, reasonCode, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(telemetryValues.size(), 3U);

    // Verify first value (tag=0x01, uint32_t)
    EXPECT_EQ(telemetryValues[0].first, 0x01);
    EXPECT_EQ(telemetryValues[0].second, 100U);

    // Verify second value (tag=0x02, uint64_t)
    EXPECT_EQ(telemetryValues[1].first, 0x02);
    EXPECT_EQ(telemetryValues[1].second, 1000U);

    // Verify third value (tag=0x03, uint32_t)
    EXPECT_EQ(telemetryValues[2].first, 0x03);
    EXPECT_EQ(telemetryValues[2].second, 10000U);
}

TEST_F(GpuMctpVdmTests,
       DecodeGetEthernetPortTelemetryCountersResponseSkipInvalidLengths)
{
    // Test that values with invalid lengths are skipped
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x03, 0x00,                         // telemetryCount = 3
        // Item 0: tag=0x01, tagInfo=0x05 (valid, length=4)
        0x01, 0x05, 0x64, 0x00, 0x00, 0x00,
        // Item 1: tag=0x02, tagInfo=0x03 (valid, length=2 - invalid, will be
        // skipped)
        0x02, 0x03, 0xAA, 0xBB,
        // Item 2: tag=0x03, tagInfo=0x07 (valid, length=8)
        0x03, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues{};

    int result = gpu::decodeGetEthernetPortTelemetryCountersResponse(
        buf, cc, reasonCode, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    // Only 2 items should be present (item 1 skipped due to invalid length)
    EXPECT_EQ(telemetryValues.size(), 2U);

    EXPECT_EQ(telemetryValues[0].first, 0x01);
    EXPECT_EQ(telemetryValues[0].second, 100U);

    EXPECT_EQ(telemetryValues[1].first, 0x03);
    EXPECT_EQ(telemetryValues[1].second, 1000U);
}

TEST_F(GpuMctpVdmTests, DecodeGetEthernetPortTelemetryCountersResponseError)
{
    std::vector<uint8_t> buf(64);
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 13,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pbuf.pack(static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters)); // command
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY)); // CC
    pbuf.pack(static_cast<uint16_t>(0xDEAD)); // reason_code
    ASSERT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues{};

    int result = gpu::decodeGetEthernetPortTelemetryCountersResponse(
        buf, cc, reasonCode, telemetryValues);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0xDEAD);
    EXPECT_EQ(telemetryValues.size(), 0U); // Should be empty on error
}

TEST_F(GpuMctpVdmTests,
       DecodeGetEthernetPortTelemetryCountersResponseInvalidBufferSize)
{
    // Buffer too small
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code
        0x01 // Truncated - missing second byte of telemetryCount
    };

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    std::vector<std::pair<uint8_t, uint64_t>> telemetryValues{};

    int result = gpu::decodeGetEthernetPortTelemetryCountersResponse(
        buf, cc, reasonCode, telemetryValues);

    EXPECT_EQ(result, EINVAL);
}

// Helper: pack a CommonNonSuccessResponse using PackBuffer
// Layout: BindingPciVid(5) + command(1) + completion_code(1) + reason_code(2)
void packNonSuccessResponse(PackBuffer& packer, uint8_t command,
                            uint8_t completionCode, uint16_t reasonCode)
{
    // BindingPciVid header for response
    // pci_vendor_id (big-endian)
    packer.pack(static_cast<uint16_t>(htobe16(gpu::nvidiaPciVendorId)));
    // instance_id: response (bit 7 clear), instance=0
    packer.pack(static_cast<uint8_t>(0x00));
    // ocp_version: type=8 (upper nibble), version=9 (lower nibble)
    packer.pack(static_cast<uint8_t>(0x89));
    // ocp_accelerator_management_msg_type = DEVICE_CAPABILITY_DISCOVERY = 0
    packer.pack(
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));

    packer.pack(command);
    packer.pack(completionCode);
    packer.pack(reasonCode); // little-endian by PackBuffer
}

// Tests for gpu::encodeSetEventSubscriptionRequest

TEST_F(GpuMctpVdmTests, EncodeSetEventSubscriptionRequestSuccess)
{
    // SetEventSubscriptionRequest: CommonRequest(7) +
    //   generation_setting(1) + receiver_setting(1) = 9 bytes
    std::vector<uint8_t> buf(256);

    const uint8_t eid = 0x42;
    int result = gpu::encodeSetEventSubscriptionRequest(2, eid, buf);
    EXPECT_EQ(result, 0);

    UnpackBuffer unpacker(std::span<const uint8_t>(buf.data(), 9));

    // BindingPciVid header
    uint16_t pciVendorId{};
    uint8_t instanceId{};
    uint8_t ocpVersion{};
    uint8_t msgType{};
    unpacker.unpack(pciVendorId);
    unpacker.unpack(instanceId);
    unpacker.unpack(ocpVersion);
    unpacker.unpack(msgType);

    EXPECT_EQ(pciVendorId, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_NE(instanceId & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(ocpVersion & ocp::accelerator_management::ocpVersionBitMask,
              ocp::accelerator_management::ocpVersion);
    EXPECT_EQ((ocpVersion & ocp::accelerator_management::ocpTypeBitMask) >>
                  ocp::accelerator_management::ocpTypeBitOffset,
              ocp::accelerator_management::ocpType);
    EXPECT_EQ(msgType, static_cast<uint8_t>(
                           gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));

    // CommonRequest fields
    uint8_t command{};
    uint8_t dataSize{};
    unpacker.unpack(command);
    unpacker.unpack(dataSize);

    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION));
    EXPECT_EQ(dataSize, 2);

    // Payload: generation_setting + receiver_setting
    uint8_t generationSetting{};
    uint8_t receiverSetting{};
    unpacker.unpack(generationSetting);
    unpacker.unpack(receiverSetting);

    EXPECT_EQ(unpacker.getError(), 0);
    EXPECT_EQ(generationSetting, 2);
    EXPECT_EQ(receiverSetting, eid);
}

TEST_F(GpuMctpVdmTests, EncodeSetEventSubscriptionRequestBufferTooSmall)
{
    std::vector<uint8_t> buf(5); // Too small for 9-byte request
    int result = gpu::encodeSetEventSubscriptionRequest(2, 0x42, buf);
    EXPECT_EQ(result, EINVAL);
}

// Tests for gpu::decodeSetEventSubscriptionResponse

TEST_F(GpuMctpVdmTests, DecodeSetEventSubscriptionResponseSuccess)
{
    // CommonNonSuccessResponse: BindingPciVid(5) + command(1) + cc(1) +
    //   reason_code(2) = 9 bytes
    std::vector<uint8_t> buf(9);
    PackBuffer packer(buf);

    packNonSuccessResponse(
        packer,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSubscriptionResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
}

TEST_F(GpuMctpVdmTests, DecodeSetEventSubscriptionResponseError)
{
    std::vector<uint8_t> buf(9);
    PackBuffer packer(buf);

    packNonSuccessResponse(
        packer,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_EVENT_SUBSCRIPTION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSubscriptionResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeSetEventSubscriptionResponseInvalidSize)
{
    std::vector<uint8_t> buf(4); // Too small for 9-byte response

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSubscriptionResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, EINVAL);
}

// Tests for gpu::encodeSetEventSourcesRequest

TEST_F(GpuMctpVdmTests, EncodeSetEventSourcesRequestSuccess)
{
    // SetEventSourcesRequest: CommonRequest(7) +
    //   messageType(1) + sources(8) = 16 bytes
    std::vector<uint8_t> buf(256);

    const uint64_t sources = 0xDEADBEEFCAFE0123;
    const uint8_t messageType = 3; // PLATFORM_ENVIRONMENTAL
    int result = gpu::encodeSetEventSourcesRequest(sources, messageType, buf);
    EXPECT_EQ(result, 0);

    UnpackBuffer unpacker(std::span<const uint8_t>(buf.data(), 16));

    // BindingPciVid header
    uint16_t pciVendorId{};
    uint8_t instanceId{};
    uint8_t ocpVersion{};
    uint8_t msgType{};
    unpacker.unpack(pciVendorId);
    unpacker.unpack(instanceId);
    unpacker.unpack(ocpVersion);
    unpacker.unpack(msgType);

    EXPECT_EQ(pciVendorId, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_NE(instanceId & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(msgType, static_cast<uint8_t>(
                           gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY));

    // CommonRequest fields
    uint8_t command{};
    uint8_t dataSize{};
    unpacker.unpack(command);
    unpacker.unpack(dataSize);

    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES));
    EXPECT_EQ(dataSize, 9);

    // Payload: messageType + sources
    uint8_t decodedMessageType{};
    uint64_t decodedSources{};
    unpacker.unpack(decodedMessageType);
    unpacker.unpack(decodedSources);

    EXPECT_EQ(unpacker.getError(), 0);
    EXPECT_EQ(decodedMessageType, messageType);
    EXPECT_EQ(decodedSources, sources);
}

TEST_F(GpuMctpVdmTests, EncodeSetEventSourcesRequestBufferTooSmall)
{
    std::vector<uint8_t> buf(5); // Too small for 16-byte request
    int result = gpu::encodeSetEventSourcesRequest(0x1, 3, buf);
    EXPECT_EQ(result, EINVAL);
}

// Tests for gpu::decodeSetEventSourcesResponse

TEST_F(GpuMctpVdmTests, DecodeSetEventSourcesResponseSuccess)
{
    std::vector<uint8_t> buf(9);
    PackBuffer packer(buf);

    packNonSuccessResponse(
        packer,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSourcesResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
}

TEST_F(GpuMctpVdmTests, DecodeSetEventSourcesResponseError)
{
    std::vector<uint8_t> buf(9);
    PackBuffer packer(buf);

    packNonSuccessResponse(
        packer,
        static_cast<uint8_t>(
            gpu::DeviceCapabilityDiscoveryCommands::SET_CURRENT_EVENT_SOURCES),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);
    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSourcesResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeSetEventSourcesResponseInvalidSize)
{
    std::vector<uint8_t> buf(4); // Too small for 9-byte response

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = gpu::decodeSetEventSourcesResponse(buf, cc, reasonCode);

    EXPECT_EQ(result, EINVAL);
}

// Tests for gpu::encodeGetCurrentUtilizationModeRequest

TEST_F(GpuMctpVdmTests, EncodeGetCurrentUtilizationModeRequestSuccess)
{
    const uint8_t instanceId = 5;
    std::vector<uint8_t> buf(256);

    int result = gpu::encodeGetCurrentUtilizationModeRequest(instanceId, buf);
    EXPECT_EQ(result, 0);

    // CommonRequest: BindingPciVid(5) + command(1) + data_size(1) = 7 bytes
    UnpackBuffer unpacker(std::span<const uint8_t>(buf.data(), 7));

    // BindingPciVid header
    uint16_t pciVendorId{};
    uint8_t instId{};
    uint8_t ocpVersion{};
    uint8_t msgType{};
    unpacker.unpack(pciVendorId);
    unpacker.unpack(instId);
    unpacker.unpack(ocpVersion);
    unpacker.unpack(msgType);

    EXPECT_EQ(pciVendorId, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(instId & ocp::accelerator_management::instanceIdBitMask,
              instanceId);
    EXPECT_NE(instId & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(ocpVersion & ocp::accelerator_management::ocpVersionBitMask,
              ocp::accelerator_management::ocpVersion);
    EXPECT_EQ((ocpVersion & ocp::accelerator_management::ocpTypeBitMask) >>
                  ocp::accelerator_management::ocpTypeBitOffset,
              ocp::accelerator_management::ocpType);
    EXPECT_EQ(msgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // CommonRequest fields
    uint8_t command{};
    uint8_t dataSize{};
    unpacker.unpack(command);
    unpacker.unpack(dataSize);

    EXPECT_EQ(unpacker.getError(), 0);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION));
    EXPECT_EQ(dataSize, 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetCurrentUtilizationModeRequestBufferTooSmall)
{
    std::vector<uint8_t> buf(3); // Too small for 7-byte request
    int result = gpu::encodeGetCurrentUtilizationModeRequest(0, buf);
    EXPECT_EQ(result, EINVAL);
}

// Tests for gpu::decodeLongRunningResponseEvent

TEST_F(GpuMctpVdmTests, DecodeLongRunningResponseEventSuccess)
{
    // LongRunningResponseEvent: instanceId(1) + completionCode(1) +
    //   reasonCode(2) = 4 bytes
    std::vector<uint8_t> buf(4);
    PackBuffer packer(buf);

    packer.pack(static_cast<uint8_t>(7));                       // instanceId
    packer.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // cc
    packer.pack(static_cast<uint16_t>(0));                      // reasonCode

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t instanceId{};
    std::span<const uint8_t> responseData;

    int result = gpu::decodeLongRunningResponseEvent(buf, cc, reasonCode,
                                                     instanceId, responseData);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(instanceId, 7);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_TRUE(responseData.empty());
}

TEST_F(GpuMctpVdmTests, DecodeLongRunningResponseEventWithResponseData)
{
    // 4-byte header + 3 bytes response data
    std::vector<uint8_t> buf(7);
    PackBuffer packer(buf);

    packer.pack(static_cast<uint8_t>(2));                       // instanceId
    packer.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS)); // cc
    packer.pack(static_cast<uint16_t>(0));                      // reasonCode

    ASSERT_EQ(packer.getError(), 0);

    // Fill response data
    buf[4] = 0xAA;
    buf[5] = 0xBB;
    buf[6] = 0xCC;

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t instanceId{};
    std::span<const uint8_t> responseData;

    int result = gpu::decodeLongRunningResponseEvent(buf, cc, reasonCode,
                                                     instanceId, responseData);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(instanceId, 2);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    ASSERT_EQ(responseData.size(), 3);
    EXPECT_EQ(responseData[0], 0xAA);
    EXPECT_EQ(responseData[1], 0xBB);
    EXPECT_EQ(responseData[2], 0xCC);
}

TEST_F(GpuMctpVdmTests, DecodeLongRunningResponseEventError)
{
    std::vector<uint8_t> buf(4);
    PackBuffer packer(buf);

    packer.pack(static_cast<uint8_t>(3));                     // instanceId
    packer.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR)); // cc
    packer.pack(static_cast<uint16_t>(0x5678));               // reasonCode

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t instanceId{};
    std::span<const uint8_t> responseData;

    int result = gpu::decodeLongRunningResponseEvent(buf, cc, reasonCode,
                                                     instanceId, responseData);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(instanceId, 3);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeLongRunningResponseEventBufferTooSmall)
{
    std::vector<uint8_t> buf(2); // Too small for 4-byte event

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint8_t instanceId{};
    std::span<const uint8_t> responseData;

    int result = gpu::decodeLongRunningResponseEvent(buf, cc, reasonCode,
                                                     instanceId, responseData);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, EncodeGetEccErrorCountsRequestSuccess)
{
    const uint8_t instanceId = 1;
    std::array<uint8_t, gpu::getEccErrorCountsRequestSize> buf{};

    int result = gpu::encodeGetEccErrorCountsRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf{std::span<const uint8_t>(buf)};
    ocp::accelerator_management::MessageType msgType{};
    uint8_t recvInstanceId = 0;
    uint8_t recvMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, recvInstanceId,
                  recvMsgType),
              0);
    EXPECT_EQ(msgType, ocp::accelerator_management::MessageType::REQUEST);
    EXPECT_EQ(recvInstanceId & ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_EQ(recvMsgType,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));

    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, 0);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetEccErrorCountsRequestBufferTooSmall)
{
    const uint8_t instanceId = 1;
    std::array<uint8_t, 1> buf{};

    int result = gpu::encodeGetEccErrorCountsRequest(instanceId, buf);

    EXPECT_EQ(result, EINVAL);
}

// Helper: pack a CommonResponse header using PackBuffer
// Layout: BindingPciVid(5) + command(1) + cc(1) + reason_code(2)
void packCommonResponseHeader(PackBuffer& packer, uint8_t msgType,
                              uint8_t command, uint8_t completionCode,
                              uint16_t reasonCode)
{
    // BindingPciVid header for response
    // pci_vendor_id (big-endian)
    packer.pack(static_cast<uint16_t>(htobe16(gpu::nvidiaPciVendorId)));
    // instance_id: response (bit 7 clear), instance=0
    packer.pack(static_cast<uint8_t>(0x00));
    // ocp_version: type=8 (upper nibble), version=9 (lower nibble)
    packer.pack(static_cast<uint8_t>(0x89));
    // ocp_accelerator_management_msg_type
    packer.pack(msgType);

    packer.pack(command);
    packer.pack(completionCode);
    packer.pack(reasonCode);
}

// Tests for gpu::decodeGetCurrentUtilizationModeResponse

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseSuccess)
{
    // CommonResponse header(9) + data_size(2) + gpuUtil(4) + memUtil(4) = 19
    std::vector<uint8_t> buf(19);
    PackBuffer packer(buf);

    packCommonResponseHeader(
        packer, static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    // data_size = 8 (2 x uint32_t)
    packer.pack(static_cast<uint16_t>(8));
    // gpuUtilization
    packer.pack(static_cast<uint32_t>(75));
    // memoryUtilization
    packer.pack(static_cast<uint32_t>(42));

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(gpuUtilization, 75U);
    EXPECT_EQ(memoryUtilization, 42U);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseZeroValues)
{
    std::vector<uint8_t> buf(19);
    PackBuffer packer(buf);

    packCommonResponseHeader(
        packer, static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    packer.pack(static_cast<uint16_t>(8));
    packer.pack(static_cast<uint32_t>(0));
    packer.pack(static_cast<uint32_t>(0));

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(gpuUtilization, 0U);
    EXPECT_EQ(memoryUtilization, 0U);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseMaxValues)
{
    std::vector<uint8_t> buf(19);
    PackBuffer packer(buf);

    packCommonResponseHeader(
        packer, static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    packer.pack(static_cast<uint16_t>(8));
    packer.pack(static_cast<uint32_t>(0xFFFFFFFF));
    packer.pack(static_cast<uint32_t>(0xFFFFFFFF));

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(gpuUtilization, 0xFFFFFFFFU);
    EXPECT_EQ(memoryUtilization, 0xFFFFFFFFU);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseError)
{
    // Non-success response: header(9) only, no data payload
    std::vector<uint8_t> buf(9);
    PackBuffer packer(buf);

    packCommonResponseHeader(
        packer, static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::ERROR),
        0x1234);

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseInvalidDataSize)
{
    // Valid header but wrong data_size (4 instead of 8)
    std::vector<uint8_t> buf(15);
    PackBuffer packer(buf);

    packCommonResponseHeader(
        packer, static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL),
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_UTILIZATION),
        static_cast<uint8_t>(
            ocp::accelerator_management::CompletionCode::SUCCESS),
        0);
    // Wrong data_size: 4 instead of expected 8
    packer.pack(static_cast<uint16_t>(4));
    packer.pack(static_cast<uint32_t>(100));

    ASSERT_EQ(packer.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentUtilizationModeResponseBufferTooSmall)
{
    std::vector<uint8_t> buf(4); // Too small for response header

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t gpuUtilization{};
    uint32_t memoryUtilization{};

    int result = gpu::decodeGetCurrentUtilizationModeResponse(
        buf, cc, reasonCode, gpuUtilization, memoryUtilization);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseSuccess)
{
    constexpr uint16_t eccDataSize = sizeof(uint16_t) + sizeof(uint32_t) * 5;
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + eccDataSize);

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(uint16_t{0});
    pbuf.pack(eccDataSize);
    pbuf.pack(uint16_t{0x0001});
    pbuf.pack(uint32_t{100});
    pbuf.pack(uint32_t{10});
    pbuf.pack(uint32_t{5});
    pbuf.pack(uint32_t{50});
    pbuf.pack(uint32_t{2});
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t flags{};
    uint32_t sramCorrected{};
    uint32_t sramUncorrectedSecded{};
    uint32_t sramUncorrectedParity{};
    uint32_t dramCorrected{};
    uint32_t dramUncorrected{};

    int result = gpu::decodeGetEccErrorCountsResponse(
        buf, cc, reasonCode, flags, sramCorrected, sramUncorrectedSecded,
        sramUncorrectedParity, dramCorrected, dramUncorrected);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(flags, 0x0001);
    EXPECT_EQ(sramCorrected, 100U);
    EXPECT_EQ(sramUncorrectedSecded, 10U);
    EXPECT_EQ(sramUncorrectedParity, 5U);
    EXPECT_EQ(dramCorrected, 50U);
    EXPECT_EQ(dramUncorrected, 2U);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseError)
{
    std::array<uint8_t, ocp::accelerator_management::commonResponseSize> buf{};

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY));
    pbuf.pack(htole16(uint16_t{0x1234}));
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t flags{};
    uint32_t sramCorrected{};
    uint32_t sramUncorrectedSecded{};
    uint32_t sramUncorrectedParity{};
    uint32_t dramCorrected{};
    uint32_t dramUncorrected{};

    int result = gpu::decodeGetEccErrorCountsResponse(
        buf, cc, reasonCode, flags, sramCorrected, sramUncorrectedSecded,
        sramUncorrectedParity, dramCorrected, dramUncorrected);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseBufferTooSmall)
{
    std::array<uint8_t, ocp::accelerator_management::commonResponseSize> buf{};

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(uint16_t{0});
    pbuf.pack(uint16_t{2});
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint16_t flags{};
    uint32_t sramCorrected{};
    uint32_t sramUncorrectedSecded{};
    uint32_t sramUncorrectedParity{};
    uint32_t dramCorrected{};
    uint32_t dramUncorrected{};

    int result = gpu::decodeGetEccErrorCountsResponse(
        buf, cc, reasonCode, flags, sramCorrected, sramUncorrectedSecded,
        sramUncorrectedParity, dramCorrected, dramUncorrected);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, EncodeGetCurrentClockFrequencyRequestSuccess)
{
    const uint8_t instanceId = 5;
    std::array<uint8_t, gpu::getCurrentClockFrequencyRequestSize> buf{};

    int result = gpu::encodeGetCurrentClockFrequencyRequest(
        instanceId, gpu::ClockType::MEMORY_CLOCK, buf);
    EXPECT_EQ(result, 0);

    UnpackBuffer ubuf{std::span<const uint8_t>(buf)};
    ocp::accelerator_management::MessageType msgType{};
    uint8_t recvInstanceId = 0;
    uint8_t recvMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, recvInstanceId,
                  recvMsgType),
              0);
    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(
        command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    uint8_t clockId = 0;
    ubuf.unpack(clockId);
    EXPECT_EQ(clockId, static_cast<uint8_t>(gpu::ClockType::MEMORY_CLOCK));
}

TEST_F(GpuMctpVdmTests, EncodeGetCurrentClockFrequencyRequestBufferTooSmall)
{
    const uint8_t instanceId = 5;
    std::array<uint8_t, 1> buf{};

    int result = gpu::encodeGetCurrentClockFrequencyRequest(
        instanceId, gpu::ClockType::MEMORY_CLOCK, buf);
    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentClockFrequencyResponseSuccess)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + sizeof(uint32_t));

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(uint16_t{0});
    pbuf.pack(htole16(static_cast<uint16_t>(sizeof(uint32_t))));
    pbuf.pack(htole32(uint32_t{405}));
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t clockFreqMHz{};

    int result = gpu::decodeGetCurrentClockFrequencyResponse(
        buf, cc, reasonCode, clockFreqMHz);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(clockFreqMHz, 405U);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentClockFrequencyResponseError)
{
    // CommonNonSuccessResponse: BindingPciVid(5) + command(1) + cc(1) +
    //   reason_code(2) = 9 bytes
    std::vector<uint8_t> buf(9);

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY));
    pbuf.pack(htole16(uint16_t{0x5678}));
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t clockFreqMHz{};

    int result = gpu::decodeGetCurrentClockFrequencyResponse(
        buf, cc, reasonCode, clockFreqMHz);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST_F(GpuMctpVdmTests, DecodeGetCurrentClockFrequencyResponseDataSizeMismatch)
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);

    PackBuffer pbuf{std::span<uint8_t>(buf)};
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 5,
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(uint16_t{0});
    pbuf.pack(htole16(uint16_t{2}));
    EXPECT_EQ(pbuf.getError(), 0);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t clockFreqMHz{};

    int result = gpu::decodeGetCurrentClockFrequencyResponse(
        buf, cc, reasonCode, clockFreqMHz);

    EXPECT_EQ(result, EINVAL);
}

} // namespace gpu_mctp_tests
