/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
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

// Tests for OcpMctpVdm::packHeader function
TEST_F(OcpMctpVdmTests, PackHeaderRequestSuccess)
{
    const uint16_t pciVendorId = 0x1234;
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 5;
    hdr.msg_type = 0x7E;

    int result = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(msg.pci_vendor_id, htobe16(pciVendorId));
    EXPECT_EQ(msg.instance_id & ocp::accelerator_management::instanceIdBitMask,
              5);
    EXPECT_NE(msg.instance_id & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(msg.ocp_version & 0x0F, ocp::accelerator_management::ocpVersion);
    EXPECT_EQ((msg.ocp_version & 0xF0) >>
                  ocp::accelerator_management::ocpTypeBitOffset,
              ocp::accelerator_management::ocpType);
    EXPECT_EQ(msg.ocp_accelerator_management_msg_type, 0x7E);
}

TEST_F(OcpMctpVdmTests, PackHeaderResponseSuccess)
{
    const uint16_t pciVendorId = 0x1234;
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    hdr.instance_id = 10;
    hdr.msg_type = 0x7E;

    int result = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(msg.pci_vendor_id, htobe16(pciVendorId));
    EXPECT_EQ(msg.instance_id & ocp::accelerator_management::instanceIdBitMask,
              10);
    EXPECT_EQ(msg.instance_id & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(msg.ocp_version & 0x0F, ocp::accelerator_management::ocpVersion);
    EXPECT_EQ((msg.ocp_version & 0xF0) >>
                  ocp::accelerator_management::ocpTypeBitOffset,
              ocp::accelerator_management::ocpType);
    EXPECT_EQ(msg.ocp_accelerator_management_msg_type, 0x7E);
}

TEST_F(OcpMctpVdmTests, PackHeaderInvalidMessageType)
{
    const uint16_t pciVendorId = 0x1234;
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type = 3; // Invalid message type
    hdr.instance_id = 5;
    hdr.msg_type = 0x7E;

    int result = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(OcpMctpVdmTests, PackHeaderInvalidInstanceId)
{
    const uint16_t pciVendorId = 0x1234;
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 32; // Out of range (0-31 valid)
    hdr.msg_type = 0x7E;

    int result = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);

    EXPECT_EQ(result, EINVAL);
}

// Tests for OcpMctpVdm::encodeReasonCode function
TEST_F(OcpMctpVdmTests, EncodeReasonCodeSuccess)
{
    const uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    const uint16_t reasonCode = 0x1234;
    const uint8_t commandCode = 0x42;

    ocp::accelerator_management::CommonNonSuccessResponse response{};

    int result = ocp::accelerator_management::encodeReasonCode(
        cc, reasonCode, commandCode, response);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(response.command, commandCode);
    EXPECT_EQ(response.completion_code, cc);
    EXPECT_EQ(response.reason_code, htole16(reasonCode));
}

// Tests for OcpMctpVdm::decodeReasonCodeAndCC function
TEST_F(OcpMctpVdmTests, DecodeReasonCodeAndCCSuccessCase)
{
    ocp::accelerator_management::CommonNonSuccessResponse response{};
    response.command = 0x42;
    response.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.reason_code = htole16(0x1234);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = ocp::accelerator_management::decodeReasonCodeAndCC(
        response, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0); // Should be 0 for SUCCESS
}

TEST_F(OcpMctpVdmTests, DecodeReasonCodeAndCCErrorCase)
{
    ocp::accelerator_management::CommonNonSuccessResponse response{};
    response.command = 0x42;
    response.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    response.reason_code = htole16(0x5678);

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int result = ocp::accelerator_management::decodeReasonCodeAndCC(
        response, cc, reasonCode);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERROR);
    EXPECT_EQ(reasonCode, 0x5678);
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

// Tests for GpuMctpVdm::packHeader function
TEST_F(GpuMctpVdmTests, PackHeaderSuccess)
{
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 5;
    hdr.msg_type = 0x7E;

    int result = gpu::packHeader(hdr, msg);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(msg.pci_vendor_id, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(msg.instance_id & ocp::accelerator_management::instanceIdBitMask,
              5);
    EXPECT_NE(msg.instance_id & ocp::accelerator_management::requestBitMask, 0);
    EXPECT_EQ(msg.ocp_version & 0x0F, ocp::accelerator_management::ocpVersion);
    EXPECT_EQ((msg.ocp_version & 0xF0) >>
                  ocp::accelerator_management::ocpTypeBitOffset,
              ocp::accelerator_management::ocpType);
    EXPECT_EQ(msg.ocp_accelerator_management_msg_type, 0x7E);
}

// Tests for GpuMctpVdm::encodeQueryDeviceIdentificationRequest function
TEST_F(GpuMctpVdmTests, EncodeQueryDeviceIdentificationRequestSuccess)
{
    const uint8_t instanceId = 3;
    std::vector<uint8_t> buf;

    int result = gpu::encodeQueryDeviceIdentificationRequest(instanceId, buf);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(buf.size(), sizeof(ocp::accelerator_management::Message) +
                              sizeof(gpu::QueryDeviceIdentificationRequest));

    // Verify header
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    EXPECT_EQ(msg.hdr.pci_vendor_id, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(msg.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(msg.hdr.instance_id & ocp::accelerator_management::requestBitMask,
              0);

    // Verify request data
    gpu::QueryDeviceIdentificationRequest request{};
    std::memcpy(&request, buf.data() + sizeof(msg), sizeof(request));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION));
    EXPECT_EQ(request.hdr.data_size, 0);
}

// Tests for GpuMctpVdm::decodeQueryDeviceIdentificationResponse function
TEST_F(GpuMctpVdmTests, DecodeQueryDeviceIdentificationResponseSuccess)
{
    // Create a mock successful response
    std::vector<uint8_t> buf(sizeof(ocp::accelerator_management::Message) +
                             sizeof(gpu::QueryDeviceIdentificationResponse));

    // Populate Message header
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Populate response data
    gpu::QueryDeviceIdentificationResponse response{};
    response.hdr.command = static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size =
        htole16(2); // Size of device_identification + instance_id
    response.device_identification =
        static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU);
    response.instance_id = 7;

    std::memcpy(buf.data() + sizeof(msg), &response, sizeof(response));

    // Test decoding
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
    // Create a mock error response
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::Message) +
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));

    // Populate Message header
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    errorResponse.command = static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    errorResponse.reason_code = htole16(0x1234);

    std::memcpy(buf.data() + sizeof(msg), &errorResponse,
                sizeof(errorResponse));

    // Test decoding
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
    // Create a too-small buffer
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::Message) + 2); // Too small

    // Populate Message header only
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Test decoding with insufficient data
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
    std::vector<uint8_t> buf;

    int result =
        gpu::encodeGetTemperatureReadingRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(buf.size(), sizeof(ocp::accelerator_management::Message) +
                              sizeof(gpu::GetTemperatureReadingRequest));

    // Verify header
    ocp::accelerator_management::Message msg{};
    std::memcpy(&msg, buf.data(), sizeof(msg));

    EXPECT_EQ(msg.hdr.pci_vendor_id, htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(msg.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(msg.hdr.instance_id & ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(msg.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    gpu::GetTemperatureReadingRequest request{};
    std::memcpy(&request, buf.data() + sizeof(msg), sizeof(request));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    EXPECT_EQ(request.hdr.data_size, sizeof(sensorId));
    EXPECT_EQ(request.sensor_id, sensorId);
}

// Tests for GpuMctpVdm::decodeGetTemperatureReadingResponse function
TEST_F(GpuMctpVdmTests, DecodeGetTemperatureReadingResponseSuccess)
{
    // Create a mock successful response
    std::vector<uint8_t> buf(sizeof(ocp::accelerator_management::Message) +
                             sizeof(gpu::GetTemperatureReadingResponse));

    // Populate Message header
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 4;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Populate response data
    gpu::GetTemperatureReadingResponse response{};
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(int32_t));

    // Set a temperature value of 75.5°C (75.5 * 256 = 19328)
    response.reading = htole32(19328);

    std::memcpy(buf.data() + sizeof(msg), &response, sizeof(response));

    // Test decoding
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
    // Create a mock error response
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::Message) +
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));

    // Populate Message header
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 4;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x4321);

    std::memcpy(buf.data() + sizeof(msg), &errorResponse,
                sizeof(errorResponse));

    // Test decoding
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
    // Create a mock response with invalid data_size
    std::vector<uint8_t> buf(sizeof(ocp::accelerator_management::Message) +
                             sizeof(gpu::GetTemperatureReadingResponse));

    // Populate Message header
    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 4;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

    // Populate response data with incorrect data_size
    gpu::GetTemperatureReadingResponse response{};
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(1); // Invalid - should be sizeof(int32_t)
    response.reading = htole32(19328);

    std::memcpy(buf.data() + sizeof(msg), &response, sizeof(response));

    // Test decoding
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    double temperatureReading{};

    int result = gpu::decodeGetTemperatureReadingResponse(
        buf, cc, reasonCode, temperatureReading);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

} // namespace gpu_mctp_tests

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
