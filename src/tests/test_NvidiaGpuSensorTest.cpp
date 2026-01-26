/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <array>
#include <bit>
#include <cerrno>
#include <cstdint>
#include <cstring>
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

    std::array<uint8_t, sizeof(response)> buf{};
    std::memcpy(buf.data(), &response, sizeof(response));

    int result =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

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

    std::array<uint8_t, sizeof(response)> buf{};
    std::memcpy(buf.data(), &response, sizeof(response));

    int result =
        ocp::accelerator_management::decodeReasonCodeAndCC(buf, cc, reasonCode);

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
    std::vector<uint8_t> buf(256);

    int result = gpu::encodeQueryDeviceIdentificationRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    gpu::QueryDeviceIdentificationRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION));
    EXPECT_EQ(request.hdr.data_size, 0);
}

// Tests for GpuMctpVdm::decodeQueryDeviceIdentificationResponse function
TEST_F(GpuMctpVdmTests, DecodeQueryDeviceIdentificationResponseSuccess)
{
    // Create a mock successful response
    std::vector<uint8_t> buf(sizeof(gpu::QueryDeviceIdentificationResponse));

    gpu::QueryDeviceIdentificationResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
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

    std::memcpy(buf.data(), &response, sizeof(response));

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
    // Create a mock successful response
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));

    ocp::accelerator_management::CommonNonSuccessResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

    gpu::packHeader(headerInfo, response.msgHdr.hdr);

    // Populate response data
    response.command = static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    response.command = static_cast<uint8_t>(
        gpu::DeviceCapabilityDiscoveryCommands::QUERY_DEVICE_IDENTIFICATION);
    response.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    response.reason_code = htole16(0x1234);

    std::memcpy(buf.data(), &response, sizeof(response));

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
    std::vector<uint8_t> buf(256);

    int result =
        gpu::encodeGetTemperatureReadingRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    gpu::GetTemperatureReadingRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
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
    std::vector<uint8_t> buf(sizeof(gpu::GetTemperatureReadingResponse));

    gpu::GetTemperatureReadingResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 4;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(int32_t));

    // Set a temperature value of 75.5°C (75.5 * 256 = 19328)
    response.reading = htole32(19328);

    std::memcpy(buf.data(), &response, sizeof(response));

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
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 3;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x4321);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

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
    std::vector<uint8_t> buf(sizeof(gpu::GetTemperatureReadingResponse));

    gpu::GetTemperatureReadingResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 4;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(1); // Invalid - should be sizeof(int32_t)
    response.reading = htole32(19328);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t, sizeof(gpu::ReadThermalParametersRequest)> buf{};

    int result =
        gpu::encodeReadThermalParametersRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    gpu::ReadThermalParametersRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS));
    EXPECT_EQ(request.hdr.data_size, sizeof(sensorId));
    EXPECT_EQ(request.sensor_id, sensorId);
}

// Tests for GpuMctpVdm::decodeReadThermalParametersResponse function
TEST_F(GpuMctpVdmTests, DecodeReadThermalParametersResponseSuccess)
{
    // Create a mock successful response
    std::array<uint8_t, sizeof(gpu::ReadThermalParametersResponse)> buf{};

    gpu::ReadThermalParametersResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 5;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(int32_t));

    // Set a threshold value of 85°C (85 * 256 = 21760)
    response.threshold = htole32(21760);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 5;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x5678);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    // Test decoding
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
    // Create a mock response with invalid data_size
    std::array<uint8_t, sizeof(gpu::ReadThermalParametersResponse)> buf{};

    gpu::ReadThermalParametersResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 5;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::READ_THERMAL_PARAMETERS);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(2); // Invalid - should be sizeof(int32_t)
    response.threshold = htole32(21760);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t, sizeof(gpu::GetPowerDrawRequest)> buf{};

    int result = gpu::encodeGetPowerDrawRequest(
        commandCode, instanceId, sensorId, averagingInterval, buf);

    EXPECT_EQ(result, 0);

    gpu::GetPowerDrawRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    EXPECT_EQ(request.hdr.command, static_cast<uint8_t>(commandCode));
    EXPECT_EQ(request.hdr.data_size,
              sizeof(sensorId) + sizeof(averagingInterval));
    EXPECT_EQ(request.sensorId, sensorId);
    EXPECT_EQ(request.averagingInterval, averagingInterval);
}

// Tests for GpuMctpVdm::decodeGetCurrentPowerDrawResponse function
TEST_F(GpuMctpVdmTests, DecodeGetCurrentPowerDrawResponseSuccess)
{
    // Create a mock successful response
    std::array<uint8_t, sizeof(gpu::GetPowerDrawResponse)> buf{};

    gpu::GetPowerDrawResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 6;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(uint32_t));

    // Set a power value of 250W
    response.power = htole32(250);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 6;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x9ABC);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    // Test decoding
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
    // Create a mock response with invalid data_size
    std::array<uint8_t, sizeof(gpu::GetPowerDrawResponse)> buf{};

    gpu::GetPowerDrawResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 6;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_POWER_DRAW);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(2); // Invalid - should be sizeof(uint32_t)
    response.power = htole32(250);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t, sizeof(gpu::GetCurrentEnergyCounterRequest)> buf{};

    int result =
        gpu::encodeGetCurrentEnergyCounterRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    gpu::GetCurrentEnergyCounterRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    EXPECT_EQ(
        request.hdr.command,
        static_cast<uint8_t>(
            gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER));
    EXPECT_EQ(request.hdr.data_size, sizeof(sensorId));
    EXPECT_EQ(request.sensor_id, sensorId);
}

// Tests for GpuMctpVdm::decodeGetCurrentEnergyCounterResponse function
TEST_F(GpuMctpVdmTests, DecodeGetCurrentEnergyCounterResponseSuccess)
{
    // Create a mock successful response
    std::array<uint8_t, sizeof(gpu::GetCurrentEnergyCounterResponse)> buf{};

    gpu::GetCurrentEnergyCounterResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 7;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(uint64_t));

    // Set an energy value of 1000 Wh (1000 * 3600 = 3600000 Joules)
    response.energy = htole64(3600000);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 7;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0xDEF0);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    // Test decoding
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
    // Create a mock response with invalid data_size
    std::array<uint8_t, sizeof(gpu::GetCurrentEnergyCounterResponse)> buf{};

    gpu::GetCurrentEnergyCounterResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 7;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_CURRENT_ENERGY_COUNTER);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(4); // Invalid - should be sizeof(uint64_t)
    response.energy = htole64(3600000);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonRequest)>
        buf{};

    int result = gpu::encodeGetDriverInformationRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    ocp::accelerator_management::CommonRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    EXPECT_EQ(request.command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION));
    EXPECT_EQ(request.data_size, 0);
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
    // Create a buffer large enough for the response with driver version string
    const std::string expectedVersion = "535.104.05";
    const size_t dataSize =
        sizeof(gpu::DriverState) + expectedVersion.size() + 1;
    std::vector<uint8_t> buf(
        sizeof(gpu::GetDriverInformationResponse) + expectedVersion.size());

    // Set up the common response header
    ocp::accelerator_management::CommonResponse hdr{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 9;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, hdr.msgHdr.hdr);

    hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION);
    hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    hdr.reserved = 0;
    hdr.data_size = htole16(static_cast<uint16_t>(dataSize));

    // Copy header to buffer
    std::memcpy(buf.data(), &hdr, sizeof(hdr));

    // Set driver state (DRIVER_STATE_LOADED)
    buf[sizeof(hdr)] =
        static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_LOADED);

    // Copy driver version string after driver state
    std::memcpy(buf.data() + sizeof(hdr) + sizeof(gpu::DriverState),
                expectedVersion.data(), expectedVersion.size() + 1);

    // Test decoding
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
    // Create a buffer for driver not loaded state
    const size_t dataSize =
        sizeof(gpu::DriverState) + 1; // Minimum version size
    std::vector<uint8_t> buf(sizeof(gpu::GetDriverInformationResponse) + 1);

    // Set up the common response header
    ocp::accelerator_management::CommonResponse hdr{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 9;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, hdr.msgHdr.hdr);

    hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION);
    hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    hdr.reserved = 0;
    hdr.data_size = htole16(static_cast<uint16_t>(dataSize));

    // Copy header to buffer
    std::memcpy(buf.data(), &hdr, sizeof(hdr));

    // Set driver state (DRIVER_STATE_NOT_LOADED)
    buf[sizeof(hdr)] =
        static_cast<uint8_t>(gpu::DriverState::DRIVER_STATE_NOT_LOADED);

    // Set a null terminator for version
    buf[sizeof(hdr) + sizeof(gpu::DriverState)] = '\0';

    // Test decoding
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 9;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0xABCD);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    // Test decoding
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
    // Create a buffer that's too small
    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonResponse)>
        buf{};

    ocp::accelerator_management::CommonResponse hdr{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 9;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, hdr.msgHdr.hdr);

    hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION);
    hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    hdr.reserved = 0;
    hdr.data_size = htole16(2); // Valid data size but buffer too small

    std::memcpy(buf.data(), &hdr, sizeof(hdr));

    // Test decoding
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
    // Create a response with data_size too small
    std::vector<uint8_t> buf(sizeof(gpu::GetDriverInformationResponse) + 10);

    ocp::accelerator_management::CommonResponse hdr{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 9;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, hdr.msgHdr.hdr);

    hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_DRIVER_INFORMATION);
    hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    hdr.reserved = 0;
    // data_size = 1 is invalid (needs at least sizeof(DriverState) + 1 = 2)
    hdr.data_size = htole16(1);

    std::memcpy(buf.data(), &hdr, sizeof(hdr));

    // Test decoding
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
    std::array<uint8_t, sizeof(gpu::GetVoltageRequest)> buf{};

    int result = gpu::encodeGetVoltageRequest(instanceId, sensorId, buf);

    EXPECT_EQ(result, 0);

    gpu::GetVoltageRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    // Verify request data
    EXPECT_EQ(
        request.hdr.command,
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE));
    EXPECT_EQ(request.hdr.data_size, sizeof(sensorId));
    EXPECT_EQ(request.sensor_id, sensorId);
}

// Tests for GpuMctpVdm::decodeGetVoltageResponse function
TEST_F(GpuMctpVdmTests, DecodeGetVoltageResponseSuccess)
{
    // Create a mock successful response
    std::array<uint8_t, sizeof(gpu::GetVoltageResponse)> buf{};

    gpu::GetVoltageResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 8;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    // Populate response data
    response.hdr.command =
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(sizeof(uint32_t));

    // Set a voltage value of 12.5V (12.5 * 1000 = 12500 mV)
    response.voltage = htole32(12500);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    // Populate error response data
    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 8;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command =
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x1234);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    // Test decoding
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
    // Create a mock response with invalid data_size
    std::array<uint8_t, sizeof(gpu::GetVoltageResponse)> buf{};

    gpu::GetVoltageResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 8;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command =
        static_cast<uint8_t>(gpu::PlatformEnvironmentalCommands::GET_VOLTAGE);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size = htole16(2); // Invalid - should be sizeof(uint32_t)
    response.voltage = htole32(12500);

    std::memcpy(buf.data(), &response, sizeof(response));

    // Test decoding
    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    uint32_t voltage{};

    int result = gpu::decodeGetVoltageResponse(buf, cc, reasonCode, voltage);

    EXPECT_EQ(result, EINVAL); // Should indicate error for invalid data size
}

TEST_F(GpuMctpVdmTests, EncodeQueryScalarGroupTelemetryV2RequestSuccess)
{
    const uint8_t instanceId = 10;
    const gpu::PciePortType portType = gpu::PciePortType::DOWNSTREAM;
    const uint8_t upstreamPortNumber = 3;
    const uint8_t portNumber = 5;
    const uint8_t groupId = 2;
    std::array<uint8_t, sizeof(gpu::QueryScalarGroupTelemetryV2Request)> buf{};

    int result = gpu::encodeQueryScalarGroupTelemetryV2Request(
        instanceId, portType, upstreamPortNumber, portNumber, groupId, buf);

    EXPECT_EQ(result, 0);

    gpu::QueryScalarGroupTelemetryV2Request request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2));
    EXPECT_EQ(request.hdr.data_size, 3);
    EXPECT_EQ(request.upstreamPortNumber,
              (static_cast<uint8_t>(portType) << 7) |
                  (upstreamPortNumber & 0x7F));
    EXPECT_EQ(request.portNumber, portNumber);
    EXPECT_EQ(request.groupId, groupId);
}

TEST_F(GpuMctpVdmTests, DecodeQueryScalarGroupTelemetryV2ResponseSuccess)
{
    const size_t numValues = 4;
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::CommonResponse) +
        numValues * sizeof(uint32_t));

    ocp::accelerator_management::CommonResponse* response =
        std::bit_cast<ocp::accelerator_management::CommonResponse*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 10;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, response->msgHdr.hdr);

    response->command = static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(numValues * sizeof(uint32_t));

    uint8_t* telemetryData =
        buf.data() + sizeof(ocp::accelerator_management::CommonResponse);

    for (int i = 0; i < static_cast<int>(numValues); ++i)
    {
        const uint32_t value = htole32((i + 1) * 100);
        std::memcpy(telemetryData + i * sizeof(value), &value, sizeof(value));
    }

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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 10;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PcieLinkCommands::QueryScalarGroupTelemetryV2);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x5678);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

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
    std::array<uint8_t, sizeof(ocp::accelerator_management::Message)> buf{};

    ocp::accelerator_management::Message msg{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 10;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, msg.hdr);
    std::memcpy(buf.data(), &msg, sizeof(msg));

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
    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonRequest)>
        buf{};

    int result = gpu::encodeListPciePortsRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    ocp::accelerator_management::CommonRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PCIE_LINK));

    EXPECT_EQ(request.command,
              static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts));
    EXPECT_EQ(request.data_size, 0);
}

TEST_F(GpuMctpVdmTests, DecodeListPciePortsResponseSuccess)
{
    const size_t totalUpstreamPorts = 2;
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::CommonResponse) + sizeof(uint16_t) +
        totalUpstreamPorts * 2 * sizeof(uint8_t));

    ocp::accelerator_management::CommonResponse* response =
        std::bit_cast<ocp::accelerator_management::CommonResponse*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 11;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, response->msgHdr.hdr);

    response->command =
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size =
        htole16(sizeof(uint16_t) + totalUpstreamPorts * 2 * sizeof(uint8_t));

    const uint16_t upstreamPorts = htole16(totalUpstreamPorts);
    std::memcpy(buf.data() +
                    sizeof(ocp::accelerator_management::CommonResponse),
                &upstreamPorts, sizeof(uint16_t));

    uint8_t* portData = std::bit_cast<uint8_t*>(
        buf.data() + sizeof(ocp::accelerator_management::CommonResponse) +
        sizeof(uint16_t));
    portData[0] = 0;
    portData[1] = 4;
    portData[2] = 1;
    portData[3] = 2;

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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 11;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command =
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0xBEEF);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

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
    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonResponse)>
        buf{};

    ocp::accelerator_management::CommonResponse* response =
        std::bit_cast<ocp::accelerator_management::CommonResponse*>(buf.data());

    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 11;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::PCIE_LINK);

    gpu::packHeader(headerInfo, response->msgHdr.hdr);

    response->command =
        static_cast<uint8_t>(gpu::PcieLinkCommands::ListPCIePorts);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(1);

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
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
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

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
        std::vector<std::vector<uint8_t>> values;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  const uint8_t* value) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        handlerData.values.emplace_back(value, value + length);
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);

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
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x02, 0x00, // telemetryCount = 2 (little-endian)
        // Item 0: tag=0x10, tagInfo=0x85 (byte-length, valueLength=2, valid=1)
        // bit 7=1 (byte-length), bits 3-1=010 (valueLength=2), bit 0=1 (valid)
        // length = 2 bytes
        0x10, 0x85, 0xAA, 0xBB,
        // Item 1: tag=0x11, tagInfo=0x8B (byte-length, valueLength=5, valid=1)
        // bit 7=1 (byte-length), bits 3-1=101 (valueLength=5), bit 0=1 (valid)
        // length = 5 bytes
        0x11, 0x8B, 0x01, 0x02, 0x03, 0x04, 0x05};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
        std::vector<std::vector<uint8_t>> values;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  const uint8_t* value) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        handlerData.values.emplace_back(value, value + length);
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);

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
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x03, 0x00,                         // telemetryCount = 3
        // Item 0: tag=0x01, tagInfo=0x05 (valid bit set)
        0x01, 0x05, 0x11, 0x22, 0x33, 0x44,
        // Item 1: tag=0x02, tagInfo=0x06 (valid bit NOT set - bit 0 = 0)
        // This should be skipped
        0x02, 0x06, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
        // Item 2: tag=0x03, tagInfo=0x01 (valid bit set)
        0x03, 0x01, 0x55};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  const uint8_t*) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);

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
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x04, 0x00,                         // telemetryCount = 4
        // Item 0: power-of-2, valid
        0x01, 0x05, 0x11, 0x22, 0x33, 0x44,
        // Item 1: byte-length, valid
        0x02, 0x83, 0xAA,
        // Item 2: power-of-2, invalid
        0x03, 0x04, 0xCC, 0xDD, 0xEE, 0xFF,
        // Item 3: byte-length, valid
        0x04, 0x89, 0x55, 0x66, 0x77, 0x88};

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    struct HandlerData
    {
        std::vector<uint8_t> tags;
        std::vector<uint8_t> lengths;
    } handlerData;

    auto handler = [&handlerData](const uint8_t tag, const uint8_t length,
                                  const uint8_t*) -> int {
        handlerData.tags.push_back(tag);
        handlerData.lengths.push_back(length);
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);

    // 3 items should be processed (item 2 skipped)
    EXPECT_EQ(handlerData.tags.size(), 3U);
    EXPECT_EQ(handlerData.tags[0], 0x01);
    EXPECT_EQ(handlerData.lengths[0], 4); // power-of-2: 1 << 2 = 4
    EXPECT_EQ(handlerData.tags[1], 0x02);
    EXPECT_EQ(handlerData.lengths[1], 1); // byte-length: 1
    EXPECT_EQ(handlerData.tags[2], 0x04);
    EXPECT_EQ(handlerData.lengths[2], 4); // byte-length: 4
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseErrorResponse)
{
    // Test with error completion code
    std::vector<uint8_t> buf(
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 10;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = 0x42;
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x1234);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int handlerCallCount = 0;
    auto handler = [&handlerCallCount](const uint8_t, const uint8_t,
                                       const uint8_t*) -> int {
        handlerCallCount++;
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x1234);
    EXPECT_EQ(handlerCallCount, 0); // Handler should not be called on error
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseInvalidBufferSize)
{
    // Test with buffer too small
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code
        0x01 // Truncated - missing second byte of telemetryCount
    };

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    auto handler = [](const uint8_t, const uint8_t, const uint8_t*) -> int {
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeAggregateResponseEmptyItems)
{
    // Test with zero items
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x00, 0x00                          // telemetryCount = 0
    };

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};

    int handlerCallCount = 0;
    auto handler = [&handlerCallCount](const uint8_t, const uint8_t,
                                       const uint8_t*) -> int {
        handlerCallCount++;
        return 0;
    };

    int result = ocp::accelerator_management::decodeAggregateResponse(
        buf, cc, reasonCode, std::move(handler));

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(handlerCallCount, 0); // No items to process
}

// Tests for GpuMctpVdm::encodeGetPortNetworkAddressesRequest function
TEST_F(GpuMctpVdmTests, EncodeGetPortNetworkAddressesRequestSuccess)
{
    const uint8_t instanceId = 12;
    const uint16_t portNumber = 5;
    std::array<uint8_t, sizeof(gpu::GetPortNetworkAddressesRequest)> buf{};

    int result =
        gpu::encodeGetPortNetworkAddressesRequest(instanceId, portNumber, buf);

    EXPECT_EQ(result, 0);

    gpu::GetPortNetworkAddressesRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::NetworkPortCommands::GetPortNetworkAddresses));
    EXPECT_EQ(request.hdr.data_size, 2);
    EXPECT_EQ(request.portNumber, htole16(portNumber));
}

TEST_F(GpuMctpVdmTests, EncodeGetPortNetworkAddressesRequestInvalidBufferSize)
{
    const uint8_t instanceId = 12;
    const uint16_t portNumber = 5;
    std::array<uint8_t, sizeof(gpu::GetPortNetworkAddressesRequest) - 1> buf{};

    int result =
        gpu::encodeGetPortNetworkAddressesRequest(instanceId, portNumber, buf);

    EXPECT_EQ(result, EINVAL);
}

// Tests for GpuMctpVdm::decodeGetPortNetworkAddressesResponse function
TEST_F(GpuMctpVdmTests, DecodeGetPortNetworkAddressesResponseSuccess)
{
    // Test with linkType and two MAC addresses
    std::vector<uint8_t> buf = {
        0x10, 0xde, 0x00, 0x89, 0x01, 0x0f, // Message header
        0x00,                               // completion_code (SUCCESS)
        0x03, 0x00,                         // telemetryCount = 3
        // Item 0: tag=0x00, tagInfo=0x00 (power-of-2, valueLength=3, valid=1)
        // length = 1 << 0 = 1 bytes
        0x00, 0x00, 0x00, // linkType = ETHERNET (0)
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 12;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command =
        static_cast<uint8_t>(gpu::NetworkPortCommands::GetPortNetworkAddresses);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0xABCD);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

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
    std::array<uint8_t, sizeof(gpu::GetEthernetPortTelemetryCountersRequest)>
        buf{};

    int result = gpu::encodeGetEthernetPortTelemetryCountersRequest(
        instanceId, portNumber, buf);

    EXPECT_EQ(result, 0);

    gpu::GetEthernetPortTelemetryCountersRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters));
    EXPECT_EQ(request.hdr.data_size, 2);
    EXPECT_EQ(request.portNumber, htole16(portNumber));
}

TEST_F(GpuMctpVdmTests,
       EncodeGetEthernetPortTelemetryCountersRequestInvalidBufferSize)
{
    const uint8_t instanceId = 13;
    const uint16_t portNumber = 7;
    std::array<uint8_t,
               sizeof(gpu::GetEthernetPortTelemetryCountersRequest) - 1>
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
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 13;
    headerInfo.msg_type = static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::NetworkPortCommands::GetEthernetPortTelemetryCounters);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0xDEAD);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

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

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

TEST_F(GpuMctpVdmTests, EncodeGetEccErrorCountsRequestSuccess)
{
    const uint8_t instanceId = 1;
    std::array<uint8_t, sizeof(gpu::GetEccErrorCountsRequest)> buf{};

    int result = gpu::encodeGetEccErrorCountsRequest(instanceId, buf);

    EXPECT_EQ(result, 0);

    gpu::GetEccErrorCountsRequest request{};
    std::memcpy(&request, buf.data(), sizeof(request));

    EXPECT_EQ(request.hdr.msgHdr.hdr.pci_vendor_id,
              htobe16(gpu::nvidiaPciVendorId));
    EXPECT_EQ(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::instanceIdBitMask,
              instanceId & ocp::accelerator_management::instanceIdBitMask);
    EXPECT_NE(request.hdr.msgHdr.hdr.instance_id &
                  ocp::accelerator_management::requestBitMask,
              0);
    EXPECT_EQ(request.hdr.msgHdr.hdr.ocp_accelerator_management_msg_type,
              static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS));
    EXPECT_EQ(request.hdr.data_size, 0);
}

TEST_F(GpuMctpVdmTests, EncodeGetEccErrorCountsRequestBufferTooSmall)
{
    const uint8_t instanceId = 1;
    std::array<uint8_t, 1> buf{};

    int result = gpu::encodeGetEccErrorCountsRequest(instanceId, buf);

    EXPECT_EQ(result, EINVAL);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseSuccess)
{
    std::vector<uint8_t> buf(sizeof(gpu::GetEccErrorCountsResponse));

    gpu::GetEccErrorCountsResponse response{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 1;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, response.hdr.msgHdr.hdr);

    response.hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS);
    response.hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response.hdr.reserved = 0;
    response.hdr.data_size =
        htole16(sizeof(gpu::GetEccErrorCountsResponse) -
                sizeof(ocp::accelerator_management::CommonResponse));

    response.flags = htole16(0x0001);
    response.sram_corrected = htole32(100);
    response.sram_uncorrected_secded = htole32(10);
    response.sram_uncorrected_parity = htole32(5);
    response.dram_corrected = htole32(50);
    response.dram_uncorrected = htole32(2);

    std::memcpy(buf.data(), &response, sizeof(response));

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::GetEccErrorCountsResponse decoded{};

    int result =
        gpu::decodeGetEccErrorCountsResponse(buf, cc, reasonCode, decoded);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(decoded.flags, 0x0001);
    EXPECT_EQ(decoded.sram_corrected, 100U);
    EXPECT_EQ(decoded.sram_uncorrected_secded, 10U);
    EXPECT_EQ(decoded.sram_uncorrected_parity, 5U);
    EXPECT_EQ(decoded.dram_corrected, 50U);
    EXPECT_EQ(decoded.dram_uncorrected, 2U);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseError)
{
    std::array<uint8_t,
               sizeof(ocp::accelerator_management::CommonNonSuccessResponse)>
        buf{};

    ocp::accelerator_management::CommonNonSuccessResponse errorResponse{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 1;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, errorResponse.msgHdr.hdr);

    errorResponse.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS);
    errorResponse.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    errorResponse.reason_code = htole16(0x1234);

    std::memcpy(buf.data(), &errorResponse, sizeof(errorResponse));

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::GetEccErrorCountsResponse decoded{};

    int result =
        gpu::decodeGetEccErrorCountsResponse(buf, cc, reasonCode, decoded);

    EXPECT_EQ(result, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x1234);
}

TEST_F(GpuMctpVdmTests, DecodeGetEccErrorCountsResponseBufferTooSmall)
{
    std::array<uint8_t, sizeof(ocp::accelerator_management::CommonResponse)>
        buf{};

    ocp::accelerator_management::CommonResponse hdr{};
    ocp::accelerator_management::BindingPciVidInfo headerInfo{};
    headerInfo.ocp_accelerator_management_msg_type = static_cast<uint8_t>(
        ocp::accelerator_management::MessageType::RESPONSE);
    headerInfo.instance_id = 1;
    headerInfo.msg_type =
        static_cast<uint8_t>(gpu::MessageType::PLATFORM_ENVIRONMENTAL);

    gpu::packHeader(headerInfo, hdr.msgHdr.hdr);

    hdr.command = static_cast<uint8_t>(
        gpu::PlatformEnvironmentalCommands::GET_ECC_ERROR_COUNTS);
    hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    hdr.reserved = 0;
    hdr.data_size = htole16(2);

    std::memcpy(buf.data(), &hdr, sizeof(hdr));

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode{};
    gpu::GetEccErrorCountsResponse decoded{};

    int result =
        gpu::decodeGetEccErrorCountsResponse(buf, cc, reasonCode, decoded);

    EXPECT_EQ(result, EINVAL);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

} // namespace gpu_mctp_tests
