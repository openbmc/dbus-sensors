/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
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
        static_cast<uint8_t>(gpu::MessageType::DEVICE_CAPABILITY_DISCOVERY);

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
} // namespace gpu_mctp_tests
