/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensor.hpp"

#include <endian.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

// OcpAmi Tests
TEST(PackMessage, goodPathTest)
{
    OcpAmiBindingPciVidInfo hdr{};
    hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
    hdr.instance_id = 0;
    hdr.msg_type = 0x04;

    uint16_t pciVendorId{0x10de};

    OcpAmiBindingPciVid msg{};

    auto rc = ocpAmiPackHeader(pciVendorId, hdr, &msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    EXPECT_EQ(msg.pci_vendor_id, htobe16(pciVendorId));
    EXPECT_EQ(msg.reserved, 0);
    EXPECT_EQ(msg.datagram, 0);
    EXPECT_EQ(msg.request, 1);
    EXPECT_EQ(msg.ocp_type, ocpAmiType);
    EXPECT_EQ(msg.ocp_version, ocpAmiVersion);
    EXPECT_EQ(msg.ocp_ami_msg_type, hdr.msg_type);
    EXPECT_EQ(msg.instance_id, hdr.instance_id);
}

TEST(PackMessage, badPathTest)
{
    OcpAmiBindingPciVidInfo hdr{};
    uint16_t pciVendorId{};

    // Message pointer is NULL
    auto rc = ocpAmiPackHeader(pciVendorId, hdr, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Instance ID out of range
    OcpAmiBindingPciVid msg{};
    hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
    hdr.instance_id = 32;
    rc = ocpAmiPackHeader(pciVendorId, hdr, &msg);
    EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}

TEST(encodeReasonCode, testGoodEncodeReasonCode)
{
    std::vector<uint8_t> responseMsg(
        sizeof(OcpAmiBindingPciVid) + sizeof(OcpAmiCommonNonSuccessResponse));
    auto* response = new (responseMsg.data()) OcpAmiMessage;

    uint8_t cc = OCP_AMI_ERROR;
    uint16_t reasonCode = OCP_AMI_REASON_NONE;

    auto rc = ocpAmiEncodeReasonCode(cc, reasonCode, 0x00, response);

    auto* resp =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<OcpAmiCommonNonSuccessResponse*>(&response->data);

    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(OCP_AMI_ERROR, resp->completion_code);
    EXPECT_EQ(0x00, resp->command);
    EXPECT_EQ(OCP_AMI_REASON_NONE, le16toh(resp->reason_code));
}

TEST(encodeReasonCode, testBadEncodeReasonCode)
{
    uint8_t cc = OCP_AMI_ERROR;
    uint16_t reasonCode = OCP_AMI_REASON_NONE;

    auto rc = ocpAmiEncodeReasonCode(cc, reasonCode, 0x00, nullptr);

    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);
}

TEST(decodeReasonCodeCC, testGoodDecodeReasonCode)
{
    std::vector<uint8_t> responseMsg{
        0x10,
        0xDE, // PCI VID
        0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
        0x89, // OCP_TYPE=8, OCP_VER=9
        0x00, // MSG_TYPE
        0x09, // command
        0x01, // completion code != OCP_AMI_SUCCESS
        0x00, // reason code
        0x00};

    auto* response = new (responseMsg.data()) OcpAmiMessage;
    size_t msgLen = responseMsg.size();

    uint8_t cc = OCP_AMI_ERROR;
    uint16_t reasonCode = OCP_AMI_REASON_NONE;

    auto rc = ocpAmiDecodeReasonCodeAndCC(response, msgLen, &cc, &reasonCode);

    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(cc, OCP_AMI_ERROR);
    EXPECT_EQ(reasonCode, 0x0000);
}

TEST(decodeReasonCodeCC, testGoodDecodeCompletionCode)
{
    std::vector<uint8_t> responseMsg{
        0x10,
        0xDE, // PCI VID
        0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
        0x89, // OCP_TYPE=8, OCP_VER=9
        0x00, // MSG_TYPE
        0x09, // command
        0x00, // completion code = OCP_AMI_SUCCESS
        0x00, // reason code
        0x02};

    auto* response = new (responseMsg.data()) OcpAmiMessage;
    size_t msgLen = responseMsg.size();

    uint8_t cc = OCP_AMI_ERROR;
    uint16_t reasonCode = OCP_AMI_REASON_NONE;

    auto rc = ocpAmiDecodeReasonCodeAndCC(response, msgLen, &cc, &reasonCode);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(cc, OCP_AMI_SUCCESS);
    EXPECT_EQ(reasonCode, OCP_AMI_REASON_NONE);
}

TEST(decodeReasonCode, testBadDecodeReasonCode)
{
    std::vector<uint8_t> responseMsg{
        0x10,
        0xDE, // PCI VID
        0x00, // RQ=0, D=0, RSVD=0, INSTANCE_ID=0
        0x89, // OCP_TYPE=8, OCP_VER=9
        0x00, // MSG_TYPE
        0x09, // command
        0x01, // completion code
        0x00, // reason code
        0x00};

    auto* response = new (responseMsg.data()) OcpAmiMessage;
    size_t msgLen = responseMsg.size();

    uint8_t cc = OCP_AMI_SUCCESS;
    uint16_t reasonCode = OCP_AMI_REASON_NONE;

    auto rc = ocpAmiDecodeReasonCodeAndCC(nullptr, msgLen, &cc, &reasonCode);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    rc = ocpAmiDecodeReasonCodeAndCC(response, msgLen, nullptr, &reasonCode);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    rc = ocpAmiDecodeReasonCodeAndCC(response, msgLen, &cc, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    rc = ocpAmiDecodeReasonCodeAndCC(
        response, msgLen - 2, &cc,
        &reasonCode); // sending msg len less then expected
    EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA_LENGTH);
}

// GpuCommon Tests
TEST(GpuCommonPackTest, PackHeader)
{
    OcpAmiBindingPciVidInfo hdr{};
    OcpAmiBindingPciVid msg{};

    hdr.ocp_ami_msg_type = OCP_AMI_REQUEST;
    hdr.instance_id = 0x04;
    hdr.msg_type = 0x03;

    auto rc = gpuPackHeader(hdr, &msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(msg.ocp_version, ocpAmiVersion);

    // message pointer is NULL
    rc = gpuPackHeader(hdr, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Instance ID out of range
    hdr.instance_id = 32;
    rc = gpuPackHeader(hdr, &msg);
    EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}

class GpuCommonTest : public ::testing::Test
{
  protected:
    OcpAmiBindingPciVidInfo hdr{};
    OcpAmiMessage* msg{};
    std::vector<uint8_t> buf;
    uint8_t instance_id{};
    uint8_t type{};
    uint8_t command{};
    uint8_t cc{};
    uint16_t reason_code{};
    uint16_t data_size{};
    size_t msg_len{};
    uint16_t pci_vendor_id = gpuNvidiaPciVendorId;

    void SetUp() override
    {
        buf.resize(1024, 0);
        msg_len = buf.size();
        msg = new (buf.data()) OcpAmiMessage;
    }

    void setOcpVersionAndVendorId()
    {
        msg->hdr.ocp_type = ocpAmiType;
        msg->hdr.ocp_version = ocpAmiVersion;
        msg->hdr.pci_vendor_id = be16toh(gpuNvidiaPciVendorId);
    }

    void changeVendorId()
    {
        msg->hdr.pci_vendor_id = 0x1234;
    }
};

TEST_F(GpuCommonTest, EncodeReasonCode)
{
    auto rc = gpuEncodeReasonCode(cc, reason_code, command, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
}

TEST_F(GpuCommonTest, DecodeReasonCode)
{
    OcpAmiCompletionCode rc{};

    setOcpVersionAndVendorId();
    rc = gpuDecodeReasonCodeAndCC(msg, msg_len, &cc, &reason_code);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    changeVendorId();
    rc = gpuDecodeReasonCodeAndCC(msg, msg_len, &cc, &reason_code);
    EXPECT_EQ(rc, OCP_AMI_ERR_INVALID_DATA);
}

// GpuSensors Tests
class GpuSensorsTest : public ::testing::Test
{
  protected:
    OcpAmiBindingPciVidInfo hdr{};
    OcpAmiMessage* msg{};
    std::vector<uint8_t> buf;
    uint8_t instance_id = 0;
    uint8_t device_instance = 1;
    uint8_t device_id = GPU_DEVICE_GPU;
    uint8_t cc = OCP_AMI_SUCCESS;
    uint16_t reason_code = OCP_AMI_REASON_NONE;
    uint8_t sensor_id = 0;
    double temperature = 25.5;
    size_t msg_len{};

    void SetUp() override
    {
        buf.resize(1024, 0);
        msg = new (buf.data()) OcpAmiMessage;
        msg_len = buf.size();
    }
};

TEST_F(GpuSensorsTest, QueryDeviceIdentificationRequestEncode)
{
    auto rc = gpuEncodeQueryDeviceIdentificationRequest(instance_id, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Check that header is properly set
    EXPECT_EQ(msg->hdr.ocp_type, ocpAmiType);
    EXPECT_EQ(msg->hdr.ocp_version, ocpAmiVersion);
    EXPECT_EQ(msg->hdr.instance_id, instance_id);

    // Check payload
    auto* request =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuQueryDeviceIdentificationRequest*>(&msg->data);
    EXPECT_EQ(request->hdr.command, GPU_QUERY_DEVICE_IDENTIFICATION);
    EXPECT_EQ(request->hdr.data_size, 0);
}

TEST_F(GpuSensorsTest, QueryDeviceIdentificationResponseEncode)
{
    auto rc = gpuEncodeQueryDeviceIdentificationResponse(
        instance_id, cc, reason_code, device_id, device_instance, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Test with error condition
    uint8_t errorCc = OCP_AMI_ERROR;
    rc = gpuEncodeQueryDeviceIdentificationResponse(
        instance_id, errorCc, reason_code, device_id, device_instance, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
}

TEST_F(GpuSensorsTest, QueryDeviceIdentificationResponseDecode)
{
    // First encode a response
    auto rc = gpuEncodeQueryDeviceIdentificationResponse(
        instance_id, cc, reason_code, device_id, device_instance, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Then decode it
    uint8_t decodedCc{};
    uint16_t decodedReasonCode{};
    uint8_t decodedDeviceId{};
    uint8_t decodedDeviceInstance{};

    rc = gpuDecodeQueryDeviceIdentificationResponse(
        msg, msg_len, &decodedCc, &decodedReasonCode, &decodedDeviceId,
        &decodedDeviceInstance);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(decodedCc, cc);
    EXPECT_EQ(decodedReasonCode, reason_code);
    EXPECT_EQ(decodedDeviceId, device_id);
    EXPECT_EQ(decodedDeviceInstance, device_instance);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingRequestEncode)
{
    auto rc =
        gpuEncodeGetTemperatureReadingRequest(instance_id, sensor_id, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Check that header is properly set
    EXPECT_EQ(msg->hdr.ocp_type, ocpAmiType);
    EXPECT_EQ(msg->hdr.ocp_version, ocpAmiVersion);
    EXPECT_EQ(msg->hdr.instance_id, instance_id);

    // Check payload
    auto* request =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<GpuGetTemperatureReadingRequest*>(&msg->data);
    EXPECT_EQ(request->hdr.command, GPU_GET_TEMPERATURE_READING);
    EXPECT_EQ(request->hdr.data_size, sizeof(sensor_id));
    EXPECT_EQ(request->sensor_id, sensor_id);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingRequestDecode)
{
    // First encode a request
    auto rc =
        gpuEncodeGetTemperatureReadingRequest(instance_id, sensor_id, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Then decode it
    uint8_t decodedSensorId = 0;
    rc = gpuDecodeGetTemperatureReadingRequest(msg, msg_len, &decodedSensorId);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(decodedSensorId, sensor_id);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingResponseEncode)
{
    auto rc = gpuEncodeGetTemperatureReadingResponse(
        instance_id, cc, reason_code, temperature, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Test with error condition
    uint8_t errorCc = OCP_AMI_ERROR;
    rc = gpuEncodeGetTemperatureReadingResponse(instance_id, errorCc,
                                                reason_code, temperature, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingResponseDecode)
{
    // First encode a response
    auto rc = gpuEncodeGetTemperatureReadingResponse(
        instance_id, cc, reason_code, temperature, msg);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);

    // Then decode it
    uint8_t decodedCc{};
    uint16_t decodedReasonCode{};
    double decodedTemperature{};

    rc = gpuDecodeGetTemperatureReadingResponse(
        msg, msg_len, &decodedCc, &decodedReasonCode, &decodedTemperature);
    EXPECT_EQ(rc, OCP_AMI_SUCCESS);
    EXPECT_EQ(decodedCc, cc);
    EXPECT_EQ(decodedReasonCode, reason_code);
    EXPECT_DOUBLE_EQ(decodedTemperature, temperature);
}

TEST_F(GpuSensorsTest, NullPointerChecks)
{
    // Test encodeQueryDeviceIdentificationRequest
    auto rc = gpuEncodeQueryDeviceIdentificationRequest(instance_id, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test encodeQueryDeviceIdentificationResponse
    rc = gpuEncodeQueryDeviceIdentificationResponse(
        instance_id, cc, reason_code, device_id, device_instance, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test decodeQueryDeviceIdentificationResponse
    uint8_t decodedCc = 0;
    uint16_t decodedReasonCode = 0;
    uint8_t decodedDeviceId = 0;
    uint8_t decodedDeviceInstance = 0;

    rc = gpuDecodeQueryDeviceIdentificationResponse(
        msg, msg_len, &decodedCc, &decodedReasonCode, nullptr,
        &decodedDeviceInstance);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    rc = gpuDecodeQueryDeviceIdentificationResponse(
        msg, msg_len, &decodedCc, &decodedReasonCode, &decodedDeviceId,
        nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test encodeGetTemperatureReadingRequest
    rc = gpuEncodeGetTemperatureReadingRequest(instance_id, sensor_id, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test decodeGetTemperatureReadingRequest
    uint8_t decodedSensorId = 0;
    rc = gpuDecodeGetTemperatureReadingRequest(nullptr, msg_len,
                                               &decodedSensorId);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    rc = gpuDecodeGetTemperatureReadingRequest(msg, msg_len, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test encodeGetTemperatureReadingResponse
    rc = gpuEncodeGetTemperatureReadingResponse(instance_id, cc, reason_code,
                                                temperature, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);

    // Test decodeGetTemperatureReadingResponse
    rc = gpuDecodeGetTemperatureReadingResponse(msg, msg_len, &decodedCc,
                                                &decodedReasonCode, nullptr);
    EXPECT_EQ(rc, OCP_AMI_ERR_NULL);
}
