/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuMctpVdm.hpp"

#include <endian.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

TEST(PackMessage, goodPathTest)
{
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 0;
    hdr.msg_type = 0x04;

    uint16_t pciVendorId{0x10de};

    ocp::accelerator_management::BindingPciVid msg{};

    auto rc = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    EXPECT_EQ(msg.pci_vendor_id, htobe16(pciVendorId));
    EXPECT_EQ(msg.reserved, 0);
    EXPECT_EQ(msg.datagram, 0);
    EXPECT_EQ(msg.request, 1);
    EXPECT_EQ(msg.ocp_type, ocp::accelerator_management::type);
    EXPECT_EQ(msg.ocp_version, ocp::accelerator_management::version);
    EXPECT_EQ(msg.ocp_accelerator_management_msg_type, hdr.msg_type);
    EXPECT_EQ(msg.instance_id, hdr.instance_id);
}

TEST(PackMessage, badPathTest)
{
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    uint16_t pciVendorId{};

    // Message pointer is NULL test is no longer valid with references
    // However, we'll leave this test block for completeness

    // Instance ID out of range
    ocp::accelerator_management::BindingPciVid msg{};
    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 32;
    auto rc = ocp::accelerator_management::packHeader(pciVendorId, hdr, msg);
    EXPECT_EQ(rc,
              ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA);
}

TEST(encodeReasonCode, testGoodEncodeReasonCode)
{
    std::vector<uint8_t> responseMsg(
        sizeof(ocp::accelerator_management::BindingPciVid) +
        sizeof(ocp::accelerator_management::CommonNonSuccessResponse));
    auto* response = new (responseMsg.data())
        ocp::accelerator_management::Message;

    uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    uint16_t reasonCode = static_cast<uint16_t>(
        ocp::accelerator_management::ReasonCode::REASON_NONE);

    auto rc = ocp::accelerator_management::encodeReasonCode(
        cc, reasonCode, 0x00, *response);

    ocp::accelerator_management::CommonNonSuccessResponse resp{};
    std::memcpy(&resp, &response->data, sizeof(resp));

    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(static_cast<uint8_t>(
                  ocp::accelerator_management::CompletionCode::ERROR),
              resp.completion_code);
    EXPECT_EQ(0x00, resp.command);
    EXPECT_EQ(static_cast<uint16_t>(
                  ocp::accelerator_management::ReasonCode::REASON_NONE),
              le16toh(resp.reason_code));
}

TEST(encodeReasonCode, testBadEncodeReasonCode)
{
    // We cannot test null pointer with references
    // This test is no longer applicable
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
        0x01, // completion code !=
              // ocp::accelerator_management::CompletionCode::SUCCESS
        0x00, // reason code
        0x00};

    auto* response = new (responseMsg.data())
        ocp::accelerator_management::Message;
    size_t msgLen = responseMsg.size();

    uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    uint16_t reasonCode = static_cast<uint16_t>(
        ocp::accelerator_management::ReasonCode::REASON_NONE);

    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        *response, msgLen, cc, reasonCode);

    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(cc, static_cast<uint8_t>(
                      ocp::accelerator_management::CompletionCode::ERROR));
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
        0x00, // completion code =
              // ocp::accelerator_management::CompletionCode::SUCCESS
        0x00, // reason code
        0x02};

    auto* response = new (responseMsg.data())
        ocp::accelerator_management::Message;
    size_t msgLen = responseMsg.size();

    uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    uint16_t reasonCode = static_cast<uint16_t>(
        ocp::accelerator_management::ReasonCode::REASON_NONE);

    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        *response, msgLen, cc, reasonCode);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(cc, static_cast<uint8_t>(
                      ocp::accelerator_management::CompletionCode::SUCCESS));
    EXPECT_EQ(reasonCode,
              static_cast<uint16_t>(
                  ocp::accelerator_management::ReasonCode::REASON_NONE));
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

    auto* response = new (responseMsg.data())
        ocp::accelerator_management::Message;
    size_t msgLen = responseMsg.size();

    uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    uint16_t reasonCode = static_cast<uint16_t>(
        ocp::accelerator_management::ReasonCode::REASON_NONE);

    // Null pointer tests are no longer applicable with references

    auto rc = ocp::accelerator_management::decodeReasonCodeAndCC(
        *response, msgLen - 2, cc,
        reasonCode); // sending msg len less then expected
    EXPECT_EQ(
        rc,
        ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA_LENGTH);
}

TEST(GpuCommonPackTest, PackHeader)
{
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::BindingPciVid msg{};

    hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(ocp::accelerator_management::MessageType::REQUEST);
    hdr.instance_id = 0x04;
    hdr.msg_type = 0x03;

    auto rc = gpu::packHeader(hdr, msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(msg.ocp_version, ocp::accelerator_management::version);

    // Null pointer test is no longer applicable with references

    // Instance ID out of range
    hdr.instance_id = 32;
    rc = gpu::packHeader(hdr, msg);
    EXPECT_EQ(rc,
              ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA);
}

class GpuCommonTest : public ::testing::Test
{
  protected:
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::Message* msg{};
    std::vector<uint8_t> buf;
    uint8_t instance_id{};
    uint8_t type{};
    uint8_t command{};
    uint8_t cc{};
    uint16_t reason_code{};
    uint16_t data_size{};
    size_t msg_len{};
    uint16_t pci_vendor_id = gpu::nvidiaPciVendorId;

    void SetUp() override
    {
        buf.resize(1024, 0);
        msg_len = buf.size();
        msg = new (buf.data()) ocp::accelerator_management::Message;
    }

    void setOcpVersionAndVendorId()
    {
        msg->hdr.ocp_type = ocp::accelerator_management::type;
        msg->hdr.ocp_version = ocp::accelerator_management::version;
        msg->hdr.pci_vendor_id = be16toh(gpu::nvidiaPciVendorId);
    }

    void changeVendorId()
    {
        msg->hdr.pci_vendor_id = 0x1234;
    }
};

TEST_F(GpuCommonTest, EncodeReasonCode)
{
    auto rc = gpu::encodeReasonCode(cc, reason_code, command, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
}

TEST_F(GpuCommonTest, DecodeReasonCode)
{
    ocp::accelerator_management::CompletionCode rc{};

    setOcpVersionAndVendorId();
    rc = gpu::decodeReasonCodeAndCC(*msg, msg_len, cc, reason_code);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    changeVendorId();
    rc = gpu::decodeReasonCodeAndCC(*msg, msg_len, cc, reason_code);
    EXPECT_EQ(rc,
              ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA);
}

class GpuSensorsTest : public ::testing::Test
{
  protected:
    ocp::accelerator_management::BindingPciVidInfo hdr{};
    ocp::accelerator_management::Message* msg{};
    std::vector<uint8_t> buf;
    uint8_t instance_id = 0;
    uint8_t device_instance = 1;
    uint8_t device_id =
        static_cast<uint8_t>(gpu::DeviceIdentification::DEVICE_GPU);
    uint8_t cc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    uint16_t reason_code = static_cast<uint16_t>(
        ocp::accelerator_management::ReasonCode::REASON_NONE);
    uint8_t sensor_id = 0;
    double temperature = 25.5;
    size_t msg_len{};

    void SetUp() override
    {
        buf.resize(1024, 0);
        msg = new (buf.data()) ocp::accelerator_management::Message;
        msg_len = buf.size();
    }
};

TEST_F(GpuSensorsTest, QueryDeviceIdentificationRequestEncode)
{
    auto rc = gpu::encodeQueryDeviceIdentificationRequest(instance_id, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Check that header is properly set
    EXPECT_EQ(msg->hdr.ocp_type, ocp::accelerator_management::type);
    EXPECT_EQ(msg->hdr.ocp_version, ocp::accelerator_management::version);
    EXPECT_EQ(msg->hdr.instance_id, instance_id);

    // Check payload
    gpu::QueryDeviceIdentificationRequest request{};
    std::memcpy(&request, &msg->data, sizeof(request));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(gpu::DeviceCapabilityDiscoveryCommands::
                                       QUERY_DEVICE_IDENTIFICATION));
    EXPECT_EQ(request.hdr.data_size, 0);
}

TEST_F(GpuSensorsTest, QueryDeviceIdentificationResponseEncode)
{
    auto rc = gpu::encodeQueryDeviceIdentificationResponse(
        instance_id, cc, reason_code, device_id, device_instance, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Test with error condition
    uint8_t errorCc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    rc = gpu::encodeQueryDeviceIdentificationResponse(
        instance_id, errorCc, reason_code, device_id, device_instance, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
}

TEST_F(GpuSensorsTest, QueryDeviceIdentificationResponseDecode)
{
    // First encode a response
    auto rc = gpu::encodeQueryDeviceIdentificationResponse(
        instance_id, cc, reason_code, device_id, device_instance, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Then decode it
    uint8_t decodedCc{};
    uint16_t decodedReasonCode{};
    uint8_t decodedDeviceId{};
    uint8_t decodedDeviceInstance{};

    rc = gpu::decodeQueryDeviceIdentificationResponse(
        *msg, msg_len, decodedCc, decodedReasonCode, decodedDeviceId,
        decodedDeviceInstance);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(decodedCc, cc);
    EXPECT_EQ(decodedReasonCode, reason_code);
    EXPECT_EQ(decodedDeviceId, device_id);
    EXPECT_EQ(decodedDeviceInstance, device_instance);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingRequestEncode)
{
    auto rc =
        gpu::encodeGetTemperatureReadingRequest(instance_id, sensor_id, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Check that header is properly set
    EXPECT_EQ(msg->hdr.ocp_type, ocp::accelerator_management::type);
    EXPECT_EQ(msg->hdr.ocp_version, ocp::accelerator_management::version);
    EXPECT_EQ(msg->hdr.instance_id, instance_id);

    // Check payload
    gpu::GetTemperatureReadingRequest request{};
    std::memcpy(&request, &msg->data, sizeof(request));

    EXPECT_EQ(request.hdr.command,
              static_cast<uint8_t>(
                  gpu::PlatformEnvironmentalCommands::GET_TEMPERATURE_READING));
    EXPECT_EQ(request.hdr.data_size, sizeof(sensor_id));
    EXPECT_EQ(request.sensor_id, sensor_id);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingRequestDecode)
{
    // First encode a request
    auto rc =
        gpu::encodeGetTemperatureReadingRequest(instance_id, sensor_id, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Then decode it
    uint8_t decodedSensorId = 0;
    rc =
        gpu::decodeGetTemperatureReadingRequest(*msg, msg_len, decodedSensorId);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(decodedSensorId, sensor_id);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingResponseEncode)
{
    auto rc = gpu::encodeGetTemperatureReadingResponse(
        instance_id, cc, reason_code, temperature, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Test with error condition
    uint8_t errorCc = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERROR);
    rc = gpu::encodeGetTemperatureReadingResponse(
        instance_id, errorCc, reason_code, temperature, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
}

TEST_F(GpuSensorsTest, GetTemperatureReadingResponseDecode)
{
    // First encode a response
    auto rc = gpu::encodeGetTemperatureReadingResponse(
        instance_id, cc, reason_code, temperature, *msg);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);

    // Then decode it
    uint8_t decodedCc{};
    uint16_t decodedReasonCode{};
    double decodedTemperature{};

    rc = gpu::decodeGetTemperatureReadingResponse(
        *msg, msg_len, decodedCc, decodedReasonCode, decodedTemperature);
    EXPECT_EQ(rc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(decodedCc, cc);
    EXPECT_EQ(decodedReasonCode, reason_code);
    EXPECT_DOUBLE_EQ(decodedTemperature, temperature);
}
