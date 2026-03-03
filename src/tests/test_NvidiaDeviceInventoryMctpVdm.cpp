#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

using namespace gpu;
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
TEST(NvidiaGpuMctpVdmTest, EncodeGetInventoryInformationRequest)
{
    std::array<uint8_t, 256> buf{};
    uint8_t instanceId = 1;
    uint8_t propertyId =
        static_cast<uint8_t>(InventoryPropertyId::BOARD_PART_NUMBER);

    auto rc = encodeGetInventoryInformationRequest(instanceId, propertyId, buf);
    EXPECT_EQ(rc, 0);

    auto* msg = reinterpret_cast<GetInventoryInformationRequest*>(buf.data());
    EXPECT_EQ(msg->hdr.command,
              static_cast<uint8_t>(
                  PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    EXPECT_EQ(msg->hdr.data_size, sizeof(propertyId));
    EXPECT_EQ(msg->property_id, propertyId);
}

TEST(NvidiaGpuMctpVdmTest, DecodeInventoryString)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<ocp::accelerator_management::CommonResponse*>(
            buf.data());

    // Fill header
    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE); // NVIDIA vendor ID
    response->msgHdr.hdr.instance_id = 0x01;              // Instance ID
    response->msgHdr.hdr.ocp_version = 0x89; // OCP version and type
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(5); // 5 bytes for "TEST1"

    const char* testStr = "TEST1";
    memcpy(buf.data() + sizeof(ocp::accelerator_management::CommonResponse),
           testStr, 5);

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::ERROR;
    uint16_t reasonCode = 0;
    InventoryValue info;

    auto rc = decodeGetInventoryInformationResponse(
        buf, cc, reasonCode, InventoryPropertyId::BOARD_PART_NUMBER, info);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_TRUE(std::holds_alternative<std::string>(info));
    EXPECT_EQ(std::get<std::string>(info), "TEST1");
}

TEST(NvidiaGpuMctpVdmTest, DecodeInventoryDeviceGuid)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<ocp::accelerator_management::CommonResponse*>(
            buf.data());

    // Fill header
    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE); // NVIDIA vendor ID
    response->msgHdr.hdr.instance_id = 0x01;              // Instance ID
    response->msgHdr.hdr.ocp_version = 0x89; // OCP version and type
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(8); // 8 bytes for DEVICE_GUID

    std::vector<uint8_t> dummyGuid = {0xDE, 0xAD, 0xBE, 0xEF,
                                      0x01, 0x23, 0x45, 0x67};
    memcpy(buf.data() + sizeof(ocp::accelerator_management::CommonResponse),
           dummyGuid.data(), dummyGuid.size());

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::ERROR;
    uint16_t reasonCode = 0;
    InventoryValue info;

    auto rc = decodeGetInventoryInformationResponse(
        buf, cc, reasonCode, InventoryPropertyId::DEVICE_GUID, info);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_TRUE(std::holds_alternative<std::vector<uint8_t>>(info));
    EXPECT_EQ(std::get<std::vector<uint8_t>>(info), dummyGuid);
}

TEST(NvidiaGpuMctpVdmTest, EncodeGetCurrentClockFrequencyRequest)
{
    std::array<uint8_t, 256> buf{};

    auto rc =
        encodeGetCurrentClockFrequencyRequest(1, ClockType::MEMORY_CLOCK, buf);
    EXPECT_EQ(rc, 0);

    auto* msg = reinterpret_cast<GetCurrentClockFrequencyRequest*>(buf.data());
    EXPECT_EQ(msg->hdr.command,
              static_cast<uint8_t>(
                  PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    EXPECT_EQ(msg->hdr.data_size, sizeof(msg->clockId));
    EXPECT_EQ(msg->clockId, static_cast<uint8_t>(ClockType::MEMORY_CLOCK));
}

TEST(NvidiaGpuMctpVdmTest, DecodeGetCurrentClockFrequencySuccess)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<GetCurrentClockFrequencyResponse*>(buf.data());

    response->hdr.msgHdr.hdr.pci_vendor_id = htobe16(0x10DE);
    response->hdr.msgHdr.hdr.instance_id = 0x01;
    response->hdr.msgHdr.hdr.ocp_version = 0x89;
    response->hdr.msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->hdr.command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY);
    response->hdr.completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->hdr.reserved = 0;
    response->hdr.data_size = htole16(sizeof(uint32_t));
    response->clockFreq = htole32(405U);

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::ERROR;
    uint16_t reasonCode = 0;
    uint32_t clockFreq = 0;

    auto rc =
        decodeGetCurrentClockFrequencyResponse(buf, cc, reasonCode, clockFreq);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(clockFreq, 405U);
}

TEST(NvidiaGpuMctpVdmTest, DecodeGetCurrentClockFrequencyError)
{
    std::array<uint8_t, 256> buf{};
    auto* response = reinterpret_cast<
        ocp::accelerator_management::CommonNonSuccessResponse*>(buf.data());

    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE);
    response->msgHdr.hdr.instance_id = 0x01;
    response->msgHdr.hdr.ocp_version = 0x89;
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    response->reason_code = htole16(0x5678);

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::SUCCESS;
    uint16_t reasonCode = 0;
    uint32_t clockFreq = 0;

    auto rc =
        decodeGetCurrentClockFrequencyResponse(buf, cc, reasonCode, clockFreq);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::ERR_NOT_READY);
    EXPECT_EQ(reasonCode, 0x5678);
}

TEST(NvidiaGpuMctpVdmTest, DecodeInventoryMinMemoryClockSuccess)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<ocp::accelerator_management::CommonResponse*>(
            buf.data());

    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE);
    response->msgHdr.hdr.instance_id = 0x01;
    response->msgHdr.hdr.ocp_version = 0x89;
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(sizeof(uint32_t));

    uint32_t minClock = htole32(405U);
    memcpy(buf.data() + sizeof(ocp::accelerator_management::CommonResponse),
           &minClock, sizeof(minClock));

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::ERROR;
    uint16_t reasonCode = 0;
    InventoryValue info;

    auto rc = decodeGetInventoryInformationResponse(
        buf, cc, reasonCode, InventoryPropertyId::MIN_MEMORY_CLOCK, info);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_TRUE(std::holds_alternative<uint32_t>(info));
    EXPECT_EQ(std::get<uint32_t>(info), 405U);
}

TEST(NvidiaGpuMctpVdmTest, DecodeInventoryMaxMemoryClockSuccess)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<ocp::accelerator_management::CommonResponse*>(
            buf.data());

    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE);
    response->msgHdr.hdr.instance_id = 0x01;
    response->msgHdr.hdr.ocp_version = 0x89;
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size = htole16(sizeof(uint32_t));

    uint32_t maxClock = htole32(12481U);
    memcpy(buf.data() + sizeof(ocp::accelerator_management::CommonResponse),
           &maxClock, sizeof(maxClock));

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::ERROR;
    uint16_t reasonCode = 0;
    InventoryValue info;

    auto rc = decodeGetInventoryInformationResponse(
        buf, cc, reasonCode, InventoryPropertyId::MAX_MEMORY_CLOCK, info);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_TRUE(std::holds_alternative<uint32_t>(info));
    EXPECT_EQ(std::get<uint32_t>(info), 12481U);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
