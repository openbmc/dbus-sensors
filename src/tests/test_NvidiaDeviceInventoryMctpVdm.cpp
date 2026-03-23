#include "MessagePackUnpackUtils.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

using namespace gpu;

TEST(NvidiaGpuMctpVdmTest, EncodeGetInventoryInformationRequest)
{
    std::array<uint8_t, 256> buf{};
    uint8_t instanceId = 1;
    uint8_t propertyId =
        static_cast<uint8_t>(InventoryPropertyId::BOARD_PART_NUMBER);

    auto rc = encodeGetInventoryInformationRequest(instanceId, propertyId, buf);
    EXPECT_EQ(rc, 0);

    UnpackBuffer ubuf(std::span<const uint8_t>(buf.data(), buf.size()));
    ocp::accelerator_management::MessageType msgType{};
    uint8_t recvInstanceId = 0;
    uint8_t recvMsgType = 0;
    EXPECT_EQ(ocp::accelerator_management::unpackHeader(
                  ubuf, gpu::nvidiaPciVendorId, msgType, recvInstanceId,
                  recvMsgType),
              0);

    uint8_t command = 0;
    ubuf.unpack(command);
    EXPECT_EQ(command,
              static_cast<uint8_t>(
                  PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    uint8_t dataSize = 0;
    ubuf.unpack(dataSize);
    EXPECT_EQ(dataSize, sizeof(propertyId));
    uint8_t recvPropertyId = 0;
    ubuf.unpack(recvPropertyId);
    EXPECT_EQ(recvPropertyId, propertyId);
    EXPECT_EQ(ubuf.getError(), 0);
}

TEST(NvidiaGpuMctpVdmTest, DecodeInventoryString)
{
    std::array<uint8_t, 256> buf{};
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(static_cast<uint16_t>(0)); // reserved
    pbuf.pack(static_cast<uint16_t>(5)); // data_size = 5 for "TEST1"
    // Pack the string bytes manually
    const char* testStr = "TEST1";
    for (int i = 0; i < 5; i++)
    {
        pbuf.pack(static_cast<uint8_t>(testStr[i]));
    }
    ASSERT_EQ(pbuf.getError(), 0);

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
    PackBuffer pbuf(buf);
    ocp::accelerator_management::packHeader(
        pbuf, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 1,
        static_cast<uint8_t>(MessageType::PLATFORM_ENVIRONMENTAL));
    pbuf.pack(static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_INVENTORY_INFORMATION));
    pbuf.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pbuf.pack(static_cast<uint16_t>(0)); // reserved
    pbuf.pack(static_cast<uint16_t>(8)); // data_size = 8 for DEVICE_GUID

    std::vector<uint8_t> dummyGuid = {0xDE, 0xAD, 0xBE, 0xEF,
                                      0x01, 0x23, 0x45, 0x67};
    for (const auto& byte : dummyGuid)
    {
        pbuf.pack(byte);
    }
    ASSERT_EQ(pbuf.getError(), 0);

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
