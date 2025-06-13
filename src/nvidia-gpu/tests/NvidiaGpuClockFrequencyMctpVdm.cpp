#include "NvidiaGpuMctpVdm.hpp"
#include "OcpMctpVdm.hpp"

#include <endian.h>

#include <array>
#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

using namespace gpu;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

TEST(NvidiaGpuClockFrequencyMctpVdmTest, EncodeGetCurrentClockFrequencyRequest)
{
    std::array<uint8_t, 256> buf{};
    uint8_t instanceId = 1;
    uint8_t clockId = static_cast<uint8_t>(ClockType::GRAPHICS_CLOCK);

    auto rc = encodeGetCurrentClockFrequencyRequest(instanceId, clockId, buf);
    EXPECT_EQ(rc, 0);

    auto* msg = reinterpret_cast<GetCurrentClockFrequencyRequest*>(buf.data());
    EXPECT_EQ(msg->hdr.command,
              static_cast<uint8_t>(
                  PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY));
    EXPECT_EQ(msg->hdr.data_size, sizeof(clockId));
    EXPECT_EQ(msg->clock_id, clockId);
}

TEST(NvidiaGpuClockFrequencyMctpVdmTest, DecodeGetCurrentClockFrequencyResponse)
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
        PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS);
    response->reserved = 0;
    response->data_size =
        htole16(sizeof(uint32_t)); // 4 bytes for clock frequency

    // Set clock frequency to 1500 MHz
    uint32_t clockFreq = htole32(1500);
    memcpy(buf.data() + sizeof(ocp::accelerator_management::CommonResponse),
           &clockFreq, sizeof(clockFreq));

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::SUCCESS;
    uint16_t reasonCode = 0;
    uint32_t frequency = 0;

    auto rc =
        decodeGetCurrentClockFrequencyResponse(buf, cc, reasonCode, frequency);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc, ocp::accelerator_management::CompletionCode::SUCCESS);
    EXPECT_EQ(reasonCode, 0);
    EXPECT_EQ(frequency, 1500);
}

TEST(NvidiaGpuClockFrequencyMctpVdmTest,
     DecodeGetCurrentClockFrequencyResponseError)
{
    std::array<uint8_t, 256> buf{};
    auto* response =
        reinterpret_cast<ocp::accelerator_management::CommonResponse*>(
            buf.data());

    // Fill header with error response
    response->msgHdr.hdr.pci_vendor_id = htobe16(0x10DE);
    response->msgHdr.hdr.instance_id = 0x01;
    response->msgHdr.hdr.ocp_version = 0x89;
    response->msgHdr.hdr.ocp_accelerator_management_msg_type =
        static_cast<uint8_t>(
            ocp::accelerator_management::MessageType::RESPONSE);

    response->command = static_cast<uint8_t>(
        PlatformEnvironmentalCommands::GET_CURRENT_CLOCK_FREQUENCY);
    response->completion_code = static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA);
    response->reserved = 0;
    response->data_size = htole16(0);

    ocp::accelerator_management::CompletionCode cc =
        ocp::accelerator_management::CompletionCode::SUCCESS;
    uint16_t reasonCode = 0;
    uint32_t frequency = 0;

    auto rc =
        decodeGetCurrentClockFrequencyResponse(buf, cc, reasonCode, frequency);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(cc,
              ocp::accelerator_management::CompletionCode::ERR_INVALID_DATA);
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
