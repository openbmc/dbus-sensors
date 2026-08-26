/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpMockTestBase.hpp"
#include "MessagePackUnpackUtils.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaLldpTlvs.hpp"
#include "OcpMctpVdm.hpp"

// NOLINTNEXTLINE(misc-include-cleaner): sd_bus_error lives here
#include <systemd/sd-bus.h>

#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{

constexpr const char* tlvsIface = "xyz.openbmc_project.Network.LLDP.TLVs";
constexpr const char* subtypePrefix =
    "xyz.openbmc_project.Network.LLDP.TLVs.IdSubtype.";
constexpr const char* directionPrefix =
    "xyz.openbmc_project.Network.LLDP.TLVs.Direction.";

constexpr const char* portName = "Port_1";

// An interface can only be registered on a path once per process, so each
// test works on a path of its own rather than inheriting whatever the test
// before it left registered.
std::string deviceNameFor()
{
    return std::string("CX_") +
           testing::UnitTest::GetInstance()->current_test_info()->name();
}

std::string portPath()
{
    return "/xyz/openbmc_project/inventory/" + deviceNameFor() + "/Port_1";
}

std::string receivedPath()
{
    return "/xyz/openbmc_project/network/lldp/" + deviceNameFor() +
           "/Port_1/received";
}

// A frame as it travels: the address every LLDP frame is sent to, a source
// address, the type that says what follows, and only then the unit.
std::vector<uint8_t> withEthernetHeader(const std::vector<uint8_t>& unit)
{
    std::vector<uint8_t> frame = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E, 0xF0,
                                  0xFB, 0x7F, 0xA4, 0x5C, 0x0C, 0x88, 0xCC};
    frame.insert(frame.end(), unit.begin(), unit.end());
    // A link pads a frame out to its own minimum length.
    frame.resize(std::max<size_t>(frame.size(), 60), 0);
    return frame;
}

constexpr uint8_t testEid = 42;
constexpr uint16_t testPort = 1;

constexpr std::chrono::seconds pumpTimeout{5};

void appendField(std::vector<uint8_t>& pdu, uint8_t type,
                 std::initializer_list<uint8_t> value)
{
    const size_t length = value.size();
    pdu.push_back(static_cast<uint8_t>((type << 1) | ((length >> 8) & 0x01)));
    pdu.push_back(static_cast<uint8_t>(length & 0xFF));
    pdu.insert(pdu.end(), value.begin(), value.end());
}

// A frame a switch would send: chassis identified by MAC, port by name, a
// time to live, a system name and the end marker.
std::vector<uint8_t> buildFrame()
{
    std::vector<uint8_t> pdu;
    appendField(pdu, 1, {4, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x00, 0x78});
    appendField(pdu, 5, {'s', 'w', '1'});
    appendField(pdu, 0, {});
    return pdu;
}

std::vector<uint8_t> buildPacketResponse(const std::vector<uint8_t>& frame)
{
    std::vector<uint8_t> buf(
        ocp::accelerator_management::commonResponseSize + frame.size());
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(gpu::NetworkPortCommands::GetLldpPacket));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::SUCCESS));
    pack.pack(static_cast<uint16_t>(0)); // reserved
    pack.pack(static_cast<uint16_t>(frame.size()));
    for (const uint8_t byte : frame)
    {
        pack.pack(byte);
    }
    return buf;
}

std::vector<uint8_t> buildEmptyPortResponse()
{
    std::vector<uint8_t> buf(ocp::accelerator_management::commonResponseSize);
    PackBuffer pack(buf);
    ocp::accelerator_management::packHeader(
        pack, gpu::nvidiaPciVendorId,
        ocp::accelerator_management::MessageType::RESPONSE, 0,
        static_cast<uint8_t>(gpu::MessageType::NETWORK_PORT));
    pack.pack(static_cast<uint8_t>(gpu::NetworkPortCommands::GetLldpPacket));
    pack.pack(static_cast<uint8_t>(
        ocp::accelerator_management::CompletionCode::ERR_NOT_READY));
    pack.pack(static_cast<uint16_t>(0));
    pack.pack(static_cast<uint16_t>(0));
    return buf;
}

class NvidiaLldpTlvsTest : public MctpMockTestBase
{
  protected:
    void SetUp() override
    {
        MctpMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        ON_CALL(mctpMock, sendRecvMsg)
            .WillByDefault(
                [this](uint8_t /*eid*/, std::span<const uint8_t> /*request*/,
                       auto callback) {
                    const std::vector<uint8_t> response =
                        held.empty() ? buildEmptyPortResponse()
                                     : buildPacketResponse(held);
                    callback(std::error_code{}, response);
                });
    }

    static std::shared_ptr<NvidiaLldpTlvs> makeTlvs(
        gpu::LldpPacketType direction)
    {
        auto tlvs = std::make_shared<NvidiaLldpTlvs>(
            objects(), requester(), deviceNameFor(), portName,
            sdbusplus::object_path(portPath()), testEid, testPort, direction);
        tlvs->update();
        return tlvs;
    }

    // What the port holds; empty means it holds no frame.
    std::vector<uint8_t> held;
};

TEST_F(NvidiaLldpTlvsTest, ReportsWhatTheFrameSays)
{
    held = buildFrame();
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_TRUE(pumpIoUntil(
        [] {
            return getProperty<std::string>(receivedPath(), tlvsIface,
                                            "ChassisId") == "00:11:22:33:44:55";
        },
        pumpTimeout));
    EXPECT_EQ(getProperty<std::string>(receivedPath(), tlvsIface, "PortId"),
              "eth0");
    EXPECT_EQ(getProperty<std::string>(receivedPath(), tlvsIface, "SystemName"),
              "sw1");
    EXPECT_EQ(
        getProperty<std::string>(receivedPath(), tlvsIface, "ChassisIdSubtype"),
        std::string(subtypePrefix) + "MacAddr");
}

// A consumer reads the direction rather than the object path, so the property
// has to carry it.
TEST_F(NvidiaLldpTlvsTest, SaysWhichDirectionItDescribes)
{
    auto received = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_EQ(getProperty<std::string>(receivedPath(), tlvsIface, "Direction"),
              std::string(directionPrefix) + "Received");
}

TEST_F(NvidiaLldpTlvsTest, PointsAtThePortItDescribes)
{
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    const auto associations = getProperty<
        std::vector<std::tuple<std::string, std::string, std::string>>>(
        receivedPath(), "xyz.openbmc_project.Association.Definitions",
        "Associations");

    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(std::get<0>(associations.front()), "monitoring");
    EXPECT_EQ(std::get<1>(associations.front()), "monitored_by");
    EXPECT_EQ(std::get<2>(associations.front()), portPath());
}

// A port with no link partner still has the object; it just has nothing to
// say through it.
TEST_F(NvidiaLldpTlvsTest, ExistsEvenWhenThePortHoldsNothing)
{
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_TRUE(getProperty<std::string>(receivedPath(), tlvsIface, "ChassisId")
                    .empty());
    EXPECT_EQ(
        getProperty<std::string>(receivedPath(), tlvsIface, "ChassisIdSubtype"),
        std::string(subtypePrefix) + "NotTransmitted");
}

// A partner that goes away must not leave what it used to say behind.
TEST_F(NvidiaLldpTlvsTest, ForgetsAFrameThePortNoLongerHolds)
{
    held = buildFrame();
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);
    ASSERT_TRUE(pumpIoUntil(
        [] {
            return !getProperty<std::string>(receivedPath(), tlvsIface,
                                             "ChassisId")
                        .empty();
        },
        pumpTimeout));

    held.clear();
    tlvs->update();

    EXPECT_TRUE(pumpIoUntil(
        [] {
            return getProperty<std::string>(receivedPath(), tlvsIface,
                                            "ChassisId")
                .empty();
        },
        pumpTimeout));
}

// A frame that cannot be made sense of is worth no more than no frame at all,
// and must not be reported as a partial one.
TEST_F(NvidiaLldpTlvsTest, ReportsNothingForAFrameItCannotRead)
{
    held = buildFrame();
    held.resize(held.size() - 3);
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_TRUE(pumpIoUntil([] { return true; }, std::chrono::seconds{1}));
    EXPECT_TRUE(getProperty<std::string>(receivedPath(), tlvsIface, "ChassisId")
                    .empty());
}

// The device reports the frame as it travelled rather than the unit alone,
// which is the form the hardware actually sends.
TEST_F(NvidiaLldpTlvsTest, ReadsAFrameThatArrivedWithItsAddresses)
{
    held = withEthernetHeader(buildFrame());
    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_TRUE(pumpIoUntil(
        [] {
            return getProperty<std::string>(receivedPath(), tlvsIface,
                                            "ChassisId") == "00:11:22:33:44:55";
        },
        pumpTimeout));
    EXPECT_EQ(getProperty<std::string>(receivedPath(), tlvsIface, "SystemName"),
              "sw1");
}

// A unit that happens to carry the type that names LLDP where a header would
// put it is still a unit, so it must not be mistaken for a framed one.
TEST_F(NvidiaLldpTlvsTest, DoesNotMistakeAUnitForAFramedOne)
{
    std::vector<uint8_t> pdu;
    // A chassis identifier long enough to reach the offset a type sits at,
    // holding those very bytes.
    appendField(pdu, 1,
                {4, 0x88, 0xCC, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                 0x88, 0xCC, 0x99});
    appendField(pdu, 2, {5, 'e', 't', 'h', '0'});
    appendField(pdu, 3, {0x00, 0x78});
    appendField(pdu, 0, {});
    held = pdu;

    auto tlvs = makeTlvs(gpu::LldpPacketType::Received);

    EXPECT_TRUE(pumpIoUntil(
        [] {
            return getProperty<std::string>(receivedPath(), tlvsIface,
                                            "PortId") == "eth0";
        },
        pumpTimeout));
}

} // namespace
