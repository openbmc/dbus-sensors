/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "LldpFrame.hpp"
#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// What one port of a network device advertises, or hears its link partner
// advertise, through the Link Layer Discovery Protocol.
//
// A port holds one frame per direction, so there is one of these per port and
// direction. The object is not an inventory item, so it lives outside the
// inventory tree and points at the port it describes through an association.
//
// The object exists whether or not the port holds a frame. A port with no
// link partner, one whose partner does not speak the protocol, and one whose
// agent is switched off all report the same thing: nothing. Making the object
// come and go would turn that into a client's problem to track.
class NvidiaLldpTlvs : public std::enable_shared_from_this<NvidiaLldpTlvs>
{
  public:
    NvidiaLldpTlvs(sdbusplus::asio::object_server& objectServer,
                   mctp::MctpRequester& mctpRequester,
                   const std::string& deviceName, const std::string& portName,
                   const sdbusplus::object_path& portPath, uint8_t eid,
                   uint16_t portNumber, gpu::LldpPacketType direction);

    ~NvidiaLldpTlvs();

    void update();

  private:
    void processResponse(const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    // Replaces everything the object reports, signalling only what changed so
    // that a client watching one field is not woken by every poll.
    void publish(const lldp::Frame& frame);

    std::shared_ptr<sdbusplus::asio::dbus_interface> tlvsInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;

    sdbusplus::object_path objectPath;
    uint8_t eid{};

    lldp::Frame reported;

    std::array<uint8_t, gpu::getLldpPacketRequestSize> request{};
    bool requestEncoded{false};
};
