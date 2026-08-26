/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>

// The LLDP agent of a network device, as a control surface.
//
// The device holds one agent for the whole of itself rather than one per port,
// so this object belongs to the device. It is not an inventory item, so it
// lives outside the inventory tree and points at the adapter it configures
// through an association.
//
// The object appears only once the device has answered that it holds an LLDP
// mode at all. A device that does not is not a device whose LLDP can be
// configured, and saying otherwise would offer a client a control that goes
// nowhere.
class NvidiaLldpConfiguration :
    public std::enable_shared_from_this<NvidiaLldpConfiguration>
{
  public:
    NvidiaLldpConfiguration(sdbusplus::asio::object_server& objectServer,
                            mctp::MctpRequester& mctpRequester,
                            const std::string& deviceName,
                            const sdbusplus::object_path& adapterPath,
                            uint8_t eid, boost::asio::io_context& io);

    ~NvidiaLldpConfiguration();

    void update();

  private:
    void sendGetModeRequest();
    void handleGetModeResponse(const std::error_code& ec,
                               std::span<const uint8_t> buffer);

    // Publishing waits for the first answer so that the values a client first
    // reads are the device's own, not a default that was never true.
    void publish(uint8_t modeData);

    int handleTransmitModeSet(const std::string& requested,
                              std::string& current);
    int handleReceiveModeSet(const std::string& requested,
                             std::string& current);

    // One write from a client can be two writes on the bus, and both
    // directions live in one byte on the device. Each setter records what it
    // was asked for and arms a short timer; the timer sends the one command
    // that carries the settled pair.
    void armSetModeTimer();
    void applyPendingMode();
    void sendSetModeRequest(uint8_t modeData);
    void handleSetModeResponse(const std::error_code& ec,
                               std::span<const uint8_t> buffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> configurationInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    sdbusplus::asio::object_server& objectServer;
    mctp::MctpRequester& mctpRequester;

    std::string deviceName;
    sdbusplus::object_path adapterPath;
    sdbusplus::object_path objectPath;
    uint8_t eid{};

    // The whole byte as the device last reported it. A write changes only the
    // bits it owns, so whatever else the byte carries survives the write.
    uint8_t deviceMode{};
    bool published{false};
    bool reportedUnsupported{false};

    gpu::LldpMode transmitMode{gpu::LldpMode::Off};
    gpu::LldpMode receiveMode{gpu::LldpMode::Off};

    std::optional<gpu::LldpMode> pendingTransmitMode;
    std::optional<gpu::LldpMode> pendingReceiveMode;

    boost::asio::steady_timer setModeTimer;
    bool setInFlight{false};

    std::array<uint8_t, gpu::getDeviceModeSettingsV2RequestSize> getRequest{};
    std::array<uint8_t, gpu::setDeviceModeSettingsV2RequestSize> setRequest{};
};
