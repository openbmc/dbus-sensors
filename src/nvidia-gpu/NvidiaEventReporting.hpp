#pragma once
#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <boost/asio.hpp>

#include <array>
#include <bitset>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>

// Type-safe event subscription types
enum class EventType : uint8_t
{
    Xid, // MessageType::PLATFORM_ENVIRONMENTAL, bit 1
};

// MessageType values are small (0, 2, 3), so use array indexing
static constexpr size_t messageTypeCount = 4;

class NvidiaEventReportingConfig :
    public std::enable_shared_from_this<NvidiaEventReportingConfig>
{
  public:
    NvidiaEventReportingConfig(uint8_t eid, mctp::MctpRequester& req,
                               std::initializer_list<EventType> events);
    NvidiaEventReportingConfig(NvidiaEventReportingConfig&&) noexcept = delete;
    NvidiaEventReportingConfig& operator=(
        NvidiaEventReportingConfig&&) noexcept = delete;
    NvidiaEventReportingConfig(const NvidiaEventReportingConfig&) = delete;
    NvidiaEventReportingConfig& operator=(const NvidiaEventReportingConfig&) =
        delete;
    ~NvidiaEventReportingConfig() = default;

    void init();

  private:
    void handleSetupSubscription(const std::error_code& ec,
                                 std::span<const uint8_t> buffer);
    void handleSetupEvents(const std::error_code& ec,
                           std::span<const uint8_t> buffer);
    void sendNextEventSource();

    uint8_t bmc_eid{8};
    uint8_t eid;
    mctp::MctpRequester& requester;
    std::array<uint64_t, messageTypeCount> eventMasks{};
    size_t currentMessageTypeIdx{0};
    std::array<uint8_t, sizeof(gpu::SetEventSourcesRequest)> sourcesReq{};
    std::array<uint8_t, sizeof(gpu::SetEventSubscriptionRequest)>
        subscriptionReq{};
};

class NvidiaEventHandler
{
  public:
    static void handleEvent(uint8_t eid, std::span<const uint8_t> buffer);

  private:
    static void handlePlatformEvent(
        uint8_t eid, std::span<const uint8_t> buffer, uint8_t messageType);
};
