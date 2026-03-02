#pragma once
#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <boost/asio.hpp>
#include <boost/container_hash/hash.hpp>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <tuple>
#include <unordered_map>

struct EventInfo
{
    bool ackRequired;
    uint8_t version;
    uint8_t eventClass;
    uint16_t eventState;
    std::vector<uint8_t> eventData;
};

using EventHandler = std::function<void(
    const EventInfo&, std::span<const uint8_t> /*eventData*/)>;

struct EventDescriptor
{
    gpu::MessageType messageType;
    uint8_t eventCode;
    EventHandler eventHandler;
};

// Maximum value of MessageType
static constexpr uint8_t messageTypeCount = 6;

class NvidiaEventReportingConfig :
    public std::enable_shared_from_this<NvidiaEventReportingConfig>
{
  public:
    NvidiaEventReportingConfig(uint8_t eid, mctp::MctpRequester& req,
                               std::initializer_list<EventDescriptor> events);
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

    static constexpr uint8_t generationSettingEnablePush = 2;
    uint8_t bmc_eid{8};
    uint8_t eid;
    mctp::MctpRequester& requester;
    std::array<uint64_t, messageTypeCount> eventMasks{};
    size_t currentMessageTypeIdx{0};
    std::array<uint8_t, gpu::setEventSourcesRequestSize> sourcesReq{};
    std::array<uint8_t, gpu::setEventSubscriptionRequestSize> subscriptionReq{};
};

class NvidiaEventHandler
{
  public:
    static void handleEvent(uint8_t eid, std::span<const uint8_t> buffer);

    static void registerEventHandler(uint8_t eid, gpu::MessageType messageType,
                                     uint8_t eventCode,
                                     const EventHandler& handler)
    {
        eventHandlers[EventKey{eid, messageType, eventCode}] = handler;
    }

  private:
    // Key is a tuple of (eid, messageType, eventCode)
    using EventKey = std::tuple<uint8_t, gpu::MessageType, uint8_t>;

    static std::unordered_map<EventKey, EventHandler, boost::hash<EventKey>>
        eventHandlers;
};
