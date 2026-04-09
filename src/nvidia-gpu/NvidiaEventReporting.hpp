#pragma once
#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <boost/asio.hpp>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
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
    NvidiaEventReportingConfig(mctp::Endpoint endpoint,
                               mctp::MctpRequester& req,
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
    mctp::Endpoint endpoint;
    mctp::MctpRequester& requester;
    std::array<uint64_t, messageTypeCount> eventMasks{};
    size_t currentMessageTypeIdx{0};
    std::array<uint8_t, gpu::setEventSourcesRequestSize> sourcesReq{};
    std::array<uint8_t, gpu::setEventSubscriptionRequestSize> subscriptionReq{};
};

struct EventKey
{
    mctp::Endpoint endpoint;
    gpu::MessageType type;
    uint8_t eventCode;
    EventKey(mctp::Endpoint endpoint, gpu::MessageType type,
             uint8_t eventCode) :
        endpoint{endpoint}, type{type}, eventCode{eventCode}
    {}
    bool operator==(const EventKey& other) const = default;
};

namespace std
{
template <>
struct hash<EventKey>
{
    size_t operator()(const EventKey& key) const
    {
        static_assert(sizeof(key) == sizeof(uint32_t),
                      "You have broken hashing for this type");
        uint32_t tmp = (key.endpoint.eid << 24) | (key.endpoint.network << 16) |
                       (static_cast<uint8_t>(key.type) << 8) | key.eventCode;
        return hash<uint32_t>{}(tmp);
    }
};
} // namespace std

class NvidiaEventHandler
{
  public:
    static void handleEvent(mctp::Endpoint endpoint,
                            std::span<const uint8_t> buffer);

    static void registerEventHandler(
        mctp::Endpoint endpoint, gpu::MessageType messageType,
        uint8_t eventCode, const EventHandler& handler)
    {
        eventHandlers[EventKey{endpoint, messageType, eventCode}] = handler;
    }

  private:
    static std::unordered_map<EventKey, EventHandler> eventHandlers;
};
