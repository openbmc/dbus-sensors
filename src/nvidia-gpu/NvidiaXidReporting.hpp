#pragma once
#include "NvidiaGpuMctpVdm.hpp"

#include <MctpRequester.hpp>
#include <boost/asio.hpp>

#include <bitset>
#include <cstdint>
#include <memory>
#include <span>

class NvidiaEventReportingConfig :
    public std::enable_shared_from_this<NvidiaEventReportingConfig>
{
  public:
    NvidiaEventReportingConfig(uint8_t eid, mctp::MctpRequester& req);
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

    uint8_t bmc_eid{8};
    uint8_t eid;
    mctp::MctpRequester& requester;
    // Today we only support events for platform environmental. if more is
    // needed, we will need to make this into an array
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
