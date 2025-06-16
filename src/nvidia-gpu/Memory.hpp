#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

class Memory : public std::enable_shared_from_this<Memory>
{
  public:
    Memory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
           sdbusplus::asio::object_server& objectServer,
           const std::string& name, mctp::MctpRequester& mctpRequester,
           uint8_t eid, boost::asio::io_context& io);

    void setMemoryType(const std::string& type);
    void update();

  private:
    void requestMemorySpeed();
    void handleMemorySpeedResponse(int sendRecvMsgResult);

    std::shared_ptr<sdbusplus::asio::dbus_interface> dimmInterface;
    std::string path;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    boost::asio::steady_timer retryTimer;
    std::shared_ptr<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyRequest)>>
        requestBuffer;
    std::shared_ptr<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyResponse)>>
        responseBuffer;
    static constexpr std::chrono::seconds retryDelay{5};
    static constexpr int maxRetryAttempts = 3;
    int retryCount{0};
};
