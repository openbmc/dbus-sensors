#pragma once

#include "MctpRequester.hpp"
#include "PropertyRetryHandler.hpp"
#include "Utils.hpp"

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
#include <vector>

class Memory : public std::enable_shared_from_this<Memory>
{
  public:
    Memory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
           sdbusplus::asio::object_server& objectServer,
           const std::string& name, mctp::MctpRequester& mctpRequester,
           uint8_t eid, boost::asio::io_context& io);

    void setProcessorAssociation(const std::string& processorPath);

  private:
    void requestMemorySpeed();
    void handleMemorySpeedResponse(int sendRecvMsgResult);

    std::shared_ptr<sdbusplus::asio::dbus_interface> dimmInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;
    std::string path;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    std::shared_ptr<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyRequest)>>
        requestBuffer;
    std::shared_ptr<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyResponse)>>
        responseBuffer;
    std::vector<Association> associations;
    PropertyRetryHandler retryHandler;
};
