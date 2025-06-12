#pragma once

#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <memory>
#include <string>
#include <array>
#include "MctpRequester.hpp"
#include <NvidiaGpuMctpVdm.hpp>

class Inventory
{
  public:
    Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& inventoryName,
              mctp::MctpRequester& mctpRequester,
              uint8_t eid);

    void requestPartNumber();
    void requestSerialNumber();
    void update();

  private:
    void handlePartNumberResponse(int sendRecvMsgResult);
    void handleSerialNumberResponse(int sendRecvMsgResult);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::string path;
    mctp::MctpRequester& mctpRequester;
    uint8_t eid;
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)> requestBuffer{};
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationResponse)> responseBuffer{};
}; 