#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <memory>
#include <string>

class Inventory
{
  public:
    enum class DeviceType
    {
        Unknown,
        GPU,
    };

    Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& inventoryName,
              mctp::MctpRequester& mctpRequester, DeviceType deviceType,
              uint8_t eid);

    void requestPartNumber();
    void requestSerialNumber();
    void update();

  private:
    void requestInventoryProperty(gpu::InventoryPropertyId propertyId,
                                  const std::string& dbusProperty,
                                  int& attempts);
    void handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId,
                                         const std::string& dbusProperty,
                                         int& attempts, int sendRecvMsgResult);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string path;
    mctp::MctpRequester& mctpRequester;
    DeviceType deviceType;
    uint8_t eid;
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>
        requestBuffer{};
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationResponse)>
        responseBuffer{};
};
