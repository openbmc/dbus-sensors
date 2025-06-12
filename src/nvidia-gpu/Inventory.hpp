#pragma once

#include "MctpRequester.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

using InventoryRequestBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>;
using InventoryResponseBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationResponse)>;

class Inventory : public std::enable_shared_from_this<Inventory>
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
              uint8_t eid, boost::asio::io_context& io);

    void fetchPartNumber();
    void fetchSerialNumber();
    void update();

    static std::optional<std::string_view> dbusPropertyNameForId(
        gpu::InventoryPropertyId propertyId);

  private:
    void requestInventoryProperty(
        gpu::InventoryPropertyId propertyId,
        std::shared_ptr<InventoryRequestBuffer> requestBuffer,
        std::shared_ptr<InventoryResponseBuffer> responseBuffer);

    void handleInventoryPropertyResponse(
        gpu::InventoryPropertyId propertyId, int sendRecvMsgResult,
        std::shared_ptr<InventoryRequestBuffer> requestBuffer,
        std::shared_ptr<InventoryResponseBuffer> responseBuffer);

    void retryRequest(gpu::InventoryPropertyId propertyId,
                      std::shared_ptr<InventoryRequestBuffer> requestBuffer,
                      std::shared_ptr<InventoryResponseBuffer> responseBuffer);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string path;
    mctp::MctpRequester& mctpRequester;
    DeviceType deviceType;
    uint8_t eid;
    boost::asio::steady_timer retryTimer;
    static constexpr std::chrono::seconds retryDelay{5};
};
