#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuMctpVdm.hpp"

#include <NvidiaGpuMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

using InventoryRequestBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationRequest)>;
using InventoryResponseBuffer =
    std::array<uint8_t, sizeof(gpu::GetInventoryInformationResponse)>;

class Inventory : public std::enable_shared_from_this<Inventory>
{
  public:
    Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& inventoryName,
              mctp::MctpRequester& mctpRequester,
              gpu::DeviceIdentification deviceType, uint8_t eid,
              boost::asio::io_context& io);

  private:
    struct PropertyInfo
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string propertyName;
        int retryCount{0};
        bool isPending{false};
    };
    void sendInventoryPropertyRequest(gpu::InventoryPropertyId propertyId);
    void handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId,
                                         int sendRecvMsgResult);
    void processNextProperty();
    void ProcessInventoryProperty(gpu::InventoryPropertyId propertyId);
    void registerProperty(
        gpu::InventoryPropertyId propertyId,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        const std::string& propertyName);
    std::optional<gpu::InventoryPropertyId> getNextPendingProperty() const;
    void markPropertyPending(gpu::InventoryPropertyId propertyId);
    void markPropertyPending(std::unordered_map<gpu::InventoryPropertyId,
                                                PropertyInfo>::iterator it);
    void markPropertyProcessed(gpu::InventoryPropertyId propertyId);
    void markPropertyProcessed(std::unordered_map<gpu::InventoryPropertyId,
                                                  PropertyInfo>::iterator it);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string name;
    mctp::MctpRequester& mctpRequester;
    gpu::DeviceIdentification deviceType;
    uint8_t eid;
    boost::asio::steady_timer retryTimer;
    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo> properties;
    std::shared_ptr<InventoryRequestBuffer> requestBuffer;
    std::shared_ptr<InventoryResponseBuffer> responseBuffer;
    static constexpr std::chrono::seconds retryDelay{5};
    static constexpr int maxRetryAttempts = 3;
};
