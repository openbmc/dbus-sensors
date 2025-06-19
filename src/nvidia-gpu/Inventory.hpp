#pragma once

#include "MctpRequester.hpp"
#include "Memory.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "PropertyRetryHandler.hpp"
#include "Utils.hpp"

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
#include <vector>

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

    void setLocationCode(const std::string& locationCode);
    void setAssociation(const std::string& chassisPath);

  private:
    struct PropertyInfo
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string propertyName;
        bool isPending{false};
    };
    void sendInventoryPropertyRequest(gpu::InventoryPropertyId propertyId);
    void handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId,
                                         int sendRecvMsgResult);
    void processNextProperty();
    void processInventoryProperty(gpu::InventoryPropertyId propertyId);
    void registerProperty(
        gpu::InventoryPropertyId propertyId,
        const std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
        const std::string& propertyName);
    std::optional<gpu::InventoryPropertyId> getNextPendingProperty() const;
    static void markPropertyPending(
        std::unordered_map<gpu::InventoryPropertyId, PropertyInfo>::iterator
            it);
    static void markPropertyProcessed(
        std::unordered_map<gpu::InventoryPropertyId, PropertyInfo>::iterator
            it);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> uuidInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> revisionIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> locationCodeIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::string name;
    mctp::MctpRequester& mctpRequester;
    gpu::DeviceIdentification deviceType;
    uint8_t eid;
    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo> properties;
    std::shared_ptr<InventoryRequestBuffer> requestBuffer;
    std::shared_ptr<InventoryResponseBuffer> responseBuffer;
    PropertyRetryHandler retryHandler;
    std::shared_ptr<Memory> memoryModule;
    sdbusplus::asio::object_server& objectServer;
    std::string inventoryPath;
    std::vector<Association> associations;
};
