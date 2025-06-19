#pragma once

#include "MctpRequester.hpp"
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

    void fetchBoardPartNumber();
    void fetchSerialNumber();
    void fetchUUID();
    void fetchMarketingName();
    void fetchDevicePartNumber();
    void update();
    void setLocationCode(const std::string& locationCode);
    std::string getInventoryPath() const;
    void setAssociation(const std::string& chassisPath);

  private:
    struct PropertyInfo
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface;
        std::string propertyName;
        int retryCount{0};
        bool isPending{false};
    };

    void requestInventoryProperty(gpu::InventoryPropertyId propertyId);
    void handleInventoryPropertyResponse(gpu::InventoryPropertyId propertyId,
                                         int sendRecvMsgResult);
    void processNextProperty();
    void fetchInventoryProperty(gpu::InventoryPropertyId propertyId);
    void registerProperty(
        gpu::InventoryPropertyId propertyId,
        std::shared_ptr<sdbusplus::asio::dbus_interface> interface,
        const std::string& propertyName);
    std::optional<gpu::InventoryPropertyId> getNextPendingProperty() const;
    void markPropertyPending(gpu::InventoryPropertyId propertyId);
    void markPropertyProcessed(gpu::InventoryPropertyId propertyId);

    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> uuidInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> revisionIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> locationCodeIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associationInterface;

    std::string name;
    mctp::MctpRequester& mctpRequester;
    DeviceType deviceType;
    uint8_t eid;
    boost::asio::steady_timer retryTimer;
    sdbusplus::asio::object_server& objectServer;
    std::string inventoryPath;
    std::unordered_map<gpu::InventoryPropertyId, PropertyInfo> properties;
    std::shared_ptr<InventoryRequestBuffer> requestBuffer;
    std::shared_ptr<InventoryResponseBuffer> responseBuffer;
    std::vector<Association> associations;
    static constexpr std::chrono::seconds retryDelay{5};
    static constexpr int maxRetryAttempts = 3;
};
