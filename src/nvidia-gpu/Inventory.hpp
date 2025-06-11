#pragma once

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

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
              const std::string& inventoryName, DeviceType deviceType);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string name;
};
