#pragma once

#include "NvidiaGpuMctpVdm.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <string>

class Inventory
{
  public:
    Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& inventoryName,
              gpu::DeviceIdentification deviceType);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string name;
};
