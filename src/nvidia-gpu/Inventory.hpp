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
    enum class DeviceType
    {
        Unknown,
        GPU,
    };
    
    Inventory(const std::shared_ptr<sdbusplus::asio::connection>& conn,
              sdbusplus::asio::object_server& objectServer,
              const std::string& inventoryName,
              mctp::MctpRequester& mctpRequester,
              DeviceType deviceType,
              uint8_t eid);

  private:
    std::shared_ptr<sdbusplus::asio::dbus_interface> acceleratorInterface;

    std::string path;
    mctp::MctpRequester& mctpRequester;
    DeviceType deviceType;
    uint8_t eid;
}; 