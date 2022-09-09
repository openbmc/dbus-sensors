
#pragma once

#include "NVMeIntf.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Inventory/Item/StorageController/server.hpp>

using ControllerBase =
    sdbusplus::xyz::openbmc_project::Inventory::Item::server::StorageController;
class NVMeController :
    public ControllerBase,
    public std::enable_shared_from_this<NVMeController>

{
  public:
    NVMeController(boost::asio::io_context& io,
                   sdbusplus::asio::object_server& objServer,
                   std::shared_ptr<sdbusplus::asio::connection> conn,
                   std::string path, std::shared_ptr<NVMeMiIntf> nvmeIntf,
                   nvme_mi_ctrl_t ctrl) :
        ControllerBase(dynamic_cast<sdbusplus::bus_t&>(*conn), path.c_str()),
        io(io), objServer(objServer), conn(conn), nvmeIntf(nvmeIntf),
        nvmeCtrl(ctrl)
    {}
    void start(){};

  private:
    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;

    std::shared_ptr<NVMeMiIntf> nvmeIntf;
    nvme_mi_ctrl_t nvmeCtrl;
};