
#pragma once

#include "NVMeIntf.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Inventory/Item/StorageController/server.hpp>

#include <utility>

using ControllerBase =
    sdbusplus::xyz::openbmc_project::Inventory::Item::server::StorageController;
class NVMeController :
    private ControllerBase,
    public std::enable_shared_from_this<NVMeController>

{
  public:
    NVMeController(boost::asio::io_context& io,
                   sdbusplus::asio::object_server& objServer,
                   std::shared_ptr<sdbusplus::asio::connection> conn,
                   std::string path, std::shared_ptr<NVMeMiIntf> nvmeIntf,
                   nvme_mi_ctrl_t ctrl) :
        ControllerBase(dynamic_cast<sdbusplus::bus_t&>(*conn), path.c_str()),
        io(io), objServer(objServer), conn(conn), path(path),
        nvmeIntf(nvmeIntf), nvmeCtrl(ctrl)
    {
        emit_added();
    }
    void start(){};

    // setup association to the secondary controllers. Clear the Association if
    // empty.
    void setSecAssoc(
        const std::vector<std::shared_ptr<NVMeController>> secCntrls);

    inline void setSecAssoc()
    {
        setSecAssoc({});
    }

    ~NVMeController() override
    {
        emit_removed();
    }

  private:
    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string path;

    std::shared_ptr<NVMeMiIntf> nvmeIntf;
    nvme_mi_ctrl_t nvmeCtrl;

    // The Association interface to secondary controllers from a primary
    // controller
    std::shared_ptr<sdbusplus::asio::dbus_interface> secAssoc;
};
