#include "NVMeController.hpp"

#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/File/error.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

NVMeController::NVMeController(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    std::shared_ptr<sdbusplus::asio::connection> conn, std::string path,
    std::shared_ptr<NVMeMiIntf> nvmeIntf, nvme_mi_ctrl_t ctrl) :
    ControllerBase(dynamic_cast<sdbusplus::bus_t&>(*conn), path.c_str()),
    io(io), objServer(objServer), conn(conn), path(path), nvmeIntf(nvmeIntf),
    nvmeCtrl(ctrl)
{
    emit_added();

    /* Admin interface */
    adminIntf = objServer.add_interface(path, adminIntfName);
    adminIntf->register_method("GetLogPage", [nvmeIntf, nvmeCtrl{ctrl}](
                                                 unsigned lid, uint32_t nsid,
                                                 uint8_t lsp, uint16_t lsi) {
        std::array<int, 2> pipe;
        if (::pipe(pipe.data()) < 0)
        {
            std::cerr << "GetLogPage fails to open pipe: "
                      << std::strerror(errno) << std::endl;
            throw sdbusplus::xyz::openbmc_project::Common::File::Error::Open();
        }

        nvmeIntf->adminGetLogPage(
            nvmeCtrl, static_cast<nvme_cmd_get_log_lid>(lid), nsid, lsp, lsi,
            [pipe](const std::error_code& ec, std::span<uint8_t> data) {
            ::close(pipe[0]);
            int fd = pipe[1];
            if (ec)
            {
                std::cerr << "fail to GetLogPage: " << ec.message()
                          << std::endl;
                close(fd);
                return;
            }

            // TODO: evaluate the impact of client not reading fast enough on
            // large trunk of data
            if (::write(fd, data.data(), data.size()) < 0)
            {
                std::cerr << "GetLogPage fails to write fd: "
                          << std::strerror(errno) << std::endl;
            };
            close(fd);
            });
        return sdbusplus::message::unix_fd{pipe[0]};
    });

    adminIntf->register_method(
        "Identify",
        [nvmeIntf, nvmeCtrl{ctrl}](uint8_t cns, uint16_t cntid, uint32_t nsid) {
        std::array<int, 2> pipe;
        if (::pipe(pipe.data()) < 0)
        {
            std::cerr << "Identify fails to open pipe: " << std::strerror(errno)
                      << std::endl;
            throw sdbusplus::xyz::openbmc_project::Common::File::Error::Open();
        }

        nvmeIntf->adminIdentify(
            nvmeCtrl, static_cast<nvme_identify_cns>(cns), nsid, cntid,
            [pipe](const std::error_code& ec, std::span<uint8_t> data) {
            ::close(pipe[0]);
            int fd = pipe[1];
            if (ec)
            {
                std::cerr << "fail to Identify: " << ec.message() << std::endl;
                close(fd);
                return;
            }
            if (write(fd, data.data(), data.size()) < 0)
            {
                std::cerr << "Identify fails to write fd: "
                          << std::strerror(errno) << std::endl;
            };
            close(fd);
            });
        return sdbusplus::message::unix_fd{pipe[0]};
        });

    adminIntf->initialize();
}

NVMeController::~NVMeController()
{
    objServer.remove_interface(adminIntf);
    emit_removed();
}

void NVMeController::setSecAssoc(
    const std::vector<std::shared_ptr<NVMeController>> secCntrls)
{

    if (secAssoc)
    {
        objServer.remove_interface(secAssoc);
        secAssoc.reset();
    }

    if (secCntrls.empty())
    {
        return;
    }

    using Association = std::tuple<std::string, std::string, std::string>;
    secAssoc = objServer.add_interface(
        path, "xyz.openbmc_project.Association.Definitions");
    std::vector<Association> associations;

    for (auto& cntrl : secCntrls)
    {
        associations.emplace_back("secondary", "primary", cntrl->path);
    }
    secAssoc->register_property("Associations", associations);
    secAssoc->initialize();
}
