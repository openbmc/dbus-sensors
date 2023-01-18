#include "NVMeController.hpp"

#include "NVMePlugin.hpp"

#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/File/error.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

using sdbusplus::xyz::openbmc_project::Inventory::Item::server::
    StorageController;
using sdbusplus::xyz::openbmc_project::NVMe::server::NVMeAdmin;

std::shared_ptr<NVMeController> NVMeController::create(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    std::shared_ptr<sdbusplus::asio::connection> conn, std::string path,
    std::shared_ptr<NVMeMiIntf> nvmeIntf, nvme_mi_ctrl_t ctrl)
{

    auto self = std::shared_ptr<NVMeController>(
        new NVMeController(io, objServer, conn, path, nvmeIntf, ctrl));
    self->init();
    return self;
}

NVMeController::NVMeController(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objServer,
    std::shared_ptr<sdbusplus::asio::connection> conn, std::string path,
    std::shared_ptr<NVMeMiIntf> nvmeIntf, nvme_mi_ctrl_t ctrl) :
    StorageController(dynamic_cast<sdbusplus::bus_t&>(*conn), path.c_str()),
    NVMeAdmin(*conn, path.c_str(),
              {{"FirmwareCommitStatus", {FwCommitStatus::Ready}}}),
    io(io), objServer(objServer), conn(conn), path(path), nvmeIntf(nvmeIntf),
    nvmeCtrl(ctrl)
{}

// Performs initialisation after shared_from_this() has been set up.
void NVMeController::init()
{
    StorageController::emit_added();
    NVMeAdmin::emit_added();
}

void NVMeController::start(std::shared_ptr<NVMeControllerPlugin> nvmePlugin)
{
    plugin = nvmePlugin;
}

sdbusplus::message::unix_fd NVMeController::getLogPage(uint8_t lid,
                                                       uint32_t nsid,
                                                       uint8_t lsp,
                                                       uint16_t lsi)
{
    std::array<int, 2> pipe;
    if (::pipe(pipe.data()) < 0)
    {
        std::cerr << "GetLogPage fails to open pipe: " << std::strerror(errno)
                  << std::endl;
        throw sdbusplus::xyz::openbmc_project::Common::File::Error::Open();
    }

    // standard NVMe Log IDs
    if (lid < uint8_t{0xC0})
    {
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

            // TODO: evaluate the impact of client not reading fast enough
            // on large trunk of data
            if (::write(fd, data.data(), data.size()) < 0)
            {
                std::cerr << "GetLogPage fails to write fd: "
                          << std::strerror(errno) << std::endl;
            };
            close(fd);
            });
    }
    // vendor Log IDs
    else if (!plugin.expired())
    {
        auto nvmePlugin = plugin.lock();
        auto handler = nvmePlugin->getGetLogPageHandler();
        if (handler)
        {
            std::function<void(const std::error_code&, std::span<uint8_t>)> cb =
                [pipe](std::error_code ec, std::span<uint8_t> data) {
                ::close(pipe[0]);
                int fd = pipe[1];
                if (ec)
                {
                    std::cerr << "fail to GetLogPage: " << ec.message()
                              << std::endl;
                    close(fd);
                    return;
                }

                // TODO: evaluate the impact of client not reading fast enough
                // on large trunk of data
                if (::write(fd, data.data(), data.size()) < 0)
                {
                    std::cerr << "GetLogPage fails to write fd: "
                              << std::strerror(errno) << std::endl;
                };
                close(fd);
            };
            handler(lid, nsid, lsp, lsi, std::move(cb));
        }
        else // No VU LogPage handler
        {
            throw sdbusplus::xyz::openbmc_project::Common::Error::
                InvalidArgument();
        }
    }
    else // No VU plugin
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }
    return sdbusplus::message::unix_fd{pipe[0]};
}

sdbusplus::message::unix_fd NVMeController::identify(uint8_t cns, uint32_t nsid,
                                                     uint16_t cntid)
{
    std::array<int, 2> pipe;
    if (::pipe(pipe.data()) < 0)
    {
        std::cerr << "Identify fails to open pipe: " << std::strerror(errno)
                  << std::endl;
        throw sdbusplus::xyz::openbmc_project::Common::File::Error::Open();
    }

    nvmeIntf->adminIdentify(
        nvmeCtrl, static_cast<nvme_identify_cns>(cns), nsid, cntid,
        [self{shared_from_this()}, pipe](const std::error_code& ec,
                                         std::span<uint8_t> data) {
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
            std::cerr << "Identify fails to write fd: " << std::strerror(errno)
                      << std::endl;
        };
        close(fd);
        });
    return sdbusplus::message::unix_fd{pipe[0]};
}

NVMeAdmin::FwCommitStatus
    NVMeController::firmwareCommitStatus(NVMeAdmin::FwCommitStatus status)
{
    auto commitStatus = this->NVMeAdmin::firmwareCommitStatus();
    // The function is only allowed to reset the status back to ready
    if (status != FwCommitStatus::Ready ||
        commitStatus == FwCommitStatus::Ready ||
        commitStatus == FwCommitStatus::InProgress)
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::NotAllowed{};
    }
    return this->NVMeAdmin::firmwareCommitStatus(status);
}

void NVMeController::firmwareCommitAsync(uint8_t commitAction,
                                         uint8_t firmwareSlot, bool bpid)
{
    auto commitStatus = this->NVMeAdmin::firmwareCommitStatus();
    if (commitStatus != FwCommitStatus::Ready)
    {
        throw sdbusplus::xyz::openbmc_project::Common::Error::NotAllowed();
    }
    this->NVMeAdmin::firmwareCommitStatus(FwCommitStatus::InProgress);
    nvmeIntf->adminFwCommit(
        nvmeCtrl, static_cast<nvme_fw_commit_ca>(commitAction & 0b111),
        firmwareSlot, bpid,
        [self{shared_from_this()}](const std::error_code& ec,
                                   nvme_status_field status) {
        if (ec)
        {
            self->NVMeAdmin::firmwareCommitStatus(FwCommitStatus::Failed);
            return;
        }
        if (status != NVME_SC_SUCCESS)
        {
            self->NVMeAdmin::firmwareCommitStatus(FwCommitStatus::RequireReset);
            return;
        }

        self->NVMeAdmin::firmwareCommitStatus(FwCommitStatus::Success);
        });
}

NVMeController::~NVMeController()
{
    NVMeAdmin::emit_removed();
    StorageController::emit_removed();
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
