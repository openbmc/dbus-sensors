#include "NVMeController.hpp"

#include "AsioHelper.hpp"
#include "NVMePlugin.hpp"

#include <sdbusplus/exception.hpp>
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

    securityInterface = objServer.add_interface(
        path, "xyz.openbmc_project.Inventory.Item.StorageControllerSecurity");
    securityInterface->register_method(
        "SecuritySend",
        [self{shared_from_this()}](boost::asio::yield_context yield,
                                   uint8_t proto, uint16_t proto_specific,
                                   std::vector<uint8_t> data) {
        return self->securitySendMethod(yield, proto, proto_specific, data);
        });
    securityInterface->register_method(
        "SecurityReceive",
        [self{shared_from_this()}](boost::asio::yield_context yield,
                                   uint8_t proto, uint16_t proto_specific,
                                   uint32_t transfer_length) {
        return self->securityReceiveMethod(yield, proto, proto_specific,
                                           transfer_length);
        });

    securityInterface->initialize();

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
    objServer.remove_interface(securityInterface);
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

void NVMeController::securitySendMethod(boost::asio::yield_context yield,
                                        uint8_t proto, uint16_t proto_specific,
                                        std::span<uint8_t> data)
{
    using callback_t = void(std::tuple<std::error_code, int>);
    auto [err, nvme_status] =
        boost::asio::async_initiate<boost::asio::yield_context, callback_t>(
            [intf{nvmeIntf}, ctrl{nvmeCtrl}, proto, proto_specific,
             &data](auto&& handler) {
        auto h = asio_helper::CopyableCallback(std::move(handler));

        intf->adminSecuritySend(
            ctrl, proto, proto_specific, data,
            [h](const std::error_code& err, int nvme_status) mutable {
            h(std::make_tuple(err, nvme_status));
            });
            },
            yield);

    // exception must be thrown outside of the async block
    checkLibNVMeError(err, nvme_status, "SecuritySend");
}

std::vector<uint8_t> NVMeController::securityReceiveMethod(
    boost::asio::yield_context yield, uint8_t proto, uint16_t proto_specific,
    uint32_t transfer_length)
{
    using callback_t =
        void(std::tuple<std::error_code, int, std::vector<uint8_t>>);
    auto [err, nvme_status, data] =
        boost::asio::async_initiate<boost::asio::yield_context, callback_t>(
            [intf{nvmeIntf}, ctrl{nvmeCtrl}, proto, proto_specific,
             transfer_length](auto&& handler) {
        auto h = asio_helper::CopyableCallback(std::move(handler));

        intf->adminSecurityReceive(ctrl, proto, proto_specific, transfer_length,
                                   [h](const std::error_code& err,
                                       int nvme_status,
                                       std::span<uint8_t> data) mutable {
            std::vector<uint8_t> d(data.begin(), data.end());
            h(std::make_tuple(err, nvme_status, d));
        });
            },
            yield);

    // exception must be thrown outside of the async block
    checkLibNVMeError(err, nvme_status, "SecurityReceive");
    return data;
}

class NVMeSdBusPlusError : public sdbusplus::exception::exception
{

  public:
    NVMeSdBusPlusError(const std::string_view& desc) : desc(desc)
    {}

    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.NVMe.NVMeError";
    }
    const char* description() const noexcept override
    {
        return desc.c_str();
    }
    int get_errno() const noexcept override
    {
        // arbitrary, sdbusplus method return ignores this errno
        return EIO;
    }

  private:
    const std::string desc;
};

/* Throws an appropriate error type for the given status from libnvme,
 * or returns normally if nvme_status == 0 */
void NVMeController::checkLibNVMeError(const std::error_code& err,
                                       int nvme_status, const char* method_name)
{
    if (nvme_status < 0)
    {
        throw sdbusplus::exception::SdBusError(err.value(), method_name);
    }
    else if (nvme_status > 0)
    {
        int val = nvme_status_get_value(nvme_status);
        int ty = nvme_status_get_type(nvme_status);
        std::string desc;

        switch (ty)
        {
            case NVME_STATUS_TYPE_NVME:
                desc =
                    std::string("NVMe: ") + nvme_status_to_string(val, false);
                break;
            case NVME_STATUS_TYPE_MI:
                desc = std::string("NVMe MI: ") + nvme_mi_status_to_string(val);
                break;
            default:
                std::cerr << "Unknown libnvme error status " << nvme_status
                          << std::endl;
                desc = "Unknown libnvme error";
        }
        throw NVMeSdBusPlusError(desc);
    }
}
