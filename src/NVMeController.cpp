#include "NVMeController.hpp"

#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Common/File/error.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

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

    auto& commitStatus = this->commitStatus;
    commitStatus = FwCommitStatus::Ready;
    adminIntf->register_property_r("FirmwareCommitStatus", commitStatus,
                                   sdbusplus::vtable::property_::emits_change,
                                   [&commitStatus](const auto&) {
        if (commitStatus != FwCommitStatus::Ready &&
            commitStatus != FwCommitStatus::InProgress)
        {
            auto tmp = commitStatus;
            commitStatus = FwCommitStatus::Ready;
            return tmp;
        }
        return commitStatus;
    });

    adminIntf->register_method(
        "FirmwareCommitAsync",
        [nvmeIntf, nvmeCtrl{ctrl},
         &commitStatus](uint8_t commitAction, uint8_t firmwareSlot, bool bpid) {
        if (commitStatus == FwCommitStatus::InProgress)
        {
            throw sdbusplus::xyz::openbmc_project::Common::Error::NotAllowed();
        }
        commitStatus = FwCommitStatus::InProgress;
        nvmeIntf->adminFwCommit(
            nvmeCtrl, static_cast<nvme_fw_commit_ca>(commitAction & 0b111),
            firmwareSlot, bpid,
            [&commitStatus](const std::error_code& ec,
                            nvme_status_field status) {
            if (ec)
            {
                commitStatus = FwCommitStatus::Failed;
                return;
            }
            if (status != NVME_SC_SUCCESS)
            {
                commitStatus = FwCommitStatus::RequireReset;
                return;
            }

            commitStatus = FwCommitStatus::Success;
            });
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

namespace sdbusplus::message::details
{
const static std::unordered_map<NVMeController::FwCommitStatus, std::string>
    mapCommitStatusString{
        {NVMeController::FwCommitStatus::Ready, "Ready"},
        {NVMeController::FwCommitStatus::InProgress, "In Progress"},
        {NVMeController::FwCommitStatus::Failed, "Failed"},
        {NVMeController::FwCommitStatus::RequireReset, "Request Reset"},
        {NVMeController::FwCommitStatus::Success, "Success"}};
const static std::unordered_map<std::string, NVMeController::FwCommitStatus>
    mapStringCommitStatus{
        {"Ready", NVMeController::FwCommitStatus::Ready},
        {"In Progress", NVMeController::FwCommitStatus::InProgress},
        {"Failed", NVMeController::FwCommitStatus::Failed},
        {"Request Reset", NVMeController::FwCommitStatus::RequireReset},
        {"Success", NVMeController::FwCommitStatus::Success}};

template <>
struct convert_from_string<NVMeController::FwCommitStatus>
{
    static std::optional<NVMeController::FwCommitStatus>
        op(const std::string& string) noexcept
    {
        auto find = mapStringCommitStatus.find(string);
        if (find == mapStringCommitStatus.end())
        {
            return {};
        }
        return {find->second};
    }
};

template <>
struct convert_to_string<NVMeController::FwCommitStatus>
{
    static std::string op(NVMeController::FwCommitStatus status)
    {
        auto find = mapCommitStatusString.find(status);
        if (find == mapCommitStatusString.end())
        {
            return {};
        }
        return find->second;
    }
};

} // namespace sdbusplus::message::details
