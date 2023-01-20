#include "NVMeMi.hpp"

#include <boost/endian.hpp>

#include <cerrno>
#include <iostream>

nvme_root_t NVMeMi::nvmeRoot = nvme_mi_create_root(stderr, DEFAULT_LOGLEVEL);

NVMeMi::NVMeMi(boost::asio::io_context& io, sdbusplus::bus_t& dbus, int bus,
               int addr) :
    io(io),
    dbus(dbus)
{
    if (!nvmeRoot)
    {
        throw std::runtime_error("invalid NVMe root");
    }

    // init mctp ep via mctpd
    int i = 0;
    for (;; i++)
    {
        try
        {
            auto msg = dbus.new_method_call(
                "xyz.openbmc_project.MCTP", "/xyz/openbmc_project/mctp",
                "au.com.CodeConstruct.MCTP", "SetupEndpoint");

            msg.append("mctpi2c" + std::to_string(bus));
            msg.append(std::vector<uint8_t>{static_cast<uint8_t>(addr)});
            auto reply = msg.call(); // throw SdBusError

            reply.read(eid);
            reply.read(nid);
            reply.read(mctpPath);
            break;
        }
        catch (const std::exception& e)
        {
            if (i < 5)
            {
                std::cerr << "retry to SetupEndpoint: " << e.what()
                          << std::endl;
            }
            else
            {
                throw std::runtime_error(e.what());
            }
        }
    }

    // open mctp endpoint
    nvmeEP = nvme_mi_open_mctp(nvmeRoot, nid, eid);
    if (!nvmeEP)
    {
        throw std::runtime_error("can't open MCTP endpoint " +
                                 std::to_string(nid) + ":" +
                                 std::to_string(eid));
    }

    // start worker thread
    workerStop = false;
    thread = std::thread([&io = workerIO, &stop = workerStop, &mtx = workerMtx,
                          &cv = workerCv]() {
        // With BOOST_ASIO_DISABLE_THREADS, boost::asio::executor_work_guard
        // issues null_event across the thread, which caused invalid invokation.
        // We implement a simple invoke machenism based std::condition_variable.
        while (1)
        {
            io.run();
            io.restart();
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock);
                if (stop)
                {
                    // exhaust all tasks and exit
                    io.run();
                    break;
                }
            }
        }
    });
}

NVMeMi::~NVMeMi()
{
    // close worker
    workerStop = true;
    {
        std::unique_lock<std::mutex> lock(workerMtx);
        workerCv.notify_all();
    }
    thread.join();

    // close EP
    if (nvmeEP)
    {
        nvme_mi_close(nvmeEP);
    }

    // TODO: delete mctp ep from mctpd
}

void NVMeMi::post(std::function<void(void)>&& func)
{
    if (!workerStop)
    {
        std::unique_lock<std::mutex> lock(workerMtx);
        if (!workerStop)
        {
            workerIO.post(std::move(func));
            workerCv.notify_all();
            return;
        }
    }
    throw std::runtime_error("NVMeMi has been stopped");
}

void NVMeMi::miSubsystemHealthStatusPoll(
    std::function<void(const std::error_code&, nvme_mi_nvm_ss_health_status*)>&&
        cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;

        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), nullptr);
        });
        return;
    }

    try
    {
        post([self{shared_from_this()}, cb{std::move(cb)}]() {
            nvme_mi_nvm_ss_health_status ss_health;
            auto rc = nvme_mi_mi_subsystem_health_status_poll(self->nvmeEP,
                                                              true, &ss_health);
            if (rc < 0)
            {

                std::cerr << "fail to subsystem_health_status_poll: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)),
                       nullptr);
                });
                return;
            }
            else if (rc > 0)
            {
                std::string_view errMsg =
                    statusToString(static_cast<nvme_mi_resp_status>(rc));
                std::cerr << "fail to subsystem_health_status_poll: " << errMsg
                          << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(std::errc::bad_message), nullptr);
                });
                return;
            }

            self->io.post(
                [cb{std::move(cb)}, ss_health{std::move(ss_health)}]() mutable {
                cb({}, &ss_health);
            });
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
}

void NVMeMi::miScanCtrl(std::function<void(const std::error_code&,
                                           const std::vector<nvme_mi_ctrl_t>&)>
                            cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;

        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }

    try
    {
        post([self{shared_from_this()}, cb{std::move(cb)}]() {
            int rc = nvme_mi_scan_ep(self->nvmeEP, true);
            if (rc < 0)
            {
                std::cerr << "fail to scan controllers: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)), {});
                });
                return;
            }
            else if (rc > 0)
            {
                std::string_view errMsg =
                    statusToString(static_cast<nvme_mi_resp_status>(rc));
                std::cerr << "fail to scan controllers: " << errMsg
                          << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(std::errc::bad_message), {});
                });
                return;
            }

            std::vector<nvme_mi_ctrl_t> list;
            nvme_mi_ctrl_t c;
            nvme_mi_for_each_ctrl(self->nvmeEP, c)
            {
                list.push_back(c);
            }
            self->io.post(
                [cb{std::move(cb)}, list{std::move(list)}]() { cb({}, list); });
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
}

void NVMeMi::adminIdentify(
    nvme_mi_ctrl_t ctrl, nvme_identify_cns cns, uint32_t nsid, uint16_t cntid,
    std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
    try
    {
        post([ctrl, cns, nsid, cntid, self{shared_from_this()},
              cb{std::move(cb)}]() {
            int rc = 0;
            std::vector<uint8_t> data;
            switch (cns)
            {
                case NVME_IDENTIFY_CNS_SECONDARY_CTRL_LIST:
                {
                    data.resize(sizeof(nvme_secondary_ctrl_list));
                    nvme_identify_args args{};
                    memset(&args, 0, sizeof(args));
                    args.result = nullptr;
                    args.data = data.data();
                    args.args_size = sizeof(args);
                    args.cns = cns;
                    args.csi = NVME_CSI_NVM;
                    args.nsid = nsid;
                    args.cntid = cntid;
                    args.cns_specific_id = NVME_CNSSPECID_NONE;
                    args.uuidx = NVME_UUID_NONE,

                    rc = nvme_mi_admin_identify_partial(ctrl, &args, 0,
                                                        data.size());

                    break;
                }

                default:
                {
                    data.resize(NVME_IDENTIFY_DATA_SIZE);
                    nvme_identify_args args{};
                    memset(&args, 0, sizeof(args));
                    args.result = nullptr;
                    args.data = data.data();
                    args.args_size = sizeof(args);
                    args.cns = cns;
                    args.csi = NVME_CSI_NVM;
                    args.nsid = nsid;
                    args.cntid = cntid;
                    args.cns_specific_id = NVME_CNSSPECID_NONE;
                    args.uuidx = NVME_UUID_NONE,

                    rc = nvme_mi_admin_identify(ctrl, &args);
                }
            }
            if (rc < 0)
            {
                std::cerr << "fail to do nvme identify: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)), {});
                });
                return;
            }
            else if (rc > 0)
            {
                std::string_view errMsg =
                    statusToString(static_cast<nvme_mi_resp_status>(rc));
                std::cerr << "fail to do nvme identify: " << errMsg
                          << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(std::errc::bad_message), {});
                });
                return;
            }

            self->io.post([cb{std::move(cb)}, data{std::move(data)}]() mutable {
                std::span<uint8_t> span{data.data(), data.size()};
                cb({}, span);
            });
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
}

static int nvme_mi_admin_get_log_telemetry_host_rae(nvme_mi_ctrl_t ctrl,
                                                    bool /*rae*/, __u64 offset,
                                                    __u32 len, void* log)
{
    return nvme_mi_admin_get_log_telemetry_host(ctrl, offset, len, log);
}

// Get Temetery Log header and return the size for hdr + data area (Area 1, 2,
// 3, or maybe 4)
int getTelemetryLog(nvme_mi_ctrl_t ctrl, bool host, bool create,
                    std::vector<uint8_t>& data)
{
    int rc = 0;
    data.resize(sizeof(nvme_telemetry_log));
    nvme_telemetry_log& log =
        *reinterpret_cast<nvme_telemetry_log*>(data.data());
    auto func = host ? nvme_mi_admin_get_log_telemetry_host_rae
                     : nvme_mi_admin_get_log_telemetry_ctrl;

    // Only host telemetry log requires create.
    if (host && create)
    {
        rc = nvme_mi_admin_get_log_create_telemetry_host(ctrl, &log);
        if (rc)
        {
            std::cerr << "failed to create telemetry host log" << std::endl;
            return rc;
        }
        return 0;
    }

    rc = func(ctrl, false, 0, sizeof(log), &log);

    if (rc)
    {
        std::cerr << "failed to retain telemetry log for "
                  << (host ? "host" : "ctrl") << std::endl;
        return rc;
    }

    long size =
        static_cast<long>((boost::endian::little_to_native(log.dalb3) + 1)) *
        NVME_LOG_TELEM_BLOCK_SIZE;

    data.resize(size);
    rc = func(ctrl, false, 0, data.size(), data.data());
    if (rc)
    {
        std::cerr << "failed to get full telemetry log for "
                  << (host ? "host" : "ctrl") << std::endl;
        return rc;
    }
    return 0;
}

void NVMeMi::adminGetLogPage(
    nvme_mi_ctrl_t ctrl, nvme_cmd_get_log_lid lid, uint32_t nsid, uint8_t lsp,
    uint16_t lsi,
    std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }

    try
    {
        post([ctrl, nsid, lid, lsp, lsi, self{shared_from_this()},
              cb{std::move(cb)}]() {
            std::vector<uint8_t> data;

            int rc = 0;
            switch (lid)
            {
                case NVME_LOG_LID_ERROR:
                {
                    data.resize(nvme_mi_xfer_size);
                    // The number of entries for most recent error logs.
                    // Currently we only do one nvme mi transfer for the
                    // error log to avoid blocking other tasks
                    static constexpr int num =
                        nvme_mi_xfer_size / sizeof(nvme_error_log_page);
                    nvme_error_log_page* log =
                        reinterpret_cast<nvme_error_log_page*>(data.data());

                    rc = nvme_mi_admin_get_log_error(ctrl, num, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get error log" << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_SMART:
                {
                    data.resize(sizeof(nvme_smart_log));
                    nvme_smart_log* log =
                        reinterpret_cast<nvme_smart_log*>(data.data());
                    rc = nvme_mi_admin_get_log_smart(ctrl, nsid, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get smart log" << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_FW_SLOT:
                {
                    data.resize(sizeof(nvme_firmware_slot));
                    nvme_firmware_slot* log =
                        reinterpret_cast<nvme_firmware_slot*>(data.data());
                    rc = nvme_mi_admin_get_log_fw_slot(ctrl, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get firmware slot" << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_CMD_EFFECTS:
                {
                    data.resize(sizeof(nvme_cmd_effects_log));
                    nvme_cmd_effects_log* log =
                        reinterpret_cast<nvme_cmd_effects_log*>(data.data());

                    // nvme rev 1.3 doesn't support csi,
                    // set to default csi = NVME_CSI_NVM
                    rc = nvme_mi_admin_get_log_cmd_effects(ctrl, NVME_CSI_NVM,
                                                           log);
                    if (rc)
                    {
                        std::cerr << "fail to get cmd supported and effects log"
                                  << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_DEVICE_SELF_TEST:
                {
                    data.resize(sizeof(nvme_self_test_log));
                    nvme_self_test_log* log =
                        reinterpret_cast<nvme_self_test_log*>(data.data());
                    rc = nvme_mi_admin_get_log_device_self_test(ctrl, log);
                    if (rc)
                    {
                        std::cerr << "fail to get device self test log"
                                  << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_CHANGED_NS:
                {
                    data.resize(sizeof(nvme_ns_list));
                    nvme_ns_list* log =
                        reinterpret_cast<nvme_ns_list*>(data.data());
                    rc =
                        nvme_mi_admin_get_log_changed_ns_list(ctrl, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get changed namespace list"
                                  << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_TELEMETRY_HOST:
                // fall through to NVME_LOG_LID_TELEMETRY_CTRL
                case NVME_LOG_LID_TELEMETRY_CTRL:
                {
                    bool host = false;
                    bool create = false;
                    if (lid == NVME_LOG_LID_TELEMETRY_HOST)
                    {
                        host = true;
                        if (lsp == NVME_LOG_TELEM_HOST_LSP_CREATE)
                        {
                            create = true;
                        }
                        else if (lsp == NVME_LOG_TELEM_HOST_LSP_RETAIN)
                        {
                            create = false;
                        }
                        else
                        {
                            std::cerr << "invalid lsp for telemetry host log"
                                      << std::endl;
                            rc = -1;
                            errno = EINVAL;
                            break;
                        }
                    }
                    else
                    {
                        host = false;
                    }

                    rc = getTelemetryLog(ctrl, host, create, data);
                }
                break;
                case NVME_LOG_LID_RESERVATION:
                {
                    data.resize(sizeof(nvme_resv_notification_log));
                    nvme_resv_notification_log* log =
                        reinterpret_cast<nvme_resv_notification_log*>(
                            data.data());

                    int rc =
                        nvme_mi_admin_get_log_reservation(ctrl, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get reservation "
                                     "notification log"
                                  << std::endl;
                        break;
                    }
                }
                break;
                case NVME_LOG_LID_SANITIZE:
                {
                    data.resize(sizeof(nvme_sanitize_log_page));
                    nvme_sanitize_log_page* log =
                        reinterpret_cast<nvme_sanitize_log_page*>(data.data());

                    int rc = nvme_mi_admin_get_log_sanitize(ctrl, false, log);
                    if (rc)
                    {
                        std::cerr << "fail to get sanitize status log"
                                  << std::endl;
                        break;
                    }
                }
                break;
                default:
                {
                    std::cerr << "unknown lid for GetLogPage" << std::endl;
                    rc = -1;
                    errno = EINVAL;
                }
            }

            if (rc < 0)
            {
                std::cerr << "fail to get log page: " << std::strerror(errno)
                          << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)), {});
                });
                return;
            }
            else if (rc > 0)
            {
                std::string_view errMsg =
                    statusToString(static_cast<nvme_mi_resp_status>(rc));
                std::cerr << "fail to get log pag: " << errMsg << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(std::errc::bad_message), {});
                    return;
                });
            }

            self->io.post([cb{std::move(cb)}, data{std::move(data)}]() mutable {
                std::span<uint8_t> span{data.data(), data.size()};
                cb({}, span);
            });
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << "NVMeMi adminGetLogPage throws: " << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
}

void NVMeMi::adminXfer(
    nvme_mi_ctrl_t ctrl, const nvme_mi_admin_req_hdr& admin_req,
    std::span<uint8_t> data, unsigned int timeout_ms,
    std::function<void(const std::error_code&, const nvme_mi_admin_resp_hdr&,
                       std::span<uint8_t>)>&& cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {}, {});
        });
        return;
    }
    try
    {
        std::vector<uint8_t> req(sizeof(nvme_mi_admin_req_hdr) + data.size());
        memcpy(req.data(), &admin_req, sizeof(nvme_mi_admin_req_hdr));
        memcpy(req.data() + sizeof(nvme_mi_admin_req_hdr), data.data(),
               data.size());
        post([ctrl, req{std::move(req)}, self{shared_from_this()}, timeout_ms,
              cb{std::move(cb)}]() mutable {
            int rc = 0;

            nvme_mi_admin_req_hdr* reqHeader =
                reinterpret_cast<nvme_mi_admin_req_hdr*>(req.data());

            size_t respDataSize =
                boost::endian::little_to_native<size_t>(reqHeader->dlen);
            off_t respDataOffset =
                boost::endian::little_to_native<off_t>(reqHeader->doff);
            size_t bufSize = sizeof(nvme_mi_admin_resp_hdr) + respDataSize;
            std::vector<uint8_t> buf(bufSize);
            nvme_mi_admin_resp_hdr* respHeader =
                reinterpret_cast<nvme_mi_admin_resp_hdr*>(buf.data());

            // set timeout
            unsigned timeout = nvme_mi_ep_get_timeout(self->nvmeEP);
            nvme_mi_ep_set_timeout(self->nvmeEP, timeout_ms);

            rc = nvme_mi_admin_xfer(ctrl, reqHeader,
                                    req.size() - sizeof(nvme_mi_admin_req_hdr),
                                    respHeader, respDataOffset, &respDataSize);
            // revert to previous timeout
            nvme_mi_ep_set_timeout(self->nvmeEP, timeout);

            if (rc < 0)
            {
                std::cerr << "failed to nvme_mi_admin_xfer" << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)), {},
                       {});
                });
                return;
            }
            // the MI interface will only consume protocol/io errors
            // The client will take the reponsibility to deal with nvme-mi
            // status flag and nvme status field(cwd3). cmd specific return
            // value (cdw0) is also client's job.

            buf.resize(sizeof(nvme_mi_admin_resp_hdr) + respDataSize);
            self->io.post([cb{std::move(cb)}, data{std::move(buf)}]() mutable {
                std::span<uint8_t> span(
                    data.begin() + sizeof(nvme_mi_admin_resp_hdr), data.end());
                cb({}, *reinterpret_cast<nvme_mi_admin_resp_hdr*>(data.data()),
                   span);
            });
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {}, {});
        });
        return;
    }
}

void NVMeMi::adminFwCommit(
    nvme_mi_ctrl_t ctrl, nvme_fw_commit_ca action, uint8_t slot, bool bpid,
    std::function<void(const std::error_code&, nvme_status_field)>&& cb)
{
    if (!nvmeEP)
    {
        std::cerr << "nvme endpoint is invalid" << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device),
               nvme_status_field::NVME_SC_MASK);
        });
        return;
    }
    try
    {
        nvme_fw_commit_args args;
        memset(&args, 0, sizeof(args));
        args.args_size = sizeof(args);
        args.action = action;
        args.slot = slot;
        args.bpid = bpid;
        io.post([ctrl, args, cb{std::move(cb)},
                 self{shared_from_this()}]() mutable {
            int rc = nvme_mi_admin_fw_commit(ctrl, &args);
            if (rc < 0)
            {

                std::cerr << "fail to nvme_mi_admin_fw_commit: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}, last_errno{errno}]() {
                    cb(std::make_error_code(static_cast<std::errc>(last_errno)),
                       nvme_status_field::NVME_SC_MASK);
                });
                return;
            }
            else if (rc >= 0)
            {
                switch (rc & 0x7ff)
                {
                    case NVME_SC_SUCCESS:
                    case NVME_SC_FW_NEEDS_CONV_RESET:
                    case NVME_SC_FW_NEEDS_SUBSYS_RESET:
                    case NVME_SC_FW_NEEDS_RESET:
                        self->io.post([rc, cb{std::move(cb)}]() {
                            cb({}, static_cast<nvme_status_field>(rc));
                        });
                        break;
                    default:
                        std::string_view errMsg = statusToString(
                            static_cast<nvme_mi_resp_status>(rc));
                        std::cerr
                            << "fail to nvme_mi_admin_fw_commit: " << errMsg
                            << std::endl;
                        self->io.post([rc, cb{std::move(cb)}]() {
                            cb(std::make_error_code(std::errc::bad_message),
                               static_cast<nvme_status_field>(rc));
                        });
                }
                return;
            }
        });
    }
    catch (const std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device),
               nvme_status_field::NVME_SC_MASK);
        });
        return;
    }
}
