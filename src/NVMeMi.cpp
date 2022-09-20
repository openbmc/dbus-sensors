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
            if (rc)
            {

                std::cerr << "fail to subsystem_health_status_poll: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(static_cast<std::errc>(errno)),
                       nullptr);
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
            if (rc)
            {
                std::cerr << "fail to scan controllers" << std::endl;
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
            if (rc)
            {
                std::cerr << "fail to do nvme identify: "
                          << std::strerror(errno) << std::endl;
                self->io.post([cb{std::move(cb)}]() {
                    cb(std::make_error_code(static_cast<std::errc>(errno)), {});
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

// Get Temetery Log header and return the size for hdr + data area (Area 1, 2,
// 3, or maybe 4)
void getTelemetryLogHost(nvme_mi_ctrl_t ctrl, bool create,
                         std::vector<uint8_t>& data)
{
    int rc = 0;
    data.resize(sizeof(nvme_telemetry_log));
    nvme_telemetry_log& log =
        *reinterpret_cast<nvme_telemetry_log*>(data.data());
    if (create)
    {
        rc = nvme_mi_admin_get_log_create_telemetry_host(ctrl, &log);
        if (rc)
        {
            std::cerr << "failed to create telemetry host log" << std::endl;
            throw std::system_error(errno, std::generic_category());
        }
        return;
    }
    else
    {
        rc = nvme_mi_admin_get_log_telemetry_host(ctrl, 0, sizeof(log), &log);
    }
    if (rc)
    {
        std::cerr << "failed to retain telemetry host log" << std::endl;
        throw std::system_error(errno, std::generic_category());
    }

    long size =
        static_cast<long>((boost::endian::little_to_native(log.dalb3) + 1)) *
        NVME_LOG_TELEM_BLOCK_SIZE;

    data.resize(size);
    rc =
        nvme_mi_admin_get_log_telemetry_host(ctrl, 0, data.size(), data.data());
    if (rc)
    {
        std::cerr << "failed to get full telemetry host log" << std::endl;
        throw std::system_error(errno, std::generic_category());
    }
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
            try
            {
                switch (lid)
                {
                    case NVME_LOG_LID_TELEMETRY_HOST:
                    {
                        bool create;
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
                            throw std::system_error(std::make_error_code(
                                std::errc::invalid_argument));
                        }
                        getTelemetryLogHost(ctrl, create, data);
                        break;
                        default:
                        {
                            std::cerr << "unknown lid for GetLogPage"
                                      << std::endl;
                            throw std::system_error(std::make_error_code(
                                std::errc::invalid_argument));
                        }
                    }
                }
            }
            catch (const std::system_error& e)
            {
                self->io.post(
                    [cb{std::move(cb)}, ec = e.code()]() { cb(ec, {}); });
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
        std::cerr << "NVMeMi adminGetLogPage throws: " << e.what() << std::endl;
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }
}
