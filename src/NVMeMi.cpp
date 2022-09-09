#include "NVMeMi.hpp"

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
    auto msg = dbus.new_method_call(
        "xyz.openbmc_project.MCTP", "/xyz/openbmc_project/mctp",
        "au.com.CodeConstruct.MCTP", "SetupEndpoint");

    msg.append("mctpi2c" + std::to_string(bus));
    msg.append(static_cast<uint8_t>(addr));
    auto reply = msg.call(); // throw SdBusError

    reply.read(eid);
    reply.read(nid);
    reply.read(mctpPath);

    // open mctp endpoint
    auto nvmeEP = nvme_mi_open_mctp(nvmeRoot, nid, eid);
    if (!nvmeEP)
    {
        throw std::runtime_error("can't open MCTP endpoint " +
                                 std::to_string(nid) + ":" +
                                 std::to_string(eid));
    }

    // start worker thread;
    thread = std::jthread([&io = workerIO]() { io.run(); });
}

NVMeMi::~NVMeMi()
{
    workerIO.stop();
    if (nvmeEP)
    {
        nvme_mi_close(nvmeEP);
    }
}

void NVMeMi::miSubsystemHealthStatusPoll(
    std::function<void(const std::error_code&, nvme_mi_nvm_ss_health_status*)>&&
        cb)
{
    if (!nvmeEP || workerIO.stopped())
    {
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), nullptr);
        });
        return;
    }

    workerIO.post([self{shared_from_this()}, cb{std::move(cb)}]() {
        nvme_mi_nvm_ss_health_status ss_health;
        auto rc = nvme_mi_mi_subsystem_health_status_poll(self->nvmeEP, true,
                                                          &ss_health);
        if (rc)
        {
            std::cerr << "can't perform Health Status Poll operation"
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

void NVMeMi::miScanCtrl(std::function<void(const std::error_code&,
                                           const std::vector<nvme_mi_ctrl_t>&)>
                            cb)
{
    if (!nvmeEP || workerIO.stopped())
    {
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }

    workerIO.post([self{shared_from_this()}, cb{std::move(cb)}]() {
        int rc = nvme_mi_scan_ep(self->nvmeEP, true);
        if (rc)
        {
            std::cerr << "can't scan controllers" << std::endl;
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
        self->io.post([cb{std::move(cb)}, list{std::move(list)}]() {
            cb({}, list);
        });
    });
}

void NVMeMi::adminIdentify(
    nvme_mi_ctrl_t ctrl, nvme_identify_cns cns, uint32_t nsid, uint16_t cntid,
    std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb)
{
    if (!nvmeEP || workerIO.stopped())
    {
        io.post([cb{std::move(cb)}]() {
            cb(std::make_error_code(std::errc::no_such_device), {});
        });
        return;
    }

    workerIO.post([=, self{shared_from_this()}, cb{std::move(cb)}]() {
        int rc = 0;
        std::vector<uint8_t> data;
        switch (cns)
        {
            default:
            {
                data.resize(NVME_IDENTIFY_DATA_SIZE);
                nvme_identify_args args;
                args.data = data.data();
                args.cns = cns;
                args.nsid = nsid;
                args.cntid = cntid;

                rc = nvme_mi_admin_identify(ctrl, &args);
            }
        }
        if (rc)
        {
            std::cerr << "can't scan controllers" << std::endl;
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
