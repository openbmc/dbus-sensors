#include "NVMeIntf.hpp"

#include <boost/asio.hpp>
#include <sdbusplus/bus.hpp>

#include <thread>

class NVMeMi : public NVMeMiIntf, public std::enable_shared_from_this<NVMeMi>
{
  public:
    NVMeMi(boost::asio::io_context& io, sdbusplus::bus_t& dbus, int bus,
           int addr);
    ~NVMeMi() override;

    int getNID() const override
    {
        return nid;
    }
    int getEID() const override
    {
        return eid;
    }
    void miSubsystemHealthStatusPoll(
        std::function<void(const std::error_code&,
                           nvme_mi_nvm_ss_health_status*)>&& cb) override;
    void miScanCtrl(std::function<void(const std::error_code&,
                                       const std::vector<nvme_mi_ctrl_t>&)>
                        cb) override;
    void adminIdentify(nvme_mi_ctrl_t ctrl, nvme_identify_cns cns,
                       uint32_t nsid, uint16_t cntid,
                       std::function<void(const std::error_code&,
                                          std::span<uint8_t>)>&& cb) override;
    void adminGetLogPage(nvme_mi_ctrl_t ctrl, nvme_cmd_get_log_lid lid,
                         uint32_t nsid, uint8_t lsp, uint16_t lsi,
                         std::function<void(const std::error_code&,
                                            std::span<uint8_t>)>&& cb) override;

  private:
    // the transfer size for nvme mi messages.
    // define in github.com/linux-nvme/libnvme/blob/master/src/nvme/mi.c
    static constexpr int nvme_mi_xfer_size = 4096;

    static nvme_root_t nvmeRoot;

    boost::asio::io_context& io;
    sdbusplus::bus_t& dbus;
    nvme_mi_ep_t nvmeEP;

    int nid;
    uint8_t eid;
    std::string mctpPath;

    bool workerStop;
    std::mutex workerMtx;
    std::condition_variable workerCv;
    boost::asio::io_context workerIO;
    std::thread thread;

    void post(std::function<void(void)>&& func);
};
