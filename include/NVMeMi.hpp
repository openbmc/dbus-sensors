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

  private:
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
