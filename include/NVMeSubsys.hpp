#include "NVMeBasic.hpp"
#include "NVMeController.hpp"
#include "NVMeDrive.hpp"
#include "NVMeSensor.hpp"
#include "NVMeStorage.hpp"
#include "Utils.hpp"
class NVMeSubsys : public std::enable_shared_from_this<NVMeSubsys>
{
  public:
    static constexpr const char* configType =
        "xyz.openbmc_project.Configuration.NVME1000";

    NVMeSubsys(boost::asio::io_context& io,
               sdbusplus::asio::object_server& objServer,
               std::shared_ptr<sdbusplus::asio::connection> conn,
               std::string path, std::string name, const SensorData& configData,
               const std::shared_ptr<NVMeIntf>& intf);

    void start();

    void stop()
    {
        ctempTimer.cancel();
    }

  private:
    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string path;
    std::string name;

    std::shared_ptr<NVMeIntf> nvmeIntf;

    /* thermal sensor for the subsystem */
    std::optional<NVMeSensor> ctemp;
    boost::asio::deadline_timer ctempTimer;

    template <class T>
    void pollCtemp(
        const std::function<void(
            std::function<void(const std::error_code&, T)>&&)>& dataFetcher,
        const std::function<std::optional<double>(T Data)>& dataParser);

    /*
    Storage interface: xyz.openbmc_project.Inventory.Item.Storage
    */
    NVMeStorage storage;

    /*
    Drive interface: xyz.openbmc_project.Inventory.Item.Drive
    */
    NVMeDrive drive;

    // map from cntrlid to controller instances
    std::map<uint16_t, std::shared_ptr<NVMeController>> controllers;
    // TODO: std::map<int, NVMeNamespace> namespaces;
};