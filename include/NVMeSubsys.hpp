#pragma once
#include "NVMeBasic.hpp"
#include "NVMeController.hpp"
#include "NVMeDrive.hpp"
#include "NVMeSensor.hpp"
#include "NVMeStorage.hpp"
#include "Utils.hpp"

class NVMeControllerPlugin;
class NVMePlugin;

class NVMeSubsystem : public std::enable_shared_from_this<NVMeSubsystem>
{
  public:
    static constexpr const char* sensorType = "NVME1000";

    NVMeSubsystem(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  const std::string& path, const std::string& name,
                  const SensorData& configData, NVMeIntf intf);

    ~NVMeSubsystem();

    void start();

    void stop();

  private:
    friend class NVMePlugin;
    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string path;
    std::string name;
    SensorData config;

    NVMeIntf nvmeIntf;

    enum class Status
    {
        Stop,
        Intiatilzing,
        Start,
    };

    Status status;

    // plugin
    std::shared_ptr<NVMePlugin> plugin;

    /* thermal sensor for the subsystem */
    std::shared_ptr<NVMeSensor> ctemp;
    std::shared_ptr<boost::asio::steady_timer> ctempTimer;

    /*
    Storage interface: xyz.openbmc_project.Inventory.Item.Storage
    */
    NVMeStorage storage;

    /*
    Drive interface: xyz.openbmc_project.Inventory.Item.Drive
    */
    NVMeDrive drive;

    // map from cntrlid to a pair of {controller, controller_plugin}
    std::map<uint16_t, std::pair<std::shared_ptr<NVMeController>,
                                 std::shared_ptr<NVMeControllerPlugin>>>
        controllers{};

    std::shared_ptr<sdbusplus::asio::dbus_interface> assocIntf;
    void createStorageAssociation();

    // make the subsystem functional/functional be enabling/disabling the
    // storage controller, namespaces and thermal sensors.
    void markFunctional(bool toggle);
};
