#pragma once
#include "NVMeController.hpp"
#include "NVMeSubsys.hpp"
#include "Utils.hpp"

class NVMePlugin;

class NVMeControllerPlugin
{
  public:
    using getlogpage_t = std::function<void(
        uint8_t lid, uint32_t nsid, uint8_t lsp, uint16_t lsi,
        std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb)>;

    // The controller plugin can only be created from NVMePlugin
    NVMeControllerPlugin(std::shared_ptr<NVMeController> cntl,
                         const SensorData&) :
        nvmeController(cntl)
    {}

    virtual ~NVMeControllerPlugin()
    {}
    virtual getlogpage_t getGetLogPageHandler()
    {
        return {};
    }

  protected:
    const std::string& getPath() const
    {
        return nvmeController->path;
    }
    sdbusplus::asio::object_server& getDbusServer()
    {
        return nvmeController->objServer;
    }
    std::shared_ptr<sdbusplus::asio::connection> getDbusConnection()
    {
        return nvmeController->conn;
    }

    boost::asio::io_context& getIOContext()
    {
        return nvmeController->io;
    }

    /**
     * adminXfer() -  transfer Raw admin cmd to the binded conntroller
     * @admin_req: request header
     * @data: request data payload
     * @timeout_ms: timeout in ms
     * @resp_data_offset: offset into request data to retrieve from controller
     * @cb: callback function after the response received.
     * @ec: error code
     * @admin_resp: response header
     * @resp_data: response data payload
     *
     * Performs an arbitrary NVMe Admin command, using the provided request
     * header, in @admin_req. The requested data is attached by @data, if any.
     *
     * On success, @cb will be called and response header and data are stored
     * in
     * @admin_resp and @resp_data, which has an optional appended payload
     * buffer. The response data does not include the Admin request header, so
     * 0 represents no payload.
     *
     * As with all Admin commands, we can request partial data from the Admin
     * Response payload, offset by @resp_data_offset. In case of resp_data
     * contains only partial data of the caller's requirement, a follow-up
     * call to adminXfer with offset is required.
     *
     * See: &struct nvme_mi_admin_req_hdr and &struct nvme_mi_admin_resp_hdr.
     *
     * @ec will be returned on failure.
     */
    void adminXfer(const nvme_mi_admin_req_hdr& admin_req,
                   std::span<uint8_t> data, unsigned int timeout_ms,
                   std::function<void(const std::error_code& ec,
                                      const nvme_mi_admin_resp_hdr& admin_resp,
                                      std::span<uint8_t> resp_data)>&& cb)
    {
        nvmeController->nvmeIntf->adminXfer(nvmeController->nvmeCtrl, admin_req,
                                            data, timeout_ms, std::move(cb));
    }
    /**
     * @brief Get cntrl_id for the binded NVMe controller
     *
     * @return cntrl_id
     */
    uint16_t getCntrlId() const
    {
        return nvmeController->getCntrlId();
    }

  private:
    std::shared_ptr<NVMeController> nvmeController;
};

class NVMePlugin
{
  public:
    NVMePlugin(std::shared_ptr<NVMeSubsystem> subsys,
               const SensorData& /*config*/) :
        subsystem(std::move(subsys)){};

    virtual ~NVMePlugin()
    {}

    std::shared_ptr<NVMeControllerPlugin>
        createControllerPlugin(const NVMeController& controller,
                               const SensorData& config)
    {
        // searching for the target controller in NVMe subsystem
        auto res = subsystem->controllers.find(controller.getCntrlId());
        if (res == subsystem->controllers.end() ||
            &controller != res->second.first.get())
        {
            throw std::runtime_error("Failed to create controller plugin: "
                                     "cannot find the controller");
        }

        // insert the plugin
        res->second.second = makeController(res->second.first, config);
        return res->second.second;
    }

    // the NVMe subsystem will start the plugin after NVMesubsystem finished
    // intialization and started.
    virtual void start()
    {
        return;
    }

    // the NVMe subsystem will stop the plugin before NVMe subsystem stop
    // itself.
    virtual void stop()
    {
        return;
    }

  protected:
    const std::string& getPath() const
    {
        return subsystem->path;
    }
    const std::string& getName() const
    {
        return subsystem->name;
    }
    boost::asio::io_context& getIOContext()
    {
        return subsystem->io;
    }
    sdbusplus::asio::object_server& getDbusServer()
    {
        return subsystem->objServer;
    }
    std::shared_ptr<sdbusplus::asio::connection> getDbusConnection()
    {
        return subsystem->conn;
    }

    const std::map<uint16_t, std::pair<std::shared_ptr<NVMeController>,
                                       std::shared_ptr<NVMeControllerPlugin>>>&
        getControllers()
    {
        return subsystem->controllers;
    }
    // The nvme plugin implemenation need to overload the function to create a
    // derived controller plugin.
    virtual std::shared_ptr<NVMeControllerPlugin>
        makeController(std::shared_ptr<NVMeController> cntl,
                       const SensorData&) = 0;

  private:
    std::shared_ptr<NVMeSubsystem> subsystem;
};
