#pragma once
class NVMePlugin
{
  public:
    using getlogpage_t = std::function<void(
        uint8_t lid, uint32_t nsid, uint8_t lsp, uint16_t lsi,
        std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb)>;
    NVMePlugin(std::shared_ptr<NVMeController> cntl) : nvmeController(cntl)
    {}
    virtual ~NVMePlugin()
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
    sdbusplus::asio::connection& getDbusConnection()
    {
        return *nvmeController->conn;
    }

    boost::asio::io_context& getIOContext()
    {
        return nvmeController->io;
    }

    /**
     * adminXfer() -  transfer Raw admin cmd to the binded conntroller
     * @admin_req: request header
     * @data: request data payload
     * @resp_data_offset: offset into request data to retrieve from controller
     * @cb: callback function after the response received.
     * @ec: error code
     * @admin_resp: response header
     * @resp_data: response data payload
     *
     * Performs an arbitrary NVMe Admin command, using the provided request
     * header, in @admin_req. The requested data is attached by @data, if any.
     *
     * On success, @cb will be called and response header and data are stored in
     * @admin_resp and @resp_data, which has an optional appended payload
     * buffer. The response data does not include the Admin request header, so 0
     * represents no payload.
     *
     * As with all Admin commands, we can request partial data from the Admin
     * Response payload, offset by @resp_data_offset. In case of resp_data
     * contains only partial data of the caller's requirement, a follow-up call
     * to adminXfer with offset is required.
     *
     * See: &struct nvme_mi_admin_req_hdr and &struct nvme_mi_admin_resp_hdr.
     *
     * @ec will be returned on failure.
     */
    void adminXfer(const nvme_mi_admin_req_hdr& admin_req,
                   std::span<uint8_t> data,
                   std::function<void(const std::error_code& ec,
                                      const nvme_mi_admin_resp_hdr& admin_resp,
                                      std::span<uint8_t> resp_data)>&& cb)
    {
        nvmeController->nvmeIntf->adminXfer(nvmeController->nvmeCtrl, admin_req,
                                            data, std::move(cb));
    }
    /**
     * @brief Get cntrl_id for the binded NVMe controller
     *
     * @return cntrl_id
     */
    uint16_t getCntrlId()
    {
        return *reinterpret_cast<uint16_t*>(
            (reinterpret_cast<uint8_t*>(nvmeController->nvmeCtrl) +
             std::max(sizeof(uint16_t), sizeof(void*))));
    }

  private:
    std::shared_ptr<NVMeController> nvmeController;
};
