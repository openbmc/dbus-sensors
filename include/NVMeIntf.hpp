#pragma once
#include <libnvme-mi.h>

#include <functional>
#include <memory>
#include <span>
#include <variant>

class NVMeBasicIntf;
class NVMeMiIntf;
/**
 * @brief a container class to hold smart ptr to NVMe Basic or NVMe MI
 * implementation instance
 */
class NVMeIntf
{
  public:
    enum class Protocol
    {
        NVMeBasic,
        NVMeMI,
    };

    NVMeIntf() = default;

    template <class IntfImpl, class... Args>
    static NVMeIntf create(Args&&... args)
    {
        NVMeIntf nvmeIntf;
        if constexpr (std::is_base_of_v<NVMeBasicIntf, IntfImpl>)
        {
            nvmeIntf.interface =
                std::make_shared<IntfImpl>(std::forward<Args>(args)...);
            return nvmeIntf;
        }

        if constexpr (std::is_base_of_v<NVMeMiIntf, IntfImpl>)
        {
            nvmeIntf.interface =
                std::make_shared<IntfImpl>(std::forward<Args>(args)...);
            return nvmeIntf;
        }

        throw std::runtime_error("Unsupported NVMe interface");
    }

    auto getInferface()
    {
        return interface;
    }

    Protocol getProtocol()
    {
        if (std::holds_alternative<std::shared_ptr<NVMeBasicIntf>>(interface))
        {
            return Protocol::NVMeBasic;
        }
        else if (std::holds_alternative<std::shared_ptr<NVMeMiIntf>>(interface))
        {
            return Protocol::NVMeMI;
        }

        throw std::runtime_error("uninitiated NVMeIntf");
    }

  private:
    std::variant<std::shared_ptr<NVMeBasicIntf>, std::shared_ptr<NVMeMiIntf>>
        interface;
};

/**
 * @brief Interface to get information via NVMe MI Basic CMD protocol.
 *
 * Can be used for implementation or mockup
 */
class NVMeBasicIntf
{
  public:
    struct DriveStatus
    {
        uint8_t SmartWarnings;
        uint8_t Temp;
        uint8_t DriveLifeUsed;
        uint8_t WarningTemp;
        uint8_t PowerState;
    };

    enum StatusFlags : uint8_t
    {
        NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY = 0x40,
        NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL = 0x20,
    };

    NVMeBasicIntf() = default;

    // The i2c bus number
    virtual int getBus() const = 0;
    // The i2c address for NVMe Basic
    virtual int getAddr() const = 0;

    // Get NVMe drive status, data address is from 00h~07h
    virtual void getStatus(
        std::function<void(const std::error_code&, DriveStatus*)>&& cb) = 0;

    virtual ~NVMeBasicIntf() = default;
};

class NVMeMiIntf
{
  public:
    virtual int getNID() const = 0;
    virtual int getEID() const = 0;
    virtual void miSubsystemHealthStatusPoll(
        std::function<void(const std::error_code&,
                           nvme_mi_nvm_ss_health_status*)>&& cb) = 0;
    virtual void
        miScanCtrl(std::function<void(const std::error_code&,
                                      const std::vector<nvme_mi_ctrl_t>&)>
                       cb) = 0;

    virtual ~NVMeMiIntf() = default;

    virtual void adminIdentify(
        nvme_mi_ctrl_t ctrl, nvme_identify_cns cns, uint32_t nsid,
        uint16_t cntid,
        std::function<void(const std::error_code&, std::span<uint8_t>)>&&
            cb) = 0;
};
