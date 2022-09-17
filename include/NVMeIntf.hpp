#pragma once
#include <libnvme-mi.h>

#include <functional>
#include <memory>
#include <span>

class NVMeIntf
{
  public:
    NVMeIntf() = default;
    virtual ~NVMeIntf() = default;
};

// Interface to get information via NVMe MI Basic CMD protocol.
class NVMeBasicIntf : public NVMeIntf
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

    ~NVMeBasicIntf() override = default;
};

class NVMeMiIntf : public NVMeIntf
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
    virtual void adminIdentify(
        nvme_mi_ctrl_t ctrl, nvme_identify_cns cns, uint32_t nsid,
        uint16_t cntid,
        std::function<void(const std::error_code&, std::span<uint8_t>)>&&
            cb) = 0;
};
