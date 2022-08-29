#pragma once
#include <functional>
#include <memory>

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
