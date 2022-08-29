#pragma once
#include <functional>
#include <memory>
#include <variant>

class NVMeBasicIntf;
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
        throw std::runtime_error("uninitiated NVMeIntf");
    }

  private:
    std::variant<std::shared_ptr<NVMeBasicIntf>> interface;
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
    virtual int getAddress() const = 0;

    // Get NVMe drive status, data address is from 00h~07h
    virtual void getStatus(
        std::function<void(const std::error_code&, DriveStatus*)>&& cb) = 0;

    virtual ~NVMeBasicIntf() = default;
};
