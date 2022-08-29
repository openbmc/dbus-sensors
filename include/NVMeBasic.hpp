#pragma once

#include "FileHandle.hpp"

#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

#include <memory>
#include <thread>

class NVMeBasicIO
{
  public:
    NVMeBasicIO(boost::asio::io_context& io,
                std::function<ssize_t(FileHandle& in, FileHandle& out)>&& f);
    boost::asio::posix::stream_descriptor reqStream;
    boost::asio::posix::stream_descriptor respStream;

  private:
    std::jthread thread;
};

class NVMeIntf
{
  public:
    NVMeIntf() = default;
    virtual ~NVMeIntf(){};
};

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
        uint8_t PEC;
    };

    enum
    {
        NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY = 0x40,
        NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL = 0x20,
    };

    NVMeBasicIntf() = default;
    virtual int getBus() const = 0;
    virtual int getAddr() const = 0;
    virtual void getStatus(
        std::function<void(const std::error_code&, DriveStatus*)>&& cb) = 0;
    virtual ~NVMeBasicIntf() override{};
};

// NVMe Basic Management Command
// NVMe MI Spec Appendix A.
class NVMeBasic :
    public NVMeBasicIntf,
    public std::enable_shared_from_this<NVMeBasic>
{
  public:
    NVMeBasic(boost::asio::io_context& io, int bus, int addr);

    int getBus() const override
    {
        return bus;
    }
    int getAddr() const override
    {
        return addr;
    }
    void getStatus(std::function<void(const std::error_code&, DriveStatus*)>&&
                       cb) override;
    virtual ~NVMeBasic() override{};

  private:
    boost::asio::io_context& io;
    int bus;
    int addr;
    std::shared_ptr<NVMeBasicIO> basicIO;
};