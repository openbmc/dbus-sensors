#pragma once

#include "FileHandle.hpp"
#include "NVMeIntf.hpp"

#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

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