#pragma once

#include "NVMeContext.hpp"

#include <boost/asio/experimental/concurrent_channel.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

#include <thread>

struct MessageRequest
{
    int bus = 0;
    uint8_t device = 0;
    uint8_t offset = 0;
    uint8_t len = 0;
};

struct MessageResponse
{
    std::span<uint8_t> validData;
    std::array<uint8_t, UINT8_MAX + 1> buffer;
};

class NVMeBasicContext : public NVMeContext
{
  public:
    NVMeBasicContext(boost::asio::io_context& io, int rootBus);
    ~NVMeBasicContext() override = default;
    void pollNVMeDevices() override;
    void readAndProcessNVMeSensor() override;
    void processResponse(std::shared_ptr<NVMeSensor>& sensor,
                         std::span<uint8_t> messageData) override;

  private:
    NVMeBasicContext(boost::asio::io_context& io, int rootBus, int cmdOut,
                     int streamIn, int streamOut, int cmdIn);
    void startThreadReceive();

    boost::asio::io_context& io;

    // The IO thread must be destructed after the stream descriptors, so
    // initialise it first. http://eel.is/c++draft/class.base.init#note-6
    //
    // Providing a stop-source to the thread execution function isn't
    // particularly useful as it will spend most of its time blocked in a system
    // call - ioctl() for the actual device communication, or read() and write()
    // on the pipes associated with reqStream and respStream. Rather than trying
    // to force a stop, rely on read()/write() failures from closed pipes to
    // coerce it to exit and thus allow completion of the join().
    std::jthread thread;

    boost::asio::io_context threadIo;

    // Destruction of the stream descriptors has the effect of issuing cancel(),
    // destroying the closure of the callback where we might be carrying
    // weak_ptrs to `this`.
    boost::asio::experimental::concurrent_channel<void(
        boost::system::error_code, MessageRequest)>
        toThread;
    boost::asio::experimental::concurrent_channel<void(
        boost::system::error_code, MessageResponse)>
        fromThread;

    enum
    {
        NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY = 0x40,
        NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL = 0x20,
    };
};
