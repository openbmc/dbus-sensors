#pragma once

#include "NVMeSensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <memory>

class NVMeContext : public std::enable_shared_from_this<NVMeContext>
{
  public:
    NVMeContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeContext();

    void addSensor(std::shared_ptr<NVMeSensor> sensor)
    {
        sensors.emplace_back(sensor);
    }

    virtual void pollNVMeDevices();
    virtual void close();
    virtual void readAndProcessNVMeSensor();
    virtual void processResponse(void* msg, size_t len);

  private:
    boost::asio::deadline_timer scanTimer;
    int rootBus; // Root bus for this drive
    boost::asio::deadline_timer mctpResponseTimer;
    boost::asio::ip::tcp::socket nvmeSlaveSocket;
    std::list<std::shared_ptr<NVMeSensor>> sensors; // used as a poll queue

    void readResponse();
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

namespace nvmeMCTP
{
void init(void);
}

NVMEMap& getNVMEMap(void);
