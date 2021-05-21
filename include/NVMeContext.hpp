#pragma once

#include "NVMeSensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <memory>

struct NVMeContext : std::enable_shared_from_this<NVMeContext>
{
    NVMeContext(boost::asio::io_service& io, int rootBus);

    virtual ~NVMeContext();

    void pollNVMeDevices();
    void close();

    boost::asio::deadline_timer scanTimer;
    int rootBus; // Root bus for this drive
    boost::asio::deadline_timer mctpResponseTimer;
    boost::asio::ip::tcp::socket nvmeSlaveSocket;
    std::list<std::shared_ptr<NVMeSensor>> sensors; // used as a poll queue
};

using NVMEMap = boost::container::flat_map<int, std::shared_ptr<NVMeContext>>;

namespace nvmeMCTP
{
void init(void);
}

NVMEMap& getNVMEMap(void);
