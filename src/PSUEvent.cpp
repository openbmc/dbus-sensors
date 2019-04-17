/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <unistd.h>

#include <PSUEvent.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

PSUEvent::PSUEvent(const std::vector<std::string>& paths,
                   sdbusplus::asio::object_server& objectServer,
                   boost::asio::io_service& io, const std::string& psuName,
                   const std::string& eventName) :
    objServer(objectServer),
    paths(paths), eventName(eventName), name(psuName)
{
    eventInterface = objServer.add_interface(
        "/xyz/openbmc_project/event/" + name + "_" + eventName,
        "xyz.openbmc_project.Control.PowerSupplyEvent");
    eventInterface->register_property(eventName, std::string("deassert"));

    if (!eventInterface->initialize())
    {
        std::cerr << "error initializing event interface\n";
    }

    int index = 0;
    std::shared_ptr<int> assert = std::make_shared<int>(0);
    for (auto path : paths)
    {
        subEvents[index] = std::make_unique<PSUSubEvent>(
            eventInterface, path, objectServer, io, eventName, assert);
        index++;
    }
}

PSUEvent::~PSUEvent()
{
    objServer.remove_interface(eventInterface);
}

PSUSubEvent::PSUSubEvent(
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
    const std::string& path, sdbusplus::asio::object_server& objectServer,
    boost::asio::io_service& io, const std::string& eventName,
    std::shared_ptr<int> asserts) :
    eventInterface(eventInterface),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0), path(path), eventName(eventName)
{
    assert = asserts;
    setupRead();
}

void PSUSubEvent::setupRead(void)
{
    boost::asio::async_read_until(
        inputDev, readBuf, '\n',
        [&](const boost::system::error_code& ec,
            std::size_t /*bytes_transfered*/) { handleResponse(ec); });
}

PSUSubEvent::~PSUSubEvent()
{
    inputDev.close();
    waitTimer.cancel();
}

void PSUSubEvent::handleResponse(const boost::system::error_code& err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return;
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            int nvalue = std::stof(response);
            responseStream.clear();
            if (nvalue != value)
            {
                updateValue(nvalue);
            }
            errCount = 0;
        }
        catch (const std::invalid_argument&)
        {
            errCount++;
        }
    }
    else
    {
        errCount++;
    }
    if (errCount >= warnAfterErrorCount)
    {
        if (errCount == warnAfterErrorCount)
        {
            std::cerr << "Failure to read event at " << path << "\n";
        }
        updateValue(0);
        errCount++;
    }
    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return;
    }
    inputDev.assign(fd);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        setupRead();
    });
}

void PSUSubEvent::updateValue(const int& newValue)
{
    if (newValue == 0)
    {
        (*assert)--;
        if (*assert == 0)
        {
            eventInterface->set_property(eventName, std::string("deassert"));
        }
        if (*assert < 0)
        {
            *assert = 0;
        }
    }
    else
        (*assert)++;
    if (*assert == 1)
    {
        eventInterface->set_property(eventName, std::string("assert"));
    }
    value = newValue;
}
