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

#include <PSUEvent.hpp>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

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
    eventInterface->register_property(eventName, bool(false));

    if (!eventInterface->initialize())
    {
        std::cerr << "error initializing event interface\n";
    }

    int index = 0;
    std::shared_ptr<std::set<std::string>> assert =
        std::make_shared<std::set<std::string>>();
    std::shared_ptr<bool> state = std::make_shared<bool>(false);
    for (const auto path : paths)
    {
        subEvents[index] = std::make_unique<PSUSubEvent>(
            eventInterface, path, objectServer, io, eventName, assert, state);
        index++;
    }
}

PSUEvent::~PSUEvent()
{
    objServer.remove_interface(eventInterface);
}

PSUSubEvent::PSUSubEvent(
    std::shared_ptr<sdbusplus::asio::dbus_interface>& eventInterface,
    const std::string& path, sdbusplus::asio::object_server& objectServer,
    boost::asio::io_service& io, const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<bool> state) :
    eventInterface(eventInterface),
    objServer(objectServer), inputDev(io, open(path.c_str(), O_RDONLY)),
    waitTimer(io), errCount(0), path(path), eventName(eventName),
    assertState(state)
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
    waitTimer.expires_from_now(boost::posix_time::milliseconds(eventPollMs));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        setupRead();
    });
}

// Any of the sub events of one event is asserted, then the event will be
// asserted. Only if non of the sub events are asserted, the event will be
// deasserted.
void PSUSubEvent::updateValue(const int& newValue)
{
    if (newValue == 0)
    {
        auto found = (*assert).find(path);
        if (found != (*assert).end())
        {
            (*assert).erase(found);

            if ((*assert).empty())
            {
                if (*assertState == true)
                {
                    *assertState = false;
                    eventInterface->set_property(eventName, *assertState);
                }
            }
        }
    }
    else
    {
        (*assert).emplace(path);
        if (*assertState == false)
        {
            *assertState = true;
            eventInterface->set_property(eventName, *assertState);
        }
    }
    value = newValue;
}
