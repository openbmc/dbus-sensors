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

#include "PSUEvent.hpp"

#include <systemd/sd-journal.h>

#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

PSUCombineEvent::PSUCombineEvent(
    sdbusplus::asio::object_server& objectServer, boost::asio::io_service& io,
    const std::string& psuName,
    boost::container::flat_map<std::string, std::vector<std::string>>&
        eventPathList,
    const std::string& combineEventName) :
    objServer(objectServer)
{
    eventInterface = objServer.add_interface(
        "/xyz/openbmc_project/State/Decorator/" + psuName + "_" +
            combineEventName,
        "xyz.openbmc_project.State.Decorator.OperationalStatus");
    eventInterface->register_property("functional", true);

    if (!eventInterface->initialize())
    {
        std::cerr << "error initializing event interface\n";
    }

    std::shared_ptr<std::set<std::string>> combineEvent =
        std::make_shared<std::set<std::string>>();
    for (const auto& pathList : eventPathList)
    {
        const std::string& eventName = pathList.first;
        std::string eventPSUName = eventName + psuName;
        for (const auto& path : pathList.second)
        {
            std::shared_ptr<std::set<std::string>> assert =
                std::make_shared<std::set<std::string>>();
            std::shared_ptr<bool> state = std::make_shared<bool>(false);
            events[eventPSUName].emplace_back(std::make_unique<PSUSubEvent>(
                eventInterface, path, io, eventName, assert, combineEvent,
                state, psuName));
            asserts.emplace_back(assert);
            states.emplace_back(state);
        }
    }
}

PSUCombineEvent::~PSUCombineEvent()
{
    events.clear();
    objServer.remove_interface(eventInterface);
}

static boost::container::flat_map<std::string,
                                  std::pair<std::string, std::string>>
    logID = {
        {"PredictiveFailure",
         {"OpenBMC.0.1.PowerSupplyFailurePredicted",
          "OpenBMC.0.1.PowerSupplyPredictedFailureRecovered"}},
        {"Failure",
         {"OpenBMC.0.1.PowerSupplyFailed", "OpenBMC.0.1.PowerSupplyRecovered"}},
        {"ACLost",
         {"OpenBMC.0.1.PowerSupplyPowerLost",
          "OpenBMC.0.1.PowerSupplyPowerRestored"}},
        {"FanFault",
         {"OpenBMC.0.1.PowerSupplyFanFailed",
          "OpenBMC.0.1.PowerSupplyFanRecovered"}},
        {"ConfigureError", {"OpenBMC.0.1.PowerSupplyConfigurationError", ""}}};

PSUSubEvent::PSUSubEvent(
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
    const std::string& path, boost::asio::io_service& io,
    const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<std::set<std::string>> combineEvent,
    std::shared_ptr<bool> state, const std::string& psuName) :
    eventInterface(eventInterface),
    inputDev(io, open(path.c_str(), O_RDONLY)), waitTimer(io), errCount(0),
    path(path), eventName(eventName), assertState(state), asserts(asserts),
    combineEvent(combineEvent), psuName(psuName)
{
    auto found = logID.find(eventName);
    if (found == logID.end())
    {
        assertMessage.clear();
        deassertMessage.clear();
    }
    else
    {
        assertMessage = found->second.first;
        deassertMessage = found->second.second;
    }

    auto fanPos = path.find("fan");
    if (fanPos != std::string::npos)
    {
        fanName = path.substr(fanPos);
        auto fanNamePos = fanName.find("_");
        if (fanNamePos != std::string::npos)
        {
            fanName = fanName.substr(0, fanNamePos);
        }
    }
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
    if (fd < 0)
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
// asserted. Only if none of the sub events are asserted, the event will be
// deasserted.
void PSUSubEvent::updateValue(const int& newValue)
{
    if (newValue == 0)
    {
        auto found = (*asserts).find(path);
        if (found == (*asserts).end())
        {
            return;
        }
        (*asserts).erase(path);

        if (!(*asserts).empty())
        {
            return;
        }
        if (*assertState == true)
        {
            *assertState = false;
            auto foundCombine = (*combineEvent).find(eventName);
            if (foundCombine == (*combineEvent).end())
            {
                return;
            }
            (*combineEvent).erase(eventName);
            if (!deassertMessage.empty())
            {
                // Fan Failed has two args
                std::string sendMessage = eventName + " deassert";
                if (deassertMessage == "OpenBMC.0.1.PowerSupplyFanRecovered")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_ERR, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s,%s",
                        psuName.c_str(), fanName.c_str(), NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_ERR, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s",
                        psuName.c_str(), NULL);
                }
            }

            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", true);
            }
        }
    }
    else
    {
        (*asserts).emplace(path);
        if (*assertState == false)
        {
            *assertState = true;
            if (!assertMessage.empty())
            {
                // Fan Failed has two args
                std::string sendMessage = eventName + " assert";
                if (assertMessage == "OpenBMC.0.1.PowerSupplyFanFailed")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_ERR, "REDFISH_MESSAGE_ID=%s", assertMessage.c_str(),
                        "REDFISH_MESSAGE_ARGS=%s,%s", psuName.c_str(),
                        fanName.c_str(), NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_ERR, "REDFISH_MESSAGE_ID=%s", assertMessage.c_str(),
                        "REDFISH_MESSAGE_ARGS=%s", psuName.c_str(), NULL);
                }
            }
            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", false);
            }
            (*combineEvent).emplace(eventName);
        }
    }
    value = newValue;
}
