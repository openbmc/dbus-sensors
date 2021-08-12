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

#include <systemd/sd-journal.h>

#include <PSUEvent.hpp>
#include <SensorPaths.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

PSUCombineEvent::PSUCombineEvent(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& psuName,
    const boost::container::flat_map<
        std::string, std::vector<std::pair<std::string, std::string>>>&
        eventPathList,
    const boost::container::flat_map<
        std::string,
        boost::container::flat_map<
            std::string, std::vector<std::pair<std::string, std::string>>>>&
        groupEventPathList,
    const std::string& combineEventName, double pollRate) :
    objServer(objectServer)
{
    std::string psuNameEscaped = sensor_paths::escapePathForDbus(psuName);
    eventInterface = objServer.add_interface(
        "/xyz/openbmc_project/State/Decorator/" + psuNameEscaped + "_" +
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
        std::shared_ptr<std::set<std::string>> assert =
            std::make_shared<std::set<std::string>>();
        std::shared_ptr<bool> state = std::make_shared<bool>(false);

        const std::string& eventName = pathList.first;
        std::string eventPSUName = eventName + psuName;
        for (const auto& namedPath : pathList.second)
        {
            auto p = std::make_shared<PSUSubEvent>(
                eventInterface, namedPath.second, namedPath.first, conn, io,
                eventName, eventName, assert, combineEvent, state, psuName,
                pollRate);
            p->setupRead();

            events[eventPSUName].emplace_back(p);
            asserts.emplace_back(assert);
            states.emplace_back(state);
        }
    }

    for (const auto& groupPathList : groupEventPathList)
    {
        for (const auto& pathList : groupPathList.second)
        {
            std::shared_ptr<std::set<std::string>> assert =
                std::make_shared<std::set<std::string>>();
            std::shared_ptr<bool> state = std::make_shared<bool>(false);

            const std::string& groupEventName = pathList.first;
            std::string eventPSUName = groupEventName + psuName;
            for (const auto& namedPath : pathList.second)
            {
                auto p = std::make_shared<PSUSubEvent>(
                    eventInterface, namedPath.second, namedPath.first, conn, io,
                    groupEventName, groupPathList.first, assert, combineEvent,
                    state, psuName, pollRate);
                p->setupRead();
                events[eventPSUName].emplace_back(p);

                asserts.emplace_back(assert);
                states.emplace_back(state);
            }
        }
    }
}

PSUCombineEvent::~PSUCombineEvent()
{
    // Clear unique_ptr first
    for (auto& event : events)
    {
        for (auto& subEventPtr : event.second)
        {
            subEventPtr.reset();
        }
    }
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
        {"ConfigureError",
         {"OpenBMC.0.1.PowerSupplyConfigurationError",
          "OpenBMC.0.1.PowerSupplyConfigurationErrorRecovered"}}};

PSUSubEvent::PSUSubEvent(
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
    const std::string& path, const std::string& objectPath,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& groupEventName,
    const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<std::set<std::string>> combineEvent,
    std::shared_ptr<bool> state, const std::string& psuName, double pollRate) :
    std::enable_shared_from_this<PSUSubEvent>(),
    eventInterface(std::move(eventInterface)), asserts(std::move(asserts)),
    combineEvent(std::move(combineEvent)), assertState(std::move(state)),
    errCount(0), path(path), objectPath(objectPath), eventName(eventName),
    waitTimer(io), inputDev(io), psuName(psuName),
    groupEventName(groupEventName), systemBus(conn)
{
    if (pollRate > 0.0)
    {
        eventPollMs = static_cast<unsigned int>(pollRate * 1000);
    }
    fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "PSU sub event failed to open file\n";
        return;
    }
    inputDev.assign(fd);

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
        auto fanNamePos = fanName.find('_');
        if (fanNamePos != std::string::npos)
        {
            fanName = fanName.substr(0, fanNamePos);
        }
    }
}

void PSUSubEvent::setupRead(void)
{
    std::shared_ptr<boost::asio::streambuf> buffer =
        std::make_shared<boost::asio::streambuf>();
    std::weak_ptr<PSUSubEvent> weakRef = weak_from_this();

    boost::asio::async_read_until(
        inputDev, *buffer, '\n',
        [weakRef, buffer](const boost::system::error_code& ec,
                          std::size_t /*bytes_transfered*/) {
            std::shared_ptr<PSUSubEvent> self = weakRef.lock();
            if (self)
            {
                self->readBuf = buffer;
                self->handleResponse(ec);
            }
        });
}

PSUSubEvent::~PSUSubEvent()
{
    waitTimer.cancel();
    inputDev.close();
}

void PSUSubEvent::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        return;
    }
    std::istream responseStream(readBuf.get());
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(responseStream, response);
            int nvalue = std::stoi(response);
            responseStream.clear();

            updateValue(nvalue);
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
    lseek(fd, 0, SEEK_SET);
    waitTimer.expires_from_now(boost::posix_time::milliseconds(eventPollMs));

    std::weak_ptr<PSUSubEvent> weakRef = weak_from_this();
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<PSUSubEvent> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        if (self)
        {
            self->setupRead();
        }
    });
}

inline std::optional<std::string>
    getEventTriggerFromPath(const std::string& path, bool asserted)
{
    std::string assertedStr = "AlarmDeasserted";
    if (asserted)
    {
        assertedStr = "AlarmAsserted";
    }

    if (boost::algorithm::ends_with(path, "_crit_alarm"))
    {
        return "WarningHigh" + assertedStr;
    }
    if (boost::algorithm::ends_with(path, "_lcrit_alarm"))
    {
        return "WarningLow" + assertedStr;
    }
    if (boost::algorithm::ends_with(path, "_max_alarm"))
    {
        return "CriticalHigh" + assertedStr;
    }
    if (boost::algorithm::ends_with(path, "_min_alarm"))
    {
        return "CriticalLow" + assertedStr;
    }

    return std::nullopt;
}

inline void sendPSUEventTrigger(
    const std::shared_ptr<sdbusplus::asio::connection>& systemBus,
    const std::string& path, const std::string& objectPath, bool asserted,
    double value)
{
    try
    {
        auto eventTrigger = getEventTriggerFromPath(path, asserted);
        if (!eventTrigger)
        {
            std::cerr << "PSUSubEvent: failed to get event triggered from "
                      << path << "\n";
            return;
        }
        auto signal = systemBus->new_signal(objectPath.c_str(),
                                            "xyz.openbmc_project.PSUSensor",
                                            eventTrigger->c_str());
        signal.append(value);
        signal.signal_send();
    }
    catch (const sdbusplus::exception::SdBusError& err)
    {
        std::cerr << "PSUSubEvent: failed to send sensor event trigger: "
                  << err.what() << "\n";
    }
}

// Any of the sub events of one event is asserted, then the event will be
// asserted. Only if none of the sub events are asserted, the event will be
// deasserted.
void PSUSubEvent::updateValue(const int& newValue)
{
    // Take no action if value already equal
    // Same semantics as Sensor::updateValue(const double&)
    if (newValue == value)
    {
        return;
    }

    if (newValue == 0)
    {
        // log deassert only after all asserts are gone
        if (!(*asserts).empty())
        {
            auto found = (*asserts).find(path);
            if (found == (*asserts).end())
            {
                return;
            }
            (*asserts).erase(path);

            return;
        }
        if (*assertState == true)
        {
            *assertState = false;
            auto foundCombine = (*combineEvent).find(groupEventName);
            if (foundCombine == (*combineEvent).end())
            {
                return;
            }
            (*combineEvent).erase(groupEventName);
            if (!deassertMessage.empty())
            {
                // Fan Failed has two args
                std::string sendMessage = eventName + " deassert";
                if (deassertMessage == "OpenBMC.0.1.PowerSupplyFanRecovered")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s,%s",
                        psuName.c_str(), fanName.c_str(), NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s",
                        psuName.c_str(), NULL);
                }

                sendPSUEventTrigger(systemBus, path, objectPath, false,
                                    newValue);
            }

            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", true);
            }
        }
    }
    else
    {
        std::cerr << "PSUSubEvent asserted by " << path << "\n";

        if ((*assertState == false) && ((*asserts).empty()))
        {
            *assertState = true;
            if (!assertMessage.empty())
            {
                // For failure and configure error, spec requires a beep
                if ((assertMessage == "OpenBMC.0.1.PowerSupplyFailed") ||
                    (assertMessage ==
                     "OpenBMC.0.1.PowerSupplyConfigurationError"))
                {
                    std::cout << " beep for " << assertMessage << "\n";
                    beep(beepPSUFailure);
                }

                // Fan Failed has two args
                std::string sendMessage = eventName + " assert";
                if (assertMessage == "OpenBMC.0.1.PowerSupplyFanFailed")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_WARNING, "REDFISH_MESSAGE_ID=%s",
                        assertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s,%s",
                        psuName.c_str(), fanName.c_str(), NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_WARNING, "REDFISH_MESSAGE_ID=%s",
                        assertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s",
                        psuName.c_str(), NULL);
                }

                sendPSUEventTrigger(systemBus, path, objectPath, true,
                                    newValue);
            }
            if ((*combineEvent).empty())
            {
                eventInterface->set_property("functional", false);
            }
            (*combineEvent).emplace(groupEventName);
        }
        (*asserts).emplace(path);
    }
    value = newValue;
}

void PSUSubEvent::beep(const uint8_t& beepPriority)
{
    systemBus->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "beep error (ec = " << ec << ")\n";
                return;
            }
        },
        "xyz.openbmc_project.BeepCode", "/xyz/openbmc_project/BeepCode",
        "xyz.openbmc_project.BeepCode", "Beep", uint8_t(beepPriority));
}
