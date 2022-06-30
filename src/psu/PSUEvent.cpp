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

#include "SensorPaths.hpp"
#include "Utils.hpp"

#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

PSUCombineEvent::PSUCombineEvent(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& psuName,
    const PowerState& powerState, EventPathList& eventPathList,
    GroupEventPathList& groupEventPathList, const std::string& combineEventName,
    double pollRate) : objServer(objectServer)
{
    std::string psuNameEscaped = sensor_paths::escapePathForDbus(psuName);
    eventInterface = objServer.add_interface(
        "/xyz/openbmc_project/State/Decorator/" + psuNameEscaped + "_" +
            combineEventName,
        "xyz.openbmc_project.State.Decorator.OperationalStatus");
    eventInterface->register_property("functional", true);

    if (!eventInterface->initialize())
    {
        lg2::error("error initializing event interface");
    }

    std::shared_ptr<std::set<std::string>> combineEvent =
        std::make_shared<std::set<std::string>>();
    for (const auto& [eventName, paths] : eventPathList)
    {
        std::shared_ptr<std::set<std::string>> assert =
            std::make_shared<std::set<std::string>>();
        std::shared_ptr<bool> state = std::make_shared<bool>(false);

        std::string eventPSUName = eventName + psuName;
        for (const auto& path : paths)
        {
            auto p = std::make_shared<PSUSubEvent>(
                eventInterface, path, conn, io, powerState, eventName,
                eventName, assert, combineEvent, state, psuName, pollRate);
            p->setupRead();

            events[eventPSUName].emplace_back(p);
            asserts.emplace_back(assert);
            states.emplace_back(state);
        }
    }

    for (const auto& [eventName, groupEvents] : groupEventPathList)
    {
        for (const auto& [groupEventName, paths] : groupEvents)
        {
            std::shared_ptr<std::set<std::string>> assert =
                std::make_shared<std::set<std::string>>();
            std::shared_ptr<bool> state = std::make_shared<bool>(false);

            std::string eventPSUName = groupEventName + psuName;
            for (const auto& path : paths)
            {
                auto p = std::make_shared<PSUSubEvent>(
                    eventInterface, path, conn, io, powerState, groupEventName,
                    eventName, assert, combineEvent, state, psuName, pollRate);
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
    for (auto& [psuName, subEvents] : events)
    {
        for (auto& subEventPtr : subEvents)
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
    const std::string& path, std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const PowerState& powerState,
    const std::string& groupEventName, const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<std::set<std::string>> combineEvent,
    std::shared_ptr<bool> state, const std::string& psuName, double pollRate) :
    eventInterface(std::move(eventInterface)), asserts(std::move(asserts)),
    combineEvent(std::move(combineEvent)), assertState(std::move(state)),
    path(path), eventName(eventName), readState(powerState),
    inputDev(io, path, boost::asio::random_access_file::read_only),
    waitTimer(io), psuName(psuName), groupEventName(groupEventName),
    systemBus(conn)
{
    if (pollRate > 0.0)
    {
        eventPollMs = static_cast<unsigned int>(pollRate * 1000);
    }

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

PSUSubEvent::~PSUSubEvent()
{
    waitTimer.cancel();
    inputDev.close();
}

void PSUSubEvent::setupRead()
{
    if (!readingStateGood(readState))
    {
        // Deassert the event
        updateValue(0);
        restartRead();
        return;
    }

    inputDev.async_read_some_at(
        0, boost::asio::buffer(buffer),
        std::bind_front(&PSUSubEvent::handleResponseStatic, weak_from_this()));
}

void PSUSubEvent::handleResponse(const boost::system::error_code& err,
                                 size_t bytesTransferred)
{
    if (err == boost::asio::error::operation_aborted)
    {
        return;
    }

    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        return;
    }

    if (err)
    {
        errCount++;
    }
    else
    {
        const char* bufferEnd = buffer.data() + bytesTransferred;
        int nvalue = 0;
        std::from_chars_result ret =
            std::from_chars(buffer.data(), bufferEnd, nvalue);
        if (ret.ec != std::errc())
        {
            errCount++;
        }
        else
        {
            updateValue(nvalue);
            errCount = 0;
        }
    }
    if (errCount >= warnAfterErrorCount)
    {
        if (errCount == warnAfterErrorCount)
        {
            lg2::error("Failure to read event at '{PATH}'", "PATH", path);
        }
        updateValue(0);
        errCount++;
    }
    restartRead();
}

void PSUSubEvent::handleResponseStatic(
    const std::weak_ptr<PSUSubEvent>& weakRef,
    const boost::system::error_code& ec, std::size_t bytesTransferred)
{
    std::shared_ptr<PSUSubEvent> self = weakRef.lock();
    if (self)
    {
        self->handleResponse(ec, bytesTransferred);
    }
}

void PSUSubEvent::handleTimeout(const std::weak_ptr<PSUSubEvent>& weakRef,
                                const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        return;
    }
    std::shared_ptr<PSUSubEvent> self = weakRef.lock();
    if (self)
    {
        self->setupRead();
    }
}

void PSUSubEvent::restartRead()
{
    waitTimer.expires_after(std::chrono::milliseconds(eventPollMs));
    waitTimer.async_wait(
        std::bind_front(&PSUSubEvent::handleTimeout, weak_from_this()));
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
        if (*assertState)
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
                if (deassertMessage == "OpenBMC.0.1.PowerSupplyFanRecovered")
                {
                    lg2::info("'{EVENT}' deassert", "EVENT", eventName,
                              "REDFISH_MESSAGE_ID", deassertMessage,
                              "REDFISH_MESSAGE_ARGS",
                              (psuName + ',' + fanName));
                }
                else
                {
                    lg2::info("'{EVENT}' deassert", "EVENT", eventName,
                              "REDFISH_MESSAGE_ID", deassertMessage,
                              "REDFISH_MESSAGE_ARGS", psuName);
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
        lg2::error("PSUSubEvent asserted by '{PATH}'", "PATH", path);

        if ((!*assertState) && ((*asserts).empty()))
        {
            *assertState = true;
            if (!assertMessage.empty())
            {
                // Fan Failed has two args
                if (assertMessage == "OpenBMC.0.1.PowerSupplyFanFailed")
                {
                    lg2::warning("'{EVENT}' assert", "EVENT", eventName,
                                 "REDFISH_MESSAGE_ID", assertMessage,
                                 "REDFISH_MESSAGE_ARGS",
                                 (psuName + ',' + fanName));
                }
                else
                {
                    lg2::warning("'{EVENT}' assert", "EVENT", eventName,
                                 "REDFISH_MESSAGE_ID", assertMessage,
                                 "REDFISH_MESSAGE_ARGS", psuName);
                }
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
