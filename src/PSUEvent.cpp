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
        std::shared_ptr<std::set<std::string>> assert =
            std::make_shared<std::set<std::string>>();
        std::shared_ptr<bool> state = std::make_shared<bool>(false);

        const std::string& eventName = pathList.first;
        std::string eventPSUName = eventName + psuName;
        for (const auto& namedPath : pathList.second)
        {
            auto p = std::make_shared<PSUSubEvent>(
                eventInterface, namedPath.second, namedPath.first, conn, io,
                eventName, eventName, assert, combineEvent, state, psuName);
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
                    state, psuName);
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
    const std::string& path, const std::string& dbusPath,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& groupEventName,
    const std::string& eventName,
    std::shared_ptr<std::set<std::string>> asserts,
    std::shared_ptr<std::set<std::string>> combineEvent,
    std::shared_ptr<bool> state, const std::string& psuName) :
    std::enable_shared_from_this<PSUSubEvent>(),
    eventInterface(eventInterface), asserts(asserts),
    combineEvent(combineEvent), assertState(state), errCount(0), path(path),
    eventName(eventName), waitTimer(io), inputDev(io), psuName(psuName),
    groupEventName(groupEventName), systemBus(conn), dbusPath(dbusPath)
{
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
        auto fanNamePos = fanName.find("_");
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

namespace
{

static constexpr const char* selMessageId = "b370836ccf2f4850ac5bee185b77893a";
static constexpr uint16_t selInvalidRecID = static_cast<uint16_t>(65535U);
static constexpr uint8_t selSystemType = 0x02;

static unsigned int getLatestRecordId()
{
    // Open the journal
    sd_journal* journalTmp = nullptr;
    if (sd_journal_open(&journalTmp, SD_JOURNAL_LOCAL_ONLY) < 0)
    {
        return selInvalidRecID;
    }
    std::unique_ptr<sd_journal, decltype(&sd_journal_close)> journal(
        journalTmp, sd_journal_close);
    journalTmp = nullptr;

    // Filter the journal based on the SEL MESSAGE_ID
    std::string match = "MESSAGE_ID=" + std::string(selMessageId);
    sd_journal_add_match(journal.get(), match.c_str(), 0);

    // Find the newest SEL entry's ID
    if (sd_journal_seek_tail(journal.get()) < 0)
    {
        return selInvalidRecID;
    }
    if (sd_journal_previous(journal.get()) > 0)
    {
        return selInvalidRecID;
    }

    // Read the field data
    const char* data = nullptr;
    size_t length = 0;
    if (sd_journal_get_data(journal.get(), "IPMI_SEL_RECORD_ID",
                            static_cast<const void**>(&data), &length) < 0)
    {
        return selInvalidRecID;
    }

    // Extract the record_id value
    std::string_view record_id_field(data, length);
    // Only use the content after the "=" character.
    record_id_field.remove_prefix(
        std::min(record_id_field.find("=") + 1, record_id_field.size()));
    unsigned long record_id;
    try
    {
        record_id = std::stoul(std::string(record_id_field), nullptr, 10);
    }
    catch (std::invalid_argument& e)
    {
        return selInvalidRecID;
    }
    catch (std::out_of_range& e)
    {
        return selInvalidRecID;
    }

    // Range check
    if (record_id < 1 || record_id >= selInvalidRecID)
    {
        return selInvalidRecID;
    }

    return static_cast<unsigned int>(record_id);
}

static unsigned int getNewRecordId(void)
{
    static unsigned int recordId = getLatestRecordId();
    if (++recordId >= selInvalidRecID)
    {
        recordId = 1;
    }
    return recordId;
}

static constexpr char IPMI_EVENT_TRIGGER_LOWER_WARN = '0';
static constexpr char IPMI_EVENT_TRIGGER_LOWER_CRIT = '2';
static constexpr char IPMI_EVENT_TRIGGER_UPPER_WARN = '7';
static constexpr char IPMI_EVENT_TRIGGER_UPPER_CRIT = '9';
static constexpr char IPMI_EVENT_TRIGGER_UNDEFINED = 'F';

static char getEventTriggerFromPath(const std::string& path)
{
    char event_code = IPMI_EVENT_TRIGGER_UNDEFINED;

    if (boost::algorithm::ends_with(path, "_crit_alarm"))
    {
        event_code = IPMI_EVENT_TRIGGER_UPPER_CRIT;
    }
    else if (boost::algorithm::ends_with(path, "_lcrit_alarm"))
    {
        event_code = IPMI_EVENT_TRIGGER_LOWER_CRIT;
    }
    else if (boost::algorithm::ends_with(path, "_max_alarm"))
    {
        event_code = IPMI_EVENT_TRIGGER_UPPER_WARN;
    }
    else if (boost::algorithm::ends_with(path, "_min_alarm"))
    {
        event_code = IPMI_EVENT_TRIGGER_LOWER_WARN;
    }

    return event_code;
}

} // namespace

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
                unsigned int recordId = getNewRecordId();
                const uint16_t genId = 0x00ff;

                if (deassertMessage == "OpenBMC.0.1.PowerSupplyFanRecovered")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s,%s",
                        psuName.c_str(), fanName.c_str(), "MESSAGE_ID=%s",
                        selMessageId, "IPMI_SEL_RECORD_ID=%d", recordId,
                        "IPMI_SEL_RECORD_TYPE=%x", selSystemType,
                        "IPMI_SEL_GENERATOR_ID=%x", genId,
                        "IPMI_SEL_SENSOR_PATH=%s", dbusPath.c_str(),
                        "IPMI_SEL_EVENT_DIR=%x", *assertState,
                        "IPMI_SEL_DATA=0%cFFFF", getEventTriggerFromPath(path),
                        NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        deassertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s",
                        psuName.c_str(), "MESSAGE_ID=%s", selMessageId,
                        "IPMI_SEL_RECORD_ID=%d", recordId,
                        "IPMI_SEL_RECORD_TYPE=%x", selSystemType,
                        "IPMI_SEL_GENERATOR_ID=%x", genId,
                        "IPMI_SEL_SENSOR_PATH=%s", dbusPath.c_str(),
                        "IPMI_SEL_EVENT_DIR=%x", *assertState,
                        "IPMI_SEL_DATA=0%cFFFF", getEventTriggerFromPath(path),
                        NULL);
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
                unsigned int recordId = getNewRecordId();
                const uint16_t genId = 0x00ff;

                if (assertMessage == "OpenBMC.0.1.PowerSupplyFanFailed")
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_WARNING, "REDFISH_MESSAGE_ID=%s",
                        assertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s,%s",
                        psuName.c_str(), fanName.c_str(), "MESSAGE_ID=%s",
                        selMessageId, "IPMI_SEL_RECORD_ID=%d", recordId,
                        "IPMI_SEL_RECORD_TYPE=%x", selSystemType,
                        "IPMI_SEL_GENERATOR_ID=%x", genId,
                        "IPMI_SEL_SENSOR_PATH=%s", dbusPath.c_str(),
                        "IPMI_SEL_EVENT_DIR=%x", *assertState,
                        "IPMI_SEL_DATA=0%cFFFF", getEventTriggerFromPath(path),
                        NULL);
                }
                else
                {
                    sd_journal_send(
                        "MESSAGE=%s", sendMessage.c_str(), "PRIORITY=%i",
                        LOG_WARNING, "REDFISH_MESSAGE_ID=%s",
                        assertMessage.c_str(), "REDFISH_MESSAGE_ARGS=%s",
                        psuName.c_str(), "MESSAGE_ID=%s", selMessageId,
                        "IPMI_SEL_RECORD_ID=%d", recordId,
                        "IPMI_SEL_RECORD_TYPE=%x", selSystemType,
                        "IPMI_SEL_GENERATOR_ID=%x", genId,
                        "IPMI_SEL_SENSOR_PATH=%s", dbusPath.c_str(),
                        "IPMI_SEL_EVENT_DIR=%x", *assertState,
                        "IPMI_SEL_DATA=0%cFFFF", getEventTriggerFromPath(path),
                        NULL);
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
