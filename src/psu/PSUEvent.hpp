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

#pragma once

#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/random_access_file.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <set>
#include <string>
#include <vector>

using EventPathList =
    boost::container::flat_map<std::string, std::vector<std::string>>;
using GroupEventPathList =
    boost::container::flat_map<std::string, EventPathList>;

class PSUSubEvent : public std::enable_shared_from_this<PSUSubEvent>
{
  public:
    PSUSubEvent(std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
                const std::string& path,
                std::shared_ptr<sdbusplus::asio::connection>& conn,
                boost::asio::io_context& io, const PowerState& powerState,
                const std::string& groupEventName, const std::string& eventName,
                std::shared_ptr<std::set<std::string>> asserts,
                std::shared_ptr<std::set<std::string>> combineEvent,
                std::shared_ptr<bool> state, const std::string& psuName,
                double pollRate, bool& skipReading);
    ~PSUSubEvent();

    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface;
    std::shared_ptr<std::set<std::string>> asserts;
    std::shared_ptr<std::set<std::string>> combineEvent;
    std::shared_ptr<bool> assertState;
    void setupRead();

  private:
    int value = 0;
    size_t errCount{0};
    std::string path;
    std::string eventName;

    PowerState readState;
    std::shared_ptr<std::array<char, 128>> buffer;
    void restartRead();
    void handleResponse(const boost::system::error_code& err,
                        size_t bytesTransferred);
    void updateValue(const int& newValue);
    boost::asio::random_access_file inputDev;
    boost::asio::steady_timer waitTimer;
    std::string psuName;
    std::string groupEventName;
    std::string fanName;
    std::string assertMessage;
    std::string deassertMessage;
    std::shared_ptr<sdbusplus::asio::connection> systemBus;
    unsigned int eventPollMs = defaultEventPollMs;
    bool& skipReading;
    static constexpr unsigned int defaultEventPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
};

class PSUCombineEvent
{
  public:
    PSUCombineEvent(sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_context& io, const std::string& psuName,
                    const PowerState& powerState, EventPathList& eventPathList,
                    GroupEventPathList& groupEventPathList,
                    const std::string& combineEventName, double pollRate,
                    bool skipRead);
    ~PSUCombineEvent();
    void setSkipRead(bool skip);

    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface;
    boost::container::flat_map<std::string,
                               std::vector<std::shared_ptr<PSUSubEvent>>>
        events;
    std::vector<std::shared_ptr<std::set<std::string>>> asserts;
    std::vector<std::shared_ptr<bool>> states;

  private:
    bool skipReading;
};
