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

#include <sdbusplus/asio/object_server.hpp>

class PSUSubEvent
{
  public:
    PSUSubEvent(std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface,
                const std::string& path, boost::asio::io_service& io,
                const std::string& eventName,
                std::shared_ptr<std::set<std::string>> asserts,
                std::shared_ptr<std::set<std::string>> combineEvent,
                std::shared_ptr<bool> state, const std::string& psuName);
    ~PSUSubEvent();

    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface;
    std::shared_ptr<std::set<std::string>> asserts;
    std::shared_ptr<std::set<std::string>> combineEvent;
    std::shared_ptr<bool> assertState;

  private:
    int value = 0;
    int errCount;
    std::string path;
    std::string eventName;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void updateValue(const int& newValue);
    boost::asio::posix::stream_descriptor inputDev;
    static constexpr unsigned int eventPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
    std::string psuName;
    std::string fanName;
    std::string assertMessage;
    std::string deassertMessage;
};

class PSUCombineEvent
{
  public:
    PSUCombineEvent(
        sdbusplus::asio::object_server& objectSever,
        boost::asio::io_service& io, const std::string& psuName,
        boost::container::flat_map<std::string, std::vector<std::string>>&
            eventPathList,
        const std::string& combineEventName);
    ~PSUCombineEvent();

    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> eventInterface;
    boost::container::flat_map<std::string,
                               std::vector<std::unique_ptr<PSUSubEvent>>>
        events;
    std::vector<std::shared_ptr<std::set<std::string>>> asserts;
    std::vector<std::shared_ptr<bool>> states;
};
