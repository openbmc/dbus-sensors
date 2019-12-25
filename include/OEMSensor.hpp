/*
// Copyright (c) 2019-present Lenovo
// 
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation 
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors 
//    may be used to endorse or promote products derived from this software without 
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "Utils.hpp"

#include <boost/container/flat_map.hpp>
#include <filesystem>
#include <fstream>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

struct OEMInfo
{
    OEMInfo(const std::string& iface, const std::string& property,
            const std::string& ptype, const std::string& dfvalue) :
        iface(iface), property(property), ptype(ptype), dfvalue(dfvalue) 
    {
    }
    std::string iface;
    std::string property;
    std::string ptype;
    std::string dfvalue;

    bool operator<(const OEMInfo& rhs) const
    {
        return (iface < rhs.iface);
    }
};

struct OEMConfig
{
    OEMConfig(const uint8_t& snrnum, const uint8_t& snrtype, const std::string& name, 
              const std::string& monitor, const std::string& exec, 
              const std::vector<OEMInfo>& oeminfo) :
        snrnum(snrnum), snrtype(snrtype), name(name), monitor(monitor), exec(exec), oeminfo(std::move(oeminfo))
    {
    }
    uint8_t snrnum;
    uint8_t snrtype;
    std::string name;
    std::string monitor;
    std::string exec;
    std::vector<OEMInfo> oeminfo;

    bool operator<(const OEMConfig& rhs) const
    {
        return (name < rhs.name);
    }
};

class OEMSensor
{
  public:
    OEMSensor(boost::asio::io_service& io,
               std::shared_ptr<sdbusplus::asio::connection>& conn,
               OEMConfig & sensorconfig);
    ~OEMSensor();

    static constexpr unsigned int sensorPollMs = 2000;

  private:
    boost::asio::deadline_timer waitTimer;
    std::shared_ptr<sdbusplus::asio::connection> mDbusConn;
    OEMConfig mSnrConfig;

    void setupRead(void);
    void handleResponse(void);
};

extern boost::container::flat_map<std::string, std::unique_ptr<OEMSensor>>
    gOemSensors;
