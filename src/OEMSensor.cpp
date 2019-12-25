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



#include <unistd.h>

#include <OEMSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <boost/process.hpp>

static constexpr bool DEBUG = false;

static int executeCmd(std::string exec)
{
    const char *cmd = exec.c_str();
    boost::process::child execProg(cmd);
    execProg.wait();
    return execProg.exit_code();
}

OEMSensor::OEMSensor(boost::asio::io_service& io,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       OEMConfig& sensorconfig) :
    waitTimer(io), mDbusConn(conn), mSnrConfig(sensorconfig)
{
    setupRead();
}

OEMSensor::~OEMSensor()
{
    waitTimer.cancel();
}

void OEMSensor::setupRead(void)
{
    if (DEBUG)
    {
        std::cerr << "enter OEMSensor::setupRead" << "\n";
        std::cerr << "sensor name: " << mSnrConfig.name << "\n";
        std::cerr << "sensor num: " << static_cast<unsigned>(mSnrConfig.snrnum) << "\n";
        std::cerr << "sensor type: " << static_cast<unsigned>(mSnrConfig.snrtype) << "\n";
        std::cerr << "monitor: " << mSnrConfig.monitor << "\n";
        std::cerr << "exec: " << mSnrConfig.exec << "\n";
    }

    std::string monitor = mSnrConfig.monitor;
    std::string exec = mSnrConfig.exec;

    if (monitor == "oneshot")
    {
        // Execute Response
        auto retCode = executeCmd(exec);

        if (0 != retCode)
        {
            std::cerr << "oneshot sensor: " << mSnrConfig.name  << "\n";
            std::cerr << "exec " << exec << " failed !"  << "\n";
        }
    }

    handleResponse();
}

void OEMSensor::handleResponse()
{
    size_t pollTime = OEMSensor::sensorPollMs;
    waitTimer.expires_from_now(boost::posix_time::milliseconds(pollTime));
    waitTimer.async_wait([&](const boost::system::error_code& ec) {
        // case of timer expired
        if (!ec)
        {
            std::string monitor = mSnrConfig.monitor;
            std::string exec = mSnrConfig.exec;
            if (DEBUG)
            {
                std::cerr << "monitor: " << monitor << "\n";
                std::cerr << "exec: " << exec << "\n";
            }

            if (monitor != "oneshot")
            {
                // Execute Response
                auto retCode = executeCmd(exec);

                if (0 != retCode)
                {
                    std::cerr << "polling sensor: " << mSnrConfig.name  << "\n";
                    std::cerr << "exec " << exec << " failed !"  << "\n";
                }
            }

            // trigger next polling
            handleResponse();
        }
        // case of being canceled
        else if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "Timer of oem sensor is cancelled. Return \n";
            return;
        }
    });
}
