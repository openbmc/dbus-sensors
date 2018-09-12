/*
// Copyright (c) 2018 Intel Corporation
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

#include <TachSensor.hpp>
#include <Utils.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <limits>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string>

static constexpr unsigned int PWM_POLL_MS = 500;
static constexpr size_t WARN_AFTER_ERROR_COUNT = 10;

TachSensor::TachSensor(const std::string &path,
                       sdbusplus::asio::object_server &objectServer,
                       std::shared_ptr<sdbusplus::asio::connection> &conn,
                       boost::asio::io_service &io, const std::string &fanName,
                       std::vector<thresholds::Threshold> &&_thresholds,
                       const std::string &sensorConfiguration) :
    Sensor(),
    path(path), objServer(objectServer), dbusConnection(conn),
    name(boost::replace_all_copy(fanName, " ", "_")),
    configuration(sensorConfiguration),
    input_dev(io, open(path.c_str(), O_RDONLY)), wait_timer(io), err_count(0),
    // todo, get these from config
    max_value(25000), min_value(0)
{
    thresholds = std::move(_thresholds);
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/fan_tach/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::HasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::HasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/fan_tach/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    set_initial_properties(conn);
    isPowerOn(dbusConnection); // first call initializes
    setup_read();
}

TachSensor::~TachSensor()
{
    // close the input dev to cancel async operations
    input_dev.close();
    wait_timer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
}

void TachSensor::setup_read(void)
{
    boost::asio::async_read_until(
        input_dev, read_buf, '\n',
        [&](const boost::system::error_code &ec,
            std::size_t /*bytes_transfered*/) { handle_response(ec); });
}

void TachSensor::handle_response(const boost::system::error_code &err)
{
    if (err == boost::system::errc::bad_file_descriptor)
    {
        return; // we're being destroyed
    }
    std::istream response_stream(&read_buf);
    if (!err)
    {
        std::string response;
        try
        {
            std::getline(response_stream, response);
            float nvalue = std::stof(response);
            response_stream.clear();
            if (nvalue != value)
            {
                update_value(nvalue);
            }
            err_count = 0;
        }
        catch (const std::invalid_argument &)
        {
            err_count++;
        }
    }
    else
    {

        err_count++;
    }
    // only send value update once
    if (err_count == WARN_AFTER_ERROR_COUNT)
    {
        // only an error if power is on
        if (isPowerOn(dbusConnection))
        {
            std::cerr << "Failure to read sensor " << name << " at " << path
                      << "\n";
            update_value(0);
        }
        else
        {
            err_count = 0; // check power again in 10 cycles
            sensorInterface->set_property(
                "Value", std::numeric_limits<double>::quiet_NaN());
        }
    }
    response_stream.clear();
    input_dev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd <= 0)
    {
        return; // we're no longer valid
    }
    input_dev.assign(fd);
    wait_timer.expires_from_now(boost::posix_time::milliseconds(PWM_POLL_MS));
    wait_timer.async_wait([&](const boost::system::error_code &ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        setup_read();
    });
}

void TachSensor::check_thresholds(void)
{
    thresholds::checkThresholds(this);
}

void TachSensor::update_value(const double &new_value)
{
    sensorInterface->set_property("Value", new_value);
    value = new_value;
    check_thresholds();
}

void TachSensor::set_initial_properties(
    std::shared_ptr<sdbusplus::asio::connection> &conn)
{
    // todo, get max and min from configuration
    sensorInterface->register_property("MaxValue", max_value);
    sensorInterface->register_property("MinValue", min_value);
    sensorInterface->register_property("Value", value);

    for (auto &threshold : thresholds)
    {
        std::shared_ptr<sdbusplus::asio::dbus_interface> iface;
        std::string level;
        std::string alarm;
        if (threshold.level == thresholds::Level::CRITICAL)
        {
            iface = thresholdInterfaceCritical;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "CriticalHigh";
                alarm = "CriticalAlarmHigh";
            }
            else
            {
                level = "CriticalLow";
                alarm = "CriticalAlarmLow";
            }
        }
        else if (threshold.level == thresholds::Level::WARNING)
        {
            iface = thresholdInterfaceWarning;
            if (threshold.direction == thresholds::Direction::HIGH)
            {
                level = "WarningHigh";
                alarm = "WarningAlarmHigh";
            }
            else
            {
                level = "WarningLow";
                alarm = "WarningAlarmLow";
            }
        }
        else
        {
            std::cerr << "Unknown threshold level" << threshold.level << "\n";
            continue;
        }
        if (!iface)
        {
            std::cout << "trying to set uninitialized interface\n";
            continue;
        }
        iface->register_property(
            level, threshold.value,
            [&](const double &request, double &oldValue) {
                oldValue = request; // todo, just let the config do this?
                threshold.value = request;
                thresholds::persistThreshold(
                    configuration,
                    "xyz.openbmc_project.Configuration.AspeedFan", threshold,
                    conn);
                return 1;
            });
        iface->register_property(alarm, false);
    }
    if (!sensorInterface->initialize())
    {
        std::cerr << "error initializing value interface\n";
    }
    if (thresholdInterfaceWarning && !thresholdInterfaceWarning->initialize())
    {
        std::cerr << "error initializing warning threshold interface\n";
    }

    if (thresholdInterfaceCritical && !thresholdInterfaceCritical->initialize())
    {
        std::cerr << "error initializing critical threshold interface\n";
    }
}
