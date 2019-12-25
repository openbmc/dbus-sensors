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


#include <fcntl.h>

#include <OEMSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/process/child.hpp>
#include <filesystem>
#include <fstream>
#include <regex>
#include <boost/asio.hpp>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/sd_event.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/server.hpp>
#include <sdbusplus/timer.hpp>

static constexpr bool DEBUG = false;

boost::container::flat_map<std::string, std::unique_ptr<OEMSensor>> gOemSensors;
namespace fs = std::filesystem;

static constexpr const char* configPrefix =
    "xyz.openbmc_project.Configuration.";
static constexpr std::array<const char*, 1> sensorTypes = {"Oem"};
static constexpr const char* sensorType =
    "xyz.openbmc_project.Configuration.OEMSensor";

bool getOemConfig(
    const std::shared_ptr<sdbusplus::asio::connection>& systemBus,
    boost::container::flat_set<OEMConfig>& oemConfigs,
    ManagedObjectType& sensorConfigs,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<
        std::string, std::shared_ptr<sdbusplus::asio::dbus_interface>>&
        sensorIfaces)
{
    bool useCache = false;
    sensorConfigs.clear();

    // use new data the first time, then refresh
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(configPrefix + std::string(type), systemBus,
                                    sensorConfigs, useCache))
        {
            return false;
        }

        useCache = true;
    }

    for (const char* type : sensorTypes)
    {
        for (const std::pair<sdbusplus::message::object_path, SensorData>&
                 sensor : sensorConfigs)
        {
            for (const SensorBaseConfiguration& config : sensor.second)
            {
                if ((configPrefix + std::string(type)) != config.first)
                {
                    continue;
                }

                auto findName = config.second.find("Name");
                if (findName == config.second.end())
                {
                    continue;
                }
                std::string nameRaw =
                    std::visit(VariantToStringVisitor(), findName->second);
                std::string name =
                    std::regex_replace(nameRaw, illegalDbusRegex, "_");

                auto findSnrNum = config.second.find("SnrNum");
                if (findSnrNum == config.second.end())
                {
                    std::cerr << "Can't find 'SnrNum' setting in " << name << "\n";
                    continue;
                }
                uint64_t snrnum = std::visit(VariantToUnsignedIntVisitor(), 
				                             findSnrNum->second);

                auto findSnrType = config.second.find("SnrType");
                if (findSnrType == config.second.end())
                {
                    std::cerr << "Can't find 'SnrType' setting in " << name << "\n";
                    continue;
                }
                uint64_t snrtype = std::visit(VariantToUnsignedIntVisitor(),
                                              findSnrType->second);

                auto findMonitor = config.second.find("Monitor");
                std::string monitor = "NA";
                std::string exec = "NA";
                if (findMonitor == config.second.end())
                {
                    std::cerr << "Can't find 'Monitor' setting in " << name << "\n";
                    std::cerr << "Set 'Monitor' and 'Exec' as 'NA'" << "\n";
                }
                else
                {
                    monitor = std::visit(VariantToStringVisitor(), findMonitor->second);
                    if (monitor == "oneshot" || monitor == "polling")
                    {
                        auto findExec = config.second.find("Exec");
                        if (findExec == config.second.end())
                        {
                            std::cerr << "Can't find 'Exec' setting in " << name << "\n";
                            continue;
                        }
                        exec = std::visit(VariantToStringVisitor(), findExec->second);
                    }
                    else
                    {
                        std::cerr << "Incorrect 'Monitor' setting in " << name << "\n";
                        continue;
                    }
                }

                std::vector<OEMInfo> oeminfoVector;
                boost::container::flat_map<std::string, std::vector<OEMInfo>> ifaceList;
                std::string OemIface = "";
                std::string OemProperty = "";
                std::string OemPtype = "";
                std::string OemDfvalue = "";
                for (const SensorBaseConfiguration& suppConfig : sensor.second)
                {
                    if (suppConfig.first.find("Offset") !=
                        std::string::npos)
                    {
                        auto oemSetting = suppConfig.second;

                        auto findOemIface = oemSetting.find("Iface");
                        auto findOemProperty = oemSetting.find("Property");
                        auto findOemPtype = oemSetting.find("Ptype");
                        auto findOemDfvalue = oemSetting.find("Dfvalue");

                        if (findOemIface == oemSetting.end() ||
                            findOemProperty == oemSetting.end() ||
                            findOemPtype == oemSetting.end() ||
                            findOemDfvalue == oemSetting.end())
                        {
                            std::cerr << "Incorrect OEM configuration setting\n";
                            break;
                        }

                        OemIface = std::visit(VariantToStringVisitor(), findOemIface->second);
                        OemPtype = std::visit(VariantToStringVisitor(), findOemPtype->second);
                        OemProperty = std::visit(VariantToStringVisitor(), findOemProperty->second);
                        OemDfvalue = std::visit(VariantToStringVisitor(), findOemDfvalue->second);

                        if (DEBUG)
                        {
                            std::cerr << "snrnum: " << static_cast<unsigned>(snrnum) << "\n";
                            std::cerr << "snrtype: " << static_cast<unsigned>(snrtype) << "\n";
                            std::cerr << "name: " << name << "\n";
                            std::cerr << "iface: " << OemIface << "\n";
                            std::cerr << "property: " << OemProperty << "\n";
                            std::cerr << "ptype: " << OemPtype << "\n";
                            std::cerr << "dfvalue: " << OemDfvalue << "\n";
                            std::cerr << "monitor: " << monitor << "\n";
                            std::cerr << "exec: " << exec << "\n";
                            std::cerr << "type: " << type << "\n";
                        }

                        oeminfoVector.emplace_back(OemIface, OemProperty, OemPtype, OemDfvalue);
                        ifaceList[OemIface].emplace_back(OemIface, OemProperty, OemPtype, OemDfvalue);
                    }
                }

                auto ifacename = name + "_" + OemIface;
                if (sensorIfaces.find(ifacename) == sensorIfaces.end())
                {
                    for (auto& oemEvt : ifaceList)
                    {
                        auto iface = objectServer.add_interface(
                                    "/xyz/openbmc_project/OEMSensor" + std::string("/") + name,
                                    oemEvt.first);
                        for (auto& oemsetting : oemEvt.second)
                        {
                            const auto property = oemsetting.property;
                            if (oemsetting.ptype == "bool")
                            {
                                bool value = false;
                                auto dfvalue = oemsetting.dfvalue; 
                            
                                if (dfvalue == "1")							
                                {
                                    value = true;
                                }
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            } 
                            else if (oemsetting.ptype == "string")
                            {
                                auto value = oemsetting.dfvalue;
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            }
/*
                            else if (oemsetting.ptype == "uint8_t")
                            {
                                uint8_t value = (uint8_t) oemsetting.dfvalue;
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            }
                            else if (oemsetting.ptype == "uint16_t")
                            {
                                uint16_t value = (uint16_t) oemsetting.dfvalue;
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            }
                            else if (oemsetting.ptype == "uint32_t")
                            {
                                uint32_t value = (uint32_t) oemsetting.dfvalue;
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            }
                            else if (oemsetting.ptype == "uint64_t")
                            {
                                uint64_t value = (uint64_t) oemsetting.dfvalue;
                                iface->register_property(property, value,
                                                         sdbusplus::asio::PropertyPermission::readWrite);
                            }
*/
                        }
                        iface->initialize();
                        sensorIfaces[ifacename] = std::move(iface);
                    }
                }

                oemConfigs.emplace(snrnum, snrtype, name, monitor, exec, oeminfoVector);
            }
        }
    }

    if (oemConfigs.size())
    {
        std::cerr << "OEM config" << (oemConfigs.size() == 1 ? " is" : "s are")
                  << " parsed\n";
        std::cerr << "OEM config size is " << oemConfigs.size() << "\n";

        return true;
    }

    std::cerr << "oemConfigs is NULL\n";
    return false;
}

void detectOEMSensor(boost::asio::io_service& io,
                     std::shared_ptr<sdbusplus::asio::connection>& systemBus,
                     boost::container::flat_set<OEMConfig>& oemConfigs)
{
    for (auto& oemsensor : oemConfigs)
    {
        if (oemsensor.monitor != "NA")
        {
            gOemSensors[oemsensor.name] = std::make_unique<OEMSensor>(io, systemBus, oemsensor);
        }
    }

    if (gOemSensors.size())
    {
        std::cerr << "Total number of OEM sensors for monitoring is " << oemConfigs.size() << "\n";
    }
    else
    {
        std::cerr << "No OEM sensors need to be monitored" << "\n";
    }
}

int main()
{
    boost::asio::io_service io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    boost::container::flat_set<OEMConfig> oemConfigs;
    systemBus->request_name("xyz.openbmc_project.OEMSensor");
    sdbusplus::asio::object_server objectServer(systemBus);
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    boost::asio::deadline_timer pingTimer(io);
    boost::asio::deadline_timer creationTimer(io);
    boost::asio::deadline_timer filterTimer(io);
    ManagedObjectType sensorConfigs;
    boost::container::flat_map<std::string,
                               std::shared_ptr<sdbusplus::asio::dbus_interface>>
        sensorIfaces;

    if (getOemConfig(systemBus, oemConfigs, sensorConfigs, objectServer, sensorIfaces))
    {
        detectOEMSensor(io, systemBus, oemConfigs);
    }
    // callback to handle configuration change
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            std::cerr << "rescan due to configuration change \n";
            if (getOemConfig(systemBus, oemConfigs, sensorConfigs, objectServer, sensorIfaces))
            {
                detectOEMSensor(io, systemBus, oemConfigs);
            }
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" + sensorType + "'",
          eventHandler);
    io.run();
    return 0;
}

