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

#include <IpmbFWVersion.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <iostream>

constexpr const bool debug = false;

static constexpr uint8_t lun = 0;
static constexpr uint8_t ipmbLeftShift = 2;
static constexpr float convertToMs = 1000;

static constexpr const char* versionPathPrefix =
    "/xyz/openbmc_project/software/";
static constexpr const char* versionIface =
    "xyz.openbmc_project.Software.Version";

using IpmbMethodType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

IpmbFWVersion::IpmbFWVersion(std::shared_ptr<sdbusplus::asio::connection>& conn,
                             boost::asio::io_service& io, const float pollRate,
                             sdbusplus::asio::object_server& objectServer,
                             const uint8_t ipmbBusIndex,
                             const uint8_t deviceAddress,
                             const std::string& versionTypeName,
                             const std::string& name,
                             const std::string& versionClass) :
    versionPollMs(static_cast<int>(pollRate * convertToMs)),
    ipmbBusIndex(ipmbBusIndex), deviceAddress(deviceAddress),
    versionName(boost::replace_all_copy(name, " ", "_")),
    versionClass(versionClass), objectServer(objectServer),
    dbusConnection(conn), ioTimer(io)
{
    if (init())
    {
        std::string dbusPath =
            versionPathPrefix + versionTypeName + "/" + versionName;

        versionInterface = objectServer.add_interface(dbusPath, versionIface);

        versionInterface->register_property(
            "Version", std::string(""),
            sdbusplus::asio::PropertyPermission::readWrite);

        versionInterface->register_property(
            "Purpose",
            std::string(
                "xyz.openbmc_project.Software.Version.VersionPurpose.Other"));

        if (!versionInterface->initialize())
        {
            std::cerr << "Error initializing version interface\n";
            return;
        }
        read();
    }
}

IpmbFWVersion::~IpmbFWVersion()
{
    ioTimer.cancel();
    objectServer.remove_interface(versionInterface);
}

bool IpmbFWVersion::init()
{
    if (versionClass == "twin_lake_version")
    {
        type = IpmbVersionType::twinLake;
    }
    else
    {
        std::cerr << "Invalid version class : " << versionClass << "\n";
        return false;
    }

    bool flag = loadDefaults();
    return flag;
}

bool IpmbFWVersion::loadDefaults()
{
    if (type == IpmbVersionType::twinLake)
    {
        /* IPMB bus index - first 6 bits is device Index and last 2
        bits is IPMB/ME channel. Hence shifting the bus to left
        by 2 bits */
        commandAddress = ipmbBusIndex << ipmbLeftShift;
        netfn = oem::twinlake_fw_version::netFn;
        command = oem::twinlake_fw_version::command;
        commandData = {0x15, 0xa0, 0, deviceAddress};
    }
    else
    {
        std::cerr << " Invalid version type \n";
        return false;
    }
    return true;
}

void IpmbFWVersion::read()
{
    ioTimer.expires_from_now(boost::posix_time::milliseconds(versionPollMs));
    ioTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        dbusConnection->async_method_call(
            [&](boost::system::error_code ec, const IpmbMethodType& response) {
                const int status = std::get<0>(response);

                if (ec || status)
                {
                    versions->incrementError();
                    read();
                    std::cerr << " Error reading from IPMB Version\n";
                    return;
                }

                const std::vector<uint8_t> data = std::get<5>(response);

                if constexpr (debug)
                {
                    std::cout << versionName << " : ";
                    for (size_t d : data)
                    {
                        std::cout << d << " ";
                    }
                    std::cout << "\n";
                }

                if (data.empty())
                {
                    versions->incrementError();
                    read();
                    std::cerr << " IPMB Version data is empty \n";
                    return;
                }

                std::string version;
                int len = data.size();

                /* First 3 bytes of index is IANA, so it has been ignored */
                for (int index = 3; index < len; index++)
                {
                    /* Converting uint8_t data to string and appending the bytes
                       using dot. After last byte, dot should be ignored and
                       for ME version, byte 6 and byte 7 of data should be
                       combined, so there is no dot between them. */

                    version += std::to_string(data[index]);
                    if ((index != (len - 1)) && (index != 6))
                    {
                        version += ".";
                    }
                }

                versionInterface->set_property("Version", version);
                read();
            },
            "xyz.openbmc_project.Ipmi.Channel.Ipmb",
            "/xyz/openbmc_project/Ipmi/Channel/Ipmb", "org.openbmc.Ipmb",
            "sendRequest", commandAddress, netfn, lun, command, commandData);
    });
}
