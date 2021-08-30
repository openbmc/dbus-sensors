/*
// Copyright (c) 2021 Arm Ltd.
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
#include <PLDMSensor.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

extern "C"
{
#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/pldm.h>
}

constexpr const bool debug = false;

constexpr const char* configInterface =
    "xyz.openbmc_project.Configuration.PLDM";

PLDMSensor::PLDMSensor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                       boost::asio::io_service& io,
                       const std::string& sensorName,
                       const std::string& configPath,
                       const std::string& configInf,
                       sdbusplus::asio::object_server& objectServer,
                       std::vector<thresholds::Threshold>&& thresholdData,
                       const std::string& sensorUnit, uint8_t eid,
                       uint16_t sensorId, uint8_t instanceId,
                       const float pollRate, float resolution, float offset,
                       double maxValue, double minValue) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdData), configPath, configInf, false, maxValue,
           minValue, conn),
    eid(eid), sensorId(sensorId), instanceId(instanceId),
    sensorPollMs(static_cast<int>(pollRate * 1000)), resolution(resolution),
    offset(offset), objectServer(objectServer), waitTimer(io)
{
    std::string dbusPath = sensor_paths::getPathForUnits(sensorUnit);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }

    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += dbusPath;
    objectPath += "/";
    objectPath += name;

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            objectPath, "xyz.openbmc_project.Sensor.Threshold.Warning");
    }

    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            objectPath, "xyz.openbmc_project.Sensor.Threshold.Critical");
    }

    association =
        objectServer.add_interface(objectPath, association::interface);
    setInitialProperties(dbusConnection, sensorUnit);
}

PLDMSensor::~PLDMSensor()
{
    waitTimer.cancel();
    objectServer.remove_interface(sensorInterface);
    objectServer.remove_interface(association);
}

void PLDMSensor::init(void)
{
    read();
}

void PLDMSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

static int pldmSendRecv(uint8_t eid, std::vector<uint8_t>& requestMsg,
                        std::vector<uint8_t>& responseMsg)
{
    int fd = pldm_open();
    if (-1 == fd)
    {
        std::cerr << "failed to init mctp" << std::endl;
        return -1;
    }

    uint8_t* pldmRespMsg = nullptr;
    size_t respMsgLen{};
    auto rc = pldm_send_recv(eid, fd, requestMsg.data(), requestMsg.size(),
                             &pldmRespMsg, &respMsgLen);
    if (rc == PLDM_REQUESTER_SUCCESS)
    {
        responseMsg.resize(respMsgLen);
        memcpy(responseMsg.data(), pldmRespMsg, respMsgLen);
        free(pldmRespMsg);
    }
    close(fd);
    return rc;
}

bool getSensorReading(uint8_t eid, uint8_t instanceId, uint8_t sensorId,
                      int32_t* reading, uint8_t* dataSize)
{
    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                                    PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES);
    std::vector<uint8_t> responseMsg;

    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc =
        encode_get_sensor_reading_req(instanceId, sensorId, 0xff, request);
    if (rc != 0)
    {
        return false;
    }

    rc = pldmSendRecv(eid, requestMsg, responseMsg);
    if (rc != 0)
    {
        return false;
    }

    *dataSize = PLDM_SENSOR_DATA_SIZE_SINT32;
    uint8_t completionCode = 0;
    uint8_t operationalState = 0;
    uint8_t eventMessageEnable = 0;
    uint8_t presentState = 0;
    uint8_t previousState = 0;
    uint8_t eventState = 0;
    uint32_t presentReading = 0;
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    rc = decode_get_sensor_reading_resp(
        response, responseMsg.size() - sizeof(pldm_msg_hdr), &completionCode,
        dataSize, &operationalState, &eventMessageEnable, &presentState,
        &previousState, &eventState,
        reinterpret_cast<uint8_t*>(&presentReading));

    if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
    {
        std::cerr << "Response Message Error: "
                  << "rc=" << rc << ",cc=" << (int)completionCode << std::endl;
        return false;
    }

    switch (*dataSize)
    {
        case PLDM_SENSOR_DATA_SIZE_UINT8:
        {
            uint8_t* p = (uint8_t*)&presentReading;
            *reading = *p;
        }
        break;
        case PLDM_SENSOR_DATA_SIZE_SINT8:
        {
            int8_t* p = (int8_t*)&presentReading;
            *reading = *p > 0 ? (uint8_t)*p : (uint8_t)*p | 0xffffff00;
        }
        break;
        case PLDM_SENSOR_DATA_SIZE_UINT16:
        {
            uint16_t* p = (uint16_t*)(&presentReading);
            *reading = *p;
        }
        break;
        case PLDM_SENSOR_DATA_SIZE_SINT16:
        {
            int16_t* p = (int16_t*)&presentReading;
            *reading = *p;
        }
        break;
        case PLDM_SENSOR_DATA_SIZE_UINT32:
        {
            uint32_t imax = std::numeric_limits<int>::max();
            uint32_t* p = (uint32_t*)&presentReading;
            *reading = *p > imax ? imax : *p;
        }
        break;
        case PLDM_SENSOR_DATA_SIZE_SINT32:
        {
            int32_t* p = (int32_t*)&presentReading;
            *reading = *p;
        }
        break;
        default:
            *reading = 0;
            break;
    }

    return true;
}

void PLDMSensor::read(void)
{
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being cancelled
        }

        // read timer error
        if (ec)
        {
            std::cerr << "timer error\n";
            return;
        }

        int32_t raw;
        uint8_t dataSize;
        if (getSensorReading(eid, instanceId, sensorId, &raw, &dataSize))
        {
            // DSP0248 27.7
            // Numeric reading conversion formula: Y = [m*X + B]
            // Y = converted reading in Units
            // X = reading from sensor
            // m = resolution
            // B = offset
            double converted = static_cast<double>(raw) * resolution + offset;
            if constexpr (debug)
            {
                std::cerr << "Convered reading:" << converted
                          << "Raw reading:" << static_cast<int>(raw) << "\n";
            }
            updateValue(converted);
        }
        else
        {
            std::cerr << "Invalid read getSensorReading\n";
            incrementError();
        }
        read();
    });
}
