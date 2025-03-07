/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensorTask.hpp"

#include "GpuTempSensor.hpp"
#include "MctpRequester.hpp"

#include <GpuSensor.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

// temperature sensors
std::vector<GpuTempSensor> tempSensors{};

using namespace std::literals;

using SensorId = uint8_t;
using SensorName = std::string;
using ChassisPath = std::string;
using ConfigData = std::tuple<mctp_eid_t, SensorId, SensorName, ChassisPath,
                              std::chrono::milliseconds>;

using ObjectPath = std::string;
using ServiceName = std::string;
using InterfaceName = std::string;
using getSubTreeRet = std::vector<
    std::pair<ObjectPath,
              std::vector<std::pair<ServiceName, std::vector<InterfaceName>>>>>;

using GpuSensorConfigMap = std::unordered_map<
    std::string, std::variant<std::vector<std::string>, std::vector<uint8_t>,
                              std::string, int64_t, uint64_t, double, int32_t,
                              uint32_t, int16_t, uint16_t, uint8_t, bool>>;

struct GpuInfo
{
    uint8_t eid;
    uint8_t instanceId;
};

constexpr uint8_t gpuTempSensorId{0};
const std::string gpuTempSensorNamePrefix{"TEMP"};
const std::string gpuNamePrefix{"GPU"};
const std::string gpuChassisPathPrefix{
    "/xyz/openbmc_project/inventory/chassis/GPU"};

static void processGpuEndpoints(
    mctp::MctpRequester& mctpRequester,
    const std::vector<uint8_t>& ocpamiEndpoints,
    const std::function<void(std::vector<GpuInfo>)>& callback)
{
    if (ocpamiEndpoints.empty())
    {
        callback({});
        return;
    }

    // Result vector to store identified GPUs
    auto gpuInfos = std::make_shared<std::vector<GpuInfo>>();

    // Use a shared counter to track when all endpoints have been processed
    auto remaining = std::make_shared<size_t>(ocpamiEndpoints.size());

    for (const auto ocpamiEndpoint : ocpamiEndpoints)
    {
        // Create request message for GPU device identification
        std::vector<uint8_t> reqMsg(
            sizeof(OcpAmiBindingPciVid) +
            sizeof(GpuQueryDeviceIdentificationRequest));

        auto* msg = new (reqMsg.data()) OcpAmiMessage;

        // Encode the QueryDeviceIdentification request
        auto rc =
            gpuEncodeQueryDeviceIdentificationRequest(ocpAmiInstanceMin, msg);
        if (rc != OCP_AMI_SUCCESS)
        {
            lg2::error(
                "processGpuEndpoints(): gpu::encodeQueryDeviceIdentificationRequest failed, rc={RC}",
                "RC", static_cast<int>(rc));

            (*remaining)--;
            if (*remaining == 0)
            {
                // All endpoints processed, return the collected GPU information
                callback(*gpuInfos);
            }
            continue;
        }

        // Placeholder to send the request and handle the response
        // asynchronously
        (void)mctpRequester;
        (void)ocpamiEndpoint;
    }
}

static void getGpuEIDs(
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester,
    std::function<void(std::vector<GpuInfo>)> callback)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    dbusConnection->async_method_call(
        [dbusConnection, &mctpRequester, callback = std::move(callback)](
            const boost::system::error_code& ec, const getSubTreeRet& ret) {
            if (ec)
            {
                lg2::error("getGPUEIDs(): Error:{ERROR}", "ERROR",
                           ec.message());
                callback({});
                return;
            }

            if (ret.empty())
            {
                callback({});
                return;
            }

            // Create a shared vector to collect all discovered endpoints
            auto ocpamiEndpoints = std::make_shared<std::vector<uint8_t>>();

            // Create a shared counter to track completion of all async calls
            auto remaining = std::make_shared<size_t>(0);

            // Process all endpoints
            for (const auto& [objPath, services] : ret)
            {
                for (const auto& [service, ifaces] : services)
                {
                    for (const auto& iface : ifaces)
                    {
                        if (iface == "xyz.openbmc_project.MCTP.Endpoint")
                        {
                            ++(*remaining);
                            dbusConnection->async_method_call(
                                [ocpamiEndpoints, remaining, &mctpRequester,
                                 callback](
                                    const boost::system::error_code& configEc,
                                    const GpuSensorConfigMap& configs) {
                                    if (!configEc)
                                    {
                                        try
                                        {
                                            // auto eid = std::get<uint8_t>(
                                            //     configs.at("EID"));
                                            auto eid = std::get<uint32_t>(
                                                configs.at("EID"));
                                            auto mctpTypes = std::get<
                                                std::vector<uint8_t>>(
                                                configs.at(
                                                    "SupportedMessageTypes"));

                                            if (std::find(mctpTypes.begin(),
                                                          mctpTypes.end(),
                                                          ocpAmiMessageType) !=
                                                mctpTypes.end())
                                            {
                                                ocpamiEndpoints->push_back(eid);

                                                lg2::info(
                                                    "Found OCPAMI Endpoint with ID {EID}",
                                                    "EID", eid);
                                            }
                                        }
                                        catch (const std::exception& e)
                                        {
                                            lg2::error(
                                                "getGPUEIDs(): Error:{ERROR}",
                                                "ERROR", e);
                                        }
                                    }
                                    else
                                    {
                                        lg2::error(
                                            "getGPUEIDs(): Error:{ERROR}",
                                            "ERROR", configEc.message());
                                    }

                                    // Decrement the counter and check if we're
                                    // done with all endpoints
                                    (*remaining)--;
                                    if (*remaining == 0)
                                    {
                                        lg2::info(
                                            "getGPUEIDs(): All endpoint queries completed, found {FOUND} OCPAMI endpoints",
                                            "FOUND", ocpamiEndpoints->size());

                                        // Now that all endpoints have been
                                        // checked, call processGpuEndpoints
                                        processGpuEndpoints(mctpRequester,
                                                            *ocpamiEndpoints,
                                                            callback);
                                    }
                                },
                                service, objPath,
                                "org.freedesktop.DBus.Properties", "GetAll",
                                iface);
                        }
                    }
                }
            }
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void createSensors(
    boost::asio::io_context& /* ctx */,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester, bool verbose)
{
    getGpuEIDs(
        dbusConnection, mctpRequester,
        [dbusConnection, verbose](const std::vector<GpuInfo>& gpuInfos) {
            if (gpuInfos.empty())
            {
                lg2::error("gpuSensorTask(): failed to get OCP AMI GPU list.");
                return;
            }

            for (const auto& gpuInfo : gpuInfos)
            {
                if (verbose)
                {
                    lg2::info(
                        "gpuSensorTask(): GPU_TEMP_SENSOR EID={EID} INSTANCE_ID={IID}.",
                        "EID", gpuInfo.eid, "IID", gpuInfo.instanceId);
                }

                std::string sensorName = gpuNamePrefix;
                sensorName += '_';
                sensorName += std::to_string(gpuInfo.instanceId);
                sensorName += '_';
                sensorName += gpuTempSensorNamePrefix;
                sensorName += '_';
                sensorName += std::to_string(gpuTempSensorId);

                tempSensors.emplace_back(
                    static_cast<sdbusplus::bus::bus&>(*dbusConnection),
                    gpuInfo.eid, gpuTempSensorId, sensorName,
                    gpuChassisPathPrefix + '_' +
                        std::to_string(gpuInfo.instanceId),
                    verbose);
            }

            // start the sensor polling
        });
}
