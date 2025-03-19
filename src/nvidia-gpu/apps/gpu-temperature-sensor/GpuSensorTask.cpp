/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#include "GpuSensorTask.hpp"

#include "GpuTempSensor.hpp"
#include "MctpRequester.hpp"

#include <nvidia_sensors.h>
#include <ocp_ami.h>
#include <transport.h>

#include <boost/asio/awaitable.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

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
constexpr std::chrono::milliseconds samplingInterval{1000ms};
const std::string gpuTempSensorNamePrefix{"TEMP"};
const std::string gpuNamePrefix{"GPU"};
const std::string gpuChassisPathPrefix{
    "/xyz/openbmc_project/inventory/chassis/GPU"};

template <typename RetType, typename CompletionToken, typename... InputArgs>
auto asyncDbusMethodCall(
    CompletionToken&& token,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::string& service, const std::string& objpath,
    const std::string& interf, const std::string& method, const InputArgs&... a)
{
    auto initiate = [&](auto handler) {
        dbusConnection->async_method_call(
            [hand = std::move(handler)](const boost::system::error_code& ec,
                                        const RetType& response) mutable {
                hand(ec, std::move(response));
            },
            service, objpath, interf, method, a...);
    };

    // Disable clang-tidy check as the error is in header
    // boost/asio/impl/awaitable.hpp and to exclude that header from clang-tidy
    // checks, clang-tidy configuration option ExcludeHeaderFilterRegex is only
    // supported in clang-tidy version 19.

    // NOLINTNEXTLINE
    return boost::asio::async_initiate<
        CompletionToken, void(boost::system::error_code, RetType)>(
        initiate, std::forward<CompletionToken>(token));
}

boost::asio::awaitable<int> getGPUEIDs(
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection,
    mctp::MctpRequester& mctpRequester, std::vector<GpuInfo>& eids)
{
    std::string searchPath{"/xyz/openbmc_project/mctp/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    getSubTreeRet ret{};

    try
    {
        ret = co_await asyncDbusMethodCall<getSubTreeRet>(
            boost::asio::use_awaitable, dbusConnection,
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
            ifaceList);
    }
    catch (const std::exception& e)
    {
        lg2::error("getGPUEIDs(): Error:{ERROR}", "ERROR", e);
        co_return -1;
    }

    if (ret.empty())
    {
        co_return -1;
    }

    std::vector<uint8_t> ocpamiEndpoints;

    for (const auto& [objPath, services] : ret)
    {
        for (const auto& [service, ifaces] : services)
        {
            for (const auto& iface : ifaces)
            {
                if (iface == "xyz.openbmc_project.MCTP.Endpoint")
                {
                    try
                    {
                        auto configs =
                            co_await asyncDbusMethodCall<GpuSensorConfigMap>(
                                boost::asio::use_awaitable, dbusConnection,
                                service, objPath,
                                "org.freedesktop.DBus.Properties", "GetAll",
                                iface);

                        auto eid = std::get<size_t>(configs["EID"]);
                        auto mctpTypes = std::get<std::vector<uint8_t>>(
                            configs["SupportedMessageTypes"]);

                        if (std::find(mctpTypes.begin(), mctpTypes.end(),
                                      OCP_AMI_MESSAGE_TYPE) != mctpTypes.end())
                        {
                            ocpamiEndpoints.push_back(eid);

                            lg2::info("Found OCPAMI Endpoint with ID {EID}",
                                      "EID", eid);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        lg2::error("getGPUEIDs(): Error:{ERROR}", "ERROR", e);
                        continue;
                    }
                }
            }
        }
    }

    for (const auto ocpamiEndpoint : ocpamiEndpoints)
    {
        std::vector<uint8_t> reqMsg(
            sizeof(struct ocp_ami_binding_pci_vid) +
            sizeof(ocp_ami_oem_nvidia_query_device_identification_req));

        auto* msg = new (reqMsg.data()) struct ocp_ami_msg;

        auto rc = ocp_ami_oem_nvidia_encode_query_device_identification_req(
            OCP_AMI_INSTANCE_MIN, msg);
        if (rc != 0)
        {
            lg2::error(
                "getGPUEIDs(): ocp_ami_oem_nvidia_encode_query_device_identification_req failed, rc={RC}",
                "RC", rc);
            continue;
        }

        std::vector<uint8_t> respMsg;
        std::tie(rc, respMsg) =
            co_await mctpRequester.sendRecvMsg(ocpamiEndpoint, reqMsg);

        if (rc != 0)
        {
            lg2::error(
                "getGPUEIDs(): MctpRequester::sendRecvMsg() failed, rc={RC}",
                "RC", rc);
            continue;
        }

        if (respMsg.empty())
        {
            lg2::error(
                "getGPUEIDs(): MctpRequester::sendRecvMsg() failed, respMsgLen=0");
            continue;
        }

        uint8_t cc = 0;
        uint16_t reasonCode = 0;
        uint8_t deviceIdentification = 0;
        uint8_t instanceId = 0;

        rc = ocp_ami_oem_nvidia_decode_query_device_identification_resp(
            new (respMsg.data()) ocp_ami_msg, respMsg.size(), &cc, &reasonCode,
            &deviceIdentification, &instanceId);
        if (rc != 0)
        {
            lg2::error(
                "getGPUEIDs(): ocp_ami_oem_nvidia_decode_query_device_identification_resp() failed, rc={RC} cc={CC} reasonCode={RESC}\n",
                "RC", rc, "CC", cc, "RESC", reasonCode);
            continue;
        }

        if (deviceIdentification == OCP_AMI_OEM_NVIDIA_DEV_ID_GPU)
        {
            lg2::info(
                "getGPUEIDs(): found GPU with EID {EID} and InstanceId {IID}",
                "EID", ocpamiEndpoint, "IID", instanceId);

            eids.push_back({ocpamiEndpoint, instanceId});
        }
    }

    co_return 0;
}

boost::asio::awaitable<void> gpuSensorTask(
    boost::asio::io_context& ctx,
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection, bool verbose)
{
    mctp::MctpRequester mctpRequester(ctx, 0x7e);

    std::vector<GpuInfo> gpuInfos;
    auto rc = co_await getGPUEIDs(dbusConnection, mctpRequester, gpuInfos);

    if (rc < 0 || gpuInfos.empty())
    {
        lg2::error("gpuSesnorTask(): failed to get OCP AMI GPU list.");
        co_return;
    }

    // create temp sensors
    std::vector<GpuTempSensor> tempSensors{};

    for (const auto gpuInfo : gpuInfos)
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
            static_cast<sdbusplus::bus::bus&>(*dbusConnection), gpuInfo.eid,
            gpuTempSensorId, sensorName,
            gpuChassisPathPrefix + '_' + std::to_string(gpuInfo.instanceId),
            verbose);
    }

    // update sensors regularly
    while (true)
    {
        if (verbose)
        {
            lg2::info("gpuSesnorTask(): updating the sensors.");
        }

        for (auto& sensor : tempSensors)
        {
            co_await sensor.update(mctpRequester);
        }

        boost::asio::steady_timer timeoutTimer{ctx};

        timeoutTimer.expires_after(samplingInterval);
        co_await timeoutTimer.async_wait(boost::asio::use_awaitable);
    }

    co_return;
}
