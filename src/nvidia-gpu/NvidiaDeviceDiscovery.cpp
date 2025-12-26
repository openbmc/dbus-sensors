/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaDeviceDiscovery.hpp"

#include "NvidiaGpuDevice.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr auto sensorPollRateMs = 1000;

void processQueryDeviceIdResponse(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::error_code& sendRecvMsgResult,
    std::span<const uint8_t> queryDeviceIdentificationResponse)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : sending message over MCTP failed, rc={RC}",
            "EID", eid, "RC", sendRecvMsgResult.message());
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint8_t responseDeviceType = 0;
    uint8_t responseInstanceId = 0;

    auto rc = gpu::decodeQueryDeviceIdentificationResponse(
        queryDeviceIdentificationResponse, cc, reasonCode, responseDeviceType,
        responseInstanceId);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : decode failed, rc={RC}, cc={CC}, reasonCode={RESC}",
            "EID", eid, "RC", rc, "CC", cc, "RESC", reasonCode);
        return;
    }

    switch (static_cast<gpu::DeviceIdentification>(responseDeviceType))
    {
        case gpu::DeviceIdentification::DEVICE_GPU:
        {
            lg2::info(
                "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            std::string gpuName =
                std::format("GPU_{}_{}", eid, responseInstanceId);

            auto gpu =
                gpuDevices
                    .insert(std::make_pair(
                        gpuName, std::make_shared<GpuDevice>(
                                     sensorPollRateMs, gpuName, conn, eid, io,
                                     mctpRequester, objectServer)))
                    .first;
            gpu->second->init();
            break;
        }

        case gpu::DeviceIdentification::DEVICE_SMA:
        {
            lg2::info(
                "Found the SMA Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            std::string smaName =
                std::format("SMA_{}_{}", eid, responseInstanceId);

            auto sma =
                smaDevices
                    .insert(std::make_pair(
                        smaName, std::make_shared<SmaDevice>(
                                     sensorPollRateMs, smaName, conn, eid, io,
                                     mctpRequester, objectServer)))
                    .first;
            sma->second->init();
            break;
        }

        case gpu::DeviceIdentification::DEVICE_PCIE:
        {
            lg2::info(
                "Found the PCIe Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId);

            std::string pcieName =
                std::format("PCIe_device_{}_{}", eid, responseInstanceId);

            auto pcieDevice =
                pcieDevices
                    .insert(std::make_pair(
                        pcieName, std::make_shared<PcieDevice>(
                                      sensorPollRateMs, pcieName, conn, eid, io,
                                      mctpRequester, objectServer)))
                    .first;
            pcieDevice->second->init();
            break;
        }
    }
}

void queryDeviceIdentification(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid)
{
    auto queryDeviceIdentificationRequest = std::make_shared<
        std::array<uint8_t, sizeof(gpu::QueryDeviceIdentificationRequest)>>();

    auto rc = gpu::encodeQueryDeviceIdentificationRequest(
        0, *queryDeviceIdentificationRequest);
    if (rc != 0)
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : encode failed, rc={RC}",
            "EID", eid, "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *queryDeviceIdentificationRequest,
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid, queryDeviceIdentificationRequest](
            const std::error_code& ec, std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(io, objectServer, gpuDevices,
                                         smaDevices, pcieDevices, conn,
                                         mctpRequester, eid, ec, response);
        });
}

void processEndpoint(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const boost::system::error_code& ec,
    const SensorBaseConfigMap& endpoint)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoint: Error:{ERROR}", "ERROR",
                   ec.message());
        return;
    }

    auto hasEid = endpoint.find("EID");
    uint8_t eid{};

    if (hasEid != endpoint.end())
    {
        const auto* eidPtr = std::get_if<uint8_t>(&hasEid->second);
        if (eidPtr != nullptr)
        {
            eid = *eidPtr;
        }
        else
        {
            lg2::error(
                "Error processing MCTP endpoint: Property EID does not have valid type.");
            return;
        }
    }
    else
    {
        lg2::error(
            "Error processing MCTP endpoint: Property EID not found in the configuration.");
        return;
    }

    auto hasMctpTypes = endpoint.find("SupportedMessageTypes");
    std::vector<uint8_t> mctpTypes{};

    if (hasMctpTypes != endpoint.end())
    {
        const auto* mctpTypePtr =
            std::get_if<std::vector<uint8_t>>(&hasMctpTypes->second);
        if (mctpTypePtr != nullptr)
        {
            mctpTypes = *mctpTypePtr;
        }
        else
        {
            lg2::error(
                "Error processing MCTP endpoint with eid {EID} : Property SupportedMessageTypes does not have valid type.",
                "EID", eid);
            return;
        }
    }
    else
    {
        lg2::error(
            "Error processing MCTP endpoint with eid {EID} : Property SupportedMessageTypes not found in the configuration.",
            "EID", eid);
        return;
    }

    if (std::find(mctpTypes.begin(), mctpTypes.end(),
                  ocp::accelerator_management::messageType) != mctpTypes.end())
    {
        lg2::info("Found OCP MCTP VDM Endpoint with ID {EID}", "EID", eid);
        queryDeviceIdentification(io, objectServer, gpuDevices, smaDevices,
                                  pcieDevices, conn, mctpRequester, eid);
    }
}

void queryEndpoints(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const boost::system::error_code& ec,
    const GetSubTreeType& ret)
{
    if (ec)
    {
        lg2::error("Error processing MCTP endpoints: {ERROR}", "ERROR",
                   ec.message());
        return;
    }

    if (ret.empty())
    {
        return;
    }

    for (const auto& [objPath, services] : ret)
    {
        for (const auto& [service, ifaces] : services)
        {
            for (const auto& iface : ifaces)
            {
                if (iface == "xyz.openbmc_project.MCTP.Endpoint")
                {
                    conn->async_method_call(
                        [&io, &objectServer, &gpuDevices, &smaDevices,
                         &pcieDevices, conn,
                         &mctpRequester](const boost::system::error_code& ec,
                                         const SensorBaseConfigMap& endpoint) {
                            processEndpoint(io, objectServer, gpuDevices,
                                            smaDevices, pcieDevices, conn,
                                            mctpRequester, ec, endpoint);
                        },
                        service, objPath, "org.freedesktop.DBus.Properties",
                        "GetAll", iface);
                }
            }
        }
    }
}

void discoverDevices(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester)
{
    std::string searchPath{"/au/com/codeconstruct/"};
    std::vector<std::string> ifaceList{{"xyz.openbmc_project.MCTP.Endpoint"}};

    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester](const boost::system::error_code& ec,
                         const GetSubTreeType& ret) {
            queryEndpoints(io, objectServer, gpuDevices, smaDevices,
                           pcieDevices, conn, mctpRequester, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    mctp::MctpRequester& mctpRequester)
{
    if (!dbusConnection)
    {
        lg2::error("Connection not created");
        return;
    }

    discoverDevices(io, objectServer, gpuDevices, smaDevices, pcieDevices,
                    dbusConnection, mctpRequester);
}

static void getEndpointProperties(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& objectPath,
    const std::string& service)
{
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester](const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint) {
            processEndpoint(io, objectServer, gpuDevices, smaDevices,
                            pcieDevices, conn, mctpRequester, ec, endpoint);
        },
        service, objectPath, "org.freedesktop.DBus.Properties", "GetAll",
        "xyz.openbmc_project.MCTP.Endpoint");
}

void handleMctpEndpointAdded(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& objectPath)
{
    lg2::info("New MCTP Endpoint detected at {PATH}", "PATH", objectPath);

    // Get the service name for this object path
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, objectPath](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            if (ec || ret.empty())
            {
                lg2::error(
                    "Error getting service for MCTP endpoint {PATH}: {ERROR}",
                    "PATH", objectPath, "ERROR", ec.message());
                return;
            }

            const std::string& service = ret[0].first;
            getEndpointProperties(io, objectServer, gpuDevices, smaDevices,
                                  pcieDevices, conn, mctpRequester, objectPath,
                                  service);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", objectPath,
        std::vector<std::string>{"xyz.openbmc_project.MCTP.Endpoint"});
}
