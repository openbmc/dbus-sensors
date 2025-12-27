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
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

static constexpr auto sensorPollRateMs = 1000;

static std::vector<std::shared_ptr<sdbusplus::bus::match_t>> associationMatches;

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
    const std::string& deviceName, const std::string& configPath,
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
                "Found the GPU with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}, Name {NAME}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId, "NAME", deviceName);

            auto gpu = gpuDevices
                           .insert(std::make_pair(
                               deviceName,
                               std::make_shared<GpuDevice>(
                                   sensorPollRateMs, deviceName, configPath,
                                   conn, eid, io, mctpRequester, objectServer)))
                           .first;
            gpu->second->init();
            break;
        }

        case gpu::DeviceIdentification::DEVICE_SMA:
        {
            lg2::info(
                "Found the SMA Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}, Name {NAME}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId, "NAME", deviceName);

            auto sma = smaDevices
                           .insert(std::make_pair(
                               deviceName,
                               std::make_shared<SmaDevice>(
                                   sensorPollRateMs, deviceName, configPath,
                                   conn, eid, io, mctpRequester, objectServer)))
                           .first;
            sma->second->init();
            break;
        }

        case gpu::DeviceIdentification::DEVICE_PCIE:
        {
            lg2::info(
                "Found the PCIe Device with EID {EID}, DeviceType {DEVTYPE}, InstanceId {IID}, Name {NAME}.",
                "EID", eid, "DEVTYPE", responseDeviceType, "IID",
                responseInstanceId, "NAME", deviceName);

            auto pcieDevice =
                pcieDevices
                    .insert(std::make_pair(
                        deviceName,
                        std::make_shared<PcieDevice>(
                            sensorPollRateMs, deviceName, configPath, conn, eid,
                            io, mctpRequester, objectServer)))
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
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& deviceName, const std::string& configPath)
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
         &mctpRequester, eid, deviceName, configPath,
         queryDeviceIdentificationRequest](const std::error_code& ec,
                                           std::span<const uint8_t> response) {
            processQueryDeviceIdResponse(
                io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                mctpRequester, eid, deviceName, configPath, ec, response);
        });
}

static void processNvidiaMctpVdmConfigSearch(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& deviceName, const std::string& inventoryPath,
    const std::string& configPath, const boost::system::error_code& ec,
    const GetSubTreeType& ret)
{
    std::string finalConfigPath = configPath;

    if (!ec && !ret.empty())
    {
        std::string objPath = ret[0].first;
        if (objPath.find(inventoryPath) != std::string::npos)
        {
            finalConfigPath = objPath;
        }
    }
    else
    {
        lg2::info(
            "EID {EID}: NvidiaMctpVdm config not found under board, using original {PATH}",
            "EID", eid, "PATH", configPath);
    }

    queryDeviceIdentification(io, objectServer, gpuDevices, smaDevices,
                              pcieDevices, conn, mctpRequester, eid, deviceName,
                              finalConfigPath);
}

static void processBoardInventoryResult(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& deviceName, const std::string& boardName,
    const std::string& configPath, const boost::system::error_code& ec,
    const GetSubTreeType& ret)
{
    std::string inventoryPath = configPath;

    if (!ec && !ret.empty())
    {
        // Search for a path that ends with boardName
        for (const auto& [objPath, services] : ret)
        {
            if (objPath.ends_with("/" + boardName) ||
                objPath.ends_with(boardName))
            {
                inventoryPath = objPath;
                lg2::info(
                    "EID {EID}: Found board inventory path {PATH} for board {BOARD}",
                    "EID", eid, "PATH", inventoryPath, "BOARD", boardName);
                break;
            }
        }
    }
    if (inventoryPath == configPath)
    {
        lg2::info(
            "EID {EID}: Board {BOARD} not found in inventory, using config path {PATH}",
            "EID", eid, "BOARD", boardName, "PATH", configPath);
        queryDeviceIdentification(io, objectServer, gpuDevices, smaDevices,
                                  pcieDevices, conn, mctpRequester, eid,
                                  deviceName, configPath);
    }
    else
    {
        // Now search for NvidiaMctpVdm config under the board inventory path
        conn->async_method_call(
            [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
             &mctpRequester, eid, deviceName, inventoryPath,
             configPath](const boost::system::error_code& ec2,
                         const GetSubTreeType& ret2) {
                processNvidiaMctpVdmConfigSearch(
                    io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                    mctpRequester, eid, deviceName, inventoryPath, configPath,
                    ec2, ret2);
            },
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree", inventoryPath, 0,
            std::vector<std::string>{
                "xyz.openbmc_project.Configuration.NvidiaMctpVdm"});
    }
}

static void findBoardInventoryPath(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& deviceName, const std::string& boardName,
    const std::string& configPath)
{
    std::string searchPath{"/xyz/openbmc_project/inventory"};
    std::vector<std::string> ifaceList{
        {"xyz.openbmc_project.Inventory.Item.Chassis"},
        {"xyz.openbmc_project.Inventory.Item.Board"}};

    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid, deviceName, boardName, configPath](
            const boost::system::error_code& ec, const GetSubTreeType& ret) {
            processBoardInventoryResult(
                io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                mctpRequester, eid, deviceName, boardName, configPath, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTree", searchPath, 0,
        ifaceList);
}

static void processConfigPropertiesResult(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& configPath, const boost::system::error_code& ec,
    const SensorBaseConfigMap& configProps)
{
    if (ec)
    {
        lg2::error(
            "EID {EID}: Failed to get config properties: {ERROR}, skipping",
            "EID", eid, "ERROR", ec.message());
        return;
    }

    // Extract the "Name" property
    auto nameIt = configProps.find("Name");
    if (nameIt == configProps.end())
    {
        lg2::error("EID {EID}: Name property not found in config, skipping",
                   "EID", eid);
        return;
    }

    const auto* namePtr = std::get_if<std::string>(&nameIt->second);
    if (namePtr == nullptr)
    {
        lg2::error("EID {EID}: Name property has invalid type, skipping", "EID",
                   eid);
        return;
    }

    const std::string& deviceName = *namePtr;
    lg2::info("EID {EID}: Found device name {NAME}", "EID", eid, "NAME",
              deviceName);

    // Check for "Board" property
    auto boardIt = configProps.find("Board");
    if (boardIt != configProps.end())
    {
        const auto* boardPtr = std::get_if<std::string>(&boardIt->second);
        if ((boardPtr != nullptr) && !boardPtr->empty())
        {
            // Board property exists, search for inventory path
            findBoardInventoryPath(io, objectServer, gpuDevices, smaDevices,
                                   pcieDevices, conn, mctpRequester, eid,
                                   deviceName, escapeName(*boardPtr),
                                   configPath);
            return;
        }
    }

    // No Board property or empty, use configPath as inventory path
    lg2::info("EID {EID}: No Board property found, using config path {PATH}",
              "EID", eid, "PATH", configPath);
    queryDeviceIdentification(io, objectServer, gpuDevices, smaDevices,
                              pcieDevices, conn, mctpRequester, eid, deviceName,
                              configPath);
}

static void getConfigProperties(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& configPath, const std::string& configService)
{
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid,
         configPath](const boost::system::error_code& ec,
                     const SensorBaseConfigMap& configProps) {
            processConfigPropertiesResult(
                io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                mctpRequester, eid, configPath, ec, configProps);
        },
        configService, configPath, "org.freedesktop.DBus.Properties", "GetAll",
        "");
}

static void getConfigService(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& configPath)
{
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid, configPath](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            if (ec || ret.empty())
            {
                lg2::error(
                    "EID {EID}: Failed to get service for config path {PATH}: {ERROR}, skipping",
                    "EID", eid, "PATH", configPath, "ERROR", ec.message());
                return;
            }

            const std::string& configService = ret[0].first;
            getConfigProperties(io, objectServer, gpuDevices, smaDevices,
                                pcieDevices, conn, mctpRequester, eid,
                                configPath, configService);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", configPath,
        std::vector<std::string>{});
}

static void processAssociationEndpointsResult(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const boost::system::error_code& ec,
    const std::variant<std::vector<std::string>>& value)
{
    if (ec)
    {
        lg2::error(
            "EID {EID}: Failed to get endpoints property: {ERROR}, skipping",
            "EID", eid, "ERROR", ec.message());
        return;
    }

    const auto* endpointsPtr = std::get_if<std::vector<std::string>>(&value);
    if (!endpointsPtr || endpointsPtr->empty())
    {
        lg2::error("EID {EID}: endpoints property is empty, skipping", "EID",
                   eid);
        return;
    }

    const std::string& configPath = (*endpointsPtr)[0];
    getConfigService(io, objectServer, gpuDevices, smaDevices, pcieDevices,
                     conn, mctpRequester, eid, configPath);
}

static void getAssociationEndpoints(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& associationPath, const std::string& associationService)
{
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester,
         eid](const boost::system::error_code& ec,
              const std::variant<std::vector<std::string>>& value) {
            processAssociationEndpointsResult(io, objectServer, gpuDevices,
                                              smaDevices, pcieDevices, conn,
                                              mctpRequester, eid, ec, value);
        },
        associationService, associationPath, "org.freedesktop.DBus.Properties",
        "Get", "xyz.openbmc_project.Association", "endpoints");
}

static void processAssociationAdded(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& associationPath, const boost::system::error_code& ec,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& ret)
{
    if (ec || ret.empty())
    {
        lg2::error("EID {EID}: Failed to get service for association {PATH}",
                   "EID", eid, "PATH", associationPath);
        return;
    }

    const std::string& associationService = ret[0].first;
    getAssociationEndpoints(io, objectServer, gpuDevices, smaDevices,
                            pcieDevices, conn, mctpRequester, eid,
                            associationPath, associationService);
}

static void handleAssociationSignal(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& associationPath, sdbusplus::message_t& msg)
{
    sdbusplus::message::object_path objPath;
    std::map<std::string, std::map<std::string, std::variant<std::string>>>
        interfaces;

    msg.read(objPath, interfaces);

    if (interfaces.contains("xyz.openbmc_project.Association"))
    {
        lg2::info("EID {EID}: Association added at {PATH}, processing...",
                  "EID", eid, "PATH", associationPath);

        conn->async_method_call(
            [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
             &mctpRequester, eid, associationPath](
                const boost::system::error_code& ec,
                const std::vector<
                    std::pair<std::string, std::vector<std::string>>>& ret) {
                processAssociationAdded(
                    io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                    mctpRequester, eid, associationPath, ec, ret);
            },
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetObject", associationPath,
            std::vector<std::string>{"xyz.openbmc_project.Association"});
    }
}

static void waitForAssociationSignal(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& associationPath)
{
    auto match = std::make_shared<sdbusplus::bus::match_t>(
        static_cast<sdbusplus::bus_t&>(*conn),
        sdbusplus::bus::match::rules::interfacesAdded() +
            sdbusplus::bus::match::rules::argNpath(0, associationPath),
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid, associationPath](sdbusplus::message_t& msg) {
            handleAssociationSignal(io, objectServer, gpuDevices, smaDevices,
                                    pcieDevices, conn, mctpRequester, eid,
                                    associationPath, msg);
        });

    // Store the match to keep it alive for the duration of the program
    associationMatches.push_back(match);
}

static void processAssociationLookupResult(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& associationPath, bool useSignalWait,
    const boost::system::error_code& ec,
    const std::vector<std::pair<std::string, std::vector<std::string>>>& ret)
{
    if (ec || ret.empty())
    {
        if (useSignalWait)
        {
            lg2::info(
                "EID {EID}: Association not found at {PATH}, waiting for signal...",
                "EID", eid, "PATH", associationPath);
            waitForAssociationSignal(io, objectServer, gpuDevices, smaDevices,
                                     pcieDevices, conn, mctpRequester, eid,
                                     associationPath);
        }
        else
        {
            lg2::error(
                "EID {EID}: No association found at {PATH}, skipping endpoint",
                "EID", eid, "PATH", associationPath);
        }
        return;
    }

    const std::string& associationService = ret[0].first;
    getAssociationEndpoints(io, objectServer, gpuDevices, smaDevices,
                            pcieDevices, conn, mctpRequester, eid,
                            associationPath, associationService);
}

static void checkAssociationAndQueryDevice(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>&
        gpuDevices,
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>&
        smaDevices,
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>&
        pcieDevices,
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, uint8_t eid,
    const std::string& objectPath, bool useSignalWait = false)
{
    std::string associationPath = objectPath + "/configured_by";

    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, eid, associationPath, useSignalWait](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                ret) {
            processAssociationLookupResult(
                io, objectServer, gpuDevices, smaDevices, pcieDevices, conn,
                mctpRequester, eid, associationPath, useSignalWait, ec, ret);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", associationPath,
        std::vector<std::string>{"xyz.openbmc_project.Association"});
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
    mctp::MctpRequester& mctpRequester, const std::string& objectPath,
    const boost::system::error_code& ec, const SensorBaseConfigMap& endpoint,
    bool useSignalWait = false)
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
        lg2::info("Found OCP MCTP VDM Endpoint with ID {EID} at {PATH}", "EID",
                  eid, "PATH", objectPath);

        checkAssociationAndQueryDevice(io, objectServer, gpuDevices, smaDevices,
                                       pcieDevices, conn, mctpRequester, eid,
                                       objectPath, useSignalWait);
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
                         &pcieDevices, conn, &mctpRequester,
                         objPath](const boost::system::error_code& ec,
                                  const SensorBaseConfigMap& endpoint) {
                            processEndpoint(io, objectServer, gpuDevices,
                                            smaDevices, pcieDevices, conn,
                                            mctpRequester, objPath, ec,
                                            endpoint);
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
         &mctpRequester, objectPath](const boost::system::error_code& ec,
                                     const SensorBaseConfigMap& endpoint) {
            processEndpoint(io, objectServer, gpuDevices, smaDevices,
                            pcieDevices, conn, mctpRequester, objectPath, ec,
                            endpoint, true);
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
    conn->async_method_call(
        [&io, &objectServer, &gpuDevices, &smaDevices, &pcieDevices, conn,
         &mctpRequester, objectPath](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                services) {
            if (ec || services.empty())
            {
                lg2::error(
                    "Failed to get service for new MCTP endpoint {PATH}: {ERROR}",
                    "PATH", objectPath, "ERROR", ec.message());
                return;
            }

            const std::string& service = services[0].first;
            getEndpointProperties(io, objectServer, gpuDevices, smaDevices,
                                  pcieDevices, conn, mctpRequester, objectPath,
                                  service);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", objectPath,
        std::vector<std::string>{"xyz.openbmc_project.MCTP.Endpoint"});
}
