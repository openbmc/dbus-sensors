/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaGpuDevice.hpp"

#include "Inventory.hpp"
#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

static constexpr const char* uuidInterfaceName = "xyz.openbmc_project.Common.UUID";

GpuDevice::GpuDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{
    inventory =
        std::make_unique<Inventory>(conn, objectServer, name, mctpRequester,
                                   Inventory::DeviceType::GPU, eid, io);
    
    uuidInterface = objectServer.add_interface(path, uuidInterfaceName);
    uuidInterface->register_property("UUID", std::string{});
    uuidInterface->initialize();
    
    fetchUUID();
    makeSensors();
}

void GpuDevice::fetchUUID()
{
    auto req = std::make_shared<InventoryRequestBuffer>();
    auto resp = std::make_shared<InventoryResponseBuffer>();
    
    int rc = gpu::encodeGetInventoryInformationRequest(
        0, static_cast<uint8_t>(gpu::InventoryPropertyId::DEVICE_GUID), *req);
    if (rc != 0)
    {
        lg2::error("Failed to encode UUID request: rc={RC}", "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *req, *resp,
        [self = weak_from_this(), resp](int sendRecvMsgResult) {
            if (auto device = self.lock())
            {
                device->handleUUIDResponse(sendRecvMsgResult, resp);
            }
        });
}

void GpuDevice::handleUUIDResponse(
    int sendRecvMsgResult,
    std::shared_ptr<InventoryResponseBuffer> responseBuffer)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error("Failed to get UUID from device: rc={RC}", "RC", sendRecvMsgResult);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    gpu::InventoryInfo info;
    int rc = gpu::decodeGetInventoryInformationResponse(
        *responseBuffer, cc, reasonCode, gpu::InventoryPropertyId::DEVICE_GUID, info);
    
    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Failed to decode UUID response: rc={RC}, cc={CC}", "RC", rc, "CC", static_cast<int>(cc));
        return;
    }

    if (!std::holds_alternative<std::string>(info))
    {
        lg2::error("Invalid UUID format received from device");
        return;
    }

    uuid = std::get<std::string>(info);
    uuidInterface->set_property("UUID", uuid);
    lg2::info("Successfully retrieved UUID from device: {UUID}", "UUID", uuid);
}

void GpuDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, objectServer,
        std::vector<thresholds::Threshold>{});

    lg2::info("Added GPU {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void GpuDevice::read()
{
    tempSensor->update();

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }
        read();
    });
}
