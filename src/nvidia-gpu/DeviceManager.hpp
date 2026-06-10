/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "MctpRequester.hpp"
#include "NvidiaGpuDevice.hpp"
#include "NvidiaGpuMctpVdm.hpp"
#include "NvidiaPcieDevice.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaDevice.hpp"
#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/error_code.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

class DeviceManager
{
  public:
    DeviceManager(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  mctp::MctpRequester& mctpRequester);

    void createSensors();
    void onConfigInterfaceRemoved(sdbusplus::message_t& message);

  private:
    void processSensorConfigs(const ManagedObjectType& resp);
    void discoverDevices(const SensorConfigs& configs,
                         const sdbusplus::object_path& path);
    void queryEndpoints(const SensorConfigs& configs,
                        const sdbusplus::object_path& path,
                        const boost::system::error_code& ec,
                        const GetSubTreeType& ret);
    void processEndpoint(const SensorConfigs& configs,
                         const sdbusplus::object_path& path,
                         const boost::system::error_code& ec,
                         const SensorBaseConfigMap& endpoint);
    void queryDeviceIdentification(const SensorConfigs& configs,
                                   const sdbusplus::object_path& path,
                                   uint8_t eid);
    void processQueryDeviceIdResponse(
        const SensorConfigs& configs, const sdbusplus::object_path& path,
        uint8_t eid, const std::error_code& sendRecvMsgResult,
        std::span<const uint8_t> queryDeviceIdentificationResponse);
    void createDeviceForType(
        const SensorConfigs& configs, const std::string& path, uint8_t eid,
        uint8_t responseDeviceType, uint8_t responseInstanceId,
        const gpu::DeviceCapabilities& caps);

    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    mctp::MctpRequester& mctpRequester;

    boost::container::flat_map<std::string, std::shared_ptr<GpuDevice>>
        gpuDevices;
    boost::container::flat_map<std::string, std::shared_ptr<SmaDevice>>
        smaDevices;
    boost::container::flat_map<std::string, std::shared_ptr<PcieDevice>>
        pcieDevices;
};
