/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSensorConfig.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

SmaDevice::SmaDevice(const SensorConfigs& configs, const std::string& name,
                     const sdbusplus::object_path& path,
                     const std::shared_ptr<sdbusplus::asio::connection>& conn,
                     uint8_t eid, boost::asio::io_context& io,
                     mctp::MctpRequester& mctpRequester,
                     sdbusplus::asio::object_server& objectServer) :
    eid(eid), sensorPollMs(std::chrono::milliseconds{configs.pollRate}),
    waitTimer(io, std::chrono::steady_clock::duration(0)),
    mctpRequester(mctpRequester), conn(conn), objectServer(objectServer),
    configs(configs), name(escapeName(name)), path(path)
{}

void SmaDevice::init()
{
    makeSensors();
}

void SmaDevice::makeSensors()
{
    tempSensor = std::make_shared<NvidiaGpuTempSensor>(
        conn, mctpRequester, name + "_TEMP_0", path, eid, smaTempSensorId,
        objectServer, std::vector<thresholds::Threshold>{},
        gpu::DeviceIdentification::DEVICE_SMA);

    initLeakSensors();

    lg2::info("Added MCA {NAME} Sensors with chassis path: {PATH}.", "NAME",
              name, "PATH", path);

    read();
}

void SmaDevice::read()
{
    tempSensor->update();

    for (auto& sensor : leakSensors)
    {
        sensor->update();
    }

    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait(
        [weak{weak_from_this()}](const boost::system::error_code& ec) {
            std::shared_ptr<SmaDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid SmaDevice reference");
                return;
            }
            if (ec)
            {
                return;
            }
            self->read();
        });
}

void SmaDevice::initLeakSensors()
{
    auto leakReq = std::make_shared<
        std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize>>();
    gpu::encodeGetLeakDetectionInfoRequest(0, *leakReq);
    mctpRequester.sendRecvMsg(
        eid, *leakReq,
        [weak{weak_from_this()}, leakReq](const std::error_code& ec,
                                          std::span<const uint8_t> response) {
            auto self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid NvidiaSmaLeakSensorCarrier reference");
                return;
            }
            if (ec)
            {
                lg2::error(
                    "Transport error probing Leak Sensor {NAME}: ec={EC}",
                    "NAME", self->name, "EC", ec.message());
                return;
            }

            ocp::accelerator_management::CompletionCode cc{};
            uint16_t reasonCode = 0;
            std::vector<gpu::LeakSensorData> parsedSensors;

            auto rc = gpu::decodeGetLeakDetectionInfoResponse(
                response, cc, reasonCode, parsedSensors);

            if (rc != 0 ||
                cc != ocp::accelerator_management::CompletionCode::SUCCESS)
            {
                lg2::error(
                    "Error creating Leak Sensor: decoding GetLeakDetectionInfo response for {NAME} failed, rc={RC}, cc={CC}, reasonCode={RESC}",
                    "NAME", self->name, "RC", rc, "CC", cc, "RESC", reasonCode);
                return;
            }

            if (parsedSensors.empty())
            {
                lg2::error(
                    "Error creating Leak Sensor: decode success but no sensors returned for {NAME}",
                    "NAME", self->name);
                return;
            }

            for (auto& parsedSensor : parsedSensors)
            {
                uint8_t sensorId = parsedSensor.sensorId;
                std::string sensorName =
                    self->name + "_LEAKDETECTOR_" + std::to_string(sensorId);

                // Use dynamic thresholds from hardware if exactly 3 are
                // provided
                std::vector<thresholds::Threshold> sensorThresholds;
                if (parsedSensor.thresholds.size() == 3)
                {
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL, thresholds::Direction::LOW,
                        parsedSensor.thresholds[0] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::WARNING, thresholds::Direction::LOW,
                        parsedSensor.thresholds[1] / 1000.0);
                    sensorThresholds.emplace_back(
                        thresholds::Level::CRITICAL,
                        thresholds::Direction::HIGH,
                        parsedSensor.thresholds[2] / 1000.0);
                }

                auto newSensor = std::make_shared<NvidiaSmaLeakSensor>(
                    self->conn, self->mctpRequester, sensorName, self->path.str,
                    self->eid, sensorId, self->objectServer,
                    std::move(sensorThresholds),
                    gpu::DeviceIdentification::DEVICE_SMA);
                newSensor->updateValue(parsedSensor.adcReadingMv / 1000.0);
                self->leakSensors.emplace_back(newSensor);
            }
            lg2::info("Added SMA Leak Sensors with chassis path: {PATH}.",
                      "PATH", self->path.str);
        });
}
