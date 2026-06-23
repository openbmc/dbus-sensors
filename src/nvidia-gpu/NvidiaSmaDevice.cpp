/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaSmaDevice.hpp"

#include "NvidiaDeviceDiscovery.hpp"
#include "NvidiaGpuTempSensor.hpp"
#include "NvidiaSmaLeakSensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

SmaDevice::SmaDevice(const SensorConfigs& configs, const std::string& name,
                     const std::string& path,
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
    auto tempReq = std::make_shared<
        std::array<uint8_t, gpu::getTemperatureReadingRequestSize>>();

    auto rc =
        gpu::encodeGetTemperatureReadingRequest(0, smaTempSensorId, *tempReq);
    if (rc != 0)
    {
        lg2::error("Error encoding GetTemperatureReadingRequest: rc={RC}", "RC",
                   rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *tempReq,
        [weak{weak_from_this()}, tempReq](const std::error_code& ec,
                                          std::span<const uint8_t> response) {
            std::shared_ptr<SmaDevice> self = weak.lock();
            if (!self)
            {
                lg2::error("Invalid SmaDevice reference");
                return;
            }

            if (!ec)
            {
                ocp::accelerator_management::CompletionCode cc{};
                uint16_t reasonCode = 0;
                double temp = 0;
                auto rc = gpu::decodeGetTemperatureReadingResponse(
                    response, cc, reasonCode, temp);

                if (rc == 0 &&
                    cc == ocp::accelerator_management::CompletionCode::SUCCESS)
                {
                    self->tempSensor = std::make_shared<NvidiaGpuTempSensor>(
                        self->conn, self->mctpRequester, self->name + "_TEMP_0",
                        self->path, self->eid, smaTempSensorId,
                        self->objectServer,
                        std::vector<thresholds::Threshold>{},
                        gpu::DeviceIdentification::DEVICE_SMA);

                    lg2::info(
                        "Added MCA {NAME} Temp Sensor with chassis path: {PATH}.",
                        "NAME", self->name, "PATH", self->path);
                }
            }

            // Probe and add leak
            auto leakReq = std::make_shared<
                std::array<uint8_t, gpu::getLeakDetectionInfoRequestSize>>();

            auto rc2 = gpu::encodeGetLeakDetectionInfoRequest(0, *leakReq);
            if (rc2 != 0)
            {
                lg2::error(
                    "Error encoding GetLeakDetectionInfoRequest: rc={RC}", "RC",
                    rc2);
                self->read();
                return;
            }

            self->mctpRequester.sendRecvMsg(
                self->eid, *leakReq,
                [weak, leakReq](const std::error_code& ec2,
                                std::span<const uint8_t> response2) {
                    std::shared_ptr<SmaDevice> self = weak.lock();
                    if (!self)
                    {
                        lg2::error("Invalid SmaDevice reference");
                        return;
                    }

                    if (!ec2)
                    {
                        ocp::accelerator_management::CompletionCode cc{};
                        uint16_t reasonCode = 0;
                        std::vector<gpu::LeakSensorData> parsedSensors;

                        auto rc = gpu::decodeGetLeakDetectionInfoResponse(
                            response2, cc, reasonCode, parsedSensors);

                        if (rc == 0 &&
                            cc == ocp::accelerator_management::CompletionCode::
                                      SUCCESS &&
                            !parsedSensors.empty())
                        {
                            self->leakSensor =
                                std::make_shared<NvidiaSmaLeakSensor>(
                                    self->conn, self->mctpRequester,
                                    self->name + "_LEAKDETECTOR_0", self->path,
                                    self->eid, self->objectServer,
                                    std::vector<thresholds::Threshold>{
                                        thresholds::Threshold(
                                            thresholds::Level::CRITICAL,
                                            thresholds::Direction::HIGH, 1.815),
                                        thresholds::Threshold(
                                            thresholds::Level::WARNING,
                                            thresholds::Direction::LOW, 1.65),
                                        thresholds::Threshold(
                                            thresholds::Level::CRITICAL,
                                            thresholds::Direction::LOW, 0.165)},
                                    gpu::DeviceIdentification::DEVICE_SMA);

                            lg2::info(
                                "Added MCA {NAME} Leak Sensor with chassis path: {PATH}.",
                                "NAME", self->name, "PATH", self->path);
                        }
                    }

                    if (!self->tempSensor && !self->leakSensor)
                    {
                        lg2::error(
                            "No valid sensors created for MCA {NAME}. Stopping poll loop.",
                            "NAME", self->name);
                        return;
                    }

                    self->read();
                });
        });
}

void SmaDevice::read()
{
    if (tempSensor)
    {
        tempSensor->update();
    }

    if (leakSensor)
    {
        leakSensor->update();
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
