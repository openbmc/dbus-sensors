/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaMetricReport.hpp"

#include <boost/asio/error.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <chrono>
#include <format>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

static const std::string telemetryService = "xyz.openbmc_project.Telemetry";

static const std::string reportManagerPath =
    "/xyz/openbmc_project/Telemetry/Reports";

static const std::string reportManagerInterface =
    "xyz.openbmc_project.Telemetry.ReportManager";

static const std::string reportInterface =
    "xyz.openbmc_project.Telemetry.Report";

static const std::string reportManagerAddReportMethod = "AddReport";

SensorMetricReport::SensorMetricReport(
    const std::string& name,
    const std::shared_ptr<sdbusplus::asio::connection>& conn) :
    reportName(name), conn(conn), configTimer(conn->get_io_context())
{}

void SensorMetricReport::appendSensorToReadingParameters(
    const std::string& sensorName, const std::string& sensorType,
    const std::string& chassisName)
{
    readingParameters.emplace_back(
        std::vector<SensorPathType>{
            std::make_tuple(std::format("/xyz/openbmc_project/sensors/{}/{}",
                                        sensorType, sensorName),
                            std::format("/redfish/v1/Chassis/{}/Sensors/{}_{}",
                                        chassisName, sensorType, sensorName))},
        operationType, collectionTimescope, collectionDurationMs);
}

void SensorMetricReport::addSensor(const std::string& sensorName,
                                   const std::string& sensorType,
                                   const std::string& chassisName)
{
    appendSensorToReadingParameters(sensorName, sensorType, chassisName);

    if (readingParameters.size() == 1)
    {
        conn->async_method_call(
            [sensorName, chassisName, this](const boost::system::error_code& ec,
                                            const std::string& objectPath) {
                handleCreateReportResponse(sensorName, chassisName, ec,
                                           objectPath);
            },
            telemetryService, reportManagerPath, reportManagerInterface,
            reportManagerAddReportMethod,
            std::format("TelemetryService/{}", reportName), reportName,
            reportingType, reportUpdates, appendLimit, reportActions,
            collectionIntervalMs, readingParameters, true);
    }
    else
    {
        // setup a timer to allow multiple sensors to be added before making the
        // DBus call to update the sensors list of the report

        configTimer.expires_after(std::chrono::seconds(1));
        configTimer.async_wait([this](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }

            conn->async_method_call(
                [this](const boost::system::error_code& ec) {
                    handlerInsertSensorToReportResponse(ec);
                },
                telemetryService,
                std::format("{}/TelemetryService/{}", reportManagerPath,
                            reportName),
                "org.freedesktop.DBus.Properties", "Set", reportInterface,
                "ReadingParameters", readingParametersVariant);
        });
    }
}

void SensorMetricReport::handleCreateReportResponse(
    const std::string& sensorName, const std::string& chassisName,
    const boost::system::error_code& ec, const std::string& objectPath)
{
    if (ec)
    {
        lg2::error(
            "Error creating metric report {REPORT} for sensor {SENSOR} of chassis {CHASSIS} : {RC}",
            "REPORT", reportName, "SENSOR", sensorName, "CHASSIS", chassisName,
            "RC", ec.message());
        return;
    }

    lg2::info(
        "Successfully created metric report {REPORT} at object path {PATH} for sensor {SENSOR} of chassis {CHASSIS}",
        "REPORT", reportName, "PATH", objectPath, "SENSOR", sensorName,
        "CHASSIS", chassisName);
}

void SensorMetricReport::handlerInsertSensorToReportResponse(
    const boost::system::error_code& ec)
{
    if (ec)
    {
        lg2::error("Error inserting sensors to metric report {REPORT} : {RC}",
                   "REPORT", reportName, "RC", ec.message());
        return;
    }

    lg2::info("Successfully inserted sensors to metric report {REPORT}",
              "REPORT", reportName);
}
