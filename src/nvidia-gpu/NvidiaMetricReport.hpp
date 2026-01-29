/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/asio/connection.hpp>

#include <cstdint>
#include <memory>
#include <string>

class SensorMetricReport
{
  public:
    SensorMetricReport(
        const std::string& name,
        const std::shared_ptr<sdbusplus::asio::connection>& conn);

    void addSensor(const std::string& sensorName, const std::string& sensorType,
                   const std::string& chassisName);

  private:
    void appendSensorToReadingParameters(const std::string& sensorName,
                                         const std::string& sensorType,
                                         const std::string& chassisName);

    void handleCreateReportResponse(
        const std::string& sensorName, const std::string& chassisName,
        const boost::system::error_code& ec, const std::string& objectPath);

    void handlerInsertSensorToReportResponse(
        const boost::system::error_code& ec);

    const std::string reportingType =
        "xyz.openbmc_project.Telemetry.Report.ReportingType.OnChange";
    const std::string reportUpdates =
        "xyz.openbmc_project.Telemetry.Report.ReportUpdates.Overwrite";
    const std::vector<std::string> reportActions = {
        "xyz.openbmc_project.Telemetry.Report.ReportActions.LogToMetricReportsCollection"};
    const std::string operationType =
        "xyz.openbmc_project.Telemetry.Report.OperationType.Average";
    const std::string collectionTimescope =
        "xyz.openbmc_project.Telemetry.Report.CollectionTimescope.Point";
    const uint64_t collectionIntervalMs = 0;
    const uint64_t collectionDurationMs = 0;
    const uint64_t appendLimit = 1;

    using SensorPathType =
        std::tuple<sdbusplus::message::object_path, std::string>;

    using ReadingParametersType =
        std::vector<std::tuple<std::vector<SensorPathType>, std::string,
                               std::string, uint64_t>>;

    std::variant<ReadingParametersType> readingParametersVariant;

    ReadingParametersType& readingParameters =
        std::get<ReadingParametersType>(readingParametersVariant);

    std::string reportName;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    boost::asio::steady_timer configTimer;
};
