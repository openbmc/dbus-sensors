#pragma once

#include <xyz/openbmc_project/Sensor/Value/common.hpp>

#include <string>

namespace sensor_paths
{

// This is an allowlist of the units a sensor can measure.

using SensorValue = sdbusplus::common::xyz::openbmc_project::sensor::Value;
using SensorValueUnit =
    sdbusplus::common::xyz::openbmc_project::sensor::Value::Unit;

const std::string unitDegreesC =
    SensorValue::convertUnitToString(SensorValueUnit::DegreesC);
const std::string unitRPMs =
    SensorValue::convertUnitToString(SensorValueUnit::RPMS);
const std::string unitVolts =
    SensorValue::convertUnitToString(SensorValueUnit::Volts);
const std::string unitMeters =
    SensorValue::convertUnitToString(SensorValueUnit::Meters);
const std::string unitAmperes =
    SensorValue::convertUnitToString(SensorValueUnit::Amperes);
const std::string unitWatts =
    SensorValue::convertUnitToString(SensorValueUnit::Watts);
const std::string unitJoules =
    SensorValue::convertUnitToString(SensorValueUnit::Joules);
const std::string unitPercent =
    SensorValue::convertUnitToString(SensorValueUnit::Percent);
const std::string unitCFM =
    SensorValue::convertUnitToString(SensorValueUnit::CFM);
const std::string unitPascals =
    SensorValue::convertUnitToString(SensorValueUnit::Pascals);
const std::string unitPercentRH =
    SensorValue::convertUnitToString(SensorValueUnit::PercentRH);

std::string getPathForUnits(const std::string& units);

std::string escapePathForDbus(const std::string& name);

} // namespace sensor_paths
