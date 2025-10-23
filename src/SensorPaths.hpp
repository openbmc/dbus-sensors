#pragma once

#include <xyz/openbmc_project/Sensor/Value/common.hpp>

#include <array>
#include <string>

namespace sensor_paths
{

struct UnitMap
{
    std::string_view dbusPathName;
    std::string_view dbusUnitName;
    std::string_view emUnitName;
};

constexpr std::array<UnitMap, 11> unitsMap = {{
    {"xyz.openbmc_project.Sensor.Value.Unit.DegreesC", "DegreesC",
     "temperature"},
    {"xyz.openbmc_project.Sensor.Value.Unit.RPMS", "RPMS", "fan_tach"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Volts", "Volts", "voltage"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Meters", "Meters", "distance"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Amperes", "Amperes", "current"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Watts", "Watts", "power"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Joules", "Joules", "energy"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Percent", "Percent", "percent"},
    {"xyz.openbmc_project.Sensor.Value.Unit.CFM", "CFM", "airflow"},
    {"xyz.openbmc_project.Sensor.Value.Unit.Pascals", "Pascals", "pressure"},
    {"xyz.openbmc_project.Sensor.Value.Unit.PercentRH", "PercentRH",
     "humidity"},
}};
// This is an allowlist of the units a sensor can measure. Should be in sync
// with
// phosphor-dbus-interfaces/blob/master/yaml/xyz/openbmc_project/Sensor/Value.interface.yaml#L38

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

std::string convertToFullUnits(const std::string& units);

} // namespace sensor_paths
