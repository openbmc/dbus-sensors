#pragma once
#include <Utils.hpp>
#include <nlohmann/json.hpp>

struct Sensor;
namespace thresholds
{
enum Level
{
    WARNING,
    CRITICAL
};
enum Direction
{
    HIGH,
    LOW
};
struct Threshold
{
    Threshold(const Level &lev, const Direction &dir, const double &val,
              bool write = true) :
        level(lev),
        direction(dir), value(val), writeable(write)
    {
    }
    Level level;
    Direction direction;
    double value;
    bool writeable;
    bool asserted = false;
};

bool parseThresholdsFromConfig(
    const SensorData &sensorData,
    std::vector<thresholds::Threshold> &thresholdVector,
    const std::string *matchLabel = nullptr);

bool parseThresholdsFromAttr(std::vector<thresholds::Threshold> &thresholds,
                             const std::string &inputPath,
                             const double &scaleFactor);
bool hasCriticalInterface(
    const std::vector<thresholds::Threshold> &thresholdVector);

bool hasWarningInterface(
    const std::vector<thresholds::Threshold> &thresholdVector);

void persistThreshold(const std::string &baseInterface, const std::string &path,
                      const thresholds::Threshold &threshold,
                      std::shared_ptr<sdbusplus::asio::connection> &conn);

void checkThresholds(Sensor *sensor);
void assertThresholds(Sensor *sensor, thresholds::Level level,
                      thresholds::Direction direction, bool assert);
} // namespace thresholds
