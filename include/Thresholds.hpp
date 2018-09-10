#pragma once
#include <Utils.hpp>
#include <nlohmann/json.hpp>

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
};

bool ParseThresholdsFromConfig(
    const SensorData &sensorData,
    std::vector<thresholds::Threshold> &thresholdVector,
    const std::string *matchLabel = nullptr);

bool ParseThresholdsFromAttr(std::vector<thresholds::Threshold> &thresholds,
                             const std::string &input_path,
                             const double scale_factor);
bool HasCriticalInterface(
    const std::vector<thresholds::Threshold> &threshold_vector);
bool HasWarningInterface(
    const std::vector<thresholds::Threshold> &threshold_vector);

void persistThreshold(const std::string &baseInterface, const std::string &path,
                      const thresholds::Threshold &threshold,
                      std::shared_ptr<sdbusplus::asio::connection> &conn);
} // namespace thresholds
