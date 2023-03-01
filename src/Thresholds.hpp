#pragma once

#include "Utils.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <nlohmann/json.hpp>

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct Sensor;
namespace thresholds
{
enum class Level
{
    WARNING,
    CRITICAL,
    PERFORMANCELOSS,
    SOFTSHUTDOWN,
    HARDSHUTDOWN,
    ERROR
};
enum class Direction
{
    HIGH,
    LOW,
    ERROR
};
struct Threshold
{
    Threshold(
        const Level& lev, const Direction& dir, const double& val,
        const double hysteresis = std::numeric_limits<double>::quiet_NaN(),
        bool write = true) :
        level(lev),
        direction(dir), value(val), hysteresis(hysteresis), writeable(write)
    {}
    Level level;
    Direction direction;
    double value;
    double hysteresis;
    bool writeable;

    bool operator==(const Threshold& rhs) const
    {
        return (level == rhs.level && direction == rhs.direction &&
                value == rhs.value);
    }
};

void assertThresholds(Sensor* sensor, double assertValue,
                      thresholds::Level level, thresholds::Direction direction,
                      bool assert);

struct TimerUsed
{
    bool used;
    Level level;
    Direction direction;
    bool assert;
};

using TimerPair = std::pair<struct TimerUsed, boost::asio::steady_timer>;

struct ThresholdTimer
{

    explicit ThresholdTimer(boost::asio::io_context& ioService) : io(ioService)
    {}

    bool hasActiveTimer(const Threshold& threshold, bool assert)
    {
        for (TimerPair& timer : timers)
        {
            if (timer.first.used)
            {
                if ((timer.first.level == threshold.level) &&
                    (timer.first.direction == threshold.direction) &&
                    (timer.first.assert == assert))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void stopTimer(const Threshold& threshold, bool assert)
    {
        struct TimerUsed timerUsed = {};
        for (TimerPair& timer : timers)
        {
            timerUsed = timer.first;
            if (timerUsed.used)
            {
                if ((timerUsed.level == threshold.level) &&
                    (timerUsed.direction == threshold.direction) &&
                    (timerUsed.assert == assert))
                {
                    timer.second.cancel();
                }
            }
        }
    }

    void startTimer(const std::weak_ptr<Sensor>& weakSensor,
                    const Threshold& threshold, bool assert,
                    double assertValue);

    boost::asio::io_context& io;
    std::list<TimerPair> timers;
};

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel = nullptr, const int* sensorIndex = nullptr);

bool parseThresholdsFromAttr(
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string& inputPath, const double& scaleFactor,
    const double& offset = 0);

struct ThresholdDefinition
{
    Level level;
    uint8_t sevOrder;
    const char* levelName;
};

constexpr static std::array<thresholds::ThresholdDefinition, 5> thresProp = {
    {{Level::WARNING, 0, "Warning"},
     {Level::CRITICAL, 1, "Critical"},
     {Level::PERFORMANCELOSS, 2, "PerformanceLoss"},
     {Level::SOFTSHUTDOWN, 3, "SoftShutdown"},
     {Level::HARDSHUTDOWN, 4, "HardShutdown"}}};

std::string getInterface(Level level);

void persistThreshold(const std::string& path, const std::string& baseInterface,
                      const thresholds::Threshold& threshold,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      size_t thresholdCount, const std::string& label);

void updateThresholds(Sensor* sensor);
// returns false if a critical threshold has been crossed, true otherwise
bool checkThresholds(Sensor* sensor);
void checkThresholdsPowerDelay(const std::weak_ptr<Sensor>& weakSensor,
                               ThresholdTimer& thresholdTimer);

} // namespace thresholds
