#pragma once
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <nlohmann/json.hpp>

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
    Threshold(const Level& lev, const Direction& dir, const double& val,
              const double& hys = std::numeric_limits<double>::quiet_NaN(),
              bool write = true) :
        level(lev),
        direction(dir), value(val), hysteresis(hys), writeable(write)
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

using TimerPair = std::pair<struct TimerUsed, boost::asio::deadline_timer>;

struct ThresholdTimer
{

    ThresholdTimer(boost::asio::io_service& ioService) : io(ioService)
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

    boost::asio::io_service& io;
    std::list<TimerPair> timers;
};

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel = nullptr, const int* sensorIndex = nullptr);

bool parseThresholdsFromAttr(std::vector<thresholds::Threshold>& thresholds,
                             const std::string& inputPath,
                             const double& scaleFactor,
                             const double& offset = 0);
bool hasCriticalInterface(
    const std::vector<thresholds::Threshold>& thresholdVector);

bool hasWarningInterface(
    const std::vector<thresholds::Threshold>& thresholdVector);

void persistThreshold(const std::string& baseInterface, const std::string& path,
                      const thresholds::Threshold& threshold,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      size_t thresholdCount, const std::string& label);

void updateThresholds(Sensor* sensor);
// returns false if a critical threshold has been crossed, true otherwise
bool checkThresholds(Sensor* sensor);
void checkThresholdsPowerDelay(const std::weak_ptr<Sensor>& weakSensor,
                               ThresholdTimer& thresholdTimer);

} // namespace thresholds
