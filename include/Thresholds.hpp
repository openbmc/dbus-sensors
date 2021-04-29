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
              bool write = true) :
        level(lev),
        direction(dir), value(val), writeable(write)
    {}
    Level level;
    Direction direction;
    double value;
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

struct ThresholdTimer : public std::enable_shared_from_this<ThresholdTimer>
{

    ThresholdTimer(boost::asio::io_service& ioService,
                   std::weak_ptr<Sensor> sensor) :
        std::enable_shared_from_this<ThresholdTimer>(),
        io(ioService), sensor(sensor)
    {}

    void stopAll()
    {
        for (TimerPair& timer : timers)
        {
            if (timer.first.used)
                timer.second.cancel();
        }
    }

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

    void startTimer(const Threshold& threshold, bool assert,
                    double assertValue);

    boost::asio::io_service& io;
    std::list<TimerPair> timers;
    std::weak_ptr<Sensor> sensor;
};

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel = nullptr);

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
void checkThresholdsPowerDelay(Sensor* sensor,
                               std::shared_ptr<ThresholdTimer> thresholdTimer);

} // namespace thresholds
