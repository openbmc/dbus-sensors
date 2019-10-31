#pragma once
#include "Utils.hpp"

#include <boost/asio/io_service.hpp>
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
    Threshold(const Level& lev, const Direction& dir, const double& val,
              bool write = true) :
        level(lev),
        direction(dir), value(val), writeable(write)
    {
    }
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

void assertThresholds(Sensor* sensor, thresholds::Level level,
                      thresholds::Direction direction, bool assert);

using TimerPair = std::pair<bool, boost::asio::deadline_timer>;

struct ThresholdTimer
{

    ThresholdTimer(boost::asio::io_service& ioService, Sensor* sensor) :
        io(ioService), sensor(sensor)
    {
    }

    void startTimer(const Threshold& threshold)
    {
        constexpr const size_t waitTime = 5;
        TimerPair* pair = nullptr;

        for (TimerPair& timer : timers)
        {
            if (!timer.first)
            {
                pair = &timer;
                break;
            }
        }
        if (pair == nullptr)
        {
            pair = &timers.emplace_back(false, boost::asio::deadline_timer(io));
        }
        pair->first = true;
        pair->second.expires_from_now(boost::posix_time::seconds(waitTime));
        pair->second.async_wait(
            [this, pair, threshold](boost::system::error_code ec) {
                pair->first = false;

                if (ec == boost::asio::error::operation_aborted)
                {
                    return; // we're being canceled
                }
                else if (ec)
                {
                    std::cerr << "timer error: " << ec.message() << "\n";
                    return;
                }
                if (isPowerOn())
                {
                    assertThresholds(sensor, threshold.level,
                                     threshold.direction, true);
                }
            });
    }

    boost::asio::io_service& io;
    std::list<TimerPair> timers;
    Sensor* sensor;
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
                      size_t thresholdCount);

void updateThresholds(Sensor* sensor);
// returns false if a critical threshold has been crossed, true otherwise
bool checkThresholds(Sensor* sensor);
void checkThresholdsPowerDelay(Sensor* sensor, ThresholdTimer& thresholdTimer);
} // namespace thresholds
