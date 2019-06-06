#pragma once
#include <Utils.hpp>
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

struct ThresholdTimer
{

    ThresholdTimer(boost::asio::io_service& io, Sensor* sensor) :
        criticalTimer(io), warningTimer(io), sensor(sensor)
    {
    }

    void startTimer(const Threshold& threshold)
    {
        constexpr const size_t waitTime = 2;

        if (threshold.level == WARNING && !warningRunning)
        {
            warningRunning = true;
            warningTimer.expires_from_now(boost::posix_time::seconds(waitTime));
            warningTimer.async_wait(
                [this, threshold](boost::system::error_code ec) {
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return; // we're being canceled
                    }
                    if (isPowerOn())
                    {
                        assertThresholds(sensor, threshold.level,
                                         threshold.direction, true);
                    }
                    warningRunning = false;
                });
        }
        else if (threshold.level == CRITICAL && !criticalRunning)
        {
            criticalRunning = true;
            criticalTimer.expires_from_now(
                boost::posix_time::seconds(waitTime));
            criticalTimer.async_wait(
                [this, threshold](boost::system::error_code ec) {
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return; // we're being canceled
                    }
                    if (isPowerOn())
                    {
                        assertThresholds(sensor, threshold.level,
                                         threshold.direction, true);
                    }
                    criticalRunning = false;
                });
        }
    }

    boost::asio::deadline_timer criticalTimer;
    boost::asio::deadline_timer warningTimer;
    bool criticalRunning = false;
    bool warningRunning = false;
    Sensor* sensor;
};

bool parseThresholdsFromConfig(
    const SensorData& sensorData,
    std::vector<thresholds::Threshold>& thresholdVector,
    const std::string* matchLabel = nullptr);

bool parseThresholdsFromAttr(std::vector<thresholds::Threshold>& thresholds,
                             const std::string& inputPath,
                             const double& scaleFactor, const double& offset = 0);
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
