#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

static constexpr unsigned int sensorPollMs = 2000;
constexpr auto sensorObjectPath = "/xyz/openbmc_project/sensors/voltage/";
enum class battery
{
    batteryLow,
    batteryFailed,
    batteryPresence,
};

class BatteryStatus :
    public Discrete,
    public std::enable_shared_from_this<BatteryStatus>
{
  public:
    BatteryStatus(sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_context& io, const std::string& sensorName,
                  const std::string& deviceName,
                  const std::string& sensorConfiguration);
    ~BatteryStatus() override;
    void setupRead(void);

  private:
    uint8_t reading = 0;
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer waitTimer;
    std::string deviceName;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    void restartRead();
    void monitorState();
};
