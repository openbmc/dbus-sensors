#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class CPUSensor : public Sensor
{
  private:
    std::string path;
    std::string objectType;
    sdbusplus::asio::object_server &objServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::string name;
    boost::asio::posix::stream_descriptor input_dev;
    boost::asio::deadline_timer wait_timer;
    boost::asio::streambuf read_buf;
    int err_count;
    double max_value;
    double min_value;
    void setup_read(void);
    void handle_response(const boost::system::error_code &err);
    void check_thresholds(void);
    void update_value(const double &new_value);

    void set_initial_properties(
        std::shared_ptr<sdbusplus::asio::connection> &conn);

  public:
    std::string configuration;
    CPUSensor(const std::string &path, const std::string &objectType,
              sdbusplus::asio::object_server &object_server,
              std::shared_ptr<sdbusplus::asio::connection> &conn,
              boost::asio::io_service &io, const std::string &fan_name,
              std::vector<thresholds::Threshold> &&thresholds,
              const std::string &configuration);
    ~CPUSensor();
    constexpr static unsigned int SENSOR_SCALE_FACTOR = 1000;
    constexpr static unsigned int SENSOR_POLL_MS = 1000;
};
