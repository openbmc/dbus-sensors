#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>

class HwmonTempSensor
{
  public:
    std::string name;
    std::string configuration;
    HwmonTempSensor(const std::string &path, const std::string &objectType,
                    sdbusplus::asio::object_server &objectServer,
                    std::shared_ptr<sdbusplus::asio::connection> &conn,
                    boost::asio::io_service &io, const std::string &fan_name,
                    std::vector<thresholds::Threshold> &&thresholds,
                    const std::string &sensorConfiguration);
    ~HwmonTempSensor();

  private:
    std::string path;
    std::string objectType;
    sdbusplus::asio::object_server &objServer;
    std::vector<thresholds::Threshold> thresholds;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensor_interface;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        threshold_interface_warning;
    std::shared_ptr<sdbusplus::asio::dbus_interface>
        threshold_interface_critical;
    boost::asio::posix::stream_descriptor input_dev;
    boost::asio::deadline_timer wait_timer;
    boost::asio::streambuf read_buf;
    double value;
    int err_count;
    double max_value;
    double min_value;
    void setup_read(void);
    void handle_response(const boost::system::error_code &err);
    void check_thresholds(void);
    void update_value(const double &new_value);
    void assert_thresholds(thresholds::Level level,
                           thresholds::Direction direction, bool assert);
    void set_initial_properties(
        std::shared_ptr<sdbusplus::asio::connection> &conn);
};