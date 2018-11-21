#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class CPUSensor : public Sensor
{
  public:
    CPUSensor(const std::string &path, const std::string &objectType,
              sdbusplus::asio::object_server &objectServer,
              std::shared_ptr<sdbusplus::asio::connection> &conn,
              boost::asio::io_service &io, const std::string &fanName,
              std::vector<thresholds::Threshold> &&thresholds,
              const std::string &configuration);
    ~CPUSensor();
    static constexpr unsigned int sensorScaleFactor = 1000;
    static constexpr unsigned int sensorPollMs = 1000;

  private:
    sdbusplus::asio::object_server &objServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    int errCount;
    void setupRead(void);
    void handleResponse(const boost::system::error_code &err);
    void checkThresholds(void) override;
};
