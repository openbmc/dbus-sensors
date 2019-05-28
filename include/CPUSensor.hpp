#pragma once

#include <Thresholds.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sensor.hpp>

class CPUSensor : public Sensor
{
  public:
    CPUSensor(const std::string& path, const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_service& io, const std::string& sensorName,
              std::vector<thresholds::Threshold>&& thresholds,
              const std::string& configuration, int cpuId, bool show,
              const std::pair<double, double>& hysteresis);
    ~CPUSensor();
    static constexpr unsigned int sensorScaleFactor = 1000;
    static constexpr unsigned int sensorPollMs = 1000;
    static constexpr size_t warnAfterErrorCount = 10;
    static constexpr double maxReading = 127;
    static constexpr double minReading = -128;
    static constexpr const char* labelTcontrol = "Tcontrol";

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::deadline_timer waitTimer;
    boost::asio::streambuf readBuf;
    std::string nameTcontrol;
    std::string path;
    double privTcontrol;
    bool show;
    int errCount;
    void setupRead(void);
    void handleResponse(const boost::system::error_code& err);
    void checkThresholds(void) override;
};

extern boost::container::flat_map<std::string, std::unique_ptr<CPUSensor>>
    gCpuSensors;
