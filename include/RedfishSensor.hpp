#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

class RedfishUnit
{
  public:
    std::string redfishType;
    std::string redfishUnits;
    std::string dbusUnits;
    double rangeMin;
    double rangeMax;
};

class RedfishUnitLookup
{
  public:
    std::vector<RedfishUnit> units;

  public:
    const RedfishUnit& lookup(const std::string& name) const;
};

class RedfishThreshold
{
  public:
    std::string redfishName;
    thresholds::Level level;
    thresholds::Direction direction;
};

class RedfishThresholdLookup
{
  public:
    std::vector<RedfishThreshold> thresholds;

  public:
    const RedfishThreshold& lookup(const std::string& name) const;
};

class RedfishCompletion
{
  public:
    std::string queryRequest;
    std::string queryResponse;

    nlohmann::json jsonResponse;

    int msElapsed = 0;
    int errorCode = 0;
};

// Forward declaration;
class RedfishConnection;

// Encapsulates a Redfish transaction
// This class does not need to be long-lived
class RedfishTransaction :
    public std::enable_shared_from_this<RedfishTransaction>
{
  public:
    // Fill these in before calling submit()
    std::string queryRequest;
    std::function<void(const RedfishCompletion& response)> callbackResponse;

    // Returns immediately, and also will call your callback later
    void submitRequest(std::shared_ptr<RedfishConnection> connection);

  private:
    std::chrono::steady_clock::time_point timeStarted;

    void handleCallback(const std::string& response, int code);
};

class RedfishSensorMatcher
{
  public:
    std::string redfishName;
    std::string redfishId;

  public:
    bool isEmpty() const;
    bool isMatch(const RedfishSensorMatcher& other) const;

    void clear();
    bool fillFromJson(const nlohmann::json& json);
};

class RedfishChassisMatcher
{
  public:
    std::string redfishName;
    std::string redfishId;
    std::string manufacturer;
    std::string model;
    std::string partNumber;
    std::string sku;
    std::string serialNumber;
    std::string sparePartNumber;
    std::string version;

  public:
    bool isEmpty() const;
    bool isMatch(const RedfishChassisMatcher& other) const;

    void clear();
    bool fillFromJson(const nlohmann::json& json);
};

class RedfishSensorCandidate :
    public std::enable_shared_from_this<RedfishSensorCandidate>
{
  public:
    std::string sensorPath;
    bool isAcceptable;

    RedfishSensorMatcher characteristics;

    // Avoids having to fetch the same Redfish URL twice during discovery
    nlohmann::json readingCache;
};

// Unlike RedfishChassis, which comes from configuration, and can be used
// across multiple servers, RedfishChassisCandidate comes from parsing
// the JSON output of a server, and is owned by that individual server.
class RedfishChassisCandidate :
    public std::enable_shared_from_this<RedfishChassisCandidate>
{
  public:
    std::string chassisPath;
    std::string sensorsPath;
    bool isAcceptable;

    RedfishChassisMatcher characteristics;

    std::vector<std::string> sensorPaths;
    bool haveSensorPaths;

    // For matching up sensors to this Chassis
    boost::container::flat_map<std::string,
                               std::shared_ptr<RedfishSensorCandidate>>
        pathsToSensorCandidates;
};

class RedfishMetricReport :
    public std::enable_shared_from_this<RedfishMetricReport>
{
  public:
    std::string reportPath;

    bool isHelpful;
    bool isCollected;

    // Avoids having to fetch the same Redfish URL twice during discovery
    nlohmann::json reportCache;
};

// Forward declaration
class RedfishSensor;

class RedfishServer : public std::enable_shared_from_this<RedfishServer>
{
  public:
    // Configuration fields
    std::string configName;
    std::string host;
    std::string protocol;
    std::string user;
    std::string password;
    int port = 0;

  public:
    int readingsReaped = 0;
    int readingsReports = 0;
    int readingsGoodReported = 0;
    int readingsGoodIndividual = 0;

    int ticksElapsed = 0;
    int ticksReading = 0;
    int ticksLaggingBehind = 0;
    int ticksDiscoveryStarting = 0;
    int ticksDiscoveryContinuing = 0;

    int64_t msNetworkTime = 0;
    int64_t msReadingTime = 0;
    int64_t msDiscoveryTime = 0;

    int transactionsStarted = 0;
    int transactionsSuccess = 0;
    int transactionsFailure = 0;

    int actionsDiscovery = 0;
    int actionsReading = 0;
    int completionsDiscovery = 0;
    int completionsReading = 0;

    // Sensors that are configured to use this Server
    std::vector<std::shared_ptr<RedfishSensor>> sensorsServed;

    bool isRelevant = false;

  public:
    void staleReaper(const std::chrono::steady_clock::time_point& now);

    void provideContext(std::shared_ptr<boost::asio::io_service> io,
                        std::shared_ptr<sdbusplus::asio::connection> conn,
                        std::shared_ptr<sdbusplus::asio::object_server> obj);

    void startTimer();
    void handleTimer();

    void startNetworking();
    void nextAction();

  private:
    // Optional because needs context passed in at construction time
    std::optional<boost::asio::steady_timer> pollTimer;
    std::shared_ptr<RedfishConnection> networkConnection;

    // Necessary to hold on to, to create RedfishSensorImpl when ready
    std::shared_ptr<boost::asio::io_service> ioContext;
    std::shared_ptr<sdbusplus::asio::connection> dbusConnection;
    std::shared_ptr<sdbusplus::asio::object_server> objectServer;

    // Information learned from Redfish during discovery
    std::string pathChassis;
    std::string pathTelemetry;
    std::string pathMetrics;
    std::vector<std::string> reportPaths;
    std::vector<std::string> chassisPaths;
    bool haveReportPaths = false;
    bool haveChassisPaths = false;

    // For matching up sensors by name during fast path
    boost::container::flat_map<std::string, std::shared_ptr<RedfishSensor>>
        pathsToSensors;

    // For matching up sensors to their intended Chassis on this server
    boost::container::flat_map<std::string,
                               std::shared_ptr<RedfishChassisCandidate>>
        pathsToChassisCandidates;

    // For keeping track of each report gathered during reading
    boost::container::flat_map<std::string,
                               std::shared_ptr<RedfishMetricReport>>
        pathsToMetricReports;

    // For remembering starting times to measure how long it took
    std::chrono::steady_clock::time_point timeDiscovery;
    std::chrono::steady_clock::time_point timeReading;
    std::chrono::steady_clock::time_point timeAction;

    bool stubActivated = false;

    // Network state
    bool discoveryDone = false;
    bool networkBusy = false;

    std::shared_ptr<RedfishTransaction> activeTransaction;

  private:
    void advanceDiscovery();
    void acceptSensors();
    void readSensors();
    void advanceReading();

    // Transaction bookends
    bool sendTransaction(const RedfishTransaction& transaction);
    bool doneTransaction(const RedfishCompletion& completion);

    // Parsers of JSON obtained from Redfish
    bool fillFromRoot(const nlohmann::json& json);
    bool fillFromTelemetry(const nlohmann::json& json);

    bool fillFromMetricCollection(const nlohmann::json& json);
    bool fillFromMetricReport(const nlohmann::json& json);

    bool fillFromChassisCollection(const nlohmann::json& json);
    bool fillFromChassisCandidate(const nlohmann::json& json,
                                  const std::string& path);

    bool fillFromSensorCollection(const nlohmann::json& json,
                                  const std::string& path);
    bool fillFromSensorCandidate(const nlohmann::json& json,
                                 const std::string& path);

    bool fillFromSensorAccepted(const nlohmann::json& json);

    int checkMetricReport(const nlohmann::json& json, int& outMatched);

    bool acceptMetricReport(const nlohmann::json& json,
                            const std::chrono::steady_clock::time_point& now);
    bool acceptSensorReading(const nlohmann::json& json,
                             const std::chrono::steady_clock::time_point& now);

    // Queriers to initiate new Redfish network communication
    void queryRoot();
    void queryTelemetry();
    void queryMetricCollection();
    void queryMetricReport(const std::string& path);
    void queryChassisCollection();
    void queryChassisCandidate(const std::string& path);
    void querySensorCollection(const std::string& path);
    void querySensorCandidate(const std::string& path);
    void collectMetricReport(const std::string& path);
    void collectSensorReading(const std::string& path);
};

// Forward declaration
class RedfishSensor;

// This object comes from entity-manager configuration, and it is allowed
// to use the same RedfishChassis with multiple Redfish servers.
class RedfishChassis : public std::enable_shared_from_this<RedfishChassis>
{
  public:
    // Configuration fields
    std::string configName;
    RedfishChassisMatcher characteristics;

    // State
    bool isRelevant = false;

  public:
    // Sensors that are configured to use this Chassis
    std::vector<std::shared_ptr<RedfishSensor>> sensorsContained;
};

// Separate object because cannot be created until info learned from server
class RedfishSensorImpl :
    public Sensor,
    public std::enable_shared_from_this<RedfishSensorImpl>
{
  public:
    RedfishSensorImpl(const std::string& objectType,
                      sdbusplus::asio::object_server& objectServer,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      const std::string& sensorName,
                      const std::string& sensorUnits,
                      std::vector<thresholds::Threshold>&& thresholdsIn,
                      const std::string& sensorConfiguration, double maxReading,
                      double minReading, const PowerState& powerState);
    ~RedfishSensorImpl() override;

  private:
    sdbusplus::asio::object_server& objectServer;

  private:
    void checkThresholds(void) override;
};

class RedfishSensor : public std::enable_shared_from_this<RedfishSensor>
{
  public:
    // Configuration fields
    std::string configName;
    RedfishSensorMatcher characteristics;

    std::string configChassis;
    std::string configServer;
    PowerState powerState;

    // Learned during initialization
    std::string inventoryPath;

    // Learned from matched Redfish object
    std::string chassisPath;
    std::string redfishPath;

    // Populated by information from matched Redfish object
    std::string units;
    double minValue = std::numeric_limits<double>::quiet_NaN();
    double maxValue = std::numeric_limits<double>::quiet_NaN();
    std::vector<thresholds::Threshold> thresholds;

    // The last known good reading
    double readingValue = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point readingWhen;

    // Sensor reading state
    bool isRelevant = false;
    bool isCollected = false;

  public:
    // Do not call this until enough information learned from Redfish
    void createImpl(std::shared_ptr<sdbusplus::asio::object_server> objServer,
                    std::shared_ptr<sdbusplus::asio::connection> dbusConn);

  public:
    std::shared_ptr<RedfishSensorImpl> impl;

    // Server and Chassis that this sensor is configured to use
    std::shared_ptr<RedfishServer> server;
    std::shared_ptr<RedfishChassis> chassis;
};
