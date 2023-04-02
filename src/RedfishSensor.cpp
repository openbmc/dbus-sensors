#include <RedfishSensor.hpp>
#include <SensorPaths.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

static constexpr bool debug = false;

static constexpr int pollIntervalTimeMs = 1000;
static constexpr int staleSensorTimeMs = 4000;

const RedfishUnit& RedfishUnitLookup::lookup(const std::string& name) const
{
    // Matching on any string is OK, there is no overlap
    for (const auto& unit : units)
    {
        if (unit.redfishType == name)
        {
            return unit;
        }
        if (unit.redfishUnits == name)
        {
            return unit;
        }
        if (unit.dbusUnits == name)
        {
            return unit;
        }
    }

    // Indicate error by returning the padding "end" element
    return *(units.crbegin());
}

const RedfishThreshold&
    RedfishThresholdLookup::lookup(const std::string& name) const
{
    for (const auto& threshold : thresholds)
    {
        if (threshold.redfishName == name)
        {
            return threshold;
        }
    }

    // Indicate error by returning the padding "end" element
    return *(thresholds.crbegin());
}

static bool characteristicMatches(const std::string& a, const std::string& b)
{
    // If empty, compare as "don't care", so will always match then
    if (a.empty() || b.empty())
    {
        return true;
    }

    // Both are not empty, so must have same content, in order to return true
    return (a == b);
}

bool RedfishChassisMatcher::isEmpty() const
{
    // All must be empty, in order to return true
    return (redfishName.empty() && redfishId.empty() && manufacturer.empty() &&
            model.empty() && partNumber.empty() && sku.empty() &&
            serialNumber.empty() && sparePartNumber.empty() && version.empty());
}

bool RedfishChassisMatcher::isMatch(const RedfishChassisMatcher& other) const
{
    // It must have at least one characteristic to match on
    if (isEmpty())
    {
        return false;
    }
    if (other.isEmpty())
    {
        return false;
    }

    // All characteristics must match, in order to return true
    if (characteristicMatches(redfishName, other.redfishName) &&
        characteristicMatches(redfishId, other.redfishId) &&
        characteristicMatches(manufacturer, other.manufacturer) &&
        characteristicMatches(model, other.model) &&
        characteristicMatches(partNumber, other.partNumber) &&
        characteristicMatches(sku, other.sku) &&
        characteristicMatches(serialNumber, other.serialNumber) &&
        characteristicMatches(sparePartNumber, other.sparePartNumber) &&
        characteristicMatches(version, other.version))
    {
        if constexpr (debug)
        {
            std::cerr << "Chassises match: name " << redfishName << ", id "
                      << redfishId << " and name " << other.redfishName
                      << ", id " << other.redfishId << "\n";
        }
        return true;
    }

    return false;
}

void RedfishChassisMatcher::clear()
{
    redfishName.clear();
    redfishId.clear();

    manufacturer.clear();
    model.clear();
    partNumber.clear();
    sku.clear();
    serialNumber.clear();
    sparePartNumber.clear();
    version.clear();
}

bool RedfishSensorMatcher::isEmpty() const
{
    // All must be empty, in order to return true
    return (redfishName.empty() && redfishId.empty());
}

bool RedfishSensorMatcher::isMatch(const RedfishSensorMatcher& other) const
{
    // It must have at least one characteristic to match on
    if (isEmpty())
    {
        return false;
    }
    if (other.isEmpty())
    {
        return false;
    }

    // All characteristics must match, in order to return true
    if (characteristicMatches(redfishName, other.redfishName) &&
        characteristicMatches(redfishId, other.redfishId))
    {
        if constexpr (debug)
        {
            std::cerr << "Sensors match: name " << redfishName << ", id "
                      << redfishId << " and name " << other.redfishName
                      << ", id " << other.redfishId << "\n";
        }
        return true;
    }

    return false;
}

void RedfishSensorMatcher::clear()
{
    redfishName.clear();
    redfishId.clear();
}

void RedfishServer::staleReaper(
    const std::chrono::steady_clock::time_point& now)
{
    for (const auto& sensorPtr : sensorsServed)
    {
        RedfishSensor& sensor = *sensorPtr;

        if (!(sensor.isRelevant))
        {
            continue;
        }

        if (!(sensor.impl))
        {
            // Sensor has not completed initialization yet
            continue;
        }

        // This is intentionally isnan(), not isfinite()
        if (std::isnan(sensor.readingValue))
        {
            // It's already dead
            continue;
        }

        auto msAge = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - sensor.readingWhen)
                         .count();

        if (msAge < staleSensorTimeMs)
        {
            // It's still alive
            continue;
        }

        std::cerr << "Sensor " << sensor.configName
                  << " reading has gone stale: " << sensor.readingValue
                  << " value, " << msAge << " ms age\n";

        sensor.readingValue = std::numeric_limits<double>::quiet_NaN();
        sensor.readingWhen = now;
        sensor.impl->updateValue(sensor.readingValue);

        ++readingsReaped;
    }
}

RedfishSensorImpl::RedfishSensorImpl(
    const std::string& objectTypeIn,
    sdbusplus::asio::object_server& objectServerIn,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConn,
    const std::string& sensorName, const std::string& sensorUnits,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, double maxReading,
    double minReading, const PowerState& powerState) :
    // FUTURE: Verify name is not double-escaped, escapeName() here,
    // but then sensor_paths::escapePathForDbus() in base class
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectTypeIn, false, false, maxReading, minReading, dbusConn,
           powerState),
    objectServer(objectServerIn)
{
    // Like ExternalSensor, this class can represent any type of sensor,
    // so caller must specify the units it will be physically measuring.
    std::string dbusPath = sensor_paths::getPathForUnits(sensorUnits);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }

    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += dbusPath;
    objectPath += '/';
    objectPath += sensorName;

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(objectPath, interface);
    }

    association =
        objectServer.add_interface(objectPath, association::interface);
    setInitialProperties(sensorUnits);

    if constexpr (debug)
    {
        std::cerr << "RedfishSensor: Constructed " << name << ", config "
                  << configurationPath << ", type " << objectType << ", path "
                  << objectPath << ", type " << objectType << ", min "
                  << minReading << ", max " << maxReading << "\n";
    }
}

RedfishSensorImpl::~RedfishSensorImpl()
{
    objectServer.remove_interface(association);
    for (const auto& iface : thresholdInterfaces)
    {
        objectServer.remove_interface(iface);
    }
    objectServer.remove_interface(sensorInterface);

    if constexpr (debug)
    {
        std::cerr << "RedfishServer: Destructed " << name << "\n";
    }
}

void RedfishSensorImpl::checkThresholds()
{
    thresholds::checkThresholds(this);
}

void RedfishSensor::createImpl(
    const std::shared_ptr<sdbusplus::asio::object_server>& objServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConn)
{
    if constexpr (debug)
    {
        std::cerr << "RedfishSensor: Creating D-Bus object for " << configName
                  << "\n";
    }

    auto thresholdsCopy = thresholds;

    impl.reset();

    impl = std::make_shared<RedfishSensorImpl>(
        "RedfishSensor", *objServer, dbusConn, configName, units,
        std::move(thresholdsCopy), inventoryPath, maxValue, minValue,
        powerState);

    if constexpr (debug)
    {
        std::cerr << "Created successfully\n";
    }
}

static void timerCallback(const std::weak_ptr<RedfishServer>& that,
                          const boost::system::error_code& ec)
{
    if (ec)
    {
        std::cerr << "Timer callback error: " << ec.message();
        return;
    }

    auto lockThat = that.lock();
    if (lockThat)
    {
        lockThat->handleTimer();
        return;
    }

    std::cerr << "Timer callback ignored: Server object has disappeared\n";
}

void RedfishServer::startTimer()
{
    // Timer does not need kicking off again if already initialized
    if (pollTimer)
    {
        std::cerr << "Server " << configName << " already initialized\n";
        return;
    }

    pollTimer.emplace(*ioContext);
    std::chrono::steady_clock::time_point when =
        std::chrono::steady_clock::now();
    when += std::chrono::milliseconds(pollIntervalTimeMs);

    auto weakThis = weak_from_this();

    pollTimer->expires_at(when);
    pollTimer->async_wait([weakThis](const boost::system::error_code& ec) {
        timerCallback(weakThis, ec);
    });

    if constexpr (debug)
    {
        std::cerr << "Server " << configName << " timer ready\n";
    }
}

// This will check state and kick off the next network action,
// as it is called from the timer callback handler.
void RedfishServer::nextAction()
{
    // Avoid doubling-up work queues by not submitting more if already busy
    if (networkBusy)
    {
        // It is normal to be continuously busy during discovery
        if (discoveryDone)
        {
            // But not normal to still have prior tick going during reading
            ++ticksLaggingBehind;
            if constexpr (debug)
            {
                std::cerr << "Server " << configName
                          << " timer tick, lagging behind: "
                          << ticksLaggingBehind << " ticks\n";
            }
        }
        else
        {
            ++ticksDiscoveryContinuing;
            if constexpr (debug)
            {
                std::cerr << "Server " << configName
                          << " timer tick, discovery continuing: "
                          << ticksDiscoveryContinuing << " ticks\n";
            }
        }
        return;
    }

    // Do discovery, in lieu of reading, until discovery all done
    if (!discoveryDone)
    {
        ++ticksDiscoveryStarting;
        timeDiscovery = timeAction;

        if constexpr (debug)
        {
            std::cerr << "Server " << configName
                      << " timer tick, discovery starting: "
                      << ticksDiscoveryStarting << " ticks\n";
        }

        advanceDiscovery();
        return;
    }

    ++ticksReading;
    timeReading = timeAction;

    if constexpr (debug)
    {
        std::cerr << "Server " << configName
                  << " timer tick, reading sensors: " << ticksReading
                  << " ticks\n";
    }

    readSensors();
}

void RedfishServer::handleTimer()
{
    std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now();

    timeAction = now;

    ++ticksElapsed;

    // Initiates the next network activity, unless network is already busy
    // Keeps track of how many times we had to skip because network was busy
    nextAction();

    // Expire stale sensors
    staleReaper(timeAction);

    std::chrono::steady_clock::time_point thisExpiry = pollTimer->expiry();
    std::chrono::steady_clock::time_point nextExpiry =
        thisExpiry + std::chrono::milliseconds(pollIntervalTimeMs);

    // This is explicitly not expires_from_now(). Instead, the expiration
    // time is incremented manually, to ensure accurate intervals, that do
    // not drift slower over time due to processing overhead.
    pollTimer->expires_at(nextExpiry);

    auto weakThis = weak_from_this();
    pollTimer->async_wait([weakThis](const boost::system::error_code& ec) {
        timerCallback(weakThis, ec);
    });

    if constexpr (debug)
    {
        // If lagging, "now" will get ahead of "thisExpiry"
        auto msLag = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - thisExpiry)
                         .count();

        std::chrono::steady_clock::time_point later =
            std::chrono::steady_clock::now();

        auto msProc =
            std::chrono::duration_cast<std::chrono::milliseconds>(later - now)
                .count();

        std::cerr << "Server " << configName << " timer: " << ticksElapsed
                  << " ticks, lagging " << msLag << " ms, processing " << msProc
                  << " ms\n";
    }
}

void RedfishServer::provideContext(
    std::shared_ptr<boost::asio::io_context> io,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    std::shared_ptr<sdbusplus::asio::object_server> obj)
{
    // Have to hold onto these, needed for various I/O during operation
    ioContext = std::move(io);
    objectServer = std::move(obj);
    dbusConnection = std::move(conn);
}

void RedfishServer::advanceDiscovery()
{
    // This can be called multiple times per tick
    ++actionsDiscovery;

    // Learn where Chassis is
    if (pathChassis.empty())
    {
        // Fills Chassis, and maybe also Telemetry
        queryRoot();
        return;
    }

    // Optionally might have learned where Telemetry is
    if (!(pathTelemetry.empty()))
    {
        // Continue to learning where MetricReports is
        if (pathMetrics.empty())
        {
            // Fills Metrics
            queryTelemetry();
            return;
        }

        // Continue to filling in the list of reports
        if (!haveReportPaths)
        {
            // Fills MetricReportPaths
            queryMetricCollection();
            return;
        }

        // Collect all metric reports, for use during preflight
        for (const std::string& reportPath : reportPaths)
        {
            auto iter = pathsToMetricReports.find(reportPath);
            if (iter == pathsToMetricReports.end())
            {
                // Fills in a pathsToMetricReports element
                queryMetricReport(reportPath);
                return;
            }
        }
    }

    // Continue to filling in the list of chassises
    if (!haveChassisPaths)
    {
        // Fills ChassisPaths
        queryChassisCollection();
        return;
    }

    // Ask chassis candidates where their sensors are
    // Query one chassis candidate per pass, until all are queried
    for (const std::string& chassisPath : chassisPaths)
    {
        auto iter = pathsToChassisCandidates.find(chassisPath);
        if (iter == pathsToChassisCandidates.end())
        {
            // Fills in a pathsToChassisCandidates element
            queryChassisCandidate(chassisPath);
            return;
        }
    }

    // Fill in the list of available sensors on each chassis
    // Query one chassis candidate per pass, until all are queried
    for (const std::string& chassisPath : chassisPaths)
    {
        auto iterCha = pathsToChassisCandidates.find(chassisPath);
        if (iterCha == pathsToChassisCandidates.end())
        {
            std::cerr
                << "Internal error: Chassis candidate was never filled in\n";

            // Should not happen if previous query was successful
            continue;
        }

        RedfishChassisCandidate& candCha = *(iterCha->second);

        if (!(candCha.isAcceptable))
        {
            // This chassis is no longer interesting to us
            continue;
        }

        std::string sensorsPath = candCha.sensorsPath;
        if (sensorsPath.empty())
        {
            // This chassis contains no sensors
            continue;
        }

        if (candCha.haveSensorPaths)
        {
            // This chassis already completed this pass
            continue;
        }

        // Fills in this chassis candidate's sensorPaths vector
        querySensorCollection(sensorsPath);
        return;
    }

    // Look at sensor candidates on all chassis candidates
    // Query one sensor candidate per pass, until all are queried
    for (const std::string& chassisPath : chassisPaths)
    {
        auto iterCha = pathsToChassisCandidates.find(chassisPath);
        if (iterCha == pathsToChassisCandidates.end())
        {
            std::cerr
                << "Internal error: Chassis candidate was never filled in\n";

            // Should not happen if previous query was successful
            continue;
        }

        RedfishChassisCandidate& candCha = *(iterCha->second);

        if (!(candCha.isAcceptable))
        {
            // This chassis is no longer interesting to us
            continue;
        }

        for (const std::string& sensorPath : candCha.sensorPaths)
        {
            auto iterSens = candCha.pathsToSensorCandidates.find(sensorPath);
            if (iterSens == candCha.pathsToSensorCandidates.end())
            {
                // Fills in pathToSensorCandidates for this sensorPath element
                querySensorCandidate(sensorPath);
                return;
            }
        }
    }

    // At this point, all necessary information has been filled in
    acceptSensors();

    discoveryDone = true;

    auto now = std::chrono::steady_clock::now();
    auto msTaken = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - timeDiscovery)
                       .count();

    // There should be only one discovery cycle if all goes well
    msDiscoveryTime += msTaken;
    ++completionsDiscovery;

    if constexpr (debug)
    {
        double average = static_cast<double>(msDiscoveryTime);
        average /= static_cast<double>(completionsDiscovery);
        auto msAverage = static_cast<int>(average);

        std::cerr << "Discovery done, " << msTaken << " ms, " << msAverage
                  << " ms average\n";
    }

    if constexpr (debug)
    {
        std::cerr << "Redfish discovery complete: " << actionsDiscovery
                  << " actions, " << completionsDiscovery
                  << " completions, ticks " << ticksDiscoveryStarting
                  << " starts, " << ticksDiscoveryContinuing << " continues\n";
    }
}

void RedfishServer::acceptSensors()
{
    if constexpr (debug)
    {
        std::cerr << "Ready to finalize what was discovered\n";
    }

    // FUTURE: Make sure this works after accepting sensorsChanged
    for (auto& sensorPair : pathsToSensors)
    {
        sensorPair.second.reset();
    }
    pathsToSensors.clear();

    size_t countDiscovered = 0;
    size_t countAlready = 0;
    size_t countNotRelevant = 0;
    size_t countAmbiguous = 0;
    size_t countNotFound = 0;
    size_t countIncomplete = 0;

    // This is the meat of the discovery algorithm, and puts the pieces
    // together, filling in pathsToSensors and instantiating sensor objects
    for (const auto& sensorPtr : sensorsServed)
    {
        RedfishSensor& sensor = *sensorPtr;

        if (!(sensor.isRelevant))
        {
            ++countNotRelevant;
            continue;
        }

        // Do not discover again if already instantiated
        if (sensor.impl)
        {
            ++countAlready;
            continue;
        }

        size_t numChaMatches = 0;
        std::shared_ptr<RedfishChassisCandidate> chaCandPtr;

        // Exactly one relevant Chassis must match desired characteristics
        for (const auto& chaCandPair : pathsToChassisCandidates)
        {
            RedfishChassisCandidate& chaCand = *(chaCandPair.second);

            if (!(chaCand.isAcceptable))
            {
                continue;
            }

            if (sensor.chassis->characteristics.isMatch(
                    chaCand.characteristics))
            {
                ++numChaMatches;
                chaCandPtr = chaCandPair.second;

                if constexpr (debug)
                {
                    std::cerr << "Chassis " << sensor.chassis->configName
                              << " likely is " << chaCand.chassisPath << "\n";
                }
            }
        }

        if (numChaMatches > 1)
        {
            std::cerr << "Chassis " << sensor.chassis->configName
                      << " configuration is ambiguous, unable to narrow down "
                         "which one it was on Redfish server "
                      << configName << "\n";

            ++countAmbiguous;
            continue;
        }

        if (numChaMatches < 1)
        {
            std::cerr << "Chassis " << sensor.chassis->configName
                      << " was not found on Redfish server " << configName
                      << "\n";

            ++countNotFound;
            continue;
        }

        // At this point, the Chassis is known, now look through its sensors
        size_t numSensMatches = 0;
        std::shared_ptr<RedfishSensorCandidate> sensCandPtr;

        // Exactly one relevant Sensor on this Chassis must match, similarly
        for (const auto& sensCandPair : chaCandPtr->pathsToSensorCandidates)
        {
            RedfishSensorCandidate& sensCand = *(sensCandPair.second);

            if (!(sensCand.isAcceptable))
            {
                continue;
            }

            if (sensor.characteristics.isMatch(sensCand.characteristics))
            {
                ++numSensMatches;
                sensCandPtr = sensCandPair.second;

                if constexpr (debug)
                {
                    std::cerr << "Sensor " << sensor.configName << " likely is "
                              << sensCand.sensorPath << "\n";
                }
            }
        }

        if (numSensMatches > 1)
        {
            std::cerr << "Sensor " << sensor.configName
                      << " configuration is ambiguous, unable to narrow down "
                         "which one it was in Redfish chassis "
                      << sensor.chassis->configName << "\n";

            ++countAmbiguous;
            continue;
        }

        if (numSensMatches < 1)
        {
            std::cerr << "Sensor " << sensor.configName
                      << " was not found in Redfish chassis "
                      << sensor.chassis->configName << "\n";

            ++countNotFound;
            continue;
        }

        // Achieved a match, cache learned Redfish paths for fast lookup later
        // Fill in what is necessary to call fillFromSensorAccepted()
        sensor.redfishPath = sensCandPtr->sensorPath;
        sensor.chassisPath = chaCandPtr->chassisPath;
        pathsToSensors[sensor.redfishPath] = sensorPtr;

        // Now we know which sensor to parse this learned information into
        if (!(fillFromSensorAccepted(sensCandPtr->readingCache)))
        {
            std::cerr << "Sensor " << sensor.configName
                      << " has incomplete information on Redfish server "
                      << configName << "\n";

            // Not enough information to call createImpl
            ++countIncomplete;
            continue;
        }

        // This sensor now is completely learned and ready to instantiate
        sensor.createImpl(objectServer, dbusConnection);

        std::cerr << "Found " << sensor.configName << " at "
                  << sensor.redfishPath << "\n";

        ++countDiscovered;
    }

    // Now that sensors are sorted, see which MetricReports they go with
    for (auto& metricReportPair : pathsToMetricReports)
    {
        RedfishMetricReport& metricReport = *(metricReportPair.second);

        metricReport.isHelpful = false;

        int sensorsIncluded = 0;
        int reportsPresent =
            checkMetricReport(metricReport.reportCache, sensorsIncluded);

        if constexpr (debug)
        {
            std::cerr << "Checking " << metricReport.reportPath
                      << " report: " << reportsPresent << " readings, "
                      << sensorsIncluded << " relevant\n";
        }

        if (sensorsIncluded < 1)
        {
            std::cerr << "Report " << metricReport.reportPath
                      << " not relevant for any of our sensors\n";
            continue;
        }
        if (reportsPresent < 1)
        {
            std::cerr << "Report " << metricReport.reportPath
                      << " not usable with our sensors\n";
            continue;
        }

        // This report will help us accelerate sensor data collection
        metricReport.isHelpful = true;

        if constexpr (debug)
        {
            std::cerr << "Report " << metricReport.reportPath
                      << " considered useful\n";
        }
    }

    size_t countBad =
        countNotRelevant + countAmbiguous + countNotFound + countIncomplete;

    std::cerr << "Server communicating: " << countDiscovered
              << " sensors found, " << countAlready << " previous, " << countBad
              << " failed\n";
    if (countBad > 0)
    {
        std::cerr << "Of those that failed: " << countNotRelevant
                  << " config problem, " << countAmbiguous << " ambiguous, "
                  << countNotFound << " not found, " << countIncomplete
                  << " server problem\n";
    }
}

void RedfishServer::readSensors()
{
    // Begin new reading cycle by marking all as not collected yet
    for (auto& metricReportPair : pathsToMetricReports)
    {
        metricReportPair.second->isCollected = false;
    }
    for (auto& sensorPair : pathsToSensors)
    {
        sensorPair.second->isCollected = false;
    }

    advanceReading();
}

void RedfishServer::advanceReading()
{
    // This can be called multiple times per tick
    ++actionsReading;

    // First, gather all MetricReports that are known to be helpful
    for (const auto& metricReportPair : pathsToMetricReports)
    {
        RedfishMetricReport& metricReport = *(metricReportPair.second);

        if (metricReport.isCollected)
        {
            // Already collected
            continue;
        }

        if (!(metricReport.isHelpful))
        {
            // This report would not be helpful to us, even if collected
            continue;
        }

        collectMetricReport(metricReport.reportPath);
        return;
    }

    // Second, gather individual Sensor objects not included in reports
    for (const auto& sensorPair : pathsToSensors)
    {
        RedfishSensor& sensor = *(sensorPair.second);

        if (sensor.isCollected)
        {
            // Already collected
            continue;
        }

        if (!(sensor.isRelevant))
        {
            // This sensor was already deemed not relevant to us
            continue;
        }

        if (!(sensor.impl))
        {
            // This sensor never was successfully instantiated
            continue;
        }

        collectSensorReading(sensor.redfishPath);
        return;
    }

    // At this point, nothing more to read, done with this cycle
    auto now = std::chrono::steady_clock::now();
    auto msTaken =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - timeReading)
            .count();

    // Makes it easy to compute average time per reading cycle
    msReadingTime += msTaken;
    ++completionsReading;

    if constexpr (debug)
    {
        double average = static_cast<double>(msReadingTime);
        average /= static_cast<double>(completionsReading);
        auto msAverage = static_cast<int>(average);

        std::cerr << "Reading done, " << msTaken << " ms, " << msAverage
                  << " ms average\n";
    }
}
