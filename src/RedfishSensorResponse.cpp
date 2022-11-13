#include <RedfishSensor.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <string>
#include <vector>

static constexpr bool debug = false;

// FUTURE: Add constexpr here, when allowed by compiler in future
static RedfishUnitLookup unitLookup{
    .units{// The min and max are arbitrary, used only if Redfish info absent
           // All these divide by 255.0 into nice IPMI-friendly increments
           {"Altitude", "Pa", "Pascals", 0, 127500},
           {"Current", "A", "Amperes", 0, 51},
           {"EnergyJoules", "J", "Joules", 0, 5100000},
           {"Percent", "%", "Percent", 0, 127.5},
           {"Power", "W", "Watts", 0, 5100},
           {"Rotational", "RPM", "RPMS", 0, 51000},
           {"Temperature", "Cel", "DegreesC", -128, 127},
           {"Voltage", "V", "Volts", 0, 510},
           {"", "", "", 0, 0}}};

// FUTURE: Add constexpr here, when allowed by compiler in future
static RedfishThresholdLookup thresholdLookup{.thresholds{
    // As there is no Redfish equivalent to D-Bus PERFORMANCELOSS
    // or HARDSHUTDOWN levels, they intentionally do not appear here
    {"UpperCaution", thresholds::Level::WARNING, thresholds::Direction::HIGH},
    {"LowerCaution", thresholds::Level::WARNING, thresholds::Direction::LOW},
    {"UpperCritical", thresholds::Level::CRITICAL, thresholds::Direction::HIGH},
    {"LowerCritical", thresholds::Level::CRITICAL, thresholds::Direction::LOW},
    {"UpperFatal", thresholds::Level::SOFTSHUTDOWN,
     thresholds::Direction::HIGH},
    {"LowerFatal", thresholds::Level::SOFTSHUTDOWN, thresholds::Direction::LOW},
    {"", thresholds::Level::ERROR, thresholds::Direction::ERROR}}};

static std::string fillFromJsonString(const std::string& name,
                                      const nlohmann::json& json)
{
    std::string empty;

    // Must be an object
    if (!(json.is_object()))
    {
        return empty;
    }

    // The given field name must be contained within the object
    auto iter = json.find(name);
    if (iter == json.end())
    {
        return empty;
    }

    // The field value must be a string
    if (!(iter->is_string()))
    {
        return empty;
    }

    std::string result = *iter;

    if constexpr (debug)
    {
        std::cerr << "Parsed string: name " << name << ", value " << result
                  << "\n";
    }

    return result;
}

static std::string fillFromJsonPath(const std::string& name,
                                    const nlohmann::json& json)
{
    std::string empty;

    // Must be an object
    if (!(json.is_object()))
    {
        return empty;
    }

    // The given field name must be contained within the object
    auto iter = json.find(name);
    if (iter == json.end())
    {
        return empty;
    }

    // The field value must be an object
    if (!(iter->is_object()))
    {
        return empty;
    }

    // The object must contain this hardcoded field name
    auto subiter = iter->find("@odata.id");
    if (subiter == iter->end())
    {
        return empty;
    }

    // The field value must be a string
    if (!(subiter->is_string()))
    {
        return empty;
    }

    std::string result = *subiter;

    if constexpr (debug)
    {
        std::cerr << "Parsed path: name " << name << ", value " << result
                  << "\n";
    }

    return result;
}

// Can have successful NaN returns, such as for a literal string "nan"
static double stringToNumber(const std::string& text, bool& outSuccessful)
{
    outSuccessful = false;

    double result = std::numeric_limits<double>::quiet_NaN();

    // FUTURE: There should be a non-throwing equivalent of std::stod
    try
    {
        result = std::stod(text);

        outSuccessful = true;
    }
    catch (const std::exception& e)
    {
        result = std::numeric_limits<double>::quiet_NaN();

        std::cerr << "Problem converting string to number: " << e.what()
                  << "\n";
    }

    if constexpr (debug)
    {
        std::cerr << "Converted string " << text << " to number " << result
                  << "\n";
    }

    return result;
}

static double fillFromJsonNumber(const std::string& name,
                                 const nlohmann::json& json,
                                 bool& outSuccessful)
{
    outSuccessful = false;

    double nan = std::numeric_limits<double>::quiet_NaN();

    // Must be an object
    if (!(json.is_object()))
    {
        return nan;
    }

    // The given field name must be contained within the object
    auto iter = json.find(name);
    if (iter == json.end())
    {
        return nan;
    }

    // JSON allows numbers to be "null" to indicate not available
    if (iter->is_null())
    {
        // This is actually considered a successful return
        outSuccessful = true;

        // FUTURE: Add more audits, and chalk this successful, not failure
        return nan;
    }

    double result;

    // Some Redfish servers wrap their numbers in strings
    if (iter->is_string())
    {
        bool successful = false;

        result = stringToNumber(*iter, successful);

        if (!successful)
        {
            return nan;
        }
    }
    else
    {
        if (iter->is_number())
        {
            result = *iter;
        }
        else
        {
            // Unsuccessful result, unrecognized type
            return nan;
        }
    }

    if constexpr (debug)
    {
        std::cerr << "Parsed number: name " << name << ", value " << result
                  << "\n";
    }

    // OK to return NaN, intentionally not testing isfinite(result) here
    outSuccessful = true;

    return result;
}

static std::vector<std::string> fillFromJsonMembers(const nlohmann::json& json,
                                                    bool& outSuccessful)
{
    std::vector<std::string> empty;

    outSuccessful = false;

    // Must be an object
    if (!(json.is_object()))
    {
        return empty;
    }

    // The object must contain this hardcoded field name
    auto iter = json.find("Members");
    if (iter == json.end())
    {
        return empty;
    }

    // The field value must be an array
    if (!(iter->is_array()))
    {
        return empty;
    }

    // FUTURE: For an additional sanity check, we could grab
    // Members@odata.count here, and compare it with the actual
    // count, to catch malformed Redfish replies.
    size_t count = iter->size();

    std::vector<std::string> result;

    for (size_t i = 0; i < count; ++i)
    {
        // The element must be an object
        if (!((*iter)[i].is_object()))
        {
            return empty;
        }

        // The object must contain this hardcoded field name
        auto subiter = (*iter)[i].find("@odata.id");
        if (subiter == (*iter)[i].end())
        {
            return empty;
        }

        // The field value must be a string
        if (!(subiter->is_string()))
        {
            return empty;
        }

        std::string value = *subiter;

        // The field value must not be empty
        if (value.empty())
        {
            return empty;
        }

        result.emplace_back(value);

        if constexpr (debug)
        {
            std::cerr << "Parsed array[" << i << "]: value " << value << "\n";
        }
    }

    // Needed to indicate successful return of a collection of zero elements
    outSuccessful = true;

    return result;
}

static std::vector<thresholds::Threshold>
    parseThresholds(const nlohmann::json& json, bool& outSuccessful)
{
    std::vector<thresholds::Threshold> empty;
    std::vector<thresholds::Threshold> results;

    outSuccessful = false;

    // It must be an object
    if (!(json.is_object()))
    {
        return empty;
    }

    for (const auto& [key, val] : json.items())
    {
        // Value must be a JSON dictionary
        if (!(val.is_object()))
        {
            std::cerr << "Skipping over threshold with unparseable value: "
                      << key << "\n";
            continue;
        }

        bool successful = false;
        double thresholdValue = fillFromJsonNumber("Reading", val, successful);

        if (!successful)
        {
            std::cerr << "Skipping over threshold with unrecognizable value: "
                      << key << "\n";
            continue;
        }

        // Not fatal error, some Redfish servers serve "null" values
        if (!(std::isfinite(thresholdValue)))
        {
            std::cerr << "Skipping over threshold with unusable value: " << key
                      << "\n";
            continue;
        }

        // Not fatal error, skip over unrecognized field names
        std::string nameCheck = thresholdLookup.lookup(key).redfishName;
        if (nameCheck.empty())
        {
            std::cerr << "Skipping over threshold with unrecognizable name: "
                      << key << "\n";
            continue;
        }

        // Name preflight is good, now do the real lookups
        thresholds::Level dbusLevel = thresholdLookup.lookup(key).level;
        thresholds::Direction dbusDirection =
            thresholdLookup.lookup(key).direction;

        results.emplace_back(dbusLevel, dbusDirection, thresholdValue);

        if constexpr (debug)
        {
            std::cerr << "Added threshold: type " << nameCheck << ", value "
                      << thresholdValue << "\n";
        }
    }

    // If made it here, parsing successful, even if zero results
    outSuccessful = true;

    return results;
}

bool RedfishChassisMatcher::fillFromJson(const nlohmann::json& json)
{
    clear();

    redfishName = fillFromJsonString("Name", json);
    redfishId = fillFromJsonString("Id", json);

    manufacturer = fillFromJsonString("Manufacturer", json);
    model = fillFromJsonString("Model", json);
    partNumber = fillFromJsonString("PartNumber", json);
    sku = fillFromJsonString("SKU", json);
    serialNumber = fillFromJsonString("SerialNumber", json);
    sparePartNumber = fillFromJsonString("SparePartNumber", json);
    version = fillFromJsonString("Version", json);

    // At least one characteristic must have been filled in
    if (isEmpty())
    {
        std::cerr << "Trouble parsing Chassis characteristics from JSON\n";

        return false;
    }

    return true;
}

bool RedfishSensorMatcher::fillFromJson(const nlohmann::json& json)
{
    clear();

    redfishName = fillFromJsonString("Name", json);
    redfishId = fillFromJsonString("Id", json);

    // At least one characteristic must have been filled in
    if (isEmpty())
    {
        std::cerr << "Trouble parsing Sensor characteristics from JSON\n";

        return false;
    }

    return true;
}

bool RedfishServer::fillFromRoot(const nlohmann::json& json)
{
    pathChassis.clear();
    pathTelemetry.clear();

    pathChassis = fillFromJsonPath("Chassis", json);
    pathTelemetry = fillFromJsonPath("TelemetryService", json);

    // Chassis is mandatory, but Telemetry is optional
    if (pathChassis.empty())
    {
        std::cerr << "Trouble parsing JSON from Redfish root\n";

        return false;
    }

    return true;
}

bool RedfishServer::fillFromTelemetry(const nlohmann::json& json)
{
    pathMetrics.clear();

    pathMetrics = fillFromJsonPath("MetricReports", json);

    if (pathMetrics.empty())
    {
        std::cerr << "Trouble parsing JSON from Redfish Telemetry\n";

        return false;
    }

    return true;
}

bool RedfishServer::fillFromMetricCollection(const nlohmann::json& json)
{
    reportPaths.clear();

    bool successful = false;
    reportPaths = fillFromJsonMembers(json, successful);

    // It is perfectly OK to successfully read a collection of zero members
    if (successful)
    {
        if constexpr (debug)
        {
            std::cerr << "Server " << configName << " contains "
                      << reportPaths.size() << " reports\n";
        }
    }
    else
    {
        std::cerr << "Trouble parsing JSON from Redfish MetricCollection\n";
    }

    haveReportPaths = successful;

    return successful;
}

bool RedfishServer::fillFromMetricReport(const nlohmann::json& json)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    std::string reportPath = fillFromJsonString("@odata.id", json);
    if (reportPath.empty())
    {
        return false;
    }

    int sensorsMatched = 0;
    int reportsIncluded = checkMetricReport(json, sensorsMatched);

    // At this early stage, there will be no sensors matched, but we want
    // to validate the JSON and make sure it was fetched without error.
    if (reportsIncluded < 0)
    {
        return false;
    }

    auto newReport = std::make_shared<RedfishMetricReport>();

    newReport->reportPath = reportPath;
    newReport->isHelpful = false;
    newReport->isCollected = false;
    newReport->reportCache = json;

    auto iter = pathsToMetricReports.find(reportPath);
    if (iter != pathsToMetricReports.end())
    {
        iter->second.reset();
    }

    pathsToMetricReports[reportPath] = newReport;

    if constexpr (debug)
    {
        std::cerr << "Parsed report: " << reportPath << ", " << reportsIncluded
                  << " reports, " << sensorsMatched << " early sensors\n";
    }

    return true;
}

bool RedfishServer::fillFromChassisCollection(const nlohmann::json& json)
{
    chassisPaths.clear();

    bool successful = false;
    chassisPaths = fillFromJsonMembers(json, successful);

    // It is perfectly OK to successfully read a collection of zero members
    if (successful)
    {
        if constexpr (debug)
        {
            std::cerr << "Server " << configName << " contains "
                      << chassisPaths.size() << " chassises\n";
        }
    }
    else
    {
        std::cerr << "Trouble parsing JSON from Redfish ChassisCollection\n";
    }

    haveChassisPaths = successful;

    return successful;
}

bool RedfishServer::fillFromChassisCandidate(const nlohmann::json& json,
                                             const std::string& path)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    std::string chassisPath = fillFromJsonString("@odata.id", json);
    if (chassisPath.empty())
    {
        return false;
    }
    if (chassisPath != path)
    {
        std::cerr << "Redfish path mismatch: " << path << " expected, "
                  << chassisPath << " received\n";

        return false;
    }

    bool isAcceptable = true;

    // It is OK for a chassis to not support sensors
    std::string sensorsPath = fillFromJsonPath("Sensors", json);
    if (sensorsPath.empty())
    {
        if constexpr (debug)
        {
            std::cerr << "Redfish chassis " << chassisPath
                      << " does not support sensors\n";
        }
        isAcceptable = false;
    }

    RedfishChassisMatcher characteristics;

    // It is OK for a chassis to not have anything we are looking for
    if (!(characteristics.fillFromJson(json)))
    {
        if constexpr (debug)
        {
            std::cerr << "Redfish chassis " << chassisPath
                      << " does not have characteristics\n";
        }
        isAcceptable = false;
    }
    if (characteristics.isEmpty())
    {
        if constexpr (debug)
        {
            std::cerr << "Redfish chassis " << chassisPath
                      << " has no distinguishing characteristics\n";
        }
        isAcceptable = false;
    }

    auto newCandidate = std::make_shared<RedfishChassisCandidate>();

    newCandidate->chassisPath = chassisPath;
    newCandidate->sensorsPath = sensorsPath;
    newCandidate->characteristics = characteristics;
    newCandidate->isAcceptable = isAcceptable;

    auto iter = pathsToChassisCandidates.find(chassisPath);
    if (iter != pathsToChassisCandidates.end())
    {
        iter->second.reset();
    }

    pathsToChassisCandidates[chassisPath] = newCandidate;

    if constexpr (debug)
    {
        std::cerr << "Parsed chassis candidate: " << chassisPath << ", "
                  << (isAcceptable ? "acceptable" : "disqualified") << "\n";
    }

    return true;
}

bool RedfishServer::fillFromSensorCollection(const nlohmann::json& json,
                                             const std::string& path)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    std::string sensorCollectionPath = fillFromJsonString("@odata.id", json);
    if (sensorCollectionPath.empty())
    {
        return false;
    }
    if (sensorCollectionPath != path)
    {
        std::cerr << "Redfish path mismatch: " << path << " expected, "
                  << sensorCollectionPath << " received\n";

        return false;
    }

    bool successful = false;
    std::vector<std::string> sensorPaths =
        fillFromJsonMembers(json, successful);

    // It is perfectly OK to successfully read a collection of zero members
    if (!successful)
    {
        std::cerr << "Trouble parsing JSON from Redfish SensorCollection\n";

        return false;
    }

    size_t numPathMatches = 0;
    std::shared_ptr<RedfishChassisCandidate> candidatePtr;

    // This sensor list must appear within exactly one chassis candidate
    for (const auto& chassisPair : pathsToChassisCandidates)
    {
        if (!(chassisPair.second->isAcceptable))
        {
            continue;
        }

        if (chassisPair.second->sensorsPath == sensorCollectionPath)
        {
            ++numPathMatches;
            candidatePtr = chassisPair.second;

            if constexpr (debug)
            {
                std::cerr << "Chassis " << candidatePtr->chassisPath
                          << " contains sensors: " << sensorCollectionPath
                          << "\n";
            }
        }
    }

    if (numPathMatches != 1)
    {
        std::cerr << "Trouble matching up SensorCollection with Chassis: "
                  << sensorCollectionPath << "\n";

        return false;
    }

    // Now that we know which candidate it was, can finally store results
    candidatePtr->sensorPaths = sensorPaths;
    candidatePtr->haveSensorPaths = successful;

    if constexpr (debug)
    {
        std::cerr << "Chassis " << candidatePtr->chassisPath << " has "
                  << sensorPaths.size() << " sensors\n";
    }

    // A chassis with zero sensors is valid Redfish but uninteresting to us
    if (sensorPaths.empty())
    {
        candidatePtr->isAcceptable = false;

        if constexpr (debug)
        {
            std::cerr << "Redfish chassis " << candidatePtr->chassisPath
                      << " has no sensors\n";
        }
    }

    return successful;
}

bool RedfishServer::fillFromSensorCandidate(const nlohmann::json& json,
                                            const std::string& path)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    std::string sensorPath = fillFromJsonString("@odata.id", json);
    if (sensorPath.empty())
    {
        return false;
    }
    if (sensorPath != path)
    {
        std::cerr << "Redfish path mismatch: " << path << " expected, "
                  << sensorPath << " received\n";

        return false;
    }

    bool isAcceptable = true;

    RedfishSensorMatcher characteristics;

    // It is OK for a sensor to not have anything we are looking for
    if (!(characteristics.fillFromJson(json)))
    {
        std::cerr << "Redfish sensor " << sensorPath
                  << " does not have characteristics\n";
        isAcceptable = false;
    }
    if (characteristics.isEmpty())
    {
        std::cerr << "Redfish sensor " << sensorPath
                  << " has no distinguishing characteristics\n";
        isAcceptable = false;
    }

    auto newCandidate = std::make_shared<RedfishSensorCandidate>();

    newCandidate->sensorPath = sensorPath;
    newCandidate->characteristics = characteristics;
    newCandidate->isAcceptable = isAcceptable;
    newCandidate->readingCache = json;

    size_t numPathMatches = 0;
    std::shared_ptr<RedfishChassisCandidate> candidatePtr;

    // This sensor path must appear within exactly one chassis candidate
    for (const auto& chassisPair : pathsToChassisCandidates)
    {
        if (!(chassisPair.second->isAcceptable))
        {
            continue;
        }

        for (const std::string& sensorCandidatePath :
             chassisPair.second->sensorPaths)
        {
            if (sensorPath == sensorCandidatePath)
            {
                ++numPathMatches;
                candidatePtr = chassisPair.second;

                if constexpr (debug)
                {
                    std::cerr << "Chassis " << candidatePtr->chassisPath
                              << " has sensor " << sensorPath << "\n";
                }
            }
        }
    }

    if (numPathMatches != 1)
    {
        std::cerr << "Trouble matching up Sensor with Chassis: " << sensorPath
                  << "\n";

        return false;
    }

    auto iter = candidatePtr->pathsToSensorCandidates.find(sensorPath);
    if (iter != candidatePtr->pathsToSensorCandidates.end())
    {
        iter->second.reset();
    }

    candidatePtr->pathsToSensorCandidates[sensorPath] = newCandidate;

    if constexpr (debug)
    {
        std::cerr << "Parsed sensor candidate: " << sensorPath << ", chassis "
                  << candidatePtr->chassisPath << ", "
                  << (isAcceptable ? "acceptable" : "disqualified") << "\n";
    }

    return true;
}

bool RedfishServer::fillFromSensorAccepted(const nlohmann::json& json)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    // The object must contain this hardcoded field name
    std::string sensorPath = fillFromJsonString("@odata.id", json);
    if (sensorPath.empty())
    {
        return false;
    }

    // Discovery must already have ran, and filled this in by now
    auto iterPair = pathsToSensors.find(sensorPath);
    if (iterPair == pathsToSensors.end())
    {
        return false;
    }

    RedfishSensor& sensor = *(iterPair->second);

    // The object must contain at least one of these two ways to define units
    std::string readingUnits = fillFromJsonString("ReadingUnits", json);
    std::string readingType = fillFromJsonString("ReadingType", json);

    std::string goodLookupSource;
    std::string dbusByUnits;
    std::string dbusByType;

    if (!(readingUnits.empty()))
    {
        dbusByUnits = unitLookup.lookup(readingUnits).dbusUnits;
        if (!(dbusByUnits.empty()))
        {
            goodLookupSource = dbusByUnits;
        }
    }
    if (!(readingType.empty()))
    {
        dbusByType = unitLookup.lookup(readingType).dbusUnits;
        if (!(dbusByType.empty()))
        {
            goodLookupSource = dbusByType;
        }
    }

    // At least one must have been resolved
    if (goodLookupSource.empty())
    {
        std::cerr << "Sensor " << sensor.configName
                  << " Redfish units missing\n";
        return false;
    }

    // If only one was resolved, we take whatever we could get
    if (dbusByUnits.empty() || dbusByType.empty())
    {
        if constexpr (debug)
        {
            std::cerr << "Sensor " << sensor.configName
                      << " Redfish units partially specified but still OK: "
                      << goodLookupSource << "\n";
        }
    }
    else
    {
        // If both were resolved, both must agree
        if (dbusByUnits != dbusByType)
        {
            std::cerr << "Sensor " << sensor.configName
                      << " Redfish units discrepancy: " << dbusByUnits << " vs "
                      << dbusByType << "\n";
            return false;
        }
    }

    // Confirmed good lookup source and preflight, now do the real lookups
    std::string dbusUnit = unitLookup.lookup(goodLookupSource).dbusUnits;
    double defaultMin = unitLookup.lookup(goodLookupSource).rangeMin;
    double defaultMax = unitLookup.lookup(goodLookupSource).rangeMax;

    // These are optional, but if specified, both must be specified
    bool successfulMin = false;
    bool successfulMax = false;
    double rangeMin =
        fillFromJsonNumber("ReadingRangeMin", json, successfulMin);
    double rangeMax =
        fillFromJsonNumber("ReadingRangeMax", json, successfulMax);

    if (successfulMin && successfulMax && std::isfinite(rangeMin) &&
        std::isfinite(rangeMax))
    {
        if constexpr (debug)
        {
            std::cerr << "Sensor " << sensor.configName
                      << " Redfish range: min " << rangeMin << ", max "
                      << rangeMax << "\n";
        }
    }
    else
    {
        rangeMin = defaultMin;
        rangeMax = defaultMax;
    }

    if (!(rangeMin < rangeMax))
    {
        std::cerr << "Sensor " << sensor.configName
                  << " Redfish range unusable: min " << rangeMin << ", max "
                  << rangeMax << "\n";
        return false;
    }

    std::vector<thresholds::Threshold> thresholds;

    auto findThresholds = json.find("Thresholds");
    if (findThresholds != json.end())
    {
        bool successful = false;
        thresholds = parseThresholds(*findThresholds, successful);

        if (successful)
        {
            if constexpr (debug)
            {
                std::cerr << "Sensor " << sensor.configName << " has "
                          << thresholds.size() << " thresholds\n";
            }
        }
        else
        {
            std::cerr << "Trouble parsing Redfish thresholds: "
                      << sensor.configName << "\n";
            return false;
        }
    }

    if constexpr (debug)
    {
        std::cerr << "Sensor " << sensor.configName
                  << " successfully parsed from " << sensorPath << "\n";
    }

    // Stuff in what we have learned from the Redfish sensor object
    // The matcher must have already filled in redfishPath and chassisPath
    sensor.units = dbusUnit;
    sensor.minValue = rangeMin;
    sensor.maxValue = rangeMax;
    sensor.thresholds = thresholds;

    return true;
}

bool RedfishServer::acceptSensorReading(
    const nlohmann::json& json,
    const std::chrono::steady_clock::time_point& now)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    // The object must contain this hardcoded field name
    std::string sensorPath = fillFromJsonString("@odata.id", json);
    if (sensorPath.empty())
    {
        return false;
    }

    // The value must be present, even if it is NaN
    bool successful = false;
    double sensorValue = fillFromJsonNumber("Reading", json, successful);
    if (!successful)
    {
        return false;
    }

    // The sensor must have made it through discovery
    auto iterPair = pathsToSensors.find(sensorPath);
    if (iterPair == pathsToSensors.end())
    {
        return false;
    }

    RedfishSensor& sensor = *(iterPair->second);

    // The sensor must have successfully been instantiated
    if (!(sensor.impl))
    {
        return false;
    }

    // Sensor found, add good reading from this report
    // OK if NaN, intentionally not testing isfinite(sensorValue) here
    sensor.readingValue = sensorValue;
    sensor.readingWhen = now;
    sensor.isCollected = true;
    sensor.impl->updateValue(sensor.readingValue);
    ++readingsGoodIndividual;

    if constexpr (debug)
    {
        std::cerr << "Accepted individual reading: sensor " << sensor.configName
                  << " = " << sensorValue << "\n";
    }

    return true;
}

// This is fast path, code should be optimized here, run as fast as possible
bool RedfishServer::acceptMetricReport(
    const nlohmann::json& json,
    const std::chrono::steady_clock::time_point& now)
{
    // Must be an object
    if (!(json.is_object()))
    {
        return false;
    }

    // The object must contain this hardcoded field name
    std::string reportPath = fillFromJsonString("@odata.id", json);
    if (reportPath.empty())
    {
        return false;
    }

    // The object must contain this hardcoded field name
    auto iterValues = json.find("MetricValues");
    if (iterValues == json.end())
    {
        return false;
    }

    // The field value must be an array
    if (!(iterValues->is_array()))
    {
        return false;
    }

    // This should already have been filled in during discovery
    auto iterReport = pathsToMetricReports.find(reportPath);
    if (iterReport == pathsToMetricReports.end())
    {
        return false;
    }

    // Unlike Members, this JSON contains no corresponding "@odata.count"
    size_t count = iterValues->size();

    // FUTURE: Consider keeping track of badReadings also
    size_t goodReadings = 0;

    for (size_t i = 0; i < count; ++i)
    {
        const auto& element = (*iterValues)[i];

        // The element must be an object
        if (!(element.is_object()))
        {
            continue;
        }

        // This indicates the sensor whose reading is being supplied here
        std::string sensorPath = fillFromJsonString("MetricProperty", element);
        if (sensorPath.empty())
        {
            continue;
        }

        // The value must be present, even if it is NaN
        bool successful = false;
        double sensorValue =
            fillFromJsonNumber("MetricValue", element, successful);
        if (!successful)
        {
            continue;
        }

        // This should already have been filled in during discovery
        auto iterPair = pathsToSensors.find(sensorPath);
        if (iterPair == pathsToSensors.end())
        {
            continue;
        }

        RedfishSensor& sensor = *(iterPair->second);

        // The sensor must have successfully been instantiated
        if (!(sensor.impl))
        {
            continue;
        }

        // Sensor found, add good reading from this report
        // OK if NaN, intentionally not testing isfinite(sensorValue) here
        sensor.readingValue = sensorValue;
        sensor.readingWhen = now;
        sensor.isCollected = true;
        sensor.impl->updateValue(sensor.readingValue);
        ++readingsGoodReported;

        if constexpr (debug)
        {
            std::cerr << "Accepted report[" << i << "]: sensor "
                      << sensor.configName << " = " << sensorValue << "\n";
        }

        ++goodReadings;
    }

    // Mark this report as having been successfully collected
    iterReport->second->isCollected = true;

    ++readingsReports;

    if constexpr (debug)
    {
        std::cerr << "Accepted report: " << goodReadings << " good readings\n";
    }

    // It is good if we got this far, even if zero readings
    return true;
}

// Used twice during Discovery, to validate existence, then to see if relevant
int RedfishServer::checkMetricReport(const nlohmann::json& json,
                                     int& outMatched)
{
    outMatched = 0;

    // Must be an object
    if (!(json.is_object()))
    {
        return -1;
    }

    // OK during preflight for report to not be in pathsToMetricReports
    // The object must contain this hardcoded field name
    auto iterValues = json.find("MetricValues");
    if (iterValues == json.end())
    {
        return -1;
    }

    // The field value must be an array
    if (!(iterValues->is_array()))
    {
        return -1;
    }

    // Unlike Members, this JSON contains no corresponding "@odata.count"
    size_t count = iterValues->size();
    int goodChecks = 0;

    for (size_t i = 0; i < count; ++i)
    {
        const auto& element = (*iterValues)[i];

        // The element must be an object
        if (!(element.is_object()))
        {
            continue;
        }

        // This indicates the sensor whose reading is being supplied here
        std::string sensorPath = fillFromJsonString("MetricProperty", element);
        if (sensorPath.empty())
        {
            continue;
        }

        // The value must be present in the JSON, even if it is NaN
        // This is only a preflight, do not store the returned value
        bool successful = false;
        fillFromJsonNumber("MetricValue", element, successful);
        if (!successful)
        {
            continue;
        }

        // This report looks like it would be useful for a sensor
        ++goodChecks;

        // OK to not find any during preflight, as pathsToSensors not yet set
        auto iterPair = pathsToSensors.find(sensorPath);
        if (iterPair == pathsToSensors.end())
        {
            continue;
        }

        // This report will be helpful to at least one of our sensors
        // Do nothing more with it, as this is just a preflight
        if constexpr (debug)
        {
            // Redo the fill, because returned value now useful for debugging
            double sensorValue =
                fillFromJsonNumber("MetricValue", element, successful);

            std::cerr << "Checked report[" << i << "]: sensor "
                      << iterPair->second->configName << " = " << sensorValue
                      << "\n";
        }

        ++outMatched;
    }

    if constexpr (debug)
    {
        std::cerr << "Checked report: " << goodChecks << " readings, "
                  << outMatched << " matched sensors\n";
    }

    return goodChecks;
}
