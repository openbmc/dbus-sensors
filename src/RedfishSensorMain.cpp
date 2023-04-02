#include <RedfishSensor.hpp>
#include <Utils.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <string>
#include <vector>

static constexpr bool debug = false;

// Default network protocol
static constexpr auto defaultProtocol = "http";

// Default port number will be looked up dynamically by protocol
static constexpr int defaultPort = -1;

static constexpr auto sensorTypes{std::to_array<const char*>(
    {"RedfishSensor", "RedfishChassis", "RedfishServer"})};

class Globals
{
  public:
    // Common globals for communication to the outside world
    std::shared_ptr<boost::asio::io_context> ioContext;
    std::shared_ptr<sdbusplus::asio::connection> systemBus;
    std::shared_ptr<sdbusplus::asio::object_server> objectServer;

    // Data structures parsed from config
    boost::container::flat_map<std::string, std::shared_ptr<RedfishServer>>
        serversConfig;
    boost::container::flat_map<std::string, std::shared_ptr<RedfishChassis>>
        chassisesConfig;
    boost::container::flat_map<std::string, std::shared_ptr<RedfishSensor>>
        sensorsConfig;

    Globals() :
        ioContext(std::make_shared<boost::asio::io_context>()),
        systemBus(std::make_shared<sdbusplus::asio::connection>(*ioContext)),
        objectServer(
            std::make_shared<sdbusplus::asio::object_server>(systemBus, true))
    {
        objectServer->add_manager("/xyz/openbmc_project/sensors");
        systemBus->request_name("xyz.openbmc_project.RedfishSensor");
    }

    Globals(const Globals& copy) = delete;
    Globals& operator=(const Globals& assign) = delete;
    ~Globals() = default;
};

// Validate each sensor has a link to a named Chassis and Server
void validateCreation(Globals& globals)
{
    // Mark all config objects as not relevant, until validated
    for (auto& serverPair : globals.serversConfig)
    {
        serverPair.second->isRelevant = false;
    }
    for (auto& chassisPair : globals.chassisesConfig)
    {
        chassisPair.second->isRelevant = false;
    }
    for (auto& sensorPair : globals.sensorsConfig)
    {
        sensorPair.second->isRelevant = false;
    }

    if constexpr (debug)
    {
        std::cerr << "Before validation: " << globals.serversConfig.size()
                  << " servers, " << globals.chassisesConfig.size()
                  << " chassises, " << globals.sensorsConfig.size()
                  << " sensors\n";
    }

    // Remove, on all servers, links back to sensors
    for (auto& serverPair : globals.serversConfig)
    {
        for (auto& sensorPtr : serverPair.second->sensorsServed)
        {
            sensorPtr.reset();
        }
        serverPair.second->sensorsServed.clear();
    }

    // Remove, on all chassises, links back to sensors
    for (auto& chassisPair : globals.chassisesConfig)
    {
        for (auto& sensorPtr : chassisPair.second->sensorsContained)
        {
            sensorPtr.reset();
        }
        chassisPair.second->sensorsContained.clear();
    }

    size_t relevantServers = 0;
    size_t relevantChassises = 0;
    size_t relevantSensors = 0;

    for (auto& sensorPair : globals.sensorsConfig)
    {
        RedfishSensor& sensor = *(sensorPair.second);

        // Remove links going the other way, links out from sensors
        sensor.server.reset();
        sensor.chassis.reset();

        auto foundServer = globals.serversConfig.find(sensor.configServer);
        if (foundServer == globals.serversConfig.end())
        {
            std::cerr << "Sensor " << sensor.configName << " has server "
                      << sensor.configServer << " not found in configuration\n";
            continue;
        }

        auto foundChassis = globals.chassisesConfig.find(sensor.configChassis);
        if (foundChassis == globals.chassisesConfig.end())
        {
            std::cerr << "Sensor " << sensor.configName << " has chassis "
                      << sensor.configChassis
                      << " not found in configuration\n";
            continue;
        }

        // Sensor looks good, repopulate links going out from it
        sensor.server = foundServer->second;
        sensor.chassis = foundChassis->second;

        // Correspondingly repopulate links back to this sensor
        foundServer->second->sensorsServed.emplace_back(sensorPair.second);
        foundChassis->second->sensorsContained.emplace_back(sensorPair.second);

        // All these objects confirmed relevant to each other
        ++relevantSensors;
        foundServer->second->isRelevant = true;
        foundChassis->second->isRelevant = true;
        sensor.isRelevant = true;
    }

    // Show me what you got
    for (auto& serverPair : globals.serversConfig)
    {
        if (serverPair.second->isRelevant)
        {
            ++relevantServers;
            if constexpr (debug)
            {
                std::cerr << "Server " << serverPair.first << " has "
                          << serverPair.second->sensorsServed.size()
                          << " sensors\n";
            }
        }
        else
        {
            std::cerr << "Server " << serverPair.first
                      << " is not relevant to sensor configuration\n";
        }
    }

    for (auto& chassisPair : globals.chassisesConfig)
    {
        if (chassisPair.second->isRelevant)
        {
            ++relevantChassises;
            if constexpr (debug)
            {
                std::cerr << "Chassis " << chassisPair.first << " has "
                          << chassisPair.second->sensorsContained.size()
                          << " sensors\n";
            }
        }
        else
        {
            std::cerr << "Chassis " << chassisPair.first
                      << " is not relevant to sensor configuration\n";
        }
    }

    if (relevantSensors > 0)
    {
        std::cerr << "Configuration accepted: " << relevantServers
                  << " servers, " << relevantChassises << " chassises, "
                  << relevantSensors << " sensors\n";
    }
}

bool fillConfigString(const SensorBaseConfigMap& baseConfigMap,
                      const std::string& interfacePath,
                      const std::string& paramName, bool isMandatory,
                      std::string& paramValue)
{
    auto paramFound = baseConfigMap.find(paramName);
    if (paramFound == baseConfigMap.end())
    {
        if (isMandatory)
        {
            std::cerr << paramName << " parameter not found in "
                      << interfacePath << "\n";
            return false;
        }

        if constexpr (debug)
        {
            std::cerr << paramName << " optional parameter not given in "
                      << interfacePath << "\n";
        }
        return true;
    }

    try
    {
        paramValue = std::visit(VariantToStringVisitor(), paramFound->second);
    }
    catch (const std::exception& e)
    {
        std::cerr << paramName << " parameter not parsed in " << interfacePath
                  << ": " << e.what() << "\n";
        return false;
    }

    if (paramValue.empty())
    {
        std::cerr << paramName << " parameter not acceptable in "
                  << interfacePath << "\n";
        return false;
    }

    if constexpr (debug)
    {
        std::cerr << paramName << " accepted in " << interfacePath << ": "
                  << paramValue << "\n";
    }
    return true;
}

bool fillConfigNumber(const SensorBaseConfigMap& baseConfigMap,
                      const std::string& interfacePath,
                      const std::string& paramName, bool isMandatory,
                      double& paramValue)
{
    auto paramFound = baseConfigMap.find(paramName);
    if (paramFound == baseConfigMap.end())
    {
        if (isMandatory)
        {
            std::cerr << paramName << " parameter not found in "
                      << interfacePath << "\n";
            return false;
        }

        if constexpr (debug)
        {
            std::cerr << paramName << " optional parameter not given in "
                      << interfacePath << "\n";
        }
        return true;
    }

    try
    {
        paramValue = std::visit(VariantToDoubleVisitor(), paramFound->second);
    }
    catch (const std::exception& e)
    {
        std::cerr << paramName << " parameter not parsed in " << interfacePath
                  << ": " << e.what() << "\n";
        return false;
    }

    if (!std::isfinite(paramValue))
    {
        std::cerr << paramName << " parameter not acceptable in "
                  << interfacePath << "\n";
        return false;
    }
    if (paramValue < 0.0)
    {
        std::cerr << paramName << " parameter not acceptable in "
                  << interfacePath << "\n";
        return false;
    }

    if constexpr (debug)
    {
        std::cerr << paramName << " accepted in " << interfacePath << ": "
                  << paramValue << "\n";
    }
    return true;
}

void startServers(Globals& globals)
{
    for (const auto& serverPair : globals.serversConfig)
    {
        RedfishServer& server = *(serverPair.second);

        if (!(server.isRelevant))
        {
            continue;
        }

        server.provideContext(globals.ioContext, globals.systemBus,
                              globals.objectServer);
        server.startNetworking();
        server.startTimer();
    }
}

void createSensorsCallback(
    Globals& globals, boost::container::flat_set<std::string>& sensorsChanged,
    const ManagedObjectType& sensorConfigurations)
{
    if constexpr (debug)
    {
        if (sensorsChanged.empty())
        {
            std::cerr << "RedfishSensor creating sensors for the first time\n";
        }
        else
        {
            std::cerr << "RedfishSensor creating sensors, changed:\n";
            for (const std::string& s : sensorsChanged)
            {
                std::cerr << s << "\n";
            }
        }
    }

    for (const std::pair<sdbusplus::message::object_path, SensorData>& sensor :
         sensorConfigurations)
    {
        const std::string& interfacePath = sensor.first.str;
        const SensorData& sensorData = sensor.second;

        const SensorBaseConfigMap* baseConfigFound = nullptr;
        std::string sensorType;
        for (const char* type : sensorTypes)
        {
            auto sensorBase = sensorData.find(configInterfaceName(type));
            if (sensorBase != sensorData.end())
            {
                baseConfigFound = &sensorBase->second;
                sensorType = type;
                break;
            }
        }
        if (baseConfigFound == nullptr)
        {
            std::cerr << "Base configuration not found for " << interfacePath
                      << "\n";
            continue;
        }

        const SensorBaseConfigMap& baseConfigMap = *baseConfigFound;
        std::string sensorName;

        // Name is mandatory
        if (!fillConfigString(baseConfigMap, interfacePath, "Name", true,
                              sensorName))
        {
            continue;
        }

        if constexpr (debug)
        {
            std::cerr << "Found config object: name " << sensorName << ", type "
                      << sensorType << "\n";
        }

        // Search for pre-existing name in the appropriate data structure
        auto findSensor = globals.sensorsConfig.end();
        auto findChassis = globals.chassisesConfig.end();
        auto findServer = globals.serversConfig.end();
        if (sensorType == "RedfishSensor")
        {
            findSensor = globals.sensorsConfig.find(sensorName);
        }
        if (sensorType == "RedfishChassis")
        {
            findChassis = globals.chassisesConfig.find(sensorName);
        }
        if (sensorType == "RedfishServer")
        {
            findServer = globals.serversConfig.find(sensorName);
        }

        bool alreadyExisting = false;
        bool foundChanged = false;

        if ((findSensor != globals.sensorsConfig.end()) ||
            (findChassis != globals.chassisesConfig.end()) ||
            (findServer != globals.serversConfig.end()))
        {
            alreadyExisting = true;
        }

        // On rescans, only update sensors we were signaled by
        std::string suffixName = "/";
        suffixName += sensor_paths::escapePathForDbus(sensorName);
        for (const std::string& it : sensorsChanged)
        {
            std::string suffixIt = "/";
            suffixIt += it;

            if (suffixIt.ends_with(suffixName))
            {
                if (findSensor != globals.sensorsConfig.end())
                {
                    findSensor->second = nullptr;
                    globals.sensorsConfig.erase(findSensor);
                }
                if (findChassis != globals.chassisesConfig.end())
                {
                    findChassis->second = nullptr;
                    globals.chassisesConfig.erase(findChassis);
                }
                if (findServer != globals.serversConfig.end())
                {
                    findServer->second = nullptr;
                    globals.serversConfig.erase(findServer);
                }

                sensorsChanged.erase(it);
                foundChanged = true;
                break;
            }
        }

        if (alreadyExisting)
        {
            if (foundChanged)
            {
                std::cerr << "Rebuilding configuration object " << sensorName
                          << "\n";
            }
            else
            {
                // Avoid disturbing existing unchanged sensors
                continue;
            }
        }
        else
        {
            if constexpr (debug)
            {
                std::cerr << "New configuration object " << sensorName << " by "
                          << (foundChanged ? "message" : "first pass") << "\n";
            }
        }

        // Handle servers separately
        if (sensorType == "RedfishServer")
        {
            std::string serverHost;
            std::string serverProtocol = defaultProtocol;
            auto serverPort = static_cast<double>(defaultPort);

            // Host is only mandatory parameter
            if (!fillConfigString(baseConfigMap, interfacePath, "Host", true,
                                  serverHost))
            {
                continue;
            }

            // Protocol and Port are optional parameters
            if (!fillConfigString(baseConfigMap, interfacePath, "Protocol",
                                  false, serverProtocol))
            {
                continue;
            }
            if (!fillConfigNumber(baseConfigMap, interfacePath, "Port", false,
                                  serverPort))
            {
                continue;
            }

            // FUTURE: Parse additional optional parameters

            auto newServer = std::make_shared<RedfishServer>();

            newServer->configName = sensorName;

            newServer->host = serverHost;
            newServer->protocol = serverProtocol;
            newServer->port = static_cast<int>(serverPort);

            // FUTURE: Provide additional optional parameters

            globals.serversConfig[sensorName] = newServer;

            if constexpr (debug)
            {
                std::cerr << "Added server " << sensorName << ": host "
                          << serverHost << ", protocol " << serverProtocol
                          << ", port " << serverPort << "\n";
            }
            continue;
        }

        // Handle chassises separately
        if (sensorType == "RedfishChassis")
        {
            std::string redName;
            std::string redId;
            std::string chaManuf;
            std::string chaModel;
            std::string chaPart;
            std::string chaSku;
            std::string chaSerial;
            std::string chaSpare;
            std::string chaVersion;

            if ((!fillConfigString(baseConfigMap, interfacePath, "RedfishName",
                                   false, redName)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "RedfishId",
                                   false, redId)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "Manufacturer",
                                   false, chaManuf)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "Model", false,
                                   chaModel)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "PartNumber",
                                   false, chaPart)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "SKU", false,
                                   chaSku)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "SerialNumber",
                                   false, chaSerial)) ||
                (!fillConfigString(baseConfigMap, interfacePath,
                                   "SparePartNumber", false, chaSpare)) ||
                (!fillConfigString(baseConfigMap, interfacePath, "Version",
                                   false, chaVersion)))
            {
                continue;
            }

            // All parameters are optional, but at least one must be given
            if (redName.empty() && redId.empty() && chaManuf.empty() &&
                chaModel.empty() && chaPart.empty() && chaSku.empty() &&
                chaSerial.empty() && chaSpare.empty() && chaVersion.empty())
            {
                std::cerr << "Chassis " << sensorName
                          << " must have at least one identifying "
                             "characteristic provided\n";
                continue;
            }

            auto newChassis = std::make_shared<RedfishChassis>();

            newChassis->configName = sensorName;

            newChassis->characteristics.redfishName = redName;
            newChassis->characteristics.redfishId = redId;

            newChassis->characteristics.manufacturer = chaManuf;
            newChassis->characteristics.model = chaModel;
            newChassis->characteristics.partNumber = chaPart;
            newChassis->characteristics.sku = chaSku;
            newChassis->characteristics.serialNumber = chaSerial;
            newChassis->characteristics.sparePartNumber = chaSpare;
            newChassis->characteristics.version = chaVersion;

            globals.chassisesConfig[sensorName] = newChassis;

            if constexpr (debug)
            {
                std::cerr << "Added chassis " << sensorName << "\n";
            }
            continue;
        }

        // At this point, the new object is assumed to be a sensor
        std::string redfishName;
        std::string redfishId;
        std::string linkChassis;
        std::string linkServer;

        PowerState readState = getPowerState(baseConfigMap);

        // The links to desired Chassis and Server are mandatory
        if ((!fillConfigString(baseConfigMap, interfacePath, "RedfishName",
                               false, redfishName)) ||
            (!fillConfigString(baseConfigMap, interfacePath, "RedfishId", false,
                               redfishId)) ||
            (!fillConfigString(baseConfigMap, interfacePath, "Chassis", true,
                               linkChassis)) ||
            (!fillConfigString(baseConfigMap, interfacePath, "Server", true,
                               linkServer)))
        {
            continue;
        }

        // RedfishName and RedfishId are optional, but one must be given
        if (redfishName.empty() && redfishId.empty())
        {
            std::cerr << "Sensor " << sensorName
                      << " must have at least one identifying characteristic "
                         "provided\n";
            continue;
        }

        auto newSensor = std::make_shared<RedfishSensor>();

        newSensor->configName = sensorName;
        newSensor->inventoryPath = interfacePath;

        newSensor->characteristics.redfishName = redfishName;
        newSensor->characteristics.redfishId = redfishId;

        newSensor->configChassis = linkChassis;
        newSensor->configServer = linkServer;
        newSensor->powerState = readState;

        globals.sensorsConfig[sensorName] = newSensor;

        if constexpr (debug)
        {
            std::cerr << "Added sensor " << sensorName << ": chassis "
                      << linkChassis << ", server " << linkServer << "\n";
        }
    }

    // The sensorsChanged list should have been entirely consumed
    for (const std::string& s : sensorsChanged)
    {
        std::cerr << "Sensor was supposed to be changed but not found: " << s
                  << "\n";
    }
    sensorsChanged.clear();

    validateCreation(globals);

    startServers(globals);
}

void createSensors(Globals& globals,
                   boost::container::flat_set<std::string>& sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        globals.systemBus,
        [&globals, &sensorsChanged](const ManagedObjectType& sensorConfigs) {
        createSensorsCallback(globals, sensorsChanged, sensorConfigs);
    });

    getter->getConfiguration(
        std::vector<std::string>(sensorTypes.begin(), sensorTypes.end()));
}

int main()
{
    if constexpr (debug)
    {
        std::cerr << "RedfishSensor: Service starting up\n";
    }

    Globals globals;

    // Not just sensors changed, Chassis and Server could also be changed
    boost::container::flat_set<std::string> sensorsChanged;

    globals.ioContext->post([&globals, &sensorsChanged]() mutable {
        createSensors(globals, sensorsChanged);
    });

    boost::asio::steady_timer filterTimer(*(globals.ioContext));
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&filterTimer, &globals,
         &sensorsChanged](sdbusplus::message_t& message) mutable {
        if (message.is_method_error())
        {
            std::cerr << "Callback method error in main\n";
            return;
        }

        const std::string messagePath = message.get_path();
        sensorsChanged.insert(messagePath);
        if constexpr (debug)
        {
            std::cerr << "Received message from " << messagePath << "\n";
        }

        // Defer action, so rapidly incoming requests can be batched up
        filterTimer.expires_from_now(std::chrono::seconds(1));

        filterTimer.async_wait(
            [&globals,
             &sensorsChanged](const boost::system::error_code& ec) mutable {
            if (ec == boost::asio::error::operation_aborted)
            {
                if constexpr (debug)
                {
                    std::cerr << "Timer cancelled\n";
                }
                return;
            }
            if (ec)
            {
                std::cerr << "Timer error: " << ec.message() << "\n";
                return;
            }

            if constexpr (debug)
            {
                std::cerr << "Now changing sensors\n";
            }
            createSensors(globals, sensorsChanged);
        });
    };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*(globals.systemBus), sensorTypes,
                                      eventHandler);

    if constexpr (debug)
    {
        std::cerr << "RedfishSensor: Service entering main loop\n";
    }

    globals.ioContext->run();

    if constexpr (debug)
    {
        std::cerr << "RedfishSensor: Service shutting down\n";
    }

    return 0;
}
