#include "NVMeSubsys.hpp"

#include "Thresholds.hpp"

std::optional<std::string>
    extractOneFromTail(std::string::const_reverse_iterator& rbegin,
                       const std::string::const_reverse_iterator& rend)
{
    std::string name;
    auto curr = rbegin;
    // remove the ending '/'s
    while (rbegin != rend && *rbegin == '/')
    {
        rbegin++;
    }
    if (rbegin == rend)
    {
        return std::nullopt;
    }
    curr = rbegin++;

    // extract word
    while (rbegin != rend && *rbegin != '/')
    {
        rbegin++;
    }
    if (rbegin == rend)
    {
        return std::nullopt;
    }
    name.append(rbegin.base(), curr.base());
    return {name};
}

// a path of "/xyz/openbmc_project/inventory/system/board/{prod}/{nvme}" will
// generates a sensor name {prod}_{nvme}
std::optional<std::string> createSensorNameFromPath(const std::string& path)
{
    if (path.empty())
    {
        return std::nullopt;
    }
    auto rbegin = path.crbegin();

    auto nvme = extractOneFromTail(rbegin, path.crend());
    auto prod = extractOneFromTail(rbegin, path.crend());
    auto board = extractOneFromTail(rbegin, path.crend());

    if (!nvme || !prod || !board || board != "board")
    {
        return std::nullopt;
    }
    std::string name{std::move(*prod)};
    name.append("_");
    name.append(*nvme);
    return name;
}

// get temporature from a NVMe Basic reading.
static double getTemperatureReading(int8_t reading)
{
    if (reading == static_cast<int8_t>(0x80) ||
        reading == static_cast<int8_t>(0x81))
    {
        // 0x80 = No temperature data or temperature data is more the 5 s
        // old 0x81 = Temperature sensor failure
        return std::numeric_limits<double>::quiet_NaN();
    }

    return reading;
}

NVMeSubsystem::NVMeSubsystem(boost::asio::io_context& asio,
                             sdbusplus::asio::object_server& server,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             const std::string& path, const std::string& name,
                             const SensorData& configData, NVMeIntf intf) :
    io(asio),
    objServer(server), conn(conn), path(path), name(name),
    nvmeIntf(std::move(intf)), ctempTimer(io)
{
    NVMeIntf::Protocol protocol{NVMeIntf::Protocol::NVMeBasic};
    try
    {
        protocol = nvmeIntf.getProtocol();
    }
    catch (const std::runtime_error&)
    {
        throw std::runtime_error("NVMe interface is null");
    }

    if (protocol == NVMeIntf::Protocol::NVMeBasic)
    {
        std::optional<std::string> sensorName = createSensorNameFromPath(path);
        if (!sensorName)
        {
            // fail to parse sensor name from path, using name instead.
            sensorName.emplace(name);
        }

        std::vector<thresholds::Threshold> sensorThresholds;
        if (!parseThresholdsFromConfig(configData, sensorThresholds))
        {
            std::cerr << "error populating thresholds for " << *sensorName
                      << "\n";
            throw std::runtime_error("error populating thresholds for " +
                                     *sensorName);
        }

        ctemp.emplace(objServer, io, conn, *sensorName,
                      std::move(sensorThresholds), path);
    }
    else
    {
        throw std::runtime_error("Unsupported NVMe interface");
    }
}

void NVMeSubsystem::start()
{
    // start to poll value for CTEMP sensor.
    if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeBasic)

    {
        auto intf =
            std::get<std::shared_ptr<NVMeBasicIntf>>(nvmeIntf.getInferface());
        ctemp_fetcher_t<NVMeBasicIntf::DriveStatus*> dataFether =
            [intf](std::function<void(const std::error_code&,
                                      NVMeBasicIntf::DriveStatus*)>&& cb) {
            intf->getStatus(std::move(cb));
        };
        ctemp_parser_t<NVMeBasicIntf::DriveStatus*> dataParser =
            [](NVMeBasicIntf::DriveStatus* status) -> std::optional<double> {
            if (status == nullptr)
            {
                return std::nullopt;
            }
            return {getTemperatureReading(status->Temp)};
        };
        pollCtemp(dataFether, dataParser);
    }
}

template <class T>
void NVMeSubsystem::pollCtemp(
    const std::function<void(std::function<void(const std::error_code&, T)>&&)>&
        dataFetcher,
    const std::function<std::optional<double>(T data)>& dataParser)
{
    ctempTimer.expires_from_now(std::chrono::seconds(1));
    ctempTimer.async_wait(std::bind_front(NVMeSubsystem::Detail::pollCtemp<T>,
                                          shared_from_this(), dataFetcher,
                                          dataParser));
}

template <class T>
void NVMeSubsystem::Detail::pollCtemp(std::shared_ptr<NVMeSubsystem> self,
                                      ctemp_fetcher_t<T> dataFetcher,
                                      ctemp_parser_t<T> dataParser,
                                      const boost::system::error_code errorCode)
{

    if (errorCode == boost::asio::error::operation_aborted)
    {
        return;
    }
    if (errorCode)
    {
        std::cerr << errorCode.message() << "\n";
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }

    if (!self->ctemp)
    {
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }

    if (!self->ctemp->readingStateGood())
    {
        self->ctemp->markAvailable(false);
        self->ctemp->updateValue(std::numeric_limits<double>::quiet_NaN());
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }

    /* Potentially defer sampling the sensor sensor if it is in error */
    if (!self->ctemp->sample())
    {
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }

    dataFetcher(
        std::bind_front(Detail::updateCtemp<T>, self, dataParser, dataFetcher));
}

template <class T>
void NVMeSubsystem::Detail::updateCtemp(
    const std::shared_ptr<NVMeSubsystem>& self, ctemp_parser_t<T> dataParser,
    ctemp_fetcher_t<T> dataFetcher, const boost::system::error_code error,
    T data)
{
    if (error)
    {
        std::cerr << "error reading ctemp from subsystem: " << self->name
                  << ", reason:" << error.message() << "\n";
        self->ctemp->markFunctional(false);
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }
    auto value = dataParser(data);
    if (!value)
    {
        self->ctemp->incrementError();
        self->pollCtemp(dataFetcher, dataParser);
        return;
    }

    self->ctemp->updateValue(*value);
    self->pollCtemp(dataFetcher, dataParser);
}
