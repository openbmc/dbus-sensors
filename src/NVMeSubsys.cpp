#include "NVMeSubsys.hpp"

#include "Thresholds.hpp"

std::optional<std::string>
    extractOneFromTail(std::string::const_reverse_iterator& rbegin,
                       const std::string::const_reverse_iterator& rend)
{
    std::string name;
    auto curr = rbegin;
    // remove the ending '\'s
    while (rbegin != rend && *rbegin == '\\')
    {
        rbegin++;
    }
    if (rbegin == rend)
    {
        return std::nullopt;
    }
    curr = rbegin++;

    // extract word
    while (rbegin != rend && *rbegin != '\\')
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
    if (path.size() == 0)
        return std::nullopt;
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
    return {name};
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

NVMeSubsys::NVMeSubsys(boost::asio::io_context& io,
                       sdbusplus::asio::object_server& objServer,
                       std::shared_ptr<sdbusplus::asio::connection> conn,
                       std::string path, std::string name,
                       const SensorData& configData,
                       const std::shared_ptr<NVMeIntf>& intf) :
    io(io),
    objServer(objServer), conn(conn), path(path), name(name), nvmeIntf(intf),
    ctempTimer(io)
{
    if (!intf)
    {
        throw std::runtime_error("NVMe interface is null");
    }
    if (dynamic_cast<NVMeBasicIntf*>(nvmeIntf.get()))
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

    // start to poll value for CTEMP sensor.
    std::function<void(std::function<void(const std::error_code&,
                                          NVMeBasicIntf::DriveStatus*)> &&)>
        dataFether =
            [nvmeIntf{this->nvmeIntf}](
                std::function<void(const std::error_code&,
                                   NVMeBasicIntf::DriveStatus*)>&& cb) {
        auto intf = dynamic_cast<NVMeBasicIntf*>(nvmeIntf.get());
        intf->getStatus(std::move(cb));
    };
    std::function<std::optional<double>(NVMeBasicIntf::DriveStatus*)>
        dataParser =
            [](NVMeBasicIntf::DriveStatus* status) -> std::optional<double> {
        if (status == nullptr)
        {
            return std::nullopt;
        }
        return {getTemperatureReading(status->Temp)};
    };
    pollCtemp(dataFether, dataParser);
}

template <class T>
void NVMeSubsys::pollCtemp(
    const std::function<void(std::function<void(const std::error_code&, T)>&&)>&
        dataFetcher,
    const std::function<std::optional<double>(T Data)>& dataParser)
{
    ctempTimer.expires_from_now(boost::posix_time::seconds(1));
    ctempTimer.async_wait(
        [self{shared_from_this()}, dataFetcher{dataFetcher},
         dataParser(dataParser)](
            const boost::system::error_code errorCode) mutable {
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

        dataFetcher([self{self->shared_from_this()}, dataFetcher{dataFetcher},
                     dataParser{dataParser}](const std::error_code& error,
                                             T data) mutable {
            if (error)
            {
                std::cerr << "error reading ctemp from subsystem: "
                          << self->name << "\n";
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
        });
    });
}