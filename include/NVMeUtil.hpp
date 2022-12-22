#pragma once
#include "NVMeSensor.hpp"

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>

#include <filesystem>
#include <iostream>
#include <optional>

inline std::filesystem::path deriveRootBusPath(int busNumber)
{
    return "/sys/bus/i2c/devices/i2c-" + std::to_string(busNumber) +
           "/mux_device";
}

inline std::optional<int> deriveRootBus(std::optional<int> busNumber)
{
    if (!busNumber)
    {
        return std::nullopt;
    }

    std::filesystem::path muxPath = deriveRootBusPath(*busNumber);

    if (!std::filesystem::is_symlink(muxPath))
    {
        return *busNumber;
    }

    std::string rootName = std::filesystem::read_symlink(muxPath).filename();
    size_t dash = rootName.find('-');
    if (dash == std::string::npos)
    {
        std::cerr << "Error finding root bus for " << rootName << "\n";
        return std::nullopt;
    }

    return std::stoi(rootName.substr(0, dash));
}

inline std::optional<std::string>
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

// a path of
// "/xyz/openbmc_project/inventory/system/board/{prod}/{nvme}/{substruct}..."
// will generates a sensor name {prod}_{nvme}_{substruct}...
inline std::optional<std::string>
    createSensorNameFromPath(const std::string& path)
{
    if (path.empty())
    {
        return std::nullopt;
    }
    auto rbegin = path.crbegin();
    std::vector<std::string> names;
    do
    {
        auto name = extractOneFromTail(rbegin, path.crend());
        if (!name)
        {
            return std::nullopt;
        }
        else if (*name == "board")
        {
            break;
        }
        names.insert(names.begin(), *name);
    } while (rbegin != path.rend());

    std::string name = boost::algorithm::join(names, "_");
    return name;
}

// Function to update NVMe temp sensor

// Function type for fetching ctemp which incaplucated in a structure of T.
// The fetcher function take a callback as input to process the result.
template <class T>
using ctemp_fetcher_t =
    std::function<void(std::function<void(const std::error_code&, T)>&&)>;

// Function type for parsing ctemp out the structure of type T.
// The parser function will return the value of ctemp or nullopt on failure.
template <class T>
using ctemp_parser_t = std::function<std::optional<double>(T data)>;

template <class T>
void pollCtemp(
    std::shared_ptr<boost::asio::steady_timer> timer,
    std::shared_ptr<NVMeSensor> sensor,
    const std::function<void(std::function<void(const std::error_code&, T)>&&)>&
        dataFetcher,
    const std::function<std::optional<double>(T data)>& dataParser);

namespace detail
{

template <class T>
void updateCtemp(std::shared_ptr<boost::asio::steady_timer> timer,
                 std::shared_ptr<NVMeSensor> sensor,
                 ctemp_parser_t<T> dataParser, ctemp_fetcher_t<T> dataFetcher,
                 const boost::system::error_code error, T data)
{
    if (error)
    {
        std::cerr << "error reading ctemp from subsystem"
                  << ", reason:" << error.message() << "\n";
        sensor->markFunctional(false);
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }
    auto value = dataParser(data);
    if (!value)
    {
        sensor->incrementError();
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }

    sensor->updateValue(*value);
    ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher, dataParser);
}

template <class T>
void pollCtemp(std::shared_ptr<boost::asio::steady_timer> timer,
               std::shared_ptr<NVMeSensor> sensor,
               ctemp_fetcher_t<T> dataFetcher, ctemp_parser_t<T> dataParser,
               const boost::system::error_code errorCode)
{

    if (errorCode == boost::asio::error::operation_aborted)
    {
        return;
    }
    if (errorCode)
    {
        std::cerr << errorCode.message() << "\n";
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }

    if (!sensor)
    {
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }

    if (!sensor->readingStateGood())
    {
        sensor->markAvailable(false);
        sensor->updateValue(std::numeric_limits<double>::quiet_NaN());
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }

    /* Potentially defer sampling the sensor sensor if it is in error */
    if (!sensor->sample())
    {
        ::pollCtemp(std::move(timer), std::move(sensor), dataFetcher,
                    dataParser);
        return;
    }

    dataFetcher(std::bind_front(detail::updateCtemp<T>, std::move(timer),
                                std::move(sensor), dataParser, dataFetcher));
}

} // namespace detail

template <class T>
void pollCtemp(
    std::shared_ptr<boost::asio::steady_timer> timer,
    std::shared_ptr<NVMeSensor> sensor,
    const std::function<void(std::function<void(const std::error_code&, T)>&&)>&
        dataFetcher,
    const std::function<std::optional<double>(T data)>& dataParser)
{
    if (!timer && !sensor)
    {
        return;
    }
    timer->expires_from_now(std::chrono::seconds(1));
    timer->async_wait(std::bind_front(detail::pollCtemp<T>, std::move(timer),
                                      std::move(sensor), dataFetcher,
                                      dataParser));
}
