/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpDiscovery.hpp"

#include "AmdBulkDevice.hpp"
#include "EntityManagerInterface.hpp"
#include "NetworkAdapter.hpp"
#include "OcpVdmIana.hpp"

#include <MctpAsyncRequester.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace ocp::bulk
{

namespace
{

namespace rules = sdbusplus::match_rules;

constexpr std::chrono::milliseconds pollInterval{120000};
constexpr std::chrono::seconds retryInterval{5};

constexpr auto boardInterface = "xyz.openbmc_project.Inventory.Item.Board";
constexpr auto assetInterface = "xyz.openbmc_project.Inventory.Decorator.Asset";
constexpr auto uuidInterface = "xyz.openbmc_project.Common.UUID";

constexpr auto endpointInterface = "xyz.openbmc_project.MCTP.Endpoint";
constexpr auto mctpEndpointInterface = "au.com.codeconstruct.MCTP.Endpoint1";
constexpr auto bridgeInterface = "au.com.codeconstruct.MCTP.Bridge1";
constexpr auto mctpSearchPath = "/au/com/codeconstruct/";
constexpr auto mctpNetworksPath = "/au/com/codeconstruct/mctp1/networks";

// arg0path only matches a prefix when the prefix ends in a slash
const std::string endpointPrefix = std::string{mctpNetworksPath} + "/";

constexpr auto mapper = sdbusplus::async::proxy()
                            .service("xyz.openbmc_project.ObjectMapper")
                            .path("/xyz/openbmc_project/object_mapper")
                            .interface("xyz.openbmc_project.ObjectMapper");

using GetSubTreeType = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;

// The printable bytes of the endpoint UUID are the NIC's board serial number,
// which is how an endpoint finds its entity-manager inventory.
std::string uuidToAscii(const std::string& uuid)
{
    std::string hex = uuid;
    std::erase(hex, '-');

    std::string ascii;
    for (size_t i = 0; i + 1 < hex.size(); i += 2)
    {
        const char* first = hex.data() + i;
        uint8_t byte = 0;
        auto [last, ec] = std::from_chars(first, first + 2, byte, 16);
        if (ec != std::errc{} || last != first + 2)
        {
            return {};
        }

        if (byte >= 0x20 && byte < 0x7F)
        {
            ascii.push_back(static_cast<char>(byte));
        }
    }

    return ascii;
}

} // namespace

MctpDiscovery::MctpDiscovery(sdbusplus::async::context& ctx,
                             mctp::AsyncRequester& requester) :
    ctx(ctx), requester(requester),
    // Hotplug can give the same EID to a different device, so endpoints are
    // tracked by their mctpd path and identified by UUID.
    endpointAdded(ctx, rules::interfacesAddedAtPath(endpointPrefix)),
    endpointRemoved(ctx, rules::interfacesRemovedAtPath(endpointPrefix)),
    connectivityChanged(ctx, rules::propertiesChangedNamespace(
                                 mctpNetworksPath, mctpEndpointInterface)),
    entityManager(ctx, {boardInterface},
                  std::bind_front(&MctpDiscovery::boardAdded, this),
                  std::bind_front(&MctpDiscovery::boardRemoved, this))
{}

auto MctpDiscovery::run() -> sdbusplus::async::task<>
{
    // Pick up the boards entity-manager already has.
    co_await entityManager.handleInventoryGet();

    ctx.spawn(watch(endpointAdded));
    ctx.spawn(watch(endpointRemoved));
    ctx.spawn(watch(connectivityChanged));

    co_await updateEndpoints();
}

auto MctpDiscovery::watch(sdbusplus::async::match& signal)
    -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        co_await signal.next();
        co_await updateEndpoints();
    }
}

void MctpDiscovery::boardAdded(const sdbusplus::object_path& path,
                               const std::string& /*interfaceName*/)
{
    ctx.spawn(addBoard(path));
}

void MctpDiscovery::boardRemoved(const sdbusplus::object_path& path,
                                 const std::string& /*interfaceName*/)
{
    std::erase_if(boardMap, [&path](const auto& board) {
        return board.second == path.str;
    });
}

auto MctpDiscovery::addBoard(sdbusplus::object_path path)
    -> sdbusplus::async::task<>
{
    std::string serial;
    try
    {
        serial =
            co_await sdbusplus::async::proxy()
                .service(entity_manager::EntityManagerInterface::serviceName)
                .path(path.str)
                .interface(assetInterface)
                .preserve()
                .get_property<std::string>(ctx, "SerialNumber");
    }
    catch (const std::exception& e)
    {
        lg2::debug("{PATH} has no serial number: {ERROR}", "PATH", path.str,
                   "ERROR", e.what());
        co_return;
    }

    if (serial.empty())
    {
        co_return;
    }

    boardMap.insert_or_assign(std::move(serial), path.str);

    co_await updateEndpoints();
}

std::string MctpDiscovery::findBoard(const std::string& uuid) const
{
    const std::string uuidAscii = uuidToAscii(uuid);
    for (const auto& [serial, path] : boardMap)
    {
        if (uuidAscii.contains(serial))
        {
            return path;
        }
    }

    return {};
}

// A signal only says something changed, so rebuild the map from mctpd.
auto MctpDiscovery::updateEndpoints() -> sdbusplus::async::task<>
{
    sdbusplus::async::lock_guard guard{updateMutex};
    co_await guard.lock();

    GetSubTreeType endpoints;
    try
    {
        endpoints = co_await mapper.call<GetSubTreeType>(
            ctx, "GetSubTree", std::string{mctpSearchPath}, 0,
            std::vector<std::string>{endpointInterface});
    }
    catch (const std::exception& e)
    {
        lg2::error("Error listing MCTP endpoints: {ERROR}", "ERROR", e.what());
        ctx.spawn(retryUpdate());
        co_return;
    }

    std::map<std::string, std::string> current;
    for (const auto& [objPath, services] : endpoints)
    {
        if (services.empty())
        {
            continue;
        }

        const auto& service = services.front().first;
        if (!co_await isAvailable(service, objPath))
        {
            continue;
        }

        current.emplace(objPath, service);
    }

    std::vector<std::string> stale;
    for (const auto& [path, endpoint] : endpointMap)
    {
        if (!current.contains(path))
        {
            stale.push_back(path);
        }
    }
    for (const auto& path : stale)
    {
        removeEndpoint(path);
    }

    // addEndpoint says whether mctpd answered, not whether it added anything.
    bool needRetry = false;
    for (const auto& [path, service] : current)
    {
        if (!co_await addEndpoint(service, path))
        {
            removeEndpoint(path);
            needRetry = true;
        }
    }

    if (needRetry)
    {
        ctx.spawn(retryUpdate());
    }
}

// A read that fails while mctpd is still settling gets no second signal.
auto MctpDiscovery::retryUpdate() -> sdbusplus::async::task<>
{
    if (std::exchange(retryPending, true))
    {
        co_return;
    }

    co_await sdbusplus::async::sleep_for(ctx, retryInterval);
    retryPending = false;
    co_await updateEndpoints();
}

auto MctpDiscovery::isAvailable(std::string service, std::string endpointPath)
    -> sdbusplus::async::task<bool>
{
    try
    {
        auto state = co_await sdbusplus::async::proxy()
                         .service(service)
                         .path(endpointPath)
                         .interface(mctpEndpointInterface)
                         .preserve()
                         .get_property<std::string>(ctx, "Connectivity");
        co_return state == "Available";
    }
    catch (const std::exception&)
    {
        co_return true;
    }
}

auto MctpDiscovery::addEndpoint(std::string service, std::string endpointPath)
    -> sdbusplus::async::task<bool>
{
    std::string uuid;
    try
    {
        uuid = co_await sdbusplus::async::proxy()
                   .service(service)
                   .path(endpointPath)
                   .interface(uuidInterface)
                   .preserve()
                   .get_property<std::string>(ctx, "UUID");
    }
    catch (const std::exception& e)
    {
        lg2::debug("{PATH} has no UUID: {ERROR}", "PATH", endpointPath, "ERROR",
                   e.what());
    }

    // Without a UUID there is no way to match the endpoint to inventory.
    if (uuid.empty())
    {
        co_return true;
    }

    auto entry = endpointMap.find(endpointPath);
    if (entry != endpointMap.end())
    {
        if (entry->second.uuid == uuid)
        {
            co_return true;
        }

        removeEndpoint(endpointPath);
    }

    auto endpoint = sdbusplus::async::proxy()
                        .service(service)
                        .path(endpointPath)
                        .interface(endpointInterface)
                        .preserve();

    uint8_t eid = 0;
    std::vector<uint8_t> types;
    try
    {
        eid = co_await endpoint.get_property<uint8_t>(ctx, "EID");
        types = co_await endpoint.get_property<std::vector<uint8_t>>(
            ctx, "SupportedMessageTypes");
    }
    catch (const std::exception& e)
    {
        lg2::debug("{PATH} could not be read: {ERROR}", "PATH", endpointPath,
                   "ERROR", e.what());
        co_return false;
    }

    if (std::ranges::find(types, mctpMessageType) == types.end())
    {
        co_return true;
    }

    // There is nowhere to publish yet, so leave the endpoint alone and pick
    // it up when its board arrives.
    const std::string boardPath = findBoard(uuid);
    if (boardPath.empty())
    {
        co_return true;
    }

    lg2::info("Found OCP VDM IANA endpoint with EID {EID}", "EID", eid);

    endpointMap[endpointPath].uuid = uuid;
    addDevice(eid, boardPath, endpointPath);
    co_return co_await addPoolEndpoints(service, endpointPath, boardPath);
}

// Devices behind a bridge get no D-Bus objects, only a pool EID range.
auto MctpDiscovery::addPoolEndpoints(
    std::string service, std::string endpointPath, std::string boardPath)
    -> sdbusplus::async::task<bool>
{
    auto bridge = sdbusplus::async::proxy()
                      .service(service)
                      .path(endpointPath)
                      .interface(bridgeInterface)
                      .preserve();

    uint8_t poolStart = 0;
    uint8_t poolEnd = 0;
    try
    {
        poolStart = co_await bridge.get_property<uint8_t>(ctx, "PoolStart");
        poolEnd = co_await bridge.get_property<uint8_t>(ctx, "PoolEnd");
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        // A plain endpoint has no bridge interface, so this is not an error.
        const std::string_view name = e.name();
        if (name == "org.freedesktop.DBus.Error.UnknownInterface" ||
            name == "org.freedesktop.DBus.Error.UnknownProperty" ||
            name == "org.freedesktop.DBus.Error.InvalidArgs")
        {
            co_return true;
        }

        lg2::debug("{PATH} has no readable EID pool: {ERROR}", "PATH",
                   endpointPath, "ERROR", e.what());
        co_return false;
    }
    catch (const std::exception& e)
    {
        lg2::debug("{PATH} has no readable EID pool: {ERROR}", "PATH",
                   endpointPath, "ERROR", e.what());
        co_return false;
    }

    for (unsigned eid = poolStart; eid <= poolEnd; ++eid)
    {
        lg2::info("Found bridged endpoint with EID {EID}", "EID", eid);
        addDevice(static_cast<uint8_t>(eid), boardPath, endpointPath);
    }

    co_return true;
}

void MctpDiscovery::addDevice(uint8_t eid, const std::string& boardPath,
                              const std::string& endpointPath)
{
    if (deviceMap.contains(eid))
    {
        return;
    }

    auto device = std::make_shared<AmdBulkDevice>(
        ctx, requester, eid, adapterFor(boardPath), pollInterval);
    deviceMap.emplace(eid, device);
    endpointMap[endpointPath].eids.push_back(eid);
    ctx.spawn(device->run());
}

void MctpDiscovery::removeEndpoint(const std::string& endpointPath)
{
    auto entry = endpointMap.find(endpointPath);
    if (entry == endpointMap.end())
    {
        return;
    }

    for (uint8_t eid : entry->second.eids)
    {
        auto device = deviceMap.find(eid);
        if (device == deviceMap.end())
        {
            continue;
        }

        lg2::info("Dropping EID {EID}", "EID", eid);
        device->second->stop();
        deviceMap.erase(device);
    }

    endpointMap.erase(entry);
}

// A bridge and the EIDs behind it are one NIC, so they share one adapter.
// The weak pointer lets it go with the last device.
std::shared_ptr<NetworkAdapter> MctpDiscovery::adapterFor(
    const std::string& boardPath)
{
    if (boardPath.empty())
    {
        return {};
    }

    auto& entry = adapterMap[boardPath];
    auto adapter = entry.lock();
    if (!adapter)
    {
        adapter = std::make_shared<NetworkAdapter>(ctx, boardPath);
        entry = adapter;
    }
    return adapter;
}

} // namespace ocp::bulk
