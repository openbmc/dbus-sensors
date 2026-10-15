/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "AmdBulkDevice.hpp"
#include "EntityManagerInterface.hpp"
#include "NetworkAdapter.hpp"

#include <MctpAsyncRequester.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/match.hpp>
#include <sdbusplus/async/mutex.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ocp::bulk
{

// Creates a device for every endpoint mctpd reports as supporting 0x7F, and
// drops it when mctpd stops publishing that endpoint.
class MctpDiscovery
{
  public:
    MctpDiscovery(sdbusplus::async::context& ctx,
                  mctp::AsyncRequester& requester);

    MctpDiscovery(const MctpDiscovery&) = delete;
    MctpDiscovery(MctpDiscovery&&) = delete;
    MctpDiscovery& operator=(const MctpDiscovery&) = delete;
    MctpDiscovery& operator=(MctpDiscovery&&) = delete;
    ~MctpDiscovery() = default;

    auto run() -> sdbusplus::async::task<>;

  private:
    auto watch(sdbusplus::async::match& signal) -> sdbusplus::async::task<>;

    void boardAdded(const sdbusplus::object_path& path,
                    const std::string& interfaceName);
    void boardRemoved(const sdbusplus::object_path& path,
                      const std::string& interfaceName);
    auto addBoard(sdbusplus::object_path path) -> sdbusplus::async::task<>;
    std::string findBoard(const std::string& uuid) const;

    auto updateEndpoints() -> sdbusplus::async::task<>;
    auto retryUpdate() -> sdbusplus::async::task<>;
    auto isAvailable(std::string service, std::string endpointPath)
        -> sdbusplus::async::task<bool>;

    // False if mctpd could not be read, so the endpoint needs a retry.
    auto addEndpoint(std::string service, std::string endpointPath)
        -> sdbusplus::async::task<bool>;
    auto addPoolEndpoints(std::string service, std::string endpointPath,
                          std::string boardPath)
        -> sdbusplus::async::task<bool>;

    void addDevice(uint8_t eid, const std::string& boardPath,
                   const std::string& endpointPath);
    void removeEndpoint(const std::string& endpointPath);
    std::shared_ptr<NetworkAdapter> adapterFor(const std::string& boardPath);

    sdbusplus::async::context& ctx;
    mctp::AsyncRequester& requester;

    // The same object path can come back as a different device, so the UUID
    // is what identifies it.
    struct Endpoint
    {
        std::string uuid;
        std::vector<uint8_t> eids;
    };

    std::map<uint8_t, std::shared_ptr<AmdBulkDevice>> deviceMap;
    std::map<std::string, Endpoint> endpointMap;
    std::map<std::string, std::weak_ptr<NetworkAdapter>> adapterMap;
    std::map<std::string, std::string> boardMap;

    sdbusplus::async::mutex updateMutex{"ocp::bulk::MctpDiscovery"};
    bool retryPending = false;

    sdbusplus::async::match endpointAdded;
    sdbusplus::async::match endpointRemoved;
    sdbusplus::async::match connectivityChanged;

    entity_manager::EntityManagerInterface entityManager;
};

} // namespace ocp::bulk
