/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "BulkTelemetryCommands.hpp"
#include "NetworkAdapter.hpp"

#include <MctpAsyncRequester.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

namespace ocp::bulk
{

// Asks one endpoint what data it can report, then polls it for that data.
class AmdBulkDevice : public std::enable_shared_from_this<AmdBulkDevice>
{
  public:
    AmdBulkDevice(sdbusplus::async::context& ctx,
                  mctp::AsyncRequester& requester, uint8_t eid,
                  std::shared_ptr<NetworkAdapter> adapter,
                  std::chrono::milliseconds pollInterval);

    AmdBulkDevice(const AmdBulkDevice&) = delete;
    AmdBulkDevice(AmdBulkDevice&&) = delete;
    AmdBulkDevice& operator=(const AmdBulkDevice&) = delete;
    AmdBulkDevice& operator=(AmdBulkDevice&&) = delete;
    ~AmdBulkDevice() = default;

    auto run() -> sdbusplus::async::task<>;

    // The running task keeps this object alive, so it has to be told to stop.
    void stop();

  private:
    struct Category
    {
        CategoryDescription description;
        std::vector<uint32_t> uniqueIds;
    };

    auto sendRecv(std::optional<std::vector<uint8_t>> request,
                  std::string_view operation)
        -> sdbusplus::async::task<std::optional<std::vector<uint8_t>>>;

    auto discoverCategories() -> sdbusplus::async::task<bool>;

    auto pollOnce() -> sdbusplus::async::task<bool>;
    auto readCategory(size_t index) -> sdbusplus::async::task<bool>;

    sdbusplus::async::context& ctx;
    mctp::AsyncRequester& requester;
    uint8_t eid;
    std::shared_ptr<NetworkAdapter> adapter;
    std::chrono::milliseconds pollInterval;

    CategoryInformation categoryInfo{};
    std::vector<Category> categories;

    bool stopped{false};
};

} // namespace ocp::bulk
