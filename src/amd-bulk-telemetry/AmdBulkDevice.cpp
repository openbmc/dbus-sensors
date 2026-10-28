/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AmdBulkDevice.hpp"

#include "BulkTelemetryCommands.hpp"
#include "NetworkAdapter.hpp"
#include "OcpVdmIana.hpp"
#include "OsfpStats.hpp"

#include <MctpAsyncRequester.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/task.hpp>
#include <sdbusplus/async/timer.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace ocp::bulk
{

AmdBulkDevice::AmdBulkDevice(sdbusplus::async::context& ctx,
                             mctp::AsyncRequester& requester, uint8_t eid,
                             std::shared_ptr<NetworkAdapter> adapter,
                             std::chrono::milliseconds pollInterval) :
    ctx(ctx), requester(requester), eid(eid), adapter(std::move(adapter)),
    pollInterval(pollInterval)
{}

void AmdBulkDevice::stop()
{
    stopped = true;
}

auto AmdBulkDevice::run() -> sdbusplus::async::task<>
{
    // Keep this object alive; discovery lets go of it in stop().
    auto self = shared_from_this();

    while (!stopped)
    {
        // A device that is still booting answers errNotReady, so keep
        // retrying instead of giving up.
        if (co_await discoverCategories())
        {
            // A reset wipes what discovery learned, so start over when
            // polling stops working.
            while (!stopped && co_await pollOnce())
            {
                co_await sdbusplus::async::sleep_for(ctx, pollInterval);
            }
        }

        if (!stopped)
        {
            co_await sdbusplus::async::sleep_for(ctx, pollInterval);
        }
    }
}

auto AmdBulkDevice::sendRecv(std::optional<std::vector<uint8_t>> request,
                             std::string_view operation)
    -> sdbusplus::async::task<std::optional<std::vector<uint8_t>>>
{
    if (!request)
    {
        lg2::error("EID {EID}: failed to encode {OP} request", "EID", eid, "OP",
                   operation);
        co_return std::nullopt;
    }

    auto response = co_await requester.sendRecvMsg(eid, *request);
    if (!response)
    {
        lg2::error("EID {EID}: {OP} failed: {MSG}", "EID", eid, "OP", operation,
                   "MSG", std::make_error_code(response.error()).message());
        co_return std::nullopt;
    }

    co_return std::move(*response);
}

auto AmdBulkDevice::discoverCategories() -> sdbusplus::async::task<bool>
{
    categories.clear();

    auto response = co_await sendRecv(encodeGetVersionRequest(), "get version");
    if (!response)
    {
        co_return false;
    }

    CompletionCode code{};
    uint32_t version = 0;
    if (!decodeGetVersionResponse(*response, code, version))
    {
        // Every vendor shares the IANA message type, so a reply that is not
        // ours means the endpoint is not ours either.
        lg2::info("EID {EID}: not a bulk telemetry endpoint", "EID", eid);
        stopped = true;
        co_return false;
    }

    if (code != CompletionCode::success)
    {
        if (code != CompletionCode::errNotReady &&
            code != CompletionCode::errBusy)
        {
            lg2::info("EID {EID}: bulk telemetry unsupported, cc={CC}", "EID",
                      eid, "CC", static_cast<unsigned>(code));
            stopped = true;
        }
        co_return false;
    }

    lg2::info("EID {EID}: bulk telemetry version {VER}", "EID", eid, "VER",
              std::format("{}.{}.{}.{}", version >> 24U,
                          (version >> 16U) & 0xFFU, (version >> 8U) & 0xFFU,
                          version & 0xFFU));

    response = co_await sendRecv(encodeCategoryInformationRequest(),
                                 "category information");
    if (!response)
    {
        co_return false;
    }

    if (!decodeCategoryInformationResponse(*response, code, categoryInfo) ||
        code != CompletionCode::success)
    {
        lg2::error("EID {EID}: category information rejected, cc={CC}", "EID",
                   eid, "CC", static_cast<unsigned>(code));
        co_return false;
    }

    lg2::info("EID {EID}: {COUNT} categories, detail count {DETAIL}, "
              "max transfer {MAX}",
              "EID", eid, "COUNT", categoryInfo.categoryCount, "DETAIL",
              categoryInfo.categoryDetailCount, "MAX",
              categoryInfo.maxTransferLength);

    for (uint8_t index = 0; index < categoryInfo.categoryCount; ++index)
    {
        response = co_await sendRecv(encodeCategoryDescriptionRequest(index),
                                     "description");
        if (!response)
        {
            co_return false;
        }

        std::vector<CategoryDescription> descriptions;
        if (!decodeCategoryDescriptionResponse(*response,
                                               categoryInfo.categoryDetailCount,
                                               code, descriptions) ||
            code != CompletionCode::success || descriptions.empty())
        {
            lg2::error("EID {EID}: description {IDX} rejected, cc={CC}", "EID",
                       eid, "IDX", index, "CC", static_cast<unsigned>(code));
            co_return false;
        }

        const CategoryDescription& description = descriptions.front();
        lg2::debug("EID {EID}: category {IDX} '{NAME}' id={ID} "
                   "elements={ELEM} instances={INST} length={LEN}",
                   "EID", eid, "IDX", index, "NAME", description.name, "ID",
                   description.categoryId, "ELEM", description.elementCount,
                   "INST", description.instanceCount, "LEN",
                   description.categoryLength);

        categories.push_back(
            Category{.description = description, .uniqueIds = {}});
    }

    for (size_t index = 0; index < categories.size(); ++index)
    {
        response = co_await sendRecv(
            encodeCategoryTagsRequest(static_cast<uint8_t>(index)), "tags");
        if (!response)
        {
            co_return false;
        }

        std::vector<uint32_t> uniqueIds;
        if (!decodeCategoryTagsResponse(*response, code, uniqueIds) ||
            code != CompletionCode::success)
        {
            lg2::error("EID {EID}: tags {IDX} rejected, cc={CC}", "EID", eid,
                       "IDX", index, "CC", static_cast<unsigned>(code));
            co_return false;
        }

        lg2::debug("EID {EID}: category {IDX} has {COUNT} tags", "EID", eid,
                   "IDX", index, "COUNT", uniqueIds.size());

        categories[index].uniqueIds = std::move(uniqueIds);
    }

    co_return true;
}

auto AmdBulkDevice::pollOnce() -> sdbusplus::async::task<bool>
{
    auto response =
        co_await sendRecv(encodeOpenTelemetryDataRequest(), "open telemetry");
    if (!response)
    {
        co_return false;
    }

    CompletionCode code{};
    if (!decodeOpenTelemetryDataResponse(*response, code) ||
        code != CompletionCode::success)
    {
        lg2::error("EID {EID}: open telemetry cc={CC}", "EID", eid, "CC",
                   static_cast<unsigned>(code));
        co_return false;
    }

    for (size_t index = 0; index < categories.size() && !stopped; ++index)
    {
        if (!co_await readCategory(index))
        {
            break;
        }
    }

    if (stopped)
    {
        co_return true;
    }

    response =
        co_await sendRecv(encodeCloseTelemetryDataRequest(), "close telemetry");
    if (response && (!decodeCloseTelemetryDataResponse(*response, code) ||
                     code != CompletionCode::success))
    {
        lg2::error("EID {EID}: close telemetry cc={CC}", "EID", eid, "CC",
                   static_cast<unsigned>(code));
    }

    co_return true;
}

auto AmdBulkDevice::readCategory(size_t index) -> sdbusplus::async::task<bool>
{
    const size_t instanceCount = categories[index].description.instanceCount;

    for (size_t instance = 0; instance < instanceCount; ++instance)
    {
        // The devices only accept one whole instance per request.
        auto response = co_await sendRecv(
            encodeTelemetryDataRequest(static_cast<uint8_t>(index), false,
                                       static_cast<uint8_t>(instance), 0, 1),
            "telemetry read");
        if (!response)
        {
            co_return false;
        }

        CompletionCode code{};
        std::vector<uint8_t> data;
        if (!decodeTelemetryDataResponse(*response, code, data) ||
            code != CompletionCode::success)
        {
            lg2::error("EID {EID}: telemetry read cc={CC}", "EID", eid, "CC",
                       static_cast<unsigned>(code));
            co_return false;
        }

        const Category& category = categories[index];
        lg2::debug("EID {EID}: '{NAME}' instance {INST} -> {BYTES} bytes",
                   "EID", eid, "NAME", category.description.name, "INST",
                   instance, "BYTES", data.size());

        updateData(category, instance, data);
    }

    co_return true;
}

// Every tag in a category shares a UID prefix that identifies the source.
void AmdBulkDevice::updateData(const Category& category, size_t instance,
                               std::span<const uint8_t> data)
{
    if (!adapter || category.uniqueIds.empty())
    {
        return;
    }

    switch (category.uniqueIds.front() >> 16U)
    {
        case osfp_stats::categoryUidPrefix:
            osfp_stats::update(*adapter, instance, data);
            break;
        default:
            break;
    }
}

} // namespace ocp::bulk
