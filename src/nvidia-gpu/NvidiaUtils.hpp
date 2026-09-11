/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/system/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

constexpr const char* metricPath = "/xyz/openbmc_project/metric/";
constexpr const char* sensorPathPrefix = "/xyz/openbmc_project/sensors/";

// Units for the Value property of xyz.openbmc_project.Metric.Value.
inline const std::string metricUnitBytes =
    "xyz.openbmc_project.Metric.Value.Unit.Bytes";
inline const std::string metricUnitCount =
    "xyz.openbmc_project.Metric.Value.Unit.Count";

// DRAM inventory name segment, joined to a GPU name with a '_' separator to
// form its DRAM inventory object, e.g. Nvidia_GPU_0 -> Nvidia_GPU_0_DRAM_0
constexpr const char* dramInventorySuffix = "DRAM_0";

constexpr const char* nvidiaManufacturer = "NVIDIA";

inline const sdbusplus::object_path inventoryPrefix{
    "/xyz/openbmc_project/inventory"};

// An object located through the mapper is named by path alone, so the service
// holding it has to be resolved before anything on it can be read.
//
// Calls done with the service that owns path and the interfaces that service
// carries on it. A path can be owned by more than one service, and the mapper
// owns the ones it holds associations for without implementing anything on
// them, so the owner reported is the first that carries an interface. The
// service name is empty when the mapper fails or names none, which lets a
// caller say what it was looking for rather than being told here.
inline void resolveObjectService(
    const std::shared_ptr<sdbusplus::asio::connection>& conn,
    const sdbusplus::object_path& path,
    std::function<void(const std::string& service,
                       const std::vector<std::string>& interfaces)>
        done)
{
    conn->async_method_call(
        [path, done{std::move(done)}](
            const boost::system::error_code& ec,
            const std::vector<std::pair<std::string, std::vector<std::string>>>&
                owners) {
            if (ec)
            {
                lg2::error("Failed to find the service owning {PATH}: {ERROR}",
                           "PATH", path, "ERROR", ec.message());
                done({}, {});
                return;
            }

            const auto owner = std::ranges::find_if(owners, [](const auto& o) {
                return !o.second.empty();
            });
            if (owner == owners.end())
            {
                done({}, {});
                return;
            }
            done(owner->first, owner->second);
        },
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetObject", path.string(),
        std::vector<std::string>{});
}
