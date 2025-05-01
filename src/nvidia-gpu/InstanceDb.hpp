/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2024 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <phosphor-logging/lg2.hpp>

#include <cstdint>
#include <expected>
#include <map>
#include <system_error>
#include <unordered_set>

namespace ocp
{

/** @class InstanceIdDb
 *  @brief Implementation of OCP instance id with simple in-memory allocation
 */
class InstanceIdDb
{
  public:
    InstanceIdDb() noexcept = default;

    ~InstanceIdDb() noexcept = default;

    /** @brief Allocate an instance ID for the given EID
     *  @param[in] eid - the EID the instance ID is associated with
     *  @return - instance id or std::error_code if there was a failure while
     *  instance id allocation.
     */
    [[nodiscard]] std::expected<uint8_t, std::error_code> next(
        const uint8_t eid) noexcept
    {
        if (!freeInstanceIds.contains(eid))
        {
            initInstanceIds(eid);
        }

        if (freeInstanceIds[eid].empty())
        {
            return std::unexpected(std::make_error_code(
                std::errc::resource_unavailable_try_again));
        }

        const auto id = *freeInstanceIds[eid].begin();
        freeInstanceIds[eid].erase(id);

        return id;
    }

    /** @brief Mark an instance id as unused
     *  @param[in] eid - the EID the instance ID is associated with
     *  @param[in] instanceId - OCP instance id to be freed
     */
    void free(const uint8_t eid, uint8_t instanceId) noexcept
    {
        if (instanceId >= maxInstanceIds || !freeInstanceIds.contains(eid) ||
            freeInstanceIds[eid].contains(instanceId))
        {
            lg2::error(
                "Invalid Instance ID, {ID}, was asked to be freed for EID: {EID}",
                "ID", static_cast<int>(instanceId), "EID",
                static_cast<int>(eid));
            return;
        }
        freeInstanceIds[eid].insert(instanceId);
    }

  private:
    void initInstanceIds(const uint8_t eid) noexcept
    {
        for (auto id = 0; id < maxInstanceIds; ++id)
        {
            freeInstanceIds[eid].insert(id);
        }
    }

  protected:
    static constexpr uint8_t maxInstanceIds = 32;

    // Map of EID to an unordered_set of unallocated instance IDs
    std::map<uint8_t, std::unordered_set<uint8_t>> freeInstanceIds;
};

} // namespace ocp
