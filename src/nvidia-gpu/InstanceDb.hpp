/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2024 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <map>
#include <optional>
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
     *  @return - instance id or std::nullopt if there are no available instance
     * IDs
     */
    [[nodiscard]] std::optional<uint8_t> next(uint8_t eid) noexcept
    {
        if (!freeInstanceIds.contains(eid))
        {
            initInstanceIds(eid);
        }

        if (freeInstanceIds[eid].empty())
        {
            return std::nullopt;
        }

        const auto id = *freeInstanceIds[eid].begin();
        freeInstanceIds[eid].erase(id);

        return id;
    }

    /** @brief Mark an instance id as unused
     *  @param[in] eid - the EID the instance ID is associated with
     *  @param[in] instanceId - OCP instance id to be freed
     *  @return - true if the instance ID was successfully freed, false
     * otherwise
     */
    void free(uint8_t eid, uint8_t instanceId) noexcept
    {
        if (instanceId >= maxInstanceIds || !freeInstanceIds.contains(eid) ||
            freeInstanceIds[eid].contains(instanceId))
        {
            // Invalid instance id was asked to be freed.
            return;
        }
        freeInstanceIds[eid].insert(instanceId);
    }

  private:
    void initInstanceIds(uint8_t eid) noexcept
    {
        for (auto id = 0; id < maxInstanceIds; ++id)
        {
            freeInstanceIds[eid].insert(id);
        }
    }

  protected:
    static constexpr uint8_t maxInstanceIds = 32;

    // Map of EID to a unordered_set of unallocated instance IDs
    std::map<uint8_t, std::unordered_set<uint8_t>> freeInstanceIds;
};

} // namespace ocp
