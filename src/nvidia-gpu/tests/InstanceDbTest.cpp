#include "InstanceDb.hpp"

#include <cstdint>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

namespace ocp
{

/** @class TestableInstanceIdDb
 *  @brief Child class of InstanceIdDb for testing protected members
 */
class TestableInstanceIdDb : public InstanceIdDb
{
  public:
    using InstanceIdDb::InstanceIdDb;

    /** @brief Get the max instance IDs for testing */
    static uint8_t getMaxInstanceIds() noexcept
    {
        return maxInstanceIds;
    }

    /** @brief Get the free instance IDs for a given EID */
    std::unordered_set<uint8_t> getFreeInstanceIds(uint8_t eid) const noexcept
    {
        auto it = freeInstanceIds.find(eid);
        if (it != freeInstanceIds.end())
        {
            return it->second;
        }
        return {};
    }

    /** @brief Check if an EID has been initialized */
    bool isEidInitialized(uint8_t eid) const noexcept
    {
        return freeInstanceIds.contains(eid);
    }

    /** @brief Get all EIDs that have been initialized */
    std::vector<uint8_t> getInitializedEids() const noexcept
    {
        std::vector<uint8_t> eids;
        eids.reserve(freeInstanceIds.size());
        for (const auto& [eid, _] : freeInstanceIds)
        {
            eids.push_back(eid);
        }
        return eids;
    }
};

class InstanceIdDbTest : public testing::Test
{
  protected:
    // Each test will create its own fresh instance
    static TestableInstanceIdDb createDb()
    {
        return TestableInstanceIdDb{};
    }
};

TEST_F(InstanceIdDbTest, Constructor)
{
    // Test that the database is properly initialized
    auto db = createDb();
    EXPECT_EQ(db.getMaxInstanceIds(), 32);
    EXPECT_TRUE(db.getInitializedEids().empty());
}

TEST_F(InstanceIdDbTest, NextFirstAllocation)
{
    auto db = createDb();
    const uint8_t eid = 5;

    // First allocation should return an instance ID
    auto result = db.next(eid);
    EXPECT_TRUE(result.has_value());
    if (result.has_value())
    {
        EXPECT_GE(result.value(), 0);
        EXPECT_LT(result.value(), db.getMaxInstanceIds());
    }

    // EID should be initialized after first allocation
    EXPECT_TRUE(db.isEidInitialized(eid));

    // Should have one less free instance ID
    auto freeIds = db.getFreeInstanceIds(eid);
    EXPECT_EQ(freeIds.size(), db.getMaxInstanceIds() - 1);
}

TEST_F(InstanceIdDbTest, NextMultipleAllocations)
{
    auto db = createDb();
    const uint8_t eid = 10;
    std::set<uint8_t> allocatedIds;

    // Allocate multiple instance IDs
    for (int i = 0; i < 10; ++i)
    {
        auto result = db.next(eid);
        EXPECT_TRUE(result.has_value());
        if (result.has_value())
        {
            allocatedIds.insert(result.value());
        }
    }

    // All allocated IDs should be unique
    EXPECT_EQ(allocatedIds.size(), 10);

    // All allocated IDs should be within valid range
    for (auto id : allocatedIds)
    {
        EXPECT_GE(id, 0);
        EXPECT_LT(id, db.getMaxInstanceIds());
    }

    // Should have correct number of free IDs remaining
    auto freeIds = db.getFreeInstanceIds(eid);
    EXPECT_EQ(freeIds.size(), db.getMaxInstanceIds() - 10);
}

TEST_F(InstanceIdDbTest, NextExhaustAllIds)
{
    auto db = createDb();
    const uint8_t eid = 15;
    std::vector<uint8_t> allocatedIds;

    // Allocate all available instance IDs
    for (int i = 0; i < db.getMaxInstanceIds(); ++i)
    {
        auto result = db.next(eid);
        EXPECT_TRUE(result.has_value());
        if (result.has_value())
        {
            allocatedIds.push_back(result.value());
        }
    }

    // Try to allocate one more - should fail
    auto result = db.next(eid);
    EXPECT_FALSE(result.has_value());

    // All allocated IDs should be unique
    std::set<uint8_t> uniqueIds(allocatedIds.begin(), allocatedIds.end());
    EXPECT_EQ(uniqueIds.size(), db.getMaxInstanceIds());
}

TEST_F(InstanceIdDbTest, NextMultipleEids)
{
    auto db = createDb();
    const uint8_t eid1 = 20;
    const uint8_t eid2 = 25;

    // Allocate from first EID
    auto result1 = db.next(eid1);
    EXPECT_TRUE(result1.has_value());

    // Allocate from second EID
    auto result2 = db.next(eid2);
    EXPECT_TRUE(result2.has_value());

    // Both EIDs should be initialized
    EXPECT_TRUE(db.isEidInitialized(eid1));
    EXPECT_TRUE(db.isEidInitialized(eid2));

    // Both should have correct number of free IDs
    auto freeIds1 = db.getFreeInstanceIds(eid1);
    auto freeIds2 = db.getFreeInstanceIds(eid2);
    EXPECT_EQ(freeIds1.size(), db.getMaxInstanceIds() - 1);
    EXPECT_EQ(freeIds2.size(), db.getMaxInstanceIds() - 1);
}

TEST_F(InstanceIdDbTest, FreeValidInstanceId)
{
    auto db = createDb();
    const uint8_t eid = 30;
    const uint8_t instanceId = 5;

    // First allocate an instance ID
    auto result = db.next(eid);
    EXPECT_TRUE(result.has_value());

    // Free a specific instance ID
    db.free(eid, instanceId);

    // The freed instance ID should be available again
    auto freeIds = db.getFreeInstanceIds(eid);
    EXPECT_TRUE(freeIds.contains(instanceId));
}

TEST_F(InstanceIdDbTest, FreeInvalidInstanceId)
{
    auto db = createDb();
    const uint8_t eid = 35;

    // Try to free an instance ID that was never allocated
    db.free(eid, 10);

    // Should not cause any issues, but the ID should not be in free list
    // since the EID was never initialized
    EXPECT_FALSE(db.isEidInitialized(eid));
}

TEST_F(InstanceIdDbTest, FreeOutOfRangeInstanceId)
{
    auto db = createDb();
    const uint8_t eid = 40;

    // Initialize the EID by allocating one ID
    auto result = db.next(eid);
    EXPECT_TRUE(result.has_value());

    // Try to free an out-of-range instance ID
    db.free(eid, db.getMaxInstanceIds() + 1);

    // Should not affect the free list
    auto freeIds = db.getFreeInstanceIds(eid);
    EXPECT_EQ(freeIds.size(), db.getMaxInstanceIds() - 1);
}

TEST_F(InstanceIdDbTest, FreeAlreadyFreedInstanceId)
{
    auto db = createDb();
    const uint8_t eid = 45;
    const uint8_t instanceId = 7;

    // Allocate an instance ID
    auto result = db.next(eid);
    EXPECT_TRUE(result.has_value());

    // Free it
    db.free(eid, instanceId);

    // Free it again - should be safe but not add duplicate
    db.free(eid, instanceId);

    // Should only have one instance of the freed ID
    auto freeIds = db.getFreeInstanceIds(eid);
    EXPECT_EQ(freeIds.count(instanceId), 1);
}

TEST_F(InstanceIdDbTest, StressTestMultipleEids)
{
    auto db = createDb();
    const std::vector<uint8_t> eids = {1, 5, 10, 15, 20, 25, 30, 35, 40, 45};
    std::map<uint8_t, std::vector<uint8_t>> allocatedPerEid;

    // Allocate 3 IDs from each EID
    for (auto eid : eids)
    {
        for (int i = 0; i < 3; ++i)
        {
            auto result = db.next(eid);
            EXPECT_TRUE(result.has_value());
            if (result.has_value())
            {
                allocatedPerEid[eid].push_back(result.value());
            }
        }
    }

    // Verify all EIDs are initialized
    for (auto eid : eids)
    {
        EXPECT_TRUE(db.isEidInitialized(eid));
        auto freeIds = db.getFreeInstanceIds(eid);
        EXPECT_EQ(freeIds.size(), db.getMaxInstanceIds() - 3);
    }

    // Free some IDs from each EID
    for (auto eid : eids)
    {
        if (!allocatedPerEid[eid].empty())
        {
            db.free(eid, allocatedPerEid[eid][0]);
        }
    }

    // Allocate one more from each EID
    for (auto eid : eids)
    {
        auto result = db.next(eid);
        EXPECT_TRUE(result.has_value());
    }
}

} // namespace ocp
