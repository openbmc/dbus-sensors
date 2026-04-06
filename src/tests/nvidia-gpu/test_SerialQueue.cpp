/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SerialQueue.hpp"

#include <boost/asio/io_context.hpp>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{

class SerialQueueTest : public ::testing::Test
{
  protected:
    boost::asio::io_context io;
    std::shared_ptr<SerialQueue> queue = std::make_shared<SerialQueue>(io);

    // poll() leaves the io_context stopped once drained; restart() before each
    // poll so successive pumps process newly posted handlers.
    void pump()
    {
        io.restart();
        io.poll();
    }
};

TEST_F(SerialQueueTest, SubmitOnIdleRunsImmediately)
{
    bool ran = false;
    queue->submit([&ran](SerialQueue::ReleaseHandle /*handle*/) {
        ran = true;
    });
    EXPECT_TRUE(ran);
}

TEST_F(SerialQueueTest, SecondSubmitQueuedUntilFirstReleases)
{
    std::optional<SerialQueue::ReleaseHandle> heldA;
    bool ranA = false;
    bool ranB = false;

    queue->submit([&](SerialQueue::ReleaseHandle handle) {
        ranA = true;
        heldA = std::move(handle); // keep the slot reserved
    });
    queue->submit([&](SerialQueue::ReleaseHandle /*handle*/) { ranB = true; });

    EXPECT_TRUE(ranA);
    EXPECT_FALSE(ranB); // B is queued behind A

    heldA.reset();      // release A's slot -> posts B
    EXPECT_FALSE(ranB); // dispatch is via io.post(), not inline
    io.poll();
    EXPECT_TRUE(ranB);
}

TEST_F(SerialQueueTest, ReleaseDispatchesViaPost)
{
    std::optional<SerialQueue::ReleaseHandle> heldA;
    bool ranB = false;

    queue->submit([&](SerialQueue::ReleaseHandle handle) {
        heldA = std::move(handle);
    });
    queue->submit([&](SerialQueue::ReleaseHandle /*handle*/) { ranB = true; });

    heldA.reset();
    EXPECT_FALSE(ranB); // not run until the io_context is pumped
    io.poll();
    EXPECT_TRUE(ranB);
}

TEST_F(SerialQueueTest, TasksRunInFifoOrder)
{
    std::vector<int> order;
    std::optional<SerialQueue::ReleaseHandle> held;

    auto makeTask = [&](int id) {
        return [&order, &held, id](SerialQueue::ReleaseHandle handle) {
            order.push_back(id);
            held = std::move(handle);
        };
    };

    queue->submit(makeTask(1));
    queue->submit(makeTask(2));
    queue->submit(makeTask(3));

    // Task 1 ran inline; release sequentially, pumping between each.
    held.reset();
    pump();
    held.reset();
    pump();
    held.reset();
    pump();

    ASSERT_EQ(order.size(), 3U);
    EXPECT_EQ(order[0], 1);
    EXPECT_EQ(order[1], 2);
    EXPECT_EQ(order[2], 3);
}

TEST_F(SerialQueueTest, ReleaseHandleMoveTransfersOwnership)
{
    std::optional<SerialQueue::ReleaseHandle> heldA;
    bool ranB = false;

    queue->submit([&](SerialQueue::ReleaseHandle handle) {
        heldA = std::move(handle);
    });
    queue->submit([&](SerialQueue::ReleaseHandle /*handle*/) { ranB = true; });

    // Move ownership to another handle; the moved-from slot must not release.
    std::optional<SerialQueue::ReleaseHandle> heldMoved = std::move(heldA);
    heldA.reset(); // moved-from -> no-op
    pump();
    EXPECT_FALSE(ranB);

    // Releasing the owning handle dispatches B exactly once.
    heldMoved.reset();
    pump();
    EXPECT_TRUE(ranB);
}

TEST_F(SerialQueueTest, EmptyQueueReleaseResetsInFlight)
{
    bool ranFirst = false;
    queue->submit([&ranFirst](SerialQueue::ReleaseHandle /*handle*/) {
        ranFirst = true;
    });
    EXPECT_TRUE(ranFirst);

    // Queue returned to idle; a later submit must again run inline.
    bool ranSecond = false;
    queue->submit([&ranSecond](SerialQueue::ReleaseHandle /*handle*/) {
        ranSecond = true;
    });
    EXPECT_TRUE(ranSecond);
}

TEST_F(SerialQueueTest, DestroyQueueWithPendingTasksNoCrash)
{
    std::optional<SerialQueue::ReleaseHandle> held;

    queue->submit([&](SerialQueue::ReleaseHandle handle) {
        held = std::move(handle);
    });
    queue->submit([](SerialQueue::ReleaseHandle /*handle*/) {});

    // Drop the queue while a handle is still outstanding.
    queue.reset();
    EXPECT_NO_THROW(held.reset()); // ReleaseHandle::reset no-ops if queue gone
    EXPECT_NO_THROW(io.poll());
}

} // namespace
