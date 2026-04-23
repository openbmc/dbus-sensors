/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SerialQueue.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>

#include <memory>
#include <utility>

void SerialQueue::ReleaseHandle::reset()
{
    if (auto q = queue.lock())
    {
        queue.reset();
        q->release();
    }
}

SerialQueue::SerialQueue(boost::asio::io_context& io) : io(io) {}

void SerialQueue::submit(Task task)
{
    if (inFlight)
    {
        pending.push(std::move(task));
        return;
    }

    inFlight = true;
    task(ReleaseHandle{weak_from_this()});
}

void SerialQueue::release()
{
    if (pending.empty())
    {
        inFlight = false;
        return;
    }

    Task next = std::move(pending.front());
    pending.pop();

    boost::asio::post(io, [self = shared_from_this(),
                           task = std::move(next)]() mutable {
        task(ReleaseHandle{std::weak_ptr<SerialQueue>{self}});
    });
}
