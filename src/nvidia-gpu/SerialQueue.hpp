/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <boost/asio/io_context.hpp>

#include <functional>
#include <memory>
#include <queue>
#include <utility>

class SerialQueue : public std::enable_shared_from_this<SerialQueue>
{
  public:
    class ReleaseHandle
    {
      public:
        ReleaseHandle() = default;
        ReleaseHandle(const ReleaseHandle&) = delete;
        ReleaseHandle& operator=(const ReleaseHandle&) = delete;

        explicit ReleaseHandle(std::weak_ptr<SerialQueue> q) :
            queue(std::move(q))
        {}

        ReleaseHandle(ReleaseHandle&& other) noexcept :
            queue(std::move(other.queue))
        {}

        ReleaseHandle& operator=(ReleaseHandle&& other) noexcept
        {
            if (this != &other)
            {
                reset();
                queue = std::move(other.queue);
            }
            return *this;
        }

        ~ReleaseHandle()
        {
            reset();
        }

      private:
        void reset();

        std::weak_ptr<SerialQueue> queue;
    };

    using Task = std::move_only_function<void(ReleaseHandle)>;

    explicit SerialQueue(boost::asio::io_context& io);

    // Run the task now if the queue is idle; otherwise enqueue it. When
    // dispatched, the task receives a move-only ReleaseHandle. The slot stays
    // reserved until that handle is destroyed, so the task must keep the
    // handle alive for as long as it considers itself "in flight" — typically
    // by moving it into the capture list of any continuation lambda. Letting
    // the handle fall out of scope is what releases the slot and dispatches the
    // next queued task.
    void submit(Task task);

  private:
    void release();

    boost::asio::io_context& io;
    bool inFlight{false};
    std::queue<Task> pending;
};
