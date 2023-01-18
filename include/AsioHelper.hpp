#pragma once

#include <memory>
#include <stdexcept>
#include <utility>

namespace asio_helper
{

// Wraps a non-copyable callable and makes it copyable (moving into
// a shared_ptr). It throws a runtime_error if called twice.
//
// Can be used to capture a non-copyable callable in a lambda, where
// the lambda itself needs to be copyable so it can be used with std::function.
//
// This is intended to use a boost::asio::basic_yield_context completion token
// as type F type. It will be called with a single argument.
template <typename F>
struct CopyableCallback
{
  public:
    CopyableCallback(F&& cb) : cb(std::make_shared<F>(std::move(cb)))
    {}

    template <typename R>
    void operator()(R&& r)
    {
        if (!cb)
        {
            throw std::runtime_error("CopyableCallback was called twice");
        }
        (*cb)(std::move(r));
        cb.reset();
    }

  private:
    std::shared_ptr<F> cb;
};

} // namespace asio_helper
