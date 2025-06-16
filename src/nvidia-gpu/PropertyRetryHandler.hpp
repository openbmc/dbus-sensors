#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <phosphor-logging/lg2.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

class PropertyRetryHandler
{
  public:
    using RetryCallback = std::function<void()>;
    using ErrorCallback = std::function<void(const std::string&)>;

    PropertyRetryHandler(boost::asio::io_context& io) :
        retryTimer(io), retryCount(0)
    {}

    // Generic retry logic that can be used by any property type
    bool handleRetry(bool success, const std::string& propertyName,
                     const RetryCallback& retryFunc,
                     const ErrorCallback& errorFunc = nullptr)
    {
        if (!success)
        {
            retryCount++;
            if (retryCount >= maxRetryAttempts)
            {
                lg2::error("{PROPERTY} failed after {ATTEMPTS} attempts",
                           "PROPERTY", propertyName, "ATTEMPTS",
                           maxRetryAttempts);
                if (errorFunc)
                {
                    errorFunc("Max retry attempts exceeded");
                }
                return false;
            }
            else
            {
                retryTimer.expires_after(retryDelay);
                retryTimer.async_wait(
                    [retryFunc](const boost::system::error_code& ec) {
                        if (ec)
                        {
                            lg2::error("Retry timer error: {ERROR}", "ERROR",
                                       ec.message());
                            return;
                        }
                        retryFunc();
                    });
                return true; // Indicates retry is scheduled
            }
        }
        else
        {
            retryCount = 0;
            return false; // No retry needed
        }
    }

    void resetRetryCount()
    {
        retryCount = 0;
    }

    int getRetryCount() const
    {
        return retryCount;
    }

    bool isRetrying() const
    {
        return retryCount > 0 && retryCount < maxRetryAttempts;
    }

  private:
    boost::asio::steady_timer retryTimer;
    int retryCount;
    static constexpr std::chrono::seconds retryDelay{5};
    static constexpr int maxRetryAttempts = 3;
};
