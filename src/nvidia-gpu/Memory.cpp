#include "Memory.hpp"

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <boost/asio/io_context.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

static constexpr const char* inventoryPrefix =
    "/xyz/openbmc_project/inventory/";
static constexpr const char* dimmIfaceName =
    "xyz.openbmc_project.Inventory.Item.Dimm";

Memory::Memory(const std::shared_ptr<sdbusplus::asio::connection>& /*conn*/,
               sdbusplus::asio::object_server& objectServer,
               const std::string& name, mctp::MctpRequester& mctpRequester,
               uint8_t eid, boost::asio::io_context& io) :
    path(std::string(inventoryPrefix) + name), mctpRequester(mctpRequester),
    eid(eid), retryTimer(io)
{
    requestBuffer = std::make_shared<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyRequest)>>();
    responseBuffer = std::make_shared<
        std::array<uint8_t, sizeof(gpu::GetCurrentClockFrequencyResponse)>>();

    dimmInterface = objectServer.add_interface(path, dimmIfaceName);
    dimmInterface->register_property("MemorySizeInKB", uint64_t(0));
    dimmInterface->register_property("MemoryConfiguredSpeedInMhz", uint32_t(0));
    dimmInterface->register_property("Manufacturer", std::string("Nvidia"));
    dimmInterface->register_property("MemoryType", std::string{});
    dimmInterface->initialize();
}

void Memory::setMemoryType(const std::string& type)
{
    dimmInterface->set_property("MemoryType", type);
}

void Memory::update()
{
    requestMemorySpeed();
}

void Memory::requestMemorySpeed()
{
    int rc = gpu::encodeGetCurrentClockFrequencyRequest(
        0, static_cast<uint8_t>(gpu::ClockType::MEMORY_CLOCK), *requestBuffer);
    if (rc != 0)
    {
        lg2::error("Failed to encode memory speed request: rc={RC}", "RC", rc);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, *requestBuffer, *responseBuffer,
        [self = weak_from_this()](int sendRecvMsgResult) {
            if (auto memory = self.lock())
            {
                memory->handleMemorySpeedResponse(sendRecvMsgResult);
            }
        });
}

void Memory::handleMemorySpeedResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error("Failed to get memory speed: rc={RC}", "RC",
                   sendRecvMsgResult);
        retryCount++;
        if (retryCount < maxRetryAttempts)
        {
            retryTimer.expires_after(retryDelay);
            retryTimer.async_wait(
                [self = weak_from_this()](const boost::system::error_code& ec) {
                    if (ec)
                    {
                        lg2::error("Retry timer error: {ERROR}", "ERROR",
                                   ec.message());
                        return;
                    }
                    if (auto memory = self.lock())
                    {
                        memory->requestMemorySpeed();
                    }
                });
        }
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    uint32_t clockFrequency = 0;
    int rc = gpu::decodeGetCurrentClockFrequencyResponse(
        *responseBuffer, cc, reasonCode, clockFrequency);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error("Failed to decode memory speed response: rc={RC}, cc={CC}",
                   "RC", rc, "CC", static_cast<int>(cc));
        retryCount++;
        if (retryCount < maxRetryAttempts)
        {
            retryTimer.expires_after(retryDelay);
            retryTimer.async_wait(
                [self = weak_from_this()](const boost::system::error_code& ec) {
                    if (ec)
                    {
                        lg2::error("Retry timer error: {ERROR}", "ERROR",
                                   ec.message());
                        return;
                    }
                    if (auto memory = self.lock())
                    {
                        memory->requestMemorySpeed();
                    }
                });
        }
        return;
    }

    // Clock frequency is already in MHz
    dimmInterface->set_property("MemoryConfiguredSpeedInMhz", clockFrequency);
    retryCount = 0;
}
