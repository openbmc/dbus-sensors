#include "MCTPReactor.hpp"

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/system/detail/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <system_error>

PHOSPHOR_LOG2_USING;

void MCTPReactor::deferSetup(const std::shared_ptr<MCTPDevice>& dev)
{
    debug("Deferring setup for MCTP device at [ {MCTP_DEVICE} ]", "MCTP_DEVICE",
          dev->describe());

    deferred.emplace(dev);
}

void MCTPReactor::untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep)
{
    server.disassociate(MCTPDEndpoint::path(ep));
}

void MCTPReactor::trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep)
{
    info("Added MCTP endpoint to device: [ {MCTP_ENDPOINT} ]", "MCTP_ENDPOINT",
         ep->describe());

    ep->subscribe(
        // Degraded
        [](const std::shared_ptr<MCTPEndpoint>&) {},
        // Available
        [](const std::shared_ptr<MCTPEndpoint>&) {},
        // Removed
        [weak{weak_from_this()}](const std::shared_ptr<MCTPEndpoint>& ep) {
        info("Removed MCTP endpoint from device: [ {MCTP_ENDPOINT} ]",
             "MCTP_ENDPOINT", ep->describe());
        if (auto self = weak.lock())
        {
            self->untrackEndpoint(ep);
            // Only defer the setup if we know inventory is still present
            if (self->devices.contains(ep->device()))
            {
                self->deferSetup(ep->device());
            }
        }
    });

    // Proxy-host the association back to the inventory at the same path as the
    // endpoint in mctpd.
    //
    // clang-format off
    // ```
    // # busctl call xyz.openbmc_project.ObjectMapper /xyz/openbmc_project/object_mapper xyz.openbmc_project.ObjectMapper GetAssociatedSubTree ooias /xyz/openbmc_project/mctp/1/9/configured_by / 0 1 xyz.openbmc_project.Configuration.MCTPDevice
    // a{sa{sas}} 1 "/xyz/openbmc_project/inventory/system/nvme/NVMe_1/NVMe_1_Temp" 1 "xyz.openbmc_project.EntityManager" 1 "xyz.openbmc_project.Configuration.MCTPDevice"
    // ```
    // clang-format on
    const std::string& item = devices.inventoryFor(ep->device());
    std::vector<Association> associations{
        {"configured_by", "configures", item}};
    server.associate(MCTPDEndpoint::path(ep), associations);
}

void MCTPReactor::setupEndpoint(const std::shared_ptr<MCTPDevice>& dev)
{
    debug(
        "Attempting to setup up MCTP endpoint for device at [ {MCTP_DEVICE} ]",
        "MCTP_DEVICE", dev->describe());
    dev->setup([weak{weak_from_this()},
                dev](const std::error_code& ec,
                     const std::shared_ptr<MCTPEndpoint>& ep) mutable {
        auto self = weak.lock();
        if (!self)
        {
            return;
        }

        if (ec)
        {
            debug(
                "Setup failed for MCTP device at [ {MCTP_DEVICE} ]: {ERROR_MESSAGE}",
                "MCTP_DEVICE", dev->describe(), "ERROR_MESSAGE", ec.message());

            self->deferSetup(dev);
            return;
        }

        self->trackEndpoint(ep);
    });
}

void MCTPReactor::tick()
{
    auto toSetup = std::exchange(deferred, {});
    for (const auto& entry : toSetup)
    {
        setupEndpoint(entry);
    }
}

void MCTPReactor::manageMCTPDevice(const std::string& path,
                                   const std::shared_ptr<MCTPDevice>& device)
{
    if (!device)
    {
        return;
    }

    debug("MCTP device inventory added at '{INVENTORY_PATH}'", "INVENTORY_PATH",
          path);

    devices.add(path, device);

    setupEndpoint(device);
}

void MCTPReactor::unmanageMCTPDevice(const std::string& path)
{
    if (!devices.contains(path))
    {
        return;
    }

    std::shared_ptr<MCTPDevice> device = devices.deviceFor(path);

    debug("MCTP device inventory removed at '{INVENTORY_PATH}'",
          "INVENTORY_PATH", path);

    deferred.erase(device);

    // Remove the device from the repository before notifying the device itself
    // of removal so we don't defer its setup
    devices.remove(device);

    debug("Stopping management of MCTP device at [ {MCTP_DEVICE} ]",
          "MCTP_DEVICE", device->describe());

    device->remove();
}
