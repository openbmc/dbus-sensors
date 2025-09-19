#include "MCTPReactor.hpp"

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "utils/Utils.hpp"

#include <boost/system/detail/error_code.hpp>
#include <phosphor-logging/lg2.hpp>

#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

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
        [](const std::shared_ptr<MCTPEndpoint>& ep) {
            debug("Endpoint entered degraded state: [ {MCTP_ENDPOINT} ]",
                  "MCTP_ENDPOINT", ep->describe());
        },
        // Available
        [](const std::shared_ptr<MCTPEndpoint>& ep) {
            debug("Endpoint entered available state: [ {MCTP_ENDPOINT} ]",
                  "MCTP_ENDPOINT", ep->describe());
        },
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
            else
            {
                info(
                    "The reactor object was destroyed concurrent to the removal of the remove match for the endpoint '{MCTP_ENDPOINT}'",
                    "MCTP_ENDPOINT", ep->describe());
            }
        });

    // Proxy-host the association back to the inventory at the same path as the
    // endpoint in mctpd.
    //
    // clang-format off
    // ```
    // # busctl call xyz.openbmc_project.ObjectMapper /xyz/openbmc_project/object_mapper xyz.openbmc_project.ObjectMapper GetAssociatedSubTree ooias /au/com/codeconstruct/mctp1/networks/1/endpoints/9/configured_by / 0 1 xyz.openbmc_project.Configuration.MCTPDevice
    // a{sa{sas}} 1 "/xyz/openbmc_project/inventory/system/nvme/NVMe_1/NVMe_1_Temp" 1 "xyz.openbmc_project.EntityManager" 1 "xyz.openbmc_project.Configuration.MCTPDevice"
    // ```
    // clang-format on
    std::optional<std::string> item = devices.inventoryFor(ep->device());
    if (!item)
    {
        error("Inventory missing for endpoint: [ {MCTP_ENDPOINT} ]",
              "MCTP_ENDPOINT", ep->describe());
        return;
    }
    std::vector<Association> associations{
        {"configured_by", "configures", *item}};
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
            info(
                "The reactor object was destroyed concurrent to the completion of the endpoint setup");
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

        try
        {
            self->trackEndpoint(ep);
        }
        catch (const MCTPException& e)
        {
            error("Failed to track endpoint '{MCTP_ENDPOINT}': {EXCEPTION}",
                  "MCTP_ENDPOINT", ep->describe(), "EXCEPTION", e);
            self->deferSetup(dev);
        }
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

    try
    {
        devices.add(path, device);
        debug("MCTP device inventory added at '{INVENTORY_PATH}'",
              "INVENTORY_PATH", path);
        setupEndpoint(device);
    }
    catch (const std::system_error& e)
    {
        if (e.code() != std::errc::device_or_resource_busy)
        {
            throw e;
        }

        auto current = devices.deviceFor(path);
        if (!current)
        {
            warning(
                "Invalid state: Failed to manage device for inventory at '{INVENTORY_PATH}', but the inventory item is unrecognised",
                "INVENTORY_PATH", path);
            return;
        }

        // TODO: Ensure remove completion happens-before add. For now this
        // happens unsynchronised. Make some noise about it.
        warning(
            "Unsynchronised endpoint reinitialsation due to configuration change at '{INVENTORY_PATH}': Removing '{MCTP_DEVICE}'",
            "INVENTORY_PATH", path, "MCTP_DEVICE", current->describe());

        unmanageMCTPDevice(path);

        devices.add(path, device);

        // Pray (this is the unsynchronised bit)
        deferSetup(device);
    }
}

void MCTPReactor::unmanageMCTPDevice(const std::string& path)
{
    auto device = devices.deviceFor(path);
    if (!device)
    {
        debug("Unrecognised inventory item: {INVENTORY_PATH}", "INVENTORY_PATH",
              path);
        return;
    }

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
