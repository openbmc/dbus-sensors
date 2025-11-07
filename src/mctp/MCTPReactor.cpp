#include "MCTPReactor.hpp"

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <boost/system/detail/error_code.hpp>
#include <phosphor-logging/lg2.hpp>
#include <phosphor-logging/lg2/flags.hpp>

#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

PHOSPHOR_LOG2_USING;

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
                switch (self->states[ep->device()->id()])
                {
                    case MCTPDeviceState::Unmanaged:
                    case MCTPDeviceState::Assigning:
                    case MCTPDeviceState::Unassigned:
                        break;
                    case MCTPDeviceState::Assigned:
                        self->next(ep->device(), MCTPDeviceState::Lost);
                        break;
                    case MCTPDeviceState::Quarantine:
                        self->terminate(ep->device());
                        break;
                    case MCTPDeviceState::Lost:
                    case MCTPDeviceState::Recovering:
                        break;
                    case MCTPDeviceState::Recovered:
                        self->next(ep->device(), MCTPDeviceState::Lost);
                        break;
                    case MCTPDeviceState::Removing:
                        // If the configuration has been replaced then we've
                        // already terminated the state tracking
                        if (self->devices.contains(ep->device()))
                        {
                            self->terminate(ep->device());
                        }
                        break;
                    case MCTPDeviceState::Pending:
                        self->next(ep->device(), MCTPDeviceState::Unassigned);
                        break;
                }
            }
            else
            {
                info(
                    "The reactor object was destroyed concurrent to the removal of the remove match for the endpoint '{MCTP_ENDPOINT}'",
                    "MCTP_ENDPOINT", ep->describe());
            }
        });

    switch (states[ep->device()->id()])
    {
        case MCTPDeviceState::Unmanaged:
            return;
        case MCTPDeviceState::Assigning:
            next(ep->device(), MCTPDeviceState::Assigned);
            break;
        case MCTPDeviceState::Unassigned:
        case MCTPDeviceState::Assigned:
        case MCTPDeviceState::Quarantine:
            next(ep->device(), MCTPDeviceState::Recovered);
            break;
        case MCTPDeviceState::Lost:
            return;
        case MCTPDeviceState::Recovering:
            next(ep->device(), MCTPDeviceState::Recovered);
            break;
        case MCTPDeviceState::Recovered:
        case MCTPDeviceState::Removing:
        case MCTPDeviceState::Pending:
            return;
    }

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
    dev->setup([weak{weak_from_this()}, wdev = std::weak_ptr<MCTPDevice>(dev)](
                   const std::error_code& ec,
                   const std::shared_ptr<MCTPEndpoint>& ep) mutable {
        auto self = weak.lock();
        if (!self)
        {
            info(
                "The reactor object was destroyed concurrent to the completion of the endpoint setup");
            return;
        }

        auto dev = wdev.lock();
        if (!dev)
        {
            info(
                "The device was destroyed concurrent to the completion of endpoint setup");
            return;
        }

        if (ec)
        {
            debug(
                "Setup failed for MCTP device at [ {MCTP_DEVICE} ], deferring: {ERROR_MESSAGE}",
                "MCTP_DEVICE", dev->describe(), "ERROR_MESSAGE", ec.message());

            switch (self->states[dev->id()])
            {
                case MCTPDeviceState::Unmanaged:
                    break;
                case MCTPDeviceState::Assigning:
                    self->next(dev, MCTPDeviceState::Unassigned);
                    break;
                case MCTPDeviceState::Unassigned:
                case MCTPDeviceState::Assigned:
                    break;
                case MCTPDeviceState::Quarantine:
                    self->terminate(dev);
                    break;
                case MCTPDeviceState::Lost:
                    break;
                case MCTPDeviceState::Recovering:
                    self->next(dev, MCTPDeviceState::Lost);
                    break;
                case MCTPDeviceState::Recovered:
                case MCTPDeviceState::Removing:
                case MCTPDeviceState::Pending:
                    break;
            }
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
            self->next(dev, MCTPDeviceState::Quarantine);
        }
    });
}

void MCTPReactor::tick()
{
    for (const auto& entry : devices)
    {
        switch (states[entry.second->id()])
        {
            case MCTPDeviceState::Unmanaged:
            case MCTPDeviceState::Assigning:
                break;
            case MCTPDeviceState::Unassigned:
                next(entry.second, MCTPDeviceState::Assigning);
                setupEndpoint(entry.second);
                break;
            case MCTPDeviceState::Assigned:
            case MCTPDeviceState::Quarantine:
                break;
            case MCTPDeviceState::Lost:
                next(entry.second, MCTPDeviceState::Recovering);
                setupEndpoint(entry.second);
                break;
            case MCTPDeviceState::Recovering:
            case MCTPDeviceState::Recovered:
            case MCTPDeviceState::Removing:
            case MCTPDeviceState::Pending:
                break;
        }
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

    if (!states.contains(device->id()))
    {
        debug(
            "Initialising state for device {DEVICE_ID} ([ {DEVICE_DESCRIPTION} ])) as {INITIAL_STATE}",
            "DEVICE_ID", lg2::hex, device->id(), "DEVICE_DESCRIPTION",
            device->describe(), "INITIAL_STATE", MCTPDeviceState::Unmanaged);
        states[device->id()] = MCTPDeviceState::Unmanaged;
    }

    switch (states[device->id()])
    {
        case MCTPDeviceState::Unmanaged:
            devices.add(path, device);
            next(device, MCTPDeviceState::Assigning);
            setupEndpoint(device);
            break;
        case MCTPDeviceState::Assigning:
        case MCTPDeviceState::Unassigned:
            break;
        case MCTPDeviceState::Assigned:
        {
            // EM may publish property changes without removal. Replace the
            // device so its state reflects EM's configuration.
            auto current = devices.deviceFor(path);
            if (!current)
            {
                warning(
                    "Invalid state: Failed to manage device for inventory at '{INVENTORY_PATH}', but the inventory item is unrecognised",
                    "INVENTORY_PATH", path);
                return;
            }

            // Unsynchronised termination so we can configure the new device.
            // Paired with presence test when handling MCTPDeviceState::Removing
            // in the subscribed endpoint removed handler
            terminate(current);
            current->remove();
            manageMCTPDevice(path, device);
            break;
        }
        case MCTPDeviceState::Quarantine:
            next(device, MCTPDeviceState::Assigning);
            break;
        case MCTPDeviceState::Lost:
        case MCTPDeviceState::Recovering:
            break;
        case MCTPDeviceState::Recovered:
            next(device, MCTPDeviceState::Assigned);
            break;
        case MCTPDeviceState::Removing:
            next(device, MCTPDeviceState::Pending);
            break;
        case MCTPDeviceState::Pending:
            break;
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

    switch (states[device->id()])
    {
        case MCTPDeviceState::Unmanaged:
            break;
        case MCTPDeviceState::Assigning:
            next(device, MCTPDeviceState::Quarantine);
            break;
        case MCTPDeviceState::Unassigned:
            terminate(device);
            break;
        case MCTPDeviceState::Assigned:
            debug("Stopping management of MCTP device at [ {MCTP_DEVICE} ]",
                  "MCTP_DEVICE", device->describe());
            next(device, MCTPDeviceState::Removing);
            device->remove();
            break;
        case MCTPDeviceState::Quarantine:
            break;
        case MCTPDeviceState::Lost:
            terminate(device);
            break;
        case MCTPDeviceState::Recovering:
            next(device, MCTPDeviceState::Quarantine);
            break;
        case MCTPDeviceState::Recovered:
        case MCTPDeviceState::Removing:
            break;
        case MCTPDeviceState::Pending:
            next(device, MCTPDeviceState::Removing);
            break;
    }
}

void MCTPReactor::next(const std::shared_ptr<MCTPDevice>& dev,
                       const MCTPDeviceState next)
{
    debug(
        "Device {DEVICE_ID} ([ {DEVICE_DESCRIPTION} ]) transitioning from {CURRENT_STATE} to {NEXT_STATE}",
        "DEVICE_ID", lg2::hex, dev->id(), "DEVICE_DESCRIPTION", dev->describe(),
        "CURRENT_STATE", states[dev->id()], "NEXT_STATE", next);
    states[dev->id()] = next;
}

void MCTPReactor::terminate(const std::shared_ptr<MCTPDevice>& dev)
{
    debug(
        "Device {DEVICE_ID} ([ {DEVICE_DESCRIPTION} ]) terminated from {CURRENT_STATE}",
        "DEVICE_ID", lg2::hex, dev->id(), "DEVICE_DESCRIPTION", dev->describe(),
        "CURRENT_STATE", states[dev->id()]);
    devices.remove(dev);
    states.erase(dev->id());
}
