#pragma once

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct AssociationServer
{
    virtual ~AssociationServer() = default;

    virtual void associate(const std::string& path,
                           const std::vector<Association>& associations) = 0;
    virtual void disassociate(const std::string& path) = 0;
};

enum class MCTPDeviceState
{
    // The presence of the device is acknowledged, but we're yet to issue the
    // initial setup request. Ensures endpoint state is tracked for at least the
    // lifetime of the inventory
    Unmanaged,

    // The endpoint setup request for the device has been issued, and we're
    // asynchronously waiting on the response.
    Assigning,

    // An endpoint setup request has been previously issued for the device
    // but it was not successful. Another setup request will be issued in the
    // future.
    Unassigned,

    // An endpoint setup request was successfully issued for the device. The
    // endpoint is currently known to be reachable and both its inventory and
    // endpoint object are published.
    Assigned,

    // The device was removed from inventory while we were waiting for a
    // setup call to complete. The anticipated response will inform whether we
    // consider the device's lifecycle to be complete, or that it be considered
    // recovered.
    Quarantine,

    // A device's endpoint object was removed while its inventory remained
    // present. A future setup request will be issued to recover the endpoint
    // configuration.
    Lost,

    // An endpoint setup request for a lost device has been issued, and we're
    // asynchronously waiting on the response.
    Recovering,

    // The endpoint setup request was successfully issued for a previously
    // lost device. As the device is known to currently be responsive, insulate
    // the configuration from observation of a subsequent inventory removal.
    // Observing an inventory addition subsequent to observation of inventory
    // removal transitions the device state back to Assigned.
    Recovered,

    // Removal of the device from inventory has resulted in a remove call being
    // issued for the device's endpoint, and we're asynchronously waiting on
    // the response.
    Removing,

    // Addition of the device's inventory has been observed concurrent to a
    // remove call whose response is yet to be observed.
    Pending,
};

class MCTPReactor : public std::enable_shared_from_this<MCTPReactor>
{
    using MCTPDeviceFactory = std::function<std::shared_ptr<MCTPDevice>(
        const std::string& interface, const std::vector<std::uint8_t>& physaddr,
        std::optional<std::uint8_t> eid)>;

  public:
    MCTPReactor() = delete;
    MCTPReactor(const MCTPReactor&) = delete;
    MCTPReactor(MCTPReactor&&) = delete;
    explicit MCTPReactor(AssociationServer& server) : server(server) {}
    ~MCTPReactor() = default;
    MCTPReactor& operator=(const MCTPReactor&) = delete;
    MCTPReactor& operator=(MCTPReactor&&) = delete;

    void tick();

    void manageMCTPDevice(const std::string& path,
                          const std::shared_ptr<MCTPDevice>& device);
    void unmanageMCTPDevice(const std::string& path);

  private:
    static std::optional<std::string> findSMBusInterface(int bus);

    AssociationServer& server;
    MCTPDeviceRepository devices;
    std::map<std::size_t, MCTPDeviceState> states;

    void deferSetup(const std::shared_ptr<MCTPDevice>& dev);
    void setupEndpoint(const std::shared_ptr<MCTPDevice>& dev);
    void trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
    void untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
    void next(const std::shared_ptr<MCTPDevice>& dev, MCTPDeviceState next);
    void terminate(const std::shared_ptr<MCTPDevice>& dev);
};
