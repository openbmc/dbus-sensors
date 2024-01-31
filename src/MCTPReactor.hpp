#pragma once

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "Utils.hpp"

#include <string>
#include <vector>

struct AssociationServer
{
    virtual ~AssociationServer() = default;

    virtual void associate(const std::string& path,
                           const std::vector<Association>& associations) = 0;
    virtual void disassociate(const std::string& path) = 0;
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
    MCTPReactor(MCTPDeviceFactory&& createDevice, AssociationServer& server) :
        createDevice(createDevice), server(server)
    {}
    ~MCTPReactor() = default;
    MCTPReactor& operator=(const MCTPReactor&) = delete;
    MCTPReactor& operator=(MCTPReactor&&) = delete;

    void tick();

    void manageMCTPDevice(const std::string& path,
                          const SensorBaseConfigMap& iface);
    void unmanageMCTPDevice(const std::string& path);
    void trackMCTPInterface(const std::string& path,
                            const SensorBaseConfigMap& iface);
    void untrackMCTPInterface(const std::string& path);

  private:
    MCTPDeviceFactory createDevice;
    AssociationServer& server;

    // Maps the inventory DBus object path exposing an MCTP interface to an MCTP
    // interface name
    std::map<std::string, std::string> interfaceInventory;
    // Maps an MCTP interface name to the interface's properties
    std::map<std::string, MCTPInterface> interfaceConfiguration;

    MCTPDeviceRepository devices;

    // Maps the inventory DBus object path exposing an MCTP device to the
    // device's MCTP properties
    std::map<std::string, SensorBaseConfigMap> unreachable;

    // Tracks MCTP devices that have failed their setup
    std::set<std::shared_ptr<MCTPDevice>> deferred;

    void deferSetup(const std::shared_ptr<MCTPDevice>& dev);
    void setupEndpoint(const std::shared_ptr<MCTPDevice>& dev);
    void trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
    void untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
};
