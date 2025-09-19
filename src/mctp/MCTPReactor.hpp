#pragma once

#include "MCTPDeviceRepository.hpp"
#include "MCTPEndpoint.hpp"
#include "utils/Utils.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
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

    // Tracks MCTP devices that have failed their setup
    std::set<std::shared_ptr<MCTPDevice>> deferred;

    void deferSetup(const std::shared_ptr<MCTPDevice>& dev);
    void setupEndpoint(const std::shared_ptr<MCTPDevice>& dev);
    void trackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
    void untrackEndpoint(const std::shared_ptr<MCTPEndpoint>& ep);
};
