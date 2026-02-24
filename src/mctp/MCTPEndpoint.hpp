#pragma once

#include "Utils.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

/**
 * @file
 * @brief Abstract and concrete classes representing MCTP concepts and
 *        behaviours.
 */

/**
 * @brief An exception type that may be thrown by implementations of the MCTP
 * abstract classes.
 *
 * This exception should be the basis for all exceptions thrown out of the MCTP
 * APIs, and should capture any other exceptions that occur.
 */
class MCTPException : public std::exception
{
  public:
    MCTPException() = delete;
    explicit MCTPException(const char* desc) : desc(desc) {}
    const char* what() const noexcept override
    {
        return desc;
    }

  private:
    const char* desc;
};

/**
 * @brief An enum of the MCTP transports described in DSP0239 v1.10.0 Section 7.
 *
 * https://www.dmtf.org/sites/default/files/standards/documents/DSP0239_1.10.0.pdf
 */
enum class MCTPTransport
{
    Reserved = 0x00,
    SMBus = 0x01,
};

/**
 * @brief Captures properties of MCTP interfaces.
 *
 * https://github.com/CodeConstruct/mctp/blob/v2.0/src/mctp.c#L668-L699
 */
struct MCTPInterface
{
    std::string name;
    MCTPTransport transport;

    auto operator<=>(const MCTPInterface& r) const = default;
};

class MCTPDevice;

/**
 * @brief Captures the behaviour of an endpoint at the MCTP layer
 *
 * The lifetime of an instance of MctpEndpoint is proportional to the lifetime
 * of the endpoint configuration. If an endpoint is deconfigured such that its
 * device has no assigned EID, then any related MctpEndpoint instance must be
 * destructed as a consequence.
 */
class MCTPEndpoint
{
  public:
    using Event = std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)>;
    using Result = std::function<void(const std::error_code& ec)>;

    virtual ~MCTPEndpoint() = default;

    /**
     * @return The Linux network ID of the network in which the endpoint
               participates
     */
    virtual int network() const = 0;

    /**
     * @return The MCTP endpoint ID of the endpoint in its network
     */
    virtual uint8_t eid() const = 0;

    /**
     * @brief Subscribe to events produced by an endpoint object across its
     *        lifecycle
     *
     * @param degraded The callback to execute when the MCTP layer indicates the
     *                 endpoint is unresponsive
     *
     * @param available The callback to execute when the MCTP layer indicates
     *                  that communication with the degraded endpoint has been
     *                  recovered
     *
     * @param removed The callback to execute when the MCTP layer indicates the
     *                endpoint has been removed.
     */
    virtual void subscribe(Event&& degraded, Event&& available,
                           Event&& removed) = 0;

    /**
     * @brief Remove the endpoint from its associated network
     */
    virtual void remove() = 0;

    /**
     * @return A formatted string representing the endpoint in terms of its
     *         address properties
     */
    virtual std::string describe() const = 0;

    /**
     * @return A shared pointer to the device instance associated with the
     *         endpoint.
     */
    virtual std::shared_ptr<MCTPDevice> device() const = 0;
};

/**
 * @brief Represents an MCTP-capable device on a bus.
 *
 * It is often known that an MCTP-capable device exists on a bus prior to the
 * MCTP stack configuring the device for communication. MctpDevice exposes the
 * ability to set-up the endpoint device for communication.
 *
 * The lifetime of an MctpDevice instance is proportional to the existence of an
 * MCTP-capable device in the system. If a device represented by an MctpDevice
 * instance is removed from the system then any related MctpDevice instance must
 * be destructed a consequence.
 *
 * Successful set-up of the device as an endpoint yields an MctpEndpoint
 * instance. The lifetime of the MctpEndpoint instance produced must not exceed
 * the lifetime of its parent MctpDevice.
 */
class MCTPDevice
{
  public:
    virtual ~MCTPDevice() = default;

    /**
     * @brief Configure the device for MCTP communication
     *
     * @param added The callback to invoke once the setup process has
     *              completed. The provided error code @p ec must be
     *              checked as the request may not have succeeded. If
     *              the request was successful then @p ep contains a
     *              valid MctpEndpoint instance.
     */
    virtual void setup(
        std::function<void(const std::error_code& ec,
                           const std::shared_ptr<MCTPEndpoint>& ep)>&&
            added) = 0;

    /**
     * @brief Remove the device and any associated endpoint from the MCTP stack.
     */
    virtual void remove() = 0;

    /**
     * @return A formatted string representing the device in terms of its
     *         address properties.
     */
    virtual std::string describe() const = 0;

    /**
     * @return An opaque, internally-stable identifier representing the device
     */
    virtual std::size_t id() const = 0;
};

class MCTPDDevice;

/**
 * @brief An implementation of MctpEndpoint in terms of the D-Bus interfaces
 *        exposed by @c mctpd.
 *
 * The lifetime of an MctpdEndpoint is proportional to the lifetime of the
 * endpoint object exposed by @c mctpd. The lifecycle of @c mctpd endpoint
 * objects is discussed here:
 *
 * https://github.com/CodeConstruct/mctp/pull/23/files#diff-00234f5f2543b8b9b8a419597e55121fe1cc57cf1c7e4ff9472bed83096bd28e
 */
class MCTPDEndpoint :
    public MCTPEndpoint,
    public std::enable_shared_from_this<MCTPDEndpoint>
{
  public:
    static std::string path(const std::shared_ptr<MCTPEndpoint>& ep);

    MCTPDEndpoint() = delete;
    MCTPDEndpoint(
        const std::shared_ptr<MCTPDDevice>& dev,
        const std::shared_ptr<sdbusplus::asio::connection>& connection,
        sdbusplus::message::object_path objpath, int network, uint8_t eid) :
        dev(dev), connection(connection), objpath(std::move(objpath)),
        mctp{network, eid}
    {}
    MCTPDEndpoint& McptdEndpoint(const MCTPDEndpoint& other) = delete;
    MCTPDEndpoint(MCTPDEndpoint&& other) noexcept = default;
    ~MCTPDEndpoint() override = default;

    int network() const override;
    uint8_t eid() const override;
    void subscribe(Event&& degraded, Event&& available,
                   Event&& removed) override;
    void remove() override;

    std::string describe() const override;

    std::shared_ptr<MCTPDevice> device() const override;

    /**
     * @brief Indicate the endpoint has been removed
     *
     * Called from the implementation of MctpdDevice for resource cleanup
     * prior to destruction. Resource cleanup is delegated by invoking the
     * notifyRemoved() callback. As the actions may be abitrary we avoid
     * invoking notifyRemoved() in the destructor.
     */
    void removed();

  private:
    std::shared_ptr<MCTPDDevice> dev;
    std::shared_ptr<sdbusplus::asio::connection> connection;
    sdbusplus::message::object_path objpath;
    struct
    {
        int network;
        uint8_t eid;
    } mctp;
    MCTPEndpoint::Event notifyAvailable;
    MCTPEndpoint::Event notifyDegraded;
    MCTPEndpoint::Event notifyRemoved;
    std::optional<sdbusplus::bus::match_t> connectivityMatch;

    void onMctpEndpointChange(sdbusplus::message_t& msg);
    void updateEndpointConnectivity(const std::string& connectivity);
};

/**
 * @brief An implementation of MctpDevice in terms of D-Bus interfaces exposed
 *        by @c mctpd.
 *
 * The construction or destruction of an MctpdDevice is not required to be
 * correlated with signals from @c mctpd. For instance, EntityManager may expose
 * the existence of an MCTP-capable device through its usual configuration
 * mechanisms.
 */
class MCTPDDevice :
    public MCTPDevice,
    public std::enable_shared_from_this<MCTPDDevice>
{
  public:
    MCTPDDevice() = delete;
    MCTPDDevice(const std::shared_ptr<sdbusplus::asio::connection>& connection,
                const std::string& interface,
                const std::vector<uint8_t>& physaddr);
    MCTPDDevice(const MCTPDDevice& other) = delete;
    MCTPDDevice(MCTPDDevice&& other) = delete;
    ~MCTPDDevice() override = default;

    void setup(std::function<void(const std::error_code& ec,
                                  const std::shared_ptr<MCTPEndpoint>& ep)>&&
                   added) override;
    void remove() override;
    std::string describe() const override;
    std::size_t id() const override;

  private:
    static void onEndpointInterfacesRemoved(
        const std::weak_ptr<MCTPDDevice>& weak, const std::string& objpath,
        sdbusplus::message_t& msg);

    std::shared_ptr<sdbusplus::asio::connection> connection;
    const std::string interface;
    const std::vector<uint8_t> physaddr;
    std::shared_ptr<MCTPDEndpoint> endpoint;
    std::unique_ptr<sdbusplus::bus::match_t> removeMatch;

    /**
     * @brief Actions to perform once endpoint setup has succeeded
     *
     * Now that the endpoint exists two tasks remain:
     *
     * 1. Setup the match capturing removal of the endpoint object by mctpd
     * 2. Invoke the callback to notify the requester that setup has completed,
     *    providing the MctpEndpoint instance associated with the MctpDevice.
     */
    void finaliseEndpoint(
        const std::string& objpath, uint8_t eid, int network,
        std::function<void(const std::error_code& ec,
                           const std::shared_ptr<MCTPEndpoint>& ep)>& added);
    void endpointRemoved();
};

class I2CMCTPDDevice : public MCTPDDevice
{
  public:
    static std::optional<SensorBaseConfigMap> match(const SensorData& config);
    static bool match(const std::set<std::string>& interfaces);
    static std::shared_ptr<I2CMCTPDDevice> from(
        const std::shared_ptr<sdbusplus::asio::connection>& connection,
        const SensorBaseConfigMap& iface);

    I2CMCTPDDevice() = delete;
    I2CMCTPDDevice(
        const std::shared_ptr<sdbusplus::asio::connection>& connection, int bus,
        uint8_t physaddr) :
        MCTPDDevice(connection, interfaceFromBus(bus), {physaddr})
    {}
    ~I2CMCTPDDevice() override = default;

  private:
    static constexpr const char* configType = "MCTPI2CTarget";

    static std::string interfaceFromBus(int bus);
};

class I3CMCTPDDevice : public MCTPDDevice
{
  public:
    static std::optional<SensorBaseConfigMap> match(const SensorData& config);
    static bool match(const std::set<std::string>& interfaces);
    static std::shared_ptr<I3CMCTPDDevice> from(
        const std::shared_ptr<sdbusplus::asio::connection>& connection,
        const SensorBaseConfigMap& iface);

    I3CMCTPDDevice() = delete;
    I3CMCTPDDevice(
        const std::shared_ptr<sdbusplus::asio::connection>& connection, int bus,
        const std::vector<uint8_t>& physaddr) :
        MCTPDDevice(connection, interfaceFromBus(bus), physaddr)
    {}
    ~I3CMCTPDDevice() override = default;

  private:
    static constexpr const char* configType = "MCTPI3CTarget";

    static std::string interfaceFromBus(int bus);
};
