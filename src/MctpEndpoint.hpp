#pragma once

#include <boost/asio/steady_timer.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <iostream>

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
class MctpException : public std::exception
{
  public:
    MctpException() = delete;
    explicit MctpException(const char* desc) : desc(desc) {}
    const char* what() const noexcept override
    {
        return desc;
    }

  private:
    const char* desc;
};

enum class MctpTransport
{
  SMBus,
};

struct MctpInterface
{
    std::string name;
    MctpTransport transport;

    auto operator<=>(const MctpInterface& r) const = default;
};

class MctpDevice;

/**
 * @brief Captures the behaviour of an endpoint at the MCTP layer
 *
 * The lifetime of an instance of MctpEndpoint is proportional to the lifetime
 * of the endpoint configuration. If an endpoint is deconfigured such that its
 * device has no assigned EID, then any related MctpEndpoint instance must be
 * destructed as a consequence.
 */
class MctpEndpoint
{
  public:
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
    virtual void subscribe(
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&& degraded,
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&&
            available,
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&&
            removed) = 0;

    /**
     * @brief Configures the Maximum Transmission Unit (MTU) for the route to
     *        the endpoint
     *
     * @param mtu The route MTU to be used for the endpoint. Note that MTU
     *            value refers to the size of the entire MCTP packet. The value
     *            provided is interpreted as accounting for the MCTP header.
     *
     * @param completed The callback to invoke once the MTU configuration has
     *                  completed. The provided error code must be checked as
     *                  the request may not have succeeded.
     */
    virtual void
        setMtu(uint32_t mtu,
               std::function<void(const std::error_code& ec)>&& completed) = 0;

    /**
     * @brief Request that the MCTP layer attempt to recover communication with
     *        an unresponsive endpoint.
     *
     * Recovery progress is indicated by invocation of the @c degraded,
     * @c available and @c removed handlers registered using the @c events()
     * API.
     */
    virtual void recover() = 0;

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
    virtual std::shared_ptr<MctpDevice> device() const = 0;
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
class MctpDevice
{
  public:
    /**
     * @brief Configure the device for MCTP communication
     *
     * @param added The callback to invoke once the setup process has
     *              completed. The provided error code @p ec must be
     *              checked as the request may not have succeeded. If
     *              the request was successful then @p ep contains a
     *              valid MctpEndpoint instance.
     */
    virtual void
        setup(std::function<void(const std::error_code& ec,
                                 const std::shared_ptr<MctpEndpoint>& ep)>&&
                  added) = 0;

    virtual void remove() = 0;

    /**
     * @return A formatted string representing the device in terms of its
     *         address properties.
     */
    virtual std::string describe() const = 0;
};

class MctpdDevice;

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
class MctpdEndpoint :
    public MctpEndpoint,
    public std::enable_shared_from_this<MctpdEndpoint>
{
  public:
    static std::string path(const std::shared_ptr<MctpEndpoint>& ep);

    MctpdEndpoint() = delete;
    MctpdEndpoint(
        const std::shared_ptr<MctpdDevice>& dev,
        const std::shared_ptr<sdbusplus::asio::connection>& connection,
        sdbusplus::message::object_path objpath, int network, uint8_t eid);
    MctpdEndpoint& McptdEndpoint(const MctpdEndpoint& other) = delete;
    MctpdEndpoint(MctpdEndpoint&& other) noexcept = default;
    virtual ~MctpdEndpoint() = default;

    int network() const override;
    uint8_t eid() const override;
    void subscribe(
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&& degraded,
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&&
            available,
        std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>&& removed)
        override;
    void setMtu(
        uint32_t mtu,
        std::function<void(const std::error_code& ec)>&& completed) override;
    void recover() override;
    void remove() override;

    std::string describe() const override;

    std::shared_ptr<MctpDevice> device() const override;

    /**
     * @brief Indicate the endpoint has been removed
     *
     * Called from the implementation of MctpdDevice::removed() for resource
     * cleanup prior to destruction.
     */
    void removed();

  private:
    std::shared_ptr<MctpdDevice> dev;
    std::shared_ptr<sdbusplus::asio::connection> connection;
    sdbusplus::message::object_path objpath;
    struct
    {
        int network;
        uint8_t eid;
    } mctp;
    std::function<void(const std::shared_ptr<MctpEndpoint>& ep)>
        notifyAvailable;
    std::function<void(const std::shared_ptr<MctpEndpoint>& ep)> notifyDegraded;
    std::function<void(const std::shared_ptr<MctpEndpoint>& ep)> notifyRemoved;
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
 * the existance of an MCTP-capable device through its usual configuration
 * mechanisms.
 */
class MctpdDevice :
    public MctpDevice,
    public std::enable_shared_from_this<MctpdDevice>
{
  public:
    MctpdDevice() = delete;
    MctpdDevice(const std::shared_ptr<sdbusplus::asio::connection>& connection,
                const std::string& interface,
                const std::vector<uint8_t>& physaddr,
                std::optional<std::uint8_t> eid = {});
    MctpdDevice(const MctpdDevice& other) = delete;
    MctpdDevice(MctpdDevice&& other) = delete;
    virtual ~MctpdDevice() = default;

    void setup(std::function<void(const std::error_code& ec,
                                  const std::shared_ptr<MctpEndpoint>& ep)>&&
                   added) override;
    void remove() override;
    std::string describe() const override;

  private:
    static void
        onEndpointInterfacesRemoved(const std::weak_ptr<MctpdDevice>& weak,
                                    const std::string& objpath,
                                    sdbusplus::message_t& msg);

    std::shared_ptr<sdbusplus::asio::connection> connection;
    const std::string interface;
    const std::vector<uint8_t> physaddr;
    const std::optional<uint8_t> eid;
    std::shared_ptr<MctpdEndpoint> endpoint;
    std::unique_ptr<sdbusplus::bus::match_t> removeMatch;

    void finaliseEndpoint(
        const std::string& objpath, uint8_t eid, int network,
        std::function<void(const std::error_code& ec,
                           const std::shared_ptr<MctpEndpoint>& ep)>&& added);
    void endpointRemoved();
};

/**
 * @brief A specialisation of MctpdDevice for SMBus devices
 *
 * Abstracts over the implementation details of specifying SMBus device address
 * parameters to @c mctpd.
 */
class SmbusMctpdDevice : public MctpdDevice
{
  public:
    SmbusMctpdDevice() = delete;
    SmbusMctpdDevice(
        const std::shared_ptr<sdbusplus::asio::connection>& connection,
        int smbus, uint8_t smdev, std::optional<std::uint8_t> eid = {});
    SmbusMctpdDevice(const SmbusMctpdDevice& other) = delete;
    SmbusMctpdDevice(SmbusMctpdDevice&& other) = delete;
    ~SmbusMctpdDevice() override = default;

    std::string describe() const override;

  private:
    const int smbus;
    const uint8_t smdev;
};
