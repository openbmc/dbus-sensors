#include "NvidiaPciePort.hpp"

#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaPcieDevice.hpp>
#include <NvidiaPcieInterface.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using std::string;

using namespace std::literals;

NvidiaPciePortInfo::NvidiaPciePortInfo(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& pcieDeviceName, const std::string& path, uint8_t eid,
    gpu::PciePortType portType, uint8_t upstreamPortNumber, uint8_t portNumber,
    sdbusplus::asio::object_server& objectServer) :
    eid(eid), portType(portType), upstreamPortNumber(upstreamPortNumber),
    portNumber(portNumber), path(path), conn(conn), mctpRequester(mctpRequester)
{
    const std::string dbusPath = pcieDevicePathPrefix + escapeName(name);

    pciePortInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Inventory.Item.Port");

    std::string portTypeStr;

    if (portType == gpu::PciePortType::UPSTREAM)
    {
        portTypeStr = "UpstreamPort";
    }
    else
    {
        portTypeStr = "DownstreamPort";
    }

    pciePortInterface->register_property(
        "PortType",
        "xyz.openbmc_project.Inventory.Item.Port.PortType." + portTypeStr);

    pciePortInterface->register_property(
        "PortProtocol",
        std::string(
            "xyz.openbmc_project.Inventory.Item.Port.PortProtocolType.PCIe"));

    pciePortInterface->register_property("CurrentSpeedGbps",
                                         static_cast<double>(0));

    pciePortInterface->register_property("ActiveWidth",
                                         std::numeric_limits<size_t>::max());

    if (!pciePortInterface->initialize())
    {
        lg2::error(
            "Error initializing PCIe Device Interface for EID={EID}, PortType={PT}, PortNumber={PN}",
            "EID", eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
    }

    std::vector<Association> associations;
    associations.emplace_back("connected_to", "connecting",
                              pcieDevicePathPrefix + pcieDeviceName);

    associationInterface =
        objectServer.add_interface(dbusPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association Interface for PCIe Port Info for EID={EID}, PortType={PT}, PortNumber={PN}",
            "EID", eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
    }
}

double NvidiaPciePortInfo::mapPcieGenToLinkSpeedGbps(uint32_t value)
{
    switch (value)
    {
        case 1:
            return 2.5;
        case 2:
            return 5.0;
        case 3:
            return 8.0;
        case 4:
            return 16.0;
        case 5:
            return 32.0;
        case 6:
            return 64.0;
        default:
            return 0.0;
    }
}

void NvidiaPciePortInfo::processResponse(
    const std::error_code& sendRecvMsgResult, std::span<const uint8_t> response)
{
    if (sendRecvMsgResult)
    {
        lg2::error(
            "Error updating PCIe Port Info: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", sendRecvMsgResult.message(), "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    const int rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Port Info: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    if (!telemetryValues.empty())
    {
        pciePortInterface->set_property(
            "CurrentSpeedGbps", mapPcieGenToLinkSpeedGbps(telemetryValues[0]));
    }

    if (telemetryValues.size() > 1)
    {
        pciePortInterface->set_property(
            "ActiveWidth", NvidiaPcieInterface::decodeLinkWidth(
                               static_cast<size_t>(telemetryValues[1])));
    }
}

void NvidiaPciePortInfo::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portType, upstreamPortNumber, portNumber, 1, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Port Info: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request,
        [weak{weak_from_this()}](const std::error_code& ec,
                                 std::span<const uint8_t> buffer) {
            std::shared_ptr<NvidiaPciePortInfo> self = weak.lock();
            if (!self)
            {
                lg2::error(
                    "Invalid reference to NvidiaPciePortInfo, EID={EID}, PortType={PT}, PortNumber={PN}",
                    "EID", self->eid, "PT",
                    static_cast<uint8_t>(self->portType), "PN",
                    self->portNumber);
                return;
            }
            self->processResponse(ec, buffer);
        });
}
