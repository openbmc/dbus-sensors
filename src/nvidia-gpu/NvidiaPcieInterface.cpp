#include "NvidiaPcieInterface.hpp"

#include "Utils.hpp"

#include <bits/basic_string.h>

#include <MctpRequester.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <OcpMctpVdm.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using std::string;

using namespace std::literals;

NvidiaPcieInterface::NvidiaPcieInterface(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    mctp::MctpRequester& mctpRequester, const std::string& name,
    const std::string& path, uint8_t eid, gpu::PciePortType portType,
    uint8_t portNumber, sdbusplus::asio::object_server& objectServer) :
    eid(eid), portType(portType), portNumber(portNumber), conn(conn),
    mctpRequester(mctpRequester)
{
    const std::string dbusPath = path + escapeName(name);

    pcieDeviceInterface = objectServer.add_interface(
        dbusPath, "xyz.openbmc_project.Inventory.Item.PCIeDevice");

    pcieDeviceInterface->register_property(
        "GenerationInUse",
        std::string(
            "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown"));

    pcieDeviceInterface->register_property("LanesInUse",
                                           std::numeric_limits<size_t>::max());

    pcieDeviceInterface->register_property(
        "GenerationSupported",
        std::string(
            "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown"));

    pcieDeviceInterface->register_property("MaxLanes", static_cast<size_t>(0));

    if (!pcieDeviceInterface->initialize())
    {
        lg2::error(
            "Error initializing PCIe Device Interface for EID={EID}, PortType={PT}, PortNumber={PN}",
            "EID", eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
    }
}

string NvidiaPcieInterface::mapPcieGeneration(uint32_t value)
{
    switch (value)
    {
        case 1:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen1";
        case 2:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen2";
        case 3:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen3";
        case 4:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen4";
        case 5:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen5";
        case 6:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Gen6";
        default:
            return "xyz.openbmc_project.Inventory.Item.PCIeSlot.Generations.Unknown";
    }
}

void NvidiaPcieInterface::processResponse(int sendRecvMsgResult)
{
    if (sendRecvMsgResult != 0)
    {
        lg2::error(
            "Error updating PCIe Interface: sending message over MCTP failed, "
            "rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", sendRecvMsgResult, "EID", eid, "PT",
            static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    ocp::accelerator_management::CompletionCode cc{};
    uint16_t reasonCode = 0;
    size_t numTelemetryValue = 0;

    auto rc = gpu::decodeQueryScalarGroupTelemetryV2Response(
        response, cc, reasonCode, numTelemetryValue, telemetryValues);

    if (rc != 0 || cc != ocp::accelerator_management::CompletionCode::SUCCESS)
    {
        lg2::error(
            "Error updating PCIe Interface: decode failed, "
            "rc={RC}, cc={CC}, reasonCode={RESC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "CC", static_cast<uint8_t>(cc), "RESC", reasonCode, "EID",
            eid, "PT", static_cast<uint8_t>(portType), "PN", portNumber);
        return;
    }

    if (!telemetryValues.empty())
    {
        pcieDeviceInterface->set_property(
            "GenerationInUse", mapPcieGeneration(telemetryValues[0]));
    }

    if (telemetryValues.size() > 1)
    {
        pcieDeviceInterface->set_property(
            "LanesInUse", static_cast<size_t>(telemetryValues[1]));
    }

    if (telemetryValues.size() > 3)
    {
        pcieDeviceInterface->set_property(
            "GenerationSupported", mapPcieGeneration(telemetryValues[3]));
    }

    if (telemetryValues.size() > 4)
    {
        pcieDeviceInterface->set_property(
            "MaxLanes", static_cast<size_t>(telemetryValues[4]));
    }
}

void NvidiaPcieInterface::update()
{
    auto rc = gpu::encodeQueryScalarGroupTelemetryV2Request(
        0, portNumber, portType, 0, 1, request);

    if (rc != 0)
    {
        lg2::error(
            "Error updating PCIe Interface: encode failed, rc={RC}, EID={EID}, PortType={PT}, PortNumber={PN}",
            "RC", rc, "EID", eid, "PT", static_cast<uint8_t>(portType), "PN",
            portNumber);
        return;
    }

    mctpRequester.sendRecvMsg(
        eid, request, response,
        [this](int sendRecvMsgResult) { processResponse(sendRecvMsgResult); });
}
