/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NetworkAdapter.hpp"

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Connector/Port/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Decorator/Asset/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/PCIeDevice/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/PCIeFunction/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/Transceiver/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/aserver.hpp>
#include <xyz/openbmc_project/Metric/Value/aserver.hpp>
#include <xyz/openbmc_project/Software/Version/aserver.hpp>
#include <xyz/openbmc_project/Software/Version/common.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <cstddef>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace ocp::bulk
{

namespace
{

constexpr auto adapterId = "NIC";
constexpr auto cablePathPrefix = "/xyz/openbmc_project/inventory/system/cable/";
constexpr auto softwarePathPrefix = "/xyz/openbmc_project/software/";
constexpr auto metricPathPrefix = "/xyz/openbmc_project/metric/";
constexpr auto transceiverGroup = "transceiver";
constexpr auto counterGroup = "nic";
constexpr auto pcieGroup = "pcie";

using VersionInfo = sdbusplus::common::xyz::openbmc_project::software::Version;
using Associations =
    std::vector<std::tuple<std::string, std::string, std::string>>;

std::string portId(size_t instance)
{
    return std::format("OSFP_{}", instance);
}

std::string portName(const std::string& boardName, size_t instance)
{
    return std::format("{}_{}", boardName, portId(instance));
}

std::string pciePortId(size_t instance)
{
    return std::format("PCIe_{}", instance);
}

std::string pcieDeviceId(const std::string& boardName, size_t instance)
{
    return std::format("{}_{}", boardName, pciePortId(instance));
}

// Only the physical function of an endpoint is reachable over sideband.
constexpr auto pcieFunctionId = "0";

PortInfo::LinkStatus linkStatusOf(const PcieLink& link)
{
    return link.available ? PortInfo::LinkStatus::LinkUp
                          : PortInfo::LinkStatus::LinkDown;
}

} // namespace

AdapterObject::AdapterObject(sdbusplus::async::context& ctx,
                             const sdbusplus::object_path& path,
                             const std::string& boardPath) :
    AdapterServer(ctx, path, std::nullopt,
                  AdapterServer::Definitions::properties_t{
                      .associations = Associations{{"contained_by",
                                                    "containing", boardPath}}},
                  signal_action::emit_object_added)
{}

PortObject::PortObject(sdbusplus::async::context& ctx,
                       const sdbusplus::object_path& path,
                       const std::string& adapterPath) :
    PortServer(
        ctx, path,
        PortServer::Port::properties_t{
            .port_type = PortInfo::PortType::Bidirectional,
            .width = osfpLaneCount,
            .link_status = PortInfo::LinkStatus::Unknown},
        PortServer::Item::properties_t{.present = false},
        PortServer::Asset::properties_t{},
        PortServer::OperationalStatus::properties_t{.functional = true},
        PortServer::Transceiver::properties_t{},
        PortServer::Definitions::properties_t{
            .associations =
                Associations{{"connected_to", "connecting", adapterPath}}},
        signal_action::emit_object_added)
{}

MetricObject::MetricObject(sdbusplus::async::context& ctx,
                           const sdbusplus::object_path& path,
                           const std::string& subjectPath,
                           MetricInfo::Unit unit, double value) :
    MetricServer(
        ctx, path,
        MetricServer::Value::properties_t{.value = value, .unit = unit},
        MetricServer::Definitions::properties_t{
            .associations =
                Associations{{"measuring", "measured_by", subjectPath}}},
        signal_action::emit_object_added)
{}

CableObject::CableObject(sdbusplus::async::context& ctx,
                         const sdbusplus::object_path& path, double length,
                         const std::string& description) :
    CableServer(ctx, path,
                CableServer::Cable::properties_t{
                    .length = length, .cable_type_description = description},
                signal_action::emit_object_added)
{}

VersionObject::VersionObject(sdbusplus::async::context& ctx,
                             const sdbusplus::object_path& path,
                             const std::string& version) :
    VersionServer(
        ctx, path,
        VersionServer::Version::properties_t{
            .version = version, .purpose = VersionInfo::VersionPurpose::Other},
        signal_action::emit_object_added)
{}

PcieDeviceObject::PcieDeviceObject(
    sdbusplus::async::context& ctx, const sdbusplus::object_path& path,
    const std::string& adapterPath, const PcieLink& link) :
    PcieDeviceServer(
        ctx, path,
        PcieDeviceServer::PCIeDevice::properties_t{
            .generation_in_use = link.generationInUse,
            .generation_supported = link.generationSupported,
            .max_lanes = link.maxLanes,
            .lanes_in_use = link.lanesInUse},
        PcieDeviceServer::Item::properties_t{.present = true},
        PcieDeviceServer::Availability::properties_t{
            .available = link.available},
        PcieDeviceServer::Definitions::properties_t{
            .associations =
                Associations{{"contained_by", "containing", adapterPath}}},
        signal_action::emit_object_added)
{}

PcieFunctionObject::PcieFunctionObject(
    sdbusplus::async::context& ctx, const sdbusplus::object_path& path,
    const std::string& devicePath, const PcieLink& link) :
    PcieFunctionServer(
        ctx, path,
        PcieFunctionServer::PCIeFunction::properties_t{
            .subsystem_id = link.subsystemId,
            .subsystem_vendor_id = link.subsystemVendorId},
        PcieFunctionServer::Definitions::properties_t{
            .associations =
                Associations{{"exposed_by", "exposing", devicePath}}},
        signal_action::emit_object_added)
{}

// A PCIe endpoint is no switch, so the adapter is the only Port parent Redfish
// offers for its error counters.
PciePortObject::PciePortObject(
    sdbusplus::async::context& ctx, const sdbusplus::object_path& path,
    const std::string& adapterPath, const PcieLink& link) :
    PciePortServer(
        ctx, path,
        PciePortServer::Port::properties_t{
            .port_protocol = PortInfo::PortProtocol::PCIe,
            .port_type = PortInfo::PortType::Upstream,
            .width = link.lanesInUse,
            .link_status = linkStatusOf(link)},
        PciePortServer::Definitions::properties_t{
            .associations =
                Associations{{"connected_to", "connecting", adapterPath}}},
        signal_action::emit_object_added)
{}

NetworkAdapter::NetworkAdapter(sdbusplus::async::context& ctx,
                               const std::string& boardPath) :
    ctx(ctx), boardName(sdbusplus::object_path(boardPath).filename()),
    adapterPath(sdbusplus::object_path(boardPath) / adapterId),
    adapter(ctx, adapterPath, boardPath)
{}

NetworkAdapter::Port& NetworkAdapter::portFor(size_t instance)
{
    auto [entry, inserted] = ports.try_emplace(instance);
    if (inserted)
    {
        entry->second.connector = std::make_unique<PortObject>(
            ctx, adapterPath / portId(instance), adapterPath.str);
    }

    return entry->second;
}

NetworkAdapter::PcieDevice& NetworkAdapter::pcieDeviceFor(size_t instance)
{
    auto [entry, inserted] = pcieDevices.try_emplace(instance);
    if (inserted)
    {
        const sdbusplus::object_path devicePath =
            adapterPath / pcieDeviceId(boardName, instance);

        PcieDevice& device = entry->second;
        device.device = std::make_unique<PcieDeviceObject>(
            ctx, devicePath, adapterPath.str, PcieLink{});
        device.function = std::make_unique<PcieFunctionObject>(
            ctx, devicePath / pcieFunctionId, devicePath.str, PcieLink{});
        device.port = std::make_unique<PciePortObject>(
            ctx, adapterPath / pciePortId(instance), adapterPath.str,
            PcieLink{});
    }

    return entry->second;
}

// FirmwareInventory lists anything with Software.Version, so an empty port
// drops its entry instead of publishing a blank one.
void NetworkAdapter::updateFirmwareVersion(Port& port, size_t instance,
                                           const std::string& version)
{
    if (version.empty())
    {
        port.software.reset();
        return;
    }

    if (port.software)
    {
        port.software->version(version);
        return;
    }

    port.software = std::make_unique<VersionObject>(
        ctx,
        sdbusplus::object_path(
            softwarePathPrefix + portName(boardName, instance)),
        version);
}

// Only a module that reports a length has a cable.
void NetworkAdapter::updateCable(Port& port, size_t instance,
                                 const Transceiver& transceiver)
{
    if (!transceiver.present || transceiver.cableLengthMeters == 0)
    {
        port.cable.reset();
        return;
    }

    const double length = transceiver.cableLengthMeters;
    const std::string mediumType =
        TransceiverInfo::convertMediumTypeToString(transceiver.mediumType);
    const std::string description =
        mediumType.substr(mediumType.rfind('.') + 1);

    if (port.cable)
    {
        port.cable->length(length);
        port.cable->cable_type_description(description);
        return;
    }

    port.cable = std::make_unique<CableObject>(
        ctx,
        sdbusplus::object_path(cablePathPrefix + portName(boardName, instance)),
        length, description);
}

void NetworkAdapter::updatePortMetric(
    Port& port, size_t instance, std::string_view group, std::string_view name,
    MetricInfo::Unit unit, double value)
{
    std::string key = std::format("{}/{}", group, name);

    auto entry = port.metrics.find(key);
    if (entry != port.metrics.end())
    {
        entry->second->value(value);
        return;
    }

    const sdbusplus::object_path metricPath =
        sdbusplus::object_path(
            metricPathPrefix +
            std::format("port_{}", portName(boardName, instance))) /
        std::string(group) / std::string(name);

    port.metrics.emplace(
        std::move(key), std::make_unique<MetricObject>(
                            ctx, metricPath,
                            (adapterPath / portId(instance)).str, unit, value));
}

void NetworkAdapter::updatePcieMetric(PcieDevice& device, size_t instance,
                                      std::string_view name,
                                      MetricInfo::Unit unit, double value)
{
    std::string key{name};

    auto entry = device.metrics.find(key);
    if (entry != device.metrics.end())
    {
        entry->second->value(value);
        return;
    }

    const std::string deviceId = pcieDeviceId(boardName, instance);
    const sdbusplus::object_path metricPath =
        sdbusplus::object_path(
            metricPathPrefix + std::format("port_{}", deviceId)) /
        std::string(pcieGroup) / std::string(name);

    device.metrics.emplace(
        std::move(key),
        std::make_unique<MetricObject>(ctx, metricPath,
                                       (adapterPath / pciePortId(instance)).str,
                                       unit, value));
}

void NetworkAdapter::clearPortMetrics(Port& port, std::string_view group)
{
    const std::string prefix = std::format("{}/", group);
    std::erase_if(port.metrics, [&prefix](const auto& metric) {
        return metric.first.starts_with(prefix);
    });
}

// Drop an empty port's metrics instead of leaving the departed module's last
// readings frozen there. The MAC counters belong to the port and stay.
void NetworkAdapter::updateOptics(Port& port, size_t instance,
                                  const Transceiver& transceiver)
{
    if (!transceiver.present)
    {
        clearPortMetrics(port, transceiverGroup);
        return;
    }

    if (transceiver.lanes)
    {
        for (size_t lane = 0; lane < transceiver.lanes->size(); lane++)
        {
            const LaneOptics& optics = (*transceiver.lanes)[lane];
            updatePortMetric(port, instance, transceiverGroup,
                             std::format("tx_output_power_lane_{}", lane),
                             MetricInfo::Unit::Watts,
                             optics.txOutputPowerWatts);
            updatePortMetric(port, instance, transceiverGroup,
                             std::format("rx_input_power_lane_{}", lane),
                             MetricInfo::Unit::Watts, optics.rxInputPowerWatts);
            updatePortMetric(port, instance, transceiverGroup,
                             std::format("tx_bias_current_lane_{}", lane),
                             MetricInfo::Unit::Amperes,
                             optics.txBiasCurrentAmps);
        }
    }

    if (transceiver.supplyVoltageVolts)
    {
        updatePortMetric(port, instance, transceiverGroup, "supply_voltage",
                         MetricInfo::Unit::Volts,
                         *transceiver.supplyVoltageVolts);
    }
}

// A port reports counters with or without a module.
void NetworkAdapter::updatePortCounters(size_t instance,
                                        std::span<const PortCounter> counters)
{
    Port& port = portFor(instance);

    for (const PortCounter& counter : counters)
    {
        updatePortMetric(port, instance, counterGroup, counter.name,
                         counter.unit, static_cast<double>(counter.value));
    }
}

void NetworkAdapter::updatePcieLink(size_t instance, const PcieLink& link)
{
    PcieDevice& entry = pcieDeviceFor(instance);

    entry.device->generation_in_use(link.generationInUse);
    entry.device->generation_supported(link.generationSupported);
    entry.device->max_lanes(link.maxLanes);
    entry.device->lanes_in_use(link.lanesInUse);
    entry.device->available(link.available);
    entry.function->subsystem_id(link.subsystemId);
    entry.function->subsystem_vendor_id(link.subsystemVendorId);
    entry.port->width(link.lanesInUse);
    entry.port->link_status(linkStatusOf(link));
}

void NetworkAdapter::updatePcieCounters(size_t instance,
                                        std::span<const PortCounter> counters)
{
    PcieDevice& device = pcieDeviceFor(instance);

    for (const PortCounter& counter : counters)
    {
        updatePcieMetric(device, instance, counter.name, counter.unit,
                         static_cast<double>(counter.value));
    }
}

void NetworkAdapter::updateTransceiver(size_t instance,
                                       const Transceiver& transceiver)
{
    Port& port = portFor(instance);
    PortObject& connector = *port.connector;

    connector.present(transceiver.present);
    connector.functional(transceiver.functional);
    connector.link_status(transceiver.linkStatus);
    connector.manufacturer(transceiver.manufacturer);
    connector.part_number(transceiver.partNumber);
    connector.serial_number(transceiver.serialNumber);
    connector.type(transceiver.type);
    connector.medium_type(transceiver.mediumType);
    connector.management_interface(transceiver.managementInterface);

    updateFirmwareVersion(port, instance, transceiver.firmwareVersion);
    updateCable(port, instance, transceiver);
    updateOptics(port, instance, transceiver);
}

} // namespace ocp::bulk
