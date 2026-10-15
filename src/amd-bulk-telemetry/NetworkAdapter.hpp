/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/async/context.hpp>
#include <sdbusplus/async/server.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Connector/Port/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Decorator/Asset/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/Cable/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/NetworkAdapter/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/PCIeDevice/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/PCIeFunction/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/PCIeSlot/common.hpp>
#include <xyz/openbmc_project/Inventory/Item/Transceiver/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/aserver.hpp>
#include <xyz/openbmc_project/Metric/Value/aserver.hpp>
#include <xyz/openbmc_project/Software/Version/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace ocp::bulk
{

inline constexpr size_t osfpLaneCount = 8;

using TransceiverInfo =
    sdbusplus::common::xyz::openbmc_project::inventory::item::Transceiver;
using PortInfo =
    sdbusplus::common::xyz::openbmc_project::inventory::connector::Port;
using MetricInfo = sdbusplus::common::xyz::openbmc_project::metric::Value;
using PcieSlotInfo =
    sdbusplus::common::xyz::openbmc_project::inventory::item::PCIeSlot;

struct LaneOptics
{
    double txOutputPowerWatts = 0.0;
    double rxInputPowerWatts = 0.0;
    double txBiasCurrentAmps = 0.0;
};

struct Transceiver
{
    bool present = false;
    bool functional = true;
    std::string manufacturer;
    std::string partNumber;
    std::string serialNumber;
    std::string firmwareVersion;
    TransceiverInfo::TransceiverType type =
        TransceiverInfo::TransceiverType::Unknown;
    TransceiverInfo::MediumType mediumType =
        TransceiverInfo::MediumType::Unknown;
    TransceiverInfo::ManagementInterface managementInterface =
        TransceiverInfo::ManagementInterface::Unknown;
    PortInfo::LinkStatus linkStatus = PortInfo::LinkStatus::Unknown;
    // Zero for a module that carries no cable, such as an optical one.
    uint32_t cableLengthMeters = 0;
    std::optional<std::array<LaneOptics, osfpLaneCount>> lanes;
    std::optional<double> supplyVoltageVolts;
};

struct PortCounter
{
    std::string_view name;
    MetricInfo::Unit unit = MetricInfo::Unit::Count;
    uint64_t value = 0;
};

// The host side link of one ASIC port.
struct PcieLink
{
    PcieSlotInfo::Generations generationSupported =
        PcieSlotInfo::Generations::Unknown;
    PcieSlotInfo::Generations generationInUse =
        PcieSlotInfo::Generations::Unknown;
    size_t maxLanes = 0;
    size_t lanesInUse = 0;
    uint16_t subsystemVendorId = 0;
    uint16_t subsystemId = 0;
    bool available = false;
};

class AdapterObject;
using AdapterServer = sdbusplus::async::server_t<
    AdapterObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::NetworkAdapter,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class AdapterObject : public AdapterServer
{
  public:
    AdapterObject(sdbusplus::async::context& ctx,
                  const sdbusplus::object_path& path,
                  const std::string& boardPath);
};

class PortObject;
using PortServer = sdbusplus::async::server_t<
    PortObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::connector::Port,
    sdbusplus::aserver::xyz::openbmc_project::inventory::Item,
    sdbusplus::aserver::xyz::openbmc_project::inventory::decorator::Asset,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::Transceiver,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class PortObject : public PortServer
{
  public:
    PortObject(sdbusplus::async::context& ctx,
               const sdbusplus::object_path& path,
               const std::string& adapterPath);
};

class MetricObject;
using MetricServer = sdbusplus::async::server_t<
    MetricObject, sdbusplus::aserver::xyz::openbmc_project::metric::Value,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class MetricObject : public MetricServer
{
  public:
    MetricObject(sdbusplus::async::context& ctx,
                 const sdbusplus::object_path& path,
                 const std::string& subjectPath, MetricInfo::Unit unit,
                 double value);
};

class CableObject;
using CableServer = sdbusplus::async::server_t<
    CableObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::Cable>;

class CableObject : public CableServer
{
  public:
    CableObject(sdbusplus::async::context& ctx,
                const sdbusplus::object_path& path, double length,
                const std::string& description);
};

class VersionObject;
using VersionServer = sdbusplus::async::server_t<
    VersionObject, sdbusplus::aserver::xyz::openbmc_project::software::Version>;

class VersionObject : public VersionServer
{
  public:
    VersionObject(sdbusplus::async::context& ctx,
                  const sdbusplus::object_path& path,
                  const std::string& version);
};

class PcieDeviceObject;
using PcieDeviceServer = sdbusplus::async::server_t<
    PcieDeviceObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::PCIeDevice,
    sdbusplus::aserver::xyz::openbmc_project::inventory::Item,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class PcieDeviceObject : public PcieDeviceServer
{
  public:
    PcieDeviceObject(sdbusplus::async::context& ctx,
                     const sdbusplus::object_path& path,
                     const std::string& adapterPath, const PcieLink& link);
};

class PcieFunctionObject;
using PcieFunctionServer = sdbusplus::async::server_t<
    PcieFunctionObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::PCIeFunction,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class PcieFunctionObject : public PcieFunctionServer
{
  public:
    PcieFunctionObject(sdbusplus::async::context& ctx,
                       const sdbusplus::object_path& path,
                       const std::string& devicePath, const PcieLink& link);
};

class PciePortObject;
using PciePortServer = sdbusplus::async::server_t<
    PciePortObject,
    sdbusplus::aserver::xyz::openbmc_project::inventory::connector::Port,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class PciePortObject : public PciePortServer
{
  public:
    PciePortObject(sdbusplus::async::context& ctx,
                   const sdbusplus::object_path& path,
                   const std::string& adapterPath, const PcieLink& link);
};

// One adapter per board, shared by every MCTP device the NIC answers on.
class NetworkAdapter
{
  public:
    NetworkAdapter(sdbusplus::async::context& ctx,
                   const std::string& boardPath);

    NetworkAdapter(const NetworkAdapter&) = delete;
    NetworkAdapter(NetworkAdapter&&) = delete;
    NetworkAdapter& operator=(const NetworkAdapter&) = delete;
    NetworkAdapter& operator=(NetworkAdapter&&) = delete;

    ~NetworkAdapter() = default;

    void updateTransceiver(size_t instance, const Transceiver& transceiver);
    void updatePortCounters(size_t instance,
                            std::span<const PortCounter> counters);
    void updatePcieLink(size_t instance, const PcieLink& link);
    void updatePcieCounters(size_t instance,
                            std::span<const PortCounter> counters);

  private:
    struct Port
    {
        std::unique_ptr<PortObject> connector;
        std::unique_ptr<VersionObject> software;
        std::unique_ptr<CableObject> cable;
        std::map<std::string, std::unique_ptr<MetricObject>> metrics;
    };

    struct PcieDevice
    {
        std::unique_ptr<PcieDeviceObject> device;
        std::unique_ptr<PcieFunctionObject> function;
        std::unique_ptr<PciePortObject> port;
        std::map<std::string, std::unique_ptr<MetricObject>> metrics;
    };

    Port& portFor(size_t instance);
    PcieDevice& pcieDeviceFor(size_t instance);
    void updateFirmwareVersion(Port& port, size_t instance,
                               const std::string& version);
    void updateCable(Port& port, size_t instance,
                     const Transceiver& transceiver);
    void updatePortMetric(Port& port, size_t instance, std::string_view group,
                          std::string_view name, MetricInfo::Unit unit,
                          double value);
    void updatePcieMetric(PcieDevice& device, size_t instance,
                          std::string_view name, MetricInfo::Unit unit,
                          double value);
    static void clearPortMetrics(Port& port, std::string_view group);
    void updateOptics(Port& port, size_t instance,
                      const Transceiver& transceiver);

    sdbusplus::async::context& ctx;
    std::string boardName;
    sdbusplus::object_path adapterPath;

    AdapterObject adapter;
    std::map<size_t, Port> ports;
    std::map<size_t, PcieDevice> pcieDevices;
};

} // namespace ocp::bulk
