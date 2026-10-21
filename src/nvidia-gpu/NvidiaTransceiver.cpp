/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "NvidiaTransceiver.hpp"

#include "Utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <string>
#include <vector>

NvidiaTransceiver::NvidiaTransceiver(
    sdbusplus::asio::object_server& objectServer,
    const sdbusplus::object_path& transceiverPath,
    const sdbusplus::object_path& portPath, uint8_t eid, uint16_t portNumber) :
    objectServer(objectServer)
{
    transceiverInterface = objectServer.add_interface(
        transceiverPath, "xyz.openbmc_project.Inventory.Item.Transceiver");

    transceiverInterface->register_property(
        "Type", std::string("xyz.openbmc_project.Inventory.Item.Transceiver."
                            "TransceiverType.Unknown"));
    transceiverInterface->register_property(
        "MediumType",
        std::string(
            "xyz.openbmc_project.Inventory.Item.Transceiver.MediumType.Unknown"));
    transceiverInterface->register_property(
        "ManagementInterfaceType",
        std::string("xyz.openbmc_project.Inventory.Item.Transceiver."
                    "ManagementInterfaceType.Unknown"));

    if (!transceiverInterface->initialize())
    {
        lg2::error(
            "Error initializing Transceiver interface, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }

    itemInterface = objectServer.add_interface(
        transceiverPath, "xyz.openbmc_project.Inventory.Item");

    if (!itemInterface->initialize())
    {
        lg2::error(
            "Error initializing Item interface for Transceiver, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }

    std::vector<Association> associations;
    associations.emplace_back("contained_by", "containing", portPath);

    associationInterface =
        objectServer.add_interface(transceiverPath, association::interface);
    associationInterface->register_property("Associations", associations);

    if (!associationInterface->initialize())
    {
        lg2::error(
            "Error initializing Association interface for Transceiver, eid={EID}, portNumber={PN}",
            "EID", eid, "PN", portNumber);
    }
}

NvidiaTransceiver::~NvidiaTransceiver()
{
    objectServer.remove_interface(transceiverInterface);
    objectServer.remove_interface(itemInterface);
    objectServer.remove_interface(associationInterface);
}
