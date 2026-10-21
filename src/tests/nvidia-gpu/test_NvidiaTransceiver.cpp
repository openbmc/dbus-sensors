/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "DbusMockTestBase.hpp"
#include "NvidiaTransceiver.hpp"
#include "TestUtils.hpp"

#include <sdbusplus/exception.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

namespace
{

using Associations =
    std::vector<std::tuple<std::string, std::string, std::string>>;

constexpr uint16_t defaultPortNumber = 1;

constexpr const char* transceiverInterface =
    "xyz.openbmc_project.Inventory.Item.Transceiver";

std::string portPathFor(const std::string& deviceName)
{
    return "/xyz/openbmc_project/inventory/" + deviceName + "/Port_1";
}

std::string transceiverPathFor(const std::string& deviceName)
{
    return portPathFor(deviceName) + "/Transceiver";
}

class NvidiaTransceiverTest : public DbusMockTestBase
{
  protected:
    static std::shared_ptr<NvidiaTransceiver> createTransceiver(
        const std::string& deviceName)
    {
        return std::make_shared<NvidiaTransceiver>(
            objects(), sdbusplus::object_path(transceiverPathFor(deviceName)),
            sdbusplus::object_path(portPathFor(deviceName)),
            test_utils::defaultEid, defaultPortNumber);
    }
};

TEST_F(NvidiaTransceiverTest, ConstructorPublishesTransceiver)
{
    const std::string deviceName = "xcvr_inventory";
    const std::shared_ptr<NvidiaTransceiver> transceiver =
        createTransceiver(deviceName);

    EXPECT_EQ(getProperty<std::string>(transceiverPathFor(deviceName),
                                       transceiverInterface, "Type"),
              "xyz.openbmc_project.Inventory.Item.Transceiver."
              "TransceiverType.Unknown");
    EXPECT_EQ(getProperty<std::string>(transceiverPathFor(deviceName),
                                       transceiverInterface, "MediumType"),
              "xyz.openbmc_project.Inventory.Item.Transceiver.MediumType."
              "Unknown");
    EXPECT_EQ(getProperty<std::string>(transceiverPathFor(deviceName),
                                       transceiverInterface,
                                       "ManagementInterfaceType"),
              "xyz.openbmc_project.Inventory.Item.Transceiver."
              "ManagementInterfaceType.Unknown");

    const Associations associations = getProperty<Associations>(
        transceiverPathFor(deviceName),
        "xyz.openbmc_project.Association.Definitions", "Associations");
    ASSERT_EQ(associations.size(), 1U);
    EXPECT_EQ(associations[0], std::make_tuple(std::string("contained_by"),
                                               std::string("containing"),
                                               portPathFor(deviceName)));
}

TEST_F(NvidiaTransceiverTest, DestructorRemovesInterfaces)
{
    const std::string deviceName = "xcvr_inventory_dtor";
    std::shared_ptr<NvidiaTransceiver> transceiver =
        createTransceiver(deviceName);
    ASSERT_NE(transceiver, nullptr);

    transceiver.reset();

    EXPECT_THROW(getProperty<std::string>(transceiverPathFor(deviceName),
                                          transceiverInterface, "Type"),
                 sdbusplus::exception_t);
    EXPECT_THROW(
        getProperty<Associations>(transceiverPathFor(deviceName),
                                  "xyz.openbmc_project.Association.Definitions",
                                  "Associations"),
        sdbusplus::exception_t);
}

} // namespace
