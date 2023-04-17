#include "NVMeMiFake.hpp"
#include "NVMeSubsys.hpp"

#include <sdbusplus/asio/connection.hpp>

#include <gtest/gtest.h>

class NVMeTest : public ::testing::Test
{
  protected:
    NVMeTest() :
        system_bus(std::make_shared<sdbusplus::asio::connection>(io)),
        object_server(system_bus),
        subsys(std::make_shared<NVMeSubsystem>(
            io, object_server, system_bus, subsys_path, "NVMe_1",
            std::move(NVMeIntf::create<NVMeMiFake>(io))))
    {}
    void SetUp() override
    {
        system_bus->request_name("xyz.openbmc_project.NVMeTest");
        subsys->start(SensorData{});
    }

    static constexpr char subsys_path[] =
        "/xyz/openbmc_project/inventory/Test_Chassis/Test_NVMe";

    boost::asio::io_service io;
    std::shared_ptr<sdbusplus::asio::connection> system_bus;
    sdbusplus::asio::object_server object_server;

    std::shared_ptr<NVMeSubsystem> subsys;
};

TEST_F(NVMeTest, TestSubsystemStartStop)
{
    boost::asio::steady_timer timer(io);

    // wait for subsystem initialization
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait([&](boost::system::error_code) {
        system_bus->async_method_call(
            [&, this](boost::system::error_code, const GetSubTreeType& result) {
            // Only PF and the enabled VF should be listed
            EXPECT_EQ(result.size(), 2);
            subsys->stop();

            // wait for storage controller destruction.
            timer.expires_after(std::chrono::seconds(1));
            timer.async_wait([&](boost::system::error_code) {
                system_bus->async_method_call(
                    [&](boost::system::error_code,
                        const GetSubTreeType& result) {
                    // not storage controller should be listed.
                    EXPECT_EQ(result.size(), 0);
                    io.stop();
                    },
                    "xyz.openbmc_project.ObjectMapper",
                    "/xyz/openbmc_project/object_mapper",
                    "xyz.openbmc_project.ObjectMapper", "GetSubTree",
                    subsys_path, 0,
                    std::vector<std::string>{"xyz.openbmc_project.Inventory."
                                             "Item.StorageController"});
            });
            },
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree", subsys_path, 0,
            std::vector<std::string>{
                "xyz.openbmc_project.Inventory.Item.StorageController"});
    });
    io.run();
}
