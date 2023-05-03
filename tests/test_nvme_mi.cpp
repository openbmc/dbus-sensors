#include "NVMeMiFake.hpp"
#include "NVMeSubsys.hpp"

#include <sdbusplus/asio/connection.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class NVMeMiMock :
    public NVMeMiIntf,
    public std::enable_shared_from_this<NVMeMiMock>
{
  public:
    NVMeMiMock(boost::asio::io_service& io) :
        fake(std::move(std::make_shared<NVMeMiFake>(io)))
    {
        ON_CALL(*this, getNID).WillByDefault([]() { return 0; });
        ON_CALL(*this, getEID).WillByDefault([]() { return 0; });
        ON_CALL(*this, miSubsystemHealthStatusPoll)
            .WillByDefault(
                [this](
                    std::function<void(const std::error_code&,
                                       nvme_mi_nvm_ss_health_status*)>&& cb) {
            return fake->miSubsystemHealthStatusPoll(std::move(cb));
            });
        ON_CALL(*this, miScanCtrl)
            .WillByDefault(
                [this](
                    std::function<void(const std::error_code& ec,
                                       const std::vector<nvme_mi_ctrl_t>& list)>
                        cb) { return fake->miScanCtrl(std::move(cb)); });
        ON_CALL(*this, adminIdentify)
            .WillByDefault(
                [this](nvme_mi_ctrl_t ctrl, nvme_identify_cns cns,
                       uint32_t nsid, uint16_t cntid,
                       std::function<void(const std::error_code&,
                                          std::span<uint8_t>)>&& cb) {
            return fake->adminIdentify(ctrl, cns, nsid, cntid, std::move(cb));
            });
        ON_CALL(*this, adminGetLogPage)
            .WillByDefault(
                [this](nvme_mi_ctrl_t ctrl, nvme_cmd_get_log_lid lid,
                       uint32_t nsid, uint8_t lsp, uint16_t lsi,
                       std::function<void(const std::error_code&,
                                          std::span<uint8_t>)>&& cb) {
            return fake->adminGetLogPage(ctrl, lid, nsid, lsp, lsi,
                                         std::move(cb));
            });
        ON_CALL(*this, adminFwCommit)
            .WillByDefault([this](nvme_mi_ctrl_t ctrl, nvme_fw_commit_ca action,
                                  uint8_t slot, bool bpid,
                                  std::function<void(const std::error_code&,
                                                     nvme_status_field)>&& cb) {
                return fake->adminFwCommit(ctrl, action, slot, bpid,
                                           std::move(cb));
            });
        ON_CALL(*this, adminXfer)
            .WillByDefault(
                [this](
                    nvme_mi_ctrl_t ctrl, const nvme_mi_admin_req_hdr& admin_req,
                    std::span<uint8_t> data, unsigned int timeout_ms,
                    std::function<void(const std::error_code& ec,
                                       const nvme_mi_admin_resp_hdr& admin_resp,
                                       std::span<uint8_t> resp_data)>&& cb) {
            return fake->adminXfer(ctrl, admin_req, data, timeout_ms,
                                   std::move(cb));
            });
        ON_CALL(*this, adminSecuritySend).WillByDefault([]() { return; });
        ON_CALL(*this, adminSecurityReceive).WillByDefault([]() { return; });
    }

    MOCK_METHOD(int, getNID, (), (const override));
    MOCK_METHOD(int, getEID, (), (const override));
    MOCK_METHOD(void, miSubsystemHealthStatusPoll,
                (std::function<void(const std::error_code&,
                                    nvme_mi_nvm_ss_health_status*)> &&),
                (override));
    MOCK_METHOD(void, miScanCtrl,
                (std::function<void(const std::error_code&,
                                    const std::vector<nvme_mi_ctrl_t>&)>),
                (override));
    MOCK_METHOD(
        void, adminIdentify,
        (nvme_mi_ctrl_t ctrl, nvme_identify_cns cns, uint32_t nsid,
         uint16_t cntid,
         std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb),
        (override));
    MOCK_METHOD(
        void, adminGetLogPage,
        (nvme_mi_ctrl_t ctrl, nvme_cmd_get_log_lid lid, uint32_t nsid,
         uint8_t lsp, uint16_t lsi,
         std::function<void(const std::error_code&, std::span<uint8_t>)>&& cb),
        (override));
    MOCK_METHOD(
        void, adminFwCommit,
        (nvme_mi_ctrl_t ctrl, nvme_fw_commit_ca action, uint8_t slot, bool bpid,
         std::function<void(const std::error_code&, nvme_status_field)>&& cb),
        (override));
    MOCK_METHOD(void, adminXfer,
                (nvme_mi_ctrl_t ctrl, const nvme_mi_admin_req_hdr& admin_req,
                 std::span<uint8_t> data, unsigned int timeout_ms,
                 std::function<void(const std::error_code& ec,
                                    const nvme_mi_admin_resp_hdr& admin_resp,
                                    std::span<uint8_t> resp_data)>&& cb),
                (override));

    MOCK_METHOD(
        void, adminSecuritySend,
        (nvme_mi_ctrl_t ctrl, uint8_t proto, uint16_t proto_specific,
         std::span<uint8_t> data,
         std::function<void(const std::error_code&, int nvme_status)>&& cb),
        (override));
    MOCK_METHOD(void, adminSecurityReceive,
                (nvme_mi_ctrl_t ctrl, uint8_t proto, uint16_t proto_specific,
                 uint32_t transfer_length,
                 std::function<void(const std::error_code&, int nvme_status,
                                    const std::span<uint8_t> data)>&& cb),
                (override));

    std::shared_ptr<NVMeMiFake> fake;
};

class NVMeTest : public ::testing::Test
{
  protected:
    NVMeTest() :
        object_server(system_bus), nvme_intf(NVMeIntf::create<NVMeMiMock>(io)),
        mock(*std::dynamic_pointer_cast<NVMeMiMock>(
                  std::get<std::shared_ptr<NVMeMiIntf>>(
                      nvme_intf.getInferface()))
                  .get()),
        subsys(std::make_shared<NVMeSubsystem>(io, object_server, system_bus,
                                               subsys_path, "NVMe_1",
                                               SensorData{}, nvme_intf))
    {}

    static void SetUpTestSuite()
    {
        system_bus =
            std::make_shared<sdbusplus::asio::connection>(NVMeTest::io);
        system_bus->request_name("xyz.openbmc_project.NVMeTest");
    }

    void SetUp() override
    {
        subsys->start();
    }

    void TearDown() override
    {
        io.restart();
    }

    static constexpr char subsys_path[] =
        "/xyz/openbmc_project/inventory/Test_Chassis/Test_NVMe";

    static boost::asio::io_service io;
    static std::shared_ptr<sdbusplus::asio::connection> system_bus;
    sdbusplus::asio::object_server object_server;

    NVMeIntf nvme_intf;
    NVMeMiMock& mock;
    std::shared_ptr<NVMeSubsystem> subsys;
};

boost::asio::io_service NVMeTest::io;
std::shared_ptr<sdbusplus::asio::connection> NVMeTest::system_bus;

/**
 * @brief Test start and stop function of NVMeSubsystem
 *
 */
TEST_F(NVMeTest, TestSubsystemStartStop)
{
    using ::testing::AtLeast;
    boost::asio::steady_timer timer(io);

    EXPECT_CALL(mock, miSubsystemHealthStatusPoll).Times(AtLeast(1));
    EXPECT_CALL(mock, adminIdentify).Times(AtLeast(1));
    EXPECT_CALL(mock, miScanCtrl).Times(AtLeast(1));

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

/**
 * @brief Test NVMeMi return DriveFunctional(NSHDS.NSS.DF) = 0
 *
 */
TEST_F(NVMeTest, TestDriveFunctional)
{

    using ::testing::AtLeast;
    boost::asio::steady_timer timer(io);

    EXPECT_CALL(mock, miSubsystemHealthStatusPoll).Times(AtLeast(1));
    EXPECT_CALL(mock, adminIdentify).Times(AtLeast(1));
    EXPECT_CALL(mock, miScanCtrl).Times(AtLeast(1));

    // wait for subsystem initialization
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait([&](boost::system::error_code) {
        system_bus->async_method_call(
            [&](boost::system::error_code, const GetSubTreeType& result) {
            // Only PF and the enabled VF should be listed
            EXPECT_EQ(result.size(), 2);

            // mimik communication error of NVMeMI request
            ON_CALL(mock, miSubsystemHealthStatusPoll)
                .WillByDefault(
                    [&](std::function<void(const std::error_code&,
                                           nvme_mi_nvm_ss_health_status*)>&&
                            cb) {
                std::cerr << "mock device not functional health poll"
                          << std::endl;
                // return status.nss.df = 0
                return io.post([cb = std::move(cb)]() {
                    nvme_mi_nvm_ss_health_status status;
                    status.nss = 0;
                    cb({}, &status);
                });
                });

            // wait for storage controller destruction.
            timer.expires_after(std::chrono::seconds(2));
            timer.async_wait([&](boost::system::error_code) {
                system_bus->async_method_call(
                    [&](boost::system::error_code,
                        const GetSubTreeType& result) {
                    // no storage controller should be listed.
                    EXPECT_EQ(result.size(), 0);

                    // restart sending DF = 1
                    ON_CALL(mock, miSubsystemHealthStatusPoll)
                        .WillByDefault(
                            [&](std::function<void(
                                    const std::error_code&,
                                    nvme_mi_nvm_ss_health_status*)>&& cb) {
                        return mock.fake->miSubsystemHealthStatusPoll(
                            std::move(cb));
                        });
                    timer.expires_after(std::chrono::seconds(2));
                    timer.async_wait([&](boost::system::error_code) {
                        system_bus->async_method_call(
                            [&](boost::system::error_code,
                                const GetSubTreeType& result) {
                            // storage controller should be restored.
                            EXPECT_EQ(result.size(), 2);

                            subsys->stop();
                            io.post([&]() { io.stop(); });
                            },
                            "xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetSubTree",
                            subsys_path, 0,
                            std::vector<std::string>{
                                "xyz.openbmc_project.Inventory."
                                "Item.StorageController"});
                    });
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

/**
 * @brief Test NVMeMi returns Drive is absent (ec = no_such_device)
 *
 */
TEST_F(NVMeTest, TestDriveAbsent)
{
    using ::testing::AtLeast;
    boost::asio::steady_timer timer(io);

    EXPECT_CALL(mock, miSubsystemHealthStatusPoll).Times(AtLeast(1));
    EXPECT_CALL(mock, adminIdentify).Times(AtLeast(1));
    EXPECT_CALL(mock, miScanCtrl).Times(AtLeast(1));

    // wait for subsystem initialization
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait([&](boost::system::error_code) {
        system_bus->async_method_call(
            [&](boost::system::error_code, const GetSubTreeType& result) {
            // Only PF and the enabled VF should be listed
            EXPECT_EQ(result.size(), 2);

            // mimik communication error of NVMeMI request
            ON_CALL(mock, miSubsystemHealthStatusPoll)
                .WillByDefault(
                    [&](std::function<void(const std::error_code&,
                                           nvme_mi_nvm_ss_health_status*)>&&
                            cb) {
                std::cerr << "mock device absent health poll" << std::endl;
                // return no_such_device
                return io.post([cb = std::move(cb)]() {
                    cb(std::make_error_code(std::errc::no_such_device),
                       nullptr);
                });
                });

            // wait for storage controller destruction.
            timer.expires_after(std::chrono::seconds(2));
            timer.async_wait([&](boost::system::error_code) {
                system_bus->async_method_call(
                    [&](boost::system::error_code,
                        const GetSubTreeType& result) {
                    // no storage controller should be listed.
                    EXPECT_EQ(result.size(), 0);

                    // restart sending normal polling result
                    ON_CALL(mock, miSubsystemHealthStatusPoll)
                        .WillByDefault(
                            [&](std::function<void(
                                    const std::error_code&,
                                    nvme_mi_nvm_ss_health_status*)>&& cb) {
                        return mock.fake->miSubsystemHealthStatusPoll(
                            std::move(cb));
                        });
                    timer.expires_after(std::chrono::seconds(2));
                    timer.async_wait([&](boost::system::error_code) {
                        system_bus->async_method_call(
                            [&](boost::system::error_code,
                                const GetSubTreeType& result) {
                            // storage controller should be restored.
                            EXPECT_EQ(result.size(), 2);

                            subsys->stop();
                            io.post([&]() { io.stop(); });
                            },
                            "xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetSubTree",
                            subsys_path, 0,
                            std::vector<std::string>{
                                "xyz.openbmc_project.Inventory."
                                "Item.StorageController"});
                    });
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

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
