#include "MCTPReactor.hpp"

#include <memory>
#include <stdexcept>
#include <system_error>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MockMCTPDevice : public MCTPDevice
{
  public:
    ~MockMCTPDevice() override = default;

    MOCK_METHOD(void, setup,
                (std::function<void(const std::error_code& ec,
                                    const std::shared_ptr<MCTPEndpoint>& ep)> &&
                 added),
                (override));
    MOCK_METHOD(void, remove, (), (override));
    MOCK_METHOD(std::string, describe, (), (const, override));
};

class MockMCTPEndpoint : public MCTPEndpoint
{
  public:
    ~MockMCTPEndpoint() override = default;

    MOCK_METHOD(int, network, (), (const, override));
    MOCK_METHOD(uint8_t, eid, (), (const, override));
    MOCK_METHOD(void, subscribe,
                (Event && degraded, Event&& available, Event&& removed),
                (override));
    MOCK_METHOD(void, remove, (), (override));
    MOCK_METHOD(std::string, describe, (), (const, override));
    MOCK_METHOD(std::shared_ptr<MCTPDevice>, device, (), (const, override));
};

class MockAssociationServer : public AssociationServer
{
  public:
    ~MockAssociationServer() override = default;

    MOCK_METHOD(void, associate,
                (const std::string& path,
                 const std::vector<Association>& associations),
                (override));
    MOCK_METHOD(void, disassociate, (const std::string& path), (override));
};

static std::shared_ptr<MockMCTPDevice> createNullMCTPDevice(
    [[maybe_unused]] const std::string& interface,
    [[maybe_unused]] const std::vector<std::uint8_t>& physaddr,
    [[maybe_unused]] std::optional<std::uint8_t> eid)
{
    return {};
}

class NullDeviceTest : public testing::Test
{
  protected:
    void SetUp() override
    {
        reactor = std::make_shared<MCTPReactor>(createNullMCTPDevice,
                                                mockServer);
    }

    MockAssociationServer mockServer;
    std::shared_ptr<MCTPReactor> reactor;
};

TEST_F(NullDeviceTest, trackInvalidConfiguration)
{
    EXPECT_THROW(reactor->trackMCTPInterface("/mctpi2c0", {}),
                 std::invalid_argument);
    EXPECT_THROW(reactor->trackMCTPInterface("/mctpi2c0",
                                             {
                                                 {"Type", {"Wrong"}},
                                             }),
                 std::invalid_argument);
    EXPECT_THROW(reactor->trackMCTPInterface("/mctpi2c0",
                                             {
                                                 {"Transport", {"Unsupported"}},
                                                 {"Type", {"MCTPInterface"}},
                                             }),
                 std::invalid_argument);
    EXPECT_THROW(reactor->trackMCTPInterface("/mctpi2c0",
                                             {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Type", {"MCTPInterface"}},
                                             }),
                 std::invalid_argument);
}

TEST_F(NullDeviceTest, trackUnsupportedTransport)
{
    EXPECT_THROW(reactor->trackMCTPInterface("/mctpi2c0",
                                             {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"Unsupported"}},
                                                 {"Type", {"MCTPInterface"}},
                                             }),
                 std::invalid_argument);
}

TEST_F(NullDeviceTest, trackValidInterface)
{
    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
}

TEST_F(NullDeviceTest, removeUntrackedInterface)
{
    reactor->untrackMCTPInterface("/untracked");
}

TEST_F(NullDeviceTest, removeTrackedInterface)
{
    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
    reactor->untrackMCTPInterface("/mctpi2c0");
}

TEST_F(NullDeviceTest, manageMCTPDeviceUntrackedInterface)
{
    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"untracked"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
}

TEST_F(NullDeviceTest, manageMCTPDeviceNoType)
{
    EXPECT_THROW(reactor->manageMCTPDevice("/NVMe", {}), std::invalid_argument);
}

TEST_F(NullDeviceTest, manageMCTPDeviceWrongType)
{
    EXPECT_THROW(reactor->manageMCTPDevice("/NVMe",
                                           {
                                               {"Type", "Wrong"},
                                           }),
                 std::invalid_argument);
}

TEST_F(NullDeviceTest, manageMCTPDeviceMissingRequired)
{
    EXPECT_THROW(reactor->manageMCTPDevice("/NVMe",
                                           {
                                               {"Interface", {"mctpi2c0"}},
                                               {"Name", {"Bad Config"}},
                                               {"Type", {"MCTPDevice"}},
                                           }),
                 std::invalid_argument);

    EXPECT_THROW(reactor->manageMCTPDevice("/NVMe",
                                           {
                                               {"Address", {"0x1d"}},
                                               {"Name", {"Bad Config"}},
                                               {"Type", {"MCTPDevice"}},
                                           }),
                 std::invalid_argument);

    EXPECT_THROW(reactor->manageMCTPDevice("/NVMe",
                                           {
                                               {"Address", {"0x1d"}},
                                               {"Interface", {"mctpi2c0"}},
                                               {"Type", {"MCTPDevice"}},
                                           }),
                 std::invalid_argument);
}

class MockDeviceTest : public testing::Test
{
  protected:
    std::shared_ptr<MCTPDevice>
        createMockDevice(const std::string& interface,
                         const std::vector<std::uint8_t>& physaddr,
                         [[maybe_unused]] std::optional<std::uint8_t> eid)
    {
        auto endpointDescription = std::format("network: 1, eid: 9");
        EXPECT_CALL(*endpoint, describe())
            .WillRepeatedly(testing::Return(endpointDescription));

        auto deviceDescription = std::format("interface: {}, physaddr: {}",
                                             interface, physaddr.at(0));
        EXPECT_CALL(*device, describe())
            .WillRepeatedly(testing::Return(deviceDescription));

        return device;
    }

    void SetUp() override
    {
        device = std::make_shared<MockMCTPDevice>();
        endpoint = std::make_shared<MockMCTPEndpoint>();

        EXPECT_CALL(*endpoint, network()).WillRepeatedly(testing::Return(1));
        EXPECT_CALL(*endpoint, eid()).WillRepeatedly(testing::Return(9));
        EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
            .WillRepeatedly(testing::SaveArg<2>(&removeHandler));
        EXPECT_CALL(*endpoint, device())
            .WillRepeatedly(testing::Return(device));
        EXPECT_CALL(*endpoint, remove()).WillRepeatedly(testing::Invoke([&]() {
            mockServer.disassociate("/xyz/openbmc_project/mctp/1/9");
        }));

        EXPECT_CALL(*device, remove()).WillRepeatedly(testing::Invoke([&]() {
            endpoint->remove();
        }));

        reactor = std::make_shared<MCTPReactor>(
            std::bind_front(&MockDeviceTest::createMockDevice, this),
            mockServer);
    }

    void TearDown() override
    {
        // https://stackoverflow.com/a/10289205
        EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(endpoint.get()));
        EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(device.get()));
    }

    MockAssociationServer mockServer;
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::shared_ptr<MockMCTPDevice> device;
    std::shared_ptr<MockMCTPEndpoint> endpoint;
    std::shared_ptr<MCTPReactor> reactor;
};

TEST_F(MockDeviceTest, manageDeviceNoInterface)
{
    EXPECT_CALL(mockServer,
                associate("/xyz/openbmc_project/mctp/1/9", testing::_))
        .Times(0);
    EXPECT_CALL(mockServer, disassociate("/xyz/openbmc_project/mctp/1/9"))
        .Times(0);
    EXPECT_CALL(*device, setup(testing::_)).Times(0);

    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"mctpi2c0"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
    reactor->unmanageMCTPDevice("/NVMe");
}

TEST_F(MockDeviceTest, manageInterfaceBeforeDevice)
{
    EXPECT_CALL(mockServer,
                associate("/xyz/openbmc_project/mctp/1/9", testing::_))
        .Times(1);
    EXPECT_CALL(mockServer, disassociate("/xyz/openbmc_project/mctp/1/9"))
        .Times(1);
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"mctpi2c0"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
    reactor->unmanageMCTPDevice("/NVMe");
}

TEST_F(MockDeviceTest, manageDeviceBeforeInterface)
{
    EXPECT_CALL(mockServer,
                associate("/xyz/openbmc_project/mctp/1/9", testing::_))
        .Times(1);
    EXPECT_CALL(mockServer, disassociate("/xyz/openbmc_project/mctp/1/9"))
        .Times(1);
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"mctpi2c0"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
    reactor->unmanageMCTPDevice("/NVMe");
}

TEST_F(MockDeviceTest, manageDeviceWithRemove)
{
    EXPECT_CALL(mockServer,
                associate("/xyz/openbmc_project/mctp/1/9", testing::_))
        .Times(2);
    EXPECT_CALL(mockServer, disassociate("/xyz/openbmc_project/mctp/1/9"))
        .Times(2);
    EXPECT_CALL(*device, setup(testing::_))
        .Times(2)
        .WillRepeatedly(
            testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"mctpi2c0"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
    ASSERT_NE(removeHandler, nullptr);
    removeHandler(endpoint);
    reactor->tick();
    reactor->unmanageMCTPDevice("/NVMe");
}

TEST_F(MockDeviceTest, manageDeviceSetupFails)
{
    EXPECT_CALL(mockServer,
                associate("/xyz/openbmc_project/mctp/1/9", testing::_))
        .Times(1);
    EXPECT_CALL(mockServer, disassociate("/xyz/openbmc_project/mctp/1/9"))
        .Times(1);

    std::shared_ptr<MockMCTPEndpoint> empty{};
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(
            std::make_error_code(std::errc::operation_not_permitted), empty))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->trackMCTPInterface("/mctpi2c0", {
                                                 {"Name", {"mctpi2c0"}},
                                                 {"Transport", {"SMBus"}},
                                                 {"Type", {"MCTPInterface"}},
                                             });
    reactor->manageMCTPDevice("/NVMe", {
                                           {"Address", {"0x1d"}},
                                           {"Interface", {"mctpi2c0"}},
                                           {"Name", {"NVMe"}},
                                           {"Type", {"MCTPDevice"}},
                                       });
    reactor->tick();
    reactor->unmanageMCTPDevice("/NVMe");
}
