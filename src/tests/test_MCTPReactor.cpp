#include "MCTPEndpoint.hpp"
#include "MCTPReactor.hpp"
#include "Utils.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

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
    MOCK_METHOD(std::size_t, id, (), (const, override));
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

class MCTPReactorFixture : public testing::Test
{
  protected:
    void SetUp() override
    {
        reactor = std::make_shared<MCTPReactor>(assoc);
        device = std::make_shared<MockMCTPDevice>();
        EXPECT_CALL(*device, describe())
            .WillRepeatedly(testing::Return("mock device"));
        EXPECT_CALL(*device, id()).WillRepeatedly(testing::Return(0UL));

        endpoint = std::make_shared<MockMCTPEndpoint>();
        EXPECT_CALL(*endpoint, device())
            .WillRepeatedly(testing::Return(device));
        EXPECT_CALL(*endpoint, describe())
            .WillRepeatedly(testing::Return("mock endpoint"));
        EXPECT_CALL(*endpoint, eid()).WillRepeatedly(testing::Return(9));
        EXPECT_CALL(*endpoint, network()).WillRepeatedly(testing::Return(1));
    }

    void TearDown() override
    {
        // https://stackoverflow.com/a/10289205
        EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(endpoint.get()));
        EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(device.get()));
    }

    MockAssociationServer assoc;
    std::shared_ptr<MCTPReactor> reactor;
    std::shared_ptr<MockMCTPDevice> device;
    std::shared_ptr<MockMCTPEndpoint> endpoint;
};

TEST_F(MCTPReactorFixture, manageNullDevice)
{
    reactor->manageMCTPDevice("/test", {});
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, manageMockDeviceSetupFailure)
{
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(
            std::make_error_code(std::errc::permission_denied), endpoint));

    reactor->manageMCTPDevice("/test", device);
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, manageMockDevice)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation));
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"));

    EXPECT_CALL(*endpoint, remove()).WillOnce(testing::Invoke([&]() {
        removeHandler(endpoint);
    }));
    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .WillOnce(testing::SaveArg<2>(&removeHandler));

    EXPECT_CALL(*device, remove()).WillOnce(testing::Invoke([&]() {
        endpoint->remove();
    }));
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->manageMCTPDevice("/test", device);
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, manageMockDeviceDeferredSetup)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation));
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"));

    EXPECT_CALL(*endpoint, remove()).WillOnce(testing::Invoke([&]() {
        removeHandler(endpoint);
    }));
    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .WillOnce(testing::SaveArg<2>(&removeHandler));

    EXPECT_CALL(*device, remove()).WillOnce(testing::Invoke([&]() {
        endpoint->remove();
    }));
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(
            std::make_error_code(std::errc::permission_denied), endpoint))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));

    reactor->manageMCTPDevice("/test", device);
    reactor->tick();
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, unmanageLostDevice)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation));
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"));

    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .WillOnce(testing::SaveArg<2>(&removeHandler));
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));
    EXPECT_CALL(*device, remove).Times(0);

    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Terminate device
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, manageMockDeviceRemoveRecovered)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(2);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(1);

    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(2)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));

    EXPECT_CALL(*device, setup(testing::_))
        .Times(2)
        .WillRepeatedly(
            testing::InvokeArgument<0>(std::error_code(), endpoint));
    EXPECT_CALL(*device, remove).Times(0);

    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Enter Assigned
    reactor->tick();
    // Terminate device
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, gracefulRemoveOfRecovered)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(2);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(2);

    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(2)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));

    EXPECT_CALL(*endpoint, remove).WillOnce(testing::Invoke([&]() {
        removeHandler(endpoint);
    }));

    EXPECT_CALL(*device, remove).WillOnce(testing::Invoke([&]() {
        endpoint->remove();
    }));

    EXPECT_CALL(*device, setup(testing::_))
        .Times(2)
        .WillRepeatedly(
            testing::InvokeArgument<0>(std::error_code(), endpoint));

    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Enter Recovered
    reactor->tick();
    // Enable transition to Assigned
    reactor->unmanageMCTPDevice("/test");
    // Re-enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Terminate device
    reactor->unmanageMCTPDevice("/test");
}

TEST_F(MCTPReactorFixture, recoverFromQuarantine)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::function<void(const std::error_code&,
                       const std::shared_ptr<MCTPEndpoint>&)>
        setupHandler;
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(2);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(1);

    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(2)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));

    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint))
        .WillOnce(testing::SaveArg<0>(&setupHandler));

    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Enter Recovering
    reactor->tick();
    // Enter Quarantined
    reactor->unmanageMCTPDevice("/test");
    // Enter Recovered
    setupHandler(std::error_code(), endpoint);
}

TEST_F(MCTPReactorFixture, assigningFromQuarantine)
{
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(1);
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MCTPEndpoint>& ep)>
        setupHandler;
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::SaveArg<0>(&setupHandler));
    // Enter Assigning
    reactor->manageMCTPDevice("/test", device);
    // Enter Quarantine
    reactor->unmanageMCTPDevice("/test");
    // Re-enter Assigning
    reactor->manageMCTPDevice("/test", device);
    // Enter Assigned
    setupHandler(std::error_code(), endpoint);
}

TEST_F(MCTPReactorFixture, lostFromRecovering)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(1);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(1);
    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint))
        .WillOnce(testing::InvokeArgument<0>(
            std::make_error_code(std::errc::permission_denied), nullptr));
    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Re-enter Lost via Recovering
    reactor->tick();
}

TEST_F(MCTPReactorFixture, lostFromRecovered)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(2);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(2);

    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(2)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));
    EXPECT_CALL(*device, setup(testing::_))
        .Times(2)
        .WillRepeatedly(
            testing::InvokeArgument<0>(std::error_code(), endpoint));

    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Lost
    removeHandler(endpoint);
    // Enter Recovered via Recovering
    reactor->tick();
    // Enter Lost from Recovered
    removeHandler(endpoint);
}

TEST_F(MCTPReactorFixture, unassignedFromPending)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(1);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(1);
    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));
    EXPECT_CALL(*device, setup(testing::_))
        .Times(1)
        .WillRepeatedly(
            testing::InvokeArgument<0>(std::error_code(), endpoint));
    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Removing by preventing invocation of remove callback
    reactor->unmanageMCTPDevice("/test");
    // Enter Pending
    reactor->manageMCTPDevice("/test", device);
    // Enter Unassigned
    removeHandler(endpoint);
}

TEST_F(MCTPReactorFixture, removingFromPending)
{
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;
    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};
    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(1);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(1);
    EXPECT_CALL(*endpoint, remove).Times(1);
    EXPECT_CALL(*endpoint, subscribe(testing::_, testing::_, testing::_))
        .Times(1)
        .WillRepeatedly(testing::SaveArg<2>(&removeHandler));
    EXPECT_CALL(*device, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), endpoint));
    EXPECT_CALL(*device, remove).Times(1).WillOnce(testing::Invoke([&]() {
        endpoint->remove();
    }));
    // Enter Assigned
    reactor->manageMCTPDevice("/test", device);
    // Enter Removing
    reactor->unmanageMCTPDevice("/test");
    // Enter Pending
    reactor->manageMCTPDevice("/test", device);
    // Re-enter Removing
    reactor->unmanageMCTPDevice("/test");
    // Terminate
    removeHandler(endpoint);
}

TEST(MCTPReactor, replaceConfiguration)
{
    MockAssociationServer assoc{};
    auto reactor = std::make_shared<MCTPReactor>(assoc);
    std::function<void(const std::shared_ptr<MCTPEndpoint>& ep)> removeHandler;

    std::vector<Association> requiredAssociation{
        {"configured_by", "configures", "/test"}};

    EXPECT_CALL(assoc,
                associate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9",
                          requiredAssociation))
        .Times(2);
    EXPECT_CALL(
        assoc,
        disassociate("/au/com/codeconstruct/mctp1/networks/1/endpoints/9"))
        .Times(2);

    auto iep = std::make_shared<MockMCTPEndpoint>();
    EXPECT_CALL(*iep, describe())
        .WillRepeatedly(testing::Return("mock endpoint: initial"));
    EXPECT_CALL(*iep, eid()).WillRepeatedly(testing::Return(9));
    EXPECT_CALL(*iep, network()).WillRepeatedly(testing::Return(1));
    EXPECT_CALL(*iep, remove()).WillOnce(testing::Invoke([&]() {
        removeHandler(iep);
    }));
    EXPECT_CALL(*iep, subscribe(testing::_, testing::_, testing::_))
        .WillOnce(testing::SaveArg<2>(&removeHandler));

    auto idev = std::make_shared<MockMCTPDevice>();
    EXPECT_CALL(*idev, describe())
        .WillRepeatedly(testing::Return("mock device: initial"));
    EXPECT_CALL(*idev, id()).WillRepeatedly(testing::Return(0UL));
    EXPECT_CALL(*idev, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), iep));
    EXPECT_CALL(*idev, remove()).WillOnce(testing::Invoke([&]() {
        iep->remove();
    }));

    EXPECT_CALL(*iep, device()).WillRepeatedly(testing::Return(idev));

    auto rep = std::make_shared<MockMCTPEndpoint>();
    EXPECT_CALL(*rep, describe())
        .WillRepeatedly(testing::Return("mock endpoint: replacement"));
    EXPECT_CALL(*rep, eid()).WillRepeatedly(testing::Return(9));
    EXPECT_CALL(*rep, network()).WillRepeatedly(testing::Return(1));
    EXPECT_CALL(*rep, remove()).WillOnce(testing::Invoke([&]() {
        removeHandler(rep);
    }));
    EXPECT_CALL(*rep, subscribe(testing::_, testing::_, testing::_))
        .WillOnce(testing::SaveArg<2>(&removeHandler));

    auto rdev = std::make_shared<MockMCTPDevice>();
    EXPECT_CALL(*rdev, describe())
        .WillRepeatedly(testing::Return("mock device: replacement"));
    EXPECT_CALL(*rdev, id()).WillRepeatedly(testing::Return(0UL));
    EXPECT_CALL(*rdev, setup(testing::_))
        .WillOnce(testing::InvokeArgument<0>(std::error_code(), rep));
    EXPECT_CALL(*rdev, remove()).WillOnce(testing::Invoke([&]() {
        rep->remove();
    }));

    EXPECT_CALL(*rep, device()).WillRepeatedly(testing::Return(rdev));

    reactor->manageMCTPDevice("/test", idev);
    reactor->manageMCTPDevice("/test", rdev);
    reactor->tick();
    reactor->unmanageMCTPDevice("/test");

    EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(idev.get()));
    EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(iep.get()));
    EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(rdev.get()));
    EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(rep.get()));
}

TEST(MCTPReactor, concurrentEndpointSetupReactorTeardown)
{
    std::weak_ptr<MockMCTPEndpoint> wep;
    std::weak_ptr<MockMCTPDevice> wdev;
    std::function<void(const std::error_code& ec,
                       const std::shared_ptr<MCTPEndpoint>& ep)>
        setupHandler;
    {
        MockAssociationServer assoc{};
        auto reactor = std::make_shared<MCTPReactor>(assoc);
        auto device = std::make_shared<MockMCTPDevice>();
        auto endpoint = std::make_shared<MockMCTPEndpoint>();
        EXPECT_CALL(*endpoint, device())
            .WillRepeatedly(testing::Return(device));
        EXPECT_CALL(*endpoint, describe())
            .WillRepeatedly(testing::Return("mock endpoint"));
        EXPECT_CALL(*endpoint, eid()).WillRepeatedly(testing::Return(9));
        EXPECT_CALL(*endpoint, network()).WillRepeatedly(testing::Return(1));

        EXPECT_CALL(*device, describe())
            .WillRepeatedly(testing::Return("mock device"));
        EXPECT_CALL(*device, id()).WillRepeatedly(testing::Return(0UL));
        EXPECT_CALL(*device, setup(testing::_))
            .WillOnce(testing::SaveArg<0>(&setupHandler));

        reactor->manageMCTPDevice("/test", device);
    }
    setupHandler(std::make_error_code(std::errc::permission_denied), nullptr);
}

TEST(MCTPReactor, manageMockDeviceDelayedSetup)
{
    std::weak_ptr<MockMCTPEndpoint> wep;
    std::weak_ptr<MockMCTPDevice> wdev;
    MockAssociationServer assoc{};
    auto reactor = std::make_shared<MCTPReactor>(assoc);
    {
        std::function<void(const std::error_code& ec,
                           const std::shared_ptr<MCTPEndpoint>& ep)>
            setupHandler;
        {
            auto device = std::make_shared<MockMCTPDevice>();
            auto endpoint = std::make_shared<MockMCTPEndpoint>();
            EXPECT_CALL(*endpoint, device())
                .WillRepeatedly(testing::Return(device));
            EXPECT_CALL(*endpoint, describe())
                .WillRepeatedly(testing::Return("mock endpoint"));
            EXPECT_CALL(*endpoint, eid()).WillRepeatedly(testing::Return(9));
            EXPECT_CALL(*endpoint, network())
                .WillRepeatedly(testing::Return(1));

            EXPECT_CALL(*device, describe())
                .WillRepeatedly(testing::Return("mock device"));
            EXPECT_CALL(*device, id()).WillRepeatedly(testing::Return(0UL));
            EXPECT_CALL(*device, setup(testing::_))
                .WillOnce(testing::InvokeArgument<0>(
                    std::make_error_code(std::errc::permission_denied),
                    endpoint))
                .WillOnce(testing::SaveArg<0>(&setupHandler));

            reactor->manageMCTPDevice("/test", device);

            reactor->tick();
            reactor->unmanageMCTPDevice("/test");
            testing::Mock::VerifyAndClearExpectations(device.get());
            wdev = device;
            testing::Mock::VerifyAndClearExpectations(endpoint.get());
            wep = endpoint;
        }
        setupHandler(std::make_error_code(std::errc::permission_denied),
                     nullptr);
    }
    EXPECT_EQ(wdev.use_count(), 0);
    EXPECT_EQ(wep.use_count(), 0);
    reactor->tick();
}
