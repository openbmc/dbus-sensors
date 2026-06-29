/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "EndpointState.hpp"

#include <optional>

#include <gtest/gtest.h>

TEST(EndpointStateTest, OnlineDegradedGoesOfflineRecovering)
{
    auto t =
        nextState(EndpointState::Online, EndpointEvent::ConnectivityDegraded);
    EXPECT_EQ(t.next, EndpointState::Recovering);
    EXPECT_EQ(t.action, EndpointAction::GoOffline);
}

TEST(EndpointStateTest, OnlineAvailableIsNoop)
{
    auto t =
        nextState(EndpointState::Online, EndpointEvent::ConnectivityAvailable);
    EXPECT_EQ(t.next, EndpointState::Online);
    EXPECT_EQ(t.action, EndpointAction::None);
}

TEST(EndpointStateTest, RecoveringAvailableGoesOnline)
{
    auto t = nextState(EndpointState::Recovering,
                       EndpointEvent::ConnectivityAvailable);
    EXPECT_EQ(t.next, EndpointState::Online);
    EXPECT_EQ(t.action, EndpointAction::GoOnline);
}

TEST(EndpointStateTest, RecoveringDuplicateDegradedIsGuarded)
{
    auto t = nextState(EndpointState::Recovering,
                       EndpointEvent::ConnectivityDegraded);
    EXPECT_EQ(t.next, EndpointState::Recovering);
    EXPECT_EQ(t.action, EndpointAction::None);
}

TEST(EndpointStateTest, InitCompleteGoesOnline)
{
    auto t = nextState(EndpointState::Init, EndpointEvent::InitComplete);
    EXPECT_EQ(t.next, EndpointState::Online);
    EXPECT_EQ(t.action, EndpointAction::GoOnline);
}

TEST(EndpointStateTest, InitDegradedGoesRecovering)
{
    auto t =
        nextState(EndpointState::Init, EndpointEvent::ConnectivityDegraded);
    EXPECT_EQ(t.next, EndpointState::Recovering);
    EXPECT_EQ(t.action, EndpointAction::GoOffline);
}

TEST(EndpointStateTest, OnlineRemovedGoesOffline)
{
    auto t = nextState(EndpointState::Online, EndpointEvent::EndpointRemoved);
    EXPECT_EQ(t.next, EndpointState::Offline);
    EXPECT_EQ(t.action, EndpointAction::GoOffline);
}

TEST(EndpointStateTest, RecoveringRemovedGoesOffline)
{
    auto t =
        nextState(EndpointState::Recovering, EndpointEvent::EndpointRemoved);
    EXPECT_EQ(t.next, EndpointState::Offline);
    EXPECT_EQ(t.action, EndpointAction::GoOffline);
}

TEST(EndpointStateTest, OfflineReaddedGoesOnline)
{
    auto t = nextState(EndpointState::Offline, EndpointEvent::EndpointReadded);
    EXPECT_EQ(t.next, EndpointState::Online);
    EXPECT_EQ(t.action, EndpointAction::GoOnline);
}

TEST(EndpointStateTest, OfflineDuplicateRemovedIsGuarded)
{
    auto t = nextState(EndpointState::Offline, EndpointEvent::EndpointRemoved);
    EXPECT_EQ(t.next, EndpointState::Offline);
    EXPECT_EQ(t.action, EndpointAction::None);
}

TEST(EndpointStateTest, OfflineConnectivityEventsAreGuarded)
{
    auto degraded =
        nextState(EndpointState::Offline, EndpointEvent::ConnectivityDegraded);
    EXPECT_EQ(degraded.next, EndpointState::Offline);
    EXPECT_EQ(degraded.action, EndpointAction::None);

    auto available =
        nextState(EndpointState::Offline, EndpointEvent::ConnectivityAvailable);
    EXPECT_EQ(available.next, EndpointState::Offline);
    EXPECT_EQ(available.action, EndpointAction::None);
}

TEST(EndpointStateTest, AddedEndpointWithUnknownUuidRescans)
{
    EXPECT_EQ(matchAddedEndpoint(std::nullopt, false), EndpointMatch::Rescan);
}

TEST(EndpointStateTest, AddedEndpointAtKnownPathReadds)
{
    EXPECT_EQ(matchAddedEndpoint(EndpointState::Offline, true),
              EndpointMatch::Readd);
}

TEST(EndpointStateTest, OfflineDeviceAtNewPathRebinds)
{
    EXPECT_EQ(matchAddedEndpoint(EndpointState::Offline, false),
              EndpointMatch::Rebind);
}

// A device mctpd still has an endpoint for cannot have moved, so another
// endpoint with its UUID must not take it away from the one it is using.
TEST(EndpointStateTest, ReachableDeviceAtNewPathIsIgnored)
{
    EXPECT_EQ(matchAddedEndpoint(EndpointState::Online, false),
              EndpointMatch::Ignore);
    EXPECT_EQ(matchAddedEndpoint(EndpointState::Recovering, false),
              EndpointMatch::Ignore);
}
