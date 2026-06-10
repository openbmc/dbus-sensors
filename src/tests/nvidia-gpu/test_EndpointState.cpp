/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "EndpointState.hpp"

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
