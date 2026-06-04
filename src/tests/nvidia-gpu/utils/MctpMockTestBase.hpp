/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "DbusMockTestBase.hpp"
#include "MockMctpRequester.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

/**
 * D-Bus fixture plus a fresh per-test MctpRequesterMock installed as the
 * link-seam target. Configure MCTP traffic with EXPECT_CALL on `mctpMock`.
 */
class MctpMockTestBase : public DbusMockTestBase
{
  protected:
    void SetUp() override
    {
        DbusMockTestBase::SetUp();
        if (testing::Test::IsSkipped())
        {
            return;
        }
        mock_mctp::setActiveMock(&mctpMock);
    }

    void TearDown() override
    {
        mock_mctp::setActiveMock(nullptr);
        EXPECT_TRUE(testing::Mock::VerifyAndClearExpectations(&mctpMock));
        DbusMockTestBase::TearDown();
    }

    testing::NiceMock<mock_mctp::MctpRequesterMock> mctpMock;
};
