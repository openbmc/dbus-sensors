/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sdbusplus/exception.hpp>

#include <cerrno>

struct Unavailable : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Error.Unavailable";
    }
    const char* description() const noexcept override
    {
        return "The device is currently unavailable.";
    }
    int get_errno() const noexcept override
    {
        return EAGAIN;
    }
};

struct InvalidArgument : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Error.InvalidArgument";
    }
    const char* description() const noexcept override
    {
        return "Invalid argument.";
    }
    int get_errno() const noexcept override
    {
        return EINVAL;
    }
};
