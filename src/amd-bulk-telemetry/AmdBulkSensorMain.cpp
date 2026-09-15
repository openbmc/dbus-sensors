/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>

int main()
{
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t inventory{ctx.get_bus(),
                                           "/xyz/openbmc_project/inventory"};

    ctx.request_name("xyz.openbmc_project.AmdBulkSensor");
    ctx.run();

    return 0;
}
