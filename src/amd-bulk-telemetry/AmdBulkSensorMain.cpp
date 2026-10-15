/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MctpDiscovery.hpp"
#include "OcpVdmIana.hpp"

#include <MctpAsyncRequester.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>

int main()
{
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t inventory{ctx.get_bus(),
                                           "/xyz/openbmc_project/inventory"};
    sdbusplus::server::manager_t metric{ctx.get_bus(),
                                        "/xyz/openbmc_project/metric"};
    sdbusplus::server::manager_t software{ctx.get_bus(),
                                          "/xyz/openbmc_project/software"};

    mctp::AsyncRequester requester{ctx, ocp::bulk::mctpBinding};
    ocp::bulk::MctpDiscovery discovery{ctx, requester};

    ctx.spawn(discovery.run());
    ctx.request_name("xyz.openbmc_project.AmdBulkSensor");
    ctx.run();

    return 0;
}
