/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// The LLDP objects of a device are gathered under one path so that they are
// reachable without walking the inventory, which they are not part of. The
// objects a port reports are placed under the one the device is configured
// through, so the two have to agree on where that is.
constexpr const char* lldpPathPrefix = "/xyz/openbmc_project/network/lldp";
