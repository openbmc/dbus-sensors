/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <MctpRequester.hpp>
#include <MessagePackUnpackUtils.hpp>
#include <NvidiaGpuMctpVdm.hpp>
#include <NvidiaLongRunningHandler.hpp>
#include <OcpMctpVdm.hpp>
#include <SerialQueue.hpp>
#include <boost/system/error_code.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// Helper that consolidates the long-running NSM command pipeline:
// queue submission, MCTP request/response, common-header decode,
// SUCCESS vs ACCEPTED branching, long-running response event handling.
//
// Per-command specifics (encode, payload decode) are supplied through
// Config callables. The helper has no knowledge of D-Bus, inventory,
// or which metric is calling it.
class NvidiaGpuLongRunningCommand :
    public std::enable_shared_from_this<NvidiaGpuLongRunningCommand>
{
  public:
    struct Config
    {
        // Identifies the metric in log messages (e.g. "GPU Current
        // Utilization").
        std::string metricName;

        gpu::MessageType messageType;
        gpu::PlatformEnvironmentalCommands commandId;

        // Size of the MCTP request buffer. The helper allocates a buffer of
        // this size once at construction and reuses it across update() calls,
        // so no per-request allocation occurs.
        size_t requestSize;

        // Fills the request bytes into `buf`. Returns 0 on success.
        std::move_only_function<int(std::span<uint8_t> buf)> encodeRequest;

        // Invoked when the device replies SUCCESS synchronously. `fullBuffer`
        // is the entire MCTP response (including the common response header)
        // so the caller can reuse the existing decodeGet*Response() helpers.
        std::move_only_function<void(std::span<const uint8_t> fullBuffer)>
            onImmediateSuccess;

        // Invoked when the long-running response event arrives successfully.
        // `buf` is an UnpackBuffer wrapping the bytes the device returned for
        // the command (no OCP/common header). The caller continues unpacking
        // payload fields directly.
        std::move_only_function<void(UnpackBuffer& buf)> onLongRunningPayload;
    };

    NvidiaGpuLongRunningCommand(
        uint8_t eid, mctp::MctpRequester& mctpRequester,
        std::shared_ptr<SerialQueue> longRunningQueue,
        std::shared_ptr<NvidiaLongRunningResponseHandler>
            longRunningResponseHandler,
        Config config);

    void update();

  private:
    void doUpdate(SerialQueue::ReleaseHandle handle);

    void processResponse(SerialQueue::ReleaseHandle handle,
                         const std::error_code& ec,
                         std::span<const uint8_t> buffer);

    void processLongRunningResponse(
        boost::system::error_code ec,
        ocp::accelerator_management::CompletionCode cc, uint16_t reasonCode,
        std::span<const uint8_t> responseData);

    uint8_t eid;

    mctp::MctpRequester& mctpRequester;

    std::shared_ptr<SerialQueue> longRunningQueue;

    std::shared_ptr<NvidiaLongRunningResponseHandler>
        longRunningResponseHandler;

    Config config;

    std::vector<uint8_t> request;
};
