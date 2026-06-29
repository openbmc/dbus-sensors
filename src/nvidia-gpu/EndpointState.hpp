/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <optional>

enum class EndpointState
{
    Init,       // device created, running init() (build); not polling yet
    Online,     // init done, endpoint reachable, polling active
    Recovering, // Connectivity Degraded, awaiting recovery (still within
                // mctpd's Treclaim window; endpoint object still present)
    Offline,    // mctpd removed the endpoint object (recovery failed / device
                // gone); awaiting re-appearance
};

enum class EndpointEvent
{
    InitComplete,          // device init() finished -> start polling
    ConnectivityDegraded,  // received Connectivity=Degraded
    ConnectivityAvailable, // received Connectivity=Available
    EndpointRemoved,       // mctpd InterfacesRemoved for this endpoint
    EndpointReadded,       // mctpd InterfacesAdded re-matched to this device
};

enum class EndpointAction
{
    None,      // no action (e.g. guard: duplicate event while Recovering)
    GoOffline, // setOffline (stop polling, NaN reading sensors)
    GoOnline,  // setOnline (start/resume polling)
};

struct Transition
{
    EndpointState next;
    EndpointAction action;
};

constexpr Transition nextState(EndpointState s, EndpointEvent e)
{
    switch (s)
    {
        case EndpointState::Init:
            switch (e)
            {
                case EndpointEvent::InitComplete:
                    return {EndpointState::Online, EndpointAction::GoOnline};
                case EndpointEvent::ConnectivityDegraded:
                    return {EndpointState::Recovering,
                            EndpointAction::GoOffline};
                case EndpointEvent::EndpointRemoved:
                    return {EndpointState::Offline, EndpointAction::GoOffline};
                case EndpointEvent::ConnectivityAvailable:
                case EndpointEvent::EndpointReadded:
                    return {EndpointState::Init, EndpointAction::None};
            }
            break;
        case EndpointState::Online:
            switch (e)
            {
                case EndpointEvent::ConnectivityDegraded:
                    return {EndpointState::Recovering,
                            EndpointAction::GoOffline};
                case EndpointEvent::EndpointRemoved:
                    return {EndpointState::Offline, EndpointAction::GoOffline};
                case EndpointEvent::ConnectivityAvailable:
                case EndpointEvent::InitComplete:
                case EndpointEvent::EndpointReadded:
                    return {EndpointState::Online, EndpointAction::None};
            }
            break;
        case EndpointState::Recovering:
            switch (e)
            {
                case EndpointEvent::ConnectivityAvailable:
                    return {EndpointState::Online, EndpointAction::GoOnline};
                case EndpointEvent::EndpointRemoved:
                    // recovery failed within Treclaim -> endpoint removed
                    return {EndpointState::Offline, EndpointAction::GoOffline};
                case EndpointEvent::ConnectivityDegraded:
                case EndpointEvent::InitComplete:
                case EndpointEvent::EndpointReadded:
                    return {EndpointState::Recovering, EndpointAction::None};
            }
            break;
        case EndpointState::Offline:
            switch (e)
            {
                case EndpointEvent::EndpointReadded:
                    return {EndpointState::Online, EndpointAction::GoOnline};
                case EndpointEvent::InitComplete:
                case EndpointEvent::ConnectivityDegraded:
                case EndpointEvent::ConnectivityAvailable:
                case EndpointEvent::EndpointRemoved:
                    return {EndpointState::Offline, EndpointAction::None};
            }
            break;
    }
    return {s, EndpointAction::None};
}

enum class EndpointMatch
{
    Readd,  // the device is back at the endpoint it was known at
    Rebind, // the device is back at a new EID, so move it there
    Ignore, // the device is still reached through another endpoint
    Rescan, // no known device has this UUID
};

// What to do with an endpoint mctpd has added, given the state of the known
// device with the same UUID, if any, and whether it was known at that path.
constexpr EndpointMatch matchAddedEndpoint(std::optional<EndpointState> known,
                                           bool samePath)
{
    if (!known)
    {
        return EndpointMatch::Rescan;
    }
    if (samePath)
    {
        return EndpointMatch::Readd;
    }
    // Only a device mctpd has let go of can have been given a new EID.
    if (*known == EndpointState::Offline)
    {
        return EndpointMatch::Rebind;
    }
    return EndpointMatch::Ignore;
}
