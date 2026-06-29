/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

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

// Side-effect action the caller must perform.
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

// Pure function: given current state + event -> next state + action.
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
                    // guard: already recovering, ignore
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
                    // guard: no endpoint object while Offline, ignore
                    return {EndpointState::Offline, EndpointAction::None};
            }
            break;
    }
    return {s, EndpointAction::None};
}
