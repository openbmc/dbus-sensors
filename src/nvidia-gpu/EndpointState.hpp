/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

enum class EndpointState
{
    Init,       // device created, running init() (build); not polling yet
    Online,     // init done, endpoint reachable, polling active
    Recovering, // Connectivity Degraded, awaiting recovery
};

enum class EndpointEvent
{
    InitComplete,         // device init() finished -> start polling
    ConnectivityDegraded, // received Connectivity=Degraded
    ConnectivityAvailable // received Connectivity=Available
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
// InterfacesRemoved/Added are not routed here (they are create/teardown,
// handled directly by DeviceManager).
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
                case EndpointEvent::ConnectivityAvailable:
                    return {EndpointState::Init, EndpointAction::None};
            }
            break;
        case EndpointState::Online:
            switch (e)
            {
                case EndpointEvent::ConnectivityDegraded:
                    return {EndpointState::Recovering,
                            EndpointAction::GoOffline};
                case EndpointEvent::ConnectivityAvailable:
                case EndpointEvent::InitComplete:
                    return {EndpointState::Online, EndpointAction::None};
            }
            break;
        case EndpointState::Recovering:
            switch (e)
            {
                case EndpointEvent::ConnectivityAvailable:
                    return {EndpointState::Online, EndpointAction::GoOnline};
                case EndpointEvent::ConnectivityDegraded:
                case EndpointEvent::InitComplete:
                    // guard: already recovering, ignore
                    return {EndpointState::Recovering, EndpointAction::None};
            }
            break;
    }
    return {s, EndpointAction::None};
}
