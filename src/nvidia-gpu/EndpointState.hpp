/*
 * SPDX-FileCopyrightText: Copyright OpenBMC Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

enum class EndpointState
{
    Online,     // endpoint reachable, polling active
    Recovering, // Connectivity Degraded, awaiting recovery
};

enum class EndpointEvent
{
    ConnectivityDegraded, // received Connectivity=Degraded
    ConnectivityAvailable // received Connectivity=Available
};

// Side-effect action the caller must perform.
enum class EndpointAction
{
    None,      // no action (including guard: duplicate event while Recovering)
    GoOffline, // setOffline (externally driven Degraded)
    GoOnline,  // setOnline
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
        case EndpointState::Online:
            switch (e)
            {
                case EndpointEvent::ConnectivityDegraded:
                    return {EndpointState::Recovering,
                            EndpointAction::GoOffline};
                case EndpointEvent::ConnectivityAvailable:
                    return {EndpointState::Online, EndpointAction::None};
            }
            break;
        case EndpointState::Recovering:
            switch (e)
            {
                case EndpointEvent::ConnectivityAvailable:
                    return {EndpointState::Online, EndpointAction::GoOnline};
                case EndpointEvent::ConnectivityDegraded:
                    // guard: already recovering, ignore duplicate trigger
                    return {EndpointState::Recovering, EndpointAction::None};
            }
            break;
    }
    return {s, EndpointAction::None};
}
