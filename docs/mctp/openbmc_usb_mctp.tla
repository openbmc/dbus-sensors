-------------------------- MODULE openbmc_usb_mctp --------------------------

EXTENDS Integers, Sequences, TLC, FiniteSets

CONSTANTS MAX_QUEUE_DEPTH, d1

const_1 == {d1}
symm_1 == Permutations(const_1)

CONSTANT Devices

DeviceStateType == { "unplugged", "plugged" }
DeviceEventType == [ dev: Devices, state: DeviceStateType ]
PlugEvent(dev) == [ dev |-> dev, state |-> "plugged" ]
UnplugEvent(dev) == [ dev |-> dev, state |-> "unplugged"]

InventoryStateType == { "absent", "present" }
InventoryEventType == [ dev: Devices, state: InventoryStateType ]
PublishEvent(dev) == [ dev |-> dev, state |-> "present" ]
RetractEvent(dev) == [ dev |-> dev, state |-> "absent" ]

NetdevStateType == { "absent", "present" }
NetdevEventType == [ dev: Devices, state: NetdevStateType ]
NetdevAddEvent(dev) == [ dev |-> dev, state |-> "present" ]
NetdevRemoveEvent(dev) == [ dev |-> dev, state |-> "absent" ]

MctpdActionType == { "assign", "remove" }
MctpdRequestType == [ dev: Devices, action: MctpdActionType ]
MctpdAssignRequest(dev) == [ dev |-> dev, action |-> "assign" ]
MctpdRemoveRequest(dev) == [ dev |-> dev, action |-> "remove" ]

MctpdEndpointStateType == { "unmanaged", "managed" }
MctpdEndpointEventType == [ dev: Devices, state: MctpdEndpointStateType ]
MctpdEndpointAddEvent(dev) == [ dev |-> dev, state |-> "managed" ]
MctpdEndpointRemoveEvent(dev) == [ dev |-> dev, state |-> "unmanaged" ]

ReactorEndpointStateType == { "unmanaged", "assigning", "unassigned", "assigned", "quarantine", "lost", "recovering", "recovered", "removing" }
ReactorEndpointEntryType == [ Devices -> ReactorEndpointStateType ]
ReactorEndpointInit == [ dev \in Devices |-> "unmanaged" ]

VARIABLES devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps

vars == <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps>>

TypeInvariant ==
    /\ devs \subseteq Devices
    /\ usb_config_q \in Seq(DeviceEventType)
    /\ inventory_q \in Seq(DeviceEventType)
    /\ mctp_iface_q \in Seq(NetdevEventType)
    /\ ifaces \subseteq Devices
    /\ reactor_dev_q \in Seq(InventoryEventType)
    /\ mctpd_q \in Seq(MctpdRequestType)
    /\ mctpd_offers \subseteq Devices
    /\ assigned \subseteq Devices
    /\ dev_accepts \subseteq Devices
    /\ mctpd_assigned \subseteq Devices
    /\ reactor_ep_q \in Seq(MctpdEndpointEventType)
    /\ reactor_eps \in ReactorEndpointEntryType

Invariants == TypeInvariant

Liveness == [](\A dev \in Devices: dev \in devs ~> (reactor_eps[dev] = "assigned" \/ dev \notin devs))

Properties == Liveness

Init ==
    /\ devs = {}
    /\ usb_config_q = <<>>
    /\ inventory_q = <<>>
    /\ mctp_iface_q = <<>>
    /\ ifaces = {}
    /\ reactor_dev_q = <<>>
    /\ mctpd_q = <<>>
    /\ mctpd_offers = {}
    /\ assigned = {}
    /\ dev_accepts = {}
    /\ mctpd_assigned = {}
    /\ reactor_ep_q = <<>>
    /\ reactor_eps = ReactorEndpointInit

Enter(set, dev) == set' = set \union { dev }
Exit(set, dev) == set' = set \ { dev }

Plug(dev) ==
    /\ dev \notin devs
    /\ Len(usb_config_q) < MAX_QUEUE_DEPTH
    /\ Len(inventory_q) < MAX_QUEUE_DEPTH
    /\ Enter(devs, dev)
    /\ usb_config_q' = Append(usb_config_q, PlugEvent(dev))
    /\ inventory_q' = Append(inventory_q, PlugEvent(dev))
    /\ UNCHANGED <<mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps>>

Unplug(dev) ==
    /\ dev \in devs
    /\ Len(usb_config_q) < MAX_QUEUE_DEPTH
    /\ Len(inventory_q) < MAX_QUEUE_DEPTH
    /\ Exit(devs, dev)
    /\ Exit(ifaces, dev)
    /\ Exit(assigned, dev)
    /\ Exit(dev_accepts, dev)
    /\ usb_config_q' = Append(usb_config_q, UnplugEvent(dev))
    /\ inventory_q' = Append(inventory_q, UnplugEvent(dev))
    /\ UNCHANGED <<mctp_iface_q, reactor_dev_q, mctpd_q, mctpd_offers, mctpd_assigned, reactor_ep_q, reactor_eps>>

PublishInventory(dev) ==
    /\ Len(reactor_dev_q) < MAX_QUEUE_DEPTH
    /\ reactor_dev_q' = Append(reactor_dev_q, PublishEvent(dev))

UnpublishInventory(dev) ==
    /\ Len(reactor_dev_q) < MAX_QUEUE_DEPTH
    /\ reactor_dev_q' = Append(reactor_dev_q, RetractEvent(dev))

UsbConfig(dev) ==
    /\ Len(mctp_iface_q) < MAX_QUEUE_DEPTH
    /\ mctp_iface_q' = Append(mctp_iface_q, NetdevAddEvent(dev))

UsbDeconfig(dev) ==
    /\ Len(mctp_iface_q) < MAX_QUEUE_DEPTH
    /\ mctp_iface_q' = Append(mctp_iface_q, NetdevRemoveEvent(dev))

MctpdAssignEndpoint(dev) ==
    /\ Len(mctpd_q) < MAX_QUEUE_DEPTH
    /\ mctpd_q' = Append(mctpd_q, MctpdAssignRequest(dev))

MctpdRemoveEndpoint(dev) ==
    /\ Len(mctpd_q) < MAX_QUEUE_DEPTH
    /\ mctpd_q' = Append(mctpd_q, MctpdRemoveRequest(dev))

MctpdEndpointAdd(dev) ==
    /\ Len(reactor_ep_q) < MAX_QUEUE_DEPTH
    /\ reactor_ep_q' = Append(reactor_ep_q, MctpdEndpointAddEvent(dev))

MctpdEndpointRemove(dev) ==
    /\ Len(reactor_ep_q) < MAX_QUEUE_DEPTH
    /\ reactor_ep_q' = Append(reactor_ep_q, MctpdEndpointRemoveEvent(dev))

MctpdControlEvent ==
    /\ \E dev \in dev_accepts:
          /\ Exit(dev_accepts, dev)
          /\ Enter(mctpd_assigned, dev)
          /\ MctpdEndpointAdd(dev)
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, reactor_eps>>

DeviceAcceptOffer(dev) ==
    /\ dev \in mctpd_offers
    /\ Exit(mctpd_offers, dev)
    /\ Enter(assigned, dev)
    /\ IF dev \in ifaces THEN Enter(dev_accepts, dev) ELSE UNCHANGED <<dev_accepts>>
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_assigned, reactor_ep_q, reactor_eps>>

MctpdIfaceEvent ==
    /\ \E dev \in mctpd_assigned \ ifaces:
        /\ Exit(mctpd_assigned, dev)
        /\ MctpdEndpointRemove(dev)
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, reactor_eps>>

MctpdQEvent ==
    /\ Len(mctpd_q) # 0
    /\ mctpd_q' = Tail(mctpd_q)
       \* FIXME: reactor_eps hack! Need method response values
    /\ LET ev == Head(mctpd_q)
       IN \/ /\ ev.action = "assign"
             /\ IF ev.dev \in devs /\ ev.dev \in ifaces
                THEN /\ Enter(mctpd_offers, ev.dev)
                     /\ UNCHANGED <<reactor_eps>>
                ELSE /\ reactor_eps' = CASE reactor_eps[ev.dev] = "assigning" -> [ reactor_eps EXCEPT ![ev.dev] = "unassigned" ]
                                         [] reactor_eps[ev.dev] = "quarantine" -> [ reactor_eps EXCEPT ![ev.dev] = "unmanaged" ]
                                         [] reactor_eps[ev.dev] = "recovering" -> [ reactor_eps EXCEPT ![ev.dev] = "lost" ]
                                         [] OTHER -> reactor_eps
                     /\ UNCHANGED <<mctpd_offers>>
             /\ UNCHANGED <<mctpd_assigned, reactor_ep_q>>
          \/ /\ ev.action = "remove"
             /\ IF ev.dev \in mctpd_assigned
                THEN /\ Exit(mctpd_assigned, ev.dev)
                     /\ MctpdEndpointRemove(ev.dev)
                ELSE UNCHANGED <<mctpd_assigned, reactor_ep_q>>
             /\ UNCHANGED <<mctpd_offers, reactor_eps>>
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, assigned, dev_accepts>>

ReactorDeferredSetup ==
    /\ \E dev \in DOMAIN reactor_eps:
          /\ reactor_eps[dev] \in { "unassigned", "lost" }
          /\ dev \in ifaces \* NOTE: Not present in upstream mctpreactor implementation
          /\ MctpdAssignEndpoint(dev)
          /\ reactor_eps' = CASE reactor_eps[dev] = "unassigned" -> [ reactor_eps EXCEPT ![dev] = "assigning" ]
                              [] reactor_eps[dev] = "lost" -> [ reactor_eps EXCEPT ![dev] = "recovering" ]
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q>>

ReactorEndpointQEvent ==
    /\ Len(reactor_ep_q) # 0
    /\ LET ev == Head(reactor_ep_q)
       IN \/ /\ ev.state = "managed"
             /\ reactor_eps' = CASE reactor_eps[ev.dev] = "assigning" -> [ reactor_eps EXCEPT ![ev.dev] = "assigned" ]
                                 [] reactor_eps[ev.dev] = "recovering" -> [ reactor_eps EXCEPT ![ev.dev] = "recovered" ]
                                 [] OTHER -> reactor_eps
          \/ /\ ev.state = "unmanaged"
             /\ reactor_eps' = CASE reactor_eps[ev.dev] = "assigned" -> [ reactor_eps EXCEPT ![ev.dev] = "lost" ]
                                 [] reactor_eps[ev.dev] = "quarantine" -> [ reactor_eps EXCEPT ![ev.dev] = "unmanaged" ]
                                 [] reactor_eps[ev.dev] = "recovered" -> [ reactor_eps EXCEPT ![ev.dev] = "lost" ]
                                 [] reactor_eps[ev.dev] = "removing" -> [ reactor_eps EXCEPT ![ev.dev] = "unmanaged" ]
                                 [] OTHER -> reactor_eps
    /\ reactor_ep_q' = Tail(reactor_ep_q)
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned>>

ReactorDeviceQEvent ==
    /\ Len(reactor_dev_q) # 0
    /\ LET ev == Head(reactor_dev_q)
       IN \/ /\ ev.state = "present"
             /\ MctpdAssignEndpoint(ev.dev)
             /\ reactor_eps' = CASE reactor_eps[ev.dev] = "unmanaged" -> [ reactor_eps EXCEPT ![ev.dev] = "assigning" ]
                                 [] reactor_eps[ev.dev] = "quarantine" -> [ reactor_eps EXCEPT ![ev.dev] = "assigning" ] \* FIXME: questionable
                                 [] reactor_eps[ev.dev] = "lost" -> [ reactor_eps EXCEPT ![ev.dev] = "assigning" ]
                                 [] reactor_eps[ev.dev] = "recovered" -> [ reactor_eps EXCEPT ![ev.dev] = "assigned" ]
                                 [] reactor_eps[ev.dev] = "removing" -> [ reactor_eps EXCEPT ![ev.dev] = "assigning" ]
                                 [] OTHER -> reactor_eps
          \/ /\ ev.state = "absent"
             /\ MctpdRemoveEndpoint(ev.dev)
             /\ reactor_eps' = CASE reactor_eps[ev.dev] = "assigning" -> [ reactor_eps EXCEPT ![ev.dev] = "quarantine" ]
                                 [] reactor_eps[ev.dev] = "unassigned" -> [ reactor_eps EXCEPT ![ev.dev] = "unmanaged" ]
                                 [] reactor_eps[ev.dev] = "assigned" -> [ reactor_eps EXCEPT ![ev.dev] = "removing" ]
                                 [] reactor_eps[ev.dev] = "lost" -> [ reactor_eps EXCEPT ![ev.dev] = "unmanaged" ]
                                 [] reactor_eps[ev.dev] = "recovered" -> [ reactor_eps EXCEPT ![ev.dev] = "removing" ]
                                 [] OTHER -> reactor_eps
    /\ reactor_dev_q' = Tail(reactor_dev_q)
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, mctp_iface_q, ifaces, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q>>

MctpIfaceQEvent ==
    /\ Len(mctp_iface_q) # 0
    /\ LET ev == Head(mctp_iface_q)
       IN IF ev.dev \in devs
          THEN \/ ev.state = "present" /\ Enter(ifaces, ev.dev)
               \/ ev.state = "absent" /\ Exit(ifaces, ev.dev)
          ELSE UNCHANGED <<ifaces>>
    /\ mctp_iface_q' = Tail(mctp_iface_q)
    /\ UNCHANGED <<devs, usb_config_q, inventory_q, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps>>

InventoryQEvent ==
    /\ Len(inventory_q) # 0
    /\ LET ev == Head(inventory_q)
       IN \/ /\ ev.state = "plugged"
             /\ PublishInventory(ev.dev)
          \/ /\ ev.state = "unplugged"
             /\ UnpublishInventory(ev.dev)
    /\ inventory_q' = Tail(inventory_q)
    /\ UNCHANGED <<devs, usb_config_q, mctp_iface_q, ifaces, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps>>

UsbConfigQEvent ==
    /\ Len(usb_config_q) # 0
    /\ LET ev == Head(usb_config_q)
       IN \/ /\ ev.state = "plugged"
             /\ UsbConfig(ev.dev) \* Interface needs an explicit "configure" step when plugged
          \/ /\ ev.state = "unplugged"
             /\ UNCHANGED <<mctp_iface_q>> \* Interface doesn't need an explicit "deconfigure" step when removed"
    /\ usb_config_q' = Tail(usb_config_q)
    /\ UNCHANGED <<devs, inventory_q, ifaces, reactor_dev_q, mctpd_q, mctpd_offers, assigned, dev_accepts, mctpd_assigned, reactor_ep_q, reactor_eps>>

Next ==
    \/ \E dev \in Devices:
        \/ Plug(dev)
        \/ Unplug(dev)
        \/ DeviceAcceptOffer(dev)
    \/ UsbConfigQEvent
    \/ InventoryQEvent
    \/ MctpIfaceQEvent
    \/ ReactorDeviceQEvent
    \/ MctpdQEvent
    \/ MctpdControlEvent
    \/ ReactorEndpointQEvent
    \/ ReactorDeferredSetup
    \/ MctpdIfaceEvent

Spec == Init /\ [][Next]_vars /\ WF_vars(Next)

=============================================================================
\* Modification History
\* Last modified Sun Nov 02 21:48:08 ACDT 2025 by andrew
\* Created Fri Oct 31 16:01:47 ACDT 2025 by andrew
