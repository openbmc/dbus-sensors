## GPIO-based Presence Detection
Author:
  Chu Lin (linchuyuan@google.com)
Primary assignee:
  Chu Lin (linchuyuan@google.com)
Created:
  2021-09-27

## Problem Description
The intent of the GPIO-based Presence Detection daemon design is to read a given
gpio line and report that line signal on dbus as a presence signal. In addition,
if a line signal goes down, it needs to have the ability to blink onboard LEDs
to signal there is a problem.

## Background and References
1. https://github.com/openbmc/phosphor-led-manager
2. https://www.dmtf.org/sites/default/files/Redfish_Cable_Management_Proposal_WIP_04-2021.pdf

## Detailed Design
The presence detection daemon has two important jobs. One is to read a gpio line
singal periodically and one is to report the read signal to dbus for other
daemons to consume. To read a gpio line, we can employ the libgpio. For more
detail about libgpio, please see https://libgpiod-dlang.dpldocs.info/gpiod.html.
Then, the daemon will report the presence to the dbus. If the presence signal
goes bad, the daemon will communicate with the phophosr-led-manager to either
blink or turn onboard LEDs on to indicate the problem. See following for better
illustration of the flow.
```

                                        ┌─────┐
                                        │start│
                                        └──┬──┘
                                           │
┌─────────────────────────┐ Notify ┌───────┴────────┐ ◄───────┐
│EntityManager New Config ├──────► │Wait  for config│         │ Invalid Config
│for Type GPIOCableSensing│        └───────┬────────┘ ────────┘
└─────────────────────────┘                │
                                           │
                                           │
                                  ┌────────┴──────────┐
                                  │Populate objects   │
                                  │on dbus and create │
                                  │gpio line polling  │
                                  │loop.              │
                                  └────────┬──────────┘
                                           │
                                           ▼
                                         ┌────┐
                                         │Wait│ ◄────────────────┐
                                         └─┬──┘                  │
                                           │                     │
                                           ▼                     │
                             ┌──────────────────────────────┐    │
                             │Read from gpio line and update│    │
                             │Dbus operational status       │ ───┘
                             └─────────────┬────────────────┘
                                           │
                                           ▼  Is OperationalStatus == False?
                                  ┌───────────────────┐
                                  │Assert on LED group│
                                  └───────────────────┘
```
## Configuration
The following example would monitor gpio line `osfp0_prsnt_l`  and associate
that to the presence of osfp0 (cable osfp0). If the presence goes out, it would
blink the attention LED group.
```
[
    {
      "Name": "osfp0",
      "Class": "Cable",
      "GpioLine": "osfp0_prsnt_l",
      "Polarity": "active_low",
      "Type": "GPIOPresence",
      "FaultLedGroup": [
          "attention"
      ]
    }
]
```

For each entry from the configuration, the daemon will create a corresponding
dbus object. For example, given the above config, it will create
```
`-/xyz
  `-/xyz/openbmc_project
    `-/xyz/openbmc_project/inventory
      `-/xyz/openbmc_project/inventory/item
        |-/xyz/openbmc_project/inventory/item/osfp0
```
The class property will tell the daemon which interface to implement. Currently,
it only has the cable interface but more interfaces will be added in the future.
By default, each object implements three interfaces. The cable interface, the
The cable interface is an empty interface for search purpose. If one wants to
list all the cable objects, he could call the ObjectMapper like the following.
The same would also apply to other future class.
```
busctl call -j xyz.openbmc_project.ObjectMapper \
/xyz/openbmc_project/object_mapper xyz.openbmc_project.ObjectMapper \
GetSubTree sias \
/xyz/openbmc_project/inventory/item 2 1 xyz.openbmc_project.Inventory.Item.Cable

"/xyz/openbmc_project/inventory/item/osfp0" : {
	"xyz.openbmc_project.GPIOPresence" : [
		"xyz.openbmc_project.Inventory.Item.Cable",
		"xyz.openbmc_project.State.Decorator.OperationalStatus"
	]
},
```
Moreover, cables usually come with connectors. For future improvement, we should
fill the connector interface to associate cables to connectors. Lastly, we have
operationalStatus. It indicates if the cable is properly seated or not. If
operationalStatus is false and FaultLedGroup is not empty, we will assert on
all LED groups from FaultLedGroup.


## Future Improvements
* This approach doesn't need to be limited for cables. The same approach can be
  applied to riser cards, storage drives and more.


## Testing
Testing can be accomplished via automated or manual testing to verify that:

* The presence state that is reported by the service is matching the result
  from linux gpioget command.

* If FaultLedGroup is not empty, onboard LED blinks if one or more cable is not
  connected properly.

* Once the faulty cable is reseated correct, the onboard LED would stop blinking
