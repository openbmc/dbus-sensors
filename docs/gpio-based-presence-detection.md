## GPIO-based Presence Detection
Author:
  Chu Lin (linchuyuan@google.com)
Primary assignee:
  Chu Lin (linchuyuan@google.com)
Created:
  2021-09-27

## Problem Description
The intent of the GPIO-based Presence Detection daemon design is to read a given
gpio line and report that line signal on dbus as a presence signal.

## Background and References
1. https://github.com/openbmc/docs/blob/master/designs/gpio-based-cable-presence.md

## Detailed Design
1). The gpio detection daemon is designed to read a gpio line  and report
the signal back to dbus periodically. The gpio detection daemon would first
scan EntityManger for possible configuration. All configs with type
xyz.openbmc_project.Configuration.GPIOStatus is meant to be for the Gpio
daemon. An example configuration looks like the following.
```
{
  "Exposes": [
    {
      "Name": "cable0",
      "GpioLine": "cable0_prsnt_l",
      "Polarity": "active_low",
      "Type": "GPIOStatus"
    }
  ]
}
```
Name is the config name. The same name will be used when the gpio daemon
creates the private dbus interface to store the actual presence status. GpioLine
is the name of the gpio line. This name should be discoverable from the gpioinfo
command.

2). Once a config is provided to the gpio daemon. The gpio daemon would
start scanning the given gpio line periodically and populate the result to dbus.
see following for an example.
```
xyz.openbmc_project.GPIOStatus      interface -         -            -
.Name                               property  s         "cable0"      emits-change
.Present                            property  b         false        emits-change
```
The Name attribute is the same name from the config file and the Present
attribute is the actual gpio line signal.

3). The gpio daemon would only create the object with the private interface.
One still needs to create another object the the correct OpenBMC public
interface so that ipmi could understand the presence of inventory. This job can
be done using EntityManager. The following config tells EntityManager to
create an object with the cable interface and associate the presence attribute
to the private interface's presence attribute.
```
  {
    "Exposes": [],
    "Probe": "xyz.openbmc_project.GPIOStatus({'Name': 'cable0'})",
    "Name": "cable0",
    "Type": "Cable",
    "xyz.openbmc_project.Inventory.Item": {
      "Present": "$Present",
      "PrettyName": "$Name"
    },
    "xyz.openbmc_project.Inventory.Item.Cable": {
    }
  }
```
The probe statement tells EM to only create this object if and only if
there is an object with interface xyz.openbmc_project.GPIOStatus and its name is
cable0. In addition, there are two interfaces attached to this object.
Interface xyz.openbmc_project.Inventory.Item has one attribute that is called
"Present" and it is value is the same as the "Present" value from probe
statement. On the other hand, interface
"xyz.openbmc_project.Inventory.Item.Cable" is an empty interface. Moreover,
One can be creative about how to create the interfaces. The above just an
example of cable presence.

## Flow
```
                                        ┌─────┐
                                        │start│
                                        └──┬──┘
                                           │
┌─────────────────────────┐ Notify ┌───────┴────────┐ ◄───────┐
│EntityManager New Config ├──────► │Wait  for config│         │ Invalid Config
│for Type GpioStatus      │        └───────┬────────┘ ────────┘
└─────────────────────────┘                │
                                           │
                                           │
 ┌────────────────────────┐       ┌────────┴──────────┐
 │EM creates the dbus obj │       │Populate objects   │
 │and associate the obj   │ probe │on dbus and create │
 │to the private obj from │◄──────┤gpio line polling  │
 │the gpio daemon         │       │loop.              │
 └────────────────────────┘       └────────┬──────────┘
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
                             └──────────────────────────────┘
```

## Testing
Testing can be accomplished via automated or manual testing to verify that:

* The presence state that is reported by the service is matching the result
  from linux gpioget command.

* If FaultLedGroup is not empty, onboard LED blinks if one or more cable is not
  connected properly.

* Once the faulty cable is reseated correct, the onboard LED would stop blinking
