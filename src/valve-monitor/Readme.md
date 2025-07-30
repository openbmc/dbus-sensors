# Valve Monitor

The valve monitor consumes the EM Configuration and provides the open status of
the specified valve as sensor data. It also generates an event message whenever
the valve's open status changes. Additionally, it offers a control interface to
open and close the valve.

## EM Configuration

The example of EM Configuration is as follows:

```json
{
  "Name": "ReturnValve",
  "OpenControlPinName": "WATER_VALVE_CLOSED_N",
  "OpenControlValue": false,
  "OpenPinName": "RETURN_CNTL_FB_D_R",
  "OpenPolarity": "High",
  "Type": "GPIOValve"
}
```

For the GPIO hardware interface, the valve's open status is indicated as a
percentage, with 100% representing fully open and 0% representing fully closed,
based on the GPIO signal state and its polarity.

## D-Bus Interfaces

The following D-Bus interfaces are implemented:

- xyz.openbmc_project.Sensor.Value
- xyz.openbmc_project.State.Decorator.OperationalStatus
- xyz.openbmc_project.State.Decorator.Availability
- xyz.openbmc_project.Association.Definitions

  Above interfaces are exposed on the following object path:

  /xyz/openbmc_project/sensors/valve_open/\<ValveName\>

  where \<ValveName\> corresponds to the valve name specified in the EM
  Configuration.

- xyz.openbmc_project.Control.Valve

  Above interface is exposed on the following object path:

  /xyz/openbmc_project/control/valve_open/\<ValveName\>

  where \<ValveName\> corresponds to the valve name specified in the EM
  Configuration.
