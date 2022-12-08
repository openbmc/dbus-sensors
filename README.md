# dbus-sensors

dbus-sensors is a collection of sensor applications that provide the
xyz.openbmc_project.Sensor collection of interfaces. They read sensor values
from hwmon, d-bus, or direct driver access to provide readings. Some advance
non-sensor features such as fan presence, pwm control, and automatic cpu
detection (x86) are also supported.

## key features

- runtime re-configurable from d-bus (entity-manager or the like)

- isolated: each sensor type is isolated into its own daemon, so a bug in one
  sensor is unlikely to affect another, and single sensor modifications are
  possible

- async single-threaded: uses sdbusplus/asio bindings

- multiple data inputs: hwmon, d-bus, direct driver access

## dbus interfaces

A typical dbus-sensors object support the following dbus interfaces:

```
Path        /xyz/openbmc_project/sensors/<type>/<sensor_name>

Interfaces  xyz.openbmc_project.Sensor.Value
            xyz.openbmc_project.Sensor.Threshold.Critical
            xyz.openbmc_project.Sensor.Threshold.Warning
            xyz.openbmc_project.State.Decorator.Availability
            xyz.openbmc_project.State.Decorator.OperationalStatus
            xyz.openbmc_project.Association.Definitions

```

Sensor interfaces collection are described
[here](https://github.com/openbmc/phosphor-dbus-interfaces/tree/master/yaml/xyz/openbmc_project/Sensor).

Consumer examples of these interfaces are
[Redfish](https://github.com/openbmc/bmcweb/blob/master/redfish-core/lib/sensors.hpp),
[Phosphor-Pid-Control](https://github.com/openbmc/phosphor-pid-control),
[IPMI SDR](https://github.com/openbmc/phosphor-host-ipmid/blob/master/dbus-sdr/sensorcommands.cpp).

## Reactor

dbus-sensor daemons are [reactors](https://github.com/openbmc/entity-manager)
that dynamically create and update sensors configuration when system
configuration gets updated.

Using asio timers and async calls, dbus-sensor daemons read sensor values and
check thresholds periodically. PropertiesChanged signals will be broadcasted for
other services to consume when value or threshold status change. OperationStatus
is set to false if the sensor is determined to be faulty.

A simple sensor example can be found
[here](https://github.com/openbmc/entity-manager/blob/master/docs/my_first_sensors.md).

## configuration

Sensor devices are described using Exposes records in configuration file. Name
and Type fields are required. Different sensor types have different fields.
Refer to entity manager
[schema](https://github.com/openbmc/entity-manager/blob/master/schemas/legacy.json)
for complete list.

## sensor documentation

- [ExternalSensor](https://github.com/openbmc/docs/blob/master/designs/external-sensor.md)
  virtual sensor
