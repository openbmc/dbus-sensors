# dbus-sensors

dbus-sensors is a collection of sensor applications that provide the
xyz.openbmc_project.Sensor collection of interfaces. They read sensor values
from hwmon, d-bus, or direct driver access to provide readings. Some advance
non-sensor features such as fan presence, pwm control, and automatic cpu
detection (x86) are also supported.

testing the hook for change-id

## key features

- runtime re-configurable from d-bus (entity-manager or the like)

- isolated: each sensor type is isolated into its own daemon, so a bug in one
  sensor is unlikely to affect another, and single sensor modifications are
  possible

- async single-threaded: uses sdbusplus/asio bindings

- multiple data inputs: hwmon, d-bus, direct driver access

## dbus interfaces

A typical dbus-sensors object support the following dbus interfaces:

```text
Path        /xyz/openbmc_project/sensors/<type>/<sensor_name>

Interfaces  xyz.openbmc_project.Sensor.Value
            xyz.openbmc_project.Sensor.Threshold.Critical
            xyz.openbmc_project.Sensor.Threshold.Warning
            xyz.openbmc_project.State.Decorator.Availability
            xyz.openbmc_project.State.Decorator.OperationalStatus
            xyz.openbmc_project.Association.Definitions

```

Sensor interfaces collection are described in
[phosphor-dbus-interfaces](https://github.com/openbmc/phosphor-dbus-interfaces/tree/master/yaml/xyz/openbmc_project/Sensor).

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

A simple sensor example can be found in
[entity-manager examples](https://github.com/openbmc/entity-manager/blob/master/docs/my_first_sensors.md).

## configuration

Sensor devices are described using Exposes records in configuration file. Name
and Type fields are required. Different sensor types have different fields.
Refer to entity manager
[schema](https://github.com/openbmc/entity-manager/blob/master/schemas/legacy.json)
for complete list.

## sensor documentation

- [ExternalSensor](https://github.com/openbmc/docs/blob/master/designs/external-sensor.md)
  virtual sensor

## Sensor Type Documentation

### ADC Sensors

ADC sensors are sensors based on an Analog to Digital Converter. They are read
via the Linux kernel Industrial I/O subsystem (IIO).

One of the more common use cases within OpenBMC is for reading these sensors
from the ADC on the Aspeed ASTXX cards.

To utilize ADC sensors feature within OpenBMC you must first define and enable
it within the kernel device tree.

When using a common OpenBMC device like the AST2600 you will find a "adc0" and
"adc1" section in the aspeed-g6.dtsi file. These are disabled by default so in
your system-specific dts you would enable and configure what you want with
something like this:

```text
iio-hwmon {
    compatible = "iio-hwmon";
    io-channels = <&adc0 0>;
    ...
}

&adc0 {
    status = "okay";
    ...
};

&adc1 {
    status = "okay";
    ...
};
```

**Note** that this is not meant to be an exhaustive list on the nuances of
configuring a device tree but really to point users in the general direction.

You will then create an entity-manager configuration file that is of type "ADC"
A very simple example would like look this:

```text
            "Index": 0,
            "Name": "P12V",
            "PowerState": "Always",
            "ScaleFactor": 1.0,
            "Type": "ADC"
```

When your system is booted, a "in0_input" file will be created within the hwmon
subsystem (/sys/class/hwmon/hwmonX). The adcsensor application will scan d-bus
for any ADC entity-manager objects, look up their "Index" value, and try to
match that with the hwmon inY_input files. When it finds a match it will create
a d-bus sensor under the xyz.openbmc_project.ADCSensor service. The sensor will
be periodically updated based on readings from the hwmon file.
