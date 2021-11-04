# dbus-sensors

dbus-sensors is a collection of sensor applications that provide the
xyz.openbmc_project.Sensor collection of interfaces. They read sensor values
from hwmon, d-bus, or direct driver access to provide readings. Some advance
non-sensor features such as fan presence, pwm control, and automatic cpu
detection (x86) are also supported.

## key features

-   runtime re-configurable from d-bus (entity-manager or the like)

-   isolated: each sensor type is isolated into its own daemon, so a bug in one
    sensor is unlikely to affect another, and single sensor modifications are
    possible

-   async single-threaded: uses sdbusplus/asio bindings

-   multiple data inputs: hwmon, d-bus, direct driver access

## sensor documentation

-   [ExternalSensor](https://github.com/openbmc/docs/blob/master/designs/external-sensor.md)
    virtual sensor

## Extra features
### Sensor mutability

-   Most of sensors are initially considered to be read only. But with this
    added features, sensors can also become writable based on mutable
    configuration provided by user through entity manager JSON config.

-   Following is the config example to configure a particular sensor to be
    writable or not by setting key "Mutable" to true/false.

        {
            "BindConnector": "1U System Fan connector 1",
            "Index": 0,
            "Mutable": true,
            "Name": "Fan 1a",
            "Thresholds": [
                {
                    "Direction": "less than",
                    "Name": "lower critical",
                    "Severity": 1,
                    "Value": 1080
                },
                {
                    "Direction": "less than",
                    "Name": "lower non critical",
                    "Severity": 0,
                    "Value": 1260
                }
            ],
            "Type": "AspeedFan"
        }

-   Initially, only PwmSensors supports this feature by accepting "Mutable"
    parameter from entity manager configuration (JSON file). This features
    will be added for rest other sensors in future if required.

-   ExternalSensor always sets Mutable to true which makes them as writable
    (Read/write) sensor by default, given its purpose of accepting sensor writes
    from an external source.

-   All other sensors are by default stes Mutable to false currently which makes
    them read only sensors until we add this feature for those particular daemon
    to make it configurable through config file to make it writable.

-   This feature will create a ValueMutability dbus interface and it's details
    are defined in openbmc/phosphor-dbus-interfaces. It has a property "Mutable"
    which will be defined as true/false as per "Mutability parameter. This
    interface will be read by IPMI host daemon to service ipmi request of
    "set sensor reading". An example dbus interface is shown below.

        NAME                                        TYPE      SIGNATURE RESULT/VALUE                             FLAGS
        ...
        xyz.openbmc_project.Sensor.Value            interface -         -                                        -
        .MaxValue                                   property  x         100                                      emits-change
        .MinValue                                   property  x         0                                        emits-change
        .Unit                                       property  s         "xyz.openbmc_project.Sensor.Value.Uni... emits-change
        .Value                                      property  d         42.7451                                  emits-change writable
        xyz.openbmc_project.Sensor.ValueMutability  interface -         -                                        -
        .Mutable                                    property  b         true                                     emits-change
        ...
