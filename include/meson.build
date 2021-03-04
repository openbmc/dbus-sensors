conf_data = configuration_data()
conf_data.set10('VALIDATION_UNSECURE_FEATURE', get_option('validate-unsecure-feature').enabled())
conf_data.set10('INSECURE_UNRESTRICTED_SENSOR_OVERRIDE', get_option('insecure-sensor-override').enabled())
conf_data.set_quoted('OBMC_HOST_INSTANCES', get_option('host-instances'))
configure_file(input: 'dbus-sensor_config.h.in',
               output: 'dbus-sensor_config.h',
               configuration: conf_data)
