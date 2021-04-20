conf_data = configuration_data()
conf_data.set('VALIDATION_UNSECURE_FEATURE', get_option('validate-unsecure-feature'))
conf_data.set('INSECURE_UNRESTRICTED_SENSOR_OVERRIDE', get_option('insecure-sensor-override'))
configure_file(input: 'dbus-sensor_config.h.in',
               output: 'dbus-sensor_config.h',
               configuration: conf_data)