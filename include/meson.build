conf_data = configuration_data()
conf_data.set10('VALIDATION_UNSECURE_FEATURE', get_option('validate-unsecure-feature').enabled())
conf_data.set10('INSECURE_UNRESTRICTED_SENSOR_OVERRIDE', get_option('insecure-sensor-override').enabled())
configure_file(input: 'dbus-sensor_config.h.in',
               output: 'dbus-sensor_config.h',
               configuration: conf_data)

tach_conf_data = configuration_data()
tach_conf_data.set('PRES_NOUSE_EVENT', get_option('pres-nouse-interrupt').enabled())
configure_file(output: 'tach_config.h',
               configuration: tach_conf_data)