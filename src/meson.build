src_inc = include_directories('.')

subdir('config')
subdir('asio')
subdir('async_task')
subdir('utils')

src_inc = include_directories('.')

peci_incdirs = []
if not meson.get_compiler('cpp').has_header('linux/peci-ioctl.h')
    peci_incdirs = ['../include']
endif

if get_option('intel-cpu').allowed()
    peci_dep = dependency('libpeci', required: true)
    subdir('intel-cpu')
endif

if get_option('smbpbi').allowed()
    subdir('smbpbi')
endif

if get_option('adc').allowed()
    subdir('adc')
endif

if get_option('exit-air').allowed()
    subdir('exit-air')
endif

if get_option('fan').allowed()
    subdir('fan')
endif

if get_option('hwmon-temp').allowed()
    subdir('hwmon-temp')
endif

if get_option('intrusion').allowed()
    subdir('intrusion')
endif

if get_option('ipmb').allowed()
    subdir('ipmb')
endif

if get_option('mctp').allowed()
    subdir('mctp')
endif

if get_option('mcu').allowed()
    subdir('mcu')
endif

if get_option('nvme').allowed()
    subdir('nvme')
endif

if get_option('psu').allowed()
    subdir('psu')
endif

if get_option('nvidia-gpu').allowed()
    subdir('nvidia-gpu')
endif

if get_option('external').allowed()
    subdir('external')
endif

if get_option('tests').allowed()
    subdir('tests')
endif

if get_option('leakdetector').allowed()
    subdir('leakdetector')
endif

if get_option('cable-monitor').allowed()
    subdir('cable-monitor')
endif
