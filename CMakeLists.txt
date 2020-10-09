cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
set (BUILD_SHARED_LIBRARIES OFF)
include (ExternalProject)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} \
    -Werror \
    -Wall \
    -Wextra \
    -Wnon-virtual-dtor \
    -Wold-style-cast \
    -Wcast-align \
    -Wunused \
    -Woverloaded-virtual \
    -Wpedantic \
    -Wnull-dereference \
    -Wdouble-promotion \
    -Wformat=2 \
    -Wno-sign-compare \
    -Wno-reorder \
"
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
set (
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} \
    -Wmisleading-indentation \
    -Wduplicated-cond \
    -Wduplicated-branches \
    -Wlogical-op \
    -Wuseless-cast \
"
)
link_libraries("stdc++fs")
endif()

# todo: get rid of nos, add the below:
#  -Wshadow \
#  -Wconversion \

set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

option (YOCTO "Enable Building in Yocto" OFF)
option (HUNTER_ENABLED "Enable hunter package pulling" OFF)

option (DISABLE_ADC "Disable installing ADC sensor" OFF)
option (DISABLE_CPU "Disable installing CPU sensor" OFF)
option (DISABLE_EXIT_AIR "Disable installing Exit Air Temp sensor" OFF)
option (DISABLE_FAN "Disable installing fan sensor" OFF)
option (DISABLE_HWMON_TEMP "Disable installing hwmon temp sensor" OFF)
option (DISABLE_INTRUSION "Disable installing intrusion sensor" OFF)
option (DISABLE_IPMB "Disable installing IPMB sensor" OFF)
option (DISABLE_MCUTEMP "Disable installing MCU temperature sensor" OFF)
option (DISABLE_PSU "Disable installing PSU sensor" OFF)
option (DISABLE_NVME "Disable installing NVME sensor" ON)

include ("cmake/HunterGate.cmake")

huntergate (URL "https://github.com/ruslo/hunter/archive/v0.18.64.tar.gz" SHA1
            "baf9c8cc4f65306f0e442b5419967b4c4c04589a")

project (sensors CXX)

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

link_directories (${EXTERNAL_INSTALL_LOCATION}/lib)

include_directories (include)

add_library(sensor_utils STATIC src/Utils.cpp src/Thresholds.cpp)

add_executable (adcsensor src/ADCSensorMain.cpp src/ADCSensor.cpp)
target_link_libraries (adcsensor PUBLIC sensor_utils)
target_link_libraries (adcsensor PUBLIC gpiodcxx)
target_link_libraries (adcsensor PUBLIC systemd)
target_link_libraries (adcsensor PUBLIC sdbusplus)
target_link_libraries (adcsensor PUBLIC sensor_utils)

add_executable (cpusensor src/CPUSensorMain.cpp src/CPUSensor.cpp)
target_link_libraries (cpusensor PUBLIC sensor_utils)
target_link_libraries (cpusensor PUBLIC gpiodcxx)
target_link_libraries (cpusensor PUBLIC systemd)
target_link_libraries (cpusensor PUBLIC sdbusplus)
target_link_libraries (cpusensor PUBLIC sensor_utils)

#peci header has #ifdef issues????
target_compile_options(cpusensor PUBLIC -Wno-header-guard)

add_executable (exitairtempsensor src/ExitAirTempSensor.cpp)
target_link_libraries (exitairtempsensor PUBLIC sensor_utils)
target_link_libraries (exitairtempsensor PUBLIC systemd)
target_link_libraries (exitairtempsensor PUBLIC sdbusplus)
target_link_libraries (exitairtempsensor PUBLIC sensor_utils)

add_executable (fansensor src/FanMain.cpp src/TachSensor.cpp src/PwmSensor.cpp)
target_link_libraries (fansensor PUBLIC sensor_utils)
target_link_libraries (fansensor PUBLIC gpiodcxx)
target_link_libraries (fansensor PUBLIC systemd)
target_link_libraries (fansensor PUBLIC sdbusplus)
target_link_libraries (fansensor PUBLIC sensor_utils)

add_executable (hwmontempsensor src/HwmonTempMain.cpp src/HwmonTempSensor.cpp)
target_link_libraries (hwmontempsensor PUBLIC sensor_utils)
target_link_libraries (hwmontempsensor PUBLIC systemd)
target_link_libraries (hwmontempsensor PUBLIC sdbusplus)
target_link_libraries (hwmontempsensor PUBLIC sensor_utils)

add_executable (intrusionsensor src/IntrusionSensorMain.cpp
                src/ChassisIntrusionSensor.cpp)
target_link_libraries (intrusionsensor PUBLIC sensor_utils)
target_link_libraries (intrusionsensor PUBLIC i2c)
target_link_libraries (intrusionsensor PUBLIC gpiodcxx)
target_link_libraries (intrusionsensor PUBLIC systemd)
target_link_libraries (intrusionsensor PUBLIC sdbusplus)
target_link_libraries (intrusionsensor PUBLIC sensor_utils)

add_executable (ipmbsensor src/IpmbSensor.cpp)
target_link_libraries (ipmbsensor PUBLIC sensor_utils)
target_link_libraries (ipmbsensor PUBLIC systemd)
target_link_libraries (ipmbsensor PUBLIC sdbusplus)
target_link_libraries (ipmbsensor PUBLIC sensor_utils)

add_executable (mcutempsensor src/MCUTempSensor.cpp)
target_link_libraries (mcutempsensor PUBLIC sensor_utils)
target_link_libraries (mcutempsensor PUBLIC i2c)
target_link_libraries (mcutempsensor PUBLIC systemd)
target_link_libraries (mcutempsensor PUBLIC sdbusplus)
target_link_libraries (mcutempsensor PUBLIC sensor_utils)

add_executable (psusensor src/PSUSensorMain.cpp src/PSUSensor.cpp src/PwmSensor.cpp src/PSUEvent.cpp)
target_link_libraries (psusensor PUBLIC sensor_utils)
target_link_libraries (psusensor PUBLIC systemd)
target_link_libraries (psusensor PUBLIC sdbusplus)
target_link_libraries (psusensor PUBLIC sensor_utils)

if (NOT DISABLE_NVME)
    add_executable (nvmesensor src/NVMeSensorMain.cpp src/NVMeSensor.cpp)
    target_link_libraries (nvmesensor PUBLIC sensor_utils)
    target_link_libraries (nvmesensor PUBLIC liblibmctp.a)
    target_link_libraries (nvmesensor PUBLIC i2c)
    target_link_libraries (nvmesensor PUBLIC systemd)
    target_link_libraries (nvmesensor PUBLIC sdbusplus)
    target_link_libraries (nvmesensor PUBLIC sensor_utils)
endif()

set (SERVICE_FILE_SRC_DIR ${PROJECT_SOURCE_DIR}/service_files)
set (SERVICE_FILE_INSTALL_DIR /lib/systemd/system/)

if (NOT DISABLE_ADC)
    install (TARGETS adcsensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.adcsensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

if (NOT DISABLE_CPU)
    install (TARGETS cpusensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.cpusensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

if (NOT DISABLE_EXIT_AIR)
    install (TARGETS exitairtempsensor DESTINATION bin)
    install (
        FILES ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.exitairsensor.service
              DESTINATION ${SERVICE_FILE_INSTALL_DIR}
    )
endif ()

if (NOT DISABLE_FAN)
    install (TARGETS fansensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.fansensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

if (NOT DISABLE_HWMON_TEMP)
    install (TARGETS hwmontempsensor DESTINATION bin)
    install (
        FILES
            ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.hwmontempsensor.service
            DESTINATION ${SERVICE_FILE_INSTALL_DIR}
    )
endif ()

if (NOT DISABLE_INTRUSION)
    install (TARGETS intrusionsensor DESTINATION bin)
    install (
        FILES
            ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.intrusionsensor.service
            DESTINATION ${SERVICE_FILE_INSTALL_DIR}
    )
endif ()

if (NOT DISABLE_IPMB)
    install (TARGETS ipmbsensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.ipmbsensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

if (NOT DISABLE_MCUTEMP)
    install (TARGETS mcutempsensor DESTINATION bin)
    install (
        FILES ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.mcutempsensor.service
              DESTINATION ${SERVICE_FILE_INSTALL_DIR}
    )
endif ()

if (NOT DISABLE_PSU)
    install (TARGETS psusensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.psusensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

if (NOT DISABLE_NVME)
    install (TARGETS nvmesensor DESTINATION bin)
    install (FILES
                 ${SERVICE_FILE_SRC_DIR}/xyz.openbmc_project.nvmesensor.service
                 DESTINATION ${SERVICE_FILE_INSTALL_DIR})
endif ()

