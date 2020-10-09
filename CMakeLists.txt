cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
set (BUILD_SHARED_LIBRARIES OFF)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
include (ExternalProject)
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

set (EXTERNAL_PACKAGES Boost sdbusplus-project nlohmann-json)
set (SENSOR_LINK_LIBS -lsystemd sdbusplus)

find_package (Boost 1.73 REQUIRED)
if (BOOST_FOUND)
	include_directories (SYSTEM ${BOOST_SRC_DIR})
else()
    externalproject_add (
        Boost URL
        https://dl.bintray.com/boostorg/release/1.66.0/source/boost_1_66_0.tar.gz
        URL_MD5 d275cd85b00022313c171f602db59fc5 SOURCE_DIR
        "${CMAKE_BINARY_DIR}/boost-src" BINARY_DIR
        "${CMAKE_BINARY_DIR}/boost-build" CONFIGURE_COMMAND "" BUILD_COMMAND ""
        INSTALL_COMMAND ""
    )
    include_directories (SYSTEM ${CMAKE_BINARY_DIR}/boost-src)
    set (CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/boost-src ${CMAKE_PREFIX_PATH})
endif()

if (NOT YOCTO)
    set (DISABLE_NVME ON) # todo allow this to build out of tree
    option (ENABLE_TEST "Enable Google Test" OFF)

    # requires apt install autoconf-archive and autoconf
    externalproject_add (sdbusplus-project PREFIX
                         ${CMAKE_BINARY_DIR}/sdbusplus-project GIT_REPOSITORY
                         https://github.com/openbmc/sdbusplus.git SOURCE_DIR
                         ${CMAKE_BINARY_DIR}/sdbusplus-src BINARY_DIR
                         ${CMAKE_BINARY_DIR}/sdbusplus-build CONFIGURE_COMMAND
                         "" BUILD_COMMAND cd ${CMAKE_BINARY_DIR}/sdbusplus-src
                         && meson build -Ddefault_library=static && ninja -C
                         build libsdbusplus.a INSTALL_COMMAND ""
                         LOG_DOWNLOAD ON)
    include_directories (SYSTEM ${CMAKE_BINARY_DIR}/sdbusplus-src)
    link_directories (${CMAKE_BINARY_DIR}/sdbusplus-src/build)

    externalproject_add (nlohmann-json PREFIX
                         ${CMAKE_CURRENT_BINARY_DIR}/nlohmann-json
                         GIT_REPOSITORY https://github.com/nlohmann/json.git
                         SOURCE_DIR ${CMAKE_BINARY_DIR}/nlohmann-json-src
                         BINARY_DIR ${CMAKE_BINARY_DIR}/nlohmann-json-build
                         CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND
                         "" LOG_DOWNLOAD ON)
    include_directories (SYSTEM ${CMAKE_BINARY_DIR}/nlohmann-json-src/include)

    if (ENABLE_TEST)
        option (HUNTER_ENABLED "Enable hunter package pulling" ON)
        hunter_add_package (GTest)

        find_package (GTest CONFIG REQUIRED)

        enable_testing ()

        add_executable (runTachTests tests/test_TachSensor.cpp src/TachSensor.cpp src/PwmSensor.cpp)
        add_test (NAME test_fansensor COMMAND runTachTests)
        target_link_libraries (runTachTests GTest::main GTest::gtest pthread
                               ${DBUS_LIBRARIES} stdc++fs)
        add_dependencies (runTachTests nlohmann-json)

        add_executable (runHwmonTempTests tests/test_HwmonTempSensor.cpp
                        src/HwmonTempSensor.cpp)
        add_test (NAME test_hwmontempsensor COMMAND runHwmonTempTests)
        target_link_libraries (runHwmonTempTests GTest::main GTest::gtest
                               pthread ${DBUS_LIBRARIES} stdc++fs)
        add_dependencies (runHwmonTempTests nlohmann-json)
    endif ()

endif ()

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
target_link_libraries (adcsensor PUBLIC ${SENSOR_LINK_LIBS})
target_link_libraries (adcsensor PUBLIC gpiodcxx)

add_executable (cpusensor src/CPUSensorMain.cpp src/CPUSensor.cpp)
target_link_libraries (cpusensor PUBLIC sensor_utils)
target_link_libraries (cpusensor PUBLIC ${SENSOR_LINK_LIBS})
target_link_libraries (cpusensor PUBLIC gpiodcxx)

#peci header has #ifdef issues????
target_compile_options(cpusensor PUBLIC -Wno-header-guard)

add_executable (exitairtempsensor src/ExitAirTempSensor.cpp)
target_link_libraries (exitairtempsensor PUBLIC sensor_utils)
target_link_libraries (exitairtempsensor PUBLIC ${SENSOR_LINK_LIBS})

add_executable (fansensor src/FanMain.cpp src/TachSensor.cpp src/PwmSensor.cpp)
target_link_libraries (fansensor PUBLIC sensor_utils)
target_link_libraries (fansensor PUBLIC ${SENSOR_LINK_LIBS})
target_link_libraries (fansensor PUBLIC gpiodcxx)

add_executable (hwmontempsensor src/HwmonTempMain.cpp src/HwmonTempSensor.cpp)
target_link_libraries (hwmontempsensor PUBLIC sensor_utils)
target_link_libraries (hwmontempsensor PUBLIC ${SENSOR_LINK_LIBS})

add_executable (intrusionsensor src/IntrusionSensorMain.cpp
                src/ChassisIntrusionSensor.cpp)
target_link_libraries (intrusionsensor PUBLIC sensor_utils)
target_link_libraries (intrusionsensor PUBLIC ${SENSOR_LINK_LIBS})
target_link_libraries (intrusionsensor PUBLIC i2c)
target_link_libraries (intrusionsensor PUBLIC gpiodcxx)

add_executable (ipmbsensor src/IpmbSensor.cpp)
target_link_libraries (ipmbsensor PUBLIC sensor_utils)
target_link_libraries (ipmbsensor PUBLIC ${SENSOR_LINK_LIBS})

add_executable (mcutempsensor src/MCUTempSensor.cpp)
target_link_libraries (mcutempsensor PUBLIC sensor_utils)
target_link_libraries (mcutempsensor PUBLIC ${SENSOR_LINK_LIBS})
target_link_libraries (mcutempsensor PUBLIC i2c)

add_executable (psusensor src/PSUSensorMain.cpp src/PSUSensor.cpp src/PwmSensor.cpp src/PSUEvent.cpp)
target_link_libraries (psusensor PUBLIC sensor_utils)
target_link_libraries (psusensor PUBLIC ${SENSOR_LINK_LIBS})

if (NOT DISABLE_NVME)
    add_executable (nvmesensor src/NVMeSensorMain.cpp src/NVMeSensor.cpp)
    target_link_libraries (nvmesensor PUBLIC sensor_utils)
    target_link_libraries (nvmesensor PUBLIC liblibmctp.a)
    target_link_libraries (nvmesensor PUBLIC i2c)
    target_link_libraries (nvmesensor PUBLIC ${SENSOR_LINK_LIBS})
endif()

if (NOT YOCTO)
    add_dependencies (adcsensor ${EXTERNAL_PACKAGES})
    add_dependencies (cpusensor ${EXTERNAL_PACKAGES})
    add_dependencies (exitairtempsensor ${EXTERNAL_PACKAGES})
    add_dependencies (fansensor ${EXTERNAL_PACKAGES})
    add_dependencies (hwmontempsensor ${EXTERNAL_PACKAGES})
    add_dependencies (intrusionsensor ${EXTERNAL_PACKAGES})
    add_dependencies (ipmbsensor ${EXTERNAL_PACKAGES})
    add_dependencies (mcutempsensor ${EXTERNAL_PACKAGES})
    add_dependencies (psusensor ${EXTERNAL_PACKAGES})
endif ()

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
