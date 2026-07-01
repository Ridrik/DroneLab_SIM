if(WIN32)
    cmake_minimum_required(VERSION 4.1.1)
    cmake_policy(SET CMP0155 OLD)
else()
    cmake_minimum_required(VERSION 4.2)
endif()

set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(DroneLab_SIM 
    VERSION 0.1.0
    DESCRIPTION "DroneLab open-source Simulation modules"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dronelab/simulation/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/src/dronelab/simulation/version.h"
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(SIMULATION_SOURCES
    src/dronelab/simulation/version.cpp
    src/dronelab/simulation/drone_simulator.cpp
    src/dronelab/simulation/dynamics/dynamic_simulator.cpp
    src/dronelab/simulation/dynamics/actuator.cpp
    src/dronelab/simulation/dynamics/battery_system.cpp
    src/dronelab/simulation/sensors/estimator.cpp
    src/dronelab/simulation/sensors/ekf.cpp
    src/dronelab/simulation/sensors/processor.cpp
    src/dronelab/simulation/sensors/imu.cpp
    src/dronelab/simulation/sensors/gps_sensor.cpp
    src/dronelab/simulation/sensors/magnetometer.cpp
    src/dronelab/simulation/sensors/barometer.cpp
    src/dronelab/simulation/control/pid_controller.cpp
    src/dronelab/simulation/control/lqr_controller.cpp
    src/dronelab/simulation/control/mixer.cpp
    src/dronelab/simulation/navigation/waypoint_behaviour.cpp
    src/dronelab/simulation/navigation/loiter_behaviour.cpp
    src/dronelab/simulation/navigation/rtl_behaviour.cpp
    src/dronelab/simulation/navigation/trajectory_generator.cpp
    src/dronelab/simulation/system/system_monitor.cpp
    src/dronelab/simulation/management/flight_manager.cpp
    src/dronelab/simulation/environment/wind_field.cpp
)

set(DRONELAB_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dist")

list(APPEND CMAKE_PREFIX_PATH
    "${SDK_ROOT}"
    "${SDK_ROOT}/lib/cmake"
)

list(APPEND CMAKE_PREFIX_PATH "${DRONELAB_SDK_DIR}")
if (NOT TARGET DroneLab::Core)
    find_package(DroneLab REQUIRED CONFIG)
endif()

find_package(spdlog REQUIRED)
find_package(Eigen3 5.0 REQUIRED CONFIG)

add_library(DroneLab_SIM_Headers INTERFACE)

target_include_directories(DroneLab_SIM_Headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
add_library(DroneLab_SIM SHARED ${SIMULATION_SOURCES})
add_library(DroneLab::Sim ALIAS DroneLab_SIM)

set_target_properties(DroneLab_SIM PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

target_link_libraries(DroneLab_SIM PRIVATE DroneLab_SIM_Headers DroneLab::Core DroneLab::Math DroneLab::task DroneLab::fault)
if(WIN32)
    set(DRONELAB_EXE_DIR "${DRONELAB_SDK_DIR}")
    set(DRONELAB_EXE "${DRONELAB_EXE_DIR}/DroneLab.exe")
else()
    set(DRONELAB_EXE_DIR "${DRONELAB_SDK_DIR}/bin")
    set(DRONELAB_EXE "${DRONELAB_EXE_DIR}/DroneLab")
endif()

set_target_properties(DroneLab_SIM PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${DRONELAB_SDK_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

add_custom_target(run_dronelab
    COMMAND
        "${DRONELAB_EXE}"
        --sim "$<TARGET_FILE:DroneLab_SIM>"
        --root "${CMAKE_SOURCE_DIR}/dist"
        --data "${CMAKE_SOURCE_DIR}"

    DEPENDS DroneLab_SIM
    WORKING_DIRECTORY "${DRONELAB_SDK_DIR}"
    USES_TERMINAL
    COMMENT "Running DroneLab with $<TARGET_FILE:DroneLab_SIM>"
)
