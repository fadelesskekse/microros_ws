# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(COMPATIBLE_BOARDS disco_l475_iot1 olimex_stm32_e407 native_posix nucleo_h743zi nucleo_f746zg blackpill_f411ce blackpill_f401ce)
if(NOT ${BOARD} IN_LIST COMPATIBLE_BOARDS)
    message(FATAL_ERROR "App $ENV{UROS_APP} not compatible with board ${BOARD}")
endif()

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(microrosapp C CXX)

############For compiling the correct controller_configs############
set(
    CONTROL_MODEL
    ""
    CACHE STRING
    "Model whose controller configuration will be compiled"
)

set(
    CONTROL_TYPE
    ""
    CACHE STRING
    "Controller type whose configuration will be compiled"
)

set(
    CONTROLLER_CONFIG_ROOT
    ""
    CACHE PATH
    "Root directory containing generated controller configurations"
)

if(CONTROL_MODEL STREQUAL "")
    message(FATAL_ERROR
        "CONTROL_MODEL was not provided. "
        "Pass -DCONTROL_MODEL=<model> to build_firmware.sh."
    )
endif()

if(CONTROL_TYPE STREQUAL "")
    message(FATAL_ERROR
        "CONTROL_TYPE was not provided. "
        "Pass -DCONTROL_TYPE=<controller> to build_firmware.sh."
    )
endif()

if(CONTROLLER_CONFIG_ROOT STREQUAL "")
    message(FATAL_ERROR
        "CONTROLLER_CONFIG_ROOT was not provided."
    )
endif()

set(
    SELECTED_CONTROLLER_DIRECTORY
    "${CONTROLLER_CONFIG_ROOT}/${CONTROL_MODEL}"
)

set(SELECTED_CONTROLLER_SOURCES)
set(ENABLED_CONTROLLER_IMPLEMENTATIONS)
set(SELECTED_CONFIG_INCLUDES "")
set(SELECTED_REGISTRATION_CALLS "")

foreach(controller_name IN LISTS CONTROL_TYPE)
    # Determine implementation type from instance name.
    if(controller_name MATCHES "^lqr")
        set(controller_implementation "lqr")
    elseif(controller_name MATCHES "^pid")
        set(controller_implementation "pid")
    elseif(controller_name MATCHES "^test")
        set(controller_implementation "test")
    else()
        message(FATAL_ERROR
            "Cannot determine controller type from: "
            "${controller_name}"
        )
    endif()

    set(
        controller_source
        "${SELECTED_CONTROLLER_DIRECTORY}/${controller_name}_config.c"
    )

    set(
        controller_header
        "${SELECTED_CONTROLLER_DIRECTORY}/${controller_name}_config.h"
    )

    if(NOT EXISTS "${controller_source}")
        message(FATAL_ERROR
            "Missing controller source: ${controller_source}"
        )
    endif()

    if(NOT EXISTS "${controller_header}")
        message(FATAL_ERROR
            "Missing controller header: ${controller_header}"
        )
    endif()

    list(APPEND
        SELECTED_CONTROLLER_SOURCES
        "${controller_source}"
    )

    list(APPEND
        ENABLED_CONTROLLER_IMPLEMENTATIONS
        "${controller_implementation}"
    )

    string(APPEND
        SELECTED_CONFIG_INCLUDES
        "#include \"${controller_name}_config.h\"\n"
    )

    if(controller_implementation STREQUAL "lqr") #do we need one of these for each type of controller?
        string(APPEND
            SELECTED_REGISTRATION_CALLS
            "    status = controller_register_lqr(\n"
            "        \"${controller_name}\",\n"
            "        ${CONTROL_MODEL}_${controller_name}_gain_matrix,\n"
            "        ${CONTROL_MODEL}_${controller_name}_gain_rows,\n"
            "        ${CONTROL_MODEL}_${controller_name}_gain_columns\n"
            "    );\n"
            "    if (status != CONTROLLER_STATUS_OK) {\n"
            "        return status;\n"
            "    }\n\n"
        )
    endif()
endforeach()

list(REMOVE_DUPLICATES
    ENABLED_CONTROLLER_IMPLEMENTATIONS
)

foreach(implementation IN LISTS ENABLED_CONTROLLER_IMPLEMENTATIONS)
    string(
        TOUPPER
        "${implementation}"
        implementation_upper
    )

    target_compile_definitions(
        app
        PRIVATE
        "CONTROLLER_${implementation_upper}_ENABLED=1"
    )
endforeach()

set(
    GENERATED_CONTROLLER_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

file(MAKE_DIRECTORY
    "${GENERATED_CONTROLLER_DIRECTORY}"
)

set(
    GENERATED_SELECTED_CONTROLLERS_SOURCE
    "${GENERATED_CONTROLLER_DIRECTORY}/selected_controllers.c"
)

file(WRITE
    "${GENERATED_SELECTED_CONTROLLERS_SOURCE}"
"#include \"controller/controllers.h\"

${SELECTED_CONFIG_INCLUDES}

controller_status_t selected_controllers_register(void)
{
    controller_status_t status;

${SELECTED_REGISTRATION_CALLS}
    return CONTROLLER_STATUS_OK;
}
"
)
set(
    JOINT_FILTER_CONFIG_ROOT
    ""
    CACHE PATH
    "Root directory containing generated joint-filter configurations"
)

if(JOINT_FILTER_CONFIG_ROOT STREQUAL "")
    message(FATAL_ERROR
        "JOINT_FILTER_CONFIG_ROOT was not provided."
    )
endif()

set(
    SELECTED_JOINT_FILTER_DIRECTORY
    "${JOINT_FILTER_CONFIG_ROOT}/${CONTROL_MODEL}"
)

set(
    SELECTED_JOINT_FILTER_SOURCE
    "${SELECTED_JOINT_FILTER_DIRECTORY}/joint_filter_config.c"
)

set(
    SELECTED_JOINT_FILTER_HEADER
    "${SELECTED_JOINT_FILTER_DIRECTORY}/joint_filter_config.h"
)

if(NOT EXISTS "${SELECTED_JOINT_FILTER_SOURCE}")
    message(FATAL_ERROR
        "Joint-filter source does not exist: "
        "${SELECTED_JOINT_FILTER_SOURCE}"
    )
endif()

if(NOT EXISTS "${SELECTED_JOINT_FILTER_HEADER}")
    message(FATAL_ERROR
        "Joint-filter header does not exist: "
        "${SELECTED_JOINT_FILTER_HEADER}"
    )
endif()

target_sources(
    app
    PRIVATE
    ${SELECTED_CONTROLLER_SOURCES}
    "${GENERATED_SELECTED_CONTROLLERS_SOURCE}"
    "${SELECTED_JOINT_FILTER_SOURCE}"
)

target_include_directories(
    app
    PRIVATE
    "${SELECTED_CONTROLLER_DIRECTORY}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${SELECTED_JOINT_FILTER_DIRECTORY}"

)


set(microros_src_dir   ${MICRO_ROS_FIRMWARE_DIR}/zephyr_apps/microros_extensions)
set(microros_build_dir ${MICRO_ROS_FIRMWARE_DIR}/build/microros_extensions)

if(${MICRO_ROS_TRANSPORT} STREQUAL "serial")
    add_compile_definitions(MICRO_ROS_TRANSPORT_SERIAL)
elseif(${MICRO_ROS_TRANSPORT} STREQUAL "udp")
    add_compile_definitions(MICRO_ROS_TRANSPORT_UDP)
elseif(${MICRO_ROS_TRANSPORT} STREQUAL "serial-usb")
    add_compile_definitions(MICRO_ROS_TRANSPORT_SERIALUSB)
endif()

FILE(GLOB app_sources src/*.c ${microros_src_dir}/microros_transports.c)
include_directories(${microros_src_dir})

target_sources(app PRIVATE ${app_sources})

zephyr_get_include_directories_for_lang_as_string(       C includes)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C definitions)
zephyr_get_compile_options_for_lang_as_string(           C options)

zephyr_get_include_directories_for_lang_as_string(       CXX includes_cxx)
zephyr_get_system_include_directories_for_lang_as_string(CXX system_includes_cxx)
zephyr_get_compile_definitions_for_lang_as_string(       CXX definitions_cxx)
zephyr_get_compile_options_for_lang_as_string(           CXX options_cxx)

set(external_project_cflags
    "${includes} ${definitions} ${options} ${system_includes}"
    )

set(external_project_cxxflags
    "${includes_cxx} ${definitions_cxx} ${options_cxx} ${system_includes_cxx}"
    )

include(ExternalProject)

set(microros_src_dir   ${MICRO_ROS_FIRMWARE_DIR}/zephyr_apps/microros_extensions)
set(microros_build_dir ${MICRO_ROS_FIRMWARE_DIR}/build/microros_extensions)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

ExternalProject_Add(
    microroslib_project                 # Name for custom target
    PREFIX     ${microros_build_dir}    # Root dir for entire project
    SOURCE_DIR ${microros_src_dir}
    BINARY_DIR ${microros_src_dir}      # This particular build system is invoked from the root
    CONFIGURE_COMMAND ""                # Skip configuring the project, e.g. with autoconf
    BUILD_COMMAND
        ${submake}
            PREFIX=${microros_src_dir}
            CC=${CMAKE_C_COMPILER}
            AR=${CMAKE_AR}
            CFLAGS=${external_project_cflags}
            CXX=${CMAKE_CXX_COMPILER}
            CXXFLAGS=${external_project_cxxflags}
            APP_META=$ENV{UROS_APP_FOLDER}/app-colcon.meta

    INSTALL_COMMAND ""      # This particular build system has no install command
    BUILD_BYPRODUCTS ${MICROROS_LIB_DIR}/libmicroros.a
    )

file(MAKE_DIRECTORY ${MICRO_ROS_FIRMWARE_DIR}/mcu_ws/install)
file(MAKE_DIRECTORY ${MICRO_ROS_FIRMWARE_DIR}/mcu_ws/install/include)

set(MICROROS_LIB_DIR     ${MICRO_ROS_FIRMWARE_DIR}/mcu_ws/install)
set(MICROROS_INCLUDE_DIR ${MICRO_ROS_FIRMWARE_DIR}/mcu_ws/install/include)

add_library(microroslib STATIC IMPORTED GLOBAL)
add_dependencies(
    microroslib
    microroslib_project
    )

add_dependencies(microroslib_project offsets_h)

set_target_properties(microroslib PROPERTIES IMPORTED_LOCATION             ${MICROROS_LIB_DIR}/libmicroros.a)
set_target_properties(microroslib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MICROROS_INCLUDE_DIR})

execute_process(
    COMMAND
        make get_package_names
    WORKING_DIRECTORY
        ${microros_src_dir}
    OUTPUT_VARIABLE
        INCLUDE_ROS2_PACKAGES
    )

foreach(pkg ${INCLUDE_ROS2_PACKAGES})
    file(MAKE_DIRECTORY ${MICROROS_INCLUDE_DIR}/${pkg})
    target_include_directories(microroslib INTERFACE ${MICROROS_INCLUDE_DIR}/${pkg})
endforeach()

target_link_libraries(app PUBLIC microroslib)
