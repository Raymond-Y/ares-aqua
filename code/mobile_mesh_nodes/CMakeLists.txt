set(QEMU_EXTRA_FLAGS -s)

option(Mobile_A "select static node" OFF)
option(Mobile_B "select static node" OFF)
IF(${Mobile_A})
    set(CONF_FILE conf_files/mobile_node_A.conf prj.conf)
ELSEIF(${Mobile_B})
    set(CONF_FILE conf_files/mobile_node_B.conf prj.conf)
ENDIF()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(code/mobile_mesh_nodes)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)