set(QEMU_EXTRA_FLAGS -s)

option(nodeW "select static node" OFF)
option(X "select static node" OFF)
option(Y "SELECT STATIC NODE" OFF)
option(Z "SELECT STATIC NODE" OFF)
IF(${nodeW})
    set(CONF_FILE conf_files/static_node_W.conf prj.conf)
ELSEIF(${X})
    set(CONF_FILE conf_files/static_node_X.conf prj.conf)
ELSEIF(${Y})
    set(CONF_FILE conf_files/static_node_Y.conf prj.conf)
ELSEIF(${Z})
    set(CONF_FILE conf_files/static_node_Z.conf prj.conf)
ENDIF()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(code/static_mesh_relays)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)

