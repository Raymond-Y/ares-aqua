set(QEMU_EXTRA_FLAGS -s)

set(DTC_OVERLAY_FILE dtc_console.overlay)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(code/base_mesh_node)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)
