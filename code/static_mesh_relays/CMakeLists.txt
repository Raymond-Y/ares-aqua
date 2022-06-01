set(QEMU_EXTRA_FLAGS -s)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(code/static_mesh_relays)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)

