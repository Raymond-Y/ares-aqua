set(QEMU_EXTRA_FLAGS -s)

set(DTC_OVERLAY_FILE thingy52_overlay.dtsi)

option(Mobile_A "select static node" OFF)
option(Mobile_B "select static node" OFF)
IF(${Mobile_A})
    set(CONF_FILE conf_files/mobile_node_A.conf prj.conf)
ELSEIF(${Mobile_B})
    set(CONF_FILE conf_files/mobile_node_B.conf prj.conf)
ENDIF()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(code/mobile_mesh_nodes)

include_directories(    
                        ../lib/pb_driver/
                        ../lib/pwm_speaker_driver/
                        )

target_sources(app PRIVATE 
                        src/main.c
                        ../lib/pwm_speaker_driver/s4589514_speaker_pwm.c
                        ../lib/pb_driver/s4589514_pb.c
                        )

# FILE(GLOB app_sources src/*.c)
# target_sources(app PRIVATE ${app_sources})



cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)