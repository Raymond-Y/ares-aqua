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

include_directories(    
                        ../lib/pb_driver/
                        )

target_sources(app PRIVATE 
                        src/main.c
                        # ../../oslib/pwm_speaker_driver/s4589514_speaker_pwm.c
                        ../lib/pb_driver/s4589514_pb.c
                        # ../../oslib/ultrasonic_driver/s4589514_ultra.c
                        # ../../oslib/scu_drivers/scu_ble/ble_mobile_static.c
                        # ../../oslib/mpu_i2c/inv_mpu.c
                        # ../../oslib/mpu_i2c/inv_mpu_dmp_motion_driver.c
                        # ../../oslib/mpu_i2c/zephyr_i2c.c
                        # ../../oslib/scu_drivers/scu_sensors/hal_imu.c
                        )

# FILE(GLOB app_sources src/*.c)
# target_sources(app PRIVATE ${app_sources})



cmake_minimum_required(VERSION 3.20.0)

set(BOARD_FLASH_RUNNER jlink)