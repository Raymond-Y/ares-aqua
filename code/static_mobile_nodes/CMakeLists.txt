cmake_minimum_required(VERSION 3.20.0)
set(BOARD_FLASH_RUNNER jlink)

option(STATIC_NODEA "SELECT STATIC NODE" OFF)
option(STATIC_NODEB "SELECT STATIC NODE" OFF)
option(STATIC_NODEC "SELECT STATIC NODE" OFF)
option(STATIC_NODED "SELECT STATIC NODE" OFF)
IF(${STATIC_NODEA})
set(CONF_FILE conf_files/prj.conf conf_files/rtt_console.conf conf_files/bt.conf conf_files/static_node_A.conf)
ELSEIF(${STATIC_NODEB})
set(CONF_FILE conf_files/prj.conf conf_files/rtt_console.conf conf_files/bt.conf conf_files/static_node_B.conf)
ELSEIF(${STATIC_NODEC})
set(CONF_FILE conf_files/prj.conf conf_files/rtt_console.conf conf_files/bt.conf conf_files/static_node_C.conf)
ELSEIF(${STATIC_NODED})
set(CONF_FILE conf_files/prj.conf conf_files/rtt_console.conf conf_files/bt.conf conf_files/static_node_D.conf)
ELSE()
set(CONF_FILE conf_files/prj.conf conf_files/rtt_console.conf conf_files/bt.conf conf_files/mobile_node.conf)
ENDIF()


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(project/static_mobile_nodes)

include_directories(    
                        # ../../oslib/rgb_led_driver/
                        # ../../oslib/scu_drivers/scu_ble/
                        # ../../oslib/hci_driver/
                        # ../../oslib/pwm_speaker_driver/
                        # ../../oslib/pb_driver/
                        inc/
                        ../../oslib/ble_mesh/
                        ../../oslib/mpu_i2c/
                        ../../oslib/scu_drivers/scu_sensors/
                        )

target_sources(app PRIVATE 
                        src/main.c
                        # ../../oslib/rgb_led_driver/s4589514_rgb_led.c
                        # ../../oslib/scu_drivers/scu_ble/s4589514_scu_ble.c
                        # ../../oslib/hci_driver/hci.c
                        # ../../oslib/pwm_speaker_driver/s4589514_speaker_pwm.c
                        # ../../oslib/pb_driver/s4589514_pb.c
                        ../../oslib/scu_drivers/scu_ble/ble_mobile_static.c
                        ../../oslib/mpu_i2c/inv_mpu.c
                        ../../oslib/mpu_i2c/inv_mpu_dmp_motion_driver.c
                        ../../oslib/mpu_i2c/zephyr_i2c.c
                        ../../oslib/scu_drivers/scu_sensors/hal_imu.c
                        )