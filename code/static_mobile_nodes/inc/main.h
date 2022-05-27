#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <data/json.h>
#include <drivers/sensor.h>

#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <toolchain.h>
#include <stdarg.h>
#include <inttypes.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <init.h>
#include <drivers/gpio.h>
// #include <drivers/pwm.h>

#include <sys/printk.h>
#include <sys/byteorder.h>
#include <sys/util.h>

#define SW1_NODE	DT_ALIAS(sw1)
#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
    #define THINGY_52           false
    #define PARTICLE_ARGON      true
#else
    #define THINGY_52           true
    #define PARTICLE_ARGON      false
#endif

// #include "ble_mobile_static.h"
// #include "hal_imu.h"


#define STACK_SIZE_MAIN_THREAD      		2048

#define THREAD_PRIORITY_MAIN		        8

void thread_main_thread(void);
