/*
 * LIS2DH (Accelerometer) Sensor Driver
 * - Initialises LIS2DH device
 * - reads and formats accelerometer data 
 * 
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>

#include <toolchain.h>
#include <stddef.h>
#include <stdarg.h>
#include <inttypes.h>

#include "s4589514_lis2dh.h"

void process_lis2dh_sample(const struct device* sensor, 
            struct accel_data* accelData) {
    static unsigned int count;
	struct sensor_value accel[3];
	// struct sensor_value temperature;
	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		// printf("x %f , y %f , z %f\n",
		//        sensor_value_to_double(&accel[0]),
		//        sensor_value_to_double(&accel[1]),
		//        sensor_value_to_double(&accel[2]));
               
        sprintf(accelData->accelX, "%.2f", 
                sensor_value_to_double(&accel[0]));
        sprintf(accelData->accelY, "%.2f", 
                sensor_value_to_double(&accel[1]));
        sprintf(accelData->accelZ, "%.2f", 
                sensor_value_to_double(&accel[2]));
	}
}

void read_lis2dh_data(const struct device* sensor, 
            struct accel_data* accelData) {

    if (sensor == NULL) {
        printf("No device found\n");
        return;
    }
    if (!device_is_ready(sensor)) {
        printf("Device %s is not ready\n", sensor->name);
        return;
    }
    process_lis2dh_sample(sensor, accelData);
}


