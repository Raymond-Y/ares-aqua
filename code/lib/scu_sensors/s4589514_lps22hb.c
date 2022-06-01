/*
 * LPS22HB (temp and pressure) Sensor Driver
 * - Initialises LPS22HB device
 * - reads temperature and pressure data
 * 
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "s4589514_lps22hb.h"

void process_lps22hb_sample(const struct device *dev, 
				struct temp_hum_pres* tempHumPres)
{
	struct sensor_value pressure, temp;

	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
		printf("Cannot read LPS22HB pressure channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read LPS22HB temperature channel\n");
		return;
	}
	
    if (dev == NULL) {
		printf("Could not get LPS22HB device\n");
		return;
	}

	/* display pressure */
	sprintf(tempHumPres->temp, "%.1f", sensor_value_to_double(&temp));
	sprintf(tempHumPres->pressure, "%.1f", sensor_value_to_double(&pressure));

	// printf("Pressure:%.1f kPa\n", sensor_value_to_double(&pressure));

	/* display temperature */
	// printf("Temperature:%.1f C\n", sensor_value_to_double(&temp));
}

