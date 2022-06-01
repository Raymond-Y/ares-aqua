/*
 * HTS211 (temperature and humidity) Sensor Driver
 * - Initialises HTS211 device
 * - reads temperature and humidity data 
 * 
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>

#include "s4589514_hts221.h"

bool init_hts221(const struct device* dev) {
	if (dev == NULL) {
		printf("Could not get HTS221 device\n");
		return false;
	}
	return true;
}

void process_hts221_sample(const struct device *dev, 
				struct temp_hum_pres* tempHumPres)
{
	struct sensor_value temp, hum;
	if (sensor_sample_fetch(dev) < 0) {
		printf("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printf("Cannot read HTS221 humidity channel\n");
		return;
	}

	/* display temperature */
	sprintf(tempHumPres->hum, "%.1f", sensor_value_to_double(&hum));

	// printf("TemperatureHTS:%.1f C\n", sensor_value_to_double(&temp));

	/* display humidity */
	// printf("Relative HumidityHTS:%.1f%%\n", sensor_value_to_double(&hum));
}

// void hts221_handler(const struct device *dev,
// 			   const struct sensor_trigger *trig)
// {
// 	process_hts221_sample(dev);
// }
