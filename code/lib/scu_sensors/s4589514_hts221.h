/*
 * HTS211 (temperature and humidity) Sensor Driver
 * - Initialises HTS211 device
 * - reads temperature and humidity data 
 * 
*/

#ifndef S4589514_HTS221_H
#define S4589514_HTS221_H

#include "s4589514_lps22hb.h"

void process_hts221_sample(const struct device *dev, 
				struct temp_hum_pres* tempHumPres);

// void hts221_handler(const struct device *dev,
// 			   const struct sensor_trigger *trig);

bool init_hts221(const struct device *dev);

#endif