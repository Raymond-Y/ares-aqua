/*
 * LPS22HB (temp and pressure) Sensor Driver
 * - Initialises LPS22HB device
 * - reads temperature and pressure data
 * 
*/

#ifndef S4589514_LPS22HB_H
#define S4589514_LPS22HB_H

struct temp_hum_pres {
    char temp[15];
    char hum[15];
    char pressure[15];
};


void process_lps22hb_sample(const struct device* lps22hb, 
                struct temp_hum_pres* tempHumPres);


#endif