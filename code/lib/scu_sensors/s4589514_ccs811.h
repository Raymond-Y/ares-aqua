/*
 * CCS811 (VOC) Sensor Driver
 * - Initialises ccs811 device
 * - reads co2 and eTVOC channels 
 * 
*/

#ifndef S4589514_CCS811_H
#define S4589514_CCS811_H

struct voc_data {
    int16_t co2Data;
    int16_t eTOVData;
};

const char* now_str(void);
int do_fetch(const struct device*, struct voc_data*);

void read_ccs811_data(const struct device* , struct voc_data*);

#endif