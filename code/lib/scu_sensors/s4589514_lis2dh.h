/*
 * LIS2DH (Accelerometer) Sensor Driver
 * - Initialises LIS2DH device
 * - reads and formats accelerometer data 
 * 
*/

#ifndef S4589514_LIS2DH_H
#define S4589514_LIS2DH_H

struct accel_data {
    char accelX[15];
    char accelY[15];
    char accelZ[15];
};

void process_lis2dh_sample(const struct device *dev, 
        struct accel_data*);

void read_lis2dh_data(const struct device* sensor, 
        struct accel_data*);



#endif