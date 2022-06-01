/**
 ************************************************************************
 * @file hal_imu.h
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Contains defines and helpers that assist the IMU thread
 **********************************************************************
 * */
#ifndef HAL_IMU_H
#define HAL_IMU_H

#define SW1_NODE	DT_ALIAS(sw1)
#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
    #define PARTICLE_ARGON      true
    #define THINGY_52           false
#else
    #define PARTICLE_ARGON      false
    #define THINGY_52           true
#endif

/* Debug Thread Stack size */
#define THREAD_IMU_RW_STACK 2048

/* Debug Thread Priority */
#define THREAD_PRIORITY_IMU -1

#define INV_SUCCESS 0
#define INV_ERROR 0x20

// int ax, ay, az;
// int gx, gy, gz;
// int mx, my, mz;
// long qw, qx, qy, qz;
// long temperature;
// unsigned long time;
// float pitch, roll, yaw;
// float heading;

enum t_axisOrder
{
    X_AXIS, // 0
    Y_AXIS, // 1
    Z_AXIS  // 2
};

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL (1 << 1)
#define UPDATE_GYRO (1 << 2)
#define UPDATE_COMPASS (1 << 3)
#define UPDATE_TEMP (1 << 4)

#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW 1
#define INT_LATCHED 1
#define INT_50US_PULSE 0

#define MAX_DMP_SAMPLE_RATE 200 // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE 512    // Max FIFO buffer size

#define ORIENT_PORTRAIT 0
#define ORIENT_LANDSCAPE 1
#define ORIENT_REVERSE_PORTRAIT 2
#define ORIENT_REVERSE_LANDSCAPE 3

//Functions

void thread_imu_rw(void);

/* IMU FUNCTIONS */
float getGyroSens(void);

unsigned short getAccelSens(void);

int setSensors(unsigned char sensors);

unsigned short getAccelSens(void);

int updateAccel(void);

int updateGyro(void);

int updateCompass(void);

int setGyroFSR(unsigned short fsr);

int setAccelFSR(unsigned char fsr);

int setLPF(unsigned short lpf);

int setSampleRate(unsigned short rate);

int setCompassSampleRate(unsigned short rate);

bool dataReady(void);

int update(unsigned char sensors);

float calcAccel(int axis);

float calcGyro(int axis);

float calcMag(int axis);

int updateTemperature(void);

int begin(void);

void printIMUData(void);

int dmpLoad(void);

int dmpBegin(unsigned short features, unsigned short fifoRate);

int dmpEnableFeatures(unsigned short mask);

int dmpSetFifoRate(unsigned short rate);

int dmpSetPedometerSteps(unsigned long steps);

int dmpSetPedometerTime(unsigned long time);

unsigned long dmpGetPedometerTime(void);

unsigned long dmpGetPedometerSteps(void);

int dmpBegin(unsigned short features, unsigned short fifoRate);

#endif