#ifndef LSM9DS1_H_
#define LSM9DS1_H_


/*
 * lsm9ds1.c
 *
 *  Created on: Jul 28, 2018
 *      Author: Brett Glidden
 *
 *  Simple driver for the LSM9DS1 9-Axis IMU
 *  Uses I2C interface of the IC
 */

/* Header includes */
#include <ti/drivers/I2C.h>


/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37


///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33


/* Function pointer type definitions */
typedef uint8_t (*lsm9ds1_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, void *data, uint8_t len);

/* Accel scale defines all possible FSR's of the accelerometer */
#define A_SCALE_2G  0
#define A_SCALE_16G 1
#define A_SCALE_4G  2
#define A_SCALE_8G  3

/* Gyro scale defines the possible full-scale ranges of the gyroscope */
#define G_SCALE_245DPS  0     // 245 DPS
#define G_SCALE_500DPS  1     // 500 DPS
#define G_SCALE_2000DPS 2    // 2000 DPS

/* Mag scale defines all possible FSR's of the magnetometer */
#define M_SCALE_4GS     0
#define M_SCALE_8GS     1
#define M_SCALE_12GS    2
#define M_SCALE_16GS    3

/* gyro_odr defines all possible data rate/bandwidth combos of the gyro */

#define G_ODR_PD    0       // Power down
#define G_ODR_149   1      // 14.9 Hz
#define G_ODR_595   2      // 59.5 Hz
#define G_ODR_119   3      // 119 Hz
#define G_ODR_238   4      // 238 Hz
#define G_ODR_476   5      // 476 Hz
#define G_ODR_952   6      // 952 Hz


/* accel_odr defines all possible output rates of the accelerometer */

#define A_POWER_DOWN    0       // Power down
#define A_ODR_10        1       // 10 Hz
#define A_ODR_50        2       // 50 Hz
#define A_ODR_119       3      // 119 Hz
#define A_ODR_238       4      // 238 Hz
#define A_ODR_476       5      // 476 Hz


/* accel_abw defines all possible anti-aliasing filter rates of the accelerometer */
#define A_ABW_408   0      // 408 Hz
#define A_ABW_211   1      // 211 Hz
#define A_ABW_105   2      // 105 Hz
#define A_ABW_50    3       // 50 Hz

/* mag_odr defines all possible output data rates of the magnetometer */
#define M_ODR_0625  0     // 0.625 Hz
#define M_ODR_125   1      // 1.25 Hz
#define M_ODR_250   2      // 2.5 Hz
#define M_ODR_5     3        // 5 Hz
#define M_ODR_10    4       // 10 Hz
#define M_ODR_20    5       // 20 Hz
#define M_ODR_40    6       // 40 Hz
#define M_ODR_80    7       // 80 Hz


#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW  1

#define INT_PUSH_PULL   0
#define INT_OPEN_DRAIN  1


struct __attribute__((__packed__)) LSM9DS1_data {
    uint32_t timestamp;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t die_temp;
};

struct LSM9DS1_conf {
    uint8_t accel_scale;
    uint8_t gyro_scale;
    uint8_t mag_scale;
    uint8_t accel_odr;
    uint8_t gyro_odr;
    uint8_t mag_odr;
    uint8_t accel_abw;
};

struct LSM9DS1 {
    struct LSM9DS1_conf conf;
    struct LSM9DS1_data data;
    lsm9ds1_com_fptr_t read;
    lsm9ds1_com_fptr_t write;
};

/* Functions listing */

/* LSM9DS1_init() -- initialize the lsm9ds1 device for I2C
 */
uint8_t LSM9DS1_init(struct LSM9DS1 * imu);


/* LSM9DS1_verify() -- verify LSM9DS1 operation, verify connection
 * This function initiates a connection to the LSM9DS1 to verify the WHO_AM_I for
 * the accelerometer, gyroscope, and magnetometer
 * Returns: Success '1', Failure '0'
 */
//uint8_t LSM9DS1_init(enum gyro_scale gScl, enum accel_scale aScl, enum mag_scale mScl, enum gyro_odr gODR, enum accel_odr aODR, enum mag_odr mODR);


/*****************************************************/

uint8_t LSM9DS1_initGyro(struct LSM9DS1 * imu);

uint8_t LSM9DS1_readGyro(struct LSM9DS1 * imu);

uint8_t LSM9DS1_setGyroScale(uint8_t gScl, struct LSM9DS1 * imu);

uint8_t LSM9DS1_setGyroODR(uint8_t gRate, struct LSM9DS1 * imu);

/****************************************************/

uint8_t LSM9DS1_initAccel(struct LSM9DS1 * imu);

uint8_t LSM9DS1_readAccel(struct LSM9DS1 * imu);

uint8_t LSM9DS1_setAccelScale(uint8_t aScl, struct LSM9DS1 * imu);

uint8_t LSM9DS1_setAccelODR(uint8_t aRate, struct LSM9DS1 * imu);

uint8_t LSM9DS1_setAccelABW(uint8_t abwRate, struct LSM9DS1 * imu);

/*****************************************************/

uint8_t LSM9DS1_initMag(struct LSM9DS1 * imu);

uint8_t LSM9DS1_readMag(struct LSM9DS1 * imu);

uint8_t LSM9DS1_setMagScale(uint8_t mScl, struct LSM9DS1 * imu);

uint8_t LSM9DS1_setMagODR(uint8_t mRate, struct LSM9DS1 * imu);

/*****************************************************/

uint8_t LSM9DS1_readTemp(struct LSM9DS1 * imu);

uint8_t LSM9DS1_calibrate(float gbias[3], float abias[3], struct LSM9DS1 * imu);

#endif /* LSM9DS1_H_ */

