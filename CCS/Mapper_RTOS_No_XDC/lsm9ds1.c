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
#include "lsm9ds1.h"
#include <stdio.h>

#define MAG_ADDR 0x1E
#define AG_ADDR 0x6B

I2C_Handle * i2c;


uint8_t msg_buffer[100];
uint8_t read_buffer[100];


uint8_t LSM9DS1_init(struct LSM9DS1 * imu) {


    // Verify communication with LSM9DS1
    uint8_t ret = 0;
    uint8_t mTest;
    imu->read(MAG_ADDR, WHO_AM_I_M, &mTest, 1);
    if (mTest != 0x3D) {
        ret |= 1;
    }

    uint8_t agTest;
    imu->read(AG_ADDR, WHO_AM_I_XG, &agTest, 1);
    if (agTest != 0x68) {
        ret |= 1;
    }

    // Gyro Initialization
    ret |= LSM9DS1_initGyro(imu);
    ret |= LSM9DS1_setGyroODR(imu->conf.gyro_odr, imu);
    ret |= LSM9DS1_setGyroScale(imu->conf.gyro_scale, imu);

    // Accelerometer Initialization
    ret |= LSM9DS1_initAccel(imu);
    ret |= LSM9DS1_setAccelODR(imu->conf.accel_odr, imu);
    ret |= LSM9DS1_setAccelScale(imu->conf.accel_scale, imu);

    // Magnetometer Initialization
    ret |= LSM9DS1_initMag(imu);
    ret |= LSM9DS1_setMagODR(imu->conf.mag_odr, imu);
    ret |= LSM9DS1_setMagScale(imu->conf.mag_scale, imu);

    // stop here for now

    return ret;
}


uint8_t LSM9DS1_initGyro(struct LSM9DS1 * imu) {
    uint8_t rslt;
    uint8_t cmd;

    // CTRL_REG1_G (Default value: 0x00)
    // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
    // ODR_G[2:0] - Output data rate selection
    // FS_G[1:0] - Gyroscope full-scale selection
    // BW_G[1:0] - Gyroscope bandwidth selection
    cmd = 0x63;
    rslt = imu->write(AG_ADDR, CTRL_REG1_G, &cmd, 1); // 119 Hz OD, 31 Hz cutoff

    // CTRL_REG2_G (Default value: 0x00)
    // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
    // INT_SEL[1:0] - INT selection configuration
    // OUT_SEL[1:0] - Out selection configuration
    cmd = 0x00;
    rslt = imu->write(AG_ADDR, CTRL_REG2_G, &cmd, 1);

    // CTRL_REG3_G (Default value: 0x00)
    // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
    // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
    // HP_EN - HPF enable (0:disabled, 1: enabled)
    // HPCF_G[3:0] - HPF cutoff frequency
    cmd = 0x00;
    rslt = imu->write(AG_ADDR, CTRL_REG3_G, &cmd, 1);

    // CTRL_REG4 (Default value: 0x38)
    // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
    // Zen_G - Z-axis output enable (0:disable, 1:enable)
    // Yen_G - Y-axis output enable (0:disable, 1:enable)
    // Xen_G - X-axis output enable (0:disable, 1:enable)
    // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
    // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
    cmd = 0x38;
    rslt = imu->write(AG_ADDR, CTRL_REG4, &cmd, 1);
    return rslt;
}

uint8_t LSM9DS1_initAccel(struct LSM9DS1 * imu)
{
    uint8_t rslt = 0;
    uint8_t cmd;

    //  CTRL_REG5_XL (0x1F) (Default value: 0x38)
    //  [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //  DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //      00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    //  Zen_XL - Z-axis output enabled
    //  Yen_XL - Y-axis output enabled
    //  Xen_XL - X-axis output enabled
    cmd = 0x38;
    rslt = imu->write(AG_ADDR, CTRL_REG5_XL, &cmd, 1);

    // CTRL_REG6_XL (0x20) (Default value: 0x00)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    // ODR_XL[2:0] - Output data rate & power mode selection
    // FS_XL[1:0] - Full-scale selection
    // BW_SCAL_ODR - Bandwidth selection
    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    cmd = 0x00;
    rslt = imu->write(AG_ADDR, CTRL_REG6_XL, &cmd, 1);

    // CTRL_REG7_XL (0x21) (Default value: 0x00)
    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
    // HR - High resolution mode (0: disable, 1: enable)
    // DCF[1:0] - Digital filter cutoff frequency
    // FDS - Filtered data selection
    // HPIS1 - HPF enabled for interrupt function
    cmd = 0x00;
    rslt = imu->write(AG_ADDR, CTRL_REG7_XL, &cmd, 1);

    return rslt;
}

uint8_t LSM9DS1_initMag(struct LSM9DS1 * imu)
{
    uint8_t rslt = 0;
    uint8_t cmd;
    // CTRL_REG1_M (Default value: 0x10)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
    // TEMP_COMP - Temperature compensation
    // OM[1:0] - X & Y axes op mode selection
    //  00:low-power, 01:medium performance
    //  10: high performance, 11:ultra-high performance
    // DO[2:0] - Output data rate selection
    // ST - Self-test enable
    //mWriteByte(CTRL_REG1_M, 0x1C); // 80 Hz ODR
    cmd = 0xDC;
    rslt = imu->write(MAG_ADDR, CTRL_REG1_M, &cmd, 1);

    // CTRL_REG2_M (Default value 0x00)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    // FS[1:0] - Full-scale configuration
    // REBOOT - Reboot memory content (0:normal, 1:reboot)
    // SOFT_RST - Reset config and user registers (0:default, 1:reset)
    cmd = 0x00;
    rslt = imu->write(MAG_ADDR, CTRL_REG2_M, &cmd, 1); // +/-4Gauss

    // CTRL_REG3_M (Default value: 0x03)
    // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
    // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
    // LP - Low-power mode cofiguration (1:enable)
    // SIM - SPI mode selection (0:write-only, 1:read/write enable)
    // MD[1:0] - Operating mode
    //  00:continuous conversion, 01:single-conversion,
    //  10,11: Power-down
    cmd = 0x00;
    rslt = imu->write(MAG_ADDR, CTRL_REG3_M, &cmd, 1); // Continuous conversion mode

    // CTRL_REG4_M (Default value: 0x00)
    // [0][0][0][0][OMZ1][OMZ0][BLE][0]
    // OMZ[1:0] - Z-axis operative mode selection
    //  00:low-power mode, 01:medium performance
    //  10:high performance, 10:ultra-high performance
    // BLE - Big/little endian data
    cmd = 0x08;
    rslt = imu->write(MAG_ADDR, CTRL_REG4_M, &cmd, 1);

    // CTRL_REG5_M (Default value: 0x00)
    // [0][BDU][0][0][0][0][0][0]
    // BDU - Block data update for magnetic data
    //  0:continuous, 1:not updated until MSB/LSB are read
    cmd = 0x00;
    rslt = imu->write(MAG_ADDR, CTRL_REG5_M, &cmd, 1);

    return rslt;
}



// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS1, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
uint8_t LSM9DS1_calibrate(float * gbias, float * abias, struct LSM9DS1 * imu)
{
    // not implemented
    return 0;
}


uint8_t LSM9DS1_readAccel(struct LSM9DS1 * imu) {
    return imu->read(AG_ADDR, OUT_X_L_XL, (void *)imu->data.accel_raw, 6);

}

uint8_t LSM9DS1_readGyro(struct LSM9DS1 * imu) {
    return imu->read(AG_ADDR, OUT_X_L_G, (void *)imu->data.gyro_raw, 6);
}

uint8_t LSM9DS1_readMag(struct LSM9DS1 * imu) {
    return imu->read(MAG_ADDR, OUT_X_L_M, (void *)imu->data.mag_raw, 6);
}

uint8_t LSM9DS1_readTemp(struct LSM9DS1 * imu) {
    return imu->read(AG_ADDR, OUT_TEMP_L, &(imu->data.die_temp), 2);

}



uint8_t LSM9DS1_setGyroScale(uint8_t gScl, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(AG_ADDR, CTRL_REG1_G, &temp, 1);
    temp &= ~(0x03 << 3);
    gScl = gScl & 0x03;
    temp |= gScl << 3;

    if (imu->write(AG_ADDR, CTRL_REG1_G, &temp, 1)) {
        return 1;       // error occurred writing byte
    }

    imu->conf.gyro_scale = gScl;
    return 0;
}

uint8_t LSM9DS1_setAccelScale(uint8_t aScl, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(AG_ADDR, CTRL_REG6_XL, &temp, 1);
    temp &= ~(0x03 << 3);

    aScl = aScl & 0x07;
    temp |= aScl << 3;

    if (imu->write(AG_ADDR, CTRL_REG6_XL, &temp, 1)) {
        return 1;       // error occurred writing byte
    }

    imu->conf.accel_scale = aScl;
    return 0;
}


uint8_t LSM9DS1_setMagScale(uint8_t mScl, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(MAG_ADDR, CTRL_REG2_M, &temp, 1);
    temp &= ~(0x03 << 5);
    mScl = mScl & 0x07;
    temp |= mScl << 5;

    if (imu->write(MAG_ADDR, CTRL_REG2_M, &temp,1)) {
        return 1;
    }

    imu->conf.mag_scale = mScl;
    return 0;
}



uint8_t LSM9DS1_setGyroODR(uint8_t gRate, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(AG_ADDR, CTRL_REG1_G, &temp, 1);
    temp &= 0x1F;
    temp |= (gRate << 5);

    if (imu->write(AG_ADDR, CTRL_REG1_G, &temp, 1)) {
        return 1;
    }

    imu->conf.gyro_odr = gRate;
    return 0;
}

uint8_t LSM9DS1_setAccelODR(uint8_t aRate, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(AG_ADDR, CTRL_REG6_XL, &temp, 1);
    temp &= 0x1F;
    temp |= (aRate << 5);

    if (imu->write(AG_ADDR, CTRL_REG6_XL, &temp, 1)) {
        return 1;
    }

    imu->conf.accel_odr = aRate;
    return 0;
}

uint8_t LSM9DS1_setMagODR(uint8_t mRate, struct LSM9DS1 * imu) {
    uint8_t temp;
    imu->read(MAG_ADDR, CTRL_REG1_M, &temp, 1);
    temp &= ~(0x07 << 2);
    temp |= (mRate << 2);

    if (imu->write(MAG_ADDR, CTRL_REG1_M, &temp, 1)) {
        return 1;
    }

    imu->conf.mag_odr = mRate;
    return 0;
}
