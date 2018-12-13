/*
 * bg_sensor_setup.c
 *
 *  Created on: Oct 21, 2018
 *      Author: Brett
 */

#include "bg_sensor_setup.h"


static I2C_Handle   i2c;
static uint8_t      msg_buffer[50];

int16_t mBiasRaw[3] = {0, 0, 0};
int16_t gBiasRaw[3] = {0, 0, 0};
int16_t aBiasRaw[3] = {0, 0, 0};

uint8_t LSM9DS1_calibrateMag(struct LSM9DS1 * imu);
uint8_t LSM9DS1_calibrateAG(struct LSM9DS1 * imu);


static int delayms(unsigned long counts){
    Task_sleep(counts * (1000 / Clock_tickPeriod));
    return 0;
}

static uint8_t I2C_writeBytes(uint8_t slaveAddress, uint8_t subAddress, void * data, uint8_t count) {
    I2C_Transaction payload;

    msg_buffer[0] = subAddress;
    memcpy(&msg_buffer[1], data, count);

    payload.slaveAddress = slaveAddress;
    payload.writeBuf = msg_buffer;
    payload.writeCount = count+1;
    payload.readCount = 0;

    if(I2C_transfer(i2c, &payload))
        return 0;
    return 1;
}

static uint8_t I2C_readBytes(uint8_t slaveAddress, uint8_t subAddress, void * readBuf, uint8_t count) {
    I2C_Transaction payload;

    payload.slaveAddress = slaveAddress;
    payload.writeBuf = &subAddress;
    payload.writeCount = 1;
    payload.readBuf = readBuf;
    payload.readCount = count;

    if(I2C_transfer(i2c, &payload))
        return 0;
    return 1;
}


uint8_t setup_i2c_comms() {
    I2C_Params i2cParams;

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CC3220S_LAUNCHXL_I2C0, &i2cParams);
    if (i2c == NULL) {
        return 1;
    }
    return 0;
}

uint8_t setup_imu(struct LSM9DS1 * imu) {
    /* set initial configuration */
    imu->read = I2C_readBytes;
    imu->write = I2C_writeBytes;
    imu->conf.accel_odr = A_ODR_119;
    imu->conf.accel_scale = A_SCALE_4G;
    imu->conf.gyro_odr = G_ODR_119;
    imu->conf.gyro_scale = G_SCALE_500DPS;
    imu->conf.mag_odr = M_ODR_80;
    imu->conf.mag_scale = M_SCALE_4GS;

    if (LSM9DS1_init(imu) != 0)
        return 1;
    //if (LSM9DS1_calibrateAG(imu) != 0)
    //    return 1;
    //if (LSM9DS1_calibrateMag(imu) != 0)
    //    return 1;

    return 0;
}

uint8_t LSM9DS1_calibrateAG(struct LSM9DS1 * imu) {
    uint8_t samples = 128;
    int ii;
    int32_t aBiasRawTemp[3] = {0, 0, 0};
    int32_t gBiasRawTemp[3] = {0, 0, 0};

    for(ii = 0; ii < samples ; ii++)
    {   // Read the gyro data stored in the FIFO
        Task_sleep(10 * (1000 / Clock_tickPeriod));
        LSM9DS1_readGyro(imu);
        gBiasRawTemp[0] += imu->data.gyro_raw[0];
        gBiasRawTemp[1] += imu->data.gyro_raw[1];
        gBiasRawTemp[2] += imu->data.gyro_raw[2];
        LSM9DS1_readAccel(imu);
        aBiasRawTemp[0] += imu->data.accel_raw[0];
        aBiasRawTemp[1] += imu->data.accel_raw[1];
        aBiasRawTemp[2] += imu->data.accel_raw[2] - 8192; // Assumes sensor facing up!
    }
    for (ii = 0; ii < 3; ii++)
    {
        gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
        aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
    }
    return 0;
}

uint8_t LSM9DS1_calibrateMag(struct LSM9DS1 * imu) {
    int16_t i, j;
    int16_t samples = 72;
    int32_t magTemp[3] = {0, 0, 0};

    for (i=0; i < samples; i++)
    {
        Task_sleep(20 * (1000 / Clock_tickPeriod));
        LSM9DS1_readMag(imu);
        magTemp[0] += imu->data.mag_raw[0];
        magTemp[1] += imu->data.mag_raw[1];
        magTemp[2] += imu->data.mag_raw[2];
    }
    for (j = 0; j < 3; j++)
    {
        mBiasRaw[j] = magTemp[j] / samples;
    }
    return 0;
}


uint8_t setup_barometer(struct bmp280_dev * bmp) {
    uint8_t rslt = 0;
    uint8_t meas_dur = 0;
    struct bmp280_config * conf = &(bmp->conf);

    bmp->dev_id = BMP280_I2C_ADDR_PRIM;
    bmp->intf = BMP280_I2C_INTF;
    bmp->read = I2C_readBytes;
    bmp->write = I2C_writeBytes;
    bmp->delay_ms = delayms;

    rslt = bmp280_init(bmp);

    if (rslt == BMP280_OK) {
        rslt = bmp280_get_config(conf, bmp);
        if (rslt == BMP280_OK) {
            conf->filter = BMP280_FILTER_COEFF_16;
            conf->os_pres = BMP280_OS_16X;
            conf->os_temp = BMP280_OS_2X;
            conf->filter = BMP280_FILTER_COEFF_16;
            conf->odr = BMP280_ODR_0_5_MS;

            rslt = bmp280_set_config(conf, bmp);
            if (rslt == BMP280_OK) {
                bmp280_set_power_mode(BMP280_NORMAL_MODE, bmp);
                meas_dur = bmp280_compute_meas_time(bmp);
            }
        }
    }
    return rslt;
}

void imu_TaskFunc(UArg arg0, UArg arg1) {
    uint8_t i;
    Queue_Elem * elem;
    MsgObj * msg;
    struct LSM9DS1 * imu = (struct LSM9DS1 *) arg0;

    while(operational_tasks.imu) {
        LSM9DS1_readAccel(imu);
        LSM9DS1_readGyro(imu);
        LSM9DS1_readMag(imu);
        LSM9DS1_readTemp(imu);
        imu->data.timestamp = Clock_getTicks();

        for (i = 0; i < 3; i++) {
            imu->data.accel_raw[i] -= aBiasRaw[i];
            imu->data.gyro_raw[i] -= gBiasRaw[i];
            imu->data.mag_raw[i] -= mBiasRaw[i];
        }

        elem = Queue_get(freeQueue);
        msg = (MsgObj *) elem;
        if (msg) {
            msg->cosmos_id = COSMOS_IMU_ID;
            msg->size = sizeof(struct LSM9DS1_data);
            memcpy(msg->data,&(imu->data), sizeof(struct LSM9DS1_data));
            Queue_put(msgQueue, (Queue_Elem *) elem);
            Semaphore_post(msgSemaphore);
        }
        Task_sleep(10 * (1000 / Clock_tickPeriod));
    }
    return;
}


void baro_TaskFunc(UArg arg0, UArg arg1) {
    Queue_Elem * elem;
    MsgObj * msg;
    struct bmp280_dev * baro = (struct bmp280_dev *)arg0;

    while(operational_tasks.baro) {
        bmp280_get_uncomp_data(&(baro->uncomp_data), baro);
        baro->data.comp_press = bmp280_comp_pres_32bit(baro->uncomp_data.uncomp_press, baro);
        baro->data.comp_temp = bmp280_comp_temp_32bit(baro->uncomp_data.uncomp_temp, baro);
        baro->data.timestamp = Clock_getTicks();

        // Add baro packet to send out
        elem = Queue_get(freeQueue);
        msg = (MsgObj *) elem;
        if (msg) {
            msg->cosmos_id = COSMOS_BARO_ID;
            msg->size = sizeof(struct bmp280_comp_data);
            memcpy(msg->data,&(baro->data), sizeof(struct bmp280_comp_data));
            Queue_put(msgQueue, (Queue_Elem *) elem);
            Semaphore_post(msgSemaphore);
        }
        Task_sleep(20 * (1000 / Clock_tickPeriod));
    }
    return;
}
