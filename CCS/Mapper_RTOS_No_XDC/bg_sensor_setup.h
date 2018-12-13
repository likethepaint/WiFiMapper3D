/*
 * bg_sensor_setup.h
 *
 *  Created on: Oct 21, 2018
 *      Author: Brett
 */

#ifndef BG_SENSOR_SETUP_H_
#define BG_SENSOR_SETUP_H_

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "Board.h"
#include "lsm9ds1.h"
#include "bmp280.h"
#include "common.h"

typedef struct sensorElements_t
{
    /* IMU instance */
    struct LSM9DS1 imu;

    /* Barometer instance */
    struct bmp280_dev baro;

    /* GPS instance */

} sensorArray;


uint8_t setup_i2c_comms();
uint8_t setup_imu(struct LSM9DS1 * imu);
uint8_t setup_barometer(struct bmp280_dev * bmp);
void imu_TaskFunc(UArg arg0, UArg arg1);
void baro_TaskFunc(UArg arg0, UArg arg1);

#endif /* BG_SENSOR_SETUP_H_ */
