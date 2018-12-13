/*
 * gps.h
 *
 *  Created on: Oct 21, 2018
 *      Author: Brett
 */

#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include "common.h"

#define NMEA    0xF0
#define DTM     0x0A
#define GBS     0x09
#define GGA     0x00
#define GLL     0x01
#define GPQ     0x40
#define GRS     0x06
#define GSA     0x02
#define GST     0x07
#define GSV     0x03
#define RMC     0x04
#define THS     0x0E
#define TXT     0x41
#define VTG     0x05
#define ZDA     0x08

#define NAV     0x01
#define SOL     0x06
#define PVT     0x07

#define HNR     0x28
#define HNRPVT  0x00

struct nav_pvt  {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint32_t resv1;
    uint16_t resv2;
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} __attribute__((packed));

struct __attribute__((__packed__)) gpsMinData {
    uint32_t timestamp;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t fixType;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t sAcc;
};


uint8_t gpsCalculateChecksum(uint8_t *data, uint8_t size, uint8_t * checksum);
uint8_t gpsDisableMsg(UART_Handle gps, uint8_t class, uint8_t id);
uint8_t gpsEnableMsg(UART_Handle gps, uint8_t class, uint8_t id);
uint8_t gpsCheckAck(UART_Handle gps);
uint8_t gpsParserStateMachine(UART_Handle * uart, uint8_t * header, void * pkt_struct);
void gps_TaskFunc(UArg arg0, UArg arg1);

#endif /* GPS_H_ */
