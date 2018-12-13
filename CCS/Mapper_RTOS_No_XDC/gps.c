/*
 * gps.c
 *
 *  Created on: Oct 21, 2018
 *      Author: Brett
 */

#include <ti/drivers/UART.h>
#include "gps.h"

#define IDLE            0
#define HEAD_UPPER      1
#define HEAD_LOWER      2
#define CLASS_ID        3
#define ID_MATCHED      4
#define THROWAWAY       5
#define IN_MSG          6
#define CALC_CHECKSUM   7


uint8_t gpsCalculateChecksum(uint8_t *data, uint8_t size, uint8_t * checksum) {
    uint8_t i = 0;
    checksum[0] = 0;
    checksum[1] = 0;
    for(i = 0; i < size; ++i) {
        checksum[0] += data[i];
        checksum[1] += checksum[0];
    }
    return 0;
}


uint8_t gpsSetRate(UART_Handle gps, uint16_t measRate, uint16_t navRate) {
    uint8_t packet[16] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    packet[6] = (uint8_t)(measRate >> 8);
    packet[7] = (uint8_t)(measRate & 0x00FF);
    packet[8] = (uint8_t)(navRate >> 8);
    packet[9] = (uint8_t)(navRate & 0x00FF);

    gpsCalculateChecksum(&packet[2], 10, &packet[12]);
    UART_write(gps, packet, 14);
    while (gpsCheckAck(gps)) {
        UART_write(gps, packet, 14);
    }
    return 0;
}


uint8_t gpsEnableMsg(UART_Handle gps, uint8_t class, uint8_t id) {
    uint8_t packet[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    packet[6] = class;
    packet[7] = id;
    packet[9] = 0x01;

    gpsCalculateChecksum(&packet[2], 12, &packet[14]);
    UART_write(gps, packet, 16);
    while (gpsCheckAck(gps)) {
        UART_write(gps, packet, 16);
    }
    return 0;
}

uint8_t gpsDisableMsg(UART_Handle gps, uint8_t class, uint8_t id) {
    uint8_t packet[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    packet[6] = class;
    packet[7] = id;

    gpsCalculateChecksum(&packet[2], 12, &packet[14]);
    UART_write(gps, packet, 16);
    while (gpsCheckAck(gps)) {
        UART_write(gps, packet, 16);
    }
    return 0;
}

uint8_t gpsCheckAck(UART_Handle gps) {
    uint8_t ret;
    uint8_t counter = 0;
    uint8_t ack_buffer[10];
    uint8_t peeker = 0x00;

    UART_control(gps, UART_CMD_ISAVAILABLE, &ret);
    if (ret) {
        while(peeker != 0xB5 && ret) {
            UART_read(gps, &peeker, 1);
            UART_control(gps, UART_CMD_ISAVAILABLE, &ret);
        }
        ack_buffer[counter] = peeker;
        while(counter < 10) {
            UART_control(gps, UART_CMD_ISAVAILABLE, &ret);
            if (ret) {
                counter++;
                UART_read(gps, &ack_buffer[counter], 1);
            }
        }
    }
    if (ack_buffer[3] != 0x01)
        return 1;
    return 0;
}

uint8_t gpsParserStateMachine(UART_Handle * uart, uint8_t * header, void * pkt_struct) {
    uint8_t buffer[100];
    uint8_t checksum[2];
    uint8_t checksum_check[2];
    uint8_t checksum_count = 0;

    uint8_t msg_index = 0;
    uint8_t msg_length = 0;
    uint8_t temp;

    uint8_t state = IDLE;
    uint8_t uart_status = 0;
    uint8_t valid_packet = 0;
    while(!valid_packet) {
        while (!uart_status){
            UART_control(*uart, UART_CMD_ISAVAILABLE, &uart_status);
            if (!uart_status)
                Task_sleep(100 * (1000 / Clock_tickPeriod));
        }
        UART_read(*uart, &temp, 1);
        switch(state) {
        case IDLE:
            if (temp == header[0]) {
                state = HEAD_UPPER;
                break;
            }
            state = IDLE;
            break;
        case HEAD_UPPER:
            if (temp == header[1]) {
                state = HEAD_LOWER;
                break;
            }
            state = IDLE;
            break;
        case HEAD_LOWER:
            if (temp == header[2]) {
                buffer[0] = temp;
                state = CLASS_ID;
                break;
            }
            state = IDLE;
            break;
        case CLASS_ID:
            if (temp == header[3]) {
                buffer[1] = temp;
                state = ID_MATCHED;
                break;
            }
            state = IDLE;
            break;
        case ID_MATCHED:
            msg_length = temp + 4;
            buffer[2] = temp;
            state = THROWAWAY;
            break;
        case THROWAWAY:
            state = IN_MSG;
            buffer[3] = temp;
            msg_index = 4;
            break;
        case IN_MSG:
            if(msg_index < msg_length) {
                buffer[msg_index] = temp;
                msg_index++;
                if (msg_index == msg_length) {
                    checksum_count = 0;
                    state = CALC_CHECKSUM;
                }
            }
            break;
        case CALC_CHECKSUM:
            if (checksum_count < 1) {
                checksum[checksum_count] = temp;
                checksum_count++;
                break;
            }
            checksum[checksum_count] = temp;
            checksum_count++;
            gpsCalculateChecksum(buffer, msg_length, checksum_check);
            if ((checksum[0] == checksum_check[0]) && (checksum[1] == checksum_check[1])) {
                valid_packet = 1;
                memcpy(pkt_struct, &buffer[4], msg_length - 4);
            }
        default:
            state = IDLE;
            break;
        }
    }

    return 0;
}

/* Simple GPS task function
 * Wait for PVT packets and then put them on the message queue
 */
void gps_TaskFunc(UArg arg0, UArg arg1) {
    struct gpsMinData gps_data;
    struct nav_pvt pvt;
    uint8_t header[4] = {0xB5, 0x62, 0x01, 0x07};
    Queue_Elem * elem;
    MsgObj * msg;
    uint8_t ret = 0;
    UART_Handle gps = *((UART_Handle *) arg0);

    // Setup default messages
    gpsDisableMsg(gps, NMEA, GGA);
    gpsDisableMsg(gps, NMEA, GLL);
    gpsDisableMsg(gps, NMEA, VTG);
    gpsDisableMsg(gps, NMEA, GSV);
    gpsDisableMsg(gps, NMEA, GSA);
    gpsDisableMsg(gps, NMEA, RMC);
    gpsEnableMsg(gps, NAV, PVT);

    gpsSetRate(gps, 500, 2);

    while(operational_tasks.gps) {
        gpsParserStateMachine(&gps, header, (void *)(&pvt));
        gps_data.timestamp = Clock_getTicks();
        gps_data.year = pvt.year;
        gps_data.month = pvt.month;
        gps_data.day = pvt.day;
        gps_data.hour = pvt.hour;
        gps_data.min = pvt.min;
        gps_data.sec = pvt.sec;
        gps_data.fixType = pvt.fixType;
        gps_data.numSV = pvt.numSV;
        gps_data.lon = pvt.lon;
        gps_data.lat = pvt.lat;
        gps_data.height = pvt.height;
        gps_data.hMSL = pvt.hMSL;
        gps_data.hAcc = pvt.hAcc;
        gps_data.vAcc = pvt.vAcc;
        gps_data.velN = pvt.velN;
        gps_data.velE = pvt.velE;
        gps_data.velD = pvt.velD;
        gps_data.sAcc = pvt.sAcc;

        // Add gps packet to send out
        elem = Queue_get(freeQueue);
        msg = (MsgObj *) elem;
        if (msg) {
            msg->cosmos_id = COSMOS_GPS_ID;
            msg->size = sizeof(struct gpsMinData);
            memcpy(msg->data,&(gps_data), sizeof(struct gpsMinData));
            Queue_put(msgQueue, (Queue_Elem *) elem);
            Semaphore_post(msgSemaphore);
        }

    }
    return;
}
