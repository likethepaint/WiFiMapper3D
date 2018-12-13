//    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

//    sensorArray sensors;

//    setup_i2c_comms();
//    setup_imu(&sensors);
//    setup_barometer(&sensors);


#include <unistd.h>

/* TI-RTOS Header files */
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>


#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/net/wifi/simplelink.h>

#include "common.h"
#include "bg_sensor_setup.h"
#include "radio_tool.h"
#include "gps.h"

/* Example/Board Header files */
#include "Board.h"

void myDelay(int count);

struct op_tasks operational_tasks;

Semaphore_Handle msgSemaphore;
Semaphore_Params msgSemaphoreParams;

Queue_Handle msgQueue;
Queue_Handle freeQueue;

Task_Struct gpsTask, imuTask, baroTask, downlinkTask, radioStatsTask, radioScanTask, isAliveTask, cmdParserTask, simplelinkTask;
Task_Struct radioToolTask;
uint8_t gpsStack[STACKSIZE];
uint8_t imuStack[STACKSIZE];
uint8_t baroStack[STACKSIZE];
uint8_t downlinkStack[STACKSIZE];
uint8_t radioToolTaskStack[STACKSIZE];
uint8_t radioScanStack[STACKSIZE];
uint8_t radioStatsStack[STACKSIZE];
uint8_t cmdParserStack[2048];
uint8_t simplelinkStack[4096];
uint8_t isAliveStack[512];

UART_Handle downlink;
UART_Params downlinkParams;

UART_Handle gps;
UART_Params gpsParams;

Task_Params taskParams;

struct bmp280_dev baro_dev;
struct LSM9DS1 imu_dev;

MsgObj q_objs[NUM_PKTS];
uint8_t d_objs[NUM_PKTS][MAX_MSG_LEN];


void downlinkTaskFunc(UArg arg0, UArg arg1) {
    Queue_Elem * elem;
    MsgObj *msg;
    uint8_t logo[2] = {0x43, 0x50};
    while(1) {
        Semaphore_pend(msgSemaphore, BIOS_WAIT_FOREVER);

        /* get message */
        elem = Queue_get(msgQueue);
        msg = (MsgObj *) elem;

        /* print value */
        UART_write(downlink, &(msg->cosmos_id), 1);
        UART_write(downlink, msg->data, msg->size);
        UART_write(downlink, logo, 2);

        /* free msg */
        Queue_put(freeQueue, (Queue_Elem *) msg);
    }
}

void isAliveTaskFunc(UArg arg0, UArg arg1) {
    while(1) {
        GPIO_toggle(Board_GPIO_LED1);
        Task_sleep(1000 * (1000 / Clock_tickPeriod));
    }
}


#define PAYLOAD_PARSE 0
#define POSSIBLE_TERM 1
uint8_t cmdParserStateMachine(uint8_t * payload) {
    uint8_t term_chars[2] = {'C','P'};
    uint8_t term_ctr = 0;

    uint8_t temp;
    uint8_t uart_status = 0;
    uint8_t valid_packet = 0;
    uint8_t payload_len = 0;
    uint8_t state = PAYLOAD_PARSE;

    while(!valid_packet) {
        UART_control(downlink, UART_CMD_ISAVAILABLE, &uart_status);
        while (!uart_status){
            Task_sleep(50 * (1000 / Clock_tickPeriod));
            UART_control(downlink, UART_CMD_ISAVAILABLE, &uart_status);
        }
        UART_read(downlink, &temp, 1);
        switch(state) {
        case PAYLOAD_PARSE:
            if (temp == term_chars[term_ctr]) {
                term_ctr++;
                state = POSSIBLE_TERM;
            } else if (payload_len < PAYLOAD_MAX_LEN) {
                    payload[payload_len++] = temp;
            } else {
                payload_len = 0;
            }
            break;
        case POSSIBLE_TERM:
            if (temp == term_chars[term_ctr]) {
                term_ctr++;
                if (term_ctr == 2) {
                    valid_packet = 1;
                }
            } else {
                uint8_t i;
                for (i = 0; i < term_ctr && payload_len < PAYLOAD_MAX_LEN; i++)
                    payload[payload_len++] = term_chars[i];
                payload[payload_len++] = temp;
                state = PAYLOAD_PARSE;
                term_ctr = 0;
            }
            break;
        default:
            payload_len = 0;
            state = PAYLOAD_PARSE;
        }
    }
    return payload_len;
}

/* Main thread to parse all input commands over downlink
 * No CCR
 */
void cmdParserTaskFunc(UArg arg0, UArg arg1) {
    uint8_t cmd_len = 0;
    uint8_t payload[PAYLOAD_MAX_LEN];

    /* Initialize the task parameters setup */
    Task_Params taskParams2;
    Task_Params_init(&taskParams2);
    taskParams2.stackSize = 4096;

    /* setup simplelink task */
    taskParams2.stack = &simplelinkStack;
    taskParams2.priority = SPAWN_TASK_PRIORITY;
    Task_construct(&simplelinkTask, (Task_FuncPtr)sl_Task, &taskParams2, NULL);

    Task_sleep(1000 * (1000 / Clock_tickPeriod));

    int16_t status = ConfigureSimpleLinkToDefaultState();
    if(status < 0) {
        while(1);
    }

    GPIO_toggle(Board_GPIO_LED0);
    setup_imu(&imu_dev);
    setup_barometer(&baro_dev);
    GPIO_toggle(Board_GPIO_LED0);

    while(1) {
        cmd_len = cmdParserStateMachine(payload);
        switch(payload[0]) {
        case CMD_RADIO:
            if (cmd_len < 2)
                break;
            if (payload[1] == RADIO_STATS_START && operational_tasks.radio_stats == 0 && cmd_len == 9) {
                uint8_t eChannel = payload[2];
                uint8_t bssid[6];
                memcpy(bssid, &(payload[3]), 6);

                // setup radio tool task
                taskParams.stackSize = STACKSIZE;
                taskParams.stack = &radioStatsStack;
                taskParams.priority = 5;
                taskParams.arg0 = eChannel;
                taskParams.arg1 = (int) bssid;
                operational_tasks.radio_stats = 1;
                Task_construct(&radioStatsTask, radioTool_StatsTask, &taskParams, NULL);

            } else if (payload[1] == RADIO_STATS_STOP && cmd_len == 2) {
                operational_tasks.radio_stats = 0;           // stop radio stats task
                Task_sleep(500 * (1000 / Clock_tickPeriod));    // wait for task program to exit
                //radioTool_StatsStop();

            } else if (payload[1] == RADIO_SCAN_START && operational_tasks.radio_scan == 0 && operational_tasks.radio_stats == 0 && cmd_len == 2) {
                // setup radio scan task
                taskParams.stackSize = STACKSIZE;
                taskParams.stack = &radioScanStack;
                taskParams.priority = 5;
                operational_tasks.radio_scan = 1;
                Task_construct(&radioScanTask, radioTool_ScanTask, &taskParams, NULL);
            }
            break;
        case CMD_IMU:
            if (cmd_len < 2)
                break;
            if (payload[1] == IMU_START && operational_tasks.imu == 0) {
                // do imu start task
                taskParams.stackSize = STACKSIZE;
                taskParams.stack = &imuStack;
                taskParams.priority = 5;
                taskParams.arg0 = (int) (&imu_dev);
                operational_tasks.imu = 1;
                Task_construct(&imuTask, imu_TaskFunc, &taskParams, NULL);

            } else if (payload[1] == IMU_STOP) {
                // stop imu task
                operational_tasks.imu = 0;                      // stop radio stats task
                Task_sleep(500 * (1000 / Clock_tickPeriod));    // wait for task program to exit
            }
            break;
        case CMD_BARO:
            if (cmd_len < 2)
                break;
            if (payload[1] == BARO_START && operational_tasks.baro == 0) {
                // do baro start task
                taskParams.stackSize = STACKSIZE;
                taskParams.stack = &baroStack;
                taskParams.priority = 5;
                taskParams.arg0 = (int) (&baro_dev);
                operational_tasks.baro = 1;
                Task_construct(&baroTask, baro_TaskFunc, &taskParams, NULL);

            } else if (payload[1] == BARO_STOP) {
                operational_tasks.baro = 0;                     // stop baro task
                Task_sleep(500 * (1000 / Clock_tickPeriod));    // wait for task program to exit
            }
            break;
        case CMD_GPS:
            if (cmd_len < 2)
                break;
            if (payload[1] == GPS_START && operational_tasks.gps == 0) {
                // do gps start task
                GPIO_write(Board_GPS_PWR_EN, 1);
                taskParams.stackSize = STACKSIZE;
                taskParams.stack = &gpsStack;
                taskParams.priority = 5;
                taskParams.arg0 = (int) (&gps);
                operational_tasks.gps = 1;
                Task_construct(&gpsTask, gps_TaskFunc, &taskParams, NULL);

            } else if (payload[1] == GPS_STOP) {
                // stop gps task
                operational_tasks.gps = 0;
                // wait for task program to exit
                Task_sleep(500 * (1000 / Clock_tickPeriod));
                GPIO_write(Board_GPS_PWR_EN, 0);
            }
            break;
        case CMD_CURRENT:
            break;
        case CMD_LED_TOGGLE:
            GPIO_toggle(Board_GPIO_LED0);
            break;
        default:
            break;
        }

    }
}

void initGPS() {
    /* Create a UART with data processing off. */
    UART_Params_init(&gpsParams);
    gpsParams.writeDataMode = UART_DATA_BINARY;
    gpsParams.readDataMode = UART_DATA_BINARY;
    gpsParams.readReturnMode = UART_RETURN_FULL;
    gpsParams.readEcho = UART_ECHO_OFF;
    gpsParams.baudRate = 9600;

    gps = UART_open(Board_UART1, &gpsParams);

    if (gps == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


void initDownlink() {

    /* Create a UART with data processing off. */
    UART_Params_init(&downlinkParams);
    downlinkParams.writeDataMode = UART_DATA_BINARY;
    downlinkParams.readDataMode = UART_DATA_BINARY;
    downlinkParams.readReturnMode = UART_RETURN_FULL;
    downlinkParams.readEcho = UART_ECHO_OFF;
    downlinkParams.baudRate = 115200;

    downlink = UART_open(Board_UART0, &downlinkParams);

    if (downlink == NULL) {
        /* UART_open() failed */
        while (1);
    }
    GPIO_write(Board_GPIO_LED1, 0);

}


/*
 *  ======== main ========
 *
 */
int main(void) {
    Board_initGeneral();
    GPIO_init();
    SPI_init();
    I2C_init();
    Timer_init();

    GPIO_write(Board_GPIO_LED1, 1);
    GPIO_write(Board_XBEE_PWR_EN, 1);
    myDelay(20e6);

    UART_init();
    /* Basic Initilization steps */
    initDownlink();
    initGPS();
    setup_i2c_comms();

    /* Setup message queues */
    uint8_t q_ctr;
    msgQueue = Queue_create(NULL, NULL);
    freeQueue = Queue_create(NULL, NULL);
    for (q_ctr = 0; q_ctr < NUM_PKTS; q_ctr++) {
        q_objs[q_ctr].data = (uint8_t *) &(d_objs[q_ctr]);
        Queue_put(freeQueue, (Queue_Elem *) &(q_objs[q_ctr].elem));
    }

    Semaphore_Params_init(&msgSemaphoreParams);
    msgSemaphore = Semaphore_create(0, &msgSemaphoreParams, NULL);
    if (msgSemaphore == NULL) {
        while(1);
    }

    /* Initialize the task parameters setup */
    Task_Params_init(&taskParams);

    /* setup cmdparser task */
    taskParams.stackSize = 2048;
    taskParams.stack = &cmdParserStack;
    taskParams.priority = 5;
    Task_construct(&cmdParserTask, cmdParserTaskFunc, &taskParams, NULL);

    /* setup isAlive task */
    taskParams.stackSize = 512;
    taskParams.stack = &isAliveStack;
    taskParams.priority = 5;
    Task_construct(&isAliveTask, isAliveTaskFunc, &taskParams, NULL);

    /* setup downlink writer task */
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &downlinkStack;
    taskParams.priority = 5;
    Task_construct(&downlinkTask, downlinkTaskFunc, &taskParams, NULL);

    operational_tasks.baro = 0;
    operational_tasks.gps = 0;
    operational_tasks.imu = 0;
    operational_tasks.radio_scan = 0;
    operational_tasks.radio_stats = 0;
    //
    //    Task_sleep(1000 * (1000 / Clock_tickPeriod));
//    /* setup GPS task */
//    taskParams.stack = &gpsTaskStack;
//    taskParams.priority = 5;
//    Task_construct(&gpsTask, gpsTaskFunc, &taskParams, NULL);
//
//    /* setup imu task */
//    taskParams.stack = &imuTaskStack;
//    taskParams.priority = 5;
//    Task_construct(&imuTask, imuTaskFunc, &taskParams, NULL);
//
//    /* setup barometer task */
//    taskParams.stack = &baroTaskStack;
//    taskParams.priority = 5;
//    Task_construct(&baroTask, baroTaskFunc, &taskParams, NULL);
//
//    /* setup downlink writer task */
//    taskParams.stack = &downlinkTaskStack;
//    taskParams.priority = 5;
//    Task_construct(&downlinkTask, downlinkTaskFunc, &taskParams, NULL);
//
//    /* setup radio tool task */
    //taskParams.stackSize = STACKSIZE;
    //taskParams.stack = &radioToolTaskStack;
    //taskParams.priority = 5;
    //Task_construct(&radioToolTask, radioToolTaskFunc, &taskParams, NULL);

    BIOS_start();

}

__asm("    .sect \".text:myDelay\"\n"
      "    .clink\n"
      "    .thumbfunc myDelay\n"
      "    .thumb\n"
      "    .global myDelay\n"
      "myDelay:\n"
      "    subs r0, #1\n"
      "    bne.n myDelay\n"
      "    bx lr\n");
