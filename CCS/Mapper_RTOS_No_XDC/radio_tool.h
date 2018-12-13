/*
 * radio_tool.h
 *
 *  Created on: Nov 3, 2018
 *      Author: Brett
 */

#ifndef RADIO_TOOL_H_
#define RADIO_TOOL_H_

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/UART.h>
#include "common.h"

#define RADIO_CMD_BUFF_SIZE_MAX 100
#define DEV_VER_LENGTH          136

#define CW_LOW_TONE             (-25)
#define CW_HIGH_TONE            (25)
#define CW_STOP                 (128)

/* Radio Tool Error Codes */
#define RADIO_TOOL_ERROR_STARTING_RADIO_TOOL        (-101)
#define RADIO_TOOL_ERROR_STOPPING_RADIO_TOOL        (-102)
#define RADIO_TOOL_ERROR_GETTING_MAC_ADDR           (-110)
#define RADIO_TOOL_ERROR_GETTING_DEV_VERSION        (-111)

#define RADIO_TOOL_ERROR_TX_TYPE_UNKNOWN            (-100)
#define RADIO_TOOL_ERROR_TX_CREATING_RAW_SOCKET     (-101)
#define RADIO_TOOL_ERROR_TX_FULL_SIZE_DATA          (-102)

#define RADIO_TOOL_ERROR_RX_CREATING_RAW_SOCKET     (-103)

#define RADIO_TOOL_ERROR_GETTING_RX_STATS           (-111)

#define RADIO_TOOL_TX_POWER_LEVEL_MIN               (0)
#define RADIO_TOOL_TX_POWER_LEVEL_MAX               (15)
#define RADIO_TOOL_TX_CW_TONE_MIN                   (-25)
#define RADIO_TOOL_TX_CW_TONE_MAX                   (25)
#define RADIO_TOOL_TX_CHANNEL_MIN                   (1)
#define RADIO_TOOL_TX_CHANNEL_MAC                   (13)
#define RADIO_TOOL_TX_PREAMBLE_MIN                  (0)
#define RADIO_TOOL_TX_PREAMBLE_MAX                  (1)
#define RADIO_TOOL_TX_SIZE_MIN                      (24)
#define RADIO_TOOL_TX_SIZE_MAX                      (1400)
#define RADIO_TOOL_TX_DELAY_MIN                     (100)
#define RADIO_TOOL_TX_DELAY_MAX                     (1000000)
#define RADIO_TOOL_TX_AMOUNT_MIN                    (0)
#define RADIO_TOOL_TX_AMOUNT_MAX                    (1000000)
#define RADIO_TOOL_TX_CCA_THRESHOLD_MIN             (0)
#define RADIO_TOOL_TX_CCA_THRESHOLD_MAX             SL_TX_INHIBIT_THRESHOLD_MAX

#define OSI_STACK_SIZE          (4096)
#define TASK_STACK_SIZE         (2048)
#define SPAWN_TASK_PRIORITY     (9)
#define PASSWD_LEN_MAX          (63)
#define PASSWD_LEN_MIN          (8)
#define WLAN_SCAN_COUNT         (20)
#define MAX_FILE_NAME_LEN       (32)
#define MAX_TEXT_PAD_SIZE       (256)
#define MAX_FILE_LIST           (20)
#define MAX_BUF_SIZE            (1400)
#define CHANNEL_MASK_ALL        (0x1FFF)
#define RSSI_TH_MAX             (-95)
#define SL_STOP_TIMEOUT         (200)
#define DEV_TYPE_LEN            (17)
#define IPV6_ADDR_LEN           (16)
#define IPV4_ADDR_LEN           (4)
#define DEVICE_ERROR            (1)
#define WLAN_ERROR              (2)
#define SOCKET_ERROR            (3)
#define NETAPP_ERROR            (4)
#define OS_ERROR                (5)
#define CMD_ERROR               (6)
#define DEFAULT_ERROR           (7)
//#define DEVICE_ERROR            ("Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
//#define WLAN_ERROR              ("WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
//#define SOCKET_ERROR            ("BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" section in errors.h")
//#define NETAPP_ERROR            ("Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h")
//#define OS_ERROR                ("OS error, please refer \"NETAPP ERRORS CODES\" section in errno.h")
//#define CMD_ERROR               ("Invalid option/command.")


typedef struct
{
    uint32_t    TimeStamp;
    uint32_t    ReceivedValidPackets;                    /* sum of the packets that been received OK (include filtered) */
    int16_t     AverageRssi;                           /* average RSSI for all valid data packets received */
}__attribute__((__packed__)) RadioTool_RX_Stats;

#define ASSERT_ON_ERROR_NO_PRINT(error_code)\
            {\
                 if(error_code < 0) \
                   {\
                        return error_code;\
                 }\
            }
#define ASSERT_ON_ERROR(ret, errortype)\
        {\
            if (assertOnError(ret, errortype))\
            {\
                return -1;\
            }\
        }

#define SHOW_WARNING(ret, errortype)        UART_PRINT("\n\r[line:%d, error code:%d] %s\n\r", __LINE__, ret, errortype);


void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent);
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);
void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t   *pSlTriggerEvent);
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent);
void SimpleLinkNetAppRequestHandler(SlNetAppRequest_t  *pNetAppRequest,
                                    SlNetAppResponse_t *pNetAppResponse);
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                      SlNetAppHttpServerResponse_t *pHttpResponse);
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent);
void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer);
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse);


void radioTool_ScanTask(UArg arg0, UArg arg1);
void radioTool_StatsTask(UArg arg0, UArg arg1);
int8_t radioTool_addBSSIDFilter(uint8_t * bssid);
uint8_t radioTool_IsActiveNwp(void);
void radioTool_setEnv(Queue_Handle * m, Queue_Handle *e, Semaphore_Handle * s);
int16_t ConfigureSimpleLinkToDefaultState();
int16_t RadioStartRX(uint8_t eChannel, uint8_t eEnableACKs);
int16_t RadioStopRX(void);
int16_t RadioGetStats(uint8_t *pRxStats);
int16_t radioTool_StatsStop();

#endif /* RADIO_TOOL_H_ */
