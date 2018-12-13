/*
 * radio_tool.c
 *
 *  Created on: Nov 3, 2018
 *      Author: Brett
 */

#include "radio_tool.h"
#include "board.h"
#include <ti/net/slneterr.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/Timer.h>

/* Definitions for template frame */
#define RATE RATE_1M
#define FRAME_TYPE          0xC8                /* QOS data */
#define FRAME_CONTROL       0x01                /* TO DS*/
#define DURATION            0x00, 0x00
#define RECEIVE_ADDR        0x55, 0x44, 0x33, 0x22, 0x11, 0x00
#define TRANSMITTER_ADDR    0x00, 0x11, 0x22, 0x33, 0x44, 0x55
#define DESTINATION_ADDR    0x55, 0x44, 0x33, 0x22, 0x11, 0x00
#define FRAME_NUMBER        0x00, 0x00
#define QOS_CONTROL         0x00, 0x00

#define RA_OFFSET               4
#define TA_OFFSET               10
#define DA_OFFSET               16
#define FRAME_SIZE              1500
#define RX_BUFFER_SIZE          1470
#define RX_RECV_TIMEOUT_SEC     0
#define RX_RECV_TIMEOUT_USEC    20000


extern unsigned int g_ConnectionStatus;
int  retVal = 0;

/**************************** Template frame **********************************/
uint8_t TemplateFrame[] = {
    /*---- wlan header start -----*/
    FRAME_TYPE,         /* version type and sub type */
    FRAME_CONTROL,      /* Frame control flag*/
    DURATION,           /* duration */
    RECEIVE_ADDR,       /* Receiver Address */
    TRANSMITTER_ADDR,   /* Transmitter Address */
    DESTINATION_ADDR,   /* destination Address*/
    FRAME_NUMBER,       /* frame number */
    QOS_CONTROL         /* QoS control */
};

typedef struct CreateFilterCmd
{
    SlWlanRxFilterRuleType_t     ruleType;       /* Header or combination filter */
    SlWlanRxFilterID_t           filterID;       /* Returned value for 'sl_WlanRxFilterAdd()' */
    SlWlanRxFilterFlags_u        flags;          /* Dictates filter behavior */
    SlWlanRxFilterRule_u         rule;           /* Match criteria */
    SlWlanRxFilterTrigger_t      trigger;        /* What are the preconditions to trigger the filter */
    SlWlanRxFilterAction_t       action;         /* Operation that execute upon a filter match */
}CreateFilterCmd_t;

int16_t rxSocket;
uint8_t     DataFrame[FRAME_SIZE];
volatile uint8_t txRequested=0;
volatile uint8_t txFinished=1;
uint8_t g_CurrentTxMode;
uint8_t DataPacket[FRAME_SIZE];
uint8_t ucTxBuffer[RADIO_CMD_BUFF_SIZE_MAX];
volatile uint8_t isActiveNwp = 0;
int32_t rssi = 0;
int32_t rssi_count = 0;

/* Simple assertion testing if NWP is on */
uint8_t radioTool_IsActiveNwp(void) {
    return isActiveNwp;
}

/* Provide info on errors from the NWP
 * Send errors to COSMOS via error packets
 */
uint8_t assertOnError(int32_t ret, int16_t err){\
    Queue_Elem * elem;
    MsgObj * msg;

    if(ret < 0) {
        elem = Queue_get(freeQueue);
        msg = (MsgObj *) elem;
        if (msg) {
            msg->cosmos_id = COSMOS_ERR_ID;
            msg->size = sizeof(err);
            memcpy(msg->data,&err, sizeof(err));
            Queue_put(msgQueue, (Queue_Elem *) elem);
            Semaphore_post(msgSemaphore);
        }
        return 1;
    }
    return 0;
}

/* Simple transceiver setup function
 * Places the NWP into a non-connected mode.
 * Suitable for transceiver operation and statistics
 */
int16_t radioTool_TranscieverSetup(void) {
    uint8_t  Policy;
    int16_t status;
    if(!radioTool_IsActiveNwp())
    {
        status = sl_Start(0, 0 ,0);
        ASSERT_ON_ERROR(status, DEVICE_ERROR);
        isActiveNwp = 1;
    }

    /* Configure Role, an set connection Policy to None. */
    status = sl_WlanSetMode(ROLE_STA);
    ASSERT_ON_ERROR(status, WLAN_ERROR);

    status = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, SL_WLAN_CONNECTION_POLICY(0, 0, 0, 0), &Policy, 1);
    ASSERT_ON_ERROR(status, WLAN_ERROR);

    status = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(status, DEVICE_ERROR);

    isActiveNwp = 0;

    /* Restart the NWP */
    status = sl_Start(0, 0 ,0);
    if(status != ROLE_STA)
    {
        ASSERT_ON_ERROR(-1, DEVICE_ERROR);
    }

    isActiveNwp = 1;

    return 0;
}

/* Scanning Task
 * Performs a scan for wireless networks.
 * Outputs each network entry found to COSMOS via the downlink
 */
void radioTool_ScanTask(UArg arg0, UArg arg1) {
    int16_t i;
    int16_t ret;
    int16_t triggeredScanTrials = 0;
    Queue_Elem * elem;
    MsgObj * msg;
    SlWlanNetworkEntry_t netEntries[10];

    if (!radioTool_IsActiveNwp()) {
        if(ConfigureSimpleLinkToDefaultState() < 0)
            return;
    }

    /* Get scan results from NWP - results would be placed inside the provided buffer */
    ret = sl_WlanGetNetworkList(0,10,&netEntries[0]);
    /* If scan policy isn't set, invoking 'sl_WlanGetNetworkList()' for the first time triggers 'one shot' scan.
     * The scan parameters would be according to the system persistent settings on enabled channels.
     * For more information, see: <simplelink user guide, page: pr.>
     */
    if(SL_ERROR_WLAN_GET_NETWORK_LIST_EAGAIN == ret)
    {
        while(triggeredScanTrials < 10)
        {
            /* We wait for one second for the NWP to complete the initiated scan and collect results */
            Task_sleep(1000 * (1000 / Clock_tickPeriod));

            /* Collect results form one-shot scans.*/
            ret = sl_WlanGetNetworkList(0,10,&netEntries[0]);
            if(ret > 0)
            {
                break;
            }
            else
            {
                /* If NWP results aren't ready, try 'MAX_SCAN_TRAILS' attempts to get results */
                triggeredScanTrials++ ;
            }
        }
    }

    /* Send the SSID packets over the downlink */
    if (ret > 0) {
        for (i = 0; i < ret; i++) {
            elem = Queue_get(freeQueue);
            msg = (MsgObj *) elem;
            if (msg) {
              msg->cosmos_id = COSMOS_SSID_ID;
              msg->size = sizeof(SlWlanNetworkEntry_t);
              memcpy(msg->data,&(netEntries[i]), sizeof(SlWlanNetworkEntry_t));
              Queue_put(msgQueue, (Queue_Elem *) elem);
              Semaphore_post(msgSemaphore);
            }
        }
    }
    operational_tasks.radio_scan = 0;
    return;
}

/* RSSI Packet sender function
 * Callback function for the stat timer. Queues
 * the current RSSI stats to the downlink msg queue.
 */
void msgSenderRxStat(Timer_Handle handle) {
    Queue_Elem * elem;
    MsgObj * msg;
    RadioTool_RX_Stats pkt_stats;

    if (operational_tasks.radio_stats == 0) {
        Timer_stop(handle);
        Timer_close(handle);
        return;
    }

    pkt_stats.TimeStamp = Clock_getTicks();
    pkt_stats.AverageRssi = (rssi_count > 0) ? rssi/rssi_count : -125;
    pkt_stats.ReceivedValidPackets = rssi_count;
    rssi_count = 0;
    rssi = 0;

    elem = Queue_get(freeQueue);
    msg = (MsgObj *) elem;
    if (msg) {
        msg->cosmos_id = COSMOS_RADIO_ID;
        msg->size = sizeof(RadioTool_RX_Stats);
        memcpy(msg->data,&(pkt_stats), sizeof(RadioTool_RX_Stats));
        Queue_put(msgQueue, (Queue_Elem *) elem);
        Semaphore_post(msgSemaphore);
    }
}

/* Setup the stat timer
 * Simple 1 seconds period for sampling
 */
int16_t setupRxStatTimer(Timer_Handle handle) {
    Timer_Params    params;
    int16_t         status;
    Timer_Params_init(&params);
    params.periodUnits = Timer_PERIOD_US;
    params.period = 1000000;
    params.timerMode  = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = msgSenderRxStat;
    handle = Timer_open(Board_TIMER0, &params);
    if (handle == NULL) {
        // Timer_open() failed
        ASSERT_ON_ERROR(-1, DEVICE_ERROR);
    }
    status = Timer_start(handle);
    ASSERT_ON_ERROR(status, DEVICE_ERROR);
    return 0;
}


/* Radio stats collection task
 * Starts collection of packets for a given channel and BSSID
 * Only collects statistics on management frames
 */
void radioTool_StatsTask(UArg arg0, UArg arg1) {
    uint8_t eChannel = (uint8_t) arg0;
    uint8_t * bssid = (uint8_t *) arg1;
    int16_t size;
    SlTransceiverRxOverHead_t *transHeader;

    // Initialize the transceiver
    if (radioTool_TranscieverSetup() < 0) {
        return;
    }

    // Setup filters for the receiver
    if (radioTool_addBSSIDFilter(bssid) < 0)
        return;

    // Setup the packet timer
    rssi_count = 0;
    rssi = 0;
    Timer_Handle statTimer;
    if (setupRxStatTimer(statTimer) < 0) {
        return;
    }

    if (RadioStartRX(eChannel, 0) < 0) {
        Timer_close(statTimer);
        return;
    }

    while(operational_tasks.radio_stats) {
        /* Start RX statistics collection */
        if(radioTool_IsActiveNwp()) {
            size = sl_Recv(rxSocket, &DataFrame, RX_BUFFER_SIZE, 0);
            if (size < 0)
                continue;
            transHeader = (SlTransceiverRxOverHead_t *)DataFrame;
            rssi_count++;
            rssi += transHeader->Rssi;
        } else {
            Task_sleep(5000 * (1000 / Clock_tickPeriod));
        }
    }
    radioTool_StatsStop();
}

/* Cleanup function for stopping RX statistics */
int16_t radioTool_StatsStop() {
    int32_t ret = 0;

    ret = sl_Close(rxSocket);
    ASSERT_ON_ERROR(ret, WLAN_ERROR);
    ret = sl_Stop(SL_STOP_TIMEOUT);
    if(ret < 0)
    {
        ASSERT_ON_ERROR(ret, DEVICE_ERROR);
    }

    /* Clear the Radio On bit*/
    isActiveNwp = 0;

    /* Return SimpleLink to default state */
    return (ConfigureSimpleLinkToDefaultState());
}

/* Add the S_MAC filter from the intial start statistics command */
int8_t radioTool_addBSSIDFilter(uint8_t * bssid) {
    int16_t ret;
    SlWlanRxFilterID_t parent;
    SlWlanRxFilterOperationCommandBuff_t filterBitmap = {{0}};
    SlWlanRxFilterOperationCommandBuff_t RxFilterIdMask = {0};

    CreateFilterCmd_t CreateFilterParams;
    memset(&CreateFilterParams, 0x0, sizeof(CreateFilterParams));

    /* Remove all 64 RX filters (8*8) */
    memset(RxFilterIdMask.FilterBitmap , 0xFF, 8);

    ret = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE,
             sizeof(SlWlanRxFilterOperationCommandBuff_t), (uint8_t*)&RxFilterIdMask.FilterBitmap);
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    /* First create drop filter for all non management packets */
    CreateFilterParams.rule.Header.CompareFunc = SL_WLAN_RX_FILTER_CMP_FUNC_NOT_EQUAL_TO;
    CreateFilterParams.action.Type = SL_WLAN_RX_FILTER_ACTION_DROP;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_L1_PAYLOAD_PATTERN;
    CreateFilterParams.trigger.ConnectionState = SL_WLAN_RX_FILTER_STATE_STA_NOT_CONNECTED;
    CreateFilterParams.trigger.Role = SL_WLAN_RX_FILTER_ROLE_TRANCIEVER;
    CreateFilterParams.trigger.ParentFilterID = 0;
    CreateFilterParams.ruleType = SL_WLAN_RX_FILTER_HEADER;
    CreateFilterParams.flags = SL_WLAN_RX_FILTER_BINARY;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_FRAME_TYPE;
    CreateFilterParams.rule.Header.Args.Value.Frametype[0] = 0;
    memset(CreateFilterParams.rule.Header.Args.Mask, 0xFF, 1);
    CreateFilterParams.trigger.Counter = SL_WLAN_RX_FILTER_NO_TRIGGER_COUNTER;
    /* Add filter. Note: Filters are not enabled yet! */
    ret = sl_WlanRxFilterAdd(CreateFilterParams.ruleType,
                             CreateFilterParams.flags,
                             &(CreateFilterParams.rule),
                             &(CreateFilterParams.trigger),
                             &(CreateFilterParams.action),
                             &(CreateFilterParams.filterID));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    /* Second create pass filter for all management packets */
    memset(&CreateFilterParams, 0x0, sizeof(CreateFilterParams));
    CreateFilterParams.rule.Header.CompareFunc = SL_WLAN_RX_FILTER_CMP_FUNC_EQUAL;
    CreateFilterParams.action.Type = SL_WLAN_RX_FILTER_ACTION_NULL;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_L1_PAYLOAD_PATTERN;
    CreateFilterParams.trigger.ConnectionState = SL_WLAN_RX_FILTER_STATE_STA_NOT_CONNECTED;
    CreateFilterParams.trigger.Role = SL_WLAN_RX_FILTER_ROLE_TRANCIEVER;
    CreateFilterParams.trigger.ParentFilterID = 0;
    CreateFilterParams.ruleType = SL_WLAN_RX_FILTER_HEADER;
    CreateFilterParams.flags = SL_WLAN_RX_FILTER_BINARY;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_FRAME_TYPE;
    CreateFilterParams.rule.Header.Args.Value.Frametype[0] = 0;
    memset(CreateFilterParams.rule.Header.Args.Mask, 0xFF, 1);
    CreateFilterParams.trigger.Counter = SL_WLAN_RX_FILTER_NO_TRIGGER_COUNTER;
    /* Add filter. Note: Filters are not enabled yet! */
    ret = sl_WlanRxFilterAdd(CreateFilterParams.ruleType,
                             CreateFilterParams.flags,
                             &(CreateFilterParams.rule),
                             &(CreateFilterParams.trigger),
                             &(CreateFilterParams.action),
                             &(CreateFilterParams.filterID));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    /* Create block filter for all non beacon packets */
    parent = CreateFilterParams.filterID;
    memset(&CreateFilterParams, 0x0, sizeof(CreateFilterParams));
    CreateFilterParams.rule.Header.CompareFunc = SL_WLAN_RX_FILTER_CMP_FUNC_NOT_EQUAL_TO;
    CreateFilterParams.action.Type = SL_WLAN_RX_FILTER_ACTION_DROP;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_L1_PAYLOAD_PATTERN;
    CreateFilterParams.trigger.ConnectionState = SL_WLAN_RX_FILTER_STATE_STA_NOT_CONNECTED;
    CreateFilterParams.trigger.Role = SL_WLAN_RX_FILTER_ROLE_TRANCIEVER;
    CreateFilterParams.trigger.ParentFilterID = parent;
    CreateFilterParams.ruleType = SL_WLAN_RX_FILTER_HEADER;
    CreateFilterParams.flags = SL_WLAN_RX_FILTER_BINARY;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_FRAME_SUBTYPE;
    CreateFilterParams.rule.Header.Args.Value.FrameSubtype[0] = 0x80;
    memset(CreateFilterParams.rule.Header.Args.Mask, 0xFF, 1);
    CreateFilterParams.trigger.Counter = SL_WLAN_RX_FILTER_NO_TRIGGER_COUNTER;
    /* Add filter. Note: Filters are not enabled yet! */
    ret = sl_WlanRxFilterAdd(CreateFilterParams.ruleType,
                             CreateFilterParams.flags,
                             &(CreateFilterParams.rule),
                             &(CreateFilterParams.trigger),
                             &(CreateFilterParams.action),
                             &(CreateFilterParams.filterID));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    /* Create pass filter for all beacon packets */
    memset(&CreateFilterParams, 0x0, sizeof(CreateFilterParams));
    CreateFilterParams.rule.Header.CompareFunc = SL_WLAN_RX_FILTER_CMP_FUNC_EQUAL;
    CreateFilterParams.action.Type = SL_WLAN_RX_FILTER_ACTION_NULL;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_L1_PAYLOAD_PATTERN;
    CreateFilterParams.trigger.ConnectionState = SL_WLAN_RX_FILTER_STATE_STA_NOT_CONNECTED;
    CreateFilterParams.trigger.Role = SL_WLAN_RX_FILTER_ROLE_TRANCIEVER;
    CreateFilterParams.trigger.ParentFilterID = 0;
    CreateFilterParams.ruleType = SL_WLAN_RX_FILTER_HEADER;
    CreateFilterParams.flags = SL_WLAN_RX_FILTER_BINARY;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_FRAME_SUBTYPE;
    CreateFilterParams.rule.Header.Args.Value.FrameSubtype[0] = 0x80;
    memset(CreateFilterParams.rule.Header.Args.Mask, 0xFF, 1);
    CreateFilterParams.trigger.Counter = SL_WLAN_RX_FILTER_NO_TRIGGER_COUNTER;
    /* Add filter. Note: Filters are not enabled yet! */
    ret = sl_WlanRxFilterAdd(CreateFilterParams.ruleType,
                             CreateFilterParams.flags,
                             &(CreateFilterParams.rule),
                             &(CreateFilterParams.trigger),
                             &(CreateFilterParams.action),
                             &(CreateFilterParams.filterID));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    /* Third create Source MAC filter */
    parent = CreateFilterParams.filterID;
    memset(&CreateFilterParams, 0x0, sizeof(CreateFilterParams));
    CreateFilterParams.rule.Header.CompareFunc = SL_WLAN_RX_FILTER_CMP_FUNC_NOT_EQUAL_TO;
    CreateFilterParams.action.Type = SL_WLAN_RX_FILTER_ACTION_DROP;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_L1_PAYLOAD_PATTERN;
    CreateFilterParams.trigger.ConnectionState = SL_WLAN_RX_FILTER_STATE_STA_NOT_CONNECTED;
    CreateFilterParams.trigger.Role = SL_WLAN_RX_FILTER_ROLE_TRANCIEVER;
    //CreateFilterParams.trigger.ParentFilterID = parent;
    CreateFilterParams.trigger.ParentFilterID = 0;
    CreateFilterParams.ruleType = SL_WLAN_RX_FILTER_HEADER;
    CreateFilterParams.flags = SL_WLAN_RX_FILTER_BINARY;
    CreateFilterParams.rule.Header.Field = SL_WLAN_RX_FILTER_HFIELD_MAC_SRC_ADDR;
    memcpy(CreateFilterParams.rule.Header.Args.Value.Mac[0], bssid, SL_WLAN_BSSID_LENGTH);
    memset(CreateFilterParams.rule.Header.Args.Mask, 0xFF, SL_WLAN_BSSID_LENGTH);
    CreateFilterParams.trigger.Counter = SL_WLAN_RX_FILTER_NO_TRIGGER_COUNTER;
    /* Add filter. Note: Filters are not enabled yet! */
    ret = sl_WlanRxFilterAdd(CreateFilterParams.ruleType,
                             CreateFilterParams.flags,
                             &(CreateFilterParams.rule),
                             &(CreateFilterParams.trigger),
                             &(CreateFilterParams.action),
                             &(CreateFilterParams.filterID));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);


    memset(&filterBitmap.FilterBitmap, 0xFF, 16);
    ret = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_STATE,
          sizeof(SlWlanRxFilterOperationCommandBuff_t), (uint8_t*)&filterBitmap);
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    return 0;
}


// Simple Link Event Handlers
/*!
 *  \brief      This function handles general events
 *  \param[in]  pDevEvent - Pointer to stucture containing general event info
 *  \return     None
 */
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    // Unused in this application
}

/*!
 *  \brief      This function handles WLAN async events
 *  \param[in]  pWlanEvent - Pointer to the structure containg WLAN event info
 *  \return     None
 */
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    /* Unused in this application */
}

/*!
 *  \brief      This function handles socket events indication
 *  \param[in]  pSock - Pointer to the stucture containing socket event info
 *  \return     None
 */
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    // Unused in this application
}

/*!
 *  \brief      This function handles network events such as IP acquisition, IP leased, IP released etc.
 *  \param[in]  pNetAppEvent - Pointer to the structure containing acquired IP
 *  \return     None
 */

void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t   *pSlTriggerEvent)
{



}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    /* Unused in this application */
}

/*!
 *  \brief      This function handles resource request
 *  \param[in]  pNetAppRequest - Contains the resource requests
 *  \param[in]  pNetAppResponse - Should be filled by the user with the relevant response information
 *  \return     None
 */
void SimpleLinkNetAppRequestHandler(SlNetAppRequest_t  *pNetAppRequest,
                                    SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

/*!
 *  \brief      This function gets triggered when HTTP Server receives
 *              application defined GET and POST HTTP tokens.
 *  \param[in]  pHttpServerEvent - Pointer indicating http server event
 *  \param[in]  pHttpServerResponse - Pointer indicating http server response
 *  \return     None
 */
/*
void SimpleLinkHttpServerCallback(SlNetAppHttpServerEvent_t *pSlHttpServerEvent,
                                  SlNetAppHttpServerResponse_t *pSlHttpServerResponse)
{

}*/

void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                      SlNetAppHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
}

/*!
 *  \brief      This function handles resource request
 *  \param[in]  pFatalErrorEvent - Contains the fatal error data
 *  \return     None
 */
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    /* Unused in this application */
}

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application */
}


void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

/* Default state configuration for the NWP */
int16_t ConfigureSimpleLinkToDefaultState()
{
     uint8_t                              ucConfigOpt;
     uint8_t                              ucPower;
     int32_t                              RetVal = -1;
     int32_t                              Mode = -1;
     uint32_t                             IfBitmap = 0;
     SlWlanScanParamCommand_t             ScanDefault = {0};
     SlWlanRxFilterOperationCommandBuff_t RxFilterIdMask = {0};

     /* Turn NWP on */
     Mode = sl_Start(0, 0, 0);
     ASSERT_ON_ERROR(Mode, DEVICE_ERROR);

     if(Mode != ROLE_STA)
     {
         /* Set NWP role as STA */
         Mode = sl_WlanSetMode(ROLE_STA);
         ASSERT_ON_ERROR(Mode, WLAN_ERROR);

         /* For changes to take affect, we restart the NWP */
         RetVal = sl_Stop(SL_STOP_TIMEOUT);
         ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

         Mode = sl_Start(0, 0, 0);
         ASSERT_ON_ERROR(Mode, DEVICE_ERROR);
     }

     if(Mode != ROLE_STA)
     {
         ASSERT_ON_ERROR(-1, DEFAULT_ERROR);
     }

     /* Set policy to auto only */
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, SL_WLAN_CONNECTION_POLICY(1,0,0,0), NULL ,0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Disable Auto Provisioning */
     RetVal = sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, NULL, 0x0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     RetVal = sl_WlanProfileDel(0xFF);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* enable DHCP client */
     RetVal = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0, 0);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Disable ipv6 */
     IfBitmap = !(SL_NETCFG_IF_IPV6_STA_LOCAL | SL_NETCFG_IF_IPV6_STA_GLOBAL);

     RetVal = sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE, sizeof(IfBitmap),(const unsigned char *)&IfBitmap);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Configure scan parameters to default */
     ScanDefault.ChannelsMask = CHANNEL_MASK_ALL;
     ScanDefault.RssiThreshold = RSSI_TH_MAX;

     RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS, sizeof(ScanDefault), (uint8_t *)&ScanDefault);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Disable scans */
     ucConfigOpt = SL_WLAN_SCAN_POLICY(0, 0);
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN , ucConfigOpt, NULL, 0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set TX power lvl to max */
     ucPower = 0;
     RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (uint8_t *)&ucPower);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set NWP Power policy to 'normal' */
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, NULL, 0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Unregister mDNS services */
     RetVal = sl_NetAppMDNSUnRegisterService(0, 0, 0);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Remove all 64 RX filters (8*8) */
     memset(RxFilterIdMask.FilterBitmap , 0xFF, 8);

     RetVal = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE, sizeof(SlWlanRxFilterOperationCommandBuff_t),(uint8_t *)&RxFilterIdMask);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set NWP role as STA */
     RetVal = sl_WlanSetMode(ROLE_STA);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* For changes to take affect, we restart the NWP */
     RetVal = sl_Stop(0xFF);
     ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

     Mode = sl_Start(0, 0, 0);
     ASSERT_ON_ERROR(Mode, DEVICE_ERROR);

     if(ROLE_STA != Mode)
     {
         ASSERT_ON_ERROR(-1, DEFAULT_ERROR);
     }
     else
     {
         isActiveNwp = 1;
     }

     return 0;
}


/* Setup the NWP to receive on the desired channel
 * Receives for an unlimited amount of time
 */
int16_t RadioStartRX(uint8_t eChannel, uint8_t eEnableACKs) {
    int16_t             ret = 0;
    SlTimeval_t         timeVal = {0};

    timeVal.tv_sec = RX_RECV_TIMEOUT_SEC;
    timeVal.tv_usec = 500000;

    ret = sl_WlanRxStatStart();
    ASSERT_ON_ERROR(ret, DEVICE_ERROR);

    rxSocket = sl_Socket(SL_AF_RF, SL_SOCK_RAW, eChannel);
    ASSERT_ON_ERROR(rxSocket, RADIO_TOOL_ERROR_RX_CREATING_RAW_SOCKET);


    ret = sl_SetSockOpt(rxSocket, SL_SOL_SOCKET, SL_SO_RCVTIMEO, &timeVal, sizeof(timeVal));
    ASSERT_ON_ERROR(ret, WLAN_ERROR);

    ret = sl_Recv(rxSocket, &DataFrame, RX_BUFFER_SIZE, 0);

    return 0;
}
