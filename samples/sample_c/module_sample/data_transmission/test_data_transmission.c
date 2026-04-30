/**
 ********************************************************************
 * @file    test_data_transmission.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "test_data_transmission.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_low_speed_data_channel.h"
#include "dji_high_speed_data_channel.h"
#include "dji_aircraft_info.h"
#include "widget_interaction_test/test_widget_interaction.h"
#include "dji_fc_subscription.h"
#include <string.h>
#include <time.h>

/* Private constants ---------------------------------------------------------*/
#define DATA_TRANSMISSION_TASK_FREQ         (1)
#define DATA_TRANSMISSION_TASK_STACK_SIZE   (2048)
#define DATA_TRANSMISSION_CSV_BASE_PATH     "/home/rsp/drone_air_system/data_from_drone/real_time_labels_"
#define DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN (256)
#define DATA_TRANSMISSION_CSV_QUEUE_SIZE    (32)

/* Private types -------------------------------------------------------------*/
typedef struct {
    char text[DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN];
    uint16_t len;
} T_DjiDataTransmissionCsvEntry;

typedef struct {
    E_DjiChannelAddress channelAddress;
    T_DjiReturnCode (*callback)(const uint8_t *data, uint16_t len);
} ChannelCallbackEntry;

/* Private functions declaration ---------------------------------------------*/
static void *UserDataTransmission_Task(void *arg);
static T_DjiReturnCode ReceiveDataFromMobile(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromCloud(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromExtensionPort(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload1(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload2(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload3(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload4(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload5(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload6(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload7(const uint8_t *data, uint16_t len);
static T_DjiReturnCode ReceiveDataFromPayload8(const uint8_t *data, uint16_t len);
static T_DjiReturnCode DataTransmission_InitCsvLogger(void);
static void DataTransmission_ProcessCsvQueue(void);
static void DataTransmission_EnqueueCsvLine(const uint8_t *data, uint16_t len);
static T_DjiReturnCode DataTransmission_WriteCsvEntry(const char *text);
static void DataTransmission_EscapeCsvField(const char *src, char *dst, size_t dstSize);
/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userDataTransmissionThread;
static T_DjiAircraftInfoBaseInfo s_aircraftInfoBaseInfo;
static T_DjiMutexHandle s_csvQueueMutex = {0};
static T_DjiDataTransmissionCsvEntry s_csvQueue[DATA_TRANSMISSION_CSV_QUEUE_SIZE];
static uint16_t s_csvQueueHead = 0;
static uint16_t s_csvQueueTail = 0;
static uint16_t s_csvQueueCount = 0;
static char s_csvPath[256];

static const ChannelCallbackEntry g_channelCallbacks[] = {
    {DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1, ReceiveDataFromPayload1},
    {DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO2, ReceiveDataFromPayload2},
    {DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO3, ReceiveDataFromPayload3},
    {DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO4, ReceiveDataFromPayload4},
    {DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO5, ReceiveDataFromPayload5},
    {DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO6, ReceiveDataFromPayload6},
    {DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO7, ReceiveDataFromPayload7},
    {DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO8, ReceiveDataFromPayload8},
};
/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_DataTransmissionStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiChannelAddress channelAddress;
    const T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidthProportionOfHighspeedChannel =
        {10, 60, 30};
    char ipAddr[DJI_IP_ADDR_STR_SIZE_MAX];
    uint16_t port;

    djiStat = DjiLowSpeedDataChannel_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data transmission module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiAircraftInfo_GetBaseInfo(&s_aircraftInfoBaseInfo);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    djiStat = DataTransmission_InitCsvLogger();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init csv logger error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
    djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromMobile);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("register receive data from mobile error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

        if ((s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3D  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3TD ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M4D  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M4TD) &&
            s_aircraftInfoBaseInfo.mountPositionType != DJI_MOUNT_POSITION_TYPE_MANIFOLD3_ONBOARD) {
        channelAddress = DJI_CHANNEL_ADDRESS_CLOUD_API;
        djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromCloud);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("register receive data from cloud error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    if (s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M400) {
        uint8_t Channel_Callback_count = (sizeof(g_channelCallbacks) / sizeof(g_channelCallbacks[0]));
        for (uint8_t i = 0; i < Channel_Callback_count; i++) {
            const ChannelCallbackEntry *entry = &g_channelCallbacks[i];
            T_DjiReturnCode djiStat = 
                            DjiLowSpeedDataChannel_RegRecvDataCallback(entry->channelAddress, entry->callback);

            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("register receive data from channel %d error.", entry->channelAddress);
                return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            }
        }
        USER_LOG_INFO("Only supports small data transmission between PSDK and MSDK.");
    } else if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 ||
        s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 ||
        s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3) {
        channelAddress = DJI_CHANNEL_ADDRESS_EXTENSION_PORT;
        djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromExtensionPort);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("register receive data from extension port error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }

        djiStat = DjiHighSpeedDataChannel_SetBandwidthProportion(bandwidthProportionOfHighspeedChannel);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set data channel bandwidth width proportion error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }

        djiStat = DjiHighSpeedDataChannel_GetDataStreamRemoteAddress(ipAddr, &port);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_DEBUG("Get data stream remote address: %s, port: %d", ipAddr, port);
        } else {
            USER_LOG_ERROR("get data stream remote address error.");
        }

    } else if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_EXTENSION_PORT
                || DJI_MOUNT_POSITION_EXTENSION_LITE_PORT == s_aircraftInfoBaseInfo.mountPosition) {
        channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1;
        djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromPayload1);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("register receive data from payload NO1 error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }

        channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO2;
        djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromPayload2);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("register receive data from payload NO1 error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }

        channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO3;
        djiStat = DjiLowSpeedDataChannel_RegRecvDataCallback(channelAddress, ReceiveDataFromPayload3);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("register receive data from payload NO1 error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }

    if (osalHandler->TaskCreate("user_transmission_task", UserDataTransmission_Task,
                                DATA_TRANSMISSION_TASK_STACK_SIZE, NULL, &s_userDataTransmissionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data transmission task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_DataTransmissionStopService(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;

    if (osalHandler->TaskDestroy(s_userDataTransmissionThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data transmission task destroy error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    DataTransmission_ProcessCsvQueue();

    if (osalHandler->MutexDestroy(s_csvQueueMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("destroy csv queue mutex failed.");
    }

    returnCode = DjiLowSpeedDataChannel_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("deinit data transmission module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static bool ConvertGpsDateTimeToLocalTimeString(uint32_t gpsDate, uint32_t gpsTime,
                                                  const char *format, char *buffer, size_t bufferSize)
{
    if (buffer == NULL || format == NULL || bufferSize == 0 || gpsDate == 0 || gpsTime == 0) {
        return false;
    }

    uint32_t year = gpsDate / 10000;
    uint32_t month = (gpsDate / 100) % 100;
    uint32_t day = gpsDate % 100;
    uint32_t hour = gpsTime / 10000;
    uint32_t minute = (gpsTime / 100) % 100;
    uint32_t second = gpsTime % 100;

    struct tm utcTime = {0};
    utcTime.tm_year = (int)year - 1900;
    utcTime.tm_mon = (int)month - 1;
    utcTime.tm_mday = (int)day;
    utcTime.tm_hour = (int)hour;
    utcTime.tm_min = (int)minute;
    utcTime.tm_sec = (int)second;
    utcTime.tm_isdst = -1;

    time_t rawTime = timegm(&utcTime);
    if (rawTime == (time_t)-1) {
        return false;
    }

    struct tm localTime = {0};
    if (localtime_r(&rawTime, &localTime) == NULL) {
        return false;
    }

    if (strftime(buffer, bufferSize, format, &localTime) == 0) {
        return false;
    }

    return true;
}

static T_DjiReturnCode DataTransmission_InitCsvLogger(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    if (osalHandler == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (osalHandler->MutexCreate(&s_csvQueueMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("create csv queue mutex failed.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // Generate timestamped CSV path using GPS date/time
    T_DjiFcSubscriptionGpsDate gpsDate = 0;
    T_DjiFcSubscriptionGpsTime gpsTime = 0;
    T_DjiDataTimestamp timestamp = {0};
    char timestampPart[32] = {0};
    bool haveGpsTime = false;
    uint32_t tries = 0;

    while (tries < 10) {
        T_DjiReturnCode pollStat = DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, (uint8_t *)&gpsDate,
            sizeof(T_DjiFcSubscriptionGpsDate), &timestamp);
        if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsDate != 0) {
            pollStat = DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, (uint8_t *)&gpsTime,
                sizeof(T_DjiFcSubscriptionGpsTime), &timestamp);
            if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsTime != 0) {
                if (ConvertGpsDateTimeToLocalTimeString(gpsDate, gpsTime,
                                                       "%Y%m%d_%H%M%S", timestampPart,
                                                       sizeof(timestampPart))) {
                    haveGpsTime = true;
                    break;
                }
            }
        }

        osalHandler->TaskSleepMs(300);
        tries++;
    }

    if (!haveGpsTime) {
        USER_LOG_ERROR("unable to get valid GPS datetime for transmission filename, using Pi time");
        time_t now = time(NULL);
        struct tm *timeInfo = localtime(&now);
        if (timeInfo != NULL) {
            strftime(s_csvPath, sizeof(s_csvPath), DATA_TRANSMISSION_CSV_BASE_PATH "%Y%m%d_%H%M%S.csv",
                     timeInfo);
        } else {
            snprintf(s_csvPath, sizeof(s_csvPath), DATA_TRANSMISSION_CSV_BASE_PATH "unknown_time.csv");
        }
    } else {
        snprintf(s_csvPath, sizeof(s_csvPath), DATA_TRANSMISSION_CSV_BASE_PATH "%s.csv", timestampPart);
    }

    s_csvQueueHead = 0;
    s_csvQueueTail = 0;
    s_csvQueueCount = 0;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void DataTransmission_EscapeCsvField(const char *src, char *dst, size_t dstSize)
{
    size_t srcIndex = 0;
    size_t dstIndex = 0;

    if (dstSize < 3) {
        return;
    }

    dst[dstIndex++] = '"';
    while (src[srcIndex] != '\0' && dstIndex + 2 < dstSize) {
        char ch = src[srcIndex++];
        if (ch == '"') {
            if (dstIndex + 3 >= dstSize) {
                break;
            }
            dst[dstIndex++] = '"';
            dst[dstIndex++] = '"';
        } else if (ch == '\r' || ch == '\n') {
            dst[dstIndex++] = ' ';
        } else {
            dst[dstIndex++] = ch;
        }
    }

    if (dstIndex + 1 < dstSize) {
        dst[dstIndex++] = '"';
    }

    if (dstIndex < dstSize) {
        dst[dstIndex] = '\0';
    } else {
        dst[dstSize - 1] = '\0';
    }
}

static T_DjiReturnCode DataTransmission_WriteCsvEntry(const char *text)
{
    FILE *csvFile = fopen(s_csvPath, "a+");
    if (csvFile == NULL) {
        USER_LOG_ERROR("open csv file failed.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (fseek(csvFile, 0, SEEK_END) != 0) {
        USER_LOG_ERROR("seek csv file end failed.");
        fclose(csvFile);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (ftell(csvFile) == 0) {
        fprintf(csvFile, "timestamp,text\n");
    }

    char timestamp[32] = "";
    T_DjiFcSubscriptionGpsDate gpsDate = 0;
    T_DjiFcSubscriptionGpsTime gpsTime = 0;
    T_DjiDataTimestamp ts = {0};
    bool haveGpsTime = false;

    T_DjiReturnCode pollStat = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, (uint8_t *)&gpsDate,
        sizeof(T_DjiFcSubscriptionGpsDate), &ts);
    if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsDate != 0) {
        pollStat = DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, (uint8_t *)&gpsTime,
            sizeof(T_DjiFcSubscriptionGpsTime), &ts);
        if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsTime != 0) {
            haveGpsTime = ConvertGpsDateTimeToLocalTimeString(gpsDate, gpsTime,
                                                             "%Y-%m-%d %H:%M:%S",
                                                             timestamp, sizeof(timestamp));
        }
    }

    if (!haveGpsTime) {
        USER_LOG_ERROR("unable to get valid GPS datetime for row timestamp, using Pi time");
        time_t now = time(NULL);
        struct tm *timeInfo = localtime(&now);
        if (timeInfo != NULL) {
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeInfo);
        }
    }

    char escapedText[DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN * 2 + 3];
    DataTransmission_EscapeCsvField(text, escapedText, sizeof(escapedText));
    fprintf(csvFile, "%s,%s\n", timestamp, escapedText);
    fflush(csvFile);
    fclose(csvFile);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void DataTransmission_ProcessCsvQueue(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if (osalHandler == NULL) {
        return;
    }

    while (1) {
        if (osalHandler->MutexLock(s_csvQueueMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return;
        }

        if (s_csvQueueCount == 0) {
            osalHandler->MutexUnlock(s_csvQueueMutex);
            break;
        }

        char text[DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN];
        uint16_t entryLen = s_csvQueue[s_csvQueueHead].len;
        memcpy(text, s_csvQueue[s_csvQueueHead].text, entryLen);
        text[entryLen] = '\0';

        s_csvQueueHead = (s_csvQueueHead + 1) % DATA_TRANSMISSION_CSV_QUEUE_SIZE;
        s_csvQueueCount--;
        osalHandler->MutexUnlock(s_csvQueueMutex);

        if (DataTransmission_WriteCsvEntry(text) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("write csv entry failed.");
        }
    }
}

static void DataTransmission_EnqueueCsvLine(const uint8_t *data, uint16_t len)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if (osalHandler == NULL || data == NULL || len == 0) {
        return;
    }

    if (osalHandler->MutexLock(s_csvQueueMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return;
    }

    if (s_csvQueueCount >= DATA_TRANSMISSION_CSV_QUEUE_SIZE) {
        osalHandler->MutexUnlock(s_csvQueueMutex);
        USER_LOG_ERROR("csv queue full, drop incoming text.");
        return;
    }

    uint16_t copyLen = len;
    if (copyLen >= DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN) {
        copyLen = DATA_TRANSMISSION_CSV_ENTRY_MAX_LEN - 1;
    }

    memcpy(s_csvQueue[s_csvQueueTail].text, data, copyLen);
    s_csvQueue[s_csvQueueTail].text[copyLen] = '\0';
    s_csvQueue[s_csvQueueTail].len = copyLen;
    s_csvQueueTail = (s_csvQueueTail + 1) % DATA_TRANSMISSION_CSV_QUEUE_SIZE;
    s_csvQueueCount++;
    osalHandler->MutexUnlock(s_csvQueueMutex);
}

static void *UserDataTransmission_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    const uint8_t dataToBeSent[] = "DJI Data Transmission Test Data.";
    T_DjiDataChannelState state = {0};
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiChannelAddress channelAddress;

    USER_UTIL_UNUSED(arg);

    while (1) {
        osalHandler->TaskSleepMs(1000 / DATA_TRANSMISSION_TASK_FREQ);

        DataTransmission_ProcessCsvQueue();

        channelAddress = DJI_CHANNEL_ADDRESS_MASTER_RC_APP;
        djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            USER_LOG_ERROR("send data to mobile error.");

        djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_DEBUG(
                "send to mobile state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                state.busyState);
        } else {
            USER_LOG_ERROR("get send to mobile channel state error.");
        }

        if (s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3D  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3TD ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M4D  ||
            s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M4TD ) {
            channelAddress = DJI_CHANNEL_ADDRESS_CLOUD_API;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to cloud error.");

            djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
            if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_DEBUG(
                    "send to cloud state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                    state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                    state.busyState);
            } else {
                USER_LOG_ERROR("get send to cloud channel state error.");
            }
        }

        if (s_aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M400){
            /*!< Code Example for Data Transmission Between PSDK and PSDK. */
            /*!< Only support Psdk on M400. */
            /*
            channelAddress = DJI_CHANNEL_ADDRESS_EXTENSION_PORT_V2_NO1;
            djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
            if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                USER_LOG_ERROR("send data to psdk error port1.");

            djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
            if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_DEBUG(
                    "send to psdk state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                    state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                    state.busyState);
            } else {
                USER_LOG_ERROR("get send to psdk channel state error.");
            }
            */
        }else{
            if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 ||
                s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 ||
                s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3) {
                channelAddress = DJI_CHANNEL_ADDRESS_EXTENSION_PORT;
                djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
                if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                    USER_LOG_ERROR("send data to extension port error.");

                djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
                if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_DEBUG(
                        "send to extension port state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                        state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                        state.busyState);
                } else {
                    USER_LOG_ERROR("get send to extension port channel state error.");
                }

                if (DjiPlatform_GetSocketHandler() != NULL) {
    #ifdef SYSTEM_ARCH_LINUX
                    djiStat = DjiHighSpeedDataChannel_SendDataStreamData(dataToBeSent, sizeof(dataToBeSent));
                    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                        USER_LOG_ERROR("send data to data stream error.");

                    djiStat = DjiHighSpeedDataChannel_GetDataStreamState(&state);
                    if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_DEBUG(
                            "data stream state: realtimeBandwidthLimit: %d, realtimeBandwidthBeforeFlowController: %d, busyState: %d.",
                            state.realtimeBandwidthLimit, state.realtimeBandwidthBeforeFlowController, state.busyState);
                    } else {
                        USER_LOG_ERROR("get data stream state error.");
                    }
    #endif
                }
            } else if (s_aircraftInfoBaseInfo.mountPosition == DJI_MOUNT_POSITION_EXTENSION_PORT) {
                channelAddress = DJI_CHANNEL_ADDRESS_PAYLOAD_PORT_NO1;
                djiStat = DjiLowSpeedDataChannel_SendData(channelAddress, dataToBeSent, sizeof(dataToBeSent));
                if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                    USER_LOG_ERROR("send data to extension port error.");

                djiStat = DjiLowSpeedDataChannel_GetSendDataState(channelAddress, &state);
                if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_DEBUG(
                        "send to extension port state: realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController: %d, busyState: %d.",
                        state.realtimeBandwidthBeforeFlowController, state.realtimeBandwidthAfterFlowController,
                        state.busyState);
                } else {
                    USER_LOG_ERROR("get send to extension port channel state error.");
                }
            }
        }
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode ReceiveDataFromMobile(const uint8_t *data, uint16_t len)
{
    char *printData = NULL;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    printData = osalHandler->Malloc(len + 1);
    if (printData == NULL) {
        USER_LOG_ERROR("malloc memory for printData fail.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    strncpy(printData, (const char *) data, len);
    printData[len] = '\0';
    USER_LOG_INFO("receive data from mobile: %s, len:%d.", printData, len);
    DjiTest_WidgetLogAppend("receive data: %s, len:%d.", printData, len);
    DataTransmission_EnqueueCsvLine(data, len);

    osalHandler->Free(printData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode ReceiveDataFromCloud(const uint8_t *data, uint16_t len)
{
    char *printData = NULL;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    printData = osalHandler->Malloc(len + 1);
    if (printData == NULL) {
        USER_LOG_ERROR("malloc memory for printData fail.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    strncpy(printData, (const char *) data, len);
    printData[len] = '\0';
    USER_LOG_INFO("receive data from cloud: %s, len:%d.", printData, len);
    DjiTest_WidgetLogAppend("receive data: %s, len:%d.", printData, len);

    osalHandler->Free(printData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode ReceiveDataFromExtensionPort(const uint8_t *data, uint16_t len)
{
    char *printData = NULL;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    printData = osalHandler->Malloc(len + 1);
    if (printData == NULL) {
        USER_LOG_ERROR("malloc memory for printData fail.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    strncpy(printData, (const char *) data, len);
    printData[len] = '\0';
    USER_LOG_INFO("receive data from extension port: %s, len:%d.", printData, len);
    DjiTest_WidgetLogAppend("receive data: %s, len:%d.", printData, len);

    osalHandler->Free(printData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode ReceiveDataFromPayload(const uint8_t *data, uint16_t len)
{
    char *printData = NULL;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    printData = osalHandler->Malloc(len + 1);
    if (printData == NULL) {
        USER_LOG_ERROR("malloc memory for printData fail.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    strncpy(printData, (const char *) data, len);
    printData[len] = '\0';
    USER_LOG_INFO("receive data from payload port: %s, len:%d.", printData, len);
    DjiTest_WidgetLogAppend("receive data: %s, len:%d.", printData, len);

    osalHandler->Free(printData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode ReceiveDataFromPayload1(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 1");
    return ReceiveDataFromPayload(data, len);
}

static T_DjiReturnCode ReceiveDataFromPayload2(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 2");
    return ReceiveDataFromPayload(data, len);
}

static T_DjiReturnCode ReceiveDataFromPayload3(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 3");
    return ReceiveDataFromPayload(data, len);
}

static T_DjiReturnCode ReceiveDataFromPayload4(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 4");
    return ReceiveDataFromPayload(data, len);
}
static T_DjiReturnCode ReceiveDataFromPayload5(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 5");
    return ReceiveDataFromPayload(data, len);
}

static T_DjiReturnCode ReceiveDataFromPayload6(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 6");
    return ReceiveDataFromPayload(data, len);
}

static T_DjiReturnCode ReceiveDataFromPayload7(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 7");
    return ReceiveDataFromPayload(data, len);
}
static T_DjiReturnCode ReceiveDataFromPayload8(const uint8_t *data, uint16_t len)
{
    USER_LOG_INFO("Receive from payload on port 8");
    return ReceiveDataFromPayload(data, len);
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
