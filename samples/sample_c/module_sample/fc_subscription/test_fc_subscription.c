/**
 ********************************************************************
 * @file    test_fc_subscription.c
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
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_time_sync.h"
#include "widget_interaction_test/test_widget_interaction.h"

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (2048)
#define FC_SUBSCRIPTION_CSV_PATH_MAX      (256)

/*
 * GPS position unit conversions:
 *   gpsPosition.x = longitude  [1e-7 degrees]
 *   gpsPosition.y = latitude   [1e-7 degrees]
 *   gpsPosition.z = altitude   [mm above sea level]
 */
#define GPS_DEG_SCALE   (1.0e-7)   /* convert raw integer → decimal degrees */
#define GPS_ALT_SCALE   (1.0e-3)   /* convert mm → metres                   */

/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);
static void WriteCsvRow(FILE *fp,
                        const char *timestamp,
                        double lat, double lon, double alt,
                        double pitch, double roll, double yaw,
                        int16_t compassX, int16_t compassY, int16_t compassZ,
                        double altitudeAglM,
                        double velX, double velY, double velZ,
                        double accelRawX, double accelRawY, double accelRawZ,
                        double gyroRawX, double gyroRawY, double gyroRawZ,
                        const int16_t escSpeed[8],
                        double baroAltitudeM);
static bool ConvertGpsDateTimeToLocalTimeString(uint32_t gpsDate, uint32_t gpsTime,
                                                const char *format, char *buffer,
                                                size_t bufferSize);
static bool CsvNeedsHeader(const char *path);

static bool ConvertGpsDateTimeToLocalTimeString(uint32_t gpsDate, uint32_t gpsTime,
                                                const char *format, char *buffer,
                                                size_t bufferSize)
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

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool            s_userFcSubscriptionDataShow = false;
static uint8_t         s_totalSatelliteNumberUsed   = 0;
static uint32_t        s_userFcSubscriptionDataCnt  = 0;

/* Shared euler angles – written by quaternion callback, read by task.
 * Access is protected by s_eulerMutex. */
static T_DjiMutexHandle s_eulerMutex = {0};
static double           s_pitch      = 0.0;
static double           s_roll       = 0.0;
static double           s_yaw        = 0.0;

/* CSV output path (set via DjiTest_FcSubscriptionSetCsvOutputPath) */
static bool s_isCsvOutputPathConfigured              = false;
static char s_csvOutputPath[FC_SUBSCRIPTION_CSV_PATH_MAX] = {0};

/* Exported functions definition ---------------------------------------------*/

T_DjiReturnCode DjiTest_FcSubscriptionSetCsvOutputPath(const char *path)
{
    if (path == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memset(s_csvOutputPath, 0, sizeof(s_csvOutputPath));
    memcpy(s_csvOutputPath, path,
           USER_UTIL_MIN(strlen(path), sizeof(s_csvOutputPath) - 1));
    s_isCsvOutputPathConfigured = true;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionStartService(void)
{
    T_DjiReturnCode   djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    /* Create mutex that protects the shared euler-angle variables */
    djiStat = osalHandler->MutexCreate(&s_eulerMutex);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create euler mutex error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    /* Quaternion at 50 Hz — callback converts to pitch/roll/yaw */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic quaternion success.");
    }

    /* Velocity at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic velocity success.");
    }

    /* GPS position at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps position success.");
    }

    /* GPS details at 1 Hz — polled in task (satellite count) */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps details success.");
    }

    /* GPS date at 1 Hz — polled in task for drone timestamp */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps date error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps date success.");
    }

    /* GPS time at 1 Hz — polled in task for drone timestamp */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps time error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps time success.");
    }

    /* Compass (magnetometer) at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic compass error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic compass success.");
    }

    /* Fused altitude (ASL) at 1 Hz — combined with home point altitude to get AGL */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude fused success.");
    }

    /* ASL altitude recorded at last takeoff at 1 Hz — used to derive AGL */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of homepoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude of homepoint success.");
    }

    /* Barometric altitude at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude barometer error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude barometer success.");
    }

    /* Raw IMU accelerometer at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration raw error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic acceleration raw success.");
    }

    /* Raw IMU gyroscope at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic angular rate raw error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic angular rate raw success.");
    }

    /* ESC/motor telemetry at 1 Hz — polled in task */
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic esc data error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic esc data success.");
    }

    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL,
                                &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};

    USER_LOG_INFO("Fc subscription sample start");
    s_userFcSubscriptionDataShow = true;

    USER_LOG_INFO("--> Step 1: Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 2: Subscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                               DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 3: Get latest value of the subscribed topics in the next 10 seconds\r\n");

    for (int i = 0; i < 10; ++i) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        } else {
            USER_LOG_INFO("velocity: x = %f y = %f z = %f healthFlag = %d, timestamp ms = %d us = %d.",
                          velocity.data.x, velocity.data.y, velocity.data.z,
                          velocity.health, timestamp.millisecond, timestamp.microsecond);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        } else {
            USER_LOG_INFO("gps position: x = %d y = %d z = %d.",
                          gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index1 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index1: capacity percent = %ld%% voltage = %ldV temperature = %.2f degree.",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index2 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index2: capacity percent = %ld%% voltage = %ldV temperature = %.2f degree.\r\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }
    }

    USER_LOG_INFO("--> Step 4: Unsubscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps date error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic gps time error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 5: Deinit fc subscription module");
    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/

/*
 * CsvNeedsHeader — returns true if the file doesn't exist yet or is empty,
 * meaning we should write the header row before the first data row.
 */
static bool CsvNeedsHeader(const char *path)
{
    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return true;   /* file doesn't exist */
    }
    int c = fgetc(fp);
    fclose(fp);
    return (c == EOF); /* file exists but is empty */
}

/*
 * WriteCsvRow — appends one data row to the telemetry CSV.
 * The file pointer must already be open in append mode ("a").
 *
 * The timestamp comes directly from the FC via T_DjiDataTimestamp and is
 * formatted as  <millisecond>.<microsecond>  (e.g. "123456789.045").
 */
static void WriteCsvRow(FILE *fp,
                        const char *timestamp,
                        double lat, double lon, double alt,
                        double pitch, double roll, double yaw,
                        int16_t compassX, int16_t compassY, int16_t compassZ,
                        double altitudeAglM,
                        double velX, double velY, double velZ,
                        double accelRawX, double accelRawY, double accelRawZ,
                        double gyroRawX, double gyroRawY, double gyroRawZ,
                        const int16_t escSpeed[8],
                        double baroAltitudeM)
{
    fprintf(fp, "%s,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,"
                "%d,%d,%d,"
                "%.2f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.4f,%.4f,%.4f,"
                "%d,%d,%d,%d,%d,%d,%d,%d,"
                "%.2f\n",
            timestamp,
            lat, lon, alt, pitch, roll, yaw,
            compassX, compassY, compassZ,
            altitudeAglM,
            velX, velY, velZ,
            accelRawX, accelRawY, accelRawZ,
            gyroRawX, gyroRawY, gyroRawZ,
            escSpeed[0], escSpeed[1], escSpeed[2], escSpeed[3],
            escSpeed[4], escSpeed[5], escSpeed[6], escSpeed[7],
            baroAltitudeM);
    fflush(fp);
}

#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

/*
 * UserFcSubscription_Task
 *
 * Runs at 1 Hz. On each tick:
 *   1. Polls GPS position and GPS details from the FC subscription buffer.
 *   2. Reads the latest pitch/roll/yaw (computed by the quaternion callback).
 *   3. Appends one row to the telemetry CSV on the Raspberry Pi.
 *
 * CSV columns:
 *   fc_timestamp, latitude_deg, longitude_deg, altitude_m,
 *   pitch_deg, roll_deg, yaw_deg,
 *   compass_x, compass_y, compass_z,
 *   altitude_agl_m,
 *   vel_x_ms, vel_y_ms, vel_z_ms,
 *   accel_raw_x_ms2, accel_raw_y_ms2, accel_raw_z_ms2,
 *   gyro_raw_x_rads, gyro_raw_y_rads, gyro_raw_z_rads,
 *   esc1_rpm..esc8_rpm,
 *   baro_altitude_m
 *
 * fc_timestamp is formatted as <millisecond>.<microsecond> from the FC clock,
 * sourced from the GPS position topic poll.
 */
static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode              djiStat;
    T_DjiFcSubscriptionVelocity  velocity    = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDetails gpsDetails  = {0};
    T_DjiFcSubscriptionCompass   compass     = {0};
    T_DjiFcSubscriptionAltitudeFused        altitudeFused        = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint  altitudeOfHomePoint  = 0;
    T_DjiFcSubscriptionAltitudeBarometer    altitudeBarometer    = 0;
    T_DjiFcSubscriptionAccelerationRaw      accelerationRaw      = {0};
    T_DjiFcSubscriptionAngularRateRaw       angularRateRaw       = {0};
    T_DjiFcSubscriptionEscData              escData              = {0};
    T_DjiDataTimestamp           timestamp   = {0};
    T_DjiOsalHandler            *osalHandler = DjiPlatform_GetOsalHandler();

    USER_UTIL_UNUSED(arg);

    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

        /* ---- Poll velocity (unchanged behaviour) ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.",
                          velocity.data.x, velocity.data.y,
                          velocity.data.z, velocity.health);
        }

        /* ---- Poll GPS position — timestamp captured here for CSV ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps position: x %d y %d z %d.",
                          gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        /* ---- Poll GPS details (satellite count) ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps details error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps total satellite number used: %d %d %d.",
                          gpsDetails.gpsSatelliteNumberUsed,
                          gpsDetails.glonassSatelliteNumberUsed,
                          gpsDetails.totalSatelliteNumberUsed);
            s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
        }

        /* ---- Poll compass (magnetometer) ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
                                                          (uint8_t *) &compass,
                                                          sizeof(T_DjiFcSubscriptionCompass),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic compass error.");
        }

        /* ---- Poll fused altitude (ASL) and home point altitude (ASL at takeoff) ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                          (uint8_t *) &altitudeFused,
                                                          sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude fused error.");
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                          (uint8_t *) &altitudeOfHomePoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude of homepoint error.");
        }

        /* ---- Poll barometric altitude ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER,
                                                          (uint8_t *) &altitudeBarometer,
                                                          sizeof(T_DjiFcSubscriptionAltitudeBarometer),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic altitude barometer error.");
        }

        /* ---- Poll raw IMU accelerometer and gyroscope ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                          (uint8_t *) &accelerationRaw,
                                                          sizeof(T_DjiFcSubscriptionAccelerationRaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic acceleration raw error.");
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW,
                                                          (uint8_t *) &angularRateRaw,
                                                          sizeof(T_DjiFcSubscriptionAngularRateRaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic angular rate raw error.");
        }

        /* ---- Poll ESC/motor telemetry ---- */
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA,
                                                          (uint8_t *) &escData,
                                                          sizeof(T_DjiFcSubscriptionEscData),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic esc data error.");
        }

        /* ---- Write telemetry CSV row ---- */
        if (s_isCsvOutputPathConfigured) {

            /* Convert raw GPS integers to human-readable units */
            double lat = (double) gpsPosition.y * GPS_DEG_SCALE;  /* latitude  [deg] */
            double lon = (double) gpsPosition.x * GPS_DEG_SCALE;  /* longitude [deg] */
            double alt = (double) gpsPosition.z * GPS_ALT_SCALE;  /* altitude  [m]   */

            /* Read latest euler angles (set by quaternion callback) */
            osalHandler->MutexLock(s_eulerMutex);
            double pitch = s_pitch;
            double roll  = s_roll;
            double yaw   = s_yaw;
            osalHandler->MutexUnlock(s_eulerMutex);

            /* Get GPS date/time for the CSV timestamp */
            char gpsTimestamp[32] = "";
            T_DjiFcSubscriptionGpsDate gpsDate = 0;
            T_DjiFcSubscriptionGpsTime gpsTime = 0;
            T_DjiDataTimestamp ts = {0};

            T_DjiReturnCode pollStat = DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, (uint8_t *)&gpsDate,
                sizeof(T_DjiFcSubscriptionGpsDate), &ts);
            if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsDate != 0) {
                pollStat = DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, (uint8_t *)&gpsTime,
                    sizeof(T_DjiFcSubscriptionGpsTime), &ts);
                if (pollStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && gpsTime != 0 &&
                    ConvertGpsDateTimeToLocalTimeString(gpsDate, gpsTime,
                                                       "%Y-%m-%d %H:%M:%S",
                                                       gpsTimestamp, sizeof(gpsTimestamp))) {
                    // valid GPS row timestamp
                } else {
                    USER_LOG_ERROR("unable to get valid GPS datetime for row timestamp, using FC time");
                    snprintf(gpsTimestamp, sizeof(gpsTimestamp), "%u.%06u",
                             timestamp.millisecond, timestamp.microsecond);
                }
            } else {
                USER_LOG_ERROR("poll GPS date error for row timestamp, using FC time");
                snprintf(gpsTimestamp, sizeof(gpsTimestamp), "%u.%06u",
                         timestamp.millisecond, timestamp.microsecond);
            }

            /* Open in append mode — creates the file if it doesn't exist */
            /* AGL altitude above the takeoff (home) point: fused ASL altitude
             * minus the ASL altitude recorded when the aircraft last took off. */
            double altitudeAgl = (double) altitudeFused - (double) altitudeOfHomePoint;

            int16_t escSpeed[8];
            for (int escIdx = 0; escIdx < 8; ++escIdx) {
                escSpeed[escIdx] = escData.esc[escIdx].speed;
            }

            bool   writeHeader = CsvNeedsHeader(s_csvOutputPath);
            FILE  *fp          = fopen(s_csvOutputPath, "a");
            if (fp != NULL) {
                if (writeHeader) {
                    fprintf(fp, "timestamp,latitude_deg,longitude_deg,"
                                "altitude_m,pitch_deg,roll_deg,yaw_deg,"
                                "compass_x,compass_y,compass_z,"
                                "altitude_agl_m,"
                                "vel_x_ms,vel_y_ms,vel_z_ms,"
                                "accel_raw_x_ms2,accel_raw_y_ms2,accel_raw_z_ms2,"
                                "gyro_raw_x_rads,gyro_raw_y_rads,gyro_raw_z_rads,"
                                "esc1_rpm,esc2_rpm,esc3_rpm,esc4_rpm,"
                                "esc5_rpm,esc6_rpm,esc7_rpm,esc8_rpm,"
                                "baro_altitude_m\n");
                }

                WriteCsvRow(fp, gpsTimestamp, lat, lon, alt, pitch, roll, yaw,
                           compass.x, compass.y, compass.z,
                           altitudeAgl,
                           (double) velocity.data.x, (double) velocity.data.y, (double) velocity.data.z,
                           (double) accelerationRaw.x, (double) accelerationRaw.y, (double) accelerationRaw.z,
                           (double) angularRateRaw.x, (double) angularRateRaw.y, (double) angularRateRaw.z,
                           escSpeed,
                           (double) altitudeBarometer);
                fclose(fp);
            } else {
                USER_LOG_ERROR("Cannot open telemetry CSV: %s", s_csvOutputPath);
            }
        }
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

/*
 * DjiTest_FcSubscriptionReceiveQuaternionCallback
 *
 * Called by PSDK at 50 Hz. Converts quaternion to pitch/roll/yaw in degrees
 * and stores them in the shared static variables for the 1 Hz task to read.
 */
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data,
                                                                       uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    T_DjiOsalHandler              *osalHandler = DjiPlatform_GetOsalHandler();

    USER_UTIL_UNUSED(dataSize);

    double pitch = (double) asinf(-2 * quaternion->q1 * quaternion->q3 +
                                   2 * quaternion->q0 * quaternion->q2) * 57.3;
    double roll  = (double) atan2f(2 * quaternion->q2 * quaternion->q3 +
                                    2 * quaternion->q0 * quaternion->q1,
                                   -2 * quaternion->q1 * quaternion->q1 -
                                    2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    double yaw   = (double) atan2f(2 * quaternion->q1 * quaternion->q2 +
                                    2 * quaternion->q0 * quaternion->q3,
                                   -2 * quaternion->q2 * quaternion->q2 -
                                    2 * quaternion->q3 * quaternion->q3 + 1) * 57.3;

    /* Store under mutex so the 1 Hz task always sees a consistent set */
    osalHandler->MutexLock(s_eulerMutex);
    s_pitch = pitch;
    s_roll  = roll;
    s_yaw   = yaw;
    osalHandler->MutexUnlock(s_eulerMutex);

    /* Console logging (throttled to 1 Hz equivalent) */
    if (s_userFcSubscriptionDataShow == true) {
        if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
            USER_LOG_INFO("receive quaternion data.");
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.",
                          timestamp->millisecond, timestamp->microsecond);
            USER_LOG_INFO("quaternion: %f %f %f %f.",
                          quaternion->q0, quaternion->q1,
                          quaternion->q2, quaternion->q3);
            USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n",
                          pitch, roll, yaw);
            DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/