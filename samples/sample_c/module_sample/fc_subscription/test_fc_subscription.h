/**
 ********************************************************************
 * @file    test_fc_subscription.h
 * @brief   This is the header file for "test_fc_subscription.c", defining the structure and
 * (exported) function prototypes.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_FC_SUBSCRIPTION_H
#define TEST_FC_SUBSCRIPTION_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_fc_subscription.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void);
T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void);
T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void);
T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number);

/**
 * @brief Set the output path for the drone telemetry CSV file.
 *
 * Must be called before DjiTest_FcSubscriptionStartService().
 * The CSV will be created (or appended to) at this path and written at 1 Hz.
 *
 * Format:
 *   timestamp,latitude_deg,longitude_deg,altitude_m,pitch_deg,roll_deg,yaw_deg,
 *   compass_x,compass_y,compass_z,altitude_agl_m,
 *   vel_x_ms,vel_y_ms,vel_z_ms,
 *   accel_raw_x_ms2,accel_raw_y_ms2,accel_raw_z_ms2,
 *   gyro_raw_x_rads,gyro_raw_y_rads,gyro_raw_z_rads,
 *   esc1_rpm,esc2_rpm,esc3_rpm,esc4_rpm,esc5_rpm,esc6_rpm,esc7_rpm,esc8_rpm,
 *   baro_altitude_m
 *
 * @param path  Absolute path on the Raspberry Pi filesystem,
 *              e.g. "/home/rsp/drone_air_system/data_from_drone/telemetry.csv"
 */
T_DjiReturnCode DjiTest_FcSubscriptionSetCsvOutputPath(const char *path);

#ifdef __cplusplus
}
#endif

#endif // TEST_FC_SUBSCRIPTION_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
