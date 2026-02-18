/**
 ********************************************************************
 * @file    test_widget.c
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
#include "test_widget.h"
#include <dji_widget.h>
#include <dji_logger.h>
#include "../utils/util_misc.h"
#include <dji_platform.h>
#include <stdio.h>
#include "dji_sdk_config.h"
#include "file_binary_array_list_en.h"
#include <stdarg.h>

/* Private constants ---------------------------------------------------------*/
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)
#define WIDGET_LOG_STRING_MAX_SIZE      (64)
#define WIDGET_LOG_LINE_MAX_NUM         (4)

/* Private types -------------------------------------------------------------*/
typedef struct {
    bool valid;
    char content[WIDGET_LOG_STRING_MAX_SIZE];
} T_DjiTestWidgetLog;

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value,
                                                    void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value,
                                                    void *userData);

/* Private values ------------------------------------------------------------*/
static T_DjiTaskHandle s_widgetTestThread;

/* csv transmision to the rc (ranavner)---------------------------------------*/
static bool s_isCsvFilePathConfigured = false;
static char s_csvFilePath[WIDGET_DIR_PATH_LEN_MAX] = {0};



static bool s_isWidgetFileDirPathConfigured = false;
static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = {0};
static T_DjiTestWidgetLog s_djiTestWidgetLog[WIDGET_LOG_LINE_MAX_NUM] = {0};
static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] = {
    {0, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {1, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {2, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {3, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {4, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {5, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {6, DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {7, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {8, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
};

static const char *s_widgetTypeNameArray[] = {
    "Unknown",
    "Button",
    "Switch",
    "Scale",
    "List",
    "Int input box"
};

static const uint32_t s_widgetHandlerListCount = sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

/* Exported functions definition ---------------------------------------------*/


void DjiTest_WidgetLogAppend(const char *fmt, ...)
{
    va_list args;
    char string[WIDGET_LOG_STRING_MAX_SIZE];
    int32_t i;

    va_start(args, fmt);
    vsnprintf(string, WIDGET_LOG_STRING_MAX_SIZE, fmt, args);
    va_end(args);

    for (i = 0; i < WIDGET_LOG_LINE_MAX_NUM; ++i) {
        if (s_djiTestWidgetLog[i].valid == false) {
            s_djiTestWidgetLog[i].valid = true;
            strcpy(s_djiTestWidgetLog[i].content, string);
            break;
        }
    }

    if (i == WIDGET_LOG_LINE_MAX_NUM) {
        for (i = 0; i < WIDGET_LOG_LINE_MAX_NUM - 1; i++) {
            strcpy(s_djiTestWidgetLog[i].content, s_djiTestWidgetLog[i + 1].content);
        }
        strcpy(s_djiTestWidgetLog[WIDGET_LOG_LINE_MAX_NUM - 1].content, string);
    }
}

T_DjiReturnCode DjiTest_WidgetStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    //Step 1 : Init DJI Widget
    djiStat = DjiWidget_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

#ifdef SYSTEM_ARCH_LINUX
    //Step 2 : Set UI Config (Linux environment)
    char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
    char tempPath[WIDGET_DIR_PATH_LEN_MAX];
    djiStat = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", curFileDirPath);
    }

    //set default ui config path
    USER_LOG_INFO("widget file: %s", tempPath);
    djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //set ui config for English language
    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //set ui config for Chinese language
    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", curFileDirPath);
    }

    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }
#else
    //Step 2 : Set UI Config (RTOS environment)
    T_DjiWidgetBinaryArrayConfig enWidgetBinaryArrayConfig = {
        .binaryArrayCount = g_EnBinaryArrayCount,
        .fileBinaryArrayList = g_EnFileBinaryArrayList
    };

    //set default ui config
    djiStat = DjiWidget_RegDefaultUiConfigByBinaryArray(&enWidgetBinaryArrayConfig);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }
#endif
    //Step 3 : Set widget handler list
    djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //Step 4 : Run widget api sample task
    if (osalHandler->TaskCreate("user_widget_task", DjiTest_WidgetTask, WIDGET_TASK_STACK_SIZE, NULL,
                                &s_widgetTestThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji widget test task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_WidgetSetConfigFilePath(const char *path)
{
    memset(s_widgetFileDirPath, 0, sizeof(s_widgetFileDirPath));
    memcpy(s_widgetFileDirPath, path, USER_UTIL_MIN(strlen(path), sizeof(s_widgetFileDirPath) - 1));
    s_isWidgetFileDirPathConfigured = true;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_WidgetSetCsvFilePath(const char *path)
{
    memset(s_csvFilePath, 0, sizeof(s_csvFilePath));
    memcpy(s_csvFilePath, path, USER_UTIL_MIN(strlen(path), sizeof(s_csvFilePath) - 1));
    s_isCsvFilePathConfigured = true;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wformat"
#endif

/*
 * DjiTest_WidgetTask
 *
 * Reads vitals.csv at 1 Hz, parses the last data line into its four fields
 * (timestamp, temp, RH, pressure) and sends them as labeled items to the
 * DJI Pilot 2 floating window.
 *
 * Expected CSV format (with optional header row):
 *   timestamp,temp,RH,pressure
 *   2026-02-18T10:30:00,23.5,61.2,1013.4
 */
void *DjiTest_WidgetTask(void *arg)
{
    char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    USER_UTIL_UNUSED(arg);

    while (1) {
        /* ----------------------------------------------------------------
         * Step 1 — Read the last data line from the CSV
         * ---------------------------------------------------------------- */
        char timestamp[64] = "N/A";
        float temp     = 0.0f;
        float rh       = 0.0f;
        float pressure = 0.0f;
        bool  parsed   = false;

        if (s_isCsvFilePathConfigured) {
            FILE *fp = fopen(s_csvFilePath, "r");
            if (fp != NULL) {
                char buf[256];
                char lastDataLine[256] = {0};

                while (fgets(buf, sizeof(buf), fp) != NULL) {
                    /* Skip header row: starts with a non-digit character
                     * (e.g. "timestamp,temp,RH,pressure").
                     * A data row always begins with a digit (year). */
                    if (buf[0] >= '0' && buf[0] <= '9') {
                        strncpy(lastDataLine, buf, sizeof(lastDataLine) - 1);
                        lastDataLine[sizeof(lastDataLine) - 1] = '\0';
                    }
                }
                fclose(fp);

                if (lastDataLine[0] != '\0') {
                    /* Strip trailing newline / carriage return */
                    size_t len = strlen(lastDataLine);
                    while (len > 0 &&
                           (lastDataLine[len - 1] == '\n' ||
                            lastDataLine[len - 1] == '\r')) {
                        lastDataLine[--len] = '\0';
                    }

                    /* Parse: "timestamp,temp,RH,pressure"
                     * %63[^,] reads everything up to the first comma. */
                    char ts[64] = {0};
                    float t = 0.0f, h = 0.0f, p = 0.0f;
                    if (sscanf(lastDataLine, "%63[^,],%f,%f,%f",
                               ts, &t, &h, &p) == 4) {
                        strncpy(timestamp, ts, sizeof(timestamp) - 1);
                        timestamp[sizeof(timestamp) - 1] = '\0';
                        temp     = t;
                        rh       = h;
                        pressure = p;
                        parsed   = true;
                    } else {
                        USER_LOG_WARN("CSV parse failed on line: %s", lastDataLine);
                    }
                }
            } else {
                USER_LOG_WARN("Cannot open CSV: %s", s_csvFilePath);
            }
        }

        /* ----------------------------------------------------------------
         * Step 2 — Compose the floating-window message
         * ---------------------------------------------------------------- */
        if (parsed) {
            snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
                     "=== Air Vitals ===\r\n"
                     "Time    : %s\r\n"
                     "Temp    : %.2f degC\r\n"
                     "RH      : %.2f %%\r\n"
                     "Pressure: %.2f hPa\r\n",
                     timestamp, temp, rh, pressure);
        } else {
            snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
                     "=== Air Vitals ===\r\n"
                     "Waiting for data...\r\n"
                     "File: %s\r\n",
                     s_isCsvFilePathConfigured ? s_csvFilePath : "(path not set)");
        }

        /* ----------------------------------------------------------------
         * Step 3 — Send to DJI Pilot 2 floating window
         * ---------------------------------------------------------------- */
        djiStat = DjiWidgetFloatingWindow_ShowMessage(message);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Floating window show message error, stat = 0x%08llX", djiStat);
        }

        /* ----------------------------------------------------------------
         * Step 4 — Sleep 1000 ms → 1 Hz update rate
         * ---------------------------------------------------------------- */
        osalHandler->TaskSleepMs(1000);
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

/* Private functions definition-----------------------------------------------*/

static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value,
                                                    void *userData)
{
    USER_UTIL_UNUSED(userData);

    DjiTest_WidgetLogAppend("Set widget: typ %s idx %d val %d\n", s_widgetTypeNameArray[widgetType], index, value);
    USER_LOG_INFO("Set widget value, widgetType = %s, widgetIndex = %d ,widgetValue = %d",
                  s_widgetTypeNameArray[widgetType], index, value);
    s_widgetValueList[index] = value;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value,
                                                    void *userData)
{
    USER_UTIL_UNUSED(userData);
    USER_UTIL_UNUSED(widgetType);

    *value = s_widgetValueList[index];

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
