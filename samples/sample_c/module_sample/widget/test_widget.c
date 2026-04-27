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
#include <string.h>
#include <math.h>       /* NAN, isnan() */
#include <stdarg.h>
#include "dji_sdk_config.h"
#include "file_binary_array_list_en.h"

/* Private constants ---------------------------------------------------------*/
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)
#define WIDGET_LOG_STRING_MAX_SIZE      (64)
#define WIDGET_LOG_LINE_MAX_NUM         (4)

#define VITALS_NUM_FIELDS               (11)

/* Private types -------------------------------------------------------------*/
typedef struct {
    bool  valid;
    char  content[WIDGET_LOG_STRING_MAX_SIZE];
} T_DjiTestWidgetLog;

/*
 * T_VitalsField — one entry in the field dictionary.
 *
 * csvHeader   : exact column name as it appears in the vitals.csv header row.
 *               Change these strings to match your actual CSV headers.
 * label       : short label shown in the DJI floating window.
 * unit        : unit suffix appended after the value (empty string = no unit).
 * fmt         : printf format for the numeric value (e.g. "%.1f").
 * colIndex    : resolved at runtime by ResolveVitalsHeaders(); do not edit.
 * value       : last parsed reading; NAN means the field was absent or empty.
 */
typedef struct {
    const char *csvHeader;
    const char *label;
    const char *unit;
    const char *fmt;
    int         colIndex;   /* -1 = not yet resolved / not found in CSV */
    float       value;      /* NAN  = missing / not parsed              */
} T_VitalsField;

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value,
                                                    void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value,
                                                    void *userData);
static void  ResolveVitalsHeaders(const char *headerLine);
static bool  ParseVitalsDataLine(const char *line, char *tsBuf, size_t tsBufSize);

/* Private values ------------------------------------------------------------*/
static T_DjiTaskHandle s_widgetTestThread;

/* csv transmission to the rc (ranavner) -------------------------------------*/
static bool s_isCsvFilePathConfigured               = false;
static char s_csvFilePath[WIDGET_DIR_PATH_LEN_MAX]  = {0};

static bool s_isWidgetFileDirPathConfigured             = false;
static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = {0};
static T_DjiTestWidgetLog s_djiTestWidgetLog[WIDGET_LOG_LINE_MAX_NUM] = {0};

/* =========================================================================
 * FIELD DICTIONARY — edit csvHeader strings to match your vitals.csv exactly.
 *
 * The code reads the CSV header row at runtime, finds each csvHeader string,
 * and records its column index. Data values are then extracted by column
 * position rather than by a fixed offset, so column order in the CSV does
 * not matter as long as the header names match.
 *
 * colIndex and value are managed at runtime — do not edit those fields.
 * ========================================================================= */
static T_VitalsField s_vitalsFields[VITALS_NUM_FIELDS] = {
    /* csvHeader                          label    unit   fmt     colIndex  value */
    { "iMet_Temp_C",                      "T",     "C",   "%.1f", -1,       NAN   },
    { "POM_Ozone_ppb",                    "O3",    "ppb", "%.1f", -1,       NAN   },
    { "Spectro_MaxIntensity",             "I",     "",    "%.0f", -1,       NAN   },
    { "Partector_Particle_Count_#/cm3",   "PM",    "/cc", "%.0f", -1,       NAN   },
    { "Aeth_Blue_BlackCarbon",            "MA",    "",    "%.3f", -1,       NAN   },
    { "TEC_Output_Current",               "TECA",  "A",   "%.2f", -1,       NAN   },
    { "TEC_Object_Temperature",           "TECT",  "C",   "%.1f", -1,       NAN   },
    { "Inline_Temp",                      "Ti",    "C",   "%.1f", -1,       NAN   },
    { "Inline_Relative_Humidity",         "RHi",   "%",   "%.1f", -1,       NAN   },
    { "Inline_Pressure_mbar",             "Pi",    "mb",  "%.1f", -1,       NAN   },
    { "Pump_RPM",                         "RPM",   "",    "%.0f", -1,       NAN   },
};

/* Set to true once the header row has been successfully parsed */
static bool s_vitalsHeadersResolved = false;

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

static const uint32_t s_widgetHandlerListCount =
    sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

/* Exported functions definition ---------------------------------------------*/

void DjiTest_WidgetLogAppend(const char *fmt, ...)
{
    va_list args;
    char    string[WIDGET_LOG_STRING_MAX_SIZE];
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
    T_DjiReturnCode   djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    /* Step 1: Init DJI Widget */
    djiStat = DjiWidget_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

#ifdef SYSTEM_ARCH_LINUX
    /* Step 2: Set UI Config (Linux environment) */
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

    USER_LOG_INFO("widget file: %s", tempPath);
    djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

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
    /* Step 2: Set UI Config (RTOS environment) */
    T_DjiWidgetBinaryArrayConfig enWidgetBinaryArrayConfig = {
        .binaryArrayCount    = g_EnBinaryArrayCount,
        .fileBinaryArrayList = g_EnFileBinaryArrayList
    };

    djiStat = DjiWidget_RegDefaultUiConfigByBinaryArray(&enWidgetBinaryArrayConfig);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }
#endif

    /* Step 3: Set widget handler list */
    djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    /* Step 4: Run widget task */
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

    /* Force header re-resolution next time the task reads the file,
     * in case the new CSV has a different column layout. */
    s_vitalsHeadersResolved = false;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* -------------------------------------------------------------------------
 * ResolveVitalsHeaders
 *
 * Parses the CSV header row and sets colIndex on every entry in
 * s_vitalsFields[] to the 0-based column number where that field's
 * csvHeader string was found. Fields whose csvHeader is not present in
 * the header row keep colIndex = -1, and their value will remain NAN.
 *
 * Called once per CSV file open (guarded by s_vitalsHeadersResolved).
 * ------------------------------------------------------------------------- */
static void ResolveVitalsHeaders(const char *headerLine)
{
    int         col = 0;
    const char *cur = headerLine;
    char        token[64];
    int         i;

    /* Reset all column indices */
    for (i = 0; i < VITALS_NUM_FIELDS; i++) {
        s_vitalsFields[i].colIndex = -1;
    }

    while (cur != NULL) {
        const char *comma = strchr(cur, ',');
        size_t      len   = comma ? (size_t)(comma - cur) : strlen(cur);

        if (len >= sizeof(token)) len = sizeof(token) - 1;
        memcpy(token, cur, len);
        token[len] = '\0';

        /* Strip trailing whitespace / CR / LF */
        {
            size_t tlen = strlen(token);
            while (tlen > 0 &&
                   (token[tlen - 1] == ' '  || token[tlen - 1] == '\t' ||
                    token[tlen - 1] == '\r' || token[tlen - 1] == '\n')) {
                token[--tlen] = '\0';
            }
        }

        /* Match against every field in the dictionary */
        for (i = 0; i < VITALS_NUM_FIELDS; i++) {
            if (strcmp(token, s_vitalsFields[i].csvHeader) == 0) {
                s_vitalsFields[i].colIndex = col;
                break;
            }
        }

        col++;
        cur = comma ? comma + 1 : NULL;
    }
}

/* -------------------------------------------------------------------------
 * ParseVitalsDataLine
 *
 * Parses one data row using the column indices resolved by
 * ResolveVitalsHeaders(). Stores each reading into the corresponding
 * s_vitalsFields[].value. Empty tokens or columns whose header was not
 * found are stored as NAN so the display can show "---" instead of a
 * misleading zero.
 *
 * The timestamp (column 0) is written to tsBuf.
 * Returns true if the timestamp was successfully read.
 * ------------------------------------------------------------------------- */
static bool ParseVitalsDataLine(const char *line, char *tsBuf, size_t tsBufSize)
{
    const char *cur = line;
    char        field[64];
    int         col = 0;
    int         i;

    /* Reset all field values to NAN */
    for (i = 0; i < VITALS_NUM_FIELDS; i++) {
        s_vitalsFields[i].value = NAN;
    }

    if (cur == NULL || *cur == '\0') return false;

    for (;;) {
        const char *comma = strchr(cur, ',');
        size_t      len   = comma ? (size_t)(comma - cur) : strlen(cur);

        if (len >= sizeof(field)) len = sizeof(field) - 1;
        memcpy(field, cur, len);
        field[len] = '\0';

        /* Strip trailing CR / LF from the last field in the row */
        {
            size_t flen = strlen(field);
            while (flen > 0 &&
                   (field[flen - 1] == '\r' || field[flen - 1] == '\n')) {
                field[--flen] = '\0';
            }
        }

        if (col == 0) {
            /* Column 0 is always the timestamp */
            size_t tsLen = strlen(field);
            if (tsLen >= tsBufSize) tsLen = tsBufSize - 1;
            memcpy(tsBuf, field, tsLen);
            tsBuf[tsLen] = '\0';
        } else {
            /* Look up which vitals field (if any) lives at this column */
            for (i = 0; i < VITALS_NUM_FIELDS; i++) {
                if (s_vitalsFields[i].colIndex == col) {
                    /* Empty token → NAN (will display as "---") */
                    s_vitalsFields[i].value =
                        (field[0] != '\0') ? strtof(field, NULL) : NAN;
                    break;
                }
            }
        }

        col++;
        if (comma == NULL) break;
        cur = comma + 1;
    }

    return (tsBuf[0] != '\0');
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
 * Runs at 1 Hz. Opens vitals.csv on every tick, resolves headers on the
 * first read (or whenever the CSV path changes), then parses the last data
 * row and displays all fields in the DJI Pilot 2 floating window.
 *
 * Fields that are absent from the CSV header or whose value is empty are
 * shown as "---" rather than a misleading zero.
 */
void *DjiTest_WidgetTask(void *arg)
{
    char              message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    T_DjiReturnCode   djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    USER_UTIL_UNUSED(arg);

    while (1) {

        /* ----------------------------------------------------------------
         * Step 1 — Read vitals.csv: resolve headers (once) then parse the
         *           last data line.
         * ---------------------------------------------------------------- */
        char ts[64] = "N/A";
        bool parsed = false;

        if (s_isCsvFilePathConfigured) {
            FILE *fp = fopen(s_csvFilePath, "r");
            if (fp != NULL) {
                char buf[512];
                char lastDataLine[512] = {0};

                while (fgets(buf, sizeof(buf), fp) != NULL) {
                    if (buf[0] >= '0' && buf[0] <= '9') {
                        /* Data row */
                        strncpy(lastDataLine, buf, sizeof(lastDataLine) - 1);
                        lastDataLine[sizeof(lastDataLine) - 1] = '\0';
                    } else if (!s_vitalsHeadersResolved && buf[0] != '\0') {
                        /*
                         * First non-data row encountered before headers are
                         * resolved → treat it as the header row.
                         * This handles CSV files where the header is the
                         * first line (the common case).
                         */
                        ResolveVitalsHeaders(buf);
                        s_vitalsHeadersResolved = true;

                        /* Log any fields that could not be matched */
                        int i;
                        for (i = 0; i < VITALS_NUM_FIELDS; i++) {
                            if (s_vitalsFields[i].colIndex == -1) {
                                USER_LOG_WARN("Vitals header not found in CSV: \"%s\"",
                                              s_vitalsFields[i].csvHeader);
                            }
                        }
                    }
                }
                fclose(fp);

                if (lastDataLine[0] != '\0') {
                    parsed = ParseVitalsDataLine(lastDataLine, ts, sizeof(ts));
                }
            } else {
                USER_LOG_WARN("Cannot open CSV: %s", s_csvFilePath);
            }
        }

        /* ----------------------------------------------------------------
         * Step 2 — Compose the floating-window message.
         *
         * Each field is formatted as  "LABEL:VALUE UNIT\r\n".
         * If a field's value is NAN (missing / empty in CSV / header not
         * found), "---" is shown instead of a number.
         *
         * DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN = 255 bytes (hard SDK limit).
         * The offset-tracked snprintf loop stops before overflowing.
         * ---------------------------------------------------------------- */
        int offset = 0;

        if (!parsed) {
            offset += snprintf(message + offset,
                               DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN - offset,
                               "=== Air Vitals ===\r\n"
                               "Waiting for data...\r\n"
                               "File: %s\r\n",
                               s_isCsvFilePathConfigured ? s_csvFilePath : "(path not set)");
        } else {
            /* Timestamp line */
            offset += snprintf(message + offset,
                               DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN - offset,
                               "%s\r\n", ts);

            /* One line per vitals field */
            int i;
            for (i = 0; i < VITALS_NUM_FIELDS; i++) {
                int remaining = DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN - offset;
                if (remaining <= 1) break;  /* No space left */

                T_VitalsField *fld = &s_vitalsFields[i];

                if (isnan(fld->value)) {
                    /* Missing / empty field → display "---" */
                    offset += snprintf(message + offset, remaining,
                                       "%s:---\r\n", fld->label);
                } else {
                    /* Format the numeric value then append label+unit */
                    char valBuf[32];
                    snprintf(valBuf, sizeof(valBuf), fld->fmt, fld->value);
                    offset += snprintf(message + offset, remaining,
                                       "%s:%s%s\r\n",
                                       fld->label, valBuf, fld->unit);
                }
            }
        }

        /* ----------------------------------------------------------------
         * Step 3 — Send to DJI Pilot 2 floating window
         * ---------------------------------------------------------------- */
        djiStat = DjiWidgetFloatingWindow_ShowMessage(message);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Floating window show message error, stat = 0x%08llX", djiStat);
        }

        /* ----------------------------------------------------------------
         * Step 4 — Sleep 1000 ms → 1 Hz
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

    DjiTest_WidgetLogAppend("Set widget: typ %s idx %d val %d\n",
                            s_widgetTypeNameArray[widgetType], index, value);
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