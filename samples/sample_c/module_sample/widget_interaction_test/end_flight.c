/**
 * End of flight from the remote controller. See end_flight.h.
 */
#include "end_flight.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "dji_logger.h"
#include "dji_fc_subscription.h"

/* The dashboard's status file is ignored once this old: a dashboard that
 * died mid-job must not cover the vitals for good. */
#define END_FLIGHT_RUNNING_MAX_AGE_S  (15 * 60)
#define END_FLIGHT_DONE_MAX_AGE_S     (10 * 60)
/* A NOTE (no USB drive, nothing stopped) shows this long, then the vitals. */
#define END_FLIGHT_NOTE_MAX_AGE_S     (30)
/* Status file kinds. */
#define KIND_NONE     0
#define KIND_RUNNING  1
#define KIND_DONE     2
#define KIND_WAITING  3   /* waiting for a USB drive: END x2 finishes without backup */
#define KIND_NOTE     4
/* After the second press, "Starting" shows until the dashboard answers. */
#define END_FLIGHT_STARTING_MS        (8000)
/* How long a refused press explains itself in the box. */
#define END_FLIGHT_REFUSED_MS         (4000)
/* Retry the flight status subscription at most this often. */
#define END_FLIGHT_SUBSCRIBE_RETRY_MS (5000)

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t s_armedAtMs = 0;      /* first press, 0 when not armed */
static uint64_t s_startedAtMs = 0;    /* second press, 0 when not started */
static time_t s_appStart = 0;         /* a DONE written before this app started is old news */
static uint64_t s_refusedAtMs = 0;    /* a press refused because of the flight status */
static const char *s_refusedText = "";
static volatile bool s_flightSubscribed = false;
static bool s_armedForSkip = false;   /* armed while it waits for a drive */
static bool s_startedSkip = false;    /* the second press asked to skip the backup */
static bool s_driveQueryPending = false;
static char s_driveText[64] = "";     /* "OK (57.1 GB free)", "NO USB DRIVE", ... */
static uint64_t s_subscribeTriedMs = 0;

static uint64_t NowMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000ULL + (uint64_t) ts.tv_nsec / 1000000ULL;
}

/* Subscribes to the flight status once the FC subscription is up. Called
 * every second from the data box task, so it's ready before any press. */
static void EnsureFlightStatusSubscribed(uint64_t now)
{
    T_DjiReturnCode stat;

    if (s_flightSubscribed ||
        (s_subscribeTriedMs != 0 && now - s_subscribeTriedMs < END_FLIGHT_SUBSCRIBE_RETRY_MS)) {
        return;
    }
    s_subscribeTriedMs = now;
    stat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                            DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, NULL);
    if (stat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_flightSubscribed = true;
        USER_LOG_INFO("End of flight: subscribed to the flight status");
    } else {
        USER_LOG_WARN("End of flight: flight status subscription failed, stat = 0x%08llX", stat);
    }
}

/* Returns the DJI flight status (0 stopped, 1 on ground, 2 in air), or -1
 * when it can't be read. */
static int FlightStatus(void)
{
    T_DjiFcSubscriptionFlightStatus status = 0;
    T_DjiDataTimestamp timestamp = {0};

    if (!s_flightSubscribed) {
        return -1;
    }
    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                (uint8_t *) &status, sizeof(status), &timestamp)
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return -1;
    }
    return (int) status;
}

static void EnsureAppStart(void)
{
    if (s_appStart == 0) {
        s_appStart = time(NULL);
    }
}

/* Reads the status file. Returns a KIND_*; body gets the text after the
 * first line. */
static int ReadStatus(char *body, size_t bodyLen)
{
    struct stat st;
    char buf[512];
    FILE *fp;
    size_t n;
    char *nl;
    time_t now = time(NULL);
    int kind;

    if (stat(END_FLIGHT_STATUS_PATH, &st) != 0) {
        return 0;
    }
    fp = fopen(END_FLIGHT_STATUS_PATH, "r");
    if (fp == NULL) {
        return 0;
    }
    n = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    buf[n] = '\0';

    nl = strchr(buf, '\n');
    if (nl == NULL) {
        return 0;
    }
    *nl = '\0';
    if (strcmp(buf, "RUNNING") == 0) {
        kind = (now - st.st_mtime <= END_FLIGHT_RUNNING_MAX_AGE_S) ? KIND_RUNNING : KIND_NONE;
    } else if (strcmp(buf, "WAITING") == 0) {
        kind = (now - st.st_mtime <= END_FLIGHT_RUNNING_MAX_AGE_S) ? KIND_WAITING : KIND_NONE;
    } else if (strcmp(buf, "DONE") == 0) {
        kind = (st.st_mtime >= s_appStart && now - st.st_mtime <= END_FLIGHT_DONE_MAX_AGE_S) ? KIND_DONE : KIND_NONE;
    } else if (strcmp(buf, "NOTE") == 0) {
        kind = (st.st_mtime >= s_appStart && now - st.st_mtime <= END_FLIGHT_NOTE_MAX_AGE_S) ? KIND_NOTE : KIND_NONE;
    } else {
        kind = KIND_NONE;
    }
    if (kind != KIND_NONE) {
        snprintf(body, bodyLen, "%s", nl + 1);
    }
    return kind;
}

/* Copies text into out, turning \n into \r\n, within outLen. */
static void CopyCrLf(char *out, size_t outLen, const char *text)
{
    size_t o = 0;
    const char *p;

    if (outLen == 0) {
        return;
    }
    for (p = text; *p != '\0'; p++) {
        if (*p == '\r') {
            continue;
        }
        if (*p == '\n') {
            if (o + 2 >= outLen) {
                break;
            }
            out[o++] = '\r';
            out[o++] = '\n';
        } else {
            if (o + 1 >= outLen) {
                break;
            }
            out[o++] = *p;
        }
    }
    out[o] = '\0';
}

static void StartEndOfFlight(bool skipBackup)
{
    /* Runs in the background, so the widget callback returns at once. The
     * dashboard does the work; its answer only goes to the log file. */
    int rc = system(skipBackup
                    ? "curl -s -m 30 -X POST -H 'Content-Type: application/json' "
                      "-d '{\"from\":\"rc\",\"skip_backup\":true}' " END_FLIGHT_URL
                      " > /tmp/mamp_end_flight_post.txt 2>&1 &"
                    : "curl -s -m 30 -X POST -H 'Content-Type: application/json' "
                      "-d '{\"from\":\"rc\"}' " END_FLIGHT_URL
                      " > /tmp/mamp_end_flight_post.txt 2>&1 &");
    USER_LOG_INFO("End of flight %s from the remote (curl start: %d)",
                  skipBackup ? "without backup" : "requested", rc);
}

/* Asks the dashboard whether it sees a USB drive, for the first press's
 * message. Called from the data box task, never the button callback. */
static void QueryDrive(void)
{
    char line[64] = "";
    FILE *fp = popen("curl -s -m 5 '" END_FLIGHT_DRIVE_URL "' 2>/dev/null", "r");
    size_t len;

    if (fp != NULL) {
        if (fgets(line, sizeof(line), fp) == NULL) {
            line[0] = '\0';
        }
        pclose(fp);
    }
    len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[--len] = '\0';
    }
    pthread_mutex_lock(&s_mutex);
    snprintf(s_driveText, sizeof(s_driveText), "%s", len > 0 ? line : "can't check");
    s_driveQueryPending = false;
    pthread_mutex_unlock(&s_mutex);
}

void EndFlight_OnButtonPress(void)
{
    char body[512];
    uint64_t now = NowMs();
    bool start = false;
    bool skip = false;
    int flight;
    int kind = KIND_NONE;

    pthread_mutex_lock(&s_mutex);
    EnsureAppStart();
    flight = FlightStatus();
    if (flight == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR || flight < 0) {
        /* Never end a flight in the air; if the status is unknown, refuse
         * too, and leave it to the dashboard. */
        s_armedAtMs = 0;
        s_refusedAtMs = now;
        s_refusedText = (flight < 0)
                        ? "END OF FLIGHT\nCan't tell if the drone\nhas landed.\nUse the dashboard.\n"
                        : "END OF FLIGHT\nOff while flying.\nLand first.\n";
        USER_LOG_WARN("End of flight refused: flight status %d", flight);
    } else if ((kind = ReadStatus(body, sizeof(body))) == KIND_RUNNING ||
               (s_startedAtMs != 0 && now - s_startedAtMs < END_FLIGHT_STARTING_MS)) {
        /* Already running: nothing to do. */
    } else if (s_armedAtMs != 0 && now - s_armedAtMs <= END_FLIGHT_CONFIRM_MS &&
               s_armedForSkip == (kind == KIND_WAITING)) {
        s_armedAtMs = 0;
        s_startedAtMs = now;
        s_startedSkip = s_armedForSkip;
        skip = s_armedForSkip;
        start = true;
    } else {
        s_armedAtMs = now;
        s_armedForSkip = (kind == KIND_WAITING);
        if (!s_armedForSkip) {
            snprintf(s_driveText, sizeof(s_driveText), "checking...");
            s_driveQueryPending = true;
        }
        USER_LOG_INFO("End of flight armed%s; press END again within %d s",
                      s_armedForSkip ? " (finish without backup)" : "", END_FLIGHT_CONFIRM_MS / 1000);
    }
    pthread_mutex_unlock(&s_mutex);

    if (start) {
        StartEndOfFlight(skip);
    }
}

bool EndFlight_GetMessage(char *out, size_t outLen)
{
    char body[512];
    char text[600];
    uint64_t now = NowMs();
    int kind;
    bool show = true;
    bool queryDrive;

    /* Only this task subscribes, and the call can block for a moment, so
     * it runs before taking the lock the button handler needs. */
    EnsureFlightStatusSubscribed(now);
    pthread_mutex_lock(&s_mutex);
    queryDrive = s_driveQueryPending;
    pthread_mutex_unlock(&s_mutex);
    if (queryDrive) {
        QueryDrive();   /* blocks up to a few seconds: outside the lock */
    }
    pthread_mutex_lock(&s_mutex);
    EnsureAppStart();
    kind = ReadStatus(body, sizeof(body));
    if (s_refusedAtMs != 0 && now - s_refusedAtMs < END_FLIGHT_REFUSED_MS) {
        snprintf(text, sizeof(text), "%s", s_refusedText);
    } else if (s_armedAtMs != 0 && now - s_armedAtMs <= END_FLIGHT_CONFIRM_MS) {
        int left = (int) ((END_FLIGHT_CONFIRM_MS - (now - s_armedAtMs) + 999) / 1000);
        if (s_armedForSkip) {
            snprintf(text, sizeof(text),
                     "FINISH WITHOUT BACKUP?\nPress END again within %d s.\n", left);
        } else {
            snprintf(text, sizeof(text),
                     "END OF FLIGHT?\nUSB drive: %s\nPress END again within %d s.\n", s_driveText, left);
        }
    } else if (s_startedAtMs != 0 && now - s_startedAtMs < END_FLIGHT_STARTING_MS &&
               kind != KIND_RUNNING && !(kind == KIND_WAITING && !s_startedSkip)) {
        snprintf(text, sizeof(text), "%s", s_startedSkip ? "END OF FLIGHT\nFinishing without backup...\n"
                                                         : "END OF FLIGHT\nStarting...\n");
    } else if (kind != KIND_NONE) {
        snprintf(text, sizeof(text), "%s", body);
    } else {
        show = false;
    }
    if (s_armedAtMs != 0 && now - s_armedAtMs > END_FLIGHT_CONFIRM_MS) {
        s_armedAtMs = 0;
    }
    pthread_mutex_unlock(&s_mutex);

    if (show) {
        CopyCrLf(out, outLen, text);
    }
    return show;
}
