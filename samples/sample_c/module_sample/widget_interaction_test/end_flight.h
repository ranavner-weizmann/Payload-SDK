/**
 * End of flight from the remote controller.
 *
 * The widget's END button asks the MAMP dashboard on this Pi to end the
 * flight: check the instruments, stop Run All, back up to the USB drive,
 * eject it, save to the SD card, and finally stop PSDK. It must be pressed
 * twice within END_FLIGHT_CONFIRM_MS, and does nothing while the drone is in
 * the air (or its flight status can't be read). The dashboard writes its
 * progress to END_FLIGHT_STATUS_PATH, and the data box shows that instead of
 * the vitals.
 */
#ifndef END_FLIGHT_H
#define END_FLIGHT_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define END_FLIGHT_WIDGET_INDEX   (4)
#define END_FLIGHT_CONFIRM_MS     (3000)
#define END_FLIGHT_STATUS_PATH    "/home/rsp/mamp_pi_dashboard/data/end_flight_rc.txt"
#define END_FLIGHT_URL            "http://127.0.0.1:5050/end_of_flight"

/* Call on each press of the END button (value 1). */
void EndFlight_OnButtonPress(void);

/* Fills out with the text the data box should show instead of the vitals,
 * with \r\n line ends. Returns false when the vitals should show. */
bool EndFlight_GetMessage(char *out, size_t outLen);

#ifdef __cplusplus
}
#endif

#endif
