#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*
 * gauge_alarm.h - alarms of the gauge (threshold, hysteresis, delay)
 * ===================================================================
 *
 * Three alarms, each one with its own settings:
 *
 *   slot 0  LOW        low level            active when value < threshold
 *   slot 1  LOW_CRIT   very low level       active when value < threshold
 *   slot 2  HIGH       high level           active when value > threshold
 *
 *   enabled     the alarm is checked (1) or ignored (0)
 *   threshold   limit, in tenths of the unit (120 = 12.0)
 *   hysteresis  distance (tenths) the value must go back from the threshold
 *               before the alarm switches off. It avoids flicker around the limit:
 *               threshold 12.0, hysteresis 1.0 -> on below 12.0, off above 13.0.
 *   delay       tenths of a second the value must stay beyond the threshold
 *               before the alarm switches on (0 = at once)
 *
 * One more setting for all: for the first "startup" seconds after power on the
 * alarms are ignored (the value needs a moment to settle).
 *
 * While the sensor has no valid value (fault, no reading yet) all alarms are off.
 *
 * The state of the alarms goes in the flags of the sensor CAN frame, and the
 * display code (SquareLine) can read it with gauge_alarm_get_flags():
 *   bit1 = GAUGE_ALARM_FLAG_LOW, bit2 = GAUGE_ALARM_FLAG_HIGH,
 *   bit3 = GAUGE_ALARM_FLAG_LOW_CRIT  (bit0 is the sensor fault, not set here)
 *
 * Tasks: update(), the set functions, save and reset are called only by the CAN
 * task. get_flags() / is_active() may be called from any task.
 */

#define GAUGE_ALARM_SLOTS       3

#define GAUGE_ALARM_FLAG_LOW        0x02
#define GAUGE_ALARM_FLAG_HIGH       0x04
#define GAUGE_ALARM_FLAG_LOW_CRIT   0x08

// Loads the saved settings (the factory ones stay if nothing is saved).
esp_err_t gauge_alarm_start(void);

// Checks the three alarms: call it regularly (every 100 ms) with the last value.
// "valid" = the value is good (no fault, a reading exists).
void gauge_alarm_update(int value_x10, bool valid);

// Alarm flags for the CAN frame (bits above) and state of one alarm.
uint8_t gauge_alarm_get_flags(void);
bool    gauge_alarm_is_active(int slot);

// Settings. The set functions work at once (NOT saved) and return
// ESP_ERR_INVALID_ARG if the slot or the value is not valid.
int       gauge_alarm_get_enabled(int slot);
int       gauge_alarm_get_threshold(int slot);     // tenths
int       gauge_alarm_get_hysteresis(int slot);    // tenths
int       gauge_alarm_get_delay(int slot);         // tenths of a second
int       gauge_alarm_get_startup_s(void);         // seconds
esp_err_t gauge_alarm_set_enabled(int slot, int on);
esp_err_t gauge_alarm_set_threshold(int slot, int32_t x10);     // 0..32767
esp_err_t gauge_alarm_set_hysteresis(int slot, int32_t x10);    // 0..32767
esp_err_t gauge_alarm_set_delay(int slot, int32_t ds);          // 0..600
esp_err_t gauge_alarm_set_startup_s(int32_t s);                 // 0..60

// Writes the settings in the permanent memory / restores the factory ones
// (and erases the saved ones).
esp_err_t gauge_alarm_save(void);
esp_err_t gauge_alarm_reset_defaults(void);