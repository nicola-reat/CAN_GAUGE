#pragma once
#include <stddef.h>
#include "esp_err.h"

/*
 * gauge_cfg.h - texts, display options and frame period of the gauge, saved in NVS
 * ====================================================================================
 *
 * The same firmware runs on every gauge: what makes one board "FUEL, in
 * liters" and another one "OIL PRESS, in bar" is stored here, in the permanent
 * memory, and changed from BusWrench.
 *
 *   name         text shown on the display (max 12 bytes)
 *   unit         unit of the value (max 4 bytes, "L", "bar", "°C" ...)
 *   decimals     digits after the point: 0, 1 or 2
 *   fault mode   what to show while the sensor is in fault:
 *                GAUGE_FAULT_DASHES = "- - -" blinking
 *                GAUGE_FAULT_LAST   = the last valid value
 *   period       time between two sensor frames on the CAN bus, in ms (20..500)
 *                (it is a CAN setting, kept here with the other saved values)
 *
 * The display code (SquareLine / LVGL) reads these values with the get functions.
 * The get functions copy the text, so they are safe to call from any task.
 */

#define GAUGE_NAME_MAX   12
#define GAUGE_UNIT_MAX   4

#define GAUGE_DEF_NAME        "FUEL"
#define GAUGE_DEF_UNIT        "L"
#define GAUGE_DEF_DECIMALS    1
#define GAUGE_DEF_FAULT_MODE  0
#define GAUGE_DEF_PERIOD_MS   100    // period of the sensor CAN frame
#define GAUGE_PERIOD_MIN_MS   20
#define GAUGE_PERIOD_MAX_MS   500

typedef enum { GAUGE_FAULT_DASHES = 0, GAUGE_FAULT_LAST = 1 } gauge_fault_mode_t;

// Loads the saved values (the defaults stay if nothing is saved).
esp_err_t gauge_cfg_start(void);

// Copy the text in "out" (always ends with 0).
void gauge_cfg_get_name(char *out, size_t n);
void gauge_cfg_get_unit(char *out, size_t n);
int  gauge_cfg_get_decimals(void);
int  gauge_cfg_get_fault_mode(void);
int  gauge_cfg_get_period_ms(void);        // period of the sensor CAN frame

// Change a value (used at once, NOT saved). ESP_ERR_INVALID_ARG if not valid.
esp_err_t gauge_cfg_set_name(const char *s);
esp_err_t gauge_cfg_set_unit(const char *s);
esp_err_t gauge_cfg_set_decimals(int d);
esp_err_t gauge_cfg_set_fault_mode(int m);
esp_err_t gauge_cfg_set_period_ms(int ms); // GAUGE_PERIOD_MIN_MS..GAUGE_PERIOD_MAX_MS

// Writes the current values in the permanent memory.
esp_err_t gauge_cfg_save(void);

// Factory values; the saved ones are erased.
esp_err_t gauge_cfg_reset_defaults(void);