#pragma once

/*
 * gauge_display.h - live content of the gauge screen
 * ===================================================
 *
 * The static layout of the screen (background, icon, labels) is designed in
 * SquareLine Studio and exported to components/ui. This module adds the parts
 * that change at run time:
 *   - the ring of segments along the border, which shows the level;
 *   - the texts: name, percentage, value with its unit, warning;
 *   - the colors, which follow the state of the alarms.
 *
 * The screen is refreshed periodically by an LVGL timer, reading the sensor
 * (adc_sensor), the alarms (gauge_alarm) and the settings (gauge_cfg). A
 * setting changed from BusWrench is therefore shown without a reboot.
 */

// Creates the ring and starts the periodic refresh of the screen. The ring
// stays off until gauge_display_sweep() is called.
// Call it once, after ui_init(), with the LVGL lock taken (lvgl_port_lock):
// it creates LVGL objects and an LVGL timer.
void gauge_display_start(void);

// Power-on test of the ring: fills it and empties it again, then hands it
// over to the normal refresh (level, alarm color or sensor fault).
// Call it once, after the backlight has been switched on, so the whole
// sweep is visible. The LVGL lock must be taken.
void gauge_display_sweep(void);