#pragma once
/*
 * adc_sensor.h - 0-5 V sensor on the ADC, filtered, checked and converted by
 * a calibration table. The settings can be changed at run time and saved in
 * the NVS (permanent memory), so they survive a power off.
 */

#include <stdbool.h>
#include "esp_err.h"

/* State of the sensor. */
typedef enum {
    ADC_FAULT_NONE = 0,    /* sensor ok */
    ADC_FAULT_LOW,         /* voltage below the low limit (wire cut, short to ground) */
    ADC_FAULT_HIGH,        /* voltage above the high limit (short to 5 V) */
} adc_fault_t;

/* One point of the calibration table. */
typedef struct {
    int mv;          /* voltage at the connector, millivolts */
    int val_x10;     /* value in tenths of the unit (140 = 14.0 L) */
} adc_cal_point_t;

#define ADC_SENSOR_MAX_POINTS  16

/* Starts everything: NVS, loading of the saved settings, ADC and its task. */
void adc_sensor_start(void);

/* ---- Measures ---- */
int         adc_sensor_get_mv(void);            /* last reading at the connector, mV (-1 = none) */
int         adc_sensor_get_filtered_mv(void);   /* after the moving average, mV (-1 = none) */
int         adc_sensor_get_value_x10(void);     /* value from the table (-1 = none yet) */
adc_fault_t adc_sensor_get_fault(void);

/* ---- Settings: they take effect at once, but they are NOT saved ---- */
void      adc_sensor_set_filter_ms(int ms);                 /* 50 ... 10000 ms */
int       adc_sensor_get_filter_ms(void);
esp_err_t adc_sensor_set_fault_limits(int low_mv, int high_mv);   /* ESP_ERR_INVALID_ARG if not valid */
void      adc_sensor_get_fault_limits(int *low_mv, int *high_mv);
/* Replaces the whole table. It is refused (ESP_ERR_INVALID_ARG) if it has less
 * than 2 or more than 16 points, or if the voltages are not in increasing order. */
esp_err_t adc_sensor_set_table(const adc_cal_point_t *pts, int n);
int       adc_sensor_get_table_n(void);
bool      adc_sensor_get_point(int i, int *mv, int *val_x10);

/* ---- Permanent memory (NVS) ---- */
esp_err_t adc_sensor_save(void);              /* writes the current settings in the NVS */
esp_err_t adc_sensor_reset_defaults(void);    /* factory values, and erases the saved ones */