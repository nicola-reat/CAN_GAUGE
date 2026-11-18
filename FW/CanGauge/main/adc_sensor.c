/*
 * ADC reading of the sensor input (S1_IN on connector J2, pin 1).
 *
 * Hardware path (see the schematic, sheet "Sensor"):
 *   S1_IN -> R13 15k -> (+) of op-amp U4, with R14 10k to GND  => divide by 0.4
 *   op-amp gain = 1 + R10/R11 = 1 + 10k/15k = 1.667
 *   total = 0.667, so 0..5 V at the connector becomes 0..3.33 V at the GPIO4.
 * To get the voltage at the connector the voltage at the pin is multiplied by 3/2.
 *
 * Limit: with 12 dB attenuation the ADC of the ESP32-S3 measures up to about
 * 3.1 V, so inputs above about 4.65 V saturate.
 *
 * Processing chain, every SENS_PERIOD_MS:
 *   64 ADC samples -> average -> mV at the pin -> mV at the connector
 *   -> fault check -> moving average (time set by the user)
 *   -> calibration table -> value
 *
 * Settings (filter time, fault limits, calibration table)
 *   At start-up they are the factory values; if a saved set is found in the NVS
 *   it replaces them. They can be changed at run time (adc_sensor_set_...) and
 *   they are written to the NVS only by adc_sensor_save().
 */

#include "adc_sensor.h"

#include <string.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"          /* nvs_flash_init() */
#include "nvs.h"                /* nvs_open(), nvs_get_..., nvs_set_... */

static const char *TAG = "adc_sensor";

#define SENS_ADC_UNIT    ADC_UNIT_1
#define SENS_ADC_CHAN    ADC_CHANNEL_3     /* GPIO4 on the ESP32-S3 */
#define SENS_ADC_ATTEN   ADC_ATTEN_DB_12   /* range about 0 ... 3.1 V */

#define SENS_SAMPLES     64                /* samples averaged for one reading */
#define SENS_PERIOD_MS   50                /* time between two readings */
#define SENS_LOG_EVERY   20                /* log every 20 readings = 1 s */

/* Voltage at the connector = voltage at the pin * NUM / DEN (1 / 0.667). */
#define SENS_SCALE_NUM   3
#define SENS_SCALE_DEN   2

/* Moving average: from 1 reading (50 ms) up to 200 readings (10 s). */
#define SENS_MAX_WINDOW  200
#define SENS_DEFAULT_FILTER_MS  1000

/* Fault detection: default limits (mV at the connector) and the number of
 * readings in a row (10 x 50 ms = 0.5 s) needed to start or to end a fault. */
#define SENS_DEFAULT_LOW_MV   250
#define SENS_DEFAULT_HIGH_MV  4600
#define SENS_FAULT_READINGS   10
#define SENS_RECOVER_READINGS 10

/* NVS: everything of this file is stored in the namespace "sensor". */
#define SENS_NVS_NAMESPACE   "sensor"

/* Factory calibration table: voltage (mV) -> value in tenths of the unit.
 * Points must be in increasing order of voltage. The values can go down
 * if the sensor works the other way (full tank = low voltage).
 * THESE ARE EXAMPLE VALUES: measure the real tank (known liters -> voltage). */
static const adc_cal_point_t DEFAULT_TABLE[] = {
    {  500,   0 },
    { 1200,  80 },
    { 2000, 200 },
    { 2800, 340 },
    { 3600, 470 },
    { 4500, 600 },
};
#define DEFAULT_TABLE_N  ((int)(sizeof(DEFAULT_TABLE) / sizeof(DEFAULT_TABLE[0])))

/* The table in use. It is read by the ADC task and changed by the CAN task,
 * so every access goes through s_table_lock (a very short critical section). */
static adc_cal_point_t s_table[ADC_SENSOR_MAX_POINTS];
static int s_table_n = 0;                          /* number of points in use */
static portMUX_TYPE s_table_lock = portMUX_INITIALIZER_UNLOCKED;

static adc_oneshot_unit_handle_t s_adc;    /* ADC unit */
static adc_cali_handle_t         s_cali;   /* factory calibration (raw -> mV) */

static volatile int s_mv = -1;             /* last reading at the connector, mV */
static volatile int s_mv_filtered = -1;    /* after the moving average, mV */
static volatile int s_value_x10 = -1;      /* value from the table */
static volatile adc_fault_t s_fault = ADC_FAULT_NONE;   /* state of the sensor */
static volatile int s_filter_ms = SENS_DEFAULT_FILTER_MS;                 /* filter time */
static volatile int s_window = SENS_DEFAULT_FILTER_MS / SENS_PERIOD_MS;   /* same, in readings */
static volatile int s_low_mv  = SENS_DEFAULT_LOW_MV;    /* fault limits */
static volatile int s_high_mv = SENS_DEFAULT_HIGH_MV;

int adc_sensor_get_mv(void)          { return s_mv; }
int adc_sensor_get_filtered_mv(void) { return s_mv_filtered; }
int adc_sensor_get_value_x10(void)   { return s_value_x10; }
adc_fault_t adc_sensor_get_fault(void) { return s_fault; }

/* ------------------------------------------------------------------------
 * Settings
 * ------------------------------------------------------------------------ */

void adc_sensor_set_filter_ms(int ms)
{
    int n = ms / SENS_PERIOD_MS;                 /* time -> number of readings */
    if (n < 1) n = 1;
    if (n > SENS_MAX_WINDOW) n = SENS_MAX_WINDOW;
    s_filter_ms = n * SENS_PERIOD_MS;            /* the value really used */
    s_window = n;                                /* the task notices the change */
}

int adc_sensor_get_filter_ms(void) { return s_filter_ms; }

esp_err_t adc_sensor_set_fault_limits(int low_mv, int high_mv)
{
    /* Both inside 0..5000 mV and the low one under the high one. */
    if (low_mv < 0 || high_mv > 5000 || low_mv >= high_mv)
        return ESP_ERR_INVALID_ARG;
    s_low_mv = low_mv;
    s_high_mv = high_mv;
    return ESP_OK;
}

void adc_sensor_get_fault_limits(int *low_mv, int *high_mv)
{
    if (low_mv)  *low_mv  = s_low_mv;
    if (high_mv) *high_mv = s_high_mv;
}

/* A table is valid with 2..16 points and strictly increasing voltages
 * (two equal voltages would make a division by zero in the interpolation). */
static bool table_valid(const adc_cal_point_t *p, int n)
{
    if (!p || n < 2 || n > ADC_SENSOR_MAX_POINTS)
        return false;
    for (int i = 1; i < n; i++)
        if (p[i].mv <= p[i - 1].mv)
            return false;
    return true;
}

esp_err_t adc_sensor_set_table(const adc_cal_point_t *pts, int n)
{
    if (!table_valid(pts, n))
        return ESP_ERR_INVALID_ARG;             /* the old table stays in use */

    portENTER_CRITICAL(&s_table_lock);
    memcpy(s_table, pts, sizeof(adc_cal_point_t) * n);
    s_table_n = n;
    portEXIT_CRITICAL(&s_table_lock);
    return ESP_OK;
}

int adc_sensor_get_table_n(void) { return s_table_n; }

bool adc_sensor_get_point(int i, int *mv, int *val_x10)
{
    bool ok = false;
    portENTER_CRITICAL(&s_table_lock);
    if (i >= 0 && i < s_table_n) {
        if (mv)      *mv      = s_table[i].mv;
        if (val_x10) *val_x10 = s_table[i].val_x10;
        ok = true;
    }
    portEXIT_CRITICAL(&s_table_lock);
    return ok;
}

/* Puts the factory values in use (it does not touch the NVS). */
static void apply_defaults(void)
{
    adc_sensor_set_filter_ms(SENS_DEFAULT_FILTER_MS);
    adc_sensor_set_fault_limits(SENS_DEFAULT_LOW_MV, SENS_DEFAULT_HIGH_MV);
    adc_sensor_set_table(DEFAULT_TABLE, DEFAULT_TABLE_N);
}

/* ------------------------------------------------------------------------
 * NVS: saving and loading
 * ------------------------------------------------------------------------ */

/* Reads the saved settings (if any) and puts them in use.
 * Every item is checked: a wrong or missing item is skipped and the factory
 * value stays. */
static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(SENS_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings: factory values");
        return;                                   /* first start: namespace not there yet */
    }

    int32_t v, lo, hi;
    if (nvs_get_i32(h, "filter_ms", &v) == ESP_OK)
        adc_sensor_set_filter_ms(v);

    if (nvs_get_i32(h, "low_mv", &lo) == ESP_OK &&
        nvs_get_i32(h, "high_mv", &hi) == ESP_OK)
        adc_sensor_set_fault_limits(lo, hi);

    /* The table is saved as one block of bytes: the whole array of points. */
    adc_cal_point_t buf[ADC_SENSOR_MAX_POINTS];
    size_t len = sizeof(buf);
    if (nvs_get_blob(h, "table", buf, &len) == ESP_OK &&
        len % sizeof(adc_cal_point_t) == 0)
        adc_sensor_set_table(buf, (int)(len / sizeof(adc_cal_point_t)));

    nvs_close(h);

    int l, hgh;
    adc_sensor_get_fault_limits(&l, &hgh);
    ESP_LOGI(TAG, "settings loaded: filter %d ms, fault limits %d..%d mV, table %d points",
             s_filter_ms, l, hgh, s_table_n);
}

esp_err_t adc_sensor_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SENS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    /* Copy of the table taken in one short step, then written at leisure. */
    adc_cal_point_t buf[ADC_SENSOR_MAX_POINTS];
    int n;
    portENTER_CRITICAL(&s_table_lock);
    n = s_table_n;
    memcpy(buf, s_table, sizeof(adc_cal_point_t) * n);
    portEXIT_CRITICAL(&s_table_lock);

    err = nvs_set_i32(h, "filter_ms", s_filter_ms);
    if (err == ESP_OK) err = nvs_set_i32(h, "low_mv", s_low_mv);
    if (err == ESP_OK) err = nvs_set_i32(h, "high_mv", s_high_mv);
    if (err == ESP_OK) err = nvs_set_blob(h, "table", buf, sizeof(adc_cal_point_t) * n);
    if (err == ESP_OK) err = nvs_commit(h);       /* nothing is really written before this */
    nvs_close(h);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "settings saved");
    else
        ESP_LOGE(TAG, "save failed (%s)", esp_err_to_name(err));
    return err;
}

esp_err_t adc_sensor_reset_defaults(void)
{
    apply_defaults();                             /* in use now */

    nvs_handle_t h;
    esp_err_t err = nvs_open(SENS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;
    err = nvs_erase_all(h);                       /* delete the saved settings */
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "factory values restored");
    return err;
}

/* ------------------------------------------------------------------------
 * Measurement
 * ------------------------------------------------------------------------ */

/* Voltage (mV) -> value (tenths of the unit), interpolating between the two
 * nearest points of the table. Outside the table the nearest end is used. */
static int mv_to_value_x10(int mv)
{
    /* Private copy of the table, so it cannot change while we use it. */
    adc_cal_point_t t[ADC_SENSOR_MAX_POINTS];
    int n;
    portENTER_CRITICAL(&s_table_lock);
    n = s_table_n;
    memcpy(t, s_table, sizeof(adc_cal_point_t) * n);
    portEXIT_CRITICAL(&s_table_lock);

    if (n < 2)
        return mv;                               /* no table: not usable */
    if (mv <= t[0].mv)
        return t[0].val_x10;                     /* below the first point */

    for (int i = 1; i < n; i++) {
        if (mv <= t[i].mv) {
            const adc_cal_point_t *a = &t[i - 1];   /* point below */
            const adc_cal_point_t *b = &t[i];       /* point above */
            /* a.value + (how far we are between the two) * (difference of values) */
            return a->val_x10 + (int)((long)(mv - a->mv) * (b->val_x10 - a->val_x10)
                                      / (b->mv - a->mv));
        }
    }
    return t[n - 1].val_x10;                     /* above the last point */
}

/* Reads the sensor every SENS_PERIOD_MS and updates the values. */
static void adc_task(void *arg)
{
    int count = 0;

    /* Moving average: a ring of the last readings and their sum. */
    static int ring[SENS_MAX_WINDOW];
    int cur_window = 0;          /* window length in use (0 = start again) */
    int idx = 0;                 /* where the next reading is written */
    int filled = 0;              /* how many readings the ring holds now */
    long sum = 0;                /* sum of the readings in the ring */

    /* Fault detection: counters of bad and good readings in a row. */
    int bad_count = 0;
    int good_count = 0;

    for (;;) {
        /* 1. Average many ADC samples: this reduces the noise of the ADC. */
        int raw_sum = 0;
        int n = 0;
        for (int i = 0; i < SENS_SAMPLES; i++) {
            int raw;
            if (adc_oneshot_read(s_adc, SENS_ADC_CHAN, &raw) == ESP_OK) {
                raw_sum += raw;
                n++;
            }
        }

        if (n > 0) {
            int raw_avg = raw_sum / n;
            int pin_mv = 0;                                   /* mV at the GPIO */
            adc_cali_raw_to_voltage(s_cali, raw_avg, &pin_mv);

            int mv = pin_mv * SENS_SCALE_NUM / SENS_SCALE_DEN;/* mV at the connector */
            s_mv = mv;

            /* 2. Fault check. A fault starts after SENS_FAULT_READINGS bad
             *    readings in a row, and ends after SENS_RECOVER_READINGS good ones. */
            int low = s_low_mv;
            int high = s_high_mv;
            int bad = (mv < low) || (mv > high);

            if (bad) {
                good_count = 0;
                if (bad_count < SENS_FAULT_READINGS)
                    bad_count++;
                if (bad_count >= SENS_FAULT_READINGS)
                    s_fault = (mv < low) ? ADC_FAULT_LOW : ADC_FAULT_HIGH;
            } else {
                bad_count = 0;
                if (s_fault != ADC_FAULT_NONE) {
                    if (++good_count >= SENS_RECOVER_READINGS) {
                        s_fault = ADC_FAULT_NONE;     /* the sensor is back */
                        good_count = 0;
                        cur_window = 0;               /* start the average again */
                    }
                } else {
                    good_count = 0;
                }
            }

            /* Only good readings, and only without a fault, go into the average.
             * During a fault the value stays at the last valid one. */
            if (!bad && s_fault == ADC_FAULT_NONE) {

                /* 3. Moving average. If the length was changed, start again. */
                int win = s_window;
                if (win != cur_window) {
                    cur_window = win;
                    idx = 0;
                    filled = 0;
                    sum = 0;
                }
                if (filled == cur_window)
                    sum -= ring[idx];            /* the oldest reading leaves the ring */
                else
                    filled++;
                ring[idx] = mv;
                sum += mv;
                idx = (idx + 1) % cur_window;

                int avg_mv = (int)(sum / filled);
                s_mv_filtered = avg_mv;

                /* 4. Voltage -> value with the calibration table. */
                s_value_x10 = mv_to_value_x10(avg_mv);
            }

            if (++count >= SENS_LOG_EVERY) {
                count = 0;
                int val = s_value_x10;
                const char *state = (s_fault == ADC_FAULT_LOW)  ? "FAULT LOW"
                                  : (s_fault == ADC_FAULT_HIGH) ? "FAULT HIGH" : "ok";
                ESP_LOGI(TAG, "raw=%d pin=%d mV  sensor=%d mV  filtered=%d mV  value=%d.%d  %s",
                         raw_avg, pin_mv, mv, s_mv_filtered, val / 10,
                         (val < 0 ? -val : val) % 10, state);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SENS_PERIOD_MS));
    }
}

void adc_sensor_start(void)
{
    /* 0. Settings: factory values first, then the saved ones (if any).
     *    nvs_flash_init() is harmless if somebody else already called it. */
    apply_defaults();
    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();                        /* the NVS area is full or of an old format */
        nerr = nvs_flash_init();
    }
    if (nerr == ESP_OK)
        load_settings();
    else
        ESP_LOGE(TAG, "NVS not available (%s): factory values, nothing can be saved",
                 esp_err_to_name(nerr));

    /* 1. Start the ADC unit. */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = SENS_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    /* 2. Set the channel: attenuation and resolution (12 bit by default). */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = SENS_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SENS_ADC_CHAN, &chan_cfg));

    /* 3. Factory calibration: it turns the raw value into millivolts. */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = SENS_ADC_UNIT,
        .chan     = SENS_ADC_CHAN,
        .atten    = SENS_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC calibration not available (%s)", esp_err_to_name(err));
        return;
    }

    /* 4. Start the reading task. */
    xTaskCreate(adc_task, "adc_sensor", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "started on ADC1 channel 3 (GPIO4)");
}