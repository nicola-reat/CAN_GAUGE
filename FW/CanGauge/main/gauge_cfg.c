/*
 * gauge_cfg.c - texts, display options and frame period of the gauge (see gauge_cfg.h)
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "gauge_cfg.h"

static const char *TAG = "gauge_cfg";

#define NVS_NS   "gauge"     // namespace in the NVS (the sensor uses "sensor")

// The values in use. They are changed by the CAN task and read by the display
// task, so every access is protected by a short critical section.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_name[GAUGE_NAME_MAX + 1] = GAUGE_DEF_NAME;
static char s_unit[GAUGE_UNIT_MAX + 1] = GAUGE_DEF_UNIT;
static int  s_decimals   = GAUGE_DEF_DECIMALS;
static int  s_fault_mode = GAUGE_DEF_FAULT_MODE;
static int  s_period_ms  = GAUGE_DEF_PERIOD_MS;

// A text is valid if it is not longer than "max" bytes and has no control
// characters (bytes from 0x80 up are allowed: they are the parts of UTF-8 symbols).
static bool valid_text(const char *s, size_t max)
{
    size_t n = strlen(s);
    if (n > max)
        return false;
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)s[i] < 0x20 || (unsigned char)s[i] == 0x7F)
            return false;
    return true;
}

/* ---------- Get ---------- */

void gauge_cfg_get_name(char *out, size_t n)
{
    if (!out || n == 0) return;
    portENTER_CRITICAL(&s_mux);
    strncpy(out, s_name, n - 1);
    out[n - 1] = 0;
    portEXIT_CRITICAL(&s_mux);
}

void gauge_cfg_get_unit(char *out, size_t n)
{
    if (!out || n == 0) return;
    portENTER_CRITICAL(&s_mux);
    strncpy(out, s_unit, n - 1);
    out[n - 1] = 0;
    portEXIT_CRITICAL(&s_mux);
}

int gauge_cfg_get_decimals(void)   { return s_decimals; }
int gauge_cfg_get_fault_mode(void) { return s_fault_mode; }
int gauge_cfg_get_period_ms(void)  { return s_period_ms; }

/* ---------- Set (used at once, not saved) ---------- */

esp_err_t gauge_cfg_set_name(const char *s)
{
    if (!s || !valid_text(s, GAUGE_NAME_MAX)) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_mux);
    strcpy(s_name, s);
    portEXIT_CRITICAL(&s_mux);
    return ESP_OK;
}

esp_err_t gauge_cfg_set_unit(const char *s)
{
    if (!s || !valid_text(s, GAUGE_UNIT_MAX)) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_mux);
    strcpy(s_unit, s);
    portEXIT_CRITICAL(&s_mux);
    return ESP_OK;
}

esp_err_t gauge_cfg_set_decimals(int d)
{
    if (d < 0 || d > 2) return ESP_ERR_INVALID_ARG;
    s_decimals = d;
    return ESP_OK;
}

esp_err_t gauge_cfg_set_fault_mode(int m)
{
    if (m != GAUGE_FAULT_DASHES && m != GAUGE_FAULT_LAST) return ESP_ERR_INVALID_ARG;
    s_fault_mode = m;
    return ESP_OK;
}

esp_err_t gauge_cfg_set_period_ms(int ms)
{
    if (ms < GAUGE_PERIOD_MIN_MS || ms > GAUGE_PERIOD_MAX_MS) return ESP_ERR_INVALID_ARG;
    s_period_ms = ms;
    return ESP_OK;
}

/* ---------- Permanent memory (NVS) ---------- */

// Reads the saved values. Anything missing or not valid leaves the default.
static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return;                                   // nothing saved yet: defaults

    char name[GAUGE_NAME_MAX + 1];
    size_t len = sizeof(name);
    if (nvs_get_str(h, "name", name, &len) == ESP_OK)
        gauge_cfg_set_name(name);

    char unit[GAUGE_UNIT_MAX + 1];
    len = sizeof(unit);
    if (nvs_get_str(h, "unit", unit, &len) == ESP_OK)
        gauge_cfg_set_unit(unit);

    uint8_t v = 0;
    if (nvs_get_u8(h, "dec", &v) == ESP_OK)
        gauge_cfg_set_decimals(v);
    if (nvs_get_u8(h, "fmode", &v) == ESP_OK)
        gauge_cfg_set_fault_mode(v);

    uint16_t per = 0;
    if (nvs_get_u16(h, "per", &per) == ESP_OK)
        gauge_cfg_set_period_ms(per);

    nvs_close(h);
    ESP_LOGI(TAG, "Loaded: name '%s', unit '%s', %d decimals, fault mode %d, period %d ms",
             s_name, s_unit, s_decimals, s_fault_mode, s_period_ms);
}

esp_err_t gauge_cfg_start(void)
{
    // The NVS may be already started by another module: calling it twice is harmless.
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e != ESP_OK)
        return e;
    load_settings();
    return ESP_OK;
}

esp_err_t gauge_cfg_save(void)
{
    char name[GAUGE_NAME_MAX + 1], unit[GAUGE_UNIT_MAX + 1];
    gauge_cfg_get_name(name, sizeof(name));
    gauge_cfg_get_unit(unit, sizeof(unit));

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;
    e = nvs_set_str(h, "name", name);
    if (e == ESP_OK) e = nvs_set_str(h, "unit", unit);
    if (e == ESP_OK) e = nvs_set_u8(h, "dec", (uint8_t)s_decimals);
    if (e == ESP_OK) e = nvs_set_u8(h, "fmode", (uint8_t)s_fault_mode);
    if (e == ESP_OK) e = nvs_set_u16(h, "per", (uint16_t)s_period_ms);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t gauge_cfg_reset_defaults(void)
{
    gauge_cfg_set_name(GAUGE_DEF_NAME);
    gauge_cfg_set_unit(GAUGE_DEF_UNIT);
    gauge_cfg_set_decimals(GAUGE_DEF_DECIMALS);
    gauge_cfg_set_fault_mode(GAUGE_DEF_FAULT_MODE);
    gauge_cfg_set_period_ms(GAUGE_DEF_PERIOD_MS);

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;
    e = nvs_erase_all(h);                         // only this namespace
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}