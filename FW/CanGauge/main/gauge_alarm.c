/*
 * gauge_alarm.c - alarms of the gauge (see gauge_alarm.h)
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "gauge_alarm.h"

static const char *TAG = "gauge_alarm";

#define NVS_NS        "alarm"
#define NVS_KEY       "cfg"
#define CFG_VERSION   1          // change it if the struct below changes

typedef struct {
    uint8_t enabled;
    int32_t thr_x10;
    int32_t hyst_x10;
    int32_t delay_ds;
} slot_cfg_t;

// This is what is saved in the NVS (one block).
typedef struct {
    uint8_t    version;
    uint8_t    startup_s;
    slot_cfg_t slot[GAUGE_ALARM_SLOTS];
} stored_t;

// Factory values: LOW 12.0, LOW_CRIT 6.0 (both on), HIGH 60.0 (off);
// hysteresis 1.0, delay 2.0 s, ignore the first 5 seconds.
#define DEFAULTS_INIT { CFG_VERSION, 5, { {1, 120, 10, 20}, {1, 60, 10, 20}, {0, 600, 10, 20} } }

static stored_t s_cfg = DEFAULTS_INIT;

// Which way each alarm works, and which bit it sets in the flags.
static const bool    s_is_low[GAUGE_ALARM_SLOTS] = { true, true, false };
static const uint8_t s_bit[GAUGE_ALARM_SLOTS] = {
    GAUGE_ALARM_FLAG_LOW, GAUGE_ALARM_FLAG_LOW_CRIT, GAUGE_ALARM_FLAG_HIGH
};

// Run-time state of each alarm.
static bool    s_active[GAUGE_ALARM_SLOTS];
static int64_t s_since_ms[GAUGE_ALARM_SLOTS] = { -1, -1, -1 };   // -1 = not beyond the limit
static volatile uint8_t s_flags;

static bool slot_ok(int slot) { return slot >= 0 && slot < GAUGE_ALARM_SLOTS; }

/* ---------- Check ---------- */

void gauge_alarm_update(int value_x10, bool valid)
{
    const int64_t now = esp_timer_get_time() / 1000;                 // ms since boot
    const bool ignoring = now < (int64_t)s_cfg.startup_s * 1000;
    uint8_t flags = 0;

    for (int i = 0; i < GAUGE_ALARM_SLOTS; i++) {
        const slot_cfg_t *c = &s_cfg.slot[i];

        // No valid value, start-up time or alarm disabled: switched off.
        if (!valid || ignoring || !c->enabled) {
            s_active[i] = false;
            s_since_ms[i] = -1;
            continue;
        }

        // "beyond" = the value is past the threshold; "back" = it came back
        // far enough (threshold + hysteresis) to switch the alarm off.
        bool beyond, back;
        if (s_is_low[i]) {
            beyond = value_x10 < c->thr_x10;
            back   = value_x10 >= c->thr_x10 + c->hyst_x10;
        } else {
            beyond = value_x10 > c->thr_x10;
            back   = value_x10 <= c->thr_x10 - c->hyst_x10;
        }

        if (!s_active[i]) {
            if (beyond) {
                if (s_since_ms[i] < 0)
                    s_since_ms[i] = now;                          // starts counting the delay
                if (now - s_since_ms[i] >= (int64_t)c->delay_ds * 100)
                    s_active[i] = true;
            } else {
                s_since_ms[i] = -1;                               // it was only a short spike
            }
        } else if (back) {
            s_active[i] = false;
            s_since_ms[i] = -1;
        }

        if (s_active[i])
            flags |= s_bit[i];
    }
    s_flags = flags;
}

uint8_t gauge_alarm_get_flags(void) { return s_flags; }
bool gauge_alarm_is_active(int slot) { return slot_ok(slot) && s_active[slot]; }

/* ---------- Get / set ---------- */

int gauge_alarm_get_enabled(int slot)    { return slot_ok(slot) ? s_cfg.slot[slot].enabled : 0; }
int gauge_alarm_get_threshold(int slot)  { return slot_ok(slot) ? (int)s_cfg.slot[slot].thr_x10 : 0; }
int gauge_alarm_get_hysteresis(int slot) { return slot_ok(slot) ? (int)s_cfg.slot[slot].hyst_x10 : 0; }
int gauge_alarm_get_delay(int slot)      { return slot_ok(slot) ? (int)s_cfg.slot[slot].delay_ds : 0; }
int gauge_alarm_get_startup_s(void)      { return s_cfg.startup_s; }

esp_err_t gauge_alarm_set_enabled(int slot, int on)
{
    if (!slot_ok(slot) || (on != 0 && on != 1)) return ESP_ERR_INVALID_ARG;
    s_cfg.slot[slot].enabled = (uint8_t)on;
    return ESP_OK;
}

esp_err_t gauge_alarm_set_threshold(int slot, int32_t x10)
{
    if (!slot_ok(slot) || x10 < 0 || x10 > 32767) return ESP_ERR_INVALID_ARG;
    s_cfg.slot[slot].thr_x10 = x10;
    return ESP_OK;
}

esp_err_t gauge_alarm_set_hysteresis(int slot, int32_t x10)
{
    if (!slot_ok(slot) || x10 < 0 || x10 > 32767) return ESP_ERR_INVALID_ARG;
    s_cfg.slot[slot].hyst_x10 = x10;
    return ESP_OK;
}

esp_err_t gauge_alarm_set_delay(int slot, int32_t ds)
{
    if (!slot_ok(slot) || ds < 0 || ds > 600) return ESP_ERR_INVALID_ARG;
    s_cfg.slot[slot].delay_ds = ds;
    return ESP_OK;
}

esp_err_t gauge_alarm_set_startup_s(int32_t s)
{
    if (s < 0 || s > 60) return ESP_ERR_INVALID_ARG;
    s_cfg.startup_s = (uint8_t)s;
    return ESP_OK;
}

/* ---------- Permanent memory (NVS) ---------- */

// Reads the saved block. If it is missing or from another version, the
// factory values stay.
static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return;

    stored_t t;
    size_t len = sizeof(t);
    if (nvs_get_blob(h, NVS_KEY, &t, &len) == ESP_OK &&
        len == sizeof(t) && t.version == CFG_VERSION) {
        s_cfg = t;
        ESP_LOGI(TAG, "Loaded alarms (startup %d s)", s_cfg.startup_s);
    }
    nvs_close(h);
}

esp_err_t gauge_alarm_start(void)
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

esp_err_t gauge_alarm_save(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;
    e = nvs_set_blob(h, NVS_KEY, &s_cfg, sizeof(s_cfg));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

esp_err_t gauge_alarm_reset_defaults(void)
{
    static const stored_t def = DEFAULTS_INIT;
    s_cfg = def;
    for (int i = 0; i < GAUGE_ALARM_SLOTS; i++) {     // all off until the next check
        s_active[i] = false;
        s_since_ms[i] = -1;
    }

    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;
    e = nvs_erase_all(h);                              // only this namespace
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}