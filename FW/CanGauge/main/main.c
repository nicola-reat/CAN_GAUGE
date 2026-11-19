/*
 * main.c - entry point of the can_gauge firmware (ESP32-S3)
 * ==========================================================
 *
 * What this file does
 *   app_main() brings the board up in this order:
 *     - analog sensor and CAN node, so the first frame on the display and on
 *       the bus already carries the saved settings and a real reading;
 *     - round display (ST77916) and LVGL, with the backlight still off;
 *     - user interface: static layout from SquareLine (ui_init) and live
 *       content (gauge_display_start), first frame drawn at once;
 *     - backlight fade-in, so the panel memory is never visible at power on,
 *       then the power-on sweep of the ring (gauge_display_sweep);
 *     - check of a new firmware (app rollback): it is confirmed only when CAN,
 *       sensor and display are working, otherwise the previous one is restored.
 *
 *   board_identify() is also defined here: it is called by can_node.c when
 *   the "Identify board" button of BusWrench is pressed.
 *
 * Modules
 *   can_node.c       CAN protocol with BusWrench (scan, update, details,
 *                    actions, parameters) and periodic sensor frame
 *   adc_sensor.c     reading, filtering and calibration of the 0-5 V input
 *   gauge_cfg.c      name, unit, decimals and fault mode of the display
 *   gauge_alarm.c    low, very low and high alarms
 *   gauge_display.c  ring of segments, values and colors on the screen
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"          // vTaskDelay()
#include "esp_log.h"
#include "esp_ota_ops.h"            // OTA partition state, rollback confirmation
#include "esp_lvgl_port.h"          // LVGL task, lock and display glue for ESP-IDF
#include "lvgl.h"
#include "ST77916.h"                // display driver: LCD_Init(), panel handles, backlight
#include "ui.h"                     // SquareLine export: ui_init()
#include "can_node.h"               // can_node_start(), can_node_bus_ok()
#include "adc_sensor.h"             // adc_sensor_start(), adc_sensor_get_mv()
#include "gauge_cfg.h"              // gauge_cfg_get_name(), shown by board_identify()
#include "gauge_display.h"          // gauge_display_start(), gauge_display_sweep()

static const char *TAG = "main";

#define CONFIRM_TIMEOUT_MS 120000   // time given to a new firmware to prove it works
#define CONFIRM_POLL_MS    500      // how often the conditions are checked
#define FIRST_FRAME_MS     100      // time left to the LVGL task for the first frame
#define FADE_STEP          4        // backlight step of the fade-in, percent
#define FADE_STEP_MS       10       // time per step (one FreeRTOS tick at 100 Hz)

/* ---------- Identify (BusWrench "Identify board") ---------- */

// White layer shown on top of every screen while the board identifies
// itself. Created at the first call, then only shown and hidden.
static lv_obj_t *s_ident_layer = NULL;
static lv_obj_t *s_ident_name  = NULL;

// Set by app_main() once LVGL and the user interface exist. The CAN node is
// started before the display, so an identify request may arrive earlier.
static volatile bool s_ui_ready = false;

// Shows (on = true) or hides (on = false) the identify layer.
// The blink timing (8 times in 4 s) is handled by can_node.c, which calls
// this function from its own task: the LVGL lock is therefore required.
// It replaces the empty weak version defined in can_node.c.
void board_identify(bool on)
{
    if (!s_ui_ready)
        return;                                             // display not started yet

    lvgl_port_lock(0);                                      // 0 = wait as long as needed

    if (!s_ident_layer) {
        // lv_layer_top() is drawn above every screen: the layer covers the
        // gauge without changing it.
        s_ident_layer = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(s_ident_layer);
        lv_obj_set_size(s_ident_layer, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(s_ident_layer, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(s_ident_layer, LV_OPA_COVER, 0);
        lv_obj_remove_flag(s_ident_layer, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title = lv_label_create(s_ident_layer);
        lv_label_set_text(title, "IDENTIFY");
        lv_obj_set_style_text_color(title, lv_color_black(), 0);
        lv_obj_align(title, LV_ALIGN_CENTER, 0, -24);

        // Name of the gauge, so the board can be told apart from the others.
        s_ident_name = lv_label_create(s_ident_layer);
        lv_obj_set_style_text_color(s_ident_name, lv_color_black(), 0);
        lv_obj_set_style_text_font(s_ident_name, &lv_font_montserrat_26, 0);
        lv_obj_align(s_ident_name, LV_ALIGN_CENTER, 0, 12);
    }

    if (on) {
        char name[GAUGE_NAME_MAX + 1];
        gauge_cfg_get_name(name, sizeof name);              // the name may have changed
        lv_label_set_text(s_ident_name, name);
        lv_obj_remove_flag(s_ident_layer, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ident_layer, LV_OBJ_FLAG_HIDDEN);
    }

    lvgl_port_unlock();
}

/* ---------- New firmware check (app rollback) ---------- */

// True when the board works well enough to be updated again over CAN:
// CAN running and at least one frame received, sensor task reading (a sensor
// fault is fine: the task runs), user interface started.
static bool board_ok(void)
{
    return can_node_bus_ok() && adc_sensor_get_mv() >= 0 && s_ui_ready;
}

// After an OTA update the new firmware starts in the "pending verify" state.
// It is confirmed only once board_ok() is true. If that does not happen within
// CONFIRM_TIMEOUT_MS the firmware marks itself invalid and reboots: the
// bootloader starts the previous firmware again, so a firmware that cannot
// talk on the bus never stays on the board.
// Nothing is done after a USB flash or when the firmware is already confirmed.
static void check_new_firmware(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t state;

    if (esp_ota_get_state_partition(run, &state) != ESP_OK ||
        state != ESP_OTA_IMG_PENDING_VERIFY)
        return;

    ESP_LOGI(TAG, "New firmware on trial: waiting for CAN, sensor and display (max %d s)",
             CONFIRM_TIMEOUT_MS / 1000);

    for (int t = 0; t < CONFIRM_TIMEOUT_MS; t += CONFIRM_POLL_MS) {
        if (board_ok()) {
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "New firmware confirmed after %d ms", t);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIRM_POLL_MS));
    }

    ESP_LOGE(TAG, "New firmware not working (CAN %s, sensor %s, display %s): rollback",
             can_node_bus_ok() ? "ok" : "NO", adc_sensor_get_mv() >= 0 ? "ok" : "NO",
             s_ui_ready ? "ok" : "NO");
    esp_ota_mark_app_invalid_rollback_and_reboot();     // does not return
}

/* ---------- Entry point ---------- */

void app_main(void)
{
    // Sensor and CAN first: both load their settings from the NVS, so the
    // display starts with the saved name and, shortly after, a real reading.
    adc_sensor_start();
    can_node_start();               // loads name, unit, alarms; starts the CAN task

    // Display hardware: QSPI bus, reset, panel configuration. The backlight
    // stays off until the first frame has been drawn.
    LCD_Init();

    // LVGL with the default configuration: this also creates the LVGL task,
    // which draws the screen and runs the LVGL timers and animations.
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = lcd_get_panel_io_handle(),
        .panel_handle  = lcd_get_panel_handle(),
        .buffer_size   = EXAMPLE_LCD_WIDTH * 40,            // draw buffer: 40 lines
        .double_buffer = true,                              // draw one buffer while the other is sent
        .hres          = EXAMPLE_LCD_WIDTH,
        .vres          = EXAMPLE_LCD_HEIGHT,
        .monochrome    = false,
        .color_format  = LV_COLOR_FORMAT_RGB565,
        .rotation      = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        .flags         = { .buff_dma   = true,              // buffers in DMA capable memory
                           .swap_bytes = true },            // byte order of this panel
    };
    if (lvgl_port_add_disp(&disp_cfg) == NULL)
        ESP_LOGE(TAG, "Display not added to LVGL");

    // User interface. The lock is needed because the LVGL task is already
    // running: it draws the first frame as soon as the lock is released.
    lvgl_port_lock(0);
    ui_init();
    gauge_display_start();
    lvgl_port_unlock();
    s_ui_ready = true;

    // Give the LVGL task time to draw the first frame and send it to the
    // panel, then switch the backlight on with a short fade-in.
    vTaskDelay(pdMS_TO_TICKS(FIRST_FRAME_MS));
    for (int b = 0; b < LCD_Backlight; b += FADE_STEP) {
        Set_Backlight(b);
        vTaskDelay(pdMS_TO_TICKS(FADE_STEP_MS));
    }
    Set_Backlight(LCD_Backlight);
    ESP_LOGI(TAG, "Backlight on (%u %%)", LCD_Backlight);

    // Power-on sweep of the ring, now that the screen is visible.
    lvgl_port_lock(0);
    gauge_display_sweep();
    lvgl_port_unlock();

    // A new firmware is confirmed only when it proves it works (see above).
    check_new_firmware();

    // app_main() may return: ESP-IDF deletes the main task and frees its
    // stack, while the CAN, sensor and LVGL tasks keep running.
}