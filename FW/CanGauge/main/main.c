/*
 * main.c - entry point of the can_gauge firmware (ESP32-S3).
 *
 * What this file does:
 *   1. starts the round display (ST77916) and the LVGL graphics library;
 *   2. shows the user interface designed with SquareLine / LVGL (ui_init);
 *   3. starts the CAN node, which answers to the PC (scan, firmware update,
 *      details, reboot, identify, rollback, parameters) and sends the sensor
 *      value - see can_node.c;
 *   4. starts the reading of the analog sensor - see adc_sensor.c;
 *   5. confirms the new firmware after a few seconds (rollback protection);
 *   6. defines board_identify(), used by the "Identify board" button of BusWrench.
 */

#include "freertos/FreeRTOS.h"      // FreeRTOS: the operating system of the ESP32
#include "freertos/task.h"          // vTaskDelay()
#include "esp_lvgl_port.h"          // glue between ESP-IDF and LVGL (lock, display)
#include "lvgl.h"                   // LVGL: the graphics library
#include "ST77916.h"                // driver of the display (LCD_Init, panel handles)
#include "ui.h"                     // the user interface (ui_init)
#include "can_node.h"               // CAN node (can_node_start)
#include "esp_ota_ops.h"            // esp_ota_mark_app_valid_cancel_rollback()
#include "esp_log.h"                // ESP_LOGI
#include "adc_sensor.h"             // analog sensor (adc_sensor_start)

// ---- IDENTIFY command: blinks the screen so the board can be recognized ----
// A white layer with a text is shown on top of the normal graphics and hidden
// again, 8 times in 4 seconds. It is called by can_node.c (it replaces the
// empty "weak" version defined there). The timing (every 250 ms) is decided
// in can_node.c: this function only has to show or hide the layer.

// The white layer. It is NULL until the first call, then it is reused:
// it is created once and only shown / hidden, never deleted.
static lv_obj_t *s_ident_layer = NULL;

// ---- Confirmation of a new firmware (app rollback) ----
// After an update the new firmware starts "on trial". If the board resets
// before this function is called, the bootloader throws the new firmware away
// and starts the previous one. So a firmware that crashes at startup can never
// leave the board stuck.
static void confirm_firmware(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t state;

    // Only a firmware that is "on trial" must be confirmed. After a flash via
    // USB, or if it was already confirmed, there is nothing to do.
    if (esp_ota_get_state_partition(run, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI("main", "New firmware confirmed");
    }
}

// on = true  -> show the white layer
// on = false -> hide it (the normal graphics are visible again)
void board_identify(bool on)
{
    // LVGL is not thread safe: only one task at a time can use it.
    // This function runs in the timer task, while LVGL has its own task,
    // so we must take the lock before touching any LVGL object.
    lvgl_port_lock(0);                     // 0 = wait as long as needed

    // First call: build the layer.
    if (!s_ident_layer) {
        // lv_layer_top() is a special layer that is always drawn above every
        // screen: the layer covers the normal graphics without changing them.
        s_ident_layer = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(s_ident_layer);                 // no border, no shadow

        // Cover the whole display (100 % of width and height).
        lv_obj_set_size(s_ident_layer, LV_PCT(100), LV_PCT(100));

        // White background, fully opaque (otherwise the graphics show through).
        lv_obj_set_style_bg_color(s_ident_layer, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(s_ident_layer, LV_OPA_COVER, 0);

        // Not scrollable: a touch on the display must not move it.
        lv_obj_remove_flag(s_ident_layer, LV_OBJ_FLAG_SCROLLABLE);

        // Black text in the middle of the screen.
        lv_obj_t *label = lv_label_create(s_ident_layer);
        lv_label_set_text(label, "THIS IS");
        lv_obj_set_style_text_color(label, lv_color_black(), 0);
        lv_obj_center(label);
    }

    // Show or hide the layer. HIDDEN is a flag of the object: with the flag
    // set the object is not drawn and does not react to the touch.
    if (on)
        lv_obj_remove_flag(s_ident_layer, LV_OBJ_FLAG_HIDDEN);  // show
    else
        lv_obj_add_flag(s_ident_layer, LV_OBJ_FLAG_HIDDEN);     // hide

    lvgl_port_unlock();                    // release LVGL for the other tasks
}

// Entry point: ESP-IDF calls app_main() once, after the system has started.
void app_main(void)
{

    // Start the display hardware: SPI/QSPI bus, reset, panel configuration.
    LCD_Init();

    //Start LVGL with the default configuration. This also creates the
    //task that draws the screen and handles the timers of the graphics.
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));     // stop with an error if it fails

    //Tell LVGL how the display is made (size, color format, buffers).
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_get_panel_io_handle(),     // connection to the display
        .panel_handle = lcd_get_panel_handle(),     // the display driver itself
        .buffer_size = EXAMPLE_LCD_WIDTH * 40,      // draw buffer: 40 lines at a time
        .double_buffer = true,                      // two buffers: draw while sending
        .hres = EXAMPLE_LCD_WIDTH,                  // horizontal resolution (pixels)
        .vres = EXAMPLE_LCD_HEIGHT,                 // vertical resolution (pixels)
        .monochrome = false,                        // color display
        .color_format = LV_COLOR_FORMAT_RGB565,     // 16 bits per pixel
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },  // no rotation
        .flags = { .buff_dma = true,                // buffers in DMA memory (faster)
                   .swap_bytes = true },            // byte order expected by this panel
    };
    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    (void)disp;                                     // not used later: avoids a warning

    //Create the user interface. It is done with the lock taken,
    //because from now on LVGL's own task is running too.
    lvgl_port_lock(0);
    ui_init();
    lvgl_port_unlock();

    //Start the CAN node (it creates its own task: from here on it answers
    //to the commands of BusWrench).
    can_node_start();          // starts CAN
    adc_sensor_start();        // starts the reading of the analog sensor
    // The display and the CAN node are running: wait a few seconds to be sure
    // that everything is stable, then confirm the firmware.
    vTaskDelay(pdMS_TO_TICKS(5000));
    confirm_firmware();


    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));            // sleep 1 second, then again
    }
}