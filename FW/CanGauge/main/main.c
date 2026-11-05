#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "ST77916.h"
#include "ui.h"

void app_main(void)
{
    LCD_Init();

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_get_panel_io_handle(),
        .panel_handle = lcd_get_panel_handle(),
        .buffer_size = EXAMPLE_LCD_WIDTH * 40,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_WIDTH,
        .vres = EXAMPLE_LCD_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
        .flags = { .buff_dma = true, .swap_bytes = true },
    };
    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    (void)disp;

 lvgl_port_lock(0);
    ui_init();
    lvgl_port_unlock();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}