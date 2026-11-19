/*
 * ST77916.c - driver of the 1.85" round display (ST77916, QSPI) and backlight
 * ============================================================================
 *
 * What this file does
 *   Brings up the display panel and its backlight. The drawing itself is done
 *   by LVGL through esp_lvgl_port, using the two handles exported here
 *   (lcd_get_panel_io_handle, lcd_get_panel_handle).
 *
 * Start-up sequence (LCD_Init)
 *   Panel   QSPI bus on SPI2, panel IO, identification of the panel version,
 *           reset, vendor initialization, display on.
 *   Light   PWM (LEDC) on the backlight pin, left OFF. app_main() switches it
 *           on with Set_Backlight() once the first frame has been drawn, so the
 *           random content of the panel memory is never visible at power on.
 *
 * Panel identification
 *   Two versions of the ST77916 module exist. Register 0x04 is read at a low
 *   clock (3 MHz) before the panel is configured:
 *     00 7F 7F 7F   default initialization of the esp_lcd_st77916 component
 *     00 02 7F 7F   vendor_specific_init_new (table below)
 *   The panel IO is then created again at the full clock (ESP_PANEL_LCD_SPI_CLK_HZ).
 *
 * Backlight
 *   LEDC low speed timer 0, 13-bit resolution, 5 kHz. Set_Backlight() takes a
 *   brightness in percent (0 = off, 100 = maximum). LCD_Backlight is the
 *   brightness used by app_main() at start-up.
 *
 * Origin
 *   Based on the Waveshare ESP32-S3 1.85" example, adapted to the pins of the
 *   CAN_GAUGE board (see ST77916.h).
 */

#include "ST77916.h"

// Opcodes of the QSPI protocol of the ST77916: they go in the top byte of the
// 32-bit command word (see ESP_PANEL_LCD_SPI_CMD_BITS).
#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

#define LCD_ID_CLK_HZ               (3 * 1000 * 1000)   // clock used to read the panel ID

static const char *TAG_LCD = "ST77916";

esp_lcd_panel_handle_t    panel_handle = NULL;      // panel driver, used by LVGL
esp_lcd_panel_io_handle_t io_handle    = NULL;      // QSPI connection to the panel

/* ---------- Handles for esp_lvgl_port ---------- */

// QSPI panel IO created by LCD_Init().
esp_lcd_panel_io_handle_t lcd_get_panel_io_handle(void)
{
    return io_handle;
}

// ST77916 panel driver created by LCD_Init().
esp_lcd_panel_handle_t lcd_get_panel_handle(void)
{
    return panel_handle;
}

/* ---------- Vendor initialization ---------- */

// Initialization commands for the panel version that answers 00 02 7F 7F to
// register 0x04. Each entry: command, data bytes, number of bytes, delay (ms)
// after the command. Values from the panel manufacturer: do not change them.
// The table ends with 0x21 (color inversion on), 0x11 (sleep out, 120 ms) and
// 0x29 (display on).
static const st77916_lcd_init_cmd_t vendor_specific_init_new[] = {
  {0xF0, (uint8_t []){0x28}, 1, 0},
  {0xF2, (uint8_t []){0x28}, 1, 0},
  {0x73, (uint8_t []){0xF0}, 1, 0},
  {0x7C, (uint8_t []){0xD1}, 1, 0},
  {0x83, (uint8_t []){0xE0}, 1, 0},
  {0x84, (uint8_t []){0x61}, 1, 0},
  {0xF2, (uint8_t []){0x82}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x01}, 1, 0},
  {0xF1, (uint8_t []){0x01}, 1, 0},
  {0xB0, (uint8_t []){0x56}, 1, 0},
  {0xB1, (uint8_t []){0x4D}, 1, 0},
  {0xB2, (uint8_t []){0x24}, 1, 0},
  {0xB4, (uint8_t []){0x87}, 1, 0},
  {0xB5, (uint8_t []){0x44}, 1, 0},
  {0xB6, (uint8_t []){0x8B}, 1, 0},
  {0xB7, (uint8_t []){0x40}, 1, 0},
  {0xB8, (uint8_t []){0x86}, 1, 0},
  {0xBA, (uint8_t []){0x00}, 1, 0},
  {0xBB, (uint8_t []){0x08}, 1, 0},
  {0xBC, (uint8_t []){0x08}, 1, 0},
  {0xBD, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x80}, 1, 0},
  {0xC1, (uint8_t []){0x10}, 1, 0},
  {0xC2, (uint8_t []){0x37}, 1, 0},
  {0xC3, (uint8_t []){0x80}, 1, 0},
  {0xC4, (uint8_t []){0x10}, 1, 0},
  {0xC5, (uint8_t []){0x37}, 1, 0},
  {0xC6, (uint8_t []){0xA9}, 1, 0},
  {0xC7, (uint8_t []){0x41}, 1, 0},
  {0xC8, (uint8_t []){0x01}, 1, 0},
  {0xC9, (uint8_t []){0xA9}, 1, 0},
  {0xCA, (uint8_t []){0x41}, 1, 0},
  {0xCB, (uint8_t []){0x01}, 1, 0},
  {0xD0, (uint8_t []){0x91}, 1, 0},
  {0xD1, (uint8_t []){0x68}, 1, 0},
  {0xD2, (uint8_t []){0x68}, 1, 0},
  {0xF5, (uint8_t []){0x00, 0xA5}, 2, 0},
  {0xDD, (uint8_t []){0x4F}, 1, 0},
  {0xDE, (uint8_t []){0x4F}, 1, 0},
  {0xF1, (uint8_t []){0x10}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0xF0, (uint8_t []){0x02}, 1, 0},
  {0xE0, (uint8_t []){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
  {0xE1, (uint8_t []){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
  {0xF0, (uint8_t []){0x10}, 1, 0},
  {0xF3, (uint8_t []){0x10}, 1, 0},
  {0xE0, (uint8_t []){0x07}, 1, 0},
  {0xE1, (uint8_t []){0x00}, 1, 0},
  {0xE2, (uint8_t []){0x00}, 1, 0},
  {0xE3, (uint8_t []){0x00}, 1, 0},
  {0xE4, (uint8_t []){0xE0}, 1, 0},
  {0xE5, (uint8_t []){0x06}, 1, 0},
  {0xE6, (uint8_t []){0x21}, 1, 0},
  {0xE7, (uint8_t []){0x01}, 1, 0},
  {0xE8, (uint8_t []){0x05}, 1, 0},
  {0xE9, (uint8_t []){0x02}, 1, 0},
  {0xEA, (uint8_t []){0xDA}, 1, 0},
  {0xEB, (uint8_t []){0x00}, 1, 0},
  {0xEC, (uint8_t []){0x00}, 1, 0},
  {0xED, (uint8_t []){0x0F}, 1, 0},
  {0xEE, (uint8_t []){0x00}, 1, 0},
  {0xEF, (uint8_t []){0x00}, 1, 0},
  {0xF8, (uint8_t []){0x00}, 1, 0},
  {0xF9, (uint8_t []){0x00}, 1, 0},
  {0xFA, (uint8_t []){0x00}, 1, 0},
  {0xFB, (uint8_t []){0x00}, 1, 0},
  {0xFC, (uint8_t []){0x00}, 1, 0},
  {0xFD, (uint8_t []){0x00}, 1, 0},
  {0xFE, (uint8_t []){0x00}, 1, 0},
  {0xFF, (uint8_t []){0x00}, 1, 0},
  {0x60, (uint8_t []){0x40}, 1, 0},
  {0x61, (uint8_t []){0x04}, 1, 0},
  {0x62, (uint8_t []){0x00}, 1, 0},
  {0x63, (uint8_t []){0x42}, 1, 0},
  {0x64, (uint8_t []){0xD9}, 1, 0},
  {0x65, (uint8_t []){0x00}, 1, 0},
  {0x66, (uint8_t []){0x00}, 1, 0},
  {0x67, (uint8_t []){0x00}, 1, 0},
  {0x68, (uint8_t []){0x00}, 1, 0},
  {0x69, (uint8_t []){0x00}, 1, 0},
  {0x6A, (uint8_t []){0x00}, 1, 0},
  {0x6B, (uint8_t []){0x00}, 1, 0},
  {0x70, (uint8_t []){0x40}, 1, 0},
  {0x71, (uint8_t []){0x03}, 1, 0},
  {0x72, (uint8_t []){0x00}, 1, 0},
  {0x73, (uint8_t []){0x42}, 1, 0},
  {0x74, (uint8_t []){0xD8}, 1, 0},
  {0x75, (uint8_t []){0x00}, 1, 0},
  {0x76, (uint8_t []){0x00}, 1, 0},
  {0x77, (uint8_t []){0x00}, 1, 0},
  {0x78, (uint8_t []){0x00}, 1, 0},
  {0x79, (uint8_t []){0x00}, 1, 0},
  {0x7A, (uint8_t []){0x00}, 1, 0},
  {0x7B, (uint8_t []){0x00}, 1, 0},
  {0x80, (uint8_t []){0x48}, 1, 0},
  {0x81, (uint8_t []){0x00}, 1, 0},
  {0x82, (uint8_t []){0x06}, 1, 0},
  {0x83, (uint8_t []){0x02}, 1, 0},
  {0x84, (uint8_t []){0xD6}, 1, 0},
  {0x85, (uint8_t []){0x04}, 1, 0},
  {0x86, (uint8_t []){0x00}, 1, 0},
  {0x87, (uint8_t []){0x00}, 1, 0},
  {0x88, (uint8_t []){0x48}, 1, 0},
  {0x89, (uint8_t []){0x00}, 1, 0},
  {0x8A, (uint8_t []){0x08}, 1, 0},
  {0x8B, (uint8_t []){0x02}, 1, 0},
  {0x8C, (uint8_t []){0xD8}, 1, 0},
  {0x8D, (uint8_t []){0x04}, 1, 0},
  {0x8E, (uint8_t []){0x00}, 1, 0},
  {0x8F, (uint8_t []){0x00}, 1, 0},
  {0x90, (uint8_t []){0x48}, 1, 0},
  {0x91, (uint8_t []){0x00}, 1, 0},
  {0x92, (uint8_t []){0x0A}, 1, 0},
  {0x93, (uint8_t []){0x02}, 1, 0},
  {0x94, (uint8_t []){0xDA}, 1, 0},
  {0x95, (uint8_t []){0x04}, 1, 0},
  {0x96, (uint8_t []){0x00}, 1, 0},
  {0x97, (uint8_t []){0x00}, 1, 0},
  {0x98, (uint8_t []){0x48}, 1, 0},
  {0x99, (uint8_t []){0x00}, 1, 0},
  {0x9A, (uint8_t []){0x0C}, 1, 0},
  {0x9B, (uint8_t []){0x02}, 1, 0},
  {0x9C, (uint8_t []){0xDC}, 1, 0},
  {0x9D, (uint8_t []){0x04}, 1, 0},
  {0x9E, (uint8_t []){0x00}, 1, 0},
  {0x9F, (uint8_t []){0x00}, 1, 0},
  {0xA0, (uint8_t []){0x48}, 1, 0},
  {0xA1, (uint8_t []){0x00}, 1, 0},
  {0xA2, (uint8_t []){0x05}, 1, 0},
  {0xA3, (uint8_t []){0x02}, 1, 0},
  {0xA4, (uint8_t []){0xD5}, 1, 0},
  {0xA5, (uint8_t []){0x04}, 1, 0},
  {0xA6, (uint8_t []){0x00}, 1, 0},
  {0xA7, (uint8_t []){0x00}, 1, 0},
  {0xA8, (uint8_t []){0x48}, 1, 0},
  {0xA9, (uint8_t []){0x00}, 1, 0},
  {0xAA, (uint8_t []){0x07}, 1, 0},
  {0xAB, (uint8_t []){0x02}, 1, 0},
  {0xAC, (uint8_t []){0xD7}, 1, 0},
  {0xAD, (uint8_t []){0x04}, 1, 0},
  {0xAE, (uint8_t []){0x00}, 1, 0},
  {0xAF, (uint8_t []){0x00}, 1, 0},
  {0xB0, (uint8_t []){0x48}, 1, 0},
  {0xB1, (uint8_t []){0x00}, 1, 0},
  {0xB2, (uint8_t []){0x09}, 1, 0},
  {0xB3, (uint8_t []){0x02}, 1, 0},
  {0xB4, (uint8_t []){0xD9}, 1, 0},
  {0xB5, (uint8_t []){0x04}, 1, 0},
  {0xB6, (uint8_t []){0x00}, 1, 0},
  {0xB7, (uint8_t []){0x00}, 1, 0},
  {0xB8, (uint8_t []){0x48}, 1, 0},
  {0xB9, (uint8_t []){0x00}, 1, 0},
  {0xBA, (uint8_t []){0x0B}, 1, 0},
  {0xBB, (uint8_t []){0x02}, 1, 0},
  {0xBC, (uint8_t []){0xDB}, 1, 0},
  {0xBD, (uint8_t []){0x04}, 1, 0},
  {0xBE, (uint8_t []){0x00}, 1, 0},
  {0xBF, (uint8_t []){0x00}, 1, 0},
  {0xC0, (uint8_t []){0x10}, 1, 0},
  {0xC1, (uint8_t []){0x47}, 1, 0},
  {0xC2, (uint8_t []){0x56}, 1, 0},
  {0xC3, (uint8_t []){0x65}, 1, 0},
  {0xC4, (uint8_t []){0x74}, 1, 0},
  {0xC5, (uint8_t []){0x88}, 1, 0},
  {0xC6, (uint8_t []){0x99}, 1, 0},
  {0xC7, (uint8_t []){0x01}, 1, 0},
  {0xC8, (uint8_t []){0xBB}, 1, 0},
  {0xC9, (uint8_t []){0xAA}, 1, 0},
  {0xD0, (uint8_t []){0x10}, 1, 0},
  {0xD1, (uint8_t []){0x47}, 1, 0},
  {0xD2, (uint8_t []){0x56}, 1, 0},
  {0xD3, (uint8_t []){0x65}, 1, 0},
  {0xD4, (uint8_t []){0x74}, 1, 0},
  {0xD5, (uint8_t []){0x88}, 1, 0},
  {0xD6, (uint8_t []){0x99}, 1, 0},
  {0xD7, (uint8_t []){0x01}, 1, 0},
  {0xD8, (uint8_t []){0xBB}, 1, 0},
  {0xD9, (uint8_t []){0xAA}, 1, 0},
  {0xF3, (uint8_t []){0x01}, 1, 0},
  {0xF0, (uint8_t []){0x00}, 1, 0},
  {0x21, (uint8_t []){0x00}, 1, 0},
  {0x11, (uint8_t []){0x00}, 1, 120},
  {0x29, (uint8_t []){0x00}, 1, 0},
};

/* ---------- Panel ---------- */

// Reads register 0x04 (panel ID). The command word is: read opcode in the top
// byte, register address in the second byte.
static esp_err_t read_panel_id(uint8_t id[4])
{
    int cmd = ((int)LCD_OPCODE_READ_CMD << 24) | ((0x04 & 0xFF) << 8);
    return esp_lcd_panel_io_rx_param(io_handle, cmd, id, 4);
}

// Creates the QSPI panel IO on the shared bus at the given clock.
static esp_err_t create_panel_io(esp_lcd_panel_io_spi_config_t *cfg, int pclk_hz)
{
    cfg->pclk_hz = pclk_hz;
    return esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ESP_PANEL_HOST_SPI_ID_DEFAULT,
                                    cfg, &io_handle);
}

// Initializes the QSPI bus and the panel and switches the display on.
// Returns 1 on success, 0 if the bus or the panel IO cannot be created.
int QSPI_Init(void)
{
    // QSPI bus: clock and four data lines, no D/C line.
    static const spi_bus_config_t host_config = {
        .data0_io_num    = ESP_PANEL_LCD_SPI_IO_DATA0,
        .data1_io_num    = ESP_PANEL_LCD_SPI_IO_DATA1,
        .sclk_io_num     = ESP_PANEL_LCD_SPI_IO_SCK,
        .data2_io_num    = ESP_PANEL_LCD_SPI_IO_DATA2,
        .data3_io_num    = ESP_PANEL_LCD_SPI_IO_DATA3,
        .data4_io_num    = -1,
        .data5_io_num    = -1,
        .data6_io_num    = -1,
        .data7_io_num    = -1,
        .max_transfer_sz = ESP_PANEL_HOST_SPI_MAX_TRANSFER_SIZE,
        .flags           = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags      = 0,
    };
    if (spi_bus_initialize(ESP_PANEL_HOST_SPI_ID_DEFAULT, &host_config, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG_LCD, "QSPI bus initialization failed");
        return 0;
    }

    // Panel IO: 32-bit commands (opcode + address), 8-bit parameters, quad mode.
    // on_color_trans_done is left NULL: esp_lvgl_port registers its own callback.
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num       = ESP_PANEL_LCD_SPI_IO_CS,
        .dc_gpio_num       = -1,
        .spi_mode          = ESP_PANEL_LCD_SPI_MODE,
        .trans_queue_depth = ESP_PANEL_LCD_SPI_TRANS_QUEUE_SZ,
        .on_color_trans_done = NULL,
        .user_ctx          = NULL,
        .lcd_cmd_bits      = ESP_PANEL_LCD_SPI_CMD_BITS,
        .lcd_param_bits    = ESP_PANEL_LCD_SPI_PARAM_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode     = 0,
            .quad_mode      = 1,
            .sio_mode       = 0,
            .lsb_first      = 0,
            .cs_high_active = 0,
        },
    };

    // Identify the panel at a low clock, which every panel version accepts.
    if (create_panel_io(&io_config, LCD_ID_CLK_HZ) != ESP_OK) {
        ESP_LOGE(TAG_LCD, "Panel IO creation failed");
        return 0;
    }
    uint8_t id[4] = {0};
    if (read_panel_id(id) == ESP_OK)
        ESP_LOGI(TAG_LCD, "Panel ID (reg 0x04): %02X %02X %02X %02X", id[0], id[1], id[2], id[3]);
    else
        ESP_LOGW(TAG_LCD, "Panel ID not readable: default initialization");

    // Same panel IO again, at the full clock. The low-clock one is deleted
    // first, so it does not keep a device slot of the SPI bus.
    esp_lcd_panel_io_del(io_handle);
    if (create_panel_io(&io_config, ESP_PANEL_LCD_SPI_CLK_HZ) != ESP_OK) {
        ESP_LOGE(TAG_LCD, "Panel IO creation failed");
        return 0;
    }

    // Initialization commands for the detected panel version. Without a table
    // the esp_lcd_st77916 component uses its own default sequence.
    st77916_vendor_config_t vendor_config = {
        .flags = { .use_qspi_interface = 1 },
    };
    if (id[0] == 0x00 && id[1] == 0x02 && id[2] == 0x7F && id[3] == 0x7F) {
        vendor_config.init_cmds      = vendor_specific_init_new;
        vendor_config.init_cmds_size = sizeof(vendor_specific_init_new) / sizeof(st77916_lcd_init_cmd_t);
        ESP_LOGI(TAG_LCD, "Panel version 2: vendor initialization");
    } else {
        ESP_LOGI(TAG_LCD, "Panel version 1: default initialization");
    }

    // Panel driver: hardware reset on its own pin, RGB565, then initialization.
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_PIN_NUM_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = EXAMPLE_LCD_COLOR_BITS,
        .flags          = { .reset_active_high = 0 },
        .vendor_config  = (void *)&vendor_config,
    };
    esp_lcd_new_panel_st77916(io_handle, &panel_config, &panel_handle);

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);   // still dark: the backlight is off
    return 1;
}

void ST77916_Init(void)
{
    if (!QSPI_Init())
        ESP_LOGE(TAG_LCD, "ST77916 initialization failed");
}

/* ---------- Backlight ---------- */

uint8_t LCD_Backlight = 80;                          // brightness set by app_main(), percent

static ledc_channel_config_t ledc_channel;

// Configures the PWM of the backlight pin and leaves the backlight off.
void Backlight_Init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_PIN_NUM_BK_LIGHT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 5000,
        .speed_mode      = LEDC_LS_MODE,
        .timer_num       = LEDC_HS_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel    = LEDC_HS_CH0_CHANNEL;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = EXAMPLE_LCD_PIN_NUM_BK_LIGHT;
    ledc_channel.speed_mode = LEDC_LS_MODE;
    ledc_channel.timer_sel  = LEDC_HS_TIMER;
    ledc_channel_config(&ledc_channel);
    ledc_fade_func_install(0);

    // Off until app_main() has drawn the first frame.
    Set_Backlight(0);
    ESP_LOGI(TAG_LCD, "Backlight ready (off)");
}

// Sets the brightness, 0..100 percent. 0 switches the backlight off; from 1
// to 100 the duty goes linearly from about 2 % to 100 % of the PWM period.
// Valid only after Backlight_Init().
void Set_Backlight(uint8_t Light)
{
    if (Light > Backlight_MAX)
        Light = Backlight_MAX;

    uint16_t duty = LEDC_MAX_Duty - (81 * (Backlight_MAX - Light));
    if (Light == 0)
        duty = 0;

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

/* ---------- Start ---------- */

// Display panel and backlight. Called once by app_main(), before LVGL.
void LCD_Init(void)
{
    ST77916_Init();
    Backlight_Init();
}