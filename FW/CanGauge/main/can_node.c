/*
 * can_node.c - CAN node of the CAN_GAUGE board (ESP32-S3)
 * ========================================================
 *
 * This file lets the board talk with the BusWrench PC application through
 * a PCAN-USB adapter (standard 11-bit IDs). The bus speed is fixed in the
 * firmware: CAN_BITRATE_KBPS, 500 kbit/s.
 *
 * What the node does
 *   SCAN        answers with the board identity (chip, hardware revision,
 *               firmware version, unique ID).
 *   HEARTBEAT   sends a frame every second, so the PC knows the board is alive.
 *   UPDATE      receives a new firmware over CAN, writes it to the spare OTA
 *               partition (ota_0 <-> ota_1), checks it and restarts into it.
 *   DETAILS     sends uptime, reset reason, CAN errors, chip temperature,
 *               build date, git commit and the state of both OTA partitions.
 *   ACTIONS     reboot, identify (blink the display), rollback to the
 *               firmware stored in the other partition.
 *   SENSOR      sends the value of the sensor every 100 ms by default (not
 *               while an update is running): value, voltage at the connector,
 *               fault and alarm flags.
 *   PARAMETERS  the PC reads and writes the settings of the board: sensor
 *               filter, fault limits, calibration table, node ID, texts of
 *               the display (name, unit, decimals, fault mode), alarms and
 *               period of the sensor frame.
 *   BUS CHECK   can_node_bus_ok() tells main.c whether the bus works, so a new
 *               firmware is confirmed only when it can be updated again.
 *
 * Parameters (PARAM_READ / PARAM_WRITE / PARAM_SAVE / PARAM_RESET)
 *   - WRITE applies the value at once, but only in RAM: it is lost at power off.
 *   - SAVE writes the current values in the permanent memory (NVS).
 *   - RESET restores the factory values and erases the saved ones.
 *   - The node ID is the identity of the board: it is saved in the NVS as soon
 *     as it is written, it is used after the next reboot, and RESET does not
 *     change it. The default is 0x45.
 *   - Name and unit are text: they travel 4 bytes per frame (name: 3 frames,
 *     unit: 1 frame).
 *   - Parameters are refused while an update is running (status BUSY).
 *
 * Safety of the update
 *   The running firmware is never touched while a new one is received: the data
 *   goes to the other partition. If anything goes wrong (cable unplugged, wrong
 *   CRC, timeout) the update is cancelled and the board keeps working.
 *
 * Frame IDs (see can_update_proto.h)
 *   0x500 + node   sensor value, sent by this board every period (100 ms default)
 *   0x600          broadcast from the PC (only SCAN is accepted)
 *   0x600 + node   commands from the PC to this board
 *   0x680 + node   replies from this board to the PC
 *   0x700 + node   file data (8 bytes per frame)
 *
 * Frame format
 *   The first data byte is always the command (PC -> board) or the message
 *   type (board -> PC). The following bytes are its parameters.
 *   All multi-byte numbers are little endian (least significant byte first).
 *
 *   The sensor frame (0x500 + node) is different: it has no command byte,
 *   it always has 6 bytes:
 *     bytes 0-1  value x 10 (int16, e.g. 325 = 32.5)
 *     bytes 2-3  voltage at the connector, in millivolts (uint16)
 *     byte  4    flags: bit0 sensor fault, bit1 low alarm, bit2 high alarm,
 *                bit3 very low alarm (see gauge_alarm.h)
 *     byte  5    counter, grows at every frame
 *
 * Structure of the file
 *   Configuration -> Update state -> Helpers -> Identity and heartbeat
 *   -> Details -> Actions -> Firmware update -> Parameters -> Sensor value
 *   -> Bus supervision and main task -> Start-up
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"        // ESP32 CAN controller driver (TWAI = CAN)
#include "esp_log.h"            // ESP_LOGI / ESP_LOGE / ESP_LOGW
#include "esp_mac.h"            // esp_efuse_mac_get_default()
#include "esp_timer.h"          // esp_timer_get_time(), software timers
#include "esp_crc.h"            // esp_crc32_le()
#include "esp_ota_ops.h"        // OTA functions (begin / write / end / set boot)
#include "esp_app_desc.h"       // esp_app_get_description() (firmware version, build date)
#include "esp_system.h"         // esp_restart(), esp_reset_reason()
#include "can_node.h"
#include "can_update_proto.h"   // command / reply codes and ID macros (shared with the PC)
#include "esp_partition.h"      // OTA partitions: address, size, content
#include "driver/temperature_sensor.h"   // internal temperature sensor of the chip
#include "esp_flash.h"          // esp_flash_get_size()
#include "adc_sensor.h"         // sensor value, filtered and checked
#include "nvs_flash.h"          // permanent memory (NVS)
#include "nvs.h"
#include "gauge_cfg.h"          // name, unit, decimals, fault mode
#include "gauge_alarm.h"        // alarms: threshold, hysteresis, delay

/* ---------- Configuration ---------- */

// CAN pins (connection to the CAN transceiver)
#define CAN_TX_GPIO   48
#define CAN_RX_GPIO   47

// CAN bus speed, kbit/s. Fixed on purpose: all the boards of the car share the
// same bus speed, and a speed changed over CAN could make the board unreachable
// if it did not match the PC. To use another speed change this value and
// rebuild; BusWrench must be set to the same speed.
#define CAN_BITRATE_KBPS  500

#if   CAN_BITRATE_KBPS == 125
#define CAN_TIMING_CONFIG()  TWAI_TIMING_CONFIG_125KBITS()
#elif CAN_BITRATE_KBPS == 250
#define CAN_TIMING_CONFIG()  TWAI_TIMING_CONFIG_250KBITS()
#elif CAN_BITRATE_KBPS == 500
#define CAN_TIMING_CONFIG()  TWAI_TIMING_CONFIG_500KBITS()
#elif CAN_BITRATE_KBPS == 1000
#define CAN_TIMING_CONFIG()  TWAI_TIMING_CONFIG_1MBITS()
#else
#error "CAN_BITRATE_KBPS must be 125, 250, 500 or 1000"
#endif

// Identity of this node.
// The node ID is read from the permanent memory (NVS) at boot, so the same
// firmware runs on several boards, each with its own number. A board that was
// never configured uses the default.
#define NODE_ID_DEFAULT  0x45
static uint8_t s_node_id   = NODE_ID_DEFAULT;   // ID in use now
static uint8_t s_node_next = NODE_ID_DEFAULT;   // ID saved in NVS: used from the next boot
#define NODE_ID  (s_node_id)                    // ID used in the frame IDs
#define HW_REV   'A'                            // hardware revision letter

// Git hash and "uncommitted changes" flag, set by the project CMakeLists.txt.
// If they are missing (build without git) the commit is shown as unavailable.
#ifndef GIT_HASH_HEX
#define GIT_HASH_HEX 0ul
#endif
#ifndef GIT_DIRTY
#define GIT_DIRTY 0
#endif

// Firmware version sent to the PC. It is NOT written here: it is read at
// startup from PROJECT_VER ("MAJOR.MINOR.PATCH") set in the project
// CMakeLists.txt, so the file and the board always show the same version.
static uint8_t s_fw_major, s_fw_minor, s_fw_patch;

static const char *TAG = "can_node";   // prefix of the log messages

// Set at the first frame received from the bus (see can_node_bus_ok()).
static volatile bool s_rx_seen = false;

// Reads "MAJOR.MINOR.PATCH" from the version string baked into the image.
// If the string has another format (e.g. a git hash) the result is 0.0.0.
static void load_version(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    unsigned a = 0, b = 0, c = 0;
    sscanf(d->version, "%u.%u.%u", &a, &b, &c);
    s_fw_major = (uint8_t)a;
    s_fw_minor = (uint8_t)b;
    s_fw_patch = (uint8_t)c;
}

// Reads the node ID from the permanent memory (namespace "node", key "id").
// If nothing is saved, or the saved number is not valid, the default stays.
// nvs_flash_init() is called here too: calling it twice is harmless.
static void load_node_id(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();               // same policy as the sensor settings
        nvs_flash_init();
    }

    nvs_handle_t h;
    if (nvs_open("node", NVS_READONLY, &h) != ESP_OK)
        return;                          // namespace not created yet: default

    uint8_t id = 0;
    if (nvs_get_u8(h, "id", &id) == ESP_OK && id >= 1 && id <= 0x7F) {
        s_node_id = id;
        s_node_next = id;
    }
    nvs_close(h);
}

// Saves the node ID in the permanent memory. It is used from the next boot.
static esp_err_t save_node_id(uint8_t id)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("node", NVS_READWRITE, &h);
    if (e != ESP_OK)
        return e;
    e = nvs_set_u8(h, "id", id);
    if (e == ESP_OK)
        e = nvs_commit(h);
    nvs_close(h);
    return e;
}

/* ---------- Update state ---------- */

// IDLE      = no update in progress
// WAIT_CRC  = BEGIN received (file size known), waiting for the CRC frame
// RECEIVING = flash prepared, blocks of the file are arriving
typedef enum { UPD_IDLE = 0, UPD_WAIT_CRC, UPD_RECEIVING } upd_state_t;

// Everything the board must remember during one update.
static struct {
    upd_state_t state;
    const esp_partition_t *part;   // flash area where the new firmware is written
    esp_ota_handle_t handle;       // handle of the OTA write session
    uint32_t total_size;           // file size announced by BEGIN
    uint32_t expected_crc;         // CRC32 of the whole file announced by the PC
    uint32_t written;              // bytes already written to flash
    uint32_t running_crc;          // CRC32 of the bytes written so far
    uint16_t next_block;           // number of the block we expect next
    uint8_t  buf[CANUPD_BLOCK_SIZE];   // RAM buffer of the block being received
    uint16_t buf_len;              // bytes received so far in the current block
    bool     overflow;             // more data than a block: the block is invalid
    int64_t  last_ms;              // time of the last update frame (for the timeout)
} s_upd;

/* ---------- Helpers ---------- */

// Milliseconds since boot.
static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// Reads a 32-bit little endian number from 4 bytes.
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Reads a 16-bit little endian number from 2 bytes.
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// Sends a CAN frame to the PC (ID 0x680 + node). All the replies go through here.
static void reply(const uint8_t *data, uint8_t len)
{
    twai_message_t m = {0};
    m.identifier = CANUPD_ID_REPLY(NODE_ID);
    m.data_length_code = len;
    memcpy(m.data, data, len);
    twai_transmit(&m, pdMS_TO_TICKS(50));    // wait at most 50 ms for a free slot
}

// Sends a 2-byte reply: message type + status (OK, TOO_BIG, FLASH, ...).
static void send_status(uint8_t msg, uint8_t status)
{
    uint8_t d[2] = { msg, status };
    reply(d, sizeof(d));
}

// Sends the answer to a block: block number (16 bit) + status.
// Status RETRY asks the PC to send the same block again.
static void send_block_ack(uint16_t block, uint8_t status)
{
    uint8_t d[4] = { CANUPD_MSG_BLOCK_ACK, (uint8_t)(block & 0xFF), (uint8_t)(block >> 8), status };
    reply(d, sizeof(d));
}

/* ---------- Identity and heartbeat ---------- */

// Answer to the scan: two frames.
//   INFO_A = chip, hardware revision, firmware version
//   INFO_B = 6-byte unique ID
// The PC gets the node number from the ID of the frame, not from the data.
static void send_info(void)
{
    uint8_t a[6] = { CANUPD_MSG_INFO_A, CANUPD_CHIP_ESP32S3, HW_REV,
                     s_fw_major, s_fw_minor, s_fw_patch };
    reply(a, sizeof(a));

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);          // the MAC address is used as UID
    uint8_t b[7] = { CANUPD_MSG_INFO_B, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] };
    reply(b, sizeof(b));
}

// Sent every second: update state + firmware version.
// The PC marks a board "offline" when the heartbeat stops.
static void send_heartbeat(void)
{
    uint8_t state = (s_upd.state != UPD_IDLE) ? 1 : 0;   // 1 = update in progress
    uint8_t h[5] = { CANUPD_MSG_HEARTBEAT, state, s_fw_major, s_fw_minor, s_fw_patch };
    reply(h, sizeof(h));
}

/* ---------- Details (answer to the DETAILS command) ---------- */

// One frame carries at most 8 bytes, so the details are split in 7 frames:
//   DETAIL_A  uptime, reset reason, CAN error counters
//   DETAIL_B  chip temperature, build date and time
//   DETAIL_C  git commit
//   PART_A/B  one pair for each OTA partition (address, size, version, ...)

// Limits a number to one byte (the CAN error counters can be bigger).
static uint8_t clamp8(uint32_t v) { return v > 255 ? 255 : (uint8_t)v; }

// Converts the ESP-IDF reset reason to the codes of the protocol.
static uint8_t reset_code(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return CANUPD_RST_POWERON;     // power on
    case ESP_RST_EXT:      return CANUPD_RST_EXTERNAL;    // reset pin
    case ESP_RST_SW:       return CANUPD_RST_SOFTWARE;    // esp_restart()
    case ESP_RST_PANIC:    return CANUPD_RST_CRASH;       // crash of the firmware
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return CANUPD_RST_WATCHDOG;    // watchdog timers
    case ESP_RST_BROWNOUT: return CANUPD_RST_BROWNOUT;    // supply voltage too low
    case ESP_RST_UNKNOWN:  return CANUPD_RST_UNKNOWN;
    default:               return CANUPD_RST_OTHER;
    }
}

// Size in bytes of the firmware image stored in a partition (0 if there is none).
// An ESP32 image is: header (24 bytes) + segments (8-byte header + data each)
// + 1 checksum byte padded to 16 bytes + optional SHA-256 (32 bytes).
// The partition is much bigger than the image, so this tells how much is used.
static uint32_t image_size(const esp_partition_t *p)
{
    uint8_t hdr[24];
    if (esp_partition_read(p, 0, hdr, sizeof(hdr)) != ESP_OK || hdr[0] != 0xE9)
        return 0;                                    // empty flash or not an image (0xE9 = magic byte)

    uint32_t pos = 24;                               // first segment starts after the header
    for (uint8_t i = 0; i < hdr[1]; i++) {           // hdr[1] = number of segments
        uint8_t sh[8];
        if (esp_partition_read(p, pos, sh, sizeof(sh)) != ESP_OK) return 0;
        pos += 8 + rd32(&sh[4]);                     // segment header + its data (length at byte 4)
        if (pos > p->size) return 0;                 // corrupted: segment outside the partition
    }
    pos = (pos + 1 + 15) & ~15u;                     // checksum byte, padded to 16
    if (hdr[23]) pos += 32;                          // hash appended after the image
    return pos <= p->size ? pos : 0;
}

// DETAIL_A: uptime, reset reason and CAN error counters.
static void send_detail_a(void)
{
    twai_status_info_t st = {0};
    twai_get_status_info(&st);                       // error counters of the controller

    uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000);   // seconds since boot
    uint8_t a[8] = { CANUPD_MSG_DETAIL_A,
                     (uint8_t)up, (uint8_t)(up >> 8), (uint8_t)(up >> 16), (uint8_t)(up >> 24),
                     reset_code(), clamp8(st.tx_error_counter), clamp8(st.rx_error_counter) };
    reply(a, sizeof(a));
}

// Internal temperature sensor of the ESP32-S3. It is installed the first time
// the details are asked, so it costs nothing until then.
static temperature_sensor_handle_t s_temp = NULL;
static bool s_temp_tried = false;

// Returns the chip temperature in degrees (rounded), or 0x80 if not available.
// It is the temperature of the chip itself, not of the air around it.
static uint8_t read_temperature(void)
{
    if (!s_temp_tried) {                       // first call: install the sensor
        s_temp_tried = true;
        // Measuring range 20..100 °C: the most accurate one for a chip at work.
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
        if (temperature_sensor_install(&cfg, &s_temp) != ESP_OK ||
            temperature_sensor_enable(s_temp) != ESP_OK)
            s_temp = NULL;                     // something failed: stay "not available"
    }
    if (!s_temp)
        return 0x80;

    float t = 0;
    if (temperature_sensor_get_celsius(s_temp, &t) != ESP_OK)
        return 0x80;
    if (t < -127) t = -127;                    // int8 limits (0x80 is reserved)
    if (t > 127)  t = 127;
    return (uint8_t)(int8_t)(t + (t >= 0 ? 0.5f : -0.5f));   // round to the nearest degree
}

// Size of the flash chip in MB (0 if it cannot be read).
static uint8_t flash_size_mb(void)
{
    uint32_t bytes = 0;
    if (esp_flash_get_size(NULL, &bytes) != ESP_OK)    // NULL = the main flash chip
        return 0;
    return clamp8(bytes / (1024 * 1024));
}

// DETAIL_B: chip temperature and build date / time.
static void send_detail_b(void)
{
    // The compiler writes the date as a text like "Nov  9 2026" and the time
    // as "11:20:05": they are converted to numbers (year - 2000, month, day, ...).
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const esp_app_desc_t *d = esp_app_get_description();
    char mon[4] = { 0 };
    int day = 0, year = 2000, hh = 0, mm = 0;
    sscanf(d->date, "%3s %d %d", mon, &day, &year);
    sscanf(d->time, "%d:%d", &hh, &mm);

    int month = 0;                                    // 1..12, found by searching the name
    for (int i = 0; i < 12; i++)
        if (strncmp(&months[i * 3], mon, 3) == 0) month = i + 1;

    uint8_t b[8] = { CANUPD_MSG_DETAIL_B, read_temperature(),   // 0x80 = not available
                     (uint8_t)(year - 2000), (uint8_t)month, (uint8_t)day,
                     (uint8_t)hh, (uint8_t)mm,
                     flash_size_mb() };                         // flash size in MB
    reply(b, sizeof(b));
}

// DETAIL_C: git hash of the source code and "uncommitted changes" flag.
static void send_detail_c(void)
{
    uint32_t h = (uint32_t)GIT_HASH_HEX;
    uint8_t c[6] = { CANUPD_MSG_DETAIL_C,
                     (uint8_t)h, (uint8_t)(h >> 8), (uint8_t)(h >> 16), (uint8_t)(h >> 24),
                     (uint8_t)GIT_DIRTY };
    reply(c, sizeof(c));
}

// PART_A + PART_B of one OTA slot (0 = ota_0, 1 = ota_1).
//   PART_A: flags, address and size (units of 64 KB), used space (KB)
//   PART_B: version of the firmware stored in the slot (0xFF.0xFF.0xFF = none)
// Flags: bit0 = running now, bit1 = starts at next boot, bit2 = valid image found.
static void send_partition(uint8_t slot)
{
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        (esp_partition_subtype_t)(ESP_PARTITION_SUBTYPE_APP_OTA_0 + slot), NULL);

    uint8_t flags = 0, addr = 0, size = 0;
    uint16_t used_kb = 0;
    uint8_t ver[3] = { 0xFF, 0xFF, 0xFF };           // 0xFF = no firmware in this slot

    if (p) {
        const esp_partition_t *run  = esp_ota_get_running_partition();
        const esp_partition_t *boot = esp_ota_get_boot_partition();
        if (run  && run->address  == p->address) flags |= 0x01;   // running now
        if (boot && boot->address == p->address) flags |= 0x02;   // starts at next boot
        addr = (uint8_t)(p->address >> 16);                       // in units of 64 KB
        size = (uint8_t)(p->size >> 16);

        uint32_t bytes = image_size(p);
        if (bytes) {
            flags |= 0x04;                                        // valid image found
            used_kb = (uint16_t)((bytes + 1023) / 1024);          // rounded up
        }

        esp_app_desc_t desc;                                      // version of that image
        if (esp_ota_get_partition_description(p, &desc) == ESP_OK) {
            unsigned a = 0, b = 0, c = 0;
            sscanf(desc.version, "%u.%u.%u", &a, &b, &c);
            ver[0] = (uint8_t)a; ver[1] = (uint8_t)b; ver[2] = (uint8_t)c;
        }
    }

    uint8_t pa[7] = { CANUPD_MSG_PART_A, slot, flags, addr, size,
                      (uint8_t)(used_kb & 0xFF), (uint8_t)(used_kb >> 8) };
    reply(pa, sizeof(pa));

    uint8_t pb[5] = { CANUPD_MSG_PART_B, slot, ver[0], ver[1], ver[2] };
    reply(pb, sizeof(pb));
}

// Answer to DETAILS: 7 frames (DETAIL_A, B, C and two for each of the two slots).
static void send_details(void)
{
    send_detail_a();
    send_detail_b();
    send_detail_c();
    send_partition(0);
    send_partition(1);
}

/* ---------- Actions: reboot, identify, rollback ---------- */

// Answer to an action command: [ACK, command, result].
// result: CANUPD_ACT_OK or CANUPD_ACT_FAILED.
static void send_action_ack(uint8_t cmd, uint8_t result)
{
    uint8_t a[3] = { CANUPD_MSG_ACTION_ACK, cmd, result };
    reply(a, sizeof(a));
}

// Waits a moment so the answer leaves the CAN controller, then restarts the chip.
// Without the delay the PC could miss the confirmation.
static void restart_soon(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// REBOOT: answer first, then restart.
static void do_reboot(void)
{
    send_action_ack(CANUPD_CMD_REBOOT, CANUPD_ACT_OK);
    restart_soon();
}

// IDENTIFY: calls board_identify(true/false) every 250 ms for 4 seconds.
// board_identify() is a "weak" function: this version does nothing. The real
// one is written in main.c (same name, no "weak"): the linker uses it instead
// of this one. This keeps the CAN code free of any display code.
__attribute__((weak)) void board_identify(bool on) { (void)on; }

static esp_timer_handle_t s_ident_timer = NULL;   // periodic timer, created on first use
static int s_ident_count = 0;                     // ticks left (16 ticks * 250 ms = 4 s)

// Called every 250 ms by the timer: switches the blink on / off.
static void identify_tick(void *arg)
{
    s_ident_count--;
    board_identify((s_ident_count % 2) == 1);   // on, off, on, off ...
    if (s_ident_count <= 0) {
        esp_timer_stop(s_ident_timer);
        board_identify(false);                  // always end with "off"
    }
}

// IDENTIFY: starts the blink and answers right away.
static void do_identify(void)
{
    if (!s_ident_timer) {                       // created the first time it is needed
        const esp_timer_create_args_t args = { .callback = identify_tick, .name = "identify" };
        if (esp_timer_create(&args, &s_ident_timer) != ESP_OK) {
            send_action_ack(CANUPD_CMD_IDENTIFY, CANUPD_ACT_FAILED);
            return;
        }
    }
    esp_timer_stop(s_ident_timer);              // restart if it was already running
    s_ident_count = 16;
    board_identify(true);
    esp_timer_start_periodic(s_ident_timer, 250 * 1000);   // period in microseconds
    send_action_ack(CANUPD_CMD_IDENTIFY, CANUPD_ACT_OK);
}

// ROLLBACK: boot the firmware stored in the other OTA slot.
// It is done only if that slot holds a valid image, otherwise nothing changes
// and the PC receives FAILED.
static void do_rollback(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *other = esp_ota_get_next_update_partition(run);   // the other slot
    esp_app_desc_t desc;

    if (!other || image_size(other) == 0 ||
        esp_ota_get_partition_description(other, &desc) != ESP_OK ||
        esp_ota_set_boot_partition(other) != ESP_OK) {
        send_action_ack(CANUPD_CMD_ROLLBACK, CANUPD_ACT_FAILED);
        return;
    }
    send_action_ack(CANUPD_CMD_ROLLBACK, CANUPD_ACT_OK);
    restart_soon();
}

/* ---------- Firmware update ---------- */

// Sequence of an update (PC -> board):
//   BEGIN (size) -> CRC (of the whole file) -> BEGIN_ACK
//   then, for each block of 256 bytes: 32 DATA frames + BLOCK_END -> BLOCK_ACK
//   at the end: FINISH -> FINISH_ACK, and the board restarts.

// Cancels the update in progress (the running firmware is never touched).
static void upd_abort(void)
{
    if (s_upd.state == UPD_RECEIVING)
        esp_ota_abort(s_upd.handle);         // release the OTA session
    s_upd.state = UPD_IDLE;
    s_upd.buf_len = 0;
    s_upd.overflow = false;
}

// BEGIN: the PC announces the size of the file.
// We pick the spare partition and check that the file fits in it.
static void on_begin(const twai_message_t *m)
{
    if (m->data_length_code < 5) return;     // 1 byte command + 4 bytes size
    upd_abort();                             // start from a clean state

    // The partition NOT currently running (ota_0 <-> ota_1).
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    const uint32_t size = rd32(&m->data[1]);

    if (!part) { send_status(CANUPD_MSG_BEGIN_ACK, CANUPD_ST_FLASH); return; }
    if (size == 0 || size > part->size) { send_status(CANUPD_MSG_BEGIN_ACK, CANUPD_ST_TOO_BIG); return; }

    s_upd.part = part;
    s_upd.total_size = size;
    s_upd.state = UPD_WAIT_CRC;       // the CRC frame follows immediately
}

// CRC: the PC sends the CRC32 of the whole file; now the flash is prepared
// (erased) and the BEGIN_ACK is sent: the PC can start sending data.
// Erasing takes time, that is why the PC waits for the BEGIN_ACK.
static void on_crc(const twai_message_t *m)
{
    if (s_upd.state != UPD_WAIT_CRC || m->data_length_code < 5) return;

    s_upd.expected_crc = rd32(&m->data[1]);

    esp_err_t err = esp_ota_begin(s_upd.part, OTA_WITH_SEQUENTIAL_WRITES, &s_upd.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        s_upd.state = UPD_IDLE;
        send_status(CANUPD_MSG_BEGIN_ACK, CANUPD_ST_FLASH);
        return;
    }

    s_upd.state = UPD_RECEIVING;
    s_upd.written = 0;
    s_upd.running_crc = 0;
    s_upd.next_block = 0;
    s_upd.buf_len = 0;
    s_upd.overflow = false;

    ESP_LOGI(TAG, "Update started: %lu bytes -> %s", (unsigned long)s_upd.total_size, s_upd.part->label);
    send_status(CANUPD_MSG_BEGIN_ACK, CANUPD_ST_OK);
}

// DATA: up to 8 bytes of the file, collected in the buffer of the current block.
// No answer is sent here: the answer is sent when the block is complete.
static void on_data(const twai_message_t *m)
{
    if (s_upd.state != UPD_RECEIVING) return;

    // More data than a block: mark it invalid, it will be requested again.
    if (s_upd.buf_len + m->data_length_code > CANUPD_BLOCK_SIZE) {
        s_upd.overflow = true;
        return;
    }
    memcpy(&s_upd.buf[s_upd.buf_len], m->data, m->data_length_code);
    s_upd.buf_len += m->data_length_code;
}

// BLOCK_END: the PC says "this block is complete" (block number + length).
// We check it, write it to flash and answer with BLOCK_ACK.
static void on_block_end(const twai_message_t *m)
{
    if (s_upd.state != UPD_RECEIVING || m->data_length_code < 5) return;

    const uint16_t block = rd16(&m->data[1]);
    const uint16_t len   = rd16(&m->data[3]);

    // Repeated block (our answer was lost): confirm again, do not write twice.
    if (s_upd.next_block > 0 && block == (uint16_t)(s_upd.next_block - 1)) {
        s_upd.buf_len = 0;
        s_upd.overflow = false;
        send_block_ack(block, CANUPD_ST_OK);
        return;
    }

    // Wrong block, missing frames or too much data: ask the PC to send it again.
    if (block != s_upd.next_block || s_upd.overflow || len == 0 ||
        len != s_upd.buf_len || s_upd.written + len > s_upd.total_size) {
        s_upd.buf_len = 0;
        s_upd.overflow = false;
        send_block_ack(block, CANUPD_ST_RETRY);
        return;
    }

    // Block is correct: write it to flash.
    esp_err_t err = esp_ota_write(s_upd.handle, s_upd.buf, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        upd_abort();
        send_block_ack(block, CANUPD_ST_FLASH);
        return;
    }

    // Update the running CRC and move on to the next block.
    s_upd.running_crc = esp_crc32_le(s_upd.running_crc, s_upd.buf, len);
    s_upd.written += len;
    s_upd.next_block++;
    s_upd.buf_len = 0;
    send_block_ack(block, CANUPD_ST_OK);
}

// FINISH: final checks, then the new firmware becomes the one to start.
static void on_finish(void)
{
    if (s_upd.state != UPD_RECEIVING) {
        send_status(CANUPD_MSG_FINISH_ACK, CANUPD_ST_INVALID);
        return;
    }

    // All bytes received and CRC equal to the one announced by the PC?
    if (s_upd.written != s_upd.total_size || s_upd.running_crc != s_upd.expected_crc) {
        ESP_LOGE(TAG, "Final check failed (size or CRC)");
        upd_abort();
        send_status(CANUPD_MSG_FINISH_ACK, CANUPD_ST_INVALID);
        return;
    }

    esp_err_t err = esp_ota_end(s_upd.handle);      // also checks the image itself
    s_upd.state = UPD_IDLE;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        send_status(CANUPD_MSG_FINISH_ACK, CANUPD_ST_INVALID);
        return;
    }

    // From the next boot the board starts from the new partition.
    err = esp_ota_set_boot_partition(s_upd.part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(err));
        send_status(CANUPD_MSG_FINISH_ACK, CANUPD_ST_FLASH);
        return;
    }

    ESP_LOGI(TAG, "Update OK, restarting...");
    send_status(CANUPD_MSG_FINISH_ACK, CANUPD_ST_OK);
    vTaskDelay(pdMS_TO_TICKS(300));                 // let the answer leave the bus
    esp_restart();
}

/* ---------- Parameters (read / write / save / factory) ---------- */

// Staging area of the calibration table. The points sent by the PC (one frame
// each) are collected here; the table in use changes only when the PC writes the
// number of points (PARAM TABLE_N), so the sensor never works with a half
// old / half new table.
static adc_cal_point_t s_stage[ADC_SENSOR_MAX_POINTS];
static uint32_t s_stage_mask = 0;      // bit i = point i has been received

// Answer with the value of a parameter: [PARAM_VALUE, parameter, index, 0, value x4].
static void send_param_value(uint8_t param, uint8_t index, uint32_t value)
{
    uint8_t v[8] = { CANUPD_MSG_PARAM_VALUE, param, index, 0,
                     (uint8_t)value, (uint8_t)(value >> 8),
                     (uint8_t)(value >> 16), (uint8_t)(value >> 24) };
    reply(v, sizeof(v));
}

// Answer with the result of a command: [PARAM_ACK, command, parameter, status].
static void send_param_ack(uint8_t cmd, uint8_t param, uint8_t status)
{
    uint8_t a[4] = { CANUPD_MSG_PARAM_ACK, cmd, param, status };
    reply(a, sizeof(a));
}

// A point of the table travels in 4 bytes: voltage in mV (low 16 bits)
// and value x10 (high 16 bits, signed).
static uint32_t pack_point(int mv, int val_x10)
{
    return (uint32_t)(uint16_t)mv | ((uint32_t)(uint16_t)(int16_t)val_x10 << 16);
}

// PARAM_READ: the PC asks for the current value of a parameter.
static void on_param_read(const twai_message_t *m)
{
    if (m->data_length_code < 3) return;
    const uint8_t param = m->data[1];
    const uint8_t index = m->data[2];
    int a = 0, b = 0;

    switch (param) {
    case CANUPD_PAR_FILTER_MS:
        send_param_value(param, 0, (uint32_t)adc_sensor_get_filter_ms());
        break;
    case CANUPD_PAR_LOW_MV:
    case CANUPD_PAR_HIGH_MV:
        adc_sensor_get_fault_limits(&a, &b);
        send_param_value(param, 0, (uint32_t)(param == CANUPD_PAR_LOW_MV ? a : b));
        break;
    case CANUPD_PAR_TABLE_N:
        send_param_value(param, 0, (uint32_t)adc_sensor_get_table_n());
        break;
    case CANUPD_PAR_TABLE_POINT:
        if (adc_sensor_get_point(index, &a, &b))
            send_param_value(param, index, pack_point(a, b));
        else
            send_param_ack(CANUPD_CMD_PARAM_READ, param, CANUPD_ST_INVALID);   // no such point
        break;
    case CANUPD_PAR_NODE_ID:
        send_param_value(param, 0, s_node_next);      // the ID saved (used from the next boot)
        break;
    case CANUPD_PAR_NAME: {
        // The name travels 4 bytes per frame: index 0, 1, 2.
        char t[16] = { 0 };
        gauge_cfg_get_name(t, sizeof(t));
        if (index < 3)
            send_param_value(param, index, rd32((const uint8_t *)&t[index * 4]));
        else
            send_param_ack(CANUPD_CMD_PARAM_READ, param, CANUPD_ST_INVALID);
        break;
    }
    case CANUPD_PAR_UNIT: {
        char t[8] = { 0 };
        gauge_cfg_get_unit(t, sizeof(t));
        send_param_value(param, 0, rd32((const uint8_t *)t));
        break;
    }
    case CANUPD_PAR_DECIMALS:
        send_param_value(param, 0, (uint32_t)gauge_cfg_get_decimals());
        break;
    case CANUPD_PAR_FAULT_MODE:
        send_param_value(param, 0, (uint32_t)gauge_cfg_get_fault_mode());
        break;
    case CANUPD_PAR_ALARM_ENABLE:
    case CANUPD_PAR_ALARM_THRESHOLD:
    case CANUPD_PAR_ALARM_HYST:
    case CANUPD_PAR_ALARM_DELAY: {
        // The index is the alarm: 0, 1 or 2.
        if (index >= GAUGE_ALARM_SLOTS) {
            send_param_ack(CANUPD_CMD_PARAM_READ, param, CANUPD_ST_INVALID);
            break;
        }
        int v = 0;
        if (param == CANUPD_PAR_ALARM_ENABLE)         v = gauge_alarm_get_enabled(index);
        else if (param == CANUPD_PAR_ALARM_THRESHOLD) v = gauge_alarm_get_threshold(index);
        else if (param == CANUPD_PAR_ALARM_HYST)      v = gauge_alarm_get_hysteresis(index);
        else                                          v = gauge_alarm_get_delay(index);
        send_param_value(param, index, (uint32_t)v);
        break;
    }
    case CANUPD_PAR_ALARM_STARTUP:
        send_param_value(param, 0, (uint32_t)gauge_alarm_get_startup_s());
        break;
    case CANUPD_PAR_SENSOR_PERIOD:
        send_param_value(param, 0, (uint32_t)gauge_cfg_get_period_ms());
        break;
    default:
        send_param_ack(CANUPD_CMD_PARAM_READ, param, CANUPD_ST_UNKNOWN);
        break;
    }
}

// PARAM_WRITE: the PC changes a parameter. It is used at once but NOT saved.
static void on_param_write(const twai_message_t *m)
{
    if (m->data_length_code < 8) return;
    const uint8_t param = m->data[1];
    const uint8_t index = m->data[2];
    const uint32_t value = rd32(&m->data[4]);

    // No changes while a firmware update is running.
    if (s_upd.state != UPD_IDLE) {
        send_param_ack(CANUPD_CMD_PARAM_WRITE, param, CANUPD_ST_BUSY);
        return;
    }

    esp_err_t err = ESP_OK;
    int lo = 0, hi = 0;

    switch (param) {
    case CANUPD_PAR_FILTER_MS:
        if (value < 50 || value > 10000) err = ESP_ERR_INVALID_ARG;
        else adc_sensor_set_filter_ms((int)value);
        break;
    case CANUPD_PAR_LOW_MV:
        adc_sensor_get_fault_limits(&lo, &hi);
        err = adc_sensor_set_fault_limits((int)value, hi);   // refused if not under the high limit
        break;
    case CANUPD_PAR_HIGH_MV:
        adc_sensor_get_fault_limits(&lo, &hi);
        err = adc_sensor_set_fault_limits(lo, (int)value);   // refused if not over the low limit
        break;
    case CANUPD_PAR_TABLE_POINT:
        if (index >= ADC_SENSOR_MAX_POINTS) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            s_stage[index].mv      = (int)(value & 0xFFFF);
            s_stage[index].val_x10 = (int)(int16_t)(value >> 16);
            s_stage_mask |= (1u << index);                   // point received
        }
        break;
    case CANUPD_PAR_TABLE_N: {
        // Apply the table: all points from 0 to n-1 must have been received.
        const uint32_t n = value;
        if (n < 2 || n > ADC_SENSOR_MAX_POINTS) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            const uint32_t need = (1u << n) - 1;             // bits 0 ... n-1
            if ((s_stage_mask & need) != need)
                err = ESP_ERR_INVALID_ARG;                   // some point is missing
            else
                err = adc_sensor_set_table(s_stage, (int)n); // also checks the order
            if (err == ESP_OK)
                s_stage_mask = 0;                            // staging area free again
        }
        break;
    }
    case CANUPD_PAR_NODE_ID:
        if (value < 1 || value > 0x7F) {
            err = ESP_ERR_INVALID_ARG;                // 0x7F is the maximum: 0x700 + node
        } else if (save_node_id((uint8_t)value) != ESP_OK) {
            send_param_ack(CANUPD_CMD_PARAM_WRITE, param, CANUPD_ST_FLASH);
            return;
        } else {
            s_node_next = (uint8_t)value;             // saved at once, active after a reboot
            ESP_LOGI(TAG, "Node ID 0x%02X saved, active after the next reboot", s_node_next);
        }
        break;
    case CANUPD_PAR_NAME: {
        // 4 characters per frame: the new part is put in the current text, then the
        // whole name is checked and applied (the PC always sends the 3 pieces).
        if (index >= 3) { err = ESP_ERR_INVALID_ARG; break; }
        char t[16] = { 0 };
        gauge_cfg_get_name(t, sizeof(t));
        for (int k = 0; k < 4; k++)
            t[index * 4 + k] = (char)(value >> (8 * k));
        t[GAUGE_NAME_MAX] = 0;
        err = gauge_cfg_set_name(t);
        break;
    }
    case CANUPD_PAR_UNIT: {
        char t[8] = { 0 };
        for (int k = 0; k < 4; k++)
            t[k] = (char)(value >> (8 * k));
        err = gauge_cfg_set_unit(t);
        break;
    }
    case CANUPD_PAR_DECIMALS:
        err = gauge_cfg_set_decimals((int)value);
        break;
    case CANUPD_PAR_FAULT_MODE:
        err = gauge_cfg_set_fault_mode((int)value);
        break;
    // Alarms: the set functions check the alarm number and the range of the value.
    case CANUPD_PAR_ALARM_ENABLE:
        err = gauge_alarm_set_enabled(index, (int)value);
        break;
    case CANUPD_PAR_ALARM_THRESHOLD:
        err = gauge_alarm_set_threshold(index, (int32_t)value);
        break;
    case CANUPD_PAR_ALARM_HYST:
        err = gauge_alarm_set_hysteresis(index, (int32_t)value);
        break;
    case CANUPD_PAR_ALARM_DELAY:
        err = gauge_alarm_set_delay(index, (int32_t)value);
        break;
    case CANUPD_PAR_ALARM_STARTUP:
        err = gauge_alarm_set_startup_s((int32_t)value);
        break;
    case CANUPD_PAR_SENSOR_PERIOD:
        err = gauge_cfg_set_period_ms((int)value);
        break;
    default:
        send_param_ack(CANUPD_CMD_PARAM_WRITE, param, CANUPD_ST_UNKNOWN);
        return;
    }

    send_param_ack(CANUPD_CMD_PARAM_WRITE, param,
                   err == ESP_OK ? CANUPD_ST_OK : CANUPD_ST_INVALID);
}

// PARAM_SAVE: writes sensor settings, gauge texts and alarms in the permanent memory (NVS).
static void on_param_save(void)
{
    if (s_upd.state != UPD_IDLE) {
        send_param_ack(CANUPD_CMD_PARAM_SAVE, 0, CANUPD_ST_BUSY);
        return;
    }
    esp_err_t e = adc_sensor_save();
    if (e == ESP_OK)
        e = gauge_cfg_save();
    if (e == ESP_OK)
        e = gauge_alarm_save();
    send_param_ack(CANUPD_CMD_PARAM_SAVE, 0, e == ESP_OK ? CANUPD_ST_OK : CANUPD_ST_FLASH);
}

// PARAM_RESET: factory values, and the saved ones are erased.
// The node ID is NOT touched: it is the identity of the board.
static void on_param_reset(void)
{
    if (s_upd.state != UPD_IDLE) {
        send_param_ack(CANUPD_CMD_PARAM_RESET, 0, CANUPD_ST_BUSY);
        return;
    }
    s_stage_mask = 0;                                         // forget the points not applied
    esp_err_t e = adc_sensor_reset_defaults();
    if (e == ESP_OK)
        e = gauge_cfg_reset_defaults();
    if (e == ESP_OK)
        e = gauge_alarm_reset_defaults();
    send_param_ack(CANUPD_CMD_PARAM_RESET, 0, e == ESP_OK ? CANUPD_ST_OK : CANUPD_ST_FLASH);
}

/* ---------- Sensor value (periodic data frame) ---------- */

// Sends the value of the sensor, every period (100 ms by default).
//   byte 0-1: value in tenths of the unit (200 = 20.0 L), int16, little endian
//   byte 2-3: voltage at the connector in millivolts, uint16, little endian
//   byte 4  : flags: bit0 = sensor fault, bit1 = low alarm, bit2 = high alarm,
//             bit3 = very low alarm (see gauge_alarm.h)
//   byte 5  : counter 0..255, it grows at every frame (shows that the board is alive)
//
// The alarms are checked here, with the same value that is sent. While the
// sensor is in fault, or has no valid value yet, all the alarms are off.
static void send_sensor_value(void)
{
    static uint8_t counter = 0;

    int value = adc_sensor_get_value_x10();         // -1 until the first valid reading
    int mv    = adc_sensor_get_mv();                // -1 until the first reading
    if (mv < 0) mv = 0;
    if (mv > 65535) mv = 65535;

    // Check the alarms with the last value (every 100 ms, like this frame).
    bool fault = (adc_sensor_get_fault() != ADC_FAULT_NONE);
    gauge_alarm_update(value, !fault && value >= 0);

    uint8_t flags = 0;
    if (fault)
        flags |= 0x01;                              // bit 0: sensor fault
    flags |= gauge_alarm_get_flags();               // bit 1 low, bit 2 high, bit 3 very low

    twai_message_t m = {0};
    m.identifier = CANUPD_ID_SENSOR(NODE_ID);
    m.data_length_code = 6;
    m.data[0] = (uint8_t)(value & 0xFF);
    m.data[1] = (uint8_t)((value >> 8) & 0xFF);
    m.data[2] = (uint8_t)(mv & 0xFF);
    m.data[3] = (uint8_t)((mv >> 8) & 0xFF);
    m.data[4] = flags;
    m.data[5] = counter++;
    twai_transmit(&m, pdMS_TO_TICKS(10));           // short wait: it is not important data
}

/* ---------- Bus supervision and main task ---------- */

// If the bus goes in error (e.g. cable unplugged) it restarts by itself.
// BUS_OFF = too many errors: the controller disconnected from the bus.
static void check_bus(void)
{
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) return;
    if (s.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();       // leave the BUS-OFF state
    } else if (s.state == TWAI_STATE_STOPPED) {
        twai_start();                   // restart the controller after recovery
    }
}

// twai_receive() that also records that the bus delivered a frame.
static esp_err_t receive_frame(twai_message_t *rx, TickType_t wait)
{
    esp_err_t e = twai_receive(rx, wait);
    if (e == ESP_OK)
        s_rx_seen = true;
    return e;
}

// See can_node.h.
bool can_node_bus_ok(void)
{
    twai_status_info_t s;
    return s_rx_seen &&
           twai_get_status_info(&s) == ESP_OK &&
           s.state == TWAI_STATE_RUNNING;
}

// Main CAN task: receives frames, runs the commands, sends the heartbeat.
static void can_task(void *arg)
{
    int64_t last_hb = 0;                // time of the last heartbeat
    int64_t last_sensor = 0;            // time of the last sensor frame

    while (1) {
        // Wait up to 10 ms for a frame; only standard data frames are used.
        twai_message_t rx;
        if (receive_frame(&rx, pdMS_TO_TICKS(10)) == ESP_OK &&
            !rx.extd && !rx.rtr && rx.data_length_code >= 1) {

            if (rx.identifier == CANUPD_ID_DATA(NODE_ID)) {
                // File data
                s_upd.last_ms = now_ms();
                on_data(&rx);

            } else if (rx.identifier == CANUPD_ID_BROADCAST) {
                // Broadcast: ONLY the scan is accepted (never update commands),
                // so a command can never reach all the boards at the same time.
                if (rx.data[0] == CANUPD_CMD_SCAN)
                    send_info();

            } else if (rx.identifier == CANUPD_ID_CMD(NODE_ID)) {
                // Commands addressed to this node only
                s_upd.last_ms = now_ms();
                switch (rx.data[0]) {
                case CANUPD_CMD_SCAN:       send_info();                                  break;
                case CANUPD_CMD_ENTER_BOOT: upd_abort(); send_status(CANUPD_MSG_BOOT_ACK, CANUPD_ST_OK); break;
                case CANUPD_CMD_BEGIN:      on_begin(&rx);                                break;
                case CANUPD_CMD_CRC:        on_crc(&rx);                                  break;
                case CANUPD_CMD_BLOCK_END:  on_block_end(&rx);                            break;
                case CANUPD_CMD_FINISH:     on_finish();                                  break;
                case CANUPD_CMD_ABORT:      upd_abort();                                  break;
                case CANUPD_CMD_DETAILS:    send_details();                               break;
                case CANUPD_CMD_REBOOT:     do_reboot();                                  break;
                case CANUPD_CMD_IDENTIFY:   do_identify();                                break;
                case CANUPD_CMD_ROLLBACK:   do_rollback();                                break;
                case CANUPD_CMD_PARAM_READ:  on_param_read(&rx);                          break;
                case CANUPD_CMD_PARAM_WRITE: on_param_write(&rx);                         break;
                case CANUPD_CMD_PARAM_SAVE:  on_param_save();                             break;
                case CANUPD_CMD_PARAM_RESET: on_param_reset();                            break;
                default:                         // unknown command: ignored, but written in the log
                    ESP_LOGW(TAG, "unknown command 0x%02X", rx.data[0]);
                    break;
                }
            }
        }

        // Update abandoned (cable unplugged, PC closed): give up after 10 seconds.
        if (s_upd.state != UPD_IDLE && now_ms() - s_upd.last_ms > 10000) {
            ESP_LOGW(TAG, "Update timeout, cancelled");
            upd_abort();
        }

        if (now_ms() - last_hb >= 1000) {            // heartbeat every second
            send_heartbeat();
            last_hb = now_ms();
        }
        // Sensor value every "period" ms (default 100, set from BusWrench),
        // not while a firmware update is running.
        if (s_upd.state == UPD_IDLE && now_ms() - last_sensor >= gauge_cfg_get_period_ms()) {
            send_sensor_value();
            last_sensor = now_ms();
        }
        check_bus();
    }
}

/* ---------- Start-up ---------- */

// Called once from app_main(): reads the version and the settings, starts
// the CAN controller at CAN_BITRATE_KBPS and creates the CAN task.
void can_node_start(void)
{
    load_version();           // fills s_fw_major / s_fw_minor / s_fw_patch
    load_node_id();           // node ID from the permanent memory (default 0x45)
    gauge_cfg_start();        // name, unit, decimals, fault mode from the permanent memory
    gauge_alarm_start();      // alarm settings from the permanent memory

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g.rx_queue_len = 64;      // a block is 32 frames sent back to back: bigger queue
    twai_timing_config_t  t = CAN_TIMING_CONFIG();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();   // filtering is done in can_task

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN started at %d kbit/s, node 0x%02X, FW %u.%u.%u",
             CAN_BITRATE_KBPS, NODE_ID, s_fw_major, s_fw_minor, s_fw_patch);

    xTaskCreate(can_task, "can_task", 6144, NULL, 5, NULL);   // stack 6 KB, priority 5
}