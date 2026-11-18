#pragma once
#include <stdint.h>

/*
 * can_update_proto.h - CAN protocol of BusWrench (shared by PC, ESP32-S3 and STM32)
 * ==================================================================================
 *
 * What the protocol does
 *   - SCAN       find the boards on the bus and read their identity
 *   - HEARTBEAT  every board says "I am alive" once a second
 *   - UPDATE     send a new firmware file to a board
 *   - DETAILS    read uptime, partitions, errors, ... of a board
 *   - ACTIONS    reboot, identify, rollback
 *   - SENSOR     a gauge board sends its value by itself (no command needed)
 *   - PARAMETERS read, write, save and reset the settings of a board
 *
 * Frame format
 *   All frames are standard 11-bit CAN frames. "node" is the node ID of the
 *   board (1..0x7F). The first data byte tells WHAT the frame is (a command
 *   from the PC, or a message from the board); the other bytes are parameters.
 *   All multi-byte numbers are little endian (least significant byte first).
 *   The only exception is the sensor frame, which has no command byte.
 *
 * IDs used by this protocol: 0x500 ... 0x7FF
 *   Keep this range free from the other messages of the car.
 *
 * Update sequence:
 *   PC -> ENTER_BOOT            board -> BOOT_ACK
 *   PC -> BEGIN(size), CRC(crc) board -> BEGIN_ACK     (sent back to back)
 *   PC -> data frames (8 bytes each) + BLOCK_END(n, len)  board -> BLOCK_ACK(n)
 *        ... repeated for every block of CANUPD_BLOCK_SIZE bytes ...
 *   PC -> FINISH                board -> FINISH_ACK, then the board restarts
 *
 * Sensor frame (ID 0x500 + node, 6 bytes, sent every "period" ms, 100 by default)
 *   bytes 0-1  value x 10 (int16, -1 = no valid value yet)
 *   bytes 2-3  voltage at the connector, in mV (uint16)
 *   byte  4    flags: bit0 sensor fault, bit1 low alarm, bit2 high alarm,
 *              bit3 very low alarm
 *   byte  5    counter, grows at every frame (0..255)
 *   It is not sent while a firmware update is running.
 *
 * Parameters
 *   PC -> PARAM_READ(param, index)             board -> PARAM_VALUE
 *   PC -> PARAM_WRITE(param, index, value)     board -> PARAM_ACK (the value works at once, in RAM)
 *   PC -> PARAM_SAVE                           board -> PARAM_ACK (writes the NVS)
 *   PC -> PARAM_RESET                          board -> PARAM_ACK (factory values)
 *   A parameter the board does not know is answered with the status UNKNOWN.
 */

/* ---------- CAN IDs ---------- */
/* The ID tells who sends and who receives: the node number is added to a base.
 * Four bases of 0x80 IDs each: with node <= 0x7F they never overlap. */
#define CANUPD_ID_SENSOR(node)  (0x500u + (node))   /* board -> all: sensor value, every "period" ms */
#define CANUPD_ID_BROADCAST     0x600u              /* PC -> all boards (only SCAN is accepted) */
#define CANUPD_ID_CMD(node)     (0x600u + (node))   /* PC -> board: commands   */
#define CANUPD_ID_REPLY(node)   (0x680u + (node))   /* board -> PC: answers    */
#define CANUPD_ID_DATA(node)    (0x700u + (node))   /* PC -> board: file data  */

/* ---------- PC -> board: commands (first data byte) ---------- */
#define CANUPD_CMD_SCAN         0x01   /* who is there? (also accepted in broadcast) */
#define CANUPD_CMD_BEGIN        0x10   /* + file size, 4 bytes                  */
#define CANUPD_CMD_CRC          0x11   /* + CRC32 of the whole file, 4 bytes    */
#define CANUPD_CMD_BLOCK_END    0x12   /* + block number (2) + block length (2) */
#define CANUPD_CMD_FINISH       0x13   /* the file is complete: check and activate it */
#define CANUPD_CMD_ABORT        0x14   /* cancel the update in progress */
#define CANUPD_CMD_ENTER_BOOT   0x15   /* get ready for an update (cancels any previous one) */
#define CANUPD_CMD_DETAILS      0x20   /* ask the board for its details (answer: DETAIL_A..PART_B) */
#define CANUPD_CMD_REBOOT       0x21   /* restart the board */
#define CANUPD_CMD_IDENTIFY     0x22   /* blink the display for a few seconds */
#define CANUPD_CMD_ROLLBACK     0x23   /* start the firmware of the other OTA slot */

/* Parameters of a board (CAN_GAUGE): the list is below, see "Parameters" */
#define CANUPD_CMD_PARAM_READ   0x30   /* + parameter, index                              */
#define CANUPD_CMD_PARAM_WRITE  0x31   /* + parameter, index, 0, value (4 bytes)          */
#define CANUPD_CMD_PARAM_SAVE   0x32   /* write all the parameters in the permanent memory */
#define CANUPD_CMD_PARAM_RESET  0x33   /* factory values, and erase the saved ones         */

/* ---------- board -> PC: messages (first data byte) ---------- */
/* Identity and life signal */
#define CANUPD_MSG_HEARTBEAT    0x80   /* + state, fw major, minor, patch       */
#define CANUPD_MSG_INFO_A       0x81   /* + chip, hw letter, fw major, minor, patch */
#define CANUPD_MSG_INFO_B       0x82   /* + UID, 6 bytes                        */

/* Answers during the update */
#define CANUPD_MSG_BEGIN_ACK    0x90   /* + status                              */
#define CANUPD_MSG_BLOCK_ACK    0x92   /* + block number (2) + status           */
#define CANUPD_MSG_FINISH_ACK   0x93   /* + status                              */
#define CANUPD_MSG_BOOT_ACK     0x95   /* + status                              */

/* Answers to DETAILS: 7 frames (A, B, C, then PART_A + PART_B for each slot) */
#define CANUPD_MSG_DETAIL_A     0xA0   /* + uptime s (4), reset code, CAN TX errors, CAN RX errors */
#define CANUPD_MSG_DETAIL_B     0xA1   /* + temperature (int8, 0x80 = n/a), year-2000, month, day, hour, minute, flash size in MB */
#define CANUPD_MSG_DETAIL_C     0xA2   /* + git hash (4, little endian), dirty flag */
#define CANUPD_MSG_PART_A       0xA3   /* + slot (0/1), flags, address/64KB, size/64KB, used KB (2) */
#define CANUPD_MSG_PART_B       0xA4   /* + slot (0/1), fw major, minor, patch (0xFF = none) */

/* Answer to REBOOT, IDENTIFY and ROLLBACK */
#define CANUPD_MSG_ACTION_ACK   0xA5   /* + command, result (0 = ok, 1 = failed) */

/* Result of an action (ACTION_ACK) */
#define CANUPD_ACT_OK           0
#define CANUPD_ACT_FAILED       1

/* Answers to the parameter commands */
#define CANUPD_MSG_PARAM_VALUE  0xB0   /* + parameter, index, 0, value (4 bytes)           */
#define CANUPD_MSG_PARAM_ACK    0xB1   /* + command, parameter, status                     */

/* ---------- Parameters (the value is always 4 bytes, little endian) ---------- */
/* Sensor */
#define CANUPD_PAR_FILTER_MS    0x01   /* moving average time, ms (50 ... 10000)           */
#define CANUPD_PAR_LOW_MV       0x02   /* fault if the voltage is below this, mV           */
#define CANUPD_PAR_HIGH_MV      0x03   /* fault if the voltage is above this, mV           */

/* Identity and CAN */
#define CANUPD_PAR_NODE_ID      0x04   /* node ID used from the next boot (saved at once) */
#define CANUPD_PAR_SENSOR_PERIOD    0x09   /* period of the sensor frame, ms (20..500) */

/* Display texts and options */
#define CANUPD_PAR_NAME         0x05   /* name, text: 4 bytes per frame, index 0..2 (max 12) */
#define CANUPD_PAR_UNIT         0x06   /* unit, text: 4 bytes, index 0 */
#define CANUPD_PAR_DECIMALS     0x07   /* digits after the point: 0, 1 or 2 */
#define CANUPD_PAR_FAULT_MODE   0x08   /* on fault: 0 = "- - -" blinking, 1 = last valid value */

/* Calibration table */
#define CANUPD_PAR_TABLE_N      0x10   /* number of points of the table (2 ... 16);
                                          WRITING it applies the points sent before        */
#define CANUPD_PAR_TABLE_POINT  0x11   /* index = point number; value = mV (2 bytes)
                                          + value x10 (2 bytes, signed)                    */

/* Alarms: the index is the alarm (0 = low, 1 = very low, 2 = high) */
#define CANUPD_PAR_ALARM_ENABLE     0x20   /* 0 = off, 1 = on */
#define CANUPD_PAR_ALARM_THRESHOLD  0x21   /* limit, tenths of the unit */
#define CANUPD_PAR_ALARM_HYST       0x22   /* hysteresis, tenths of the unit */
#define CANUPD_PAR_ALARM_DELAY      0x23   /* delay, tenths of a second (0..600) */
#define CANUPD_PAR_ALARM_STARTUP    0x24   /* seconds after power on with the alarms ignored (index 0) */

/* PART_A flags: bit0 = running now, bit1 = starts at next boot, bit2 = valid image found */

/* Reset reasons (DETAIL_A): why the board restarted the last time */
#define CANUPD_RST_UNKNOWN      0
#define CANUPD_RST_POWERON      1      /* power was applied */
#define CANUPD_RST_EXTERNAL     2      /* reset pin */
#define CANUPD_RST_SOFTWARE     3      /* the firmware asked for it (reboot, update) */
#define CANUPD_RST_CRASH        4      /* firmware error */
#define CANUPD_RST_WATCHDOG     5      /* the firmware stopped answering */
#define CANUPD_RST_BROWNOUT     6      /* supply voltage dropped too low */
#define CANUPD_RST_OTHER        7

/* ---------- Status codes (in the answers of the update and of the parameters) ---------- */
#define CANUPD_ST_OK            0
#define CANUPD_ST_RETRY         1      /* send the block again                  */
#define CANUPD_ST_TOO_BIG       2      /* file does not fit in the flash area   */
#define CANUPD_ST_BUSY          3
#define CANUPD_ST_FLASH         4      /* flash write/erase error               */
#define CANUPD_ST_INVALID       5      /* wrong size, CRC or not a valid image; for a parameter: value not valid */
#define CANUPD_ST_UNKNOWN       6      /* unknown parameter                     */

/* ---------- Chip types (INFO_A) ---------- */
#define CANUPD_CHIP_ESP32S3     1
#define CANUPD_CHIP_STM32       2

/* ---------- Size of one block of the file (bytes) ---------- */
/* 256 bytes = 32 frames of 8 bytes, then one BLOCK_END and one BLOCK_ACK. */
#define CANUPD_BLOCK_SIZE       256