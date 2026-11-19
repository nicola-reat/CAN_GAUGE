/*
 * gauge_display.c - live content of the gauge screen (see gauge_display.h)
 * ========================================================================
 *
 * What this file does
 *   The static layout comes from SquareLine Studio. This file creates the
 *   ring of segments at start-up and then refreshes texts, colors and lit
 *   segments with the values of the sensor, the alarms and the settings.
 *
 * Objects used from the SquareLine export (keep these names in the editor)
 *   ui_ScreenGauge   screen, parent of the ring object
 *   ui_ImgIcon       icon above the name; white PNG, recolored at run time
 *   ui_LabelName     name of the gauge, e.g. "FUEL"
 *   ui_LabelPercent  level in percent, e.g. "55%"
 *   ui_LabelLiters   value with decimals and unit, e.g. "33.0 L"
 *   ui_LabelWarn     warning text, empty when there is no warning
 *
 * Ring
 *   SEG_N arcs placed on a circle, filled clockwise from the top. They are
 *   drawn by one LVGL object (s_ring) in its draw event; when a segment
 *   changes color only its own bounding box is invalidated, so a refresh
 *   redraws a few small areas instead of the whole screen. The number of lit
 *   segments follows the level; their color follows the alarms:
 *     green    no alarm
 *     orange   low alarm
 *     red      very low or high alarm (blinking)
 *     red      sensor fault: whole ring blinking, "- - -" in the center
 *   A sensor fault and a very low level are both red and blinking, because
 *   in both cases the level must be checked; they differ in the number of
 *   lit segments (whole ring / only a few) and in the center text.
 *
 * Level in percent
 *   The lowest value of the calibration table is 0 %, the highest is 100 %.
 *   No extra parameter is needed: a new table sent from BusWrench also
 *   changes the scale of the ring.
 *
 * Sensor fault
 *   Depends on the fault mode set in gauge_cfg:
 *     GAUGE_FAULT_DASHES  "- - -" blinking, whole ring red and blinking
 *     GAUGE_FAULT_LAST    last valid value, with the fault warning
 *   Before the first reading "- - -" is shown, not blinking, ring off.
 *
 * Start-up sweep
 *   gauge_display_sweep(), called by app_main() once the backlight is on,
 *   fills the ring and empties it again (SWEEP_UP_MS, SWEEP_HOLD_MS,
 *   SWEEP_DOWN_MS), always in COL_SWEEP and whatever the sensor state, as a
 *   power-on test of the display. Until then the ring is off; afterwards it
 *   shows the real state: level and alarm color, or the blinking red ring
 *   of a sensor fault. Texts are refreshed normally during the sweep; only the
 *   ring is driven by the animation.
 *
 * Threads
 *   display_tick() is called by an LVGL timer, inside the LVGL task, so it
 *   may change LVGL objects without taking the LVGL lock. The get functions
 *   of adc_sensor, gauge_alarm and gauge_cfg are safe from any task.
 */

#include <stdio.h>
#include <string.h>
#include "lvgl.h"
#include "ui.h"              // objects of the SquareLine export
#include "adc_sensor.h"      // sensor value, sensor fault, calibration table
#include "gauge_alarm.h"     // alarm flags
#include "gauge_cfg.h"       // name, unit, decimals, fault mode
#include "gauge_display.h"

/* ---------- Ring geometry (round display, 360 x 360 pixels) ---------- */
#define SEG_N        36      // number of segments around the circle
#define SEG_GAP_DEG  2       // empty space between two segments, degrees
#define RING_OUT_R   168     // outer radius of the ring, pixels (screen radius: 180)
#define RING_W       20      // thickness of the ring, pixels

/* ---------- Colors (0xRRGGBB) ---------- */
#define COL_GREEN    0x2EE08A    // no alarm
#define COL_ORANGE   0xFFA31A    // low alarm
#define COL_RED      0xFF4D4D    // very low or high alarm
#define COL_RED_DIM  0x5A1F1F    // red alarm, "off" phase of the blink
#define COL_OFF      0x1E262B    // segment not lit
#define COL_WHITE    0xFFFFFF    // percentage without alarm
#define COL_ICON     0xC9D3D8    // icon without alarm

/* ---------- Timing ---------- */
#define TICK_MS      100     // refresh period of the screen, ms
#define BLINK_TICKS  5       // refreshes per blink phase (5 x 100 ms = 0.5 s)

/* ---------- Start-up sweep ---------- */
#define SWEEP_UP_MS    700   // empty -> full
#define SWEEP_HOLD_MS  150   // pause on full
#define SWEEP_DOWN_MS  700   // full -> empty
#define COL_SWEEP      COL_GREEN   // ring color during the whole sweep

/* ---------- State ---------- */

static lv_obj_t *s_ring;                        // object that draws the ring
static uint32_t  s_seg_col[SEG_N];              // current color of each segment

// Last name shown: the label is changed only when the name changes.
static char     s_last_name[GAUGE_NAME_MAX + 1] = "";

static int      s_last_valid = -1;              // last valid value x 10, for GAUGE_FAULT_LAST
static uint32_t s_tick       = 0;               // refresh counter, drives the blink

// Start-up sweep. While it runs, display_tick() stores the ring state it
// would draw in s_target_*, and the animation draws the ring instead.
static bool     s_sweeping   = true;
static int      s_target_on  = 0;               // lit segments for the current level
static uint32_t s_target_col = COL_GREEN;       // their color

/* ---------- Ring ---------- */

// Start and end angle of segment i, in LVGL degrees (0 = 3 o'clock, growing
// clockwise). The offset of 270 degrees puts segment 0 at 12 o'clock.
static void seg_angles(int i, int *start, int *end)
{
    const int step = 360 / SEG_N;               // angle of one segment and its gap

    *start = (270 + i * step + SEG_GAP_DEG / 2) % 360;
    *end   = (270 + (i + 1) * step - SEG_GAP_DEG / 2) % 360;
}

// Center of the ring, in screen coordinates.
static lv_point_t ring_center(void)
{
    lv_area_t a;
    lv_obj_get_coords(s_ring, &a);
    lv_point_t c = { a.x1 + lv_area_get_width(&a) / 2, a.y1 + lv_area_get_height(&a) / 2 };
    return c;
}

// Draw event of the ring object: draws every segment with its color.
// LVGL clips the drawing to the invalidated areas, so only the segments
// that changed are actually rendered.
static void ring_draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_point_t  c     = ring_center();

    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.center  = c;
    dsc.radius  = RING_OUT_R;                   // outer radius; the arc grows inwards by width
    dsc.width   = RING_W;
    dsc.rounded = 0;                            // square ends

    for (int i = 0; i < SEG_N; i++) {
        int start, end;
        seg_angles(i, &start, &end);
        dsc.start_angle = start;
        dsc.end_angle   = end;
        dsc.color       = lv_color_hex(s_seg_col[i]);
        lv_draw_arc(layer, &dsc);
    }
}

// Asks LVGL to redraw segment i only (its bounding box, plus a 1 px margin
// for the anti-aliased edges).
static void invalidate_seg(int i)
{
    int start, end;
    seg_angles(i, &start, &end);

    lv_point_t c = ring_center();
    lv_area_t  area;
    lv_draw_arc_get_area(c.x, c.y, RING_OUT_R, start, end, RING_W, false, &area);
    lv_area_increase(&area, 1, 1);
    lv_obj_invalidate_area(s_ring, &area);
}

// Creates the ring object: a transparent, full-screen object that only
// draws the segments. It is placed behind the SquareLine objects.
static void create_ring(void)
{
    s_ring = lv_obj_create(ui_ScreenGauge);
    lv_obj_remove_style_all(s_ring);                        // no background, border or padding
    lv_obj_set_size(s_ring, LV_PCT(100), LV_PCT(100));
    lv_obj_center(s_ring);
    lv_obj_remove_flag(s_ring, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_ring, ring_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_move_background(s_ring);                         // icon and labels stay on top

    for (int i = 0; i < SEG_N; i++)
        s_seg_col[i] = COL_OFF;
}

// Lights the first "on" segments with "color" and turns the others off.
// Only the segments whose color really changes are redrawn.
static void set_ring(int on, uint32_t color)
{
    for (int i = 0; i < SEG_N; i++) {
        uint32_t c = (i < on) ? color : COL_OFF;
        if (s_seg_col[i] != c) {
            s_seg_col[i] = c;
            invalidate_seg(i);
        }
    }
}

// Ring state requested by display_tick(): drawn at once, or kept as the
// target of the start-up sweep while the sweep is running.
static void show_ring(int on, uint32_t color)
{
    s_target_on  = on;
    s_target_col = color;
    if (!s_sweeping)
        set_ring(on, color);
}

/* ---------- Start-up sweep ---------- */

// Animation step: lights "value" segments with the sweep color.
static void sweep_exec(void *var, int32_t value)
{
    (void)var;
    set_ring((int)value, COL_SWEEP);
}

// End of the sweep: the ring shows the real state from now on (level,
// alarm color or sensor fault), as computed by display_tick().
static void sweep_done(lv_anim_t *a)
{
    (void)a;
    s_sweeping = false;
    set_ring(s_target_on, s_target_col);
}

// End of the upward phase: after a short pause the ring empties again.
static void sweep_full(lv_anim_t *a)
{
    (void)a;

    lv_anim_t down;
    lv_anim_init(&down);
    lv_anim_set_var(&down, s_ring);
    lv_anim_set_exec_cb(&down, sweep_exec);
    lv_anim_set_values(&down, SEG_N, 0);
    lv_anim_set_delay(&down, SWEEP_HOLD_MS);
    lv_anim_set_duration(&down, SWEEP_DOWN_MS);
    lv_anim_set_path_cb(&down, lv_anim_path_linear);       // constant speed: one segment after the other
    lv_anim_set_completed_cb(&down, sweep_done);
    lv_anim_start(&down);
}

// Starts the upward phase of the sweep (see gauge_display.h).
void gauge_display_sweep(void)
{
    lv_anim_t up;
    lv_anim_init(&up);
    lv_anim_set_var(&up, s_ring);
    lv_anim_set_exec_cb(&up, sweep_exec);
    lv_anim_set_values(&up, 0, SEG_N);
    lv_anim_set_duration(&up, SWEEP_UP_MS);
    lv_anim_set_path_cb(&up, lv_anim_path_linear);         // constant speed: one segment after the other
    lv_anim_set_completed_cb(&up, sweep_full);
    lv_anim_start(&up);
}

/* ---------- Helpers ---------- */

// Lowest and highest value of the calibration table, in tenths of the unit.
// They are the 0 % and the 100 % of the ring.
static void table_range(int *vmin, int *vmax)
{
    int n = adc_sensor_get_table_n();

    *vmin = 0;
    *vmax = 0;
    for (int i = 0; i < n; i++) {
        int mv, v;
        if (!adc_sensor_get_point(i, &mv, &v))
            continue;
        if (i == 0 || v < *vmin) *vmin = v;
        if (i == 0 || v > *vmax) *vmax = v;
    }
}

// Writes a value with its unit, e.g. "33.0 L".
//   v_x10  value in tenths of the unit (330 = 33.0)
//   dec    digits after the point, 0..2
// The value has a resolution of one tenth: with 2 decimals the last digit is
// always 0, with 0 decimals the value is rounded to the nearest integer.
static void format_value(char *out, size_t n, int v_x10, int dec, const char *unit)
{
    int         a    = v_x10 < 0 ? -v_x10 : v_x10;   // digits from the absolute value
    const char *sign = v_x10 < 0 ? "-" : "";

    if (dec <= 0)
        snprintf(out, n, "%s%d %s", sign, (a + 5) / 10, unit);
    else if (dec == 1)
        snprintf(out, n, "%s%d.%d %s", sign, a / 10, a % 10, unit);
    else
        snprintf(out, n, "%s%d.%d0 %s", sign, a / 10, a % 10, unit);
}

// Sets the text of a label only when it changes, to avoid useless redraws.
static void set_text(lv_obj_t *lbl, const char *txt)
{
    if (strcmp(lv_label_get_text(lbl), txt) != 0)
        lv_label_set_text(lbl, txt);
}

// Colors of the percentage, of the warning and of the icon.
// The icon is a white PNG: the recolor style paints it with "icon".
static void set_colors(uint32_t percent, uint32_t warn, uint32_t icon)
{
    lv_obj_set_style_text_color(ui_LabelPercent, lv_color_hex(percent), 0);
    lv_obj_set_style_text_color(ui_LabelWarn, lv_color_hex(warn), 0);
    lv_obj_set_style_image_recolor(ui_ImgIcon, lv_color_hex(icon), 0);
    lv_obj_set_style_image_recolor_opa(ui_ImgIcon, LV_OPA_COVER, 0);
}

/* ---------- Periodic refresh ---------- */

// Refreshes the screen. Called by the LVGL timer every TICK_MS, and once by
// gauge_display_start() to draw the first frame without waiting.
static void display_tick(lv_timer_t *t)
{
    (void)t;

    s_tick++;
    bool blink_on = ((s_tick / BLINK_TICKS) % 2) == 0;     // alternates every BLINK_TICKS

    // Name: it changes only when edited from BusWrench.
    char name[GAUGE_NAME_MAX + 1];
    gauge_cfg_get_name(name, sizeof name);
    if (strcmp(name, s_last_name) != 0) {
        lv_label_set_text(ui_LabelName, name);
        strcpy(s_last_name, name);
    }

    // Settings and sensor state.
    char unit[GAUGE_UNIT_MAX + 1];
    gauge_cfg_get_unit(unit, sizeof unit);
    int  dec   = gauge_cfg_get_decimals();
    int  v     = adc_sensor_get_value_x10();                // -1 until the first valid reading
    bool fault = adc_sensor_get_fault() != ADC_FAULT_NONE;
    bool valid = !fault && v >= 0;
    if (valid)
        s_last_valid = v;

    // No valid value: sensor fault, or no reading yet after power on.
    // With GAUGE_FAULT_LAST the normal screen is kept with the last valid value.
    bool show_last = fault && gauge_cfg_get_fault_mode() == GAUGE_FAULT_LAST
                     && s_last_valid >= 0;
    if (!valid && !show_last) {
        set_text(ui_LabelPercent, (fault && !blink_on) ? "" : "- - -");
        set_text(ui_LabelLiters, "");
        set_text(ui_LabelWarn, fault ? "SENSORE GUASTO" : "");
        set_colors(COL_WHITE, COL_RED, fault ? COL_RED : COL_ICON);
        show_ring(fault ? SEG_N : 0, blink_on ? COL_RED : COL_RED_DIM);
        return;
    }
    if (show_last)
        v = s_last_valid;

    // Level in percent on the range of the calibration table, limited to 0..100.
    int vmin, vmax;
    table_range(&vmin, &vmax);
    int pct = (vmax > vmin) ? (v - vmin) * 100 / (vmax - vmin) : 0;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    // Color and warning from the alarms; the most severe alarm wins.
    uint8_t     flags = gauge_alarm_get_flags();
    uint32_t    col   = COL_GREEN;
    const char *warn  = "";
    bool        blink = false;

    if (flags & GAUGE_ALARM_FLAG_LOW_CRIT) {
        col = COL_RED;    warn = "RISERVA";       blink = true;
    } else if (flags & GAUGE_ALARM_FLAG_HIGH) {
        col = COL_RED;    warn = "LIVELLO ALTO";  blink = true;
    } else if (flags & GAUGE_ALARM_FLAG_LOW) {
        col = COL_ORANGE; warn = "LIVELLO BASSO";
    }
    if (show_last)
        warn = "SENSORE GUASTO";                            // value shown, but not reliable

    // Texts.
    char buf[24];
    snprintf(buf, sizeof buf, "%d%%", pct);
    set_text(ui_LabelPercent, buf);
    format_value(buf, sizeof buf, v, dec, unit);
    set_text(ui_LabelLiters, buf);
    set_text(ui_LabelWarn, warn);

    // Percentage and icon take the alarm color only while an alarm is active.
    bool alarm = (col != COL_GREEN);
    set_colors(alarm ? col : COL_WHITE, col, alarm ? col : COL_ICON);

    // Lit segments, rounded to the nearest one. Any level above 0 % lights at
    // least one segment, so an almost empty tank is not shown as empty.
    int on = (pct * SEG_N + 50) / 100;
    if (pct > 0 && on == 0)
        on = 1;
    show_ring(on, (blink && !blink_on) ? COL_RED_DIM : col);
}

/* ---------- Start ---------- */

void gauge_display_start(void)
{
    create_ring();
    lv_timer_create(display_tick, TICK_MS, NULL);
    display_tick(NULL);                                     // texts of the first frame
}