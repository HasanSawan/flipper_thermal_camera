/**
 * Camera view: draws the dithered image plus the readout sidebar, the spot
 * meter and the USB-stream screen, and maps the key bindings.
 *
 * The draw callback only reads the published display buffer and never touches
 * the sensor, so it cannot be stalled by I2C, SD or USB activity.
 */

#include "thermal_cam.h"

#include <math.h>
#include <string.h>

/* Sidebar geometry (x 87..127, 41 px wide). */
#define SB_X THERMAL_SIDEBAR_X
#define SB_W (128 - THERMAL_SIDEBAR_X)

const char* const thermal_mode_names[CamModeCount] = {
    [CamModeLinear] = "Linear",
    [CamModeInverted] = "Inverted",
    [CamModeHistEq] = "Hist EQ",
    [CamModeContour] = "Contour",
    [CamModeThreshold] = "Threshold",
    [CamModeSpot] = "Spot",
    [CamModeUsb] = "USB",
    [CamModeHuman] = "Human",
    [CamModeBle] = "Bluetooth",
};

const uint8_t thermal_mode_order[CamModeCount] = {
    CamModeLinear,
    CamModeInverted,
    CamModeHistEq,
    CamModeContour,
    CamModeThreshold,
    CamModeSpot,
    CamModeHuman,
    CamModeUsb,
    CamModeBle,
};

uint8_t thermal_mode_number(uint8_t mode) {
    for(uint8_t i = 0; i < CamModeCount; i++) {
        if(thermal_mode_order[i] == mode) return (uint8_t)(i + 1);
    }
    return 0;
}

/** "3 Hist EQ": the mode's position in the Left/Right cycle, then its name. */
static void mode_title(char* buf, size_t len, uint8_t mode) {
    snprintf(buf, len, "%u %s", (unsigned)thermal_mode_number(mode), thermal_mode_names[mode]);
}

static const char* const sensor_state_text[] = {
    [SensorStateInit] = "Starting...",
    [SensorStateOk] = "",
    [SensorStateNoDevice] = "No sensor found",
    [SensorStateBusError] = "I2C error",
    [SensorStateBadEeprom] = "Bad calibration",
};

static const char* const rate_short[Mlx90640RateCount] =
    {"0.5Hz", "1Hz", "2Hz", "4Hz", "8Hz", "16Hz", "32Hz", "64Hz"};

/* ---------------------------------------------------------------------------
 * Formatting
 * ------------------------------------------------------------------------ */

static void format_tenths(char* buf, size_t len, float t) {
    if(t > 999.0f) t = 999.0f;
    if(t < -999.0f) t = -999.0f;
    if(!isfinite(t)) t = 0.0f;

    int v = (int)lroundf(t * 10.0f);
    const bool neg = v < 0;
    if(neg) v = -v;

    snprintf(buf, len, "%s%d.%d", neg ? "-" : "", v / 10, v % 10);
}

void thermal_format_temp(char* buf, size_t len, float celsius, bool fahrenheit) {
    format_tenths(buf, len, fahrenheit ? (celsius * 9.0f / 5.0f + 32.0f) : celsius);
}

void thermal_format_delta(char* buf, size_t len, float celsius_delta, bool fahrenheit) {
    format_tenths(buf, len, fahrenheit ? (celsius_delta * 9.0f / 5.0f) : celsius_delta);
}

static inline char unit_char(const ThermalSettings* s) {
    return s->fahrenheit ? 'F' : 'C';
}

/**
 * Short label for whatever Up/Down currently adjusts. Shown permanently in a
 * corner of every screen so the key never does something invisible.
 */
static void updown_label(const ThermalSettings* s, char* buf, size_t len) {
    char t1[12];

    switch(s->render_mode) {
    case CamModeLinear:
    case CamModeInverted: {
        const unsigned g = thermal_gamma_x100[s->gamma_idx];
        snprintf(buf, len, "Gamma %u.%u", g / 100u, (g / 10u) % 10u);
        break;
    }
    case CamModeHistEq:
        snprintf(buf, len, "EQ %u%%", (unsigned)s->eq_idx * 10u);
        break;
    case CamModeContour: {
        const unsigned c = thermal_contour_x10[s->contour_idx];
        if(c % 10u) {
            snprintf(buf, len, "Step %u.%u%c", c / 10u, c % 10u, unit_char(s));
        } else {
            snprintf(buf, len, "Step %u%c", c / 10u, unit_char(s));
        }
        break;
    }
    case CamModeThreshold:
        thermal_format_temp(t1, sizeof(t1), (float)s->threshold_low, s->fahrenheit);
        switch(s->threshold_mode) {
        case ThresholdAbove:
            snprintf(buf, len, ">%s%c", t1, unit_char(s));
            break;
        case ThresholdBelow:
            snprintf(buf, len, "<%s%c", t1, unit_char(s));
            break;
        case ThresholdBand: {
            const float lo = s->fahrenheit ? s->threshold_low * 1.8f + 32.0f : s->threshold_low;
            const float hi = s->fahrenheit ? s->threshold_high * 1.8f + 32.0f : s->threshold_high;
            /* thresholds are clamped to -40..300 C, so these fit in 4 chars */
            snprintf(
                buf,
                len,
                "%d..%d%c",
                (int)lroundf(lo) % 1000,
                (int)lroundf(hi) % 1000,
                unit_char(s));
            break;
        }
        default:
            snprintf(buf, len, "Thr off");
            break;
        }
        break;
    case CamModeSpot:
        snprintf(buf, len, "%upx", (unsigned)thermal_spot_sizes[s->spot_idx]);
        break;
    case CamModeUsb:
    case CamModeBle:
        snprintf(buf, len, "%s", rate_short[s->rate]);
        break;
    case CamModeHuman: {
        const int o = s->body_offset_x10;
        const int a = o < 0 ? -o : o;
        snprintf(buf, len, "%c%d.%d%c", o < 0 ? '-' : '+', a / 10, a % 10, unit_char(s));
        break;
    }
    default:
        buf[0] = '\0';
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Overlay primitives
 *
 * Every marker is drawn as a white knockout first, then black on top, so it
 * stays readable whichever way the dither went underneath it.
 * ------------------------------------------------------------------------ */

static void draw_crosshair(Canvas* canvas, int x, int y) {
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x - 3, y - 3, 7, 7);

    canvas_set_color(canvas, ColorBlack);
    canvas_draw_line(canvas, x - 3, y, x - 1, y);
    canvas_draw_line(canvas, x + 1, y, x + 3, y);
    canvas_draw_line(canvas, x, y - 3, x, y - 1);
    canvas_draw_line(canvas, x, y + 1, x, y + 3);
}

static void draw_hot_marker(Canvas* canvas, int x, int y) {
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x - 2, y - 2, 5, 5);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, x - 1, y - 1, 3, 3);
}

static void draw_cold_marker(Canvas* canvas, int x, int y) {
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x - 2, y - 2, 5, 5);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_frame(canvas, x - 2, y - 2, 5, 5);
}

/** Small inverted badge, used for the mode name and REC indicator.
 *  Returns its width so badges can be chained. */
static int draw_badge(Canvas* canvas, int x, int y, const char* text) {
    const int w = canvas_string_width(canvas, text) + 3;

    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, x, y, w, 9);
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_str(canvas, x + 2, y + 7, text);
    canvas_set_color(canvas, ColorBlack);
    return w;
}

/** 5x9 up/down arrow pair in the current colour: "this is what Up/Down does". */
static void draw_updown_glyph(Canvas* canvas, int x, int y) {
    canvas_draw_dot(canvas, x + 2, y);
    canvas_draw_line(canvas, x + 1, y + 1, x + 3, y + 1);
    canvas_draw_line(canvas, x, y + 2, x + 4, y + 2);

    canvas_draw_line(canvas, x, y + 6, x + 4, y + 6);
    canvas_draw_line(canvas, x + 1, y + 7, x + 3, y + 7);
    canvas_draw_dot(canvas, x + 2, y + 8);
}

/** Inverted badge carrying the Up/Down glyph and the parameter value. */
static int draw_updown_badge(Canvas* canvas, int x, int y, const char* text) {
    canvas_set_font(canvas, FontSecondary);
    const int w = canvas_string_width(canvas, text) + 11;

    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, x, y, w, 9);
    canvas_set_color(canvas, ColorWhite);
    draw_updown_glyph(canvas, x + 2, y);
    canvas_draw_str(canvas, x + 9, y + 7, text);
    canvas_set_color(canvas, ColorBlack);
    return w;
}

/** Plain-ink (non-inverted) Up/Down hint, right-aligned to `right_x`. */
static void draw_updown_hint_right(Canvas* canvas, int right_x, int y, const char* text) {
    canvas_set_font(canvas, FontSecondary);
    const int tw = canvas_string_width(canvas, text);
    const int x = right_x - tw - 7;
    canvas_set_color(canvas, ColorBlack);
    draw_updown_glyph(canvas, x, y);
    canvas_draw_str(canvas, x + 7, y + 8, text);
}

/* ---------------------------------------------------------------------------
 * Big seven-segment digits for the spot meter
 *
 * The largest built-in font is ~15 px tall; a spot meter wants a number you
 * can read at arm's length, so the digits are drawn from boxes.
 * ------------------------------------------------------------------------ */

#define SEG_W       13
#define SEG_H       25
#define SEG_T       3
#define SEG_GAP     3
#define SEG_DOT_W   4
#define SEG_MINUS_W 9

/* Bit order: a b c d e f g */
static const uint8_t seg_map[10] = {
    0x3F, /* 0: abcdef  */
    0x06, /* 1: bc      */
    0x5B, /* 2: abdeg   */
    0x4F, /* 3: abcdg   */
    0x66, /* 4: bcfg    */
    0x6D, /* 5: acdfg   */
    0x7D, /* 6: acdefg  */
    0x07, /* 7: abc     */
    0x7F, /* 8: all     */
    0x6F, /* 9: abcdfg  */
};

static void draw_seg_digit(Canvas* canvas, int x, int y, uint8_t segs) {
    const int half = SEG_H / 2;
    if(segs & 0x01) canvas_draw_box(canvas, x + 1, y, SEG_W - 2, SEG_T); /* a */
    if(segs & 0x02) canvas_draw_box(canvas, x + SEG_W - SEG_T, y + 1, SEG_T, half); /* b */
    if(segs & 0x04)
        canvas_draw_box(canvas, x + SEG_W - SEG_T, y + half, SEG_T, SEG_H - half - 1); /* c */
    if(segs & 0x08) canvas_draw_box(canvas, x + 1, y + SEG_H - SEG_T, SEG_W - 2, SEG_T); /* d */
    if(segs & 0x10) canvas_draw_box(canvas, x, y + half, SEG_T, SEG_H - half - 1); /* e */
    if(segs & 0x20) canvas_draw_box(canvas, x, y + 1, SEG_T, half); /* f */
    if(segs & 0x40) canvas_draw_box(canvas, x + 1, y + half - 1, SEG_W - 2, SEG_T); /* g */
}

static int seg_text_width(const char* s) {
    int w = 0;
    for(const char* p = s; *p; p++) {
        if(*p >= '0' && *p <= '9') {
            w += SEG_W + SEG_GAP;
        } else if(*p == '.') {
            w += SEG_DOT_W + SEG_GAP;
        } else if(*p == '-') {
            w += SEG_MINUS_W + SEG_GAP;
        }
    }
    return w > 0 ? w - SEG_GAP : 0;
}

static void draw_seg_text(Canvas* canvas, int x, int y, const char* s) {
    for(const char* p = s; *p; p++) {
        if(*p >= '0' && *p <= '9') {
            draw_seg_digit(canvas, x, y, seg_map[*p - '0']);
            x += SEG_W + SEG_GAP;
        } else if(*p == '.') {
            canvas_draw_box(canvas, x, y + SEG_H - SEG_T, SEG_DOT_W - 1, SEG_T);
            x += SEG_DOT_W + SEG_GAP;
        } else if(*p == '-') {
            canvas_draw_box(canvas, x, y + SEG_H / 2 - 1, SEG_MINUS_W, SEG_T);
            x += SEG_MINUS_W + SEG_GAP;
        }
    }
}

/** Degree sign + unit letter, top-aligned with the big digits. */
static void draw_unit(Canvas* canvas, int x, int y, bool fahrenheit) {
    canvas_draw_circle(canvas, x + 2, y + 2, 2);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, x + 6, y + 9, fahrenheit ? "F" : "C");
}

/* ---------------------------------------------------------------------------
 * Screens
 * ------------------------------------------------------------------------ */

static void draw_sidebar(Canvas* canvas, const DisplayBuffer* db, const ThermalSettings* s) {
    char buf[16];

    canvas_set_color(canvas, ColorBlack);
    canvas_draw_line(canvas, THERMAL_IMG_W, 0, THERMAL_IMG_W, 63);

    /* Header strip carries the unit so the values themselves stay short. */
    canvas_draw_box(canvas, SB_X - 1, 0, SB_W + 1, 9);
    canvas_set_color(canvas, ColorWhite);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, SB_X + 1, 7, "TEMP");
    /* unit right-aligned, with a drawn degree sign (the fonts have none) */
    const char* unit = s->fahrenheit ? "F" : "C";
    const int ux = 126 - canvas_string_width(canvas, unit);
    canvas_draw_str(canvas, ux, 7, unit);
    canvas_draw_circle(canvas, ux - 3, 2, 1);
    canvas_set_color(canvas, ColorBlack);

    struct {
        const char* label;
        float value;
        int y;
    } rows[3] = {
        {"MAX", db->t_max, 11},
        {"CTR", db->t_center, 29},
        {"MIN", db->t_min, 47},
    };

    for(int i = 0; i < 3; i++) {
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, SB_X + 1, rows[i].y + 6, rows[i].label);

        thermal_format_temp(buf, sizeof(buf), rows[i].value, s->fahrenheit);
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, SB_X + 1, rows[i].y + 16, buf);
    }
}

static void draw_status_badges(Canvas* canvas, ThermalCamApp* app, int x, int y) {
    canvas_set_font(canvas, FontSecondary);
    if(app->recording) {
        char rec[20];
        snprintf(rec, sizeof(rec), "REC %lu", (unsigned long)app->rec_frame_count);
        draw_badge(canvas, x, y, rec);
    } else if(app->paused) {
        draw_badge(canvas, x, y, "PAUSED");
    }
}

static void draw_image_screen(
    Canvas* canvas,
    ThermalCamApp* app,
    const DisplayBuffer* db,
    const ThermalSettings* s) {
    char label[32];

    canvas_draw_xbm(canvas, 0, 0, THERMAL_IMG_W, THERMAL_IMG_H, db->bitmap);

    draw_hot_marker(canvas, db->max_x, db->max_y);
    draw_cold_marker(canvas, db->min_x, db->min_y);
    draw_crosshair(canvas, THERMAL_IMG_W / 2, THERMAL_IMG_H / 2);

    /* Mode number and name. */
    canvas_set_font(canvas, FontSecondary);
    mode_title(label, sizeof(label), s->render_mode);
    draw_badge(canvas, 0, 0, label);
    draw_status_badges(canvas, app, 0, 10);

    updown_label(s, label, sizeof(label));
    draw_updown_badge(canvas, 0, THERMAL_IMG_H - 9, label);

    draw_sidebar(canvas, db, s);
}

static void draw_spot_screen(
    Canvas* canvas,
    ThermalCamApp* app,
    const DisplayBuffer* db,
    const ThermalSettings* s) {
    char buf[24];

    /* -- header: mode badge left, spot size (the Up/Down value) right -- */
    canvas_set_font(canvas, FontSecondary);
    mode_title(buf, sizeof(buf), CamModeSpot);
    const int bw = draw_badge(canvas, 0, 0, buf);
    draw_status_badges(canvas, app, bw + 2, 0);

    snprintf(buf, sizeof(buf), "%upx", (unsigned)db->spot_size);
    draw_updown_hint_right(canvas, 127, 0, buf);

    /* -- big number, centred in the space left of the unit -- */
    thermal_format_temp(buf, sizeof(buf), db->t_spot, s->fahrenheit);
    const int tw = seg_text_width(buf);
    const int area_w = 110;
    int x = (area_w - tw) / 2;
    if(x < 0) x = 0;
    canvas_set_color(canvas, ColorBlack);
    draw_seg_text(canvas, x, 11, buf);
    draw_unit(canvas, x + tw + 3, 11, s->fahrenheit);

    /* -- history graph -- */
    const int gy1 = GRAPH_Y + GRAPH_H - 1;
    canvas_set_font(canvas, FontSecondary);

    if(!db->graph_valid) {
        canvas_draw_str_aligned(
            canvas, GRAPH_W / 2, GRAPH_Y + GRAPH_H / 2, AlignCenter, AlignCenter, "collecting...");
    } else {
        /* dotted guides at top and bottom of the plotted range */
        for(int gx = GRAPH_X; gx < GRAPH_X + GRAPH_W; gx += 3) {
            canvas_draw_dot(canvas, gx, GRAPH_Y);
            canvas_draw_dot(canvas, gx, gy1);
        }
        for(int i = 0; i < GRAPH_W; i++) {
            if(db->graph_top[i] == GRAPH_EMPTY) continue;
            canvas_draw_line(canvas, GRAPH_X + i, db->graph_top[i], GRAPH_X + i, db->graph_bot[i]);
        }
        /* newest-sample marker on the right edge */
        if(db->graph_top[GRAPH_W - 1] != GRAPH_EMPTY) {
            canvas_draw_box(canvas, GRAPH_X + GRAPH_W - 2, db->graph_top[GRAPH_W - 1] - 1, 2, 3);
        }

        /* window max / period / window min in the right column */
        thermal_format_temp(buf, sizeof(buf), db->graph_win_max, s->fahrenheit);
        canvas_draw_str_aligned(canvas, 127, GRAPH_Y, AlignRight, AlignTop, buf);
        thermal_format_temp(buf, sizeof(buf), db->graph_win_min, s->fahrenheit);
        canvas_draw_str_aligned(canvas, 127, 64, AlignRight, AlignBottom, buf);
    }

    const uint16_t sec = db->graph_seconds ? db->graph_seconds :
                                             thermal_graph_seconds[s->graph_idx];
    if(sec >= 60) {
        snprintf(buf, sizeof(buf), "%um", (unsigned)(sec / 60));
    } else {
        snprintf(buf, sizeof(buf), "%us", (unsigned)sec);
    }
    canvas_draw_str_aligned(canvas, 127, GRAPH_Y + GRAPH_H / 2, AlignRight, AlignCenter, buf);
    canvas_draw_line(canvas, GRAPH_X + GRAPH_W + 1, GRAPH_Y, GRAPH_X + GRAPH_W + 1, gy1);
}

/** Status page shared by the USB and Bluetooth stream modes. */
static void draw_stream_screen(
    Canvas* canvas,
    ThermalCamApp* app,
    const DisplayBuffer* db,
    const ThermalSettings* s,
    SensorState sensor) {
    char buf[48];
    char t[12];
    const bool ble = s->render_mode == CamModeBle;

    canvas_set_font(canvas, FontSecondary);
    mode_title(buf, sizeof(buf), s->render_mode);
    draw_badge(canvas, 0, 0, buf);
    canvas_draw_str_aligned(canvas, 127, 0, AlignRight, AlignTop, ble ? "Web BT" : "ttyACM1");

    const bool want = ble ? app->ble_want : app->usb_want;
    const uint32_t sent = ble ? app->ble_frames_sent : app->usb_frames_sent;
    const uint32_t dropped = ble ? app->ble_frames_dropped : app->usb_frames_dropped;

    const char* headline;
    const char* hint = NULL; /* replaces the counters when set */
    if(ble) {
        const BleState bs = app->ble_state;
        if(!want) {
            headline = (bs == BleStateError) ? "BT unavailable" : "Stream off";
        } else if(bs == BleStateRadioOff) {
            headline = "Bluetooth is off";
            hint = "Turn on: Settings > BT";
        } else if(bs == BleStateStreaming) {
            headline = "STREAMING";
        } else if(bs == BleStateConnected) {
            headline = "Connected";
        } else {
            headline = "Waiting for PC";
        }
    } else {
        const UsbState us = app->usb_state;
        if(!want) {
            headline = (us == UsbStateError) ? "USB busy" : "Stream off";
        } else if(us == UsbStateStreaming) {
            headline = "STREAMING";
        } else {
            headline = "Waiting for PC";
        }
    }

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 20, AlignCenter, AlignCenter, headline);

    canvas_set_font(canvas, FontSecondary);
    if(sensor != SensorStateOk) {
        canvas_draw_str_aligned(
            canvas, 64, 33, AlignCenter, AlignCenter, sensor_state_text[sensor]);
    } else if(hint) {
        canvas_draw_str_aligned(canvas, 64, 33, AlignCenter, AlignCenter, hint);
    } else if(want) {
        snprintf(
            buf, sizeof(buf), "%lu frames  %lu drop", (unsigned long)sent, (unsigned long)dropped);
        canvas_draw_str_aligned(canvas, 64, 33, AlignCenter, AlignCenter, buf);
    } else {
        canvas_draw_str_aligned(canvas, 64, 33, AlignCenter, AlignCenter, "Press OK to start");
    }

    if(sensor == SensorStateOk && db->valid) {
        thermal_format_temp(t, sizeof(t), db->t_max, s->fahrenheit);
        snprintf(
            buf,
            sizeof(buf),
            "%u.%u fps   max %s%c",
            (unsigned)(db->fps_x10 / 10),
            (unsigned)(db->fps_x10 % 10),
            t,
            unit_char(s));
        canvas_draw_str_aligned(canvas, 64, 44, AlignCenter, AlignCenter, buf);
    }

    /* bottom row: Up/Down = refresh rate, OK = start/stop */
    char label[32];
    updown_label(s, label, sizeof(label));
    draw_updown_badge(canvas, 0, 55, label);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(
        canvas, 127, 63, AlignRight, AlignBottom, want ? "OK: stop" : "OK: start");
}

/** Shown on entering Human mode until OK is pressed, once per session. */
static void draw_human_disclaimer(Canvas* canvas) {
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, 0, 0, 128, 11);
    canvas_set_color(canvas, ColorWhite);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 2, AlignCenter, AlignTop, "NOT A MEDICAL DEVICE");
    canvas_set_color(canvas, ColorBlack);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 14, AlignCenter, AlignTop, "A rough screening estimate.");
    canvas_draw_str_aligned(canvas, 64, 24, AlignCenter, AlignTop, "It can be wrong. Confirm any");
    canvas_draw_str_aligned(canvas, 64, 34, AlignCenter, AlignTop, "reading with a thermometer.");

    canvas_draw_rbox(canvas, 34, 50, 60, 13, 3);
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_str_aligned(canvas, 64, 53, AlignCenter, AlignTop, "OK: I agree");
    canvas_set_color(canvas, ColorBlack);
}

static void draw_human_screen(
    Canvas* canvas,
    ThermalCamApp* app,
    const DisplayBuffer* db,
    const ThermalSettings* s) {
    char buf[32];

    if(!app->human_ack) {
        draw_human_disclaimer(canvas);
        return;
    }

    /* -- header: mode badge left, body offset (the Up/Down value) right -- */
    canvas_set_font(canvas, FontSecondary);
    mode_title(buf, sizeof(buf), CamModeHuman);
    const int bw = draw_badge(canvas, 0, 0, buf);
    draw_status_badges(canvas, app, bw + 2, 0);
    updown_label(s, buf, sizeof(buf));
    draw_updown_hint_right(canvas, 127, 0, buf);

    const HumanStatus st = (HumanStatus)db->human_status;

    /* -- big estimated body temperature -- */
    if(st == HumanNone) {
        snprintf(buf, sizeof(buf), "--.-");
    } else {
        thermal_format_temp(buf, sizeof(buf), db->t_body, s->fahrenheit);
    }
    const int tw = seg_text_width(buf);
    int x = (110 - tw) / 2;
    if(x < 0) x = 0;
    canvas_set_color(canvas, ColorBlack);
    draw_seg_text(canvas, x, 11, buf);
    draw_unit(canvas, x + tw + 3, 11, s->fahrenheit);

    /* -- verdict -- */
    const char* verdict;
    bool alert = false;
    switch(st) {
    case HumanFever:
        verdict = "HIGH - FEVER?";
        alert = true;
        break;
    case HumanLow:
        verdict = "LOW TEMPERATURE";
        alert = true;
        break;
    case HumanNormal:
        verdict = "NORMAL";
        break;
    default:
        verdict = (db->t_skin > 43.0f) ? "Too hot to be skin" : "Face the sensor";
        break;
    }
    canvas_set_font(canvas, FontPrimary);
    if(alert) {
        canvas_draw_box(canvas, 0, 39, 128, 12);
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_str_aligned(canvas, 64, 45, AlignCenter, AlignCenter, verdict);
        canvas_set_color(canvas, ColorBlack);
    } else {
        canvas_draw_str_aligned(canvas, 64, 45, AlignCenter, AlignCenter, verdict);
    }

    /* -- footer: the raw skin reading and the standing disclaimer -- */
    canvas_set_font(canvas, FontSecondary);
    char t[12];
    thermal_format_temp(t, sizeof(t), db->t_skin, s->fahrenheit);
    snprintf(buf, sizeof(buf), "skin %s%c", t, unit_char(s));
    canvas_draw_str_aligned(canvas, 0, 64, AlignLeft, AlignBottom, buf);
    canvas_draw_str_aligned(canvas, 127, 64, AlignRight, AlignBottom, "not medical");
}

/* ---------------------------------------------------------------------------
 * Draw
 * ------------------------------------------------------------------------ */

static void draw_toast(Canvas* canvas, ThermalCamApp* app) {
    if(furi_get_tick() >= app->toast_until_tick) return;

    furi_mutex_acquire(app->toast_mutex, FuriWaitForever);
    canvas_set_font(canvas, FontSecondary);
    const int w = canvas_string_width(canvas, app->toast) + 6;
    const int x = (128 - w) / 2;

    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x, 26, w, 13);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_frame(canvas, x, 26, w, 13);
    canvas_draw_str(canvas, x + 3, 35, app->toast);
    furi_mutex_release(app->toast_mutex);
}

static void camera_draw_callback(Canvas* canvas, void* model) {
    ThermalViewModel* m = model;
    if(!m || !m->app) return;
    ThermalCamApp* app = m->app;

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    /* Snapshot the settings we need for drawing; these are plain scalars and
     * a torn read would at worst use last frame's value for one frame. */
    const ThermalSettings s = app->settings;
    const SensorState state = app->sensor_state;
    const DisplayBuffer* db = &app->disp[app->disp_active];

    if(cam_mode_is_stream(s.render_mode)) {
        draw_stream_screen(canvas, app, db, &s, state);
        draw_toast(canvas, app);
        return;
    }

    if(state != SensorStateOk || !db->valid) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 24, AlignCenter, AlignCenter, "Thermal Camera");

        canvas_set_font(canvas, FontSecondary);
        const char* msg = (state < COUNT_OF(sensor_state_text)) ? sensor_state_text[state] : "";
        if(state == SensorStateOk) msg = "Waiting for frame...";
        canvas_draw_str_aligned(canvas, 64, 38, AlignCenter, AlignCenter, msg);

        if(state == SensorStateNoDevice) {
            canvas_draw_str_aligned(
                canvas, 64, 50, AlignCenter, AlignCenter, "SDA=pin15 SCL=pin16");
            canvas_draw_str_aligned(
                canvas, 64, 60, AlignCenter, AlignCenter, "3V3=pin9 GND=pin18");
        } else {
            canvas_draw_str_aligned(canvas, 64, 56, AlignCenter, AlignCenter, "Back: settings");
        }
        draw_toast(canvas, app);
        return;
    }

    if(s.render_mode == CamModeSpot) {
        draw_spot_screen(canvas, app, db, &s);
    } else if(s.render_mode == CamModeHuman) {
        draw_human_screen(canvas, app, db, &s);
    } else {
        draw_image_screen(canvas, app, db, &s);
    }

    draw_toast(canvas, app);
}

/* ---------------------------------------------------------------------------
 * Input
 * ------------------------------------------------------------------------ */

static void cycle_mode(ThermalCamApp* app, int delta) {
    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    /* Step through the display order, not the enum. */
    int pos = (int)thermal_mode_number(app->settings.render_mode) - 1 + delta;
    while(pos < 0)
        pos += CamModeCount;
    while(pos >= CamModeCount)
        pos -= CamModeCount;
    const uint8_t m = thermal_mode_order[pos];
    app->settings.render_mode = m;
    furi_mutex_release(app->settings_mutex);

    /* Leaving a stream screen always ends its stream, so a port never stays
     * in streaming mode behind the user's back. */
    if(m != CamModeUsb && app->usb_want) app->usb_want = false;
    if(m != CamModeBle && app->ble_want) app->ble_want = false;

    app->settings_dirty = true;

    char msg[32];
    if(m == CamModeHuman) {
        snprintf(
            msg, sizeof(msg), "%u/%u Human: e=0.98", (unsigned)(pos + 1), (unsigned)CamModeCount);
    } else {
        snprintf(
            msg,
            sizeof(msg),
            "%u/%u %s",
            (unsigned)(pos + 1),
            (unsigned)CamModeCount,
            thermal_mode_names[m]);
    }
    thermal_toast(app, msg);
}

static int step_index(int idx, int delta, int count) {
    idx += delta;
    if(idx < 0) idx = 0;
    if(idx > count - 1) idx = count - 1;
    return idx;
}

/** Up/Down: adjust the one parameter that belongs to the current mode. */
static void adjust_param(ThermalCamApp* app, int delta) {
    char msg[48];
    char label[32];
    bool just_enabled = false;

    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    ThermalSettings* s = &app->settings;

    switch(s->render_mode) {
    case CamModeLinear:
    case CamModeInverted:
        s->gamma_idx = (uint8_t)step_index(s->gamma_idx, delta, GAMMA_STEPS);
        break;

    case CamModeHistEq:
        s->eq_idx = (uint8_t)step_index(s->eq_idx, delta, EQ_STEPS);
        break;

    case CamModeContour:
        s->contour_idx = (uint8_t)step_index(s->contour_idx, delta, CONTOUR_STEPS);
        break;

    case CamModeThreshold: {
        /* The first press switches a disabled threshold on, so the key always
         * does something visible. */
        if(s->threshold_mode == ThresholdOff) {
            s->threshold_mode = ThresholdAbove;
            just_enabled = true;
        }
        const int lo = s->threshold_low;
        int v = lo + delta;
        if(v < -40) v = -40;
        if(v > 300) v = 300;
        const int moved = v - lo;
        s->threshold_low = (int16_t)v;

        if(s->threshold_mode == ThresholdBand) {
            /* Band slides as a whole, keeping its width. */
            int hi = s->threshold_high + moved;
            if(hi > 300) hi = 300;
            s->threshold_high = (int16_t)hi;
        }
        if(s->threshold_high < s->threshold_low) s->threshold_high = s->threshold_low;
        break;
    }

    case CamModeSpot:
        s->spot_idx = (uint8_t)step_index(s->spot_idx, delta, SPOT_STEPS);
        break;

    case CamModeUsb:
    case CamModeBle:
        s->rate = (uint8_t)step_index(s->rate, delta, Mlx90640RateCount);
        break;

    case CamModeHuman: {
        int o = s->body_offset_x10 + delta;
        if(o < BODY_OFFSET_MIN_X10) o = BODY_OFFSET_MIN_X10;
        if(o > BODY_OFFSET_MAX_X10) o = BODY_OFFSET_MAX_X10;
        s->body_offset_x10 = (int8_t)o;
        break;
    }

    default:
        break;
    }

    const uint8_t mode = s->render_mode;
    const uint8_t spot = thermal_spot_sizes[s->spot_idx];
    updown_label(s, label, sizeof(label));
    furi_mutex_release(app->settings_mutex);

    app->settings_dirty = true;

    switch(mode) {
    case CamModeHistEq:
        snprintf(msg, sizeof(msg), "EQ strength %s", label + 3);
        break;
    case CamModeContour:
        snprintf(msg, sizeof(msg), "Contour step %s", label + 5); /* skip "Step " */
        break;
    case CamModeThreshold:
        snprintf(msg, sizeof(msg), "%sThreshold %s", just_enabled ? "On: " : "", label);
        break;
    case CamModeSpot: {
        static const uint8_t pixels[] = {1, 9, 21, 37};
        const uint8_t n = (spot == 1) ? pixels[0] :
                          (spot == 3) ? pixels[1] :
                          (spot == 5) ? pixels[2] :
                                        pixels[3];
        snprintf(msg, sizeof(msg), "Spot %upx (%u pixels)", (unsigned)spot, (unsigned)n);
        break;
    }
    case CamModeUsb:
    case CamModeBle:
        snprintf(msg, sizeof(msg), "Refresh %s", label);
        break;
    case CamModeHuman:
        snprintf(msg, sizeof(msg), "Body offset %s", label);
        break;
    default:
        snprintf(msg, sizeof(msg), "%s", label);
        break;
    }
    thermal_toast(app, msg);
}

static bool camera_input_callback(InputEvent* event, void* context) {
    ThermalCamApp* app = context;
    furi_assert(app);

    const uint8_t mode = app->settings.render_mode;

    /* Human mode starts behind its disclaimer: OK accepts it, and nothing
     * else happens on that screen except leaving it. */
    if(mode == CamModeHuman && !app->human_ack &&
       (event->key == InputKeyOk || event->key == InputKeyUp || event->key == InputKeyDown)) {
        if(event->key == InputKeyOk && event->type == InputTypeShort) app->human_ack = true;
        return true;
    }

    switch(event->key) {
    case InputKeyOk:
        if(mode == CamModeUsb) {
            /* OK toggles the stream; there is no image to capture here. */
            if(event->type == InputTypeShort) app->usb_want = !app->usb_want;
            return true;
        }
        if(mode == CamModeBle) {
            if(event->type == InputTypeShort) app->ble_want = !app->ble_want;
            return true;
        }
        if(event->type == InputTypeShort) {
            if(app->sensor_state == SensorStateOk) {
                app->capture_request = true;
            } else {
                thermal_toast(app, "No sensor");
            }
        } else if(event->type == InputTypeLong) {
            if(app->sensor_state == SensorStateOk && !app->recording) {
                app->recording = true;
            }
        } else if(event->type == InputTypeRelease) {
            /* Hold-to-record: releasing OK ends the take. A plain short press
             * never sets `recording`, so this is a no-op for stills. */
            if(app->recording) app->recording = false;
        }
        return true;

    case InputKeyLeft:
    case InputKeyRight:
        if(event->type == InputTypeShort) {
            if(app->recording) {
                thermal_toast(
                    app,
                    app->rec_remote ? "REC from PC: stop it there" : "Release OK to stop REC");
            } else {
                cycle_mode(app, event->key == InputKeyRight ? +1 : -1);
            }
        }
        return true;

    case InputKeyUp:
    case InputKeyDown:
        if(event->type == InputTypeShort || event->type == InputTypeRepeat) {
            adjust_param(app, event->key == InputKeyUp ? +1 : -1);
        }
        return true;

    case InputKeyBack:
        if(event->type == InputTypeShort) {
            thermal_settings_view_build(app);
            view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSettings);
        } else if(event->type == InputTypeLong || event->type == InputTypeRepeat) {
            /* Hold Back to exit. Repeat is accepted too, so a Long event
             * lost for any reason cannot leave the user stuck in the app.
             * The firmware finishes the exit when the key is released. */
            thermal_request_exit(app);
        }
        return true;

    default:
        return false;
    }
}

/* ---------------------------------------------------------------------------
 * About screen
 * ------------------------------------------------------------------------ */

static void about_draw_callback(Canvas* canvas, void* model) {
    UNUSED(model);
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    canvas_draw_rframe(canvas, 0, 0, 128, 64, 3);

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 4, AlignCenter, AlignTop, "Thermal Camera");

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(
        canvas, 64, 15, AlignCenter, AlignTop, "Version " THERMAL_APP_VERSION " - MLX90640");

    canvas_draw_line(canvas, 20, 26, 107, 26);

    canvas_draw_str_aligned(canvas, 64, 29, AlignCenter, AlignTop, "Designed by");

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 39, AlignCenter, AlignTop, THERMAL_APP_AUTHOR);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 64, 51, AlignCenter, AlignTop, THERMAL_APP_EMAIL);
}

static bool about_input_callback(InputEvent* event, void* context) {
    ThermalCamApp* app = context;
    if(event->key == InputKeyBack && event->type == InputTypeShort) {
        view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSettings);
    }
    /* Consume everything: Back must return to the settings list, not run the
     * settings-closing navigation handler. */
    return true;
}

View* thermal_about_view_alloc(ThermalCamApp* app) {
    View* view = view_alloc();
    view_set_context(view, app);
    view_set_draw_callback(view, about_draw_callback);
    view_set_input_callback(view, about_input_callback);
    return view;
}

void thermal_about_view_free(View* view) {
    if(view) view_free(view);
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------ */

View* thermal_camera_view_alloc(ThermalCamApp* app) {
    furi_assert(app);

    View* view = view_alloc();
    view_allocate_model(view, ViewModelTypeLockFree, sizeof(ThermalViewModel));

    ThermalViewModel* m = view_get_model(view);
    m->app = app;
    view_commit_model(view, false);

    view_set_context(view, app);
    view_set_draw_callback(view, camera_draw_callback);
    view_set_input_callback(view, camera_input_callback);

    return view;
}

void thermal_camera_view_free(View* view) {
    if(view) view_free(view);
}
