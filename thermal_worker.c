/**
 * Sensor worker thread.
 *
 * Everything that can block -- I2C transfers, the calibration maths, SD card
 * writes, USB transfers -- happens here. The GUI thread only ever reads the
 * published display buffer, so a slow SD write or a stalled USB host can never
 * stall or crash the UI.
 */

#include "thermal_cam.h"
#include "usb_stream.h"
#include "ble_stream.h"

#include <furi_hal_rtc.h>
#include <datetime/datetime.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TAG "ThermalWorker"

#define WORKER_STACK_SIZE (5 * 1024) /* graph log + BLE paths need more than 4 kB */
#define FRAME_TIMEOUT_MS  1500
#define RETRY_DELAY_MS    800

/* Chess-mode frames carry one subpage each, so two reads make a full image. */
#define FRAMES_BEFORE_VALID 2

typedef struct {
    uint8_t rate;
    uint8_t speed;
    bool chess_mode;
    bool applied;
} AppliedConfig;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------ */

static void make_timestamp(char* buf, size_t len) {
    DateTime dt;
    furi_hal_rtc_get_datetime(&dt);
    snprintf(
        buf,
        len,
        "%04u%02u%02u_%02u%02u%02u",
        (unsigned)dt.year,
        (unsigned)dt.month,
        (unsigned)dt.day,
        (unsigned)dt.hour,
        (unsigned)dt.minute,
        (unsigned)dt.second);
}

static void settings_snapshot(ThermalCamApp* app, ThermalSettings* out) {
    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    *out = app->settings;
    furi_mutex_release(app->settings_mutex);
}

/** Apply the settings that require talking to the sensor. */
static void
    apply_sensor_config(ThermalCamApp* app, const ThermalSettings* s, AppliedConfig* applied) {
    if(applied->applied && applied->speed == s->speed && applied->rate == s->rate &&
       applied->chess_mode == s->chess_mode) {
        return;
    }

    if(!applied->applied || applied->speed != s->speed) {
        mlx90640_set_speed(app->mlx, (Mlx90640Speed)s->speed);
    }
    if(!applied->applied || applied->rate != s->rate) {
        mlx90640_set_rate(app->mlx, (Mlx90640Rate)s->rate);
    }
    if(!applied->applied || applied->chess_mode != s->chess_mode) {
        mlx90640_set_chess_mode(app->mlx, s->chess_mode);
    }

    applied->speed = s->speed;
    applied->rate = s->rate;
    applied->chess_mode = s->chess_mode;
    applied->applied = true;
}

/* ---------------------------------------------------------------------------
 * Frame statistics
 * ------------------------------------------------------------------------ */

typedef struct {
    float t_min, t_max, t_center;
    uint16_t idx_min, idx_max;
    float t_spot;
    uint8_t spot_count;
    float t_skin; /* hottest pixel of the central face area */
} FrameStats;

/* Human mode looks for the hottest point of a face (the inner eye corners and
 * forehead run warmest) inside this central window of the sensor. */
#define FACE_X0 8
#define FACE_X1 23
#define FACE_Y0 3
#define FACE_Y1 20

/**
 * Average over a disc of `diameter` sensor pixels, centred on pixel (16, 12)
 * -- the pixel just below-right of the optical centre. For odd diameters this
 * gives the familiar shapes: 1 -> 1 px, 3 -> 3x3 (9), 5 -> 21, 7 -> 37.
 */
static float spot_average(const float* temps, uint8_t diameter, uint8_t* count_out) {
    const int cx = THERMAL_SENSOR_W / 2;
    const int cy = THERMAL_SENSOR_H / 2;
    const int r = diameter / 2;
    /* (d/2)^2, i.e. r^2 + r + 0.25, compared in integers as 4x */
    const int lim4 = (int)diameter * (int)diameter;

    float sum = 0.0f;
    int n = 0;
    for(int dy = -r; dy <= r; dy++) {
        for(int dx = -r; dx <= r; dx++) {
            if(4 * (dx * dx + dy * dy) > lim4) continue;
            sum += temps[(cy + dy) * THERMAL_SENSOR_W + (cx + dx)];
            n++;
        }
    }
    *count_out = (uint8_t)n;
    return n ? sum / (float)n : temps[cy * THERMAL_SENSOR_W + cx];
}

static void compute_stats(const float* temps, const ThermalSettings* s, FrameStats* st) {
    float mn = temps[0];
    float mx = temps[0];
    uint16_t imn = 0;
    uint16_t imx = 0;

    for(uint16_t i = 1; i < THERMAL_SENSOR_W * THERMAL_SENSOR_H; i++) {
        const float t = temps[i];
        if(t < mn) {
            mn = t;
            imn = i;
        }
        if(t > mx) {
            mx = t;
            imx = i;
        }
    }

    /* Sidebar centre readout: the four central pixels, steadier than one. */
    const int c0 = 11 * THERMAL_SENSOR_W + 15;
    const float centre = (temps[c0] + temps[c0 + 1] + temps[c0 + THERMAL_SENSOR_W] +
                          temps[c0 + THERMAL_SENSOR_W + 1]) *
                         0.25f;

    st->t_min = mn;
    st->t_max = mx;
    st->t_center = centre;
    st->idx_min = imn;
    st->idx_max = imx;
    st->t_spot = spot_average(temps, thermal_spot_sizes[s->spot_idx], &st->spot_count);

    float skin = temps[FACE_Y0 * THERMAL_SENSOR_W + FACE_X0];
    for(int y = FACE_Y0; y <= FACE_Y1; y++) {
        for(int x = FACE_X0; x <= FACE_X1; x++) {
            const float t = temps[y * THERMAL_SENSOR_W + x];
            if(t > skin) skin = t;
        }
    }
    st->t_skin = skin;
}

/* ---------------------------------------------------------------------------
 * Human mode
 *
 * A screening estimate only: the hottest skin in the face area, smoothed,
 * plus a user-calibrated skin-to-core offset. The status needs a few frames
 * in a row and a small hysteresis before it changes, so the LED and the
 * vibration alert do not flicker at a boundary.
 * ------------------------------------------------------------------------ */

#define HUMAN_SKIN_MIN      30.0f /* colder than any face: nobody in view */
#define HUMAN_SKIN_MAX      43.0f /* hotter than any skin: a mug, a lamp... */
#define HUMAN_HYST          0.1f
#define HUMAN_STABLE_FRAMES 3

typedef struct {
    float ema;
    bool have;
    uint8_t status; /* HumanStatus, debounced */
    uint8_t candidate;
    uint8_t stable;
    uint8_t shown; /* status currently on the LED */
} HumanState;

static uint8_t human_classify(const ThermalSettings* s, float body, uint8_t current) {
    const float fever = (float)thermal_fever_x10[s->fever_idx] / 10.0f;
    const float low = (float)thermal_low_x10[s->low_idx] / 10.0f;

    /* Leaving a state needs HUMAN_HYST past its edge. */
    const float fever_edge = (current == HumanFever) ? fever - HUMAN_HYST : fever;
    const float low_edge = (current == HumanLow) ? low + HUMAN_HYST : low;

    if(body >= fever_edge) return HumanFever;
    if(body <= low_edge) return HumanLow;
    return HumanNormal;
}

static void human_update(HumanState* h, const ThermalSettings* s, float skin) {
    uint8_t want;
    if(skin < HUMAN_SKIN_MIN || skin > HUMAN_SKIN_MAX) {
        h->have = false;
        want = HumanNone;
    } else {
        if(!h->have) {
            h->ema = skin;
            h->have = true;
        } else {
            h->ema += (skin - h->ema) * 0.3f;
        }
        const float body = h->ema + (float)s->body_offset_x10 / 10.0f;
        want = human_classify(s, body, h->status);
    }

    if(want == h->status) {
        h->stable = 0;
    } else if(want == h->candidate) {
        if(++h->stable >= HUMAN_STABLE_FRAMES) {
            h->status = want;
            h->stable = 0;
        }
    } else {
        h->candidate = want;
        h->stable = 1;
    }
}

static float human_body(const HumanState* h, const ThermalSettings* s) {
    return h->ema + (float)s->body_offset_x10 / 10.0f;
}

/** Show the status on the LED; alert once when it turns abnormal. */
static void human_signal(ThermalCamApp* app, HumanState* h) {
    if(h->status == h->shown) return;
    h->shown = h->status;

    switch(h->status) {
    case HumanFever:
        notification_message(app->notifications, &sequence_set_only_red_255);
        notification_message(app->notifications, &sequence_double_vibro);
        break;
    case HumanLow:
        notification_message(app->notifications, &sequence_set_only_blue_255);
        notification_message(app->notifications, &sequence_single_vibro);
        break;
    case HumanNormal:
        notification_message(app->notifications, &sequence_set_only_green_255);
        break;
    default:
        notification_message(app->notifications, &sequence_reset_rgb);
        break;
    }
}

/** Leaving Human mode: LED off, and start from scratch next time. */
static void human_reset(ThermalCamApp* app, HumanState* h) {
    if(h->shown != HumanNone) notification_message(app->notifications, &sequence_reset_rgb);
    memset(h, 0, sizeof(*h));
}

/** Decide the display range for this frame, smoothing when asked to. */
static void resolve_range(
    const ThermalSettings* s,
    const FrameStats* st,
    float* smooth_lo,
    float* smooth_hi,
    bool first,
    float* out_lo,
    float* out_hi) {
    float lo, hi;

    if(s->range_mode == RangeManual) {
        lo = (float)s->manual_min;
        hi = (float)s->manual_max;
    } else {
        lo = st->t_min;
        hi = st->t_max;

        if(s->range_mode == RangeAutoLock) {
            if(first) {
                *smooth_lo = lo;
                *smooth_hi = hi;
            } else {
                /* Slew-limited so the picture does not pump frame to frame. */
                const float k = 0.15f;
                *smooth_lo += (lo - *smooth_lo) * k;
                *smooth_hi += (hi - *smooth_hi) * k;
            }
            lo = *smooth_lo;
            hi = *smooth_hi;
        }
    }

    if(hi - lo < 1.0f) {
        const float mid = (hi + lo) * 0.5f;
        lo = mid - 0.5f;
        hi = mid + 0.5f;
    }

    *out_lo = lo;
    *out_hi = hi;
}

/* ---------------------------------------------------------------------------
 * Spot-meter history
 *
 * One column per graph pixel. Columns advance with wall-clock time, not with
 * frames, so the time axis is honest at any refresh rate. Each column keeps
 * the min/max it saw plus its last value, which lets the drawing join
 * neighbouring columns into a continuous trace.
 * ------------------------------------------------------------------------ */

/**
 * Called with columns that are about to fall out of the buffer (or when the
 * buffer is reset), so the graph log can write them before they are lost.
 * The columns are the `h->pending` ones ending at `h->last_done`.
 */
typedef void (*HistoryFlush)(SpotHistory* h, void* ctx);

static void history_flush(SpotHistory* h, HistoryFlush flush, void* ctx) {
    if(h->pending && flush) flush(h, ctx);
    h->pending = 0;
}

/** Before a reset or on exit: the column in progress is final too. */
static void history_close(SpotHistory* h, HistoryFlush flush, void* ctx) {
    if(h->started && h->has[h->head]) {
        h->last_done = h->head;
        h->pending++;
    }
    history_flush(h, flush, ctx);
}

static void history_reset(SpotHistory* h, uint16_t seconds, uint32_t now) {
    memset(h->has, 0, sizeof(h->has));
    h->head = GRAPH_W - 1;
    h->last_done = GRAPH_W - 1;
    h->pending = 0;
    h->seconds = seconds;
    h->col_ticks = furi_ms_to_ticks((uint32_t)seconds * 1000u / GRAPH_W);
    if(h->col_ticks == 0) h->col_ticks = 1;
    h->col_start_tick = now;
    h->started = true;
}

static void
    history_add(SpotHistory* h, uint16_t seconds, float value, HistoryFlush flush, void* flush_ctx) {
    const uint32_t now = furi_get_tick();

    if(!h->started || h->seconds != seconds) {
        history_close(h, flush, flush_ctx);
        history_reset(h, seconds, now);
    }

    uint32_t elapsed = now - h->col_start_tick;
    if(elapsed >= h->col_ticks) {
        const uint32_t steps = elapsed / h->col_ticks;
        if(steps >= GRAPH_W) {
            /* Away longer than the whole window (e.g. sat in settings with the
             * sensor unplugged): nothing in the buffer is still in range. */
            history_close(h, flush, flush_ctx);
            history_reset(h, seconds, now);
        } else {
            for(uint32_t i = 0; i < steps; i++) {
                h->last_done = h->head;
                h->pending++;
                /* The ring is full of unlogged columns: write them out before
                 * the next step overwrites the oldest one. */
                if(h->pending >= GRAPH_W - 1) history_flush(h, flush, flush_ctx);
                h->head = (uint8_t)((h->head + 1) % GRAPH_W);
                h->has[h->head] = false;
            }
            h->col_start_tick += steps * h->col_ticks;
        }
    }

    const uint8_t c = h->head;
    if(!h->has[c]) {
        h->mn[c] = value;
        h->mx[c] = value;
        h->stamp[c] = furi_hal_rtc_get_timestamp();
        h->has[c] = true;
    } else {
        if(value < h->mn[c]) h->mn[c] = value;
        if(value > h->mx[c]) h->mx[c] = value;
    }
    h->last[c] = value;
}

/** Map the history to screen rows for the draw callback. */
static void history_publish(const SpotHistory* h, DisplayBuffer* db) {
    float lo = 0.0f;
    float hi = 0.0f;
    bool any = false;

    for(int i = 0; i < GRAPH_W; i++) {
        if(!h->has[i]) continue;
        if(!any) {
            lo = h->mn[i];
            hi = h->mx[i];
            any = true;
        } else {
            if(h->mn[i] < lo) lo = h->mn[i];
            if(h->mx[i] > hi) hi = h->mx[i];
        }
    }

    db->graph_valid = any;
    db->graph_seconds = h->seconds;
    if(!any) return;

    db->graph_win_min = lo;
    db->graph_win_max = hi;

    /* At least 1 degree of vertical span so sensor noise does not look like
     * a mountain range, plus a little headroom top and bottom. */
    float span = hi - lo;
    if(span < 1.0f) {
        const float mid = (hi + lo) * 0.5f;
        lo = mid - 0.5f;
        hi = mid + 0.5f;
        span = 1.0f;
    }
    lo -= span * 0.08f;
    hi += span * 0.08f;
    db->graph_lo = lo;
    db->graph_hi = hi;

    const float scale = (float)(GRAPH_H - 1) / (hi - lo);
#define ROW(v) ((int)(GRAPH_Y + GRAPH_H - 1) - (int)(((v) - lo) * scale + 0.5f))

    bool have_prev = false;
    float prev = 0.0f;
    for(int x = 0; x < GRAPH_W; x++) {
        const int c = (h->head + 1 + x) % GRAPH_W; /* oldest first */
        if(!h->has[c]) {
            db->graph_top[x] = GRAPH_EMPTY;
            db->graph_bot[x] = GRAPH_EMPTY;
            continue;
        }

        float cmin = h->mn[c];
        float cmax = h->mx[c];
        /* Join to the previous sample so the trace has no gaps. */
        if(have_prev) {
            if(prev < cmin) cmin = prev;
            if(prev > cmax) cmax = prev;
        }
        int top = ROW(cmax);
        int bot = ROW(cmin);
        if(top < GRAPH_Y) top = GRAPH_Y;
        if(bot > GRAPH_Y + GRAPH_H - 1) bot = GRAPH_Y + GRAPH_H - 1;
        if(bot < top) bot = top;
        db->graph_top[x] = (uint8_t)top;
        db->graph_bot[x] = (uint8_t)bot;

        prev = h->last[c];
        have_prev = true;
    }
#undef ROW
}

/* ---------------------------------------------------------------------------
 * Publishing
 * ------------------------------------------------------------------------ */

static void
    fill_render_config(const ThermalSettings* s, float lo, float hi, ThermalRenderConfig* rc) {
    const float contour_unit = (float)thermal_contour_x10[s->contour_idx] / 10.0f;

    rc->range_min = lo;
    rc->range_max = hi;
    rc->mode = cam_mode_is_image(s->render_mode) ? (RenderMode)s->render_mode : RenderLinear;
    /* The threshold belongs to the Threshold view only. */
    rc->threshold_mode = (s->render_mode == CamModeThreshold) ? (ThresholdMode)s->threshold_mode :
                                                                ThresholdOff;
    rc->threshold_low = (float)s->threshold_low;
    rc->threshold_high = (float)s->threshold_high;
    rc->bilinear = s->bilinear;
    rc->mirror = s->mirror;
    rc->gamma = (float)thermal_gamma_x100[s->gamma_idx] / 100.0f;
    rc->histeq_strength = (float)s->eq_idx / 10.0f;
    /* Contour steps are in the display unit, so Fahrenheit users get lines on
     * round Fahrenheit values: (1.8 t + 32) / step == (t + 17.78) / (step/1.8) */
    if(s->fahrenheit) {
        rc->contour_step = contour_unit / 1.8f;
        rc->contour_offset = 32.0f / 1.8f;
    } else {
        rc->contour_step = contour_unit;
        rc->contour_offset = 0.0f;
    }
}

static void publish_frame(
    ThermalCamApp* app,
    const ThermalSettings* s,
    const FrameStats* st,
    float lo,
    float hi,
    float ambient,
    uint16_t fps_x10,
    const HumanState* hs) {
    /* Fill the buffer the GUI is not currently reading, then flip the index. */
    const uint8_t back = app->disp_active ? 0 : 1;
    DisplayBuffer* db = &app->disp[back];

    /* Only the image modes need the dithered bitmap; skipping it in the spot
     * and USB modes frees the CPU for a higher frame rate. */
    if(cam_mode_is_image(s->render_mode)) {
        ThermalRenderConfig rc;
        fill_render_config(s, lo, hi, &rc);
        thermal_render(app->temps, &rc, db->bitmap);
    }

    db->t_min = st->t_min;
    db->t_max = st->t_max;
    db->t_center = st->t_center;
    db->t_ambient = ambient;
    db->fps_x10 = fps_x10;
    db->t_spot = st->t_spot;
    db->spot_count = st->spot_count;
    db->spot_size = thermal_spot_sizes[s->spot_idx];

    db->t_skin = st->t_skin;
    db->t_body = hs->have ? human_body(hs, s) : st->t_skin + (float)s->body_offset_x10 / 10.0f;
    db->human_status = hs->status;

    if(s->render_mode == CamModeSpot) {
        history_publish(&app->history, db);
    }

    /* Marker positions in display space, accounting for mirroring. */
    uint8_t mnx =
        (uint8_t)((st->idx_min % THERMAL_SENSOR_W) * (THERMAL_IMG_W - 1) / (THERMAL_SENSOR_W - 1));
    uint8_t mxx =
        (uint8_t)((st->idx_max % THERMAL_SENSOR_W) * (THERMAL_IMG_W - 1) / (THERMAL_SENSOR_W - 1));
    if(s->mirror) {
        mnx = (uint8_t)(THERMAL_IMG_W - 1 - mnx);
        mxx = (uint8_t)(THERMAL_IMG_W - 1 - mxx);
    }

    db->min_x = mnx;
    db->max_x = mxx;
    db->min_y =
        (uint8_t)((st->idx_min / THERMAL_SENSOR_W) * (THERMAL_IMG_H - 1) / (THERMAL_SENSOR_H - 1));
    db->max_y =
        (uint8_t)((st->idx_max / THERMAL_SENSOR_W) * (THERMAL_IMG_H - 1) / (THERMAL_SENSOR_H - 1));
    db->valid = true;

    app->disp_active = back;
}

/* ---------------------------------------------------------------------------
 * Saving
 * ------------------------------------------------------------------------ */

static void fill_export_config(
    const ThermalSettings* s,
    const float* temps,
    float lo,
    float hi,
    ThermalExportConfig* cfg) {
    cfg->temps = temps;
    cfg->range_min = lo;
    cfg->range_max = hi;
    cfg->palette = (ThermalPalette)s->palette;
    /* Saved images match the screen: masked only in the Threshold view. */
    cfg->threshold_mode = (s->render_mode == CamModeThreshold) ? (ThresholdMode)s->threshold_mode :
                                                                 ThresholdOff;
    cfg->threshold_low = (float)s->threshold_low;
    cfg->threshold_high = (float)s->threshold_high;
    cfg->scale = s->export_scale;
    cfg->bilinear = s->bilinear;
    cfg->mirror = s->mirror;
}

/** "-12.34" without %f, which would pull in the heavy float formatter. */
static void format_centi(char* buf, size_t len, float v) {
    if(!isfinite(v)) v = 0.0f;
    if(v > 9999.0f) v = 9999.0f;
    if(v < -9999.0f) v = -9999.0f;
    int c = (int)lroundf(v * 100.0f);
    const bool neg = c < 0;
    if(neg) c = -c;
    snprintf(buf, len, "%s%d.%02d", neg ? "-" : "", c / 100, c % 100);
}

static bool write_str(File* f, const char* str) {
    const size_t n = strlen(str);
    return storage_file_write(f, str, n) == n;
}

static bool history_write_header(File* f, bool fahrenheit) {
    return write_str(
        f,
        fahrenheit ? "time,unix_time,min_F,max_F,last_F\r\n" :
                     "time,unix_time,min_C,max_C,last_C\r\n");
}

/**
 * Write `count` columns ending at column `last`, oldest first. Each row is
 * one graph column: its start time and the min / max / last spot temperature
 * seen during it. Columns without data (sensor gaps) are skipped.
 */
static bool
    history_write_rows(File* f, const SpotHistory* h, uint8_t last, uint8_t count, bool fahrenheit) {
    char line[96];
    char mn[16], mx[16], ls[16];

    for(int i = count - 1; i >= 0; i--) {
        const int c = ((int)last - i + GRAPH_W) % GRAPH_W;
        if(!h->has[c]) continue;

        const float k = fahrenheit ? 1.8f : 1.0f;
        const float o = fahrenheit ? 32.0f : 0.0f;
        format_centi(mn, sizeof(mn), h->mn[c] * k + o);
        format_centi(mx, sizeof(mx), h->mx[c] * k + o);
        format_centi(ls, sizeof(ls), h->last[c] * k + o);

        DateTime dt;
        datetime_timestamp_to_datetime(h->stamp[c], &dt);
        snprintf(
            line,
            sizeof(line),
            "%04u-%02u-%02u %02u:%02u:%02u,%lu,%s,%s,%s\r\n",
            (unsigned)dt.year,
            (unsigned)dt.month,
            (unsigned)dt.day,
            (unsigned)dt.hour,
            (unsigned)dt.minute,
            (unsigned)dt.second,
            (unsigned long)h->stamp[c],
            mn,
            mx,
            ls);
        if(!write_str(f, line)) return false;
    }
    return true;
}

/** Save the whole spot-history window, including the column in progress. */
static bool save_graph_csv(ThermalCamApp* app, const char* path, bool fahrenheit) {
    File* f = storage_file_alloc(app->storage);
    bool ok = storage_file_open(f, path, FSAM_WRITE, FSOM_CREATE_ALWAYS);
    if(ok) {
        ok = history_write_header(f, fahrenheit) &&
             history_write_rows(f, &app->history, app->history.head, GRAPH_W, fahrenheit);
        storage_file_close(f);
    }
    storage_file_free(f);
    return ok;
}

/* -- continuous graph log --------------------------------------------------
 * With "Graph Log" on, every column that is about to scroll off the graph is
 * appended to LOG_<start time>.csv, so the file holds the complete history
 * for as long as the app runs. One file per session. */

typedef struct {
    ThermalCamApp* app;
    char path[96]; /* empty until the first write of this session */
    bool fahrenheit;
    bool failed;
} GraphLog;

static void graph_log_flush(SpotHistory* h, void* ctx) {
    GraphLog* log = ctx;
    ThermalCamApp* app = log->app;
    if(log->failed) return;

    const bool fresh = log->path[0] == '\0';
    if(fresh) {
        char stamp[24];
        make_timestamp(stamp, sizeof(stamp));
        snprintf(log->path, sizeof(log->path), THERMAL_APP_DIR "/LOG_%s.csv", stamp);
    }

    File* f = storage_file_alloc(app->storage);
    bool ok = storage_file_open(f, log->path, FSAM_WRITE, FSOM_OPEN_APPEND);
    if(ok) {
        if(fresh) ok = history_write_header(f, log->fahrenheit);
        if(ok) ok = history_write_rows(f, h, h->last_done, h->pending, log->fahrenheit);
        storage_file_close(f);
    }
    storage_file_free(f);

    if(ok) {
        thermal_toast(app, fresh ? "Graph log started" : "Graph log saved");
    } else {
        /* Do not retry every column and flood the screen with errors. */
        log->failed = true;
        thermal_toast(app, "Graph log failed - check SD");
        notification_message(app->notifications, &sequence_error);
    }
}

static void do_still_capture(
    ThermalCamApp* app,
    const ThermalSettings* s,
    float lo,
    float hi,
    bool with_graph) {
    char stamp[24];
    char path[128];
    make_timestamp(stamp, sizeof(stamp));

    ThermalExportConfig cfg;
    fill_export_config(s, app->temps, lo, hi, &cfg);

    bool ok = true;
    int written = 0;

    if(s->export_format == ExportPng || s->export_format == ExportBoth) {
        snprintf(path, sizeof(path), THERMAL_APP_DIR "/IR_%s.png", stamp);
        if(thermal_export_png(app->storage, path, &cfg)) {
            written++;
        } else {
            ok = false;
        }
    }

    if(s->export_format == ExportTiff || s->export_format == ExportBoth) {
        snprintf(path, sizeof(path), THERMAL_APP_DIR "/IR_%s.tif", stamp);
        if(thermal_export_tiff(app->storage, path, &cfg)) {
            written++;
        } else {
            ok = false;
        }
    }

    if(s->save_csv) {
        snprintf(path, sizeof(path), THERMAL_APP_DIR "/IR_%s.csv", stamp);
        if(!thermal_export_csv(app->storage, path, app->temps)) ok = false;
    }

    if(with_graph) {
        snprintf(path, sizeof(path), THERMAL_APP_DIR "/IR_%s_graph.csv", stamp);
        if(save_graph_csv(app, path, s->fahrenheit)) {
            written++;
        } else {
            ok = false;
        }
    }

    char msg[48];
    if(ok && written > 0) {
        snprintf(msg, sizeof(msg), with_graph ? "Saved IR_%s +graph" : "Saved IR_%s", stamp);
        thermal_toast(app, msg);
        notification_message(app->notifications, &sequence_success);
    } else {
        thermal_toast(app, "Save failed - check SD");
        notification_message(app->notifications, &sequence_error);
    }
}

typedef struct {
    bool active;
    char dir[96];
    File* raw_file;
    uint32_t frames;
} RecordingState;

static void recording_start(ThermalCamApp* app, const ThermalSettings* s, RecordingState* rec) {
    char stamp[24];
    make_timestamp(stamp, sizeof(stamp));

    snprintf(rec->dir, sizeof(rec->dir), THERMAL_APP_DIR "/REC_%s", stamp);
    if(!storage_simply_mkdir(app->storage, rec->dir)) {
        thermal_toast(app, "Cannot create folder");
        notification_message(app->notifications, &sequence_error);
        return;
    }

    rec->frames = 0;
    rec->raw_file = NULL;

    if(s->anim_format == AnimRawBin) {
        char path[128];
        snprintf(path, sizeof(path), "%s/frames.bin", rec->dir);
        rec->raw_file = storage_file_alloc(app->storage);
        if(!storage_file_open(rec->raw_file, path, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
            storage_file_free(rec->raw_file);
            rec->raw_file = NULL;
            thermal_toast(app, "Cannot open raw file");
            notification_message(app->notifications, &sequence_error);
            return;
        }
    }

    rec->active = true;
    app->rec_frame_count = 0;
    thermal_toast(app, "REC started");
    notification_message(app->notifications, &sequence_blink_start_red);
}

static void recording_frame(
    ThermalCamApp* app,
    const ThermalSettings* s,
    RecordingState* rec,
    float lo,
    float hi) {
    if(!rec->active) return;

    if(s->anim_format == AnimRawBin) {
        if(rec->raw_file && !thermal_export_raw_append(rec->raw_file, app->temps)) {
            thermal_toast(app, "Raw write failed");
        }
    } else {
        char path[128];
        snprintf(path, sizeof(path), "%s/f%05lu.png", rec->dir, (unsigned long)rec->frames);

        ThermalExportConfig cfg;
        fill_export_config(s, app->temps, lo, hi, &cfg);
        /* Frames are written at native resolution so the capture keeps up. */
        cfg.scale = 1;

        if(!thermal_export_png(app->storage, path, &cfg)) {
            thermal_toast(app, "Frame write failed");
        }
    }

    rec->frames++;
    app->rec_frame_count = rec->frames;
}

static void recording_stop(ThermalCamApp* app, RecordingState* rec) {
    if(!rec->active) return;

    if(rec->raw_file) {
        storage_file_close(rec->raw_file);
        storage_file_free(rec->raw_file);
        rec->raw_file = NULL;
    }

    char msg[48];
    snprintf(msg, sizeof(msg), "REC saved %lu frames", (unsigned long)rec->frames);
    thermal_toast(app, msg);
    notification_message(app->notifications, &sequence_blink_stop);
    notification_message(app->notifications, &sequence_success);

    rec->active = false;
}

/* ---------------------------------------------------------------------------
 * Streaming to a computer: USB or Bluetooth
 *
 * Both carry the same packets and accept the same commands. Only one can be
 * open at a time, because each belongs to its own view mode and leaving the
 * mode closes it.
 * ------------------------------------------------------------------------ */

typedef struct {
    UsbStream* usb;
    BleStream* ble;
    bool usb_had_host;
} Streams;

static void stream_fill_meta(ThermalCamApp* app, const ThermalSettings* s, UsbFrameMeta* m) {
    m->ambient = mlx90640_get_ta(app->mlx);
    m->emissivity_pct = s->emissivity_pct;
    m->rate = s->rate;
    m->speed = s->speed;
    m->chess = s->chess_mode;
    m->subpage = (uint8_t)mlx90640_get_subpage(app->mlx);
}

static void streams_send_info(ThermalCamApp* app, Streams* st, const ThermalSettings* s) {
    char text[USB_STREAM_TEXT_MAX];
    snprintf(
        text,
        sizeof(text),
        "app=thermal_cam;ver=" THERMAL_APP_VERSION
        ";w=32;h=24;e=%u;r=%u;s=%u;c=%u;unit=%s;rec=%u;link=%s",
        (unsigned)s->emissivity_pct,
        (unsigned)s->rate,
        (unsigned)s->speed,
        s->chess_mode ? 1u : 0u,
        s->fahrenheit ? "F" : "C",
        app->recording ? 1u : 0u,
        st->ble ? "ble" : "usb");

    UsbFrameMeta m;
    stream_fill_meta(app, s, &m);
    if(st->usb) usb_stream_send_info(st->usb, &m, text);
    if(st->ble) ble_stream_send_info(st->ble, &m, text);
}

/** Parse a decimal number after the command letter; false if malformed. */
static bool parse_uint(const char* p, int* out) {
    if(*p < '0' || *p > '9') return false;
    int v = 0;
    while(*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        if(v > 10000) return false;
        p++;
    }
    if(*p != '\0') return false;
    *out = v;
    return true;
}

/**
 * Apply one command line from the computer. Settings changes go through the
 * mutex just like the settings menu, so the rest of the worker picks them up
 * normally. Returns true when the computer should get an info packet back.
 */
static bool apply_command(ThermalCamApp* app, Streams* st, const char* line) {
    int v = 0;
    bool changed = false;
    bool send_info = false;

    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    switch(line[0]) {
    case '?':
        send_info = true;
        break;
    case 'E':
    case 'e':
        if(parse_uint(line + 1, &v) && v >= 50 && v <= 100) {
            app->settings.emissivity_pct = (uint8_t)v;
            changed = true;
        }
        break;
    case 'R':
    case 'r':
        if(parse_uint(line + 1, &v) && v < Mlx90640RateCount) {
            app->settings.rate = (uint8_t)v;
            changed = true;
        }
        break;
    case 'S':
    case 's':
        if(parse_uint(line + 1, &v) && v < Mlx90640SpeedCount) {
            app->settings.speed = (uint8_t)v;
            changed = true;
        }
        break;
    case 'C':
    case 'c':
        if(parse_uint(line + 1, &v) && v <= 1) {
            app->settings.chess_mode = (v == 1);
            changed = true;
        }
        break;
    case 'K':
    case 'k':
        app->capture_request = true;
        break;
    case 'G':
    case 'g':
        app->graph_request = true;
        break;
    case 'V':
    case 'v':
        /* Record on the Flipper's SD card, started and stopped remotely. */
        if(parse_uint(line + 1, &v) && v <= 1) {
            if(v == 1 && !app->recording && app->sensor_state == SensorStateOk) {
                app->rec_remote = true;
                app->recording = true;
            } else if(v == 0 && app->recording) {
                app->recording = false;
            }
            send_info = true;
        }
        break;
    case 'M':
    case 'm':
        if(st->ble && parse_uint(line + 1, &v)) {
            ble_stream_set_chunk(st->ble, (uint16_t)v);
            send_info = true; /* arriving intact confirms the new size */
        }
        break;
    default:
        FURI_LOG_D(TAG, "unknown command '%s'", line);
        break;
    }
    furi_mutex_release(app->settings_mutex);

    if(changed) {
        app->settings_dirty = true;
        send_info = true; /* confirm the new state back to the PC */
    }
    return send_info;
}

static void streams_handle_commands(ThermalCamApp* app, Streams* st) {
    char line[24];
    for(;;) {
        bool got = false;
        if(st->usb) got = usb_stream_poll_command(st->usb, line, sizeof(line));
        if(!got && st->ble) got = ble_stream_poll_command(st->ble, line, sizeof(line));
        if(!got) break;

        if(apply_command(app, st, line)) {
            ThermalSettings snap;
            settings_snapshot(app, &snap);
            streams_send_info(app, st, &snap);
        }
    }
}

/** Open or close the ports to match what the GUI asked for. */
static void streams_sync(ThermalCamApp* app, Streams* st) {
    if(app->usb_want && !st->usb) {
        st->usb = usb_stream_start();
        if(st->usb) {
            app->usb_frames_sent = 0;
            app->usb_frames_dropped = 0;
            app->usb_state = UsbStateWaiting;
            thermal_toast(app, "USB stream on ttyACM1");
            notification_message(app->notifications, &sequence_blink_start_cyan);
        } else {
            app->usb_want = false;
            app->usb_state = UsbStateError;
            thermal_toast(app, "USB busy - cannot stream");
            notification_message(app->notifications, &sequence_error);
        }
    } else if(!app->usb_want && st->usb) {
        usb_stream_stop(st->usb);
        st->usb = NULL;
        app->usb_state = UsbStateOff;
        notification_message(app->notifications, &sequence_blink_stop);
    }

    if(app->ble_want && !st->ble) {
        st->ble = ble_stream_start();
        if(st->ble) {
            app->ble_frames_sent = 0;
            app->ble_frames_dropped = 0;
            app->ble_state = BleStateWaiting;
            thermal_toast(app, "Bluetooth stream on");
            notification_message(app->notifications, &sequence_blink_start_blue);
        } else {
            app->ble_want = false;
            app->ble_state = BleStateError;
            thermal_toast(app, "Bluetooth unavailable");
            notification_message(app->notifications, &sequence_error);
        }
    } else if(!app->ble_want && st->ble) {
        ble_stream_stop(st->ble);
        st->ble = NULL;
        app->ble_state = BleStateOff;
        notification_message(app->notifications, &sequence_blink_stop);
    }

    /* A recording the computer started ends with the connection. */
    if(!st->usb && !st->ble && app->rec_remote && app->recording) app->recording = false;
    if(!st->usb) st->usb_had_host = false;
}

static void streams_update_state(ThermalCamApp* app, Streams* st, const ThermalSettings* s) {
    if(st->usb) {
        const bool had = st->usb_had_host;
        const bool host = usb_stream_host_connected(st->usb);
        if(host && !had) streams_send_info(app, st, s);
        st->usb_had_host = host;
        if(!host) app->usb_state = UsbStateWaiting;
    }
    if(st->ble) {
        switch(ble_stream_link(st->ble)) {
        case BleLinkRadioOff:
            app->ble_state = BleStateRadioOff;
            break;
        case BleLinkAdvertising:
            app->ble_state = BleStateWaiting;
            break;
        case BleLinkConnected:
            app->ble_state = BleStateConnected;
            break;
        case BleLinkReady:
            if(app->ble_state != BleStateStreaming) app->ble_state = BleStateConnected;
            break;
        }
    }
}

static void streams_send_frame(ThermalCamApp* app, Streams* st, const ThermalSettings* s) {
    if(!st->usb && !st->ble) return;

    UsbFrameMeta meta;
    stream_fill_meta(app, s, &meta);

    if(st->usb) {
        const UsbSendResult r = usb_stream_send_frame(st->usb, &meta, app->temps);
        if(r == UsbSendOk) {
            app->usb_frames_sent++;
            app->usb_state = UsbStateStreaming;
        } else if(r == UsbSendTimeout) {
            app->usb_frames_dropped++;
        } else {
            app->usb_state = UsbStateWaiting;
        }
    }
    if(st->ble) {
        const UsbSendResult r = ble_stream_send_frame(st->ble, &meta, app->temps);
        if(r == UsbSendOk) {
            app->ble_frames_sent++;
            app->ble_state = BleStateStreaming;
        } else if(r == UsbSendTimeout) {
            app->ble_frames_dropped++;
        }
    }
}

static void streams_close(ThermalCamApp* app, Streams* st) {
    /* Always hand the ports back, or the Flipper would be left in dual-USB
     * mode or without its mobile-app Bluetooth link. */
    if(st->usb) {
        usb_stream_stop(st->usb);
        st->usb = NULL;
        app->usb_state = UsbStateOff;
    }
    if(st->ble) {
        ble_stream_stop(st->ble);
        st->ble = NULL;
        app->ble_state = BleStateOff;
    }
}

static void do_graph_capture(ThermalCamApp* app, const ThermalSettings* s) {
    char stamp[24];
    char path[128];
    char msg[48];
    make_timestamp(stamp, sizeof(stamp));
    snprintf(path, sizeof(path), THERMAL_APP_DIR "/IR_%s_graph.csv", stamp);

    if(save_graph_csv(app, path, s->fahrenheit)) {
        snprintf(msg, sizeof(msg), "Saved graph IR_%s", stamp);
        thermal_toast(app, msg);
        notification_message(app->notifications, &sequence_success);
    } else {
        thermal_toast(app, "Save failed - check SD");
        notification_message(app->notifications, &sequence_error);
    }
}

/* ---------------------------------------------------------------------------
 * Thread body
 * ------------------------------------------------------------------------ */

static int32_t thermal_worker_thread(void* context) {
    ThermalCamApp* app = context;
    furi_assert(app);

    ThermalSettings s;
    settings_snapshot(app, &s);

    AppliedConfig applied = {0};
    RecordingState rec = {0};
    Streams streams = {0};
    HumanState human = {0};
    GraphLog graph_log = {.app = app};
    uint8_t prev_mode = s.render_mode;

    float smooth_lo = 0.0f;
    float smooth_hi = 0.0f;
    float last_lo = 20.0f;
    float last_hi = 40.0f;
    uint32_t frames_ok = 0;
    uint32_t last_tick = furi_get_tick();
    uint16_t fps_x10 = 0;

    /* Start from a plausible scene so the half of the image that the first
     * chess-mode subpage does not cover is never uninitialised memory. */
    for(int i = 0; i < THERMAL_SENSOR_W * THERMAL_SENSOR_H; i++) {
        app->temps[i] = 25.0f;
    }

    mlx90640_bus_acquire(app->mlx, (Mlx90640Speed)s.speed);

    while(app->worker_running) {
        settings_snapshot(app, &s);

        /* Streams first: they must be switched on/off even while the sensor
         * is missing, or a port could get stuck in streaming mode. */
        streams_sync(app, &streams);
        streams_handle_commands(app, &streams);
        streams_update_state(app, &streams, &s);

        if(s.render_mode != prev_mode) {
            if(prev_mode == CamModeHuman) human_reset(app, &human);
            prev_mode = s.render_mode;
        }

        /* A new log file for each stretch of time the log is switched on. */
        graph_log.fahrenheit = s.fahrenheit;
        if(!s.graph_log) {
            graph_log.path[0] = '\0';
            graph_log.failed = false;
        }

        /* -- (re)initialise until the sensor answers -- */
        if(!app->mlx->initialised) {
            const Mlx90640Status st = mlx90640_init(app->mlx);
            if(st != Mlx90640OkResult) {
                app->sensor_state = (st == Mlx90640ErrNoDevice) ? SensorStateNoDevice :
                                    (st == Mlx90640ErrEeprom)   ? SensorStateBadEeprom :
                                                                  SensorStateBusError;
                applied.applied = false;
                frames_ok = 0;

                /* Yield in small slices so exit and stream changes stay
                 * responsive. */
                for(int i = 0; i < RETRY_DELAY_MS / 50 && app->worker_running; i++) {
                    furi_delay_ms(50);
                    if(app->usb_want != (streams.usb != NULL)) break;
                    if(app->ble_want != (streams.ble != NULL)) break;
                }
                continue;
            }
            app->sensor_state = SensorStateOk;
        }

        apply_sensor_config(app, &s, &applied);

        if(app->settings_dirty) {
            app->settings_dirty = false;
            /* Re-render immediately so palette/threshold changes are visible
             * even while paused or between slow frames. */
            if(frames_ok >= FRAMES_BEFORE_VALID) {
                FrameStats st2;
                compute_stats(app->temps, &s, &st2);
                resolve_range(&s, &st2, &smooth_lo, &smooth_hi, false, &last_lo, &last_hi);
                publish_frame(
                    app, &s, &st2, last_lo, last_hi, mlx90640_get_ta(app->mlx), fps_x10, &human);
            }
        }

        /* -- recording lifecycle -- */
        if(app->recording && !rec.active) {
            recording_start(app, &s, &rec);
            if(!rec.active) app->recording = false;
        } else if(!app->recording && rec.active) {
            recording_stop(app, &rec);
        }
        if(!app->recording) app->rec_remote = false;

        if(app->paused) {
            furi_delay_ms(30);
            continue;
        }

        /* -- read and convert one subpage -- */
        const Mlx90640Status st = mlx90640_read_frame(app->mlx, FRAME_TIMEOUT_MS);
        if(st != Mlx90640OkResult) {
            FURI_LOG_W(TAG, "frame read failed: %d", (int)st);
            app->sensor_state = SensorStateBusError;
            /* Force a full re-init: the sensor may have been unplugged. */
            app->mlx->initialised = false;
            continue;
        }

        app->sensor_state = SensorStateOk;

        /* Human mode measures skin, whatever the emissivity setting says. */
        const uint8_t e_pct = (s.render_mode == CamModeHuman) ? HUMAN_EMISSIVITY_PCT :
                                                                s.emissivity_pct;
        const float emissivity = (float)e_pct / 100.0f;
        const float ta = mlx90640_get_ta(app->mlx);
        /* Reflected temperature: the usual approximation for an indoor scene. */
        const float tr = ta - 8.0f;

        mlx90640_calculate_to(app->mlx, emissivity, tr, app->temps);
        mlx90640_fix_bad_pixels(app->mlx, app->temps);

        frames_ok++;
        if(frames_ok < FRAMES_BEFORE_VALID) continue;

        /* fps, smoothed a little so the readout is readable */
        const uint32_t now = furi_get_tick();
        const uint32_t dt = now - last_tick;
        last_tick = now;
        if(dt > 0) {
            const uint16_t inst = (uint16_t)((furi_kernel_get_tick_frequency() * 10u) / dt);
            fps_x10 = fps_x10 ? (uint16_t)((fps_x10 * 3u + inst) / 4u) : inst;
        }

        FrameStats stats;
        compute_stats(app->temps, &s, &stats);
        resolve_range(
            &s,
            &stats,
            &smooth_lo,
            &smooth_hi,
            frames_ok == FRAMES_BEFORE_VALID,
            &last_lo,
            &last_hi);

        /* History keeps running in every mode, so switching to the spot meter
         * shows the recent past straight away. */
        history_add(
            &app->history,
            thermal_graph_seconds[s.graph_idx],
            stats.t_spot,
            s.graph_log ? graph_log_flush : NULL,
            &graph_log);

        if(s.render_mode == CamModeHuman) {
            human_update(&human, &s, stats.t_skin);
            /* Nothing is signalled until the disclaimer has been accepted. */
            if(app->human_ack) human_signal(app, &human);
        }

        /* -- stream before drawing: the PC is the latency-sensitive consumer -- */
        streams_send_frame(app, &streams, &s);

        publish_frame(app, &s, &stats, last_lo, last_hi, ta, fps_x10, &human);

        /* -- capture work, after publishing so the UI stays live -- */
        if(app->capture_request) {
            app->capture_request = false;
            /* In the spot meter the graph is part of what is on screen. */
            do_still_capture(app, &s, last_lo, last_hi, s.render_mode == CamModeSpot);
        }
        if(app->graph_request) {
            app->graph_request = false;
            do_graph_capture(app, &s);
        }

        recording_frame(app, &s, &rec, last_lo, last_hi);
    }

    if(rec.active) recording_stop(app, &rec);
    app->rec_remote = false;

    /* Whatever the graph log has not written yet. */
    if(s.graph_log) history_close(&app->history, graph_log_flush, &graph_log);

    human_reset(app, &human);
    streams_close(app, &streams);

    mlx90640_bus_release(app->mlx);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------ */

void thermal_worker_start(ThermalCamApp* app) {
    furi_assert(app);
    if(app->worker) return;

    app->worker_running = true;
    app->worker =
        furi_thread_alloc_ex("ThermalWorker", WORKER_STACK_SIZE, thermal_worker_thread, app);
    furi_thread_start(app->worker);
}

void thermal_worker_stop(ThermalCamApp* app) {
    furi_assert(app);
    if(!app->worker) return;

    app->worker_running = false;
    furi_thread_join(app->worker);
    furi_thread_free(app->worker);
    app->worker = NULL;
}
