/**
 * Settings: defaults, persistence and the settings menu.
 */

#include "thermal_cam.h"

#include <stddef.h>
#include <string.h>

#define SETTINGS_PATH    THERMAL_APP_DIR "/settings.bin"
#define SETTINGS_MAGIC   0x4D4C5831UL /* "MLX1" */
#define SETTINGS_VERSION 3 /* v1/v2 files still load: see thermal_settings_load() */

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    ThermalSettings settings;
} SettingsFile;

/* ---------------------------------------------------------------------------
 * Parameter tables
 * ------------------------------------------------------------------------ */

const uint8_t thermal_gamma_x100[GAMMA_STEPS] =
    {40, 50, 60, 70, 80, 90, 100, 120, 140, 170, 200, 250};
const uint8_t thermal_contour_x10[CONTOUR_STEPS] = {5, 10, 20, 30, 50, 100};
const uint8_t thermal_spot_sizes[SPOT_STEPS] = {1, 3, 5, 7};
const uint16_t thermal_graph_seconds[GRAPH_STEPS] = {10, 30, 60, 120, 300, 600};
const uint16_t thermal_fever_x10[FEVER_STEPS] = {373, 375, 378, 380, 385};
const uint16_t thermal_low_x10[LOW_STEPS] = {340, 345, 350, 355, 360};

/* ---------------------------------------------------------------------------
 * Defaults and persistence
 * ------------------------------------------------------------------------ */

static void set_v3_defaults(ThermalSettings* s) {
    s->graph_log = false;
    s->body_offset_x10 = BODY_OFFSET_DEFAULT_X10;
    s->fever_idx = FEVER_DEFAULT_IDX;
    s->low_idx = LOW_DEFAULT_IDX;
}

void thermal_settings_set_defaults(ThermalSettings* s) {
    furi_assert(s);
    memset(s, 0, sizeof(*s));

    s->range_mode = RangeAutoLock;
    s->manual_min = 20;
    s->manual_max = 40;
    s->emissivity_pct = 95; /* skin, paint, most matte surfaces */
    s->rate = Mlx90640Rate8Hz;
    s->speed = Mlx90640Speed400k;
    s->render_mode = RenderLinear;
    s->palette = PaletteIronbow;
    s->threshold_mode = ThresholdOff;
    s->threshold_low = 40;
    s->threshold_high = 80;
    s->export_scale = 4; /* 128x96 */
    s->export_format = ExportPng;
    s->anim_format = AnimPngSeq;
    s->save_csv = false;
    s->bilinear = true;
    s->mirror = false;
    s->fahrenheit = false;
    s->chess_mode = true; /* recommended by the datasheet */
    s->gamma_idx = GAMMA_DEFAULT_IDX;
    s->eq_idx = EQ_DEFAULT_IDX;
    s->contour_idx = CONTOUR_DEFAULT_IDX;
    s->spot_idx = SPOT_DEFAULT_IDX;
    s->graph_idx = GRAPH_DEFAULT_IDX;
    set_v3_defaults(s);
}

/* A bool read back from a file can hold any byte. Loading such a value as a
 * bool is undefined behaviour (and the compiler may assume 0/1), so fix it up
 * through its byte representation. */
static void fix_bool(bool* b) {
    uint8_t raw;
    memcpy(&raw, b, 1);
    raw = raw ? 1 : 0;
    memcpy(b, &raw, 1);
}

void thermal_settings_sanitize(ThermalSettings* s) {
    /* Clamp anything a hand-edited, stale or partially written file could
     * have put out of range. Every table index is checked, so nothing read
     * from the SD card can ever index past a table. */
    if(s->range_mode >= RangeModeCount) s->range_mode = RangeAutoLock;
    if(s->emissivity_pct < 50 || s->emissivity_pct > 100) s->emissivity_pct = 95;
    if(s->rate >= Mlx90640RateCount) s->rate = Mlx90640Rate8Hz;
    if(s->speed >= Mlx90640SpeedCount) s->speed = Mlx90640Speed400k;
    if(s->render_mode >= CamModeCount) s->render_mode = CamModeLinear;
    /* Never start the app straight into a streaming mode: streaming is
     * opt-in. */
    if(cam_mode_is_stream(s->render_mode)) s->render_mode = CamModeLinear;
    if(s->palette >= PaletteCount) s->palette = PaletteIronbow;
    if(s->threshold_mode >= ThresholdCount) s->threshold_mode = ThresholdOff;
    if(s->export_format >= ExportFormatCount) s->export_format = ExportPng;
    if(s->anim_format >= AnimFormatCount) s->anim_format = AnimPngSeq;
    if(s->export_scale != 1 && s->export_scale != 2 && s->export_scale != 4 &&
       s->export_scale != 8) {
        s->export_scale = 4;
    }
    if(s->manual_min < -40 || s->manual_min > 300) s->manual_min = 20;
    if(s->manual_max < -40 || s->manual_max > 300) s->manual_max = 40;
    if(s->manual_max <= s->manual_min) s->manual_max = (int16_t)(s->manual_min + 10);
    if(s->threshold_low < -40 || s->threshold_low > 300) s->threshold_low = 40;
    if(s->threshold_high < -40 || s->threshold_high > 300) s->threshold_high = 80;
    if(s->gamma_idx >= GAMMA_STEPS) s->gamma_idx = GAMMA_DEFAULT_IDX;
    if(s->eq_idx >= EQ_STEPS) s->eq_idx = EQ_DEFAULT_IDX;
    if(s->contour_idx >= CONTOUR_STEPS) s->contour_idx = CONTOUR_DEFAULT_IDX;
    if(s->spot_idx >= SPOT_STEPS) s->spot_idx = SPOT_DEFAULT_IDX;
    if(s->graph_idx >= GRAPH_STEPS) s->graph_idx = GRAPH_DEFAULT_IDX;
    if(s->body_offset_x10 < BODY_OFFSET_MIN_X10 || s->body_offset_x10 > BODY_OFFSET_MAX_X10) {
        s->body_offset_x10 = BODY_OFFSET_DEFAULT_X10;
    }
    if(s->fever_idx >= FEVER_STEPS) s->fever_idx = FEVER_DEFAULT_IDX;
    if(s->low_idx >= LOW_STEPS) s->low_idx = LOW_DEFAULT_IDX;
    fix_bool(&s->save_csv);
    fix_bool(&s->bilinear);
    fix_bool(&s->mirror);
    fix_bool(&s->fahrenheit);
    fix_bool(&s->chess_mode);
    fix_bool(&s->graph_log);
}

void thermal_settings_load(ThermalCamApp* app) {
    furi_assert(app);
    thermal_settings_set_defaults(&app->settings);

    File* file = storage_file_alloc(app->storage);
    if(storage_file_open(file, SETTINGS_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        SettingsFile sf;
        const size_t head = offsetof(SettingsFile, settings);
        if(storage_file_read(file, &sf, head) == head && sf.magic == SETTINGS_MAGIC &&
           sf.version >= 1 && sf.version <= SETTINGS_VERSION && sf.size > 0 &&
           sf.size <= sizeof(ThermalSettings)) {
            /* Fields are only ever appended, so an older (shorter) file is a
             * prefix of the current struct: copy what it has over the
             * defaults and the newer fields keep their default values. */
            ThermalSettings loaded = app->settings;
            if(storage_file_read(file, &loaded, sf.size) == sf.size) {
                if(sf.version < 2) {
                    /* v1 was saved with its own trailing padding, which now
                     * overlaps the first v2 field: reset the v2 block. */
                    loaded.gamma_idx = GAMMA_DEFAULT_IDX;
                    loaded.eq_idx = EQ_DEFAULT_IDX;
                    loaded.contour_idx = CONTOUR_DEFAULT_IDX;
                    loaded.spot_idx = SPOT_DEFAULT_IDX;
                    loaded.graph_idx = GRAPH_DEFAULT_IDX;
                }
                if(sf.version < 3) {
                    /* Same for the v3 block and a v2 file's padding. */
                    set_v3_defaults(&loaded);
                }
                app->settings = loaded;
            }
        }
        storage_file_close(file);
    }
    storage_file_free(file);

    thermal_settings_sanitize(&app->settings);
}

void thermal_settings_save(ThermalCamApp* app) {
    furi_assert(app);

    SettingsFile sf = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .size = sizeof(ThermalSettings),
    };

    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    sf.settings = app->settings;
    furi_mutex_release(app->settings_mutex);

    File* file = storage_file_alloc(app->storage);
    if(storage_file_open(file, SETTINGS_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_write(file, &sf, sizeof(sf));
        storage_file_close(file);
    }
    storage_file_free(file);
}

/* ---------------------------------------------------------------------------
 * Settings menu
 * ------------------------------------------------------------------------ */

static const char* const range_names[RangeModeCount] = {"Auto", "Auto Smooth", "Manual"};
static const char* const rate_names[Mlx90640RateCount] =
    {"0.5 Hz", "1 Hz", "2 Hz", "4 Hz", "8 Hz", "16 Hz", "32 Hz", "64 Hz"};
static const char* const speed_names[Mlx90640SpeedCount] = {"100 kHz", "400 kHz", "1 MHz*"};
static const char* const export_names[ExportFormatCount] = {"PNG", "TIFF", "PNG+TIFF"};
static const char* const anim_names[AnimFormatCount] = {"PNG seq", "RAW bin"};
static const char* const onoff[2] = {"OFF", "ON"};
static const char* const unit_names[2] = {"Celsius", "Fahrenht"};
static const char* const readmode_names[2] = {"Interleave", "Chess"};

static const uint8_t scale_values[4] = {1, 2, 4, 8};
static const char* const scale_names[4] = {"32x24", "64x48", "128x96", "256x192"};

/** Mutate one field under the settings mutex and flag the worker. */
#define SETTING_CHANGE(app, body)                                   \
    do {                                                            \
        furi_mutex_acquire((app)->settings_mutex, FuriWaitForever); \
        body;                                                       \
        furi_mutex_release((app)->settings_mutex);                  \
        (app)->settings_dirty = true;                               \
    } while(0)

static void on_range_mode(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, range_names[i]);
    SETTING_CHANGE(app, app->settings.range_mode = i);
}

static void on_range_min(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const int16_t v = (int16_t)(-40 + (int)i * 5);

    char buf[12];
    snprintf(buf, sizeof(buf), "%d C", (int)v);
    variable_item_set_current_value_text(item, buf);
    SETTING_CHANGE(app, app->settings.manual_min = v);
}

static void on_range_max(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const int16_t v = (int16_t)(-40 + (int)i * 5);

    char buf[12];
    snprintf(buf, sizeof(buf), "%d C", (int)v);
    variable_item_set_current_value_text(item, buf);
    SETTING_CHANGE(app, app->settings.manual_max = v);
}

static void on_emissivity(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const uint8_t pct = (uint8_t)(50 + i);

    char buf[12];
    snprintf(buf, sizeof(buf), "0.%02u", (unsigned)(pct % 100));
    if(pct == 100) snprintf(buf, sizeof(buf), "1.00");
    variable_item_set_current_value_text(item, buf);
    SETTING_CHANGE(app, app->settings.emissivity_pct = pct);
}

static void on_rate(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, rate_names[i]);
    SETTING_CHANGE(app, app->settings.rate = i);
}

static void on_speed(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, speed_names[i]);
    SETTING_CHANGE(app, app->settings.speed = i);
}

static void on_readmode(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, readmode_names[i]);
    SETTING_CHANGE(app, app->settings.chess_mode = (i == 1));
}

static void on_interp(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, onoff[i]);
    SETTING_CHANGE(app, app->settings.bilinear = (i == 1));
}

static void on_mirror(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, onoff[i]);
    SETTING_CHANGE(app, app->settings.mirror = (i == 1));
}

static void on_units(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, unit_names[i]);
    SETTING_CHANGE(app, app->settings.fahrenheit = (i == 1));
}

static void on_thr_mode(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, thermal_threshold_names[i]);
    SETTING_CHANGE(app, app->settings.threshold_mode = i);
}

static void on_thr_low(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const int16_t v = (int16_t)(-40 + (int)i * 5);

    char buf[12];
    snprintf(buf, sizeof(buf), "%d C", (int)v);
    variable_item_set_current_value_text(item, buf);
    SETTING_CHANGE(app, app->settings.threshold_low = v);
}

static void on_thr_high(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const int16_t v = (int16_t)(-40 + (int)i * 5);

    char buf[12];
    snprintf(buf, sizeof(buf), "%d C", (int)v);
    variable_item_set_current_value_text(item, buf);
    SETTING_CHANGE(app, app->settings.threshold_high = v);
}

static void on_palette(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, thermal_palette_names[i]);
    SETTING_CHANGE(app, app->settings.palette = i);
}

static void on_export(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, export_names[i]);
    SETTING_CHANGE(app, app->settings.export_format = i);
}

static void on_scale(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, scale_names[i]);
    SETTING_CHANGE(app, app->settings.export_scale = scale_values[i]);
}

static void on_csv(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, onoff[i]);
    SETTING_CHANGE(app, app->settings.save_csv = (i == 1));
}

static void on_anim(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, anim_names[i]);
    SETTING_CHANGE(app, app->settings.anim_format = i);
}

/* -- mode parameters (the values Up/Down adjusts on the camera screen) -- */

static void set_gamma_text(VariableItem* item, uint8_t i) {
    char buf[12];
    const unsigned g = thermal_gamma_x100[i];
    snprintf(buf, sizeof(buf), "%u.%02u", g / 100u, g % 100u);
    variable_item_set_current_value_text(item, buf);
}

static void on_gamma(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_gamma_text(item, i);
    SETTING_CHANGE(app, app->settings.gamma_idx = i);
}

static void set_eq_text(VariableItem* item, uint8_t i) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%u%%", (unsigned)i * 10u);
    variable_item_set_current_value_text(item, buf);
}

static void on_eq(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_eq_text(item, i);
    SETTING_CHANGE(app, app->settings.eq_idx = i);
}

static void set_contour_text(VariableItem* item, uint8_t i, bool fahrenheit) {
    char buf[12];
    const unsigned c = thermal_contour_x10[i];
    const char u = fahrenheit ? 'F' : 'C';
    if(c % 10u) {
        snprintf(buf, sizeof(buf), "%u.%u %c", c / 10u, c % 10u, u);
    } else {
        snprintf(buf, sizeof(buf), "%u %c", c / 10u, u);
    }
    variable_item_set_current_value_text(item, buf);
}

static void on_contour(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_contour_text(item, i, app->settings.fahrenheit);
    SETTING_CHANGE(app, app->settings.contour_idx = i);
}

static void set_spot_text(VariableItem* item, uint8_t i) {
    static const char* const names[SPOT_STEPS] = {"1 px", "3 px", "5 px", "7 px"};
    variable_item_set_current_value_text(item, names[i]);
}

static void on_spot(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_spot_text(item, i);
    SETTING_CHANGE(app, app->settings.spot_idx = i);
}

static void set_graph_text(VariableItem* item, uint8_t i) {
    char buf[12];
    const unsigned sec = thermal_graph_seconds[i];
    if(sec >= 60) {
        snprintf(buf, sizeof(buf), "%u min", sec / 60u);
    } else {
        snprintf(buf, sizeof(buf), "%u s", sec);
    }
    variable_item_set_current_value_text(item, buf);
}

static void on_graph(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_graph_text(item, i);
    SETTING_CHANGE(app, app->settings.graph_idx = i);
}

static void on_graph_log(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    variable_item_set_current_value_text(item, onoff[i]);
    SETTING_CHANGE(app, app->settings.graph_log = (i == 1));
}

/* -- Human mode -- */

static void set_offset_text(VariableItem* item, int8_t x10) {
    char buf[12];
    const int a = x10 < 0 ? -x10 : x10;
    snprintf(buf, sizeof(buf), "%c%d.%d C", x10 < 0 ? '-' : '+', a / 10, a % 10);
    variable_item_set_current_value_text(item, buf);
}

static void on_body_offset(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    const int8_t v = (int8_t)(BODY_OFFSET_MIN_X10 + (int)i);
    set_offset_text(item, v);
    SETTING_CHANGE(app, app->settings.body_offset_x10 = v);
}

static void set_x10_text(VariableItem* item, uint16_t x10) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%u.%u C", (unsigned)(x10 / 10u), (unsigned)(x10 % 10u));
    variable_item_set_current_value_text(item, buf);
}

static void on_fever(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_x10_text(item, thermal_fever_x10[i]);
    SETTING_CHANGE(app, app->settings.fever_idx = i);
}

static void on_low(VariableItem* item) {
    ThermalCamApp* app = variable_item_get_context(item);
    const uint8_t i = variable_item_get_current_value_index(item);
    set_x10_text(item, thermal_low_x10[i]);
    SETTING_CHANGE(app, app->settings.low_idx = i);
}

/* Rows are counted as they are added, so the action rows' indices can never
 * drift out of step when rows are added or reordered. */
static uint32_t row_count = 0;
static uint32_t about_index = UINT32_MAX;
static uint32_t reset_index = UINT32_MAX;
static uint32_t exit_index = UINT32_MAX;

/* -- factory reset -- */

static void confirm_reset_callback(DialogExResult result, void* context) {
    ThermalCamApp* app = context;

    if(result == DialogExResultRight) {
        /* Streams belong to modes the defaults do not start in. */
        app->usb_want = false;
        app->ble_want = false;

        furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
        thermal_settings_set_defaults(&app->settings);
        furi_mutex_release(app->settings_mutex);

        app->settings_dirty = true;
        thermal_settings_save(app);
        thermal_settings_view_build(app);
        variable_item_list_set_selected_item(app->settings_list, reset_index);
        notification_message(app->notifications, &sequence_success);
    }

    if(result == DialogExResultRight || result == DialogExResultLeft) {
        view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSettings);
    }
}

static void show_reset_confirm(ThermalCamApp* app) {
    DialogEx* d = app->confirm_dialog;
    dialog_ex_reset(d);
    dialog_ex_set_header(d, "Factory reset?", 64, 2, AlignCenter, AlignTop);
    dialog_ex_set_text(
        d,
        "All settings go back\nto their defaults.\nSaved images are kept.",
        64,
        16,
        AlignCenter,
        AlignTop);
    dialog_ex_set_left_button_text(d, "Cancel");
    dialog_ex_set_right_button_text(d, "Reset");
    dialog_ex_set_result_callback(d, confirm_reset_callback);
    dialog_ex_set_context(d, app);
    view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdConfirm);
}

static VariableItem* add_row(
    VariableItemList* list,
    const char* label,
    uint8_t values,
    VariableItemChangeCallback cb,
    void* ctx) {
    row_count++;
    return variable_item_list_add(list, label, values, cb, ctx);
}

static void on_enter(void* context, uint32_t index) {
    ThermalCamApp* app = context;
    if(index == about_index) {
        view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdAbout);
    } else if(index == reset_index) {
        show_reset_confirm(app);
    } else if(index == exit_index) {
        thermal_request_exit(app);
    }
}

/** Index of `value` in a -40..300 step-5 list. */
static uint8_t temp_index(int16_t value) {
    int i = ((int)value + 40) / 5;
    if(i < 0) i = 0;
    if(i > 68) i = 68;
    return (uint8_t)i;
}
#define TEMP_CHOICES 69 /* -40..300 in steps of 5 */

void thermal_settings_view_build(ThermalCamApp* app) {
    furi_assert(app);

    VariableItemList* list = app->settings_list;
    variable_item_list_reset(list);
    row_count = 0;

    ThermalSettings s;
    furi_mutex_acquire(app->settings_mutex, FuriWaitForever);
    s = app->settings;
    furi_mutex_release(app->settings_mutex);

    VariableItem* it;
    char buf[12];

    /* -- image -- */
    it = add_row(list, "Range", RangeModeCount, on_range_mode, app);
    variable_item_set_current_value_index(it, s.range_mode);
    variable_item_set_current_value_text(it, range_names[s.range_mode]);

    it = add_row(list, "Range Min", TEMP_CHOICES, on_range_min, app);
    variable_item_set_current_value_index(it, temp_index(s.manual_min));
    snprintf(buf, sizeof(buf), "%d C", (int)s.manual_min);
    variable_item_set_current_value_text(it, buf);

    it = add_row(list, "Range Max", TEMP_CHOICES, on_range_max, app);
    variable_item_set_current_value_index(it, temp_index(s.manual_max));
    snprintf(buf, sizeof(buf), "%d C", (int)s.manual_max);
    variable_item_set_current_value_text(it, buf);

    it = add_row(list, "Interpolate", 2, on_interp, app);
    variable_item_set_current_value_index(it, s.bilinear ? 1 : 0);
    variable_item_set_current_value_text(it, onoff[s.bilinear ? 1 : 0]);

    it = add_row(list, "Mirror", 2, on_mirror, app);
    variable_item_set_current_value_index(it, s.mirror ? 1 : 0);
    variable_item_set_current_value_text(it, onoff[s.mirror ? 1 : 0]);

    it = add_row(list, "Units", 2, on_units, app);
    variable_item_set_current_value_index(it, s.fahrenheit ? 1 : 0);
    variable_item_set_current_value_text(it, unit_names[s.fahrenheit ? 1 : 0]);

    /* -- per-mode parameters; also adjustable live with Up/Down -- */
    it = add_row(list, "Gamma", GAMMA_STEPS, on_gamma, app);
    variable_item_set_current_value_index(it, s.gamma_idx);
    set_gamma_text(it, s.gamma_idx);

    it = add_row(list, "EQ Strength", EQ_STEPS, on_eq, app);
    variable_item_set_current_value_index(it, s.eq_idx);
    set_eq_text(it, s.eq_idx);

    it = add_row(list, "Contour Step", CONTOUR_STEPS, on_contour, app);
    variable_item_set_current_value_index(it, s.contour_idx);
    set_contour_text(it, s.contour_idx, s.fahrenheit);

    it = add_row(list, "Spot Size", SPOT_STEPS, on_spot, app);
    variable_item_set_current_value_index(it, s.spot_idx);
    set_spot_text(it, s.spot_idx);

    it = add_row(list, "Graph Time", GRAPH_STEPS, on_graph, app);
    variable_item_set_current_value_index(it, s.graph_idx);
    set_graph_text(it, s.graph_idx);

    it = add_row(list, "Graph Log", 2, on_graph_log, app);
    variable_item_set_current_value_index(it, s.graph_log ? 1 : 0);
    variable_item_set_current_value_text(it, onoff[s.graph_log ? 1 : 0]);

    /* -- Human mode -- */
    it = add_row(
        list, "Body Offset", BODY_OFFSET_MAX_X10 - BODY_OFFSET_MIN_X10 + 1, on_body_offset, app);
    variable_item_set_current_value_index(it, (uint8_t)(s.body_offset_x10 - BODY_OFFSET_MIN_X10));
    set_offset_text(it, s.body_offset_x10);

    it = add_row(list, "Fever At", FEVER_STEPS, on_fever, app);
    variable_item_set_current_value_index(it, s.fever_idx);
    set_x10_text(it, thermal_fever_x10[s.fever_idx]);

    it = add_row(list, "Low At", LOW_STEPS, on_low, app);
    variable_item_set_current_value_index(it, s.low_idx);
    set_x10_text(it, thermal_low_x10[s.low_idx]);

    /* -- thresholds -- */
    it = add_row(list, "Threshold", ThresholdCount, on_thr_mode, app);
    variable_item_set_current_value_index(it, s.threshold_mode);
    variable_item_set_current_value_text(it, thermal_threshold_names[s.threshold_mode]);

    it = add_row(list, "Thr Low", TEMP_CHOICES, on_thr_low, app);
    variable_item_set_current_value_index(it, temp_index(s.threshold_low));
    snprintf(buf, sizeof(buf), "%d C", (int)s.threshold_low);
    variable_item_set_current_value_text(it, buf);

    it = add_row(list, "Thr High", TEMP_CHOICES, on_thr_high, app);
    variable_item_set_current_value_index(it, temp_index(s.threshold_high));
    snprintf(buf, sizeof(buf), "%d C", (int)s.threshold_high);
    variable_item_set_current_value_text(it, buf);

    /* -- sensor -- */
    it = add_row(list, "Emissivity", 51, on_emissivity, app);
    variable_item_set_current_value_index(it, (uint8_t)(s.emissivity_pct - 50));
    if(s.emissivity_pct == 100) {
        snprintf(buf, sizeof(buf), "1.00");
    } else {
        snprintf(buf, sizeof(buf), "0.%02u", (unsigned)s.emissivity_pct);
    }
    variable_item_set_current_value_text(it, buf);

    it = add_row(list, "Refresh", Mlx90640RateCount, on_rate, app);
    variable_item_set_current_value_index(it, s.rate);
    variable_item_set_current_value_text(it, rate_names[s.rate]);

    it = add_row(list, "I2C Speed", Mlx90640SpeedCount, on_speed, app);
    variable_item_set_current_value_index(it, s.speed);
    variable_item_set_current_value_text(it, speed_names[s.speed]);

    it = add_row(list, "Read Mode", 2, on_readmode, app);
    variable_item_set_current_value_index(it, s.chess_mode ? 1 : 0);
    variable_item_set_current_value_text(it, readmode_names[s.chess_mode ? 1 : 0]);

    /* -- saving -- */
    it = add_row(list, "Palette", PaletteCount, on_palette, app);
    variable_item_set_current_value_index(it, s.palette);
    variable_item_set_current_value_text(it, thermal_palette_names[s.palette]);

    it = add_row(list, "Save As", ExportFormatCount, on_export, app);
    variable_item_set_current_value_index(it, s.export_format);
    variable_item_set_current_value_text(it, export_names[s.export_format]);

    uint8_t scale_idx = 2;
    for(uint8_t i = 0; i < COUNT_OF(scale_values); i++) {
        if(scale_values[i] == s.export_scale) scale_idx = i;
    }
    it = add_row(list, "Img Size", COUNT_OF(scale_values), on_scale, app);
    variable_item_set_current_value_index(it, scale_idx);
    variable_item_set_current_value_text(it, scale_names[scale_idx]);

    it = add_row(list, "Save CSV", 2, on_csv, app);
    variable_item_set_current_value_index(it, s.save_csv ? 1 : 0);
    variable_item_set_current_value_text(it, onoff[s.save_csv ? 1 : 0]);

    it = add_row(list, "Animation", AnimFormatCount, on_anim, app);
    variable_item_set_current_value_index(it, s.anim_format);
    variable_item_set_current_value_text(it, anim_names[s.anim_format]);

    /* -- actions: press OK -- */
    it = add_row(list, "Factory Reset", 1, NULL, app);
    variable_item_set_current_value_text(it, "OK >");
    reset_index = row_count - 1;

    it = add_row(list, "About", 1, NULL, app);
    variable_item_set_current_value_text(it, "v" THERMAL_APP_VERSION " >");
    about_index = row_count - 1;

    it = add_row(list, "Exit App", 1, NULL, app);
    variable_item_set_current_value_text(it, "OK >");
    exit_index = row_count - 1;

    variable_item_list_set_enter_callback(list, on_enter, app);
}
