/**
 * MLX90640 thermal camera for Flipper Zero -- shared app types.
 */
#pragma once

#include "mlx90640.h"
#include "img_export.h"
#include "render.h"

#include <furi.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/variable_item_list.h>
#include <gui/modules/dialog_ex.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>

#ifdef __cplusplus
extern "C" {
#endif

#define THERMAL_APP_VERSION "1.2"
#define THERMAL_APP_AUTHOR  "Hasan SAWAN"
#define THERMAL_APP_EMAIL   "hsn.saw@gmail.com"

#define THERMAL_APP_DIR EXT_PATH("thermal_cam")

typedef enum {
    ViewIdCamera = 0,
    ViewIdSettings,
    ViewIdAbout,
    ViewIdConfirm, /* "reset all settings?" */
} ThermalViewId;

typedef enum {
    SensorStateInit = 0,
    SensorStateOk,
    SensorStateNoDevice,
    SensorStateBusError,
    SensorStateBadEeprom,
} SensorState;

/**
 * What Left/Right cycles through. The first five are image render modes and
 * share their values with RenderMode; the rest replace the image.
 *
 * The values are stored in the settings file, so new modes are only ever
 * appended. The order on screen is thermal_mode_order[], not this enum.
 */
typedef enum {
    CamModeLinear = RenderLinear,
    CamModeInverted = RenderInverted,
    CamModeHistEq = RenderHistEq,
    CamModeContour = RenderContour,
    CamModeThreshold = RenderThreshold,
    CamModeSpot = RenderModeCount, /* big spot-temperature readout + history */
    CamModeUsb, /* screen off, frames streamed over USB */
    /* -- 1.2 -- */
    CamModeHuman, /* body-temperature screening estimate */
    CamModeBle, /* screen off, frames streamed over Bluetooth LE */
    CamModeCount,
} CamMode;

extern const char* const thermal_mode_names[CamModeCount];
/** Left/Right order; the on-screen mode number is the position in here + 1. */
extern const uint8_t thermal_mode_order[CamModeCount];
uint8_t thermal_mode_number(uint8_t mode);

/** Modes whose screen is only a status page for a stream to a computer. */
static inline bool cam_mode_is_stream(uint8_t mode) {
    return mode == CamModeUsb || mode == CamModeBle;
}

static inline bool cam_mode_is_image(uint8_t mode) {
    return mode < (uint8_t)RenderModeCount;
}

typedef enum {
    ExportPng = 0,
    ExportTiff,
    ExportBoth,
    ExportFormatCount,
} ExportFormat;

typedef enum {
    AnimPngSeq = 0, /* one PNG per frame, viewable straight away */
    AnimRawBin, /* float32 dump, fastest possible capture */
    AnimFormatCount,
} AnimFormat;

typedef enum {
    RangeAuto = 0, /* track min/max of each frame */
    RangeAutoLock, /* track, but smooth so the image does not pump */
    RangeManual,
    RangeModeCount,
} RangeMode;

/* ---------------------------------------------------------------------------
 * Tables behind the mode-specific Up/Down parameters. Settings store an index
 * into these so a stale file can never produce an out-of-range value.
 * ------------------------------------------------------------------------ */

#define GAMMA_STEPS 12
extern const uint8_t thermal_gamma_x100[GAMMA_STEPS]; /* 0.40 .. 2.50 */
#define GAMMA_DEFAULT_IDX 6 /* 1.00 */

#define EQ_STEPS       11 /* 0 %, 10 % .. 100 % */
#define EQ_DEFAULT_IDX 10

#define CONTOUR_STEPS 6
extern const uint8_t thermal_contour_x10[CONTOUR_STEPS]; /* 0.5 .. 10 degrees */
#define CONTOUR_DEFAULT_IDX 2 /* 2 degrees */

#define SPOT_STEPS 4
extern const uint8_t thermal_spot_sizes[SPOT_STEPS]; /* 1, 3, 5, 7 sensor px */
#define SPOT_DEFAULT_IDX 2 /* 5 px diameter = 21 pixels */

#define GRAPH_STEPS 6
extern const uint16_t thermal_graph_seconds[GRAPH_STEPS]; /* 10 s .. 10 min */
#define GRAPH_DEFAULT_IDX 2 /* 60 s */

/* Human mode. Skin reads colder than the body core, so the estimate adds an
 * offset (Up/Down in Human mode) that the user calibrates against a clinical
 * thermometer. */
#define HUMAN_EMISSIVITY_PCT    98 /* human skin, all complexions */
#define BODY_OFFSET_MIN_X10     (-20)
#define BODY_OFFSET_MAX_X10     50
#define BODY_OFFSET_DEFAULT_X10 10 /* +1.0 C */
#define FEVER_STEPS             5
extern const uint16_t thermal_fever_x10[FEVER_STEPS]; /* 37.3 .. 38.5 */
#define FEVER_DEFAULT_IDX 3 /* 38.0 C */
#define LOW_STEPS         5
extern const uint16_t thermal_low_x10[LOW_STEPS]; /* 34.0 .. 36.0 */
#define LOW_DEFAULT_IDX 2 /* 35.0 C */

/**
 * Persisted in a small config file next to the captures.
 *
 * New fields are only ever appended: the loader copies however many bytes the
 * file holds on top of the defaults, so an older file still loads and simply
 * leaves the newer fields at their defaults.
 */
typedef struct {
    /* -- v1 -- */
    uint8_t range_mode; /* RangeMode */
    int16_t manual_min; /* degrees C */
    int16_t manual_max;
    uint8_t emissivity_pct; /* 50..100 */
    uint8_t rate; /* Mlx90640Rate */
    uint8_t speed; /* Mlx90640Speed */
    uint8_t render_mode; /* CamMode (named for file compatibility) */
    uint8_t palette; /* ThermalPalette, used on export */
    uint8_t threshold_mode; /* ThresholdMode */
    int16_t threshold_low; /* degrees C */
    int16_t threshold_high;
    uint8_t export_scale; /* 1, 2, 4 or 8 */
    uint8_t export_format; /* ExportFormat */
    uint8_t anim_format; /* AnimFormat */
    bool save_csv;
    bool bilinear;
    bool mirror;
    bool fahrenheit;
    bool chess_mode;
    /* -- v2 (app 1.1) -- */
    uint8_t gamma_idx; /* index into thermal_gamma_x100 */
    uint8_t eq_idx; /* strength = eq_idx * 10 % */
    uint8_t contour_idx; /* index into thermal_contour_x10 */
    uint8_t spot_idx; /* index into thermal_spot_sizes */
    uint8_t graph_idx; /* index into thermal_graph_seconds */
    /* -- v3 (app 1.2) -- */
    bool graph_log; /* append the spot history to a CSV each time it fills */
    int8_t body_offset_x10; /* Human mode: core minus skin, 0.1 C steps */
    uint8_t fever_idx; /* index into thermal_fever_x10 */
    uint8_t low_idx; /* index into thermal_low_x10 */
} ThermalSettings;

/* ---------------------------------------------------------------------------
 * Published frame
 * ------------------------------------------------------------------------ */

/* Spot-meter graph geometry (display pixels). */
#define GRAPH_X     0
#define GRAPH_Y     38
#define GRAPH_W     98
#define GRAPH_H     26
#define GRAPH_EMPTY 0xFF

/**
 * One rendered frame plus its readout values.
 *
 * Two of these are kept and swapped by index so the draw callback never has to
 * take a lock: the worker fills the inactive buffer, then publishes it with a
 * single index store.
 */
typedef struct {
    uint8_t bitmap[THERMAL_IMG_BYTES];
    float t_min;
    float t_max;
    float t_center;
    float t_ambient;
    uint8_t min_x, min_y;
    uint8_t max_x, max_y;
    uint16_t fps_x10;
    bool valid;

    /* Spot meter */
    float t_spot;
    uint8_t spot_size; /* diameter, sensor pixels */
    uint8_t spot_count; /* pixels averaged */
    /* History, already mapped to screen rows (GRAPH_EMPTY = no data). Column
     * 0 is the oldest, GRAPH_W-1 the newest. */
    uint8_t graph_top[GRAPH_W];
    uint8_t graph_bot[GRAPH_W];
    float graph_lo; /* value at the bottom edge */
    float graph_hi; /* value at the top edge */
    float graph_win_min; /* actual extremes in the window */
    float graph_win_max;
    uint16_t graph_seconds;
    bool graph_valid;

    /* Human mode */
    float t_skin; /* hottest skin in the central face area, raw */
    float t_body; /* smoothed skin + offset: the estimate shown */
    uint8_t human_status; /* HumanStatus */
} DisplayBuffer;

typedef enum {
    HumanNone = 0, /* nothing warm enough to be skin in view */
    HumanLow,
    HumanNormal,
    HumanFever,
} HumanStatus;

/** Spot history, owned by the worker. One column per graph pixel. */
typedef struct {
    float mn[GRAPH_W];
    float mx[GRAPH_W];
    float last[GRAPH_W];
    uint32_t stamp[GRAPH_W]; /* RTC time (unix s) the column started */
    bool has[GRAPH_W];
    uint8_t head; /* column currently being filled */
    uint8_t last_done; /* most recently completed column */
    uint8_t pending; /* completed columns not yet written to the graph log */
    uint32_t col_start_tick;
    uint32_t col_ticks;
    uint16_t seconds; /* period the buffer was built for */
    bool started;
} SpotHistory;

typedef enum {
    UsbStateOff = 0, /* not streaming */
    UsbStateWaiting, /* port up, no program has the port open */
    UsbStateStreaming,
    UsbStateError, /* could not switch USB mode */
} UsbState;

typedef enum {
    BleStateOff = 0, /* not streaming */
    BleStateWaiting, /* advertising, no computer connected */
    BleStateConnected, /* connected, not yet subscribed to the data */
    BleStateStreaming,
    BleStateRadioOff, /* Bluetooth is switched off in the Flipper settings */
    BleStateError, /* could not start the profile */
} BleState;

typedef struct {
    Gui* gui;
    ViewDispatcher* view_dispatcher;
    View* camera_view;
    View* about_view;
    VariableItemList* settings_list;
    DialogEx* confirm_dialog;
    NotificationApp* notifications;
    Storage* storage;
    FuriTimer* redraw_timer;

    Mlx90640* mlx;
    FuriThread* worker;

    FuriMutex* settings_mutex;
    ThermalSettings settings;

    DisplayBuffer disp[2];
    volatile uint8_t disp_active;

    /* Worker-owned: the temperatures behind the currently displayed frame. */
    float* temps;
    /* Worker-owned: spot-meter history. */
    SpotHistory history;

    volatile bool worker_running;
    volatile bool settings_dirty;
    volatile bool paused;
    volatile bool capture_request;
    volatile bool graph_request; /* save the spot history as CSV */
    volatile bool recording;
    volatile bool rec_remote; /* recording was started from the computer */
    volatile uint32_t rec_frame_count;
    volatile SensorState sensor_state;

    /* USB streaming. The GUI sets usb_want; the worker owns the port and
     * reports back through the other fields. */
    volatile bool usb_want;
    volatile UsbState usb_state;
    volatile uint32_t usb_frames_sent;
    volatile uint32_t usb_frames_dropped;

    /* Bluetooth streaming, same split as USB. */
    volatile bool ble_want;
    volatile BleState ble_state;
    volatile uint32_t ble_frames_sent;
    volatile uint32_t ble_frames_dropped;

    /* Human mode: the disclaimer has been acknowledged this session. */
    volatile bool human_ack;
    volatile bool exiting;

    /* Short status line shown over the image, cleared on a deadline. */
    char toast[48];
    volatile uint32_t toast_until_tick;
    FuriMutex* toast_mutex;
} ThermalCamApp;

/** The camera View's model holds only a back-pointer; it never changes. */
typedef struct {
    ThermalCamApp* app;
} ThermalViewModel;

/* -- settings ------------------------------------------------------------ */
void thermal_settings_set_defaults(ThermalSettings* s);
void thermal_settings_sanitize(ThermalSettings* s);
void thermal_settings_load(ThermalCamApp* app);
void thermal_settings_save(ThermalCamApp* app);

/* -- views --------------------------------------------------------------- */
View* thermal_camera_view_alloc(ThermalCamApp* app);
void thermal_camera_view_free(View* view);
void thermal_settings_view_build(ThermalCamApp* app);
View* thermal_about_view_alloc(ThermalCamApp* app);
void thermal_about_view_free(View* view);

/* -- worker -------------------------------------------------------------- */
void thermal_worker_start(ThermalCamApp* app);
void thermal_worker_stop(ThermalCamApp* app);

/* -- app --------------------------------------------------------------- */
/** Leave the app: stops the worker's I/O and the view dispatcher. */
void thermal_request_exit(ThermalCamApp* app);

/* -- helpers ------------------------------------------------------------- */
void thermal_toast(ThermalCamApp* app, const char* text);
void thermal_format_temp(char* buf, size_t len, float celsius, bool fahrenheit);
/** Format a temperature *difference* (no 32 degree offset for Fahrenheit). */
void thermal_format_delta(char* buf, size_t len, float celsius_delta, bool fahrenheit);

#ifdef __cplusplus
}
#endif
