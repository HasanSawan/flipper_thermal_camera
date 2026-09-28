/**
 * MLX90640 Thermal Camera for Flipper Zero -- version 1.2
 * Designed by Hasan SAWAN <hsn.saw@gmail.com>
 *
 * Threading model
 *   GUI thread     : view dispatcher, drawing, key handling
 *   Worker thread  : I2C, calibration maths, rendering, SD card and USB I/O
 *   Redraw timer   : nudges the camera view at a fixed rate
 *
 * The two threads share only the double-buffered display data (published with
 * a single index store) and the settings struct (guarded by a mutex), so the
 * UI can never be blocked by a slow sensor read or SD write.
 */

#include "thermal_cam.h"

#include <string.h>
#include <stdlib.h>

#define TAG "ThermalCam"

#define REDRAW_PERIOD_MS 40 /* 25 Hz; the sensor is always slower than this */
#define TOAST_MS         1500

void thermal_toast(ThermalCamApp* app, const char* text) {
    furi_assert(app && text);

    furi_mutex_acquire(app->toast_mutex, FuriWaitForever);
    strncpy(app->toast, text, sizeof(app->toast) - 1);
    app->toast[sizeof(app->toast) - 1] = '\0';
    furi_mutex_release(app->toast_mutex);

    app->toast_until_tick = furi_get_tick() + furi_ms_to_ticks(TOAST_MS);
}

void thermal_request_exit(ThermalCamApp* app) {
    furi_assert(app);
    if(app->exiting) return;
    app->exiting = true;

    /* Let the worker start closing files and ports straight away, while the
     * key that asked for this may still be held down. */
    app->recording = false;
    app->usb_want = false;
    app->ble_want = false;
    app->worker_running = false;

    thermal_toast(app, "Exiting...");
    view_dispatcher_stop(app->view_dispatcher);
}

/* ---------------------------------------------------------------------------
 * Callbacks
 * ------------------------------------------------------------------------ */

static void redraw_timer_callback(void* context) {
    ThermalCamApp* app = context;
    /* Committing the (empty) model is what asks the dispatcher to repaint.
     * It is a no-op when the camera view is not the current one. */
    ThermalViewModel* m = view_get_model(app->camera_view);
    UNUSED(m);
    view_commit_model(app->camera_view, true);
}

/**
 * Reached only from the settings list: the camera view consumes Back itself.
 */
static bool thermal_navigation_callback(void* context) {
    ThermalCamApp* app = context;

    thermal_settings_save(app);
    app->settings_dirty = true;
    app->paused = false;

    view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdCamera);
    return true; /* handled: do not exit the app */
}

/** Back on the confirmation dialog returns to the settings list. */
static uint32_t confirm_previous_callback(void* context) {
    UNUSED(context);
    return ViewIdSettings;
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------ */

static ThermalCamApp* thermal_cam_app_alloc(void) {
    ThermalCamApp* app = malloc(sizeof(ThermalCamApp));
    furi_check(app);
    memset(app, 0, sizeof(ThermalCamApp));

    app->temps = malloc(sizeof(float) * THERMAL_SENSOR_W * THERMAL_SENSOR_H);
    furi_check(app->temps);

    app->settings_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->toast_mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    app->gui = furi_record_open(RECORD_GUI);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    app->storage = furi_record_open(RECORD_STORAGE);

    storage_simply_mkdir(app->storage, THERMAL_APP_DIR);
    thermal_settings_load(app);

    app->mlx = mlx90640_alloc();
    app->sensor_state = SensorStateInit;
    app->disp_active = 0;

    app->view_dispatcher = view_dispatcher_alloc();
    /* Note: on firmware older than 0.98 this needs an explicit
     * view_dispatcher_enable_queue(app->view_dispatcher); here. */
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_navigation_event_callback(
        app->view_dispatcher, thermal_navigation_callback);

    app->camera_view = thermal_camera_view_alloc(app);
    view_dispatcher_add_view(app->view_dispatcher, ViewIdCamera, app->camera_view);

    app->settings_list = variable_item_list_alloc();
    view_dispatcher_add_view(
        app->view_dispatcher, ViewIdSettings, variable_item_list_get_view(app->settings_list));

    app->about_view = thermal_about_view_alloc(app);
    view_dispatcher_add_view(app->view_dispatcher, ViewIdAbout, app->about_view);

    app->confirm_dialog = dialog_ex_alloc();
    View* confirm_view = dialog_ex_get_view(app->confirm_dialog);
    view_set_previous_callback(confirm_view, confirm_previous_callback);
    view_dispatcher_add_view(app->view_dispatcher, ViewIdConfirm, confirm_view);

    app->redraw_timer = furi_timer_alloc(redraw_timer_callback, FuriTimerTypePeriodic, app);

    return app;
}

static void thermal_cam_app_free(ThermalCamApp* app) {
    furi_assert(app);

    /* Stop producers before anything they touch is torn down. */
    thermal_worker_stop(app);

    if(app->redraw_timer) {
        furi_timer_stop(app->redraw_timer);
        furi_timer_free(app->redraw_timer);
    }

    thermal_settings_save(app);

    view_dispatcher_remove_view(app->view_dispatcher, ViewIdCamera);
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdSettings);
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdAbout);
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdConfirm);

    thermal_camera_view_free(app->camera_view);
    thermal_about_view_free(app->about_view);
    variable_item_list_free(app->settings_list);
    dialog_ex_free(app->confirm_dialog);
    view_dispatcher_free(app->view_dispatcher);

    mlx90640_free(app->mlx);

    furi_record_close(RECORD_STORAGE);
    furi_record_close(RECORD_NOTIFICATION);
    furi_record_close(RECORD_GUI);

    furi_mutex_free(app->toast_mutex);
    furi_mutex_free(app->settings_mutex);

    free(app->temps);
    free(app);
}

/* ---------------------------------------------------------------------------
 * Entry point
 * ------------------------------------------------------------------------ */

int32_t thermal_cam_app(void* p) {
    UNUSED(p);

    ThermalCamApp* app = thermal_cam_app_alloc();

    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdCamera);

    thermal_worker_start(app);
    furi_timer_start(app->redraw_timer, furi_ms_to_ticks(REDRAW_PERIOD_MS));

    view_dispatcher_run(app->view_dispatcher);

    thermal_cam_app_free(app);
    return 0;
}
