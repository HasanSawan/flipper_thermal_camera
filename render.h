/**
 * 1-bit rendering for the thermal camera.
 *
 * The Flipper display has no greyscale, so continuous tone is faked with an
 * ordered (Bayer 8x8) dither -- the same trick used for shading in 1-bit
 * games. Output is an XBM bitmap: LSB-first within each byte, stride
 * (width + 7) / 8, which is exactly what canvas_draw_xbm() consumes.
 */
#pragma once

#include "img_export.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The sensor is 4:3 and the screen is 2:1. Filling the full 64 px height at
 * the sensor's own aspect ratio gives 24 -> 64 (x2.667), so 32 -> 85 px wide,
 * leaving 43 px for the readout sidebar. Nothing is stretched. */
#define THERMAL_IMG_W      85
#define THERMAL_IMG_H      64
#define THERMAL_IMG_STRIDE ((THERMAL_IMG_W + 7) / 8) /* 11 bytes */
#define THERMAL_IMG_BYTES  (THERMAL_IMG_STRIDE * THERMAL_IMG_H) /* 704 */

#define THERMAL_SIDEBAR_X (THERMAL_IMG_W + 2) /* 87 */

/** How temperatures are mapped to 1-bit dots. */
typedef enum {
    RenderLinear = 0, /* dot density rises with temperature */
    RenderInverted, /* dot density falls with temperature */
    RenderHistEq, /* histogram-equalised, maximises local contrast */
    RenderContour, /* iso-thermal contour lines only */
    RenderThreshold, /* solid fill for pixels passing the threshold */
    RenderModeCount,
} RenderMode;

extern const char* const thermal_render_names[RenderModeCount];

typedef struct {
    float range_min;
    float range_max;
    RenderMode mode;
    ThresholdMode threshold_mode;
    float threshold_low;
    float threshold_high;
    bool bilinear;
    bool mirror;

    /* Linear / Inverted: tone curve applied before dithering. 1.0 = straight;
     * above 1 spends more dot levels on the hot end, below 1 on the cold end. */
    float gamma;
    /* Hist EQ: 0 = plain linear, 1 = fully equalised. */
    float histeq_strength;
    /* Contour: one iso-line wherever (t + contour_offset) crosses a multiple of
     * contour_step (both in degrees C). The offset lets Fahrenheit users get
     * lines on round Fahrenheit values. */
    float contour_step;
    float contour_offset;
} ThermalRenderConfig;

/**
 * Render 768 temperatures into an XBM bitmap.
 * `xbm_out` must hold THERMAL_IMG_BYTES and is fully overwritten.
 */
void thermal_render(const float* temps, const ThermalRenderConfig* cfg, uint8_t* xbm_out);

#ifdef __cplusplus
}
#endif
