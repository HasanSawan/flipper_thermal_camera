#include "render.h"

#include <math.h>
#include <string.h>

const char* const thermal_render_names[RenderModeCount] = {
    "Linear",
    "Inverted",
    "Hist EQ",
    "Contour",
    "Threshold",
};

/* Ordered dither matrix. Values 0..63 compared against the pixel level
 * rescaled to 0..64, so level 0 never lights and level 255 always does. */
static const uint8_t bayer8[64] = {
    0,  32, 8,  40, 2,  34, 10, 42, 48, 16, 56, 24, 50, 18, 58, 26, 12, 44, 4,  36, 14, 46,
    6,  38, 60, 28, 52, 20, 62, 30, 54, 22, 3,  35, 11, 43, 1,  33, 9,  41, 51, 19, 59, 27,
    49, 17, 57, 25, 15, 47, 7,  39, 13, 45, 5,  37, 63, 31, 55, 23, 61, 29, 53, 21,
};

#define HIST_BINS 64

static inline void xbm_set(uint8_t* xbm, int x, int y) {
    /* LSB-first within each byte, matching canvas_draw_xbm(). */
    xbm[y * THERMAL_IMG_STRIDE + (x >> 3)] |= (uint8_t)(1u << (x & 7));
}

static float sample_temp(const float* temps, float sx, float sy, bool bilinear) {
    if(!bilinear) {
        int x = (int)(sx + 0.5f);
        int y = (int)(sy + 0.5f);
        if(x < 0) x = 0;
        if(x > THERMAL_SENSOR_W - 1) x = THERMAL_SENSOR_W - 1;
        if(y < 0) y = 0;
        if(y > THERMAL_SENSOR_H - 1) y = THERMAL_SENSOR_H - 1;
        return temps[y * THERMAL_SENSOR_W + x];
    }

    if(sx < 0.0f) sx = 0.0f;
    if(sy < 0.0f) sy = 0.0f;
    if(sx > THERMAL_SENSOR_W - 1.0f) sx = THERMAL_SENSOR_W - 1.0f;
    if(sy > THERMAL_SENSOR_H - 1.0f) sy = THERMAL_SENSOR_H - 1.0f;

    const int x0 = (int)sx;
    const int y0 = (int)sy;
    const int x1 = (x0 < THERMAL_SENSOR_W - 1) ? x0 + 1 : x0;
    const int y1 = (y0 < THERMAL_SENSOR_H - 1) ? y0 + 1 : y0;

    const float fx = sx - (float)x0;
    const float fy = sy - (float)y0;

    const float t00 = temps[y0 * THERMAL_SENSOR_W + x0];
    const float t10 = temps[y0 * THERMAL_SENSOR_W + x1];
    const float t01 = temps[y1 * THERMAL_SENSOR_W + x0];
    const float t11 = temps[y1 * THERMAL_SENSOR_W + x1];

    const float top = t00 + (t10 - t00) * fx;
    const float bot = t01 + (t11 - t01) * fx;
    return top + (bot - top) * fy;
}

static bool passes_threshold(const ThermalRenderConfig* cfg, float t) {
    switch(cfg->threshold_mode) {
    case ThresholdAbove:
        return t >= cfg->threshold_low;
    case ThresholdBelow:
        return t <= cfg->threshold_low;
    case ThresholdBand:
        return (t >= cfg->threshold_low) && (t <= cfg->threshold_high);
    case ThresholdOff:
    default:
        return true;
    }
}

/** Build a 64-bin CDF over the source frame, used by RenderHistEq. */
static void build_histeq_lut(const float* temps, float lo, float span, uint8_t* lut) {
    uint16_t hist[HIST_BINS];
    memset(hist, 0, sizeof(hist));

    for(int i = 0; i < THERMAL_SENSOR_W * THERMAL_SENSOR_H; i++) {
        float n = (temps[i] - lo) / span;
        if(n < 0.0f) n = 0.0f;
        if(n > 1.0f) n = 1.0f;
        int bin = (int)(n * (HIST_BINS - 1) + 0.5f);
        hist[bin]++;
    }

    uint32_t cum = 0;
    const uint32_t total = THERMAL_SENSOR_W * THERMAL_SENSOR_H;
    for(int b = 0; b < HIST_BINS; b++) {
        cum += hist[b];
        lut[b] = (uint8_t)((cum * 255u) / total);
    }
}

void thermal_render(const float* temps, const ThermalRenderConfig* cfg, uint8_t* xbm) {
    furi_assert(temps && cfg && xbm);

    memset(xbm, 0, THERMAL_IMG_BYTES);

    float span = cfg->range_max - cfg->range_min;
    if(span < 0.1f) span = 0.1f;
    const float lo = cfg->range_min;

    uint8_t histeq_lut[HIST_BINS];
    int eq_k256 = 0; /* blend weight towards the equalised level, 0..256 */
    if(cfg->mode == RenderHistEq) {
        build_histeq_lut(temps, lo, span, histeq_lut);

        float k = cfg->histeq_strength;
        if(!(k >= 0.0f)) k = 0.0f; /* also catches NaN */
        if(k > 1.0f) k = 1.0f;
        eq_k256 = (int)(k * 256.0f + 0.5f);
    }

    /* Tone curve as a 256-entry table: 256 powf() per frame instead of one
     * per output pixel. */
    uint8_t gamma_lut[256];
    const bool use_gamma = (cfg->mode == RenderLinear || cfg->mode == RenderInverted) &&
                           cfg->gamma > 0.05f && fabsf(cfg->gamma - 1.0f) > 0.001f;
    if(use_gamma) {
        for(int i = 0; i < 256; i++) {
            const float v = powf((float)i / 255.0f, cfg->gamma) * 255.0f + 0.5f;
            gamma_lut[i] = (uint8_t)(v > 255.0f ? 255.0f : v);
        }
    }

    /*
     * Contour mode needs to compare neighbouring quantised levels, so it keeps
     * the previous row's levels rather than re-sampling. Levels are absolute
     * (multiples of contour_step), so a line always means the same temperature
     * regardless of the auto-range.
     */
    int16_t level_prev[THERMAL_IMG_W];
    int16_t level_curr[THERMAL_IMG_W];
    float contour_step = cfg->contour_step;
    if(!(contour_step >= 0.1f)) contour_step = 0.1f;
    const float contour_inv = 1.0f / contour_step;

    for(int y = 0; y < THERMAL_IMG_H; y++) {
        const float sy = (float)y * (float)(THERMAL_SENSOR_H - 1) / (float)(THERMAL_IMG_H - 1);

        for(int x = 0; x < THERMAL_IMG_W; x++) {
            const int src_x = cfg->mirror ? (THERMAL_IMG_W - 1 - x) : x;
            const float sx =
                (float)src_x * (float)(THERMAL_SENSOR_W - 1) / (float)(THERMAL_IMG_W - 1);

            const float t = sample_temp(temps, sx, sy, cfg->bilinear);
            const bool keep = passes_threshold(cfg, t);

            float norm = (t - lo) / span;
            if(norm < 0.0f) norm = 0.0f;
            if(norm > 1.0f) norm = 1.0f;

            uint8_t level = (uint8_t)(norm * 255.0f + 0.5f);

            switch(cfg->mode) {
            case RenderInverted:
                if(use_gamma) level = gamma_lut[level];
                level = (uint8_t)(255 - level);
                break;

            case RenderHistEq: {
                /* Blend per pixel against the full-resolution level, so 0 %
                 * is exactly Linear rather than a 64-step approximation. */
                const int bin = (int)(norm * (HIST_BINS - 1) + 0.5f);
                const int eq = histeq_lut[bin];
                level = (uint8_t)((int)level + (((eq - (int)level) * eq_k256) >> 8));
                break;
            }

            case RenderContour: {
                /* Store the quantised band; edges are drawn below. Masked
                 * pixels get a sentinel so no line is drawn inside them. */
                if(!keep) {
                    level_curr[x] = INT16_MIN;
                } else {
                    const float q = floorf((t + cfg->contour_offset) * contour_inv);
                    level_curr[x] =
                        (int16_t)(q < -30000.0f ? -30000.0f : (q > 30000.0f ? 30000.0f : q));
                }
                continue;
            }

            case RenderThreshold:
                /* Solid where the threshold passes, blank elsewhere. With no
                 * threshold configured, split at the middle of the range so
                 * the mode still shows something useful instead of a blank
                 * screen. */
                if(cfg->threshold_mode == ThresholdOff) {
                    if(norm >= 0.5f) xbm_set(xbm, x, y);
                } else if(keep) {
                    xbm_set(xbm, x, y);
                }
                continue;

            case RenderLinear:
            default:
                if(use_gamma) level = gamma_lut[level];
                break;
            }

            if(!keep) continue;

            /* Ordered dither: level rescaled to 0..64 vs the 0..63 matrix. */
            const int dith = ((int)level * 65) >> 8;
            if(dith > (int)bayer8[((y & 7) << 3) | (x & 7)]) {
                xbm_set(xbm, x, y);
            }
        }

        if(cfg->mode == RenderContour) {
            for(int x = 0; x < THERMAL_IMG_W; x++) {
                const int16_t c = level_curr[x];
                if(c == INT16_MIN) continue;
                bool edge = false;
                if(x > 0 && level_curr[x - 1] != INT16_MIN && c != level_curr[x - 1]) edge = true;
                if(y > 0 && level_prev[x] != INT16_MIN && c != level_prev[x]) edge = true;
                if(edge) xbm_set(xbm, x, y);
            }
            memcpy(level_prev, level_curr, sizeof(level_prev));
        }
    }
}
