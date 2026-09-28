/**
 * Image export for the MLX90640 thermal camera app.
 *
 * PNG is written with "stored" (uncompressed) deflate blocks, which produces a
 * fully spec-compliant file without linking a compressor. Both writers stream
 * row by row, so even a 256x192 export only ever holds one row in RAM.
 */
#pragma once

#include <furi.h>
#include <storage/storage.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define THERMAL_SENSOR_W 32
#define THERMAL_SENSOR_H 24

/** False-colour maps applied on export. */
typedef enum {
    PaletteIronbow = 0,
    PaletteRainbow,
    PaletteGrayscale,
    PaletteHotMetal,
    PaletteArctic,
    PaletteCount,
} ThermalPalette;

extern const char* const thermal_palette_names[PaletteCount];

/** How the threshold setting masks pixels. */
typedef enum {
    ThresholdOff = 0,
    ThresholdAbove, /* show only pixels >= low */
    ThresholdBelow, /* show only pixels <= low */
    ThresholdBand, /* show only low <= t <= high */
    ThresholdCount,
} ThresholdMode;

extern const char* const thermal_threshold_names[ThresholdCount];

/** Everything the exporter needs to turn 768 temperatures into an image. */
typedef struct {
    const float* temps; /* 768 values, degrees C */
    float range_min; /* maps to palette index 0 */
    float range_max; /* maps to palette index 255 */
    ThermalPalette palette;
    ThresholdMode threshold_mode;
    float threshold_low;
    float threshold_high;
    uint8_t scale; /* 1, 2, 4 or 8 */
    bool bilinear;
    bool mirror;
} ThermalExportConfig;

/** Map a normalised 0..255 level through a palette. */
void thermal_palette_lookup(ThermalPalette palette, uint8_t level, uint8_t* rgb);

/**
 * Write an RGB PNG. Returns true on success.
 * `path` is a full path; the parent directory must exist.
 */
bool thermal_export_png(Storage* storage, const char* path, const ThermalExportConfig* cfg);

/** Write an uncompressed RGB TIFF. Returns true on success. */
bool thermal_export_tiff(Storage* storage, const char* path, const ThermalExportConfig* cfg);

/** Write the raw 32x24 temperatures as CSV, one row per sensor line. */
bool thermal_export_csv(Storage* storage, const char* path, const float* temps);

/** Append 768 float32 temperatures to an open file (raw animation capture). */
bool thermal_export_raw_append(File* file, const float* temps);

#ifdef __cplusplus
}
#endif
