/**
 * MLX90640 driver for Flipper Zero
 *
 * Calibration algorithm ported from the Melexis MLX90640 reference driver
 * (https://github.com/melexis/mlx90640-library), Apache License 2.0,
 * (C) 2017 Melexis N.V. Transport layer rewritten for furi_hal_i2c.
 *
 * Two Flipper-specific deviations from the reference driver:
 *  1. furi_hal_i2c_trx() cannot be used: it emits a STOP between the address
 *     write and the data read. The MLX90640 requires a repeated START, so we
 *     drive tx_ext/rx_ext with EndAwaitRestart / BeginRestart instead.
 *  2. The external I2C bus is hardcoded to 100 kHz by the firmware. We
 *     override I2C3->TIMINGR after acquiring the bus to reach 400 kHz / 1 MHz.
 */
#pragma once

#include <furi.h>
#include <furi_hal.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Device constants (from the Melexis reference header)
 * ------------------------------------------------------------------------ */

#define MLX90640_I2C_ADDR_7BIT 0x33
/* furi_hal passes the address straight to LL_I2C_HandleTransfer, which expects
 * it pre-shifted into 8-bit form. */
#define MLX90640_I2C_ADDR_8BIT (MLX90640_I2C_ADDR_7BIT << 1)

#define MLX90640_EEPROM_START_ADDRESS 0x2400
#define MLX90640_EEPROM_DUMP_NUM      832
#define MLX90640_PIXEL_DATA_START     0x0400
#define MLX90640_PIXEL_NUM            768
#define MLX90640_LINE_NUM             24
#define MLX90640_COLUMN_NUM           32
#define MLX90640_AUX_DATA_START       0x0700
#define MLX90640_AUX_NUM              64
#define MLX90640_STATUS_REG           0x8000
#define MLX90640_INIT_STATUS_VALUE    0x0030
#define MLX90640_CTRL_REG             0x800D

/* frameData layout: [0..767] pixels, [768..831] aux, [832] ctrl, [833] subpage */
#define MLX90640_FRAME_WORDS 834

#define MLX90640_SCALEALPHA 0.000001f

/* Rated object-temperature span of the part. Results are clamped to this so a
 * corrupted frame can never put NaN/Inf into the display or auto-range path. */
#define MLX90640_TEMP_MIN (-40.0f)
#define MLX90640_TEMP_MAX (300.0f)

/* ---------------------------------------------------------------------------
 * Bus speed
 * ------------------------------------------------------------------------ */

typedef enum {
    Mlx90640Speed100k = 0, /* firmware default, ~3 fps ceiling */
    Mlx90640Speed400k = 1, /* recommended, needs 2.2k-4.7k pull-ups */
    Mlx90640Speed1M = 2, /* FM+, experimental, short wires + strong pull-ups */
    Mlx90640SpeedCount,
} Mlx90640Speed;

/* Refresh rate codes written to bits [9:7] of the control register. */
typedef enum {
    Mlx90640Rate0_5Hz = 0,
    Mlx90640Rate1Hz = 1,
    Mlx90640Rate2Hz = 2,
    Mlx90640Rate4Hz = 3,
    Mlx90640Rate8Hz = 4,
    Mlx90640Rate16Hz = 5,
    Mlx90640Rate32Hz = 6,
    Mlx90640Rate64Hz = 7,
    Mlx90640RateCount,
} Mlx90640Rate;

/* ---------------------------------------------------------------------------
 * Calibration parameters (layout matches the Melexis reference struct)
 * ------------------------------------------------------------------------ */

typedef struct {
    int16_t kVdd;
    int16_t vdd25;
    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;
    int16_t gainEE;
    float tgc;
    float cpKv;
    float cpKta;
    uint8_t resolutionEE;
    uint8_t calibrationModeEE;
    float KsTa;
    float ksTo[5];
    int16_t ct[5];
    uint16_t alpha[768];
    uint8_t alphaScale;
    int16_t offset[768];
    int8_t kta[768];
    uint8_t ktaScale;
    int8_t kv[768];
    uint8_t kvScale;
    float cpAlpha[2];
    int16_t cpOffset[2];
    float ilChessC[3];
    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
} Mlx90640Params;

typedef enum {
    Mlx90640OkResult = 0,
    Mlx90640ErrNoDevice,
    Mlx90640ErrI2c,
    Mlx90640ErrEeprom,
    Mlx90640ErrFrame,
    Mlx90640ErrTimeout,
} Mlx90640Status;

typedef struct {
    Mlx90640Params* params; /* heap: ~4.7 kB */
    uint16_t* frame; /* heap: MLX90640_FRAME_WORDS words */
    bool bus_acquired;
    bool initialised;
    Mlx90640Speed speed;
} Mlx90640;

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------ */

/** Allocate driver state. Does not touch the bus. */
Mlx90640* mlx90640_alloc(void);

/** Free driver state. Releases the bus if still held. */
void mlx90640_free(Mlx90640* mlx);

/** Acquire the external I2C bus and apply the requested clock speed.
 *  Must be paired with mlx90640_bus_release(). */
void mlx90640_bus_acquire(Mlx90640* mlx, Mlx90640Speed speed);

/** Release the external I2C bus. Safe to call when not held. */
void mlx90640_bus_release(Mlx90640* mlx);

/** Change clock speed while the bus is held. */
void mlx90640_set_speed(Mlx90640* mlx, Mlx90640Speed speed);

/** Probe for the sensor. Bus must be held. */
bool mlx90640_probe(Mlx90640* mlx);

/** Dump EEPROM and extract calibration parameters. Bus must be held.
 *  Allocates a 3 kB scratch buffer internally and frees it before returning. */
Mlx90640Status mlx90640_init(Mlx90640* mlx);

/** Set refresh rate. Bus must be held. */
bool mlx90640_set_rate(Mlx90640* mlx, Mlx90640Rate rate);

/** Select chess (recommended) or interleaved readout pattern. */
bool mlx90640_set_chess_mode(Mlx90640* mlx, bool chess);

/** Read one subpage into the internal frame buffer. Bus must be held.
 *  Blocks until the data-ready flag is set or timeout_ms elapses. */
Mlx90640Status mlx90640_read_frame(Mlx90640* mlx, uint32_t timeout_ms);

/** Convert the internal frame buffer to 768 temperatures in degrees C.
 *  `emissivity` typically 0.95; `tr` is reflected temperature (Ta - 8 is a
 *  reasonable default). `out` must hold 768 floats. */
void mlx90640_calculate_to(Mlx90640* mlx, float emissivity, float tr, float* out);

/** Ambient (die) temperature of the last frame, in degrees C. */
float mlx90640_get_ta(Mlx90640* mlx);

/** Subpage index (0 or 1) of the last frame. */
int mlx90640_get_subpage(Mlx90640* mlx);

/** Replace broken/outlier pixels with the average of their valid neighbours. */
void mlx90640_fix_bad_pixels(Mlx90640* mlx, float* to);

#ifdef __cplusplus
}
#endif
