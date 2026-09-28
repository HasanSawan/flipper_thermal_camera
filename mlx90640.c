/**
 * MLX90640 driver for Flipper Zero -- implementation.
 *
 * Calibration algorithm ported from the Melexis MLX90640 reference driver,
 * Apache License 2.0, (C) 2017 Melexis N.V.
 */

#include "mlx90640.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TAG "MLX90640"

/* I2C transaction timeouts (ms). A 1536-byte read at 100 kHz takes ~140 ms. */
#define MLX_TIMEOUT_SHORT 50
#define MLX_TIMEOUT_BULK  600

/* ---------------------------------------------------------------------------
 * Bus speed override
 *
 * furi_hal configures the external bus (I2C3) at 100 kHz unconditionally. We
 * rewrite TIMINGR directly after acquire; no firmware patch is needed. The
 * peripheral must be disabled (PE=0) while TIMINGR is changed.
 *
 * STM32WB55 I2C3 base 0x40005C00; CR1 at +0x00, TIMINGR at +0x10.
 * ------------------------------------------------------------------------ */

/* Overridable so host-side tests can point this at ordinary memory. */
#ifndef MLX_I2C3_BASE_ADDR
#define MLX_I2C3_BASE_ADDR 0x40005C00UL
#endif

#define I2C3_REG_CR1     (*(volatile uint32_t*)(MLX_I2C3_BASE_ADDR + 0x00UL))
#define I2C3_REG_TIMINGR (*(volatile uint32_t*)(MLX_I2C3_BASE_ADDR + 0x10UL))

/* TIMINGR values for I2CCLK = 64 MHz.
 * 100k / 400k are the values the firmware itself uses (CubeMX generated).
 * 1M is computed: PRESC=0, SCLDEL=3, SDADEL=0, SCLH=0x19, SCLL=0x24
 *   -> tSCL = (0x24+1 + 0x19+1) * 15.625 ns = 984 ns ~= 1.016 MHz */
static const uint32_t mlx_timings[Mlx90640SpeedCount] = {
    [Mlx90640Speed100k] = 0x10707DBCUL,
    [Mlx90640Speed400k] = 0x00602173UL,
    [Mlx90640Speed1M] = 0x00301924UL,
};

static void mlx_apply_speed(Mlx90640Speed speed) {
    if(speed >= Mlx90640SpeedCount) speed = Mlx90640Speed400k;

    uint32_t cr1 = I2C3_REG_CR1;
    I2C3_REG_CR1 = cr1 & ~1UL; /* PE = 0 */
    I2C3_REG_TIMINGR = mlx_timings[speed];
    I2C3_REG_CR1 = cr1 | 1UL; /* PE = 1 */
}

/* ---------------------------------------------------------------------------
 * Transport
 * ------------------------------------------------------------------------ */

/**
 * Read `count` 16-bit words starting at `start_addr` into `out`.
 *
 * The MLX90640 needs: START, addr+W, reg_hi, reg_lo, REPEATED START, addr+R,
 * data..., STOP. furi_hal_i2c_trx() would emit a STOP instead of the repeated
 * START, so the two halves are issued explicitly.
 *
 * Data is read as big-endian bytes directly into `out` and byte-swapped in
 * place, so no bounce buffer is needed for the 1536-byte pixel read.
 */
static bool mlx_read_words(uint16_t start_addr, uint16_t count, uint16_t* out) {
    furi_assert(out);
    if(count == 0) return true;

    const uint8_t cmd[2] = {(uint8_t)(start_addr >> 8), (uint8_t)(start_addr & 0xFF)};
    const uint32_t timeout = (count > 64) ? MLX_TIMEOUT_BULK : MLX_TIMEOUT_SHORT;

    if(!furi_hal_i2c_tx_ext(
           &furi_hal_i2c_handle_external,
           MLX90640_I2C_ADDR_8BIT,
           false,
           cmd,
           sizeof(cmd),
           FuriHalI2cBeginStart,
           FuriHalI2cEndAwaitRestart, /* hold the bus, no STOP */
           timeout)) {
        return false;
    }

    if(!furi_hal_i2c_rx_ext(
           &furi_hal_i2c_handle_external,
           MLX90640_I2C_ADDR_8BIT,
           false,
           (uint8_t*)out,
           (size_t)count * 2u,
           FuriHalI2cBeginRestart, /* repeated START */
           FuriHalI2cEndStop,
           timeout)) {
        return false;
    }

    /* Big-endian on the wire -> host order. */
    uint8_t* raw = (uint8_t*)out;
    for(uint16_t i = 0; i < count; i++) {
        out[i] = (uint16_t)((uint16_t)raw[i * 2] << 8) | (uint16_t)raw[i * 2 + 1];
    }
    return true;
}

static bool mlx_write_word(uint16_t reg_addr, uint16_t value) {
    const uint8_t cmd[4] = {
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };
    return furi_hal_i2c_tx(
        &furi_hal_i2c_handle_external, MLX90640_I2C_ADDR_8BIT, cmd, sizeof(cmd), MLX_TIMEOUT_SHORT);
}

/* ---------------------------------------------------------------------------
 * Small helpers
 * ------------------------------------------------------------------------ */

static inline float pow2f_i(int e) {
    return ldexpf(1.0f, e);
}

/* Sign-extend an n-bit field held in a wider integer. */
static inline int sign_extend(int value, int bits) {
    const int half = 1 << (bits - 1);
    return (value >= half) ? (value - (1 << bits)) : value;
}

/* ---------------------------------------------------------------------------
 * EEPROM parameter extraction
 *
 * Word indices and bit fields follow the Melexis reference driver exactly.
 * eeData[] is the 832-word dump starting at 0x2400.
 * ------------------------------------------------------------------------ */

static void extract_vdd(const uint16_t* ee, Mlx90640Params* p) {
    int16_t kVdd = (int16_t)(int8_t)((ee[51] & 0xFF00) >> 8);
    kVdd = (int16_t)(kVdd * 32);

    int16_t vdd25 = (int16_t)(ee[51] & 0x00FF);
    /* Multiply rather than shift: the reference driver shifts a negative
     * value here, which is undefined behaviour in C. */
    vdd25 = (int16_t)((vdd25 - 256) * 32 - 8192);

    p->kVdd = kVdd;
    p->vdd25 = vdd25;
}

static void extract_ptat(const uint16_t* ee, Mlx90640Params* p) {
    float KvPTAT = (float)sign_extend((ee[50] & 0xFC00) >> 10, 6) / 4096.0f;
    float KtPTAT = (float)sign_extend(ee[50] & 0x03FF, 10) / 8.0f;

    p->KvPTAT = KvPTAT;
    p->KtPTAT = KtPTAT;
    p->vPTAT25 = ee[49];
    /* Note: masked but deliberately NOT shifted before the divide -- this
     * matches the reference, which computes (ee[16] & 0xF000) / 2^14. */
    p->alphaPTAT = (float)(ee[16] & 0xF000) / 16384.0f + 8.0f;
}

static void extract_gain(const uint16_t* ee, Mlx90640Params* p) {
    p->gainEE = (int16_t)ee[48];
}

static void extract_tgc(const uint16_t* ee, Mlx90640Params* p) {
    p->tgc = (float)(int8_t)(ee[60] & 0x00FF) / 32.0f;
}

static void extract_resolution(const uint16_t* ee, Mlx90640Params* p) {
    p->resolutionEE = (uint8_t)((ee[56] & 0x3000) >> 12);
}

static void extract_ksta(const uint16_t* ee, Mlx90640Params* p) {
    p->KsTa = (float)(int8_t)((ee[60] & 0xFF00) >> 8) / 8192.0f;
}

static void extract_ksto(const uint16_t* ee, Mlx90640Params* p) {
    const int step = ((ee[63] & 0x3000) >> 12) * 10;

    p->ct[0] = -40;
    p->ct[1] = 0;
    p->ct[2] = (int16_t)((ee[63] & 0x00F0) >> 4); /* nibble 2 */
    p->ct[3] = (int16_t)((ee[63] & 0x0F00) >> 8); /* nibble 3 */
    p->ct[2] = (int16_t)(p->ct[2] * step);
    p->ct[3] = (int16_t)(p->ct[2] + p->ct[3] * step);
    p->ct[4] = 400;

    const int ks_to_scale_exp = (ee[63] & 0x000F) + 8; /* nibble 1 + 8 */
    const float ks_to_scale = pow2f_i(ks_to_scale_exp);

    p->ksTo[0] = (float)(int8_t)(ee[61] & 0x00FF) / ks_to_scale;
    p->ksTo[1] = (float)(int8_t)((ee[61] & 0xFF00) >> 8) / ks_to_scale;
    p->ksTo[2] = (float)(int8_t)(ee[62] & 0x00FF) / ks_to_scale;
    p->ksTo[3] = (float)(int8_t)((ee[62] & 0xFF00) >> 8) / ks_to_scale;
    p->ksTo[4] = -0.0002f;
}

/* Must run before extract_alpha(): alpha subtracts tgc * mean(cpAlpha). */
static void extract_cp(const uint16_t* ee, Mlx90640Params* p) {
    const int alpha_scale_exp = ((ee[32] & 0xF000) >> 12) + 27;

    float alphaSP[2];
    int16_t offsetSP[2];

    int a0 = sign_extend(ee[57] & 0x03FF, 10);
    int a1 = sign_extend((ee[57] & 0xFC00) >> 10, 6);
    alphaSP[0] = (float)a0 / pow2f_i(alpha_scale_exp);
    alphaSP[1] = (1.0f + (float)a1 / 128.0f) * alphaSP[0];

    int o0 = sign_extend(ee[58] & 0x03FF, 10);
    int o1 = sign_extend((ee[58] & 0xFC00) >> 10, 6);
    offsetSP[0] = (int16_t)o0;
    offsetSP[1] = (int16_t)(o1 + o0);

    const int kta_scale1_exp = ((ee[56] & 0x00F0) >> 4) + 8;
    const int kv_scale_exp = (ee[56] & 0x0F00) >> 8;

    p->cpAlpha[0] = alphaSP[0];
    p->cpAlpha[1] = alphaSP[1];
    p->cpOffset[0] = offsetSP[0];
    p->cpOffset[1] = offsetSP[1];
    p->cpKta = (float)(int8_t)(ee[59] & 0x00FF) / pow2f_i(kta_scale1_exp);
    p->cpKv = (float)(int8_t)((ee[59] & 0xFF00) >> 8) / pow2f_i(kv_scale_exp);
}

static void extract_alpha(const uint16_t* ee, Mlx90640Params* p, float* scratch) {
    int acc_row[24];
    int acc_col[32];

    const int acc_rem_scale = ee[32] & 0x000F; /* nibble 1 */
    const int acc_col_scale = (ee[32] & 0x00F0) >> 4; /* nibble 2 */
    const int acc_row_scale = (ee[32] & 0x0F00) >> 8; /* nibble 3 */
    int alpha_scale = ((ee[32] & 0xF000) >> 12) + 30; /* nibble 4 + 30 */
    const int alpha_ref = ee[33];

    for(int i = 0; i < 6; i++) {
        const int p_idx = i * 4;
        acc_row[p_idx + 0] = sign_extend(ee[34 + i] & 0x000F, 4);
        acc_row[p_idx + 1] = sign_extend((ee[34 + i] & 0x00F0) >> 4, 4);
        acc_row[p_idx + 2] = sign_extend((ee[34 + i] & 0x0F00) >> 8, 4);
        acc_row[p_idx + 3] = sign_extend((ee[34 + i] & 0xF000) >> 12, 4);
    }

    for(int i = 0; i < 8; i++) {
        const int p_idx = i * 4;
        acc_col[p_idx + 0] = sign_extend(ee[40 + i] & 0x000F, 4);
        acc_col[p_idx + 1] = sign_extend((ee[40 + i] & 0x00F0) >> 4, 4);
        acc_col[p_idx + 2] = sign_extend((ee[40 + i] & 0x0F00) >> 8, 4);
        acc_col[p_idx + 3] = sign_extend((ee[40 + i] & 0xF000) >> 12, 4);
    }

    const float cp_mean = p->tgc * (p->cpAlpha[0] + p->cpAlpha[1]) / 2.0f;

    for(int i = 0; i < 24; i++) {
        for(int j = 0; j < 32; j++) {
            const int idx = 32 * i + j;
            float v = (float)sign_extend((ee[64 + idx] & 0x03F0) >> 4, 6);
            v *= (float)(1 << acc_rem_scale);
            /* acc_row/acc_col are signed: scale by multiplication, since
             * shifting a negative value left is undefined behaviour. */
            v += (float)(alpha_ref + acc_row[i] * (1 << acc_row_scale) +
                         acc_col[j] * (1 << acc_col_scale));
            v /= pow2f_i(alpha_scale);
            v -= cp_mean;
            scratch[idx] = MLX90640_SCALEALPHA / v;
        }
    }

    /* Renormalise into uint16 with a shared exponent. */
    float max_v = scratch[0];
    for(int i = 1; i < 768; i++) {
        if(scratch[i] > max_v) max_v = scratch[i];
    }

    alpha_scale = 0;
    while(max_v < 32768.0f && alpha_scale < 63) {
        max_v *= 2.0f;
        alpha_scale++;
    }

    const float sc = pow2f_i(alpha_scale);
    for(int i = 0; i < 768; i++) {
        float t = scratch[i] * sc;
        if(t < 0.0f) t = 0.0f;
        if(t > 65535.0f) t = 65535.0f;
        p->alpha[i] = (uint16_t)(t + 0.5f);
    }
    p->alphaScale = (uint8_t)alpha_scale;
}

static void extract_offset(const uint16_t* ee, Mlx90640Params* p) {
    int occ_row[24];
    int occ_col[32];

    const int occ_rem_scale = ee[16] & 0x000F;
    const int occ_col_scale = (ee[16] & 0x00F0) >> 4;
    const int occ_row_scale = (ee[16] & 0x0F00) >> 8;
    const int16_t offset_ref = (int16_t)ee[17];

    for(int i = 0; i < 6; i++) {
        const int p_idx = i * 4;
        occ_row[p_idx + 0] = sign_extend(ee[18 + i] & 0x000F, 4);
        occ_row[p_idx + 1] = sign_extend((ee[18 + i] & 0x00F0) >> 4, 4);
        occ_row[p_idx + 2] = sign_extend((ee[18 + i] & 0x0F00) >> 8, 4);
        occ_row[p_idx + 3] = sign_extend((ee[18 + i] & 0xF000) >> 12, 4);
    }

    for(int i = 0; i < 8; i++) {
        const int p_idx = i * 4;
        occ_col[p_idx + 0] = sign_extend(ee[24 + i] & 0x000F, 4);
        occ_col[p_idx + 1] = sign_extend((ee[24 + i] & 0x00F0) >> 4, 4);
        occ_col[p_idx + 2] = sign_extend((ee[24 + i] & 0x0F00) >> 8, 4);
        occ_col[p_idx + 3] = sign_extend((ee[24 + i] & 0xF000) >> 12, 4);
    }

    for(int i = 0; i < 24; i++) {
        for(int j = 0; j < 32; j++) {
            const int idx = 32 * i + j;
            int v = sign_extend((ee[64 + idx] & 0xFC00) >> 10, 6);
            v *= (1 << occ_rem_scale);
            /* Signed operands: multiply instead of shifting left. */
            v +=
                offset_ref + occ_row[i] * (1 << occ_row_scale) + occ_col[j] * (1 << occ_col_scale);
            p->offset[idx] = (int16_t)v;
        }
    }
}

static void extract_kta(const uint16_t* ee, Mlx90640Params* p, float* scratch) {
    int8_t kta_rc[4];

    kta_rc[0] = (int8_t)((ee[54] & 0xFF00) >> 8); /* KtaRoCo */
    kta_rc[2] = (int8_t)(ee[54] & 0x00FF); /* KtaReCo */
    kta_rc[1] = (int8_t)((ee[55] & 0xFF00) >> 8); /* KtaRoCe */
    kta_rc[3] = (int8_t)(ee[55] & 0x00FF); /* KtaReCe */

    const int kta_scale1 = ((ee[56] & 0x00F0) >> 4) + 8;
    const int kta_scale2 = ee[56] & 0x000F;

    for(int i = 0; i < 24; i++) {
        for(int j = 0; j < 32; j++) {
            const int idx = 32 * i + j;
            const int split = 2 * (idx / 32 - (idx / 64) * 2) + idx % 2;

            float v = (float)sign_extend((ee[64 + idx] & 0x000E) >> 1, 3);
            v *= (float)(1 << kta_scale2);
            v += (float)kta_rc[split];
            v /= pow2f_i(kta_scale1);
            scratch[idx] = v;
        }
    }

    float max_v = fabsf(scratch[0]);
    for(int i = 1; i < 768; i++) {
        const float a = fabsf(scratch[i]);
        if(a > max_v) max_v = a;
    }

    int scale = 0;
    while(max_v < 63.4f && scale < 63) {
        max_v *= 2.0f;
        scale++;
    }

    const float sc = pow2f_i(scale);
    for(int i = 0; i < 768; i++) {
        float t = scratch[i] * sc;
        t = (t < 0.0f) ? (t - 0.5f) : (t + 0.5f);
        if(t < -128.0f) t = -128.0f;
        if(t > 127.0f) t = 127.0f;
        p->kta[i] = (int8_t)t;
    }
    p->ktaScale = (uint8_t)scale;
}

static void extract_kv(const uint16_t* ee, Mlx90640Params* p, float* scratch) {
    int kv_t[4];

    kv_t[0] = sign_extend((ee[52] & 0xF000) >> 12, 4); /* KvRoCo */
    kv_t[2] = sign_extend((ee[52] & 0x0F00) >> 8, 4); /* KvReCo */
    kv_t[1] = sign_extend((ee[52] & 0x00F0) >> 4, 4); /* KvRoCe */
    kv_t[3] = sign_extend(ee[52] & 0x000F, 4); /* KvReCe */

    const int kv_scale = (ee[56] & 0x0F00) >> 8;

    for(int i = 0; i < 24; i++) {
        for(int j = 0; j < 32; j++) {
            const int idx = 32 * i + j;
            const int split = 2 * (idx / 32 - (idx / 64) * 2) + idx % 2;
            scratch[idx] = (float)kv_t[split] / pow2f_i(kv_scale);
        }
    }

    float max_v = fabsf(scratch[0]);
    for(int i = 1; i < 768; i++) {
        const float a = fabsf(scratch[i]);
        if(a > max_v) max_v = a;
    }

    int scale = 0;
    while(max_v < 63.4f && scale < 63) {
        max_v *= 2.0f;
        scale++;
    }

    const float sc = pow2f_i(scale);
    for(int i = 0; i < 768; i++) {
        float t = scratch[i] * sc;
        t = (t < 0.0f) ? (t - 0.5f) : (t + 0.5f);
        if(t < -128.0f) t = -128.0f;
        if(t > 127.0f) t = 127.0f;
        p->kv[i] = (int8_t)t;
    }
    p->kvScale = (uint8_t)scale;
}

static void extract_cilc(const uint16_t* ee, Mlx90640Params* p) {
    uint8_t mode = (uint8_t)((ee[10] & 0x0800) >> 4);
    p->calibrationModeEE = mode ^ 0x80;

    p->ilChessC[0] = (float)sign_extend(ee[53] & 0x003F, 6) / 16.0f;
    p->ilChessC[1] = (float)sign_extend((ee[53] & 0x07C0) >> 6, 5) / 2.0f;
    p->ilChessC[2] = (float)sign_extend((ee[53] & 0xF800) >> 11, 5) / 8.0f;
}

static Mlx90640Status extract_deviating(const uint16_t* ee, Mlx90640Params* p) {
    uint16_t broken = 0;
    uint16_t outlier = 0;

    for(int i = 0; i < 5; i++) {
        p->brokenPixels[i] = 0xFFFF;
        p->outlierPixels[i] = 0xFFFF;
    }

    for(uint16_t idx = 0; idx < 768; idx++) {
        if(ee[64 + idx] == 0) {
            if(broken < 5) p->brokenPixels[broken] = idx;
            broken++;
        } else if((ee[64 + idx] & 0x0001) != 0) {
            if(outlier < 5) p->outlierPixels[outlier] = idx;
            outlier++;
        }
    }

    /* More than 4 of either kind means the sensor is not usable. */
    if(broken > 4 || outlier > 4 || (broken + outlier) > 4) {
        return Mlx90640ErrEeprom;
    }
    return Mlx90640OkResult;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */

Mlx90640* mlx90640_alloc(void) {
    Mlx90640* mlx = malloc(sizeof(Mlx90640));
    furi_check(mlx);
    memset(mlx, 0, sizeof(Mlx90640));

    mlx->params = malloc(sizeof(Mlx90640Params));
    furi_check(mlx->params);
    memset(mlx->params, 0, sizeof(Mlx90640Params));

    mlx->frame = malloc(sizeof(uint16_t) * MLX90640_FRAME_WORDS);
    furi_check(mlx->frame);
    memset(mlx->frame, 0, sizeof(uint16_t) * MLX90640_FRAME_WORDS);

    mlx->speed = Mlx90640Speed400k;
    return mlx;
}

void mlx90640_free(Mlx90640* mlx) {
    if(!mlx) return;
    mlx90640_bus_release(mlx);
    if(mlx->params) free(mlx->params);
    if(mlx->frame) free(mlx->frame);
    free(mlx);
}

void mlx90640_bus_acquire(Mlx90640* mlx, Mlx90640Speed speed) {
    furi_assert(mlx);
    if(mlx->bus_acquired) return;

    furi_hal_i2c_acquire(&furi_hal_i2c_handle_external);
    mlx->bus_acquired = true;
    mlx->speed = speed;
    mlx_apply_speed(speed);
}

void mlx90640_bus_release(Mlx90640* mlx) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return;

    furi_hal_i2c_release(&furi_hal_i2c_handle_external);
    mlx->bus_acquired = false;
}

void mlx90640_set_speed(Mlx90640* mlx, Mlx90640Speed speed) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) {
        mlx->speed = speed;
        return;
    }
    mlx->speed = speed;
    mlx_apply_speed(speed);
}

bool mlx90640_probe(Mlx90640* mlx) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return false;
    return furi_hal_i2c_is_device_ready(
        &furi_hal_i2c_handle_external, MLX90640_I2C_ADDR_8BIT, MLX_TIMEOUT_SHORT);
}

Mlx90640Status mlx90640_init(Mlx90640* mlx) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return Mlx90640ErrNoDevice;

    if(!mlx90640_probe(mlx)) {
        FURI_LOG_E(TAG, "no device at 0x%02X", MLX90640_I2C_ADDR_7BIT);
        return Mlx90640ErrNoDevice;
    }

    uint16_t* ee = malloc(sizeof(uint16_t) * MLX90640_EEPROM_DUMP_NUM);
    if(!ee) return Mlx90640ErrEeprom;

    /* Read the dump in chunks so a single transaction never runs too long. */
    Mlx90640Status status = Mlx90640OkResult;
    const uint16_t chunk = 128;
    for(uint16_t off = 0; off < MLX90640_EEPROM_DUMP_NUM; off += chunk) {
        uint16_t n = MLX90640_EEPROM_DUMP_NUM - off;
        if(n > chunk) n = chunk;
        if(!mlx_read_words(MLX90640_EEPROM_START_ADDRESS + off, n, ee + off)) {
            FURI_LOG_E(TAG, "eeprom read failed at +%u", off);
            status = Mlx90640ErrI2c;
            break;
        }
    }

    if(status != Mlx90640OkResult) {
        free(ee);
        return status;
    }

    float* scratch = malloc(sizeof(float) * MLX90640_PIXEL_NUM);
    if(!scratch) {
        free(ee);
        return Mlx90640ErrEeprom;
    }

    Mlx90640Params* p = mlx->params;

    extract_vdd(ee, p);
    extract_ptat(ee, p);
    extract_gain(ee, p);
    extract_tgc(ee, p);
    extract_resolution(ee, p);
    extract_ksta(ee, p);
    extract_ksto(ee, p);
    extract_cp(ee, p); /* before alpha */
    extract_alpha(ee, p, scratch);
    extract_offset(ee, p);
    extract_kta(ee, p, scratch);
    extract_kv(ee, p, scratch);
    extract_cilc(ee, p);
    status = extract_deviating(ee, p);

    free(scratch);
    free(ee);

    if(status != Mlx90640OkResult) {
        FURI_LOG_E(TAG, "too many bad pixels");
        return status;
    }

    mlx->initialised = true;
    FURI_LOG_I(TAG, "init ok, alphaScale=%u ktaScale=%u", p->alphaScale, p->ktaScale);
    return Mlx90640OkResult;
}

bool mlx90640_set_rate(Mlx90640* mlx, Mlx90640Rate rate) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return false;
    if(rate >= Mlx90640RateCount) return false;

    uint16_t ctrl = 0;
    if(!mlx_read_words(MLX90640_CTRL_REG, 1, &ctrl)) return false;

    ctrl = (uint16_t)((ctrl & 0xFC7F) | (uint16_t)((rate & 0x07) << 7));
    return mlx_write_word(MLX90640_CTRL_REG, ctrl);
}

bool mlx90640_set_chess_mode(Mlx90640* mlx, bool chess) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return false;

    uint16_t ctrl = 0;
    if(!mlx_read_words(MLX90640_CTRL_REG, 1, &ctrl)) return false;

    ctrl = chess ? (uint16_t)(ctrl | 0x1000) : (uint16_t)(ctrl & ~0x1000);
    return mlx_write_word(MLX90640_CTRL_REG, ctrl);
}

Mlx90640Status mlx90640_read_frame(Mlx90640* mlx, uint32_t timeout_ms) {
    furi_assert(mlx);
    if(!mlx->bus_acquired) return Mlx90640ErrNoDevice;

    uint16_t* fd = mlx->frame;
    uint16_t status_reg = 0;

    /* Wait for the data-ready flag, yielding so the GUI thread keeps running. */
    const uint32_t deadline = furi_get_tick() + furi_ms_to_ticks(timeout_ms);
    bool ready = false;
    while(furi_get_tick() < deadline) {
        if(!mlx_read_words(MLX90640_STATUS_REG, 1, &status_reg)) return Mlx90640ErrI2c;
        if(status_reg & 0x0008) {
            ready = true;
            break;
        }
        furi_delay_ms(1);
    }
    if(!ready) return Mlx90640ErrTimeout;

    /* Clear data-ready so the sensor can start filling the next subpage. */
    if(!mlx_write_word(MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE)) return Mlx90640ErrI2c;

    if(!mlx_read_words(MLX90640_PIXEL_DATA_START, MLX90640_PIXEL_NUM, &fd[0])) {
        return Mlx90640ErrI2c;
    }
    if(!mlx_read_words(MLX90640_AUX_DATA_START, MLX90640_AUX_NUM, &fd[768])) {
        return Mlx90640ErrI2c;
    }

    uint16_t ctrl = 0;
    if(!mlx_read_words(MLX90640_CTRL_REG, 1, &ctrl)) return Mlx90640ErrI2c;

    fd[832] = ctrl;
    fd[833] = status_reg & 0x0001; /* subpage */

    /* Gain word of exactly zero would divide by zero downstream. */
    if((int16_t)fd[778] == 0) return Mlx90640ErrFrame;

    return Mlx90640OkResult;
}

/*
 * Vdd and Ta feed every other calculation and are also shown directly, so both
 * guard their divisors and their results. A partial EEPROM read -- entirely
 * possible with marginal pull-ups at 400 kHz -- can leave kVdd or KtPTAT at
 * zero, and an Inf escaping from here would propagate into every pixel.
 */
#define MLX_VDD_NOMINAL 3.3f
#define MLX_TA_FALLBACK 25.0f

static float mlx_get_vdd(const uint16_t* fd, const Mlx90640Params* p) {
    if(p->kVdd == 0) return MLX_VDD_NOMINAL;

    float vdd = (float)(int16_t)fd[810];
    const int resolution_ram = (fd[832] & 0x0C00) >> 10;
    const float correction = pow2f_i(p->resolutionEE) / pow2f_i(resolution_ram);

    vdd = (correction * vdd - (float)p->vdd25) / (float)p->kVdd + MLX_VDD_NOMINAL;

    if(!isfinite(vdd)) return MLX_VDD_NOMINAL;
    return vdd;
}

static float mlx_get_ta(const uint16_t* fd, const Mlx90640Params* p) {
    if(p->KtPTAT == 0.0f) return MLX_TA_FALLBACK;

    const float vdd = mlx_get_vdd(fd, p);

    const float ptat = (float)(int16_t)fd[800];
    float ptat_art = (float)(int16_t)fd[768];

    const float denom = ptat * p->alphaPTAT + ptat_art;
    if(denom == 0.0f) return MLX_TA_FALLBACK;

    ptat_art = (ptat / denom) * pow2f_i(18);

    const float kv_term = 1.0f + p->KvPTAT * (vdd - MLX_VDD_NOMINAL);
    if(kv_term == 0.0f) return MLX_TA_FALLBACK;

    float ta = ptat_art / kv_term - (float)p->vPTAT25;
    ta = ta / p->KtPTAT + 25.0f;

    if(!isfinite(ta)) return MLX_TA_FALLBACK;
    return ta;
}

float mlx90640_get_ta(Mlx90640* mlx) {
    furi_assert(mlx);
    return mlx_get_ta(mlx->frame, mlx->params);
}

int mlx90640_get_subpage(Mlx90640* mlx) {
    furi_assert(mlx);
    return (int)mlx->frame[833];
}

void mlx90640_calculate_to(Mlx90640* mlx, float emissivity, float tr, float* out) {
    furi_assert(mlx && out);
    const Mlx90640Params* p = mlx->params;
    const uint16_t* fd = mlx->frame;

    if(emissivity < 0.05f) emissivity = 0.05f;

    const uint16_t sub_page = fd[833];
    const float vdd = mlx_get_vdd(fd, p);
    const float ta = mlx_get_ta(fd, p);

    float ta4 = ta + 273.15f;
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;

    float tr4 = tr + 273.15f;
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;

    const float ta_tr = tr4 - (tr4 - ta4) / emissivity;

    const float kta_scale = pow2f_i(p->ktaScale);
    const float kv_scale = pow2f_i(p->kvScale);
    const float alpha_scale = pow2f_i(p->alphaScale);

    float alpha_corr_r[4];
    alpha_corr_r[0] = 1.0f / (1.0f + p->ksTo[0] * 40.0f);
    alpha_corr_r[1] = 1.0f;
    alpha_corr_r[2] = 1.0f + p->ksTo[1] * (float)p->ct[2];
    alpha_corr_r[3] = alpha_corr_r[2] * (1.0f + p->ksTo[2] * (float)(p->ct[3] - p->ct[2]));

    const float gain = (float)p->gainEE / (float)(int16_t)fd[778];

    const uint8_t mode = (uint8_t)((fd[832] & 0x1000) >> 5);

    const float ta_corr = 1.0f + p->cpKta * (ta - 25.0f);
    const float vdd_corr = 1.0f + p->cpKv * (vdd - 3.3f);

    float ir_data_cp[2];
    ir_data_cp[0] = (float)(int16_t)fd[776] * gain;
    ir_data_cp[1] = (float)(int16_t)fd[808] * gain;

    ir_data_cp[0] -= (float)p->cpOffset[0] * ta_corr * vdd_corr;
    if(mode == p->calibrationModeEE) {
        ir_data_cp[1] -= (float)p->cpOffset[1] * ta_corr * vdd_corr;
    } else {
        ir_data_cp[1] -= ((float)p->cpOffset[1] + p->ilChessC[0]) * ta_corr * vdd_corr;
    }

    for(int idx = 0; idx < MLX90640_PIXEL_NUM; idx++) {
        const int il_pattern = idx / 32 - (idx / 64) * 2;
        const int chess_pattern = il_pattern ^ (idx - (idx / 2) * 2);
        const int conversion_pattern =
            ((idx + 2) / 4 - (idx + 3) / 4 + (idx + 1) / 4 - idx / 4) * (1 - 2 * il_pattern);

        const int pattern = (mode == 0) ? il_pattern : chess_pattern;

        /* Each frame carries only one subpage; leave the other half untouched
         * so the previous values persist (this is what makes chess mode look
         * continuous rather than flickering). */
        if(pattern != (int)sub_page) continue;

        float ir_data = (float)(int16_t)fd[idx] * gain;

        const float kta = (float)p->kta[idx] / kta_scale;
        const float kv = (float)p->kv[idx] / kv_scale;

        ir_data -=
            (float)p->offset[idx] * (1.0f + kta * (ta - 25.0f)) * (1.0f + kv * (vdd - 3.3f));

        if(mode != p->calibrationModeEE) {
            ir_data += p->ilChessC[2] * (float)(2 * il_pattern - 1) -
                       p->ilChessC[1] * (float)conversion_pattern;
        }

        ir_data -= p->tgc * ir_data_cp[sub_page];
        ir_data /= emissivity;

        if(p->alpha[idx] == 0) {
            out[idx] = ta;
            continue;
        }

        float alpha_comp = MLX90640_SCALEALPHA * alpha_scale / (float)p->alpha[idx];
        alpha_comp *= (1.0f + p->KsTa * (ta - 25.0f));

        float sx = alpha_comp * alpha_comp * alpha_comp * (ir_data + alpha_comp * ta_tr);
        sx = sqrtf(sqrtf(fabsf(sx))) * p->ksTo[1];

        const float denom0 = alpha_comp * (1.0f - p->ksTo[1] * 273.15f) + sx;
        float to = (denom0 == 0.0f) ? ta : sqrtf(sqrtf(fabsf(ir_data / denom0 + ta_tr))) - 273.15f;

        /* The first pass only picks the correction band, but a non-finite
         * value here would make every comparison below false and select the
         * wrong band, so fall back to ambient. */
        if(!isfinite(to)) to = ta;

        int range;
        if(to < (float)p->ct[1]) {
            range = 0;
        } else if(to < (float)p->ct[2]) {
            range = 1;
        } else if(to < (float)p->ct[3]) {
            range = 2;
        } else if(to < (float)p->ct[4]) {
            range = 3;
        } else {
            range = 4;
        }

        /* alpha_corr_r only has 4 entries; range 4 reuses the top correction,
         * matching the reference driver's indexing of ksTo[4] with ct[4]. */
        const float corr = alpha_corr_r[(range > 3) ? 3 : range];
        const float denom1 =
            alpha_comp * corr * (1.0f + p->ksTo[range] * (to - (float)p->ct[range]));

        float result = (denom1 == 0.0f) ? to :
                                          sqrtf(sqrtf(fabsf(ir_data / denom1 + ta_tr))) - 273.15f;

        /*
         * A corrupted read or a garbled EEPROM can drive this to NaN or Inf.
         * Letting that reach the caller would poison the frame min/max and
         * the auto-range, so clamp to the part's rated span instead.
         */
        if(!isfinite(result)) result = ta;
        if(result < MLX90640_TEMP_MIN) result = MLX90640_TEMP_MIN;
        if(result > MLX90640_TEMP_MAX) result = MLX90640_TEMP_MAX;

        out[idx] = result;
    }
}

void mlx90640_fix_bad_pixels(Mlx90640* mlx, float* to) {
    furi_assert(mlx && to);
    const Mlx90640Params* p = mlx->params;

    for(int pass = 0; pass < 2; pass++) {
        const uint16_t* list = (pass == 0) ? p->brokenPixels : p->outlierPixels;

        for(int i = 0; i < 5; i++) {
            const uint16_t idx = list[i];
            if(idx == 0xFFFF) continue;

            const int row = idx / 32;
            const int col = idx % 32;

            float sum = 0.0f;
            int n = 0;

            /* Average the horizontal neighbours; fall back to vertical. */
            if(col > 0) {
                sum += to[idx - 1];
                n++;
            }
            if(col < 31) {
                sum += to[idx + 1];
                n++;
            }
            if(n == 0) {
                if(row > 0) {
                    sum += to[idx - 32];
                    n++;
                }
                if(row < 23) {
                    sum += to[idx + 32];
                    n++;
                }
            }

            if(n > 0) to[idx] = sum / (float)n;
        }
    }
}
