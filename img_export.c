#include "img_export.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TAG "ThermalExport"

#define MAX_EXPORT_W (THERMAL_SENSOR_W * 8) /* 256 */
#define PNG_ROW_MAX  (1 + MAX_EXPORT_W * 3) /* filter byte + RGB */

const char* const thermal_palette_names[PaletteCount] = {
    "Ironbow",
    "Rainbow",
    "Gray",
    "HotMetal",
    "Arctic",
};

const char* const thermal_threshold_names[ThresholdCount] = {
    "Off",
    "Above",
    "Below",
    "Band",
};

/* ---------------------------------------------------------------------------
 * Palettes
 * ------------------------------------------------------------------------ */

typedef struct {
    uint8_t pos; /* 0..255 */
    uint8_t r, g, b;
} GradientStop;

static const GradientStop grad_ironbow[] = {
    {0, 0, 0, 0},
    {51, 40, 0, 90},
    {102, 130, 0, 120},
    {153, 220, 60, 40},
    {204, 255, 170, 0},
    {255, 255, 255, 255},
};

static const GradientStop grad_rainbow[] = {
    {0, 0, 0, 131},
    {32, 0, 60, 170},
    {96, 5, 255, 255},
    {160, 255, 255, 0},
    {223, 250, 0, 0},
    {255, 128, 0, 0},
};

static const GradientStop grad_hotmetal[] = {
    {0, 0, 0, 0},
    {84, 180, 0, 0},
    {168, 255, 180, 0},
    {255, 255, 255, 255},
};

static const GradientStop grad_arctic[] = {
    {0, 0, 0, 40},
    {64, 0, 110, 200},
    {128, 120, 220, 255},
    {192, 255, 240, 180},
    {255, 255, 255, 255},
};

static void gradient_eval(const GradientStop* stops, size_t n, uint8_t level, uint8_t* rgb) {
    if(level <= stops[0].pos) {
        rgb[0] = stops[0].r;
        rgb[1] = stops[0].g;
        rgb[2] = stops[0].b;
        return;
    }
    for(size_t i = 1; i < n; i++) {
        if(level <= stops[i].pos) {
            const GradientStop* a = &stops[i - 1];
            const GradientStop* b = &stops[i];
            const int span = (int)b->pos - (int)a->pos;
            const int t = (span > 0) ? (((int)level - (int)a->pos) * 255 / span) : 0;

            rgb[0] = (uint8_t)((int)a->r + (((int)b->r - (int)a->r) * t) / 255);
            rgb[1] = (uint8_t)((int)a->g + (((int)b->g - (int)a->g) * t) / 255);
            rgb[2] = (uint8_t)((int)a->b + (((int)b->b - (int)a->b) * t) / 255);
            return;
        }
    }
    rgb[0] = stops[n - 1].r;
    rgb[1] = stops[n - 1].g;
    rgb[2] = stops[n - 1].b;
}

void thermal_palette_lookup(ThermalPalette palette, uint8_t level, uint8_t* rgb) {
    switch(palette) {
    case PaletteRainbow:
        gradient_eval(grad_rainbow, COUNT_OF(grad_rainbow), level, rgb);
        break;
    case PaletteGrayscale:
        rgb[0] = rgb[1] = rgb[2] = level;
        break;
    case PaletteHotMetal:
        gradient_eval(grad_hotmetal, COUNT_OF(grad_hotmetal), level, rgb);
        break;
    case PaletteArctic:
        gradient_eval(grad_arctic, COUNT_OF(grad_arctic), level, rgb);
        break;
    case PaletteIronbow:
    default:
        gradient_eval(grad_ironbow, COUNT_OF(grad_ironbow), level, rgb);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Sampling: sensor grid -> output row of RGB
 * ------------------------------------------------------------------------ */

static float sample_temp(const ThermalExportConfig* cfg, float sx, float sy) {
    if(!cfg->bilinear) {
        int x = (int)(sx + 0.5f);
        int y = (int)(sy + 0.5f);
        if(x < 0) x = 0;
        if(x > THERMAL_SENSOR_W - 1) x = THERMAL_SENSOR_W - 1;
        if(y < 0) y = 0;
        if(y > THERMAL_SENSOR_H - 1) y = THERMAL_SENSOR_H - 1;
        return cfg->temps[y * THERMAL_SENSOR_W + x];
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

    const float t00 = cfg->temps[y0 * THERMAL_SENSOR_W + x0];
    const float t10 = cfg->temps[y0 * THERMAL_SENSOR_W + x1];
    const float t01 = cfg->temps[y1 * THERMAL_SENSOR_W + x0];
    const float t11 = cfg->temps[y1 * THERMAL_SENSOR_W + x1];

    const float top = t00 + (t10 - t00) * fx;
    const float bot = t01 + (t11 - t01) * fx;
    return top + (bot - top) * fy;
}

static bool temp_passes_threshold(const ThermalExportConfig* cfg, float t) {
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

/** Fill one output row with RGB triples. `out` must hold width*3 bytes. */
static void
    build_rgb_row(const ThermalExportConfig* cfg, int y, int width, int height, uint8_t* out) {
    float span = cfg->range_max - cfg->range_min;
    if(span < 0.1f) span = 0.1f;

    /* Map output pixel centres onto the sensor grid. */
    const float sy =
        (height > 1) ? ((float)y * (float)(THERMAL_SENSOR_H - 1) / (float)(height - 1)) : 0.0f;

    for(int x = 0; x < width; x++) {
        const int src_x = cfg->mirror ? (width - 1 - x) : x;
        const float sx = (width > 1) ?
                             ((float)src_x * (float)(THERMAL_SENSOR_W - 1) / (float)(width - 1)) :
                             0.0f;

        const float t = sample_temp(cfg, sx, sy);

        if(!temp_passes_threshold(cfg, t)) {
            /* Masked pixels render as near-black so the kept region pops. */
            out[x * 3 + 0] = 12;
            out[x * 3 + 1] = 12;
            out[x * 3 + 2] = 16;
            continue;
        }

        float norm = (t - cfg->range_min) / span;
        if(norm < 0.0f) norm = 0.0f;
        if(norm > 1.0f) norm = 1.0f;

        thermal_palette_lookup(cfg->palette, (uint8_t)(norm * 255.0f + 0.5f), &out[x * 3]);
    }
}

/* ---------------------------------------------------------------------------
 * CRC32 / Adler32
 * ------------------------------------------------------------------------ */

/* Nibble-wise table: 16 entries instead of 256, same result, 1/16 the RAM. */
static const uint32_t crc32_nibble[16] = {
    0x00000000,
    0x1DB71064,
    0x3B6E20C8,
    0x26D930AC,
    0x76DC4190,
    0x6B6B51F4,
    0x4DB26158,
    0x5005713C,
    0xEDB88320,
    0xF00F9344,
    0xD6D6A3E8,
    0xCB61B38C,
    0x9B64C2B0,
    0x86D3D2D4,
    0xA00AE278,
    0xBDBDF21C,
};

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ crc32_nibble[crc & 0x0F];
        crc = (crc >> 4) ^ crc32_nibble[crc & 0x0F];
    }
    return crc;
}

#define ADLER_MOD 65521u

static void adler32_update(uint32_t* a, uint32_t* b, const uint8_t* data, size_t len) {
    uint32_t s1 = *a;
    uint32_t s2 = *b;
    for(size_t i = 0; i < len; i++) {
        s1 += data[i];
        if(s1 >= ADLER_MOD) s1 -= ADLER_MOD;
        s2 += s1;
        if(s2 >= ADLER_MOD) s2 -= ADLER_MOD;
    }
    *a = s1;
    *b = s2;
}

/* ---------------------------------------------------------------------------
 * Byte helpers
 * ------------------------------------------------------------------------ */

static void put_be32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void put_le16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static bool write_all(File* file, const void* data, size_t len) {
    return storage_file_write(file, data, len) == len;
}

/* ---------------------------------------------------------------------------
 * PNG
 * ------------------------------------------------------------------------ */

static bool png_write_chunk(File* file, const char* type, const uint8_t* data, uint32_t len) {
    uint8_t hdr[8];
    put_be32(hdr, len);
    memcpy(hdr + 4, type, 4);

    uint32_t crc = crc32_update(0xFFFFFFFFu, (const uint8_t*)type, 4);
    if(len) crc = crc32_update(crc, data, len);
    crc ^= 0xFFFFFFFFu;

    uint8_t tail[4];
    put_be32(tail, crc);

    if(!write_all(file, hdr, 8)) return false;
    if(len && !write_all(file, data, len)) return false;
    return write_all(file, tail, 4);
}

bool thermal_export_png(Storage* storage, const char* path, const ThermalExportConfig* cfg) {
    furi_assert(storage && path && cfg && cfg->temps);

    uint8_t scale = cfg->scale ? cfg->scale : 1;
    if(scale > 8) scale = 8;

    const int width = THERMAL_SENSOR_W * scale;
    const int height = THERMAL_SENSOR_H * scale;
    const uint32_t row_bytes = 1u + (uint32_t)width * 3u; /* filter byte + RGB */
    const uint32_t raw_size = row_bytes * (uint32_t)height;

    /* Stored deflate blocks cap at 65535 bytes each. */
    const uint32_t block_max = 65535u;
    const uint32_t n_blocks = (raw_size + block_max - 1u) / block_max;
    /* zlib header (2) + per-block headers (5) + payload + adler (4) */
    const uint32_t idat_len = 2u + n_blocks * 5u + raw_size + 4u;

    File* file = storage_file_alloc(storage);
    if(!storage_file_open(file, path, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        FURI_LOG_E(TAG, "cannot open %s", path);
        storage_file_free(file);
        return false;
    }

    uint8_t* row = malloc(PNG_ROW_MAX);
    if(!row) {
        storage_file_close(file);
        storage_file_free(file);
        return false;
    }

    bool ok = true;

    /* -- signature -- */
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    ok = write_all(file, sig, sizeof(sig));

    /* -- IHDR -- */
    if(ok) {
        uint8_t ihdr[13];
        put_be32(ihdr + 0, (uint32_t)width);
        put_be32(ihdr + 4, (uint32_t)height);
        ihdr[8] = 8; /* bit depth */
        ihdr[9] = 2; /* colour type: truecolour RGB */
        ihdr[10] = 0; /* deflate */
        ihdr[11] = 0; /* adaptive filtering */
        ihdr[12] = 0; /* no interlace */
        ok = png_write_chunk(file, "IHDR", ihdr, sizeof(ihdr));
    }

    /*
     * IDAT written incrementally: the chunk length and CRC are computed as we
     * stream, so the full image never has to exist in RAM.
     */
    if(ok) {
        uint8_t hdr[8];
        put_be32(hdr, idat_len);
        memcpy(hdr + 4, "IDAT", 4);
        ok = write_all(file, hdr, 8);
    }

    uint32_t crc = crc32_update(0xFFFFFFFFu, (const uint8_t*)"IDAT", 4);
    uint32_t adler_a = 1;
    uint32_t adler_b = 0;

    /* zlib header: CMF=0x78 (deflate, 32K window), FLG=0x01 -> 0x7801 % 31 == 0 */
    if(ok) {
        const uint8_t zhdr[2] = {0x78, 0x01};
        crc = crc32_update(crc, zhdr, 2);
        ok = write_all(file, zhdr, 2);
    }

    uint32_t remaining = raw_size;
    uint32_t block_left = 0;

    for(int y = 0; ok && y < height; y++) {
        row[0] = 0; /* filter type: None */
        build_rgb_row(cfg, y, width, height, row + 1);

        /*
         * A stored block can end mid-row, so each row is emitted in pieces
         * that never straddle a block boundary. A new block header is written
         * lazily, whenever the current block is exhausted and bytes remain.
         */
        uint32_t row_off = 0;
        while(ok && row_off < row_bytes) {
            if(block_left == 0) {
                const uint32_t this_block = (remaining > block_max) ? block_max : remaining;
                const bool final = (this_block == remaining);

                uint8_t bhdr[5];
                bhdr[0] = final ? 0x01 : 0x00; /* BFINAL, BTYPE=00 (stored) */
                put_le16(bhdr + 1, (uint16_t)this_block);
                put_le16(bhdr + 3, (uint16_t)(~this_block & 0xFFFFu));

                crc = crc32_update(crc, bhdr, sizeof(bhdr));
                ok = write_all(file, bhdr, sizeof(bhdr));
                if(!ok) break;

                block_left = this_block;
            }

            uint32_t n = row_bytes - row_off;
            if(n > block_left) n = block_left;

            crc = crc32_update(crc, row + row_off, n);
            adler32_update(&adler_a, &adler_b, row + row_off, n);
            ok = write_all(file, row + row_off, n);

            row_off += n;
            block_left -= n;
            remaining -= n;
        }
    }

    /* zlib trailer: Adler-32 of the uncompressed data, big-endian. */
    if(ok) {
        uint8_t adler[4];
        put_be32(adler, (adler_b << 16) | adler_a);
        crc = crc32_update(crc, adler, sizeof(adler));
        ok = write_all(file, adler, sizeof(adler));
    }

    if(ok) {
        uint8_t tail[4];
        put_be32(tail, crc ^ 0xFFFFFFFFu);
        ok = write_all(file, tail, sizeof(tail));
    }

    if(ok) ok = png_write_chunk(file, "IEND", NULL, 0);

    free(row);
    storage_file_close(file);
    storage_file_free(file);

    if(!ok) FURI_LOG_E(TAG, "png write failed: %s", path);
    return ok;
}

/* ---------------------------------------------------------------------------
 * TIFF (uncompressed RGB, single strip)
 * ------------------------------------------------------------------------ */

#define TIFF_TAG_COUNT   10
#define TIFF_BPS_OFFSET  8u /* BitsPerSample array lives right after the header */
#define TIFF_DATA_OFFSET 14u /* 8 byte header + 6 byte BitsPerSample */

static void tiff_put_entry(
    uint8_t* p,
    uint16_t tag,
    uint16_t type, /* 3 = SHORT, 4 = LONG */
    uint32_t count,
    uint32_t value) {
    put_le16(p + 0, tag);
    put_le16(p + 2, type);
    put_le32(p + 4, count);
    /* Values of 4 bytes or fewer are stored inline, left-justified. */
    if(type == 3 && count == 1) {
        put_le16(p + 8, (uint16_t)value);
        put_le16(p + 10, 0);
    } else {
        put_le32(p + 8, value);
    }
}

bool thermal_export_tiff(Storage* storage, const char* path, const ThermalExportConfig* cfg) {
    furi_assert(storage && path && cfg && cfg->temps);

    uint8_t scale = cfg->scale ? cfg->scale : 1;
    if(scale > 8) scale = 8;

    const int width = THERMAL_SENSOR_W * scale;
    const int height = THERMAL_SENSOR_H * scale;
    const uint32_t row_bytes = (uint32_t)width * 3u;
    const uint32_t raw_size = row_bytes * (uint32_t)height;
    const uint32_t ifd_offset = TIFF_DATA_OFFSET + raw_size;

    File* file = storage_file_alloc(storage);
    if(!storage_file_open(file, path, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        FURI_LOG_E(TAG, "cannot open %s", path);
        storage_file_free(file);
        return false;
    }

    uint8_t* row = malloc(row_bytes);
    if(!row) {
        storage_file_close(file);
        storage_file_free(file);
        return false;
    }

    bool ok = true;

    /* -- header: little endian, magic 42, IFD offset -- */
    uint8_t hdr[TIFF_DATA_OFFSET];
    hdr[0] = 'I';
    hdr[1] = 'I';
    put_le16(hdr + 2, 42);
    put_le32(hdr + 4, ifd_offset);
    /* BitsPerSample: three SHORTs, too big for the inline value field */
    put_le16(hdr + 8, 8);
    put_le16(hdr + 10, 8);
    put_le16(hdr + 12, 8);
    ok = write_all(file, hdr, sizeof(hdr));

    for(int y = 0; ok && y < height; y++) {
        build_rgb_row(cfg, y, width, height, row);
        ok = write_all(file, row, row_bytes);
    }

    /* -- IFD: entries must be sorted by tag -- */
    if(ok) {
        uint8_t ifd[2 + TIFF_TAG_COUNT * 12 + 4];
        put_le16(ifd, TIFF_TAG_COUNT);
        uint8_t* e = ifd + 2;

        tiff_put_entry(e + 0 * 12, 256, 3, 1, (uint32_t)width); /* ImageWidth */
        tiff_put_entry(e + 1 * 12, 257, 3, 1, (uint32_t)height); /* ImageLength */
        tiff_put_entry(e + 2 * 12, 258, 3, 3, TIFF_BPS_OFFSET); /* BitsPerSample */
        tiff_put_entry(e + 3 * 12, 259, 3, 1, 1); /* Compression: none */
        tiff_put_entry(e + 4 * 12, 262, 3, 1, 2); /* Photometric: RGB */
        tiff_put_entry(e + 5 * 12, 273, 4, 1, TIFF_DATA_OFFSET); /* StripOffsets */
        tiff_put_entry(e + 6 * 12, 277, 3, 1, 3); /* SamplesPerPixel */
        tiff_put_entry(e + 7 * 12, 278, 3, 1, (uint32_t)height); /* RowsPerStrip */
        tiff_put_entry(e + 8 * 12, 279, 4, 1, raw_size); /* StripByteCounts */
        tiff_put_entry(e + 9 * 12, 284, 3, 1, 1); /* PlanarConfig: chunky */

        put_le32(ifd + 2 + TIFF_TAG_COUNT * 12, 0); /* no next IFD */
        ok = write_all(file, ifd, sizeof(ifd));
    }

    free(row);
    storage_file_close(file);
    storage_file_free(file);

    if(!ok) FURI_LOG_E(TAG, "tiff write failed: %s", path);
    return ok;
}

/* ---------------------------------------------------------------------------
 * CSV / raw
 * ------------------------------------------------------------------------ */

bool thermal_export_csv(Storage* storage, const char* path, const float* temps) {
    furi_assert(storage && path && temps);

    File* file = storage_file_alloc(storage);
    if(!storage_file_open(file, path, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_free(file);
        return false;
    }

    bool ok = true;
    char line[THERMAL_SENSOR_W * 8 + 4];

    for(int y = 0; ok && y < THERMAL_SENSOR_H; y++) {
        size_t pos = 0;
        for(int x = 0; x < THERMAL_SENSOR_W; x++) {
            const float t = temps[y * THERMAL_SENSOR_W + x];
            /* Fixed 2 decimals, avoiding %f which pulls in heavy formatting. */
            int scaled = (int)lroundf(t * 100.0f);
            /* Sign printed separately: -0.50 has a whole part of 0. */
            const bool neg = scaled < 0;
            if(neg) scaled = -scaled;

            const int n = snprintf(
                line + pos,
                sizeof(line) - pos,
                "%s%s%d.%02d",
                (x == 0) ? "" : ",",
                neg ? "-" : "",
                scaled / 100,
                scaled % 100);
            if(n < 0 || (size_t)n >= sizeof(line) - pos) {
                ok = false;
                break;
            }
            pos += (size_t)n;
        }
        if(!ok) break;

        if(pos + 2 < sizeof(line)) {
            line[pos++] = '\r';
            line[pos++] = '\n';
        }
        ok = write_all(file, line, pos);
    }

    storage_file_close(file);
    storage_file_free(file);
    return ok;
}

bool thermal_export_raw_append(File* file, const float* temps) {
    furi_assert(file && temps);
    const size_t len = sizeof(float) * THERMAL_SENSOR_W * THERMAL_SENSOR_H;
    return storage_file_write(file, temps, len) == len;
}
