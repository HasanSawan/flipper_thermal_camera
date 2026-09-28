#include "usb_stream.h"

#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Packet building (pure, no hardware)
 * ------------------------------------------------------------------------ */

/* Nibble-wise CRC-32: a 16-entry table instead of 256, fast enough for
 * ~1.5 kB per frame and cheap on flash. */
static const uint32_t crc_nibble[16] = {
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

uint32_t usb_stream_crc32(const uint8_t* data, size_t len) {
    uint32_t c = 0xFFFFFFFFUL;
    for(size_t i = 0; i < len; i++) {
        c ^= data[i];
        c = (c >> 4) ^ crc_nibble[c & 0x0F];
        c = (c >> 4) ^ crc_nibble[c & 0x0F];
    }
    return c ^ 0xFFFFFFFFUL;
}

static inline void put_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static int16_t to_centi(float t) {
    /* NaN compares false everywhere and lands on 0 */
    if(!(t > -327.0f)) return (t < 0.0f) ? -32700 : 0;
    if(t > 327.0f) return 32700;
    const float c = t * 100.0f;
    return (int16_t)(c < 0.0f ? c - 0.5f : c + 0.5f);
}

size_t usb_stream_build_packet(
    uint8_t* out,
    UsbPacketType type,
    uint32_t seq,
    uint32_t time_ms,
    const UsbFrameMeta* meta,
    const float* temps,
    const char* text) {
    uint16_t payload = 0;

    if(type == UsbPacketFrame) {
        payload = USB_STREAM_FRAME_PAYLOAD;
        uint8_t* p = out + USB_STREAM_HEADER;
        for(int i = 0; i < USB_STREAM_PIXELS; i++) {
            put_u16(p + i * 2, (uint16_t)to_centi(temps[i]));
        }
    } else {
        size_t n = text ? strlen(text) : 0;
        if(n > USB_STREAM_TEXT_MAX) n = USB_STREAM_TEXT_MAX;
        memcpy(out + USB_STREAM_HEADER, text, n);
        payload = (uint16_t)n;
    }

    put_u32(out + 0, USB_STREAM_MAGIC);
    out[4] = USB_STREAM_VERSION;
    out[5] = (uint8_t)type;
    put_u16(out + 6, payload);
    put_u32(out + 8, seq);
    put_u32(out + 12, time_ms);
    put_u16(out + 16, (uint16_t)to_centi(meta ? meta->ambient : 0.0f));
    out[18] = meta ? meta->emissivity_pct : 0;
    out[19] = meta ? (uint8_t)((meta->chess ? 1u : 0u) | ((meta->subpage & 1u) << 1)) : 0;
    out[20] = meta ? meta->rate : 0;
    out[21] = meta ? meta->speed : 0;
    out[22] = 0;
    out[23] = 0;

    const size_t body = USB_STREAM_HEADER + payload;
    put_u32(out + body, usb_stream_crc32(out, body));
    return body + USB_STREAM_CRC;
}

#ifndef USB_STREAM_HOST_TEST

/* ---------------------------------------------------------------------------
 * Transport
 * ------------------------------------------------------------------------ */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_usb.h>
#include <furi_hal_usb_cdc.h>
#include <cli/cli_vcp.h>

#define TAG "UsbStream"

#define STREAM_IF      1 /* CDC interface 1 = /dev/ttyACM1 */
#define PKT_SIZE       CDC_DATA_SZ /* 64-byte full-speed bulk packets */
#define PKT_TIMEOUT_MS 60

/* Firmware 1.3+ moved the command-line USB port behind a CliVcp record.
 * Older firmware used cli_session_open/close; keep both working. */
#ifndef RECORD_CLI_VCP
#include <cli/cli.h>
#define USB_STREAM_LEGACY_CLI 1
#endif

struct UsbStream {
    FuriSemaphore* tx_sem;
    volatile bool rx_pending;
    uint32_t seq;
    uint8_t* packet; /* USB_STREAM_PACKET_MAX, heap: keeps the worker stack small */

    uint8_t rx_buf[PKT_SIZE];
    size_t rx_len;
    size_t rx_pos;
    char line[40];
    size_t line_len;

#ifndef USB_STREAM_LEGACY_CLI
    CliVcp* cli_vcp;
#endif
};

/* These run in the USB interrupt: only signal, never block. */
static void on_tx_complete(void* context) {
    UsbStream* s = context;
    furi_semaphore_release(s->tx_sem);
}

static void on_rx(void* context) {
    UsbStream* s = context;
    s->rx_pending = true;
}

static CdcCallbacks stream_callbacks = {
    .tx_ep_callback = on_tx_complete,
    .rx_ep_callback = on_rx,
    .state_callback = NULL,
    .ctrl_line_callback = NULL,
    .config_callback = NULL,
};

UsbStream* usb_stream_start(void) {
    UsbStream* s = malloc(sizeof(UsbStream));
    if(!s) return NULL;
    memset(s, 0, sizeof(UsbStream));

    s->packet = malloc(USB_STREAM_PACKET_MAX);
    if(!s->packet) {
        free(s);
        return NULL;
    }
    s->tx_sem = furi_semaphore_alloc(1, 1);

    /* Same sequence as the firmware's own USB-UART bridge on channel 1. */
    furi_hal_usb_unlock();
    if(!furi_hal_usb_set_config(&usb_cdc_dual, NULL)) {
        FURI_LOG_E(TAG, "cannot switch to dual CDC");
        furi_semaphore_free(s->tx_sem);
        free(s->packet);
        free(s);
        return NULL;
    }

#ifdef USB_STREAM_LEGACY_CLI
    Cli* cli = furi_record_open(RECORD_CLI);
    cli_session_open(cli, &cli_vcp);
    furi_record_close(RECORD_CLI);
#else
    s->cli_vcp = furi_record_open(RECORD_CLI_VCP);
    cli_vcp_enable(s->cli_vcp);
#endif

    furi_hal_cdc_set_callbacks(STREAM_IF, &stream_callbacks, s);
    FURI_LOG_I(TAG, "streaming on CDC interface %d", STREAM_IF);
    return s;
}

void usb_stream_stop(UsbStream* s) {
    if(!s) return;

    furi_hal_cdc_set_callbacks(STREAM_IF, NULL, NULL);

#ifdef USB_STREAM_LEGACY_CLI
    Cli* cli = furi_record_open(RECORD_CLI);
    cli_session_close(cli);
    furi_hal_usb_unlock();
    furi_hal_usb_set_config(&usb_cdc_single, NULL);
    cli_session_open(cli, &cli_vcp);
    furi_record_close(RECORD_CLI);
#else
    cli_vcp_disable(s->cli_vcp);
    furi_hal_usb_unlock();
    furi_hal_usb_set_config(&usb_cdc_single, NULL);
    cli_vcp_enable(s->cli_vcp);
    furi_record_close(RECORD_CLI_VCP);
#endif

    furi_semaphore_free(s->tx_sem);
    free(s->packet);
    free(s);
}

bool usb_stream_host_connected(UsbStream* s) {
    UNUSED(s);
    return (furi_hal_cdc_get_ctrl_line_state(STREAM_IF) & CdcCtrlLineDTR) != 0;
}

static UsbSendResult send_packet(UsbStream* s, size_t len) {
    if(!usb_stream_host_connected(s)) return UsbSendNoHost;

    /* Chunk into 64-byte bulk packets, one in flight at a time. Packet sizes
     * are never a multiple of 64, so the final short packet ends the USB
     * transfer and the host sees the frame immediately. */
    size_t off = 0;
    while(off < len) {
        if(furi_semaphore_acquire(s->tx_sem, furi_ms_to_ticks(PKT_TIMEOUT_MS)) != FuriStatusOk) {
            return UsbSendTimeout;
        }
        const size_t n = (len - off > PKT_SIZE) ? PKT_SIZE : (len - off);
        furi_hal_cdc_send(STREAM_IF, s->packet + off, (uint16_t)n);
        off += n;
    }
    return UsbSendOk;
}

UsbSendResult usb_stream_send_frame(UsbStream* s, const UsbFrameMeta* meta, const float* temps) {
    furi_assert(s && meta && temps);
    if(!usb_stream_host_connected(s)) return UsbSendNoHost;

    const size_t len = usb_stream_build_packet(
        s->packet, UsbPacketFrame, s->seq++, furi_get_tick(), meta, temps, NULL);
    return send_packet(s, len);
}

UsbSendResult usb_stream_send_info(UsbStream* s, const UsbFrameMeta* meta, const char* text) {
    furi_assert(s && text);
    if(!usb_stream_host_connected(s)) return UsbSendNoHost;

    const size_t len = usb_stream_build_packet(
        s->packet, UsbPacketInfo, s->seq++, furi_get_tick(), meta, NULL, text);
    return send_packet(s, len);
}

bool usb_stream_poll_command(UsbStream* s, char* line, size_t line_size) {
    furi_assert(s && line && line_size > 0);

    for(;;) {
        if(s->rx_pos >= s->rx_len) {
            if(!s->rx_pending) return false;
            s->rx_pending = false;
            const int32_t n = furi_hal_cdc_receive(STREAM_IF, s->rx_buf, PKT_SIZE);
            s->rx_len = (n > 0) ? (size_t)n : 0;
            s->rx_pos = 0;
            if(s->rx_len == 0) return false;
        }

        while(s->rx_pos < s->rx_len) {
            const char c = (char)s->rx_buf[s->rx_pos++];
            if(c == '\r') continue;
            if(c == '\n') {
                const size_t n = (s->line_len < line_size - 1) ? s->line_len : line_size - 1;
                memcpy(line, s->line, n);
                line[n] = '\0';
                s->line_len = 0;
                if(n > 0) return true;
                continue;
            }
            /* Over-long lines are truncated rather than overflowing. */
            if(s->line_len < sizeof(s->line)) s->line[s->line_len++] = c;
        }
    }
}

#endif /* USB_STREAM_HOST_TEST */
