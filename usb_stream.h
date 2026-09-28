/**
 * USB streaming of calibrated frames.
 *
 * The Flipper is switched to its dual-port USB mode (the same thing the
 * built-in USB-UART Bridge does with "USB channel 1"):
 *   port 0  (/dev/ttyACM0)  stays the normal command line, so qFlipper and
 *                           ufbt keep working
 *   port 1  (/dev/ttyACM1)  carries the thermal stream
 *
 * Wire format, all little-endian, one packet per frame:
 *
 *   off size  field
 *     0   4   magic "TCAM"
 *     4   1   protocol version (1)
 *     5   1   packet type: 1 = frame, 2 = info text
 *     6   2   payload length in bytes
 *     8   4   sequence number (increments per packet)
 *    12   4   device time, ms
 *    16   2   ambient (die) temperature, int16 centi-degrees C
 *    18   1   emissivity, percent
 *    19   1   flags: bit0 chess mode, bit1 last subpage (0/1)
 *    20   1   refresh-rate code (0 = 0.5 Hz .. 7 = 64 Hz)
 *    21   1   I2C speed code (0 = 100 kHz, 1 = 400 kHz, 2 = 1 MHz)
 *    22   2   reserved, 0
 *    24   N   payload
 *                frame: 768 x int16 centi-degrees C, row-major, 32 x 24
 *                info : UTF-8 text, "key=value;key=value..."
 *  24+N   4   CRC-32 (IEEE, same as PNG/zip) of bytes 0 .. 24+N-1
 *
 * Commands from the PC are ASCII lines ending in '\n':
 *   ?        send an info packet
 *   E<pct>   emissivity, 50..100
 *   R<code>  refresh-rate code, 0..7
 *   S<code>  I2C speed code, 0..2
 *   C<0|1>   chess (1) or interleaved (0) read mode
 *   K        capture a still to the SD card, as if OK was pressed
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_STREAM_MAGIC         0x4D414354UL /* "TCAM" read as little-endian u32 */
#define USB_STREAM_VERSION       1
#define USB_STREAM_HEADER        24
#define USB_STREAM_CRC           4
#define USB_STREAM_PIXELS        768
#define USB_STREAM_FRAME_PAYLOAD (USB_STREAM_PIXELS * 2)
#define USB_STREAM_TEXT_MAX      200
#define USB_STREAM_PACKET_MAX    (USB_STREAM_HEADER + USB_STREAM_FRAME_PAYLOAD + USB_STREAM_CRC)

typedef enum {
    UsbPacketFrame = 1,
    UsbPacketInfo = 2,
} UsbPacketType;

/** Per-frame metadata carried in the packet header. */
typedef struct {
    float ambient;
    uint8_t emissivity_pct;
    uint8_t rate;
    uint8_t speed;
    bool chess;
    uint8_t subpage;
} UsbFrameMeta;

typedef enum {
    UsbSendOk = 0,
    UsbSendNoHost, /* nothing has the port open: frame skipped, not an error */
    UsbSendTimeout, /* host stopped reading mid-frame */
} UsbSendResult;

typedef struct UsbStream UsbStream;

/** Switch USB to dual-port mode and claim port 1. NULL if USB is busy. */
UsbStream* usb_stream_start(void);

/** Give port 1 back and restore the normal single-port USB mode. */
void usb_stream_stop(UsbStream* stream);

/** True when a program on the PC has the port open (DTR asserted). */
bool usb_stream_host_connected(UsbStream* stream);

UsbSendResult
    usb_stream_send_frame(UsbStream* stream, const UsbFrameMeta* meta, const float* temps);

UsbSendResult usb_stream_send_info(UsbStream* stream, const UsbFrameMeta* meta, const char* text);

/**
 * Collect any bytes the PC sent. Returns true and fills `line` (NUL-terminated,
 * without the newline) each time a complete command line is available; call
 * repeatedly until it returns false.
 */
bool usb_stream_poll_command(UsbStream* stream, char* line, size_t line_size);

/* -- pure helpers, exposed for host-side tests -- */

/** Build a packet into `out` (USB_STREAM_PACKET_MAX bytes). Returns its size. */
size_t usb_stream_build_packet(
    uint8_t* out,
    UsbPacketType type,
    uint32_t seq,
    uint32_t time_ms,
    const UsbFrameMeta* meta,
    const float* temps,
    const char* text);

uint32_t usb_stream_crc32(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif
