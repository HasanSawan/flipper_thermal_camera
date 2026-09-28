/**
 * Bluetooth LE streaming of calibrated frames.
 *
 * Uses the Flipper's own BLE serial service (the one the mobile app talks to),
 * so a browser can connect with Web Bluetooth and no extra pairing profile is
 * needed. While streaming, the app takes over the service's data callbacks
 * from the firmware's RPC handler; stopping restores the default profile, so
 * the mobile app works again afterwards.
 *
 * The packets are byte-for-byte the USB stream packets (see usb_stream.h),
 * split into BLE indications of at most `chunk` bytes. The host sends the same
 * ASCII commands, plus:
 *   M<n>     largest indication payload the host can take, 20..243 bytes
 *            (its ATT MTU minus 3). Default 20, which every host accepts.
 *
 * Nothing is sent until the host has sent a line starting with '?', so a phone
 * that connects for RPC is never flooded with frames.
 *
 * GATT, as seen from the host:
 *   service 8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000
 *   19ed82ae-ed21-4c9d-4145-228e62fe0000  write: commands to the Flipper
 *   19ed82ae-ed21-4c9d-4145-228e61fe0000  indicate: stream from the Flipper
 * Both characteristics need an encrypted (paired) link; the Flipper shows a
 * PIN the first time.
 */
#pragma once

#include "usb_stream.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_STREAM_CHUNK_MIN 20
#define BLE_STREAM_CHUNK_MAX 243

typedef enum {
    BleLinkRadioOff = 0, /* Bluetooth disabled in the Flipper's settings */
    BleLinkAdvertising,
    BleLinkConnected, /* connected, host has not asked for the stream yet */
    BleLinkReady, /* host sent '?': frames are wanted */
} BleLink;

typedef struct BleStream BleStream;

/** Take over the BLE serial service. NULL if the radio stack cannot. */
BleStream* ble_stream_start(void);

/** Hand the service back to the firmware and restore its default profile. */
void ble_stream_stop(BleStream* stream);

BleLink ble_stream_link(BleStream* stream);

/** Set the indication payload size, clamped to the limits above. */
void ble_stream_set_chunk(BleStream* stream, uint16_t chunk);

UsbSendResult
    ble_stream_send_frame(BleStream* stream, const UsbFrameMeta* meta, const float* temps);

UsbSendResult ble_stream_send_info(BleStream* stream, const UsbFrameMeta* meta, const char* text);

/** Same contract as usb_stream_poll_command(). */
bool ble_stream_poll_command(BleStream* stream, char* line, size_t line_size);

#ifdef __cplusplus
}
#endif
