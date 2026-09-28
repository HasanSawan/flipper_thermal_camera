#include "ble_stream.h"

#include <furi.h>
#include <furi_hal_bt.h>
#include <bt/bt_service/bt.h>
#include <profiles/serial_profile.h>

#include <string.h>
#include <stdlib.h>

#define TAG "BleStream"

#define RX_BUF_SIZE          128
#define CHUNK_TIMEOUT_MS     400 /* one indication, including the host's confirm */
#define TX_RETRIES           5
#define TIMEOUTS_BEFORE_IDLE 3 /* host stopped confirming: wait for a new '?' */

struct BleStream {
    Bt* bt;
    FuriHalBleProfileBase* profile;

    FuriSemaphore* tx_sem; /* released when the host confirms an indication */
    FuriStreamBuffer* rx;

    volatile bool connected;
    volatile bool radio_on;
    bool ready; /* host sent '?' on this connection; worker-owned */
    uint8_t timeouts;
    uint16_t chunk;

    uint32_t seq;
    uint8_t* packet; /* USB_STREAM_PACKET_MAX, heap */

    char line[40];
    size_t line_len;
};

/* Runs in the BLE event thread: only copy and signal, never block. */
static uint16_t on_serial_event(SerialServiceEvent event, void* context) {
    BleStream* s = context;

    if(event.event == SerialServiceEventTypeDataReceived) {
        furi_stream_buffer_send(s->rx, event.data.buffer, event.data.size, 0);
        return (uint16_t)furi_stream_buffer_spaces_available(s->rx);
    }
    if(event.event == SerialServiceEventTypeDataSent) {
        furi_semaphore_release(s->tx_sem);
    }
    return 0;
}

/* Runs in the Bt service thread. On every new connection the service hands
 * the serial callbacks to its RPC handler; take them back. */
static void on_bt_status(BtStatus status, void* context) {
    BleStream* s = context;

    s->radio_on = (status != BtStatusOff && status != BtStatusUnavailable);
    if(status == BtStatusConnected) {
        furi_stream_buffer_reset(s->rx);
        ble_profile_serial_set_event_callback(s->profile, RX_BUF_SIZE, on_serial_event, s);
        s->connected = true;
    } else {
        s->connected = false;
    }
}

BleStream* ble_stream_start(void) {
    BleStream* s = malloc(sizeof(BleStream));
    if(!s) return NULL;
    memset(s, 0, sizeof(BleStream));

    s->packet = malloc(USB_STREAM_PACKET_MAX);
    if(!s->packet) {
        free(s);
        return NULL;
    }
    s->tx_sem = furi_semaphore_alloc(1, 0);
    s->rx = furi_stream_buffer_alloc(RX_BUF_SIZE, 1);
    s->chunk = BLE_STREAM_CHUNK_MIN;

    s->bt = furi_record_open(RECORD_BT);

    /* Drop any phone that is connected for RPC, then restart the serial
     * profile to get a handle on it. The status callback goes in first so
     * the advertising / connected transitions are not missed. */
    bt_disconnect(s->bt);
    bt_set_status_changed_callback(s->bt, on_bt_status, s);
    s->profile = bt_profile_start(s->bt, ble_profile_serial, NULL);
    if(!s->profile) {
        FURI_LOG_E(TAG, "cannot start serial profile");
        bt_set_status_changed_callback(s->bt, NULL, NULL);
        bt_profile_restore_default(s->bt);
        furi_record_close(RECORD_BT);
        furi_stream_buffer_free(s->rx);
        furi_semaphore_free(s->tx_sem);
        free(s->packet);
        free(s);
        return NULL;
    }

    /* If Bluetooth is off in the settings nothing advertises and no status
     * change will arrive, so ask the radio directly. */
    s->radio_on = furi_hal_bt_is_active();
    FURI_LOG_I(TAG, "streaming over BLE serial");
    return s;
}

void ble_stream_stop(BleStream* s) {
    if(!s) return;

    /* The Bt service runs the status callback on its own thread and handles
     * its messages in order, so once bt_disconnect() has returned no status
     * callback can still be running with this stream. */
    bt_set_status_changed_callback(s->bt, NULL, NULL);
    bt_disconnect(s->bt);
    ble_profile_serial_set_event_callback(s->profile, 0, NULL, NULL);
    bt_profile_restore_default(s->bt);
    furi_record_close(RECORD_BT);

    furi_stream_buffer_free(s->rx);
    furi_semaphore_free(s->tx_sem);
    free(s->packet);
    free(s);
}

BleLink ble_stream_link(BleStream* s) {
    if(!s->connected) {
        s->ready = false;
        return s->radio_on ? BleLinkAdvertising : BleLinkRadioOff;
    }
    return s->ready ? BleLinkReady : BleLinkConnected;
}

void ble_stream_set_chunk(BleStream* s, uint16_t chunk) {
    if(chunk < BLE_STREAM_CHUNK_MIN) chunk = BLE_STREAM_CHUNK_MIN;
    if(chunk > BLE_STREAM_CHUNK_MAX) chunk = BLE_STREAM_CHUNK_MAX;
    s->chunk = chunk;
}

static UsbSendResult send_packet(BleStream* s, size_t len) {
    /* A confirm that arrived after an earlier timeout must not let the next
     * chunk go before its own confirm. */
    while(furi_semaphore_acquire(s->tx_sem, 0) == FuriStatusOk) {
    }

    size_t off = 0;
    while(off < len) {
        if(!s->connected) return UsbSendNoHost;

        const size_t n = (len - off > s->chunk) ? s->chunk : (len - off);
        bool queued = false;
        for(int i = 0; i < TX_RETRIES && !queued; i++) {
            queued = ble_profile_serial_tx(s->profile, s->packet + off, (uint16_t)n);
            if(!queued) furi_delay_ms(5);
        }
        if(!queued ||
           furi_semaphore_acquire(s->tx_sem, furi_ms_to_ticks(CHUNK_TIMEOUT_MS)) != FuriStatusOk) {
            if(++s->timeouts >= TIMEOUTS_BEFORE_IDLE) {
                /* The host went away without disconnecting, or stopped
                 * listening: stop sending until it asks again. */
                s->ready = false;
                s->timeouts = 0;
            }
            return UsbSendTimeout;
        }
        off += n;
    }
    s->timeouts = 0;
    return UsbSendOk;
}

UsbSendResult ble_stream_send_frame(BleStream* s, const UsbFrameMeta* meta, const float* temps) {
    furi_assert(s && meta && temps);
    if(ble_stream_link(s) != BleLinkReady) return UsbSendNoHost;

    const size_t len = usb_stream_build_packet(
        s->packet, UsbPacketFrame, s->seq++, furi_get_tick(), meta, temps, NULL);
    return send_packet(s, len);
}

UsbSendResult ble_stream_send_info(BleStream* s, const UsbFrameMeta* meta, const char* text) {
    furi_assert(s && text);
    if(ble_stream_link(s) != BleLinkReady) return UsbSendNoHost;

    const size_t len = usb_stream_build_packet(
        s->packet, UsbPacketInfo, s->seq++, furi_get_tick(), meta, NULL, text);
    return send_packet(s, len);
}

bool ble_stream_poll_command(BleStream* s, char* line, size_t line_size) {
    furi_assert(s && line && line_size > 0);

    uint8_t c;
    while(furi_stream_buffer_receive(s->rx, &c, 1, 0) == 1) {
        if(c == '\r') continue;
        if(c == '\n') {
            const size_t n = (s->line_len < line_size - 1) ? s->line_len : line_size - 1;
            memcpy(line, s->line, n);
            line[n] = '\0';
            s->line_len = 0;
            if(n == 0) continue;
            /* The host announcing itself is what starts the stream. */
            if(line[0] == '?') {
                s->ready = s->connected;
                s->timeouts = 0;
            }
            return true;
        }
        if(s->line_len < sizeof(s->line)) s->line[s->line_len++] = (char)c;
    }
    return false;
}
