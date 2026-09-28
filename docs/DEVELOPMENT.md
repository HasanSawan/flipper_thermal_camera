# Development notes

Technical background for people changing the code. For using the app, see the [README](../README.md). Version history is in [CHANGELOG.md](CHANGELOG.md).

## Source layout

| File | Role |
|---|---|
| `thermal_cam.c` | Entry point, app allocation, view dispatcher, redraw timer |
| `thermal_cam.h` | Shared types: settings, display buffer, app state, mode enum |
| `thermal_worker.c` | Sensor thread: I²C, calibration, stats, spot history, capture, recording, USB streaming |
| `thermal_view.c` | Camera screen (image, spot meter, USB), key handling, About screen |
| `thermal_settings.c` | Parameter tables, settings file load/save/migration, settings menu |
| `mlx90640.c/.h` | Sensor driver and Melexis calibration port |
| `render.c/.h` | 1-bit rendering: Bayer dither, gamma, histogram EQ, contours, threshold |
| `img_export.c/.h` | PNG / TIFF / CSV / raw writers, palettes |
| `usb_stream.c/.h` | Dual-CDC USB port handling and the stream packet format |
| `ble_stream.c/.h` | Bluetooth LE transport for the same packets, over the Flipper's BLE serial service |
| `tools/viewer.html` | Browser viewer (Web Serial and Web Bluetooth), single file, no dependencies |
| `tools/raw2png.py` | Converts raw recordings to PNG / GIF |

## Threading

| Thread | Does |
|---|---|
| GUI | View dispatcher, drawing, key handling |
| Worker (5 kB stack) | I²C, calibration maths, rendering, SD writes, USB and BLE transfers |
| Timer | Nudges a redraw at 25 Hz |

The threads share only the double-buffered `DisplayBuffer` (published with a single index store, so the draw path takes no lock), the settings struct (mutex-guarded) and a handful of `volatile` flags. A slow SD write, a stalled I²C transfer or a USB host that stops reading can never block the UI.

Buffers that would be large on the 5 kB worker stack live on the heap: the frame temperatures, the spot history (inside the app struct), and the 1.5 kB USB packet buffer (allocated when streaming starts).

## Two Flipper-specific I²C problems

**`furi_hal_i2c_trx()` cannot talk to this sensor.** It issues a STOP between the register write and the data read. The MLX90640 needs a *repeated START*; with a STOP the address pointer is discarded and you read garbage. The driver pairs `furi_hal_i2c_tx_ext(..., FuriHalI2cEndAwaitRestart)` with `furi_hal_i2c_rx_ext(..., FuriHalI2cBeginRestart)`. The address is passed pre-shifted (`0x33 << 1 = 0x66`) because `furi_hal` hands it straight to the ST LL layer.

**The external I²C bus is hardcoded to 100 kHz.** The driver rewrites `I2C3->TIMINGR` after acquiring the bus (disable PE, set timing, re-enable). 100 kHz and 400 kHz use the firmware's own validated timing constants; the 1 MHz value is computed and is marked experimental.

## Display

The sensor is 32×24 (4:3), the screen 128×64 (2:1). Filling the full 64 px height at the sensor's own aspect gives an 85 px wide image and a 43 px sidebar. Continuous tone uses an 8×8 Bayer ordered dither written into an XBM bitmap (LSB-first, stride `(w+7)/8`) and drawn with one `canvas_draw_xbm` call.

### Per-mode parameters

| Mode | Parameter | Implementation |
|---|---|---|
| Linear / Inverted | Gamma 0.4–2.5 | 256-entry LUT built once per frame, applied before dithering |
| Hist EQ | Strength 0–100 % | 64-bin CDF; blended per pixel against the full-resolution linear level, so 0 % is exactly Linear |
| Contour | Step 0.5–10 ° | Absolute iso-lines at multiples of the step: `floor((t + offset) / step)`. For °F, `step/1.8` and `offset = 32/1.8` so lines land on round Fahrenheit values. Lines do not move when the auto-range changes |
| Threshold | Threshold temperature | Above / Below move one value; Band slides both edges together |
| Spot | Spot diameter 1/3/5/7 px | Disc of pixels with `4(dx²+dy²) ≤ d²` around sensor pixel (16, 12): 1, 9, 21, 37 pixels |
| Human | Body offset −2.0 – +5.0 °C | Estimate = EMA(hottest pixel in columns 8–23, rows 3–20) + offset, at emissivity 0.98. Status needs 3 frames in a row and 0.1 °C hysteresis to change. Skin below 30 °C or above 43 °C counts as nobody in view |
| USB / Bluetooth | Refresh rate | Existing rate setting |

Mode values are stored in the settings file, so new modes are appended to `CamMode` (Human = 7, Bluetooth = 8). The Left/Right order and the on-screen number come from `thermal_mode_order[]`.

The threshold mask is only passed to the renderer and exporter in the Threshold view (`fill_render_config`, `fill_export_config`).

All parameter values are stored as **indices into fixed tables** (`thermal_settings.c`), and every index is range-checked on load, so a corrupted settings file can never index past a table.

### Spot history

`SpotHistory` holds one column per graph pixel (98). Columns advance by wall-clock time (`period / 98` per column), not per frame, so the time axis stays honest at any refresh rate. Each column keeps min, max and last value; the publisher joins each column to the previous one's last value, which draws a continuous trace without storing individual samples. A gap longer than the whole window, or a change of period, resets the buffer. The history runs in every mode, so the spot screen already has data when you switch to it. Tick wrap-around is handled by unsigned subtraction.

The worker converts the history to screen rows (`graph_top` / `graph_bot`) when it publishes a frame, so the draw callback only draws lines.

Each column also stores the RTC time it started, for the CSV files. **Graph log:** every completed column counts as `pending`. Before the ring would overwrite an unlogged column (at `pending == GRAPH_W − 1`), and before any reset (period change, long gap) or exit, the pending columns plus the one in progress are appended to `LOG_<time>.csv` through a flush callback.

## Settings file

`/ext/thermal_cam/settings.bin`: `{u32 magic "MLX1", u16 version, u16 size}` followed by `size` bytes of `ThermalSettings`.

Fields are only ever **appended**. The loader copies `size` bytes over the defaults, so an older file loads as a prefix and the newer fields keep their defaults. Version 1 (app 1.0) and version 2 (app 1.1) files are migrated this way; the fields newer than the file are explicitly reset in case old trailing padding overlapped them. Version 3 (app 1.2) added `graph_log`, `body_offset_x10`, `fever_idx` and `low_idx`. Files with a newer version, an oversized length or a bad magic are ignored.

`thermal_settings_sanitize()` then clamps every field. Booleans are normalised through their byte representation, because loading a `bool` that holds e.g. `0xB0` is undefined behaviour. The app never starts in USB or Bluetooth mode, since streaming is opt-in. **Factory Reset** writes the defaults and saves them.

## USB streaming

### Port handling

Streaming uses the same sequence as the firmware's USB-UART Bridge on "channel 1":

1. `furi_hal_usb_unlock()`, `furi_hal_usb_set_config(&usb_cdc_dual, NULL)`
2. `cli_vcp_enable()`, so the command line stays on interface 0 (`/dev/ttyACM0`)
3. `furi_hal_cdc_set_callbacks(1, ...)`: the stream owns interface 1 (`/dev/ttyACM1`)

Stopping reverses this and restores `usb_cdc_single`. The worker owns the port: the GUI only sets `usb_want`, and the worker opens and closes it. It always closes it on exit, so the Flipper is never left in dual mode. Leaving the USB screen also ends the stream.

Firmware before the `CliVcp` record existed used `cli_session_open/close`. `usb_stream.c` detects that with `#ifndef RECORD_CLI_VCP` and uses the old calls.

Frames are sent only while the host has the port open (DTR set). Each 64-byte bulk packet waits for the previous one's TX-complete callback, with a 60 ms timeout. A timeout counts the frame as dropped and never blocks the worker for longer. Stream packets are 1564 bytes, which is not a multiple of 64, so the final short packet ends the USB transfer and the host sees each frame immediately.

### Packet format

All little-endian; one packet per frame.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | Magic `TCAM` |
| 4 | 1 | Protocol version (1) |
| 5 | 1 | Type: 1 = frame, 2 = info text |
| 6 | 2 | Payload length |
| 8 | 4 | Sequence number (per packet; gaps = drops) |
| 12 | 4 | Device time, ms |
| 16 | 2 | Ambient (die) temperature, int16 centi-°C |
| 18 | 1 | Emissivity, percent |
| 19 | 1 | Flags: bit0 chess mode, bit1 subpage |
| 20 | 1 | Refresh-rate code (0 = 0.5 Hz … 7 = 64 Hz) |
| 21 | 1 | I²C speed code (0 = 100 kHz, 1 = 400 kHz, 2 = 1 MHz) |
| 22 | 2 | Reserved (0) |
| 24 | N | Frame: 768 × int16 centi-°C, row-major 32×24 · Info: UTF-8 `key=value;...` |
| 24+N | 4 | CRC-32 (IEEE, as zip/PNG) over bytes 0 … 24+N−1 |

Commands from the PC are ASCII lines ending in `\n`:

| Command | Effect |
|---|---|
| `?` | Reply with an info packet |
| `E<50..100>` | Emissivity in percent |
| `R<0..7>` | Refresh-rate code |
| `S<0..2>` | I²C speed code |
| `C<0/1>` | Read mode: 1 chess, 0 interleaved |
| `K` | Save a still to the SD card |
| `G` | Save the spot-meter graph as CSV on the SD card |
| `V<0/1>` | Stop / start recording to the SD card (a recording started this way also stops when the stream closes) |
| `M<20..243>` | Bluetooth only: largest indication payload (ATT MTU − 3) |

Malformed or out-of-range commands are ignored. Every accepted change is confirmed with an info packet. Info text since 1.2 also carries `rec=0|1` and `link=usb|ble`.

## Bluetooth streaming

`ble_stream.c` reuses the Flipper's own BLE serial service (the one the phone app uses) instead of adding a GATT profile:

1. `bt_disconnect()`, register a status callback, then `bt_profile_start(bt, ble_profile_serial, NULL)` to get the profile handle.
2. On every connection the Bt service hands the serial callbacks to its RPC handler. The status callback (which runs after that, on the Bt thread) calls `ble_profile_serial_set_event_callback()` to take them back.
3. Received bytes go into a stream buffer and are parsed as command lines. Nothing is sent until the host sends a line starting with `?`, so a phone connecting for RPC is never flooded.
4. Each packet goes out in indications of `chunk` bytes; the next one waits for the host's confirmation (`DataSent`, 400 ms timeout). Three timeouts in a row stop the stream until the host says `?` again.
5. Stopping clears the status callback, disconnects (which serialises with the Bt thread), clears the data callback and calls `bt_profile_restore_default()`.

The app cannot read the negotiated MTU, so the default chunk is 20 bytes; the viewer asks for `M180` and falls back to `M20` if no packet arrives intact. Characteristics need an authenticated link, so the first connection pairs with the PIN shown on the Flipper. UUIDs: service `8fe5b3d5-2e7f-4a98-2a48-7acc60fe0000`, from Flipper (indicate) `19ed82ae-ed21-4c9d-4145-228e61fe0000`, to Flipper (write) `19ed82ae-ed21-4c9d-4145-228e62fe0000`.

## Exiting

`view_dispatcher_run()` returns only after every key that was down is released, so holding Back closes the app when Back is let go. `thermal_request_exit()` (Back long or repeat, or Settings → Exit App) first clears `worker_running`, `recording`, `usb_want` and `ble_want`, so the worker is already closing files and ports by then. Writing your own client is straightforward: sync on `TCAM`, check the length, then check the CRC.

## Calibration

Ported from the Melexis MLX90640 reference driver (Apache 2.0, © 2017 Melexis N.V.): EEPROM dump → parameter extraction → per-pixel gain/offset/Kta/Kv correction → `To` with the four-band Ks correction. Chess mode delivers one subpage per frame; the app keeps the previous subpage so the picture looks continuous.

Fuzzing the port found and fixed three defects that are also in the reference driver: undefined left-shifts of negative values in three extraction functions, and `Ta`/`Vdd` becoming Inf when a calibration constant reads back as zero.

## Testing

- **Build:** CI builds the app with uFBT against the release, release-candidate and dev SDKs on every push (see `.github/workflows/build.yml`). The code compiles cleanly with the SDK's warning flags (`-Wall -Wextra -Werror`).
- **Logic:** spot-history maths, gamma / EQ / contour rendering, stream command parsing and settings migration were exercised on the host under AddressSanitizer and UndefinedBehaviorSanitizer, including thousands of randomised settings files.
- **Screens:** the images in `docs/images/screens/` are drawn by the app's own drawing code through the firmware's u8g2 library and fonts, with a simulated scene.
- **Stream protocol:** packets were validated independently in Python (CRC-32, framing, sequence numbers, decoded temperatures), and the viewer was tested in Chromium against recorded packets.

### Hardware notes

1. **Accuracy:** check against something of known temperature (the palm of your hand is about 33 °C).
2. **USB mode switching** relies on the firmware's dual-CDC configuration; third-party firmware may differ.
3. **1 MHz I²C** is experimental.
4. **Firmware older than 0.98** needs `view_dispatcher_enable_queue()` in `thermal_cam_app_alloc()` (commented in place).
