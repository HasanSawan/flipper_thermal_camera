# Thermal Camera

Turn your Flipper Zero into a pocket thermal camera with an MLX90640 32x24 infrared sensor connected to the GPIO header.

## Features

- Live thermal image with max, centre and min temperatures, and markers for the hottest and coldest spots
- Nine view modes: Linear, Inverted, Histogram EQ, Contour, Threshold, Spot meter, Human, USB and Bluetooth
- Up and Down always adjust the setting of the current mode: gamma, EQ strength, contour step, threshold, spot size, body offset or refresh rate
- Spot meter with a large readout and a temperature-history graph (10 s to 10 min), saved as CSV or logged continuously
- Human mode: body-temperature screening estimate with LED and vibration alerts. Not a medical device
- Save stills as colour PNG or TIFF (five palettes) plus raw CSV temperatures
- Hold OK to record an animation
- Stream live over USB or Bluetooth to a browser viewer with measurement points, boxes, graphs, recording and saving of the last few minutes

## Hardware

- MLX90640 breakout board, BAA (110 degrees) or BAB (55 degrees). Adafruit, Pimoroni, SparkFun and generic boards work
- VIN to pin 9 (3.3 V). Do not use pin 1 (5 V)
- GND to pin 18
- SDA to pin 15 (C1)
- SCL to pin 16 (C0)
- For a stable 400 kHz bus, add 2.2k to 4.7k pull-up resistors from SDA and SCL to 3.3 V and keep the wires short

## Controls

- **OK**: save a still image. In Spot mode the graph is saved too. In USB and Bluetooth modes, start or stop streaming
- **Hold OK**: record while held
- **Left / Right**: change view mode
- **Up / Down**: adjust the current mode's setting
- **Back**: settings
- **Hold Back**: exit

## Streaming viewer

Open the viewer page from the tools folder of the project repository in Chrome, Chromium or Edge, then click Connect USB or Connect Bluetooth. Bluetooth runs at about 1 to 4 frames per second.

## Saved files

Images, CSV files, graphs, logs and recordings are saved on the SD card in the thermal_cam folder.

## Disclaimer

Human mode gives a rough screening estimate only and can be wrong by a degree or more. It is not a medical device. Always confirm with a clinical thermometer.

Full documentation, wiring photos and the browser viewer: see the project repository on GitHub.
