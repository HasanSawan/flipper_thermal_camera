# Thermal Camera

Turn your Flipper Zero into a thermal camera with an MLX90640 32x24 infrared sensor connected to the GPIO header.

## Features

- Live thermal image with max, centre and min temperatures
- Nine view modes: Linear, Inverted, Hist EQ, Contour, Threshold, Spot, Human, USB and Bluetooth
- Spot meter with a temperature-history graph
- Human mode: body-temperature screening estimate. Not a medical device
- Save stills as PNG, TIFF and CSV, and record animations
- Live streaming over USB or Bluetooth to a browser viewer

## Wiring

- VIN to pin 9 (3.3 V). Do not use pin 1 (5 V)
- GND to pin 18
- SDA to pin 15 (C1)
- SCL to pin 16 (C0)

If you get I2C errors, set I2C Speed to 100 kHz in Settings.

## Controls

- **OK**: save a still. In USB and Bluetooth modes, start or stop streaming
- **Hold OK**: record while held
- **Left / Right**: change view mode
- **Up / Down**: adjust the current mode's setting
- **Back**: settings
- **Hold Back**: exit

## Saved files

Files are saved on the SD card in the thermal_cam folder.

## Disclaimer

Human mode gives a rough estimate only and can be wrong by a degree or more. It is not a medical device.

The browser viewer is in the tools folder of the project repository on GitHub.
