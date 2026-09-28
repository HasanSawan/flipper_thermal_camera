#!/usr/bin/env python3
"""
Convert a RAW animation captured by the Flipper thermal camera app into PNGs
and (optionally) an animated GIF.

The app writes RAW takes as `REC_<timestamp>/frames.bin`: a flat sequence of
frames, each 768 little-endian float32 values in degrees Celsius, row-major,
32 wide by 24 high. There is no header -- the frame count is the file size
divided by 3072.

Usage:
    python3 raw2png.py frames.bin                  # PNGs at 8x, ironbow
    python3 raw2png.py frames.bin --gif out.gif    # also build a GIF
    python3 raw2png.py frames.bin --palette rainbow --scale 4
    python3 raw2png.py frames.bin --range 20 40    # fixed scale across frames

Requires Pillow:  pip install --user Pillow
"""

import argparse
import os
import struct
import sys

W, H = 32, 24
FRAME_VALUES = W * H
FRAME_BYTES = FRAME_VALUES * 4

# Gradient stops matching the palettes in img_export.c, so PC-side renders
# look the same as the ones the Flipper writes itself.
PALETTES = {
    "ironbow": [(0, (0, 0, 0)), (51, (40, 0, 90)), (102, (130, 0, 120)),
                (153, (220, 60, 40)), (204, (255, 170, 0)), (255, (255, 255, 255))],
    "rainbow": [(0, (0, 0, 131)), (32, (0, 60, 170)), (96, (5, 255, 255)),
                (160, (255, 255, 0)), (223, (250, 0, 0)), (255, (128, 0, 0))],
    "gray": [(0, (0, 0, 0)), (255, (255, 255, 255))],
    "hotmetal": [(0, (0, 0, 0)), (84, (180, 0, 0)), (168, (255, 180, 0)),
                 (255, (255, 255, 255))],
    "arctic": [(0, (0, 0, 40)), (64, (0, 110, 200)), (128, (120, 220, 255)),
               (192, (255, 240, 180)), (255, (255, 255, 255))],
}


def build_lut(stops):
    lut = []
    for level in range(256):
        if level <= stops[0][0]:
            lut.append(stops[0][1])
            continue
        placed = False
        for i in range(1, len(stops)):
            if level <= stops[i][0]:
                p0, c0 = stops[i - 1]
                p1, c1 = stops[i]
                span = p1 - p0
                t = ((level - p0) / span) if span else 0.0
                lut.append(tuple(int(c0[k] + (c1[k] - c0[k]) * t) for k in range(3)))
                placed = True
                break
        if not placed:
            lut.append(stops[-1][1])
    return lut


def read_frames(path):
    size = os.path.getsize(path)
    count = size // FRAME_BYTES
    if count == 0:
        sys.exit(f"{path}: too small to contain a frame ({size} bytes)")
    if size % FRAME_BYTES:
        print(f"warning: {size % FRAME_BYTES} trailing bytes ignored "
              f"(capture may have been cut short)", file=sys.stderr)

    with open(path, "rb") as fh:
        for _ in range(count):
            chunk = fh.read(FRAME_BYTES)
            if len(chunk) < FRAME_BYTES:
                return
            yield struct.unpack("<768f", chunk)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="frames.bin from a REC_* folder")
    ap.add_argument("--outdir", default=None, help="output folder (default: alongside input)")
    ap.add_argument("--palette", default="ironbow", choices=sorted(PALETTES))
    ap.add_argument("--scale", type=int, default=8, help="pixel scale factor (default 8)")
    ap.add_argument("--range", nargs=2, type=float, metavar=("MIN", "MAX"),
                    help="fixed temperature range; default auto-scales per frame")
    ap.add_argument("--gif", metavar="PATH", help="also write an animated GIF")
    ap.add_argument("--fps", type=float, default=8.0, help="GIF frame rate (default 8)")
    ap.add_argument("--no-png", action="store_true", help="skip per-frame PNGs")
    args = ap.parse_args()

    try:
        from PIL import Image
    except ImportError:
        sys.exit("Pillow is required:  pip install --user Pillow")

    outdir = args.outdir or os.path.join(os.path.dirname(os.path.abspath(args.input)), "png")
    if not args.no_png:
        os.makedirs(outdir, exist_ok=True)

    lut = build_lut(PALETTES[args.palette])
    frames = list(read_frames(args.input))
    if not frames:
        sys.exit("no complete frames found")

    if args.range:
        lo_fixed, hi_fixed = args.range
    else:
        lo_fixed = hi_fixed = None

    images = []
    global_lo = min(min(f) for f in frames)
    global_hi = max(max(f) for f in frames)

    for i, temps in enumerate(frames):
        lo = lo_fixed if lo_fixed is not None else min(temps)
        hi = hi_fixed if hi_fixed is not None else max(temps)
        span = (hi - lo) or 1.0

        img = Image.new("RGB", (W, H))
        px = img.load()
        for y in range(H):
            for x in range(W):
                n = (temps[y * W + x] - lo) / span
                n = 0.0 if n < 0 else (1.0 if n > 1 else n)
                px[x, y] = lut[int(n * 255 + 0.5)]

        if args.scale > 1:
            img = img.resize((W * args.scale, H * args.scale), Image.LANCZOS)

        if not args.no_png:
            img.save(os.path.join(outdir, f"frame_{i:05d}.png"))
        if args.gif:
            images.append(img)

    print(f"{len(frames)} frames   range {global_lo:.1f}..{global_hi:.1f} C")
    if not args.no_png:
        print(f"PNGs -> {outdir}")

    if args.gif and images:
        images[0].save(args.gif, save_all=True, append_images=images[1:],
                       duration=int(1000 / args.fps), loop=0)
        print(f"GIF  -> {args.gif}")


if __name__ == "__main__":
    main()
