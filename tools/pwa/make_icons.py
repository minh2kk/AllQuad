#!/usr/bin/env python3
"""Draws the app icons (tools/pwa/icon-192.png, icon-512.png): the "AQ" badge of the configurator's
sidebar on a full-bleed gradient, letters inside the 80 % safe zone so Android can mask them (maskable).
Run once after changing the look; the PNGs are committed. Needs Pillow."""
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONTS = ["/usr/share/fonts/truetype/google-fonts/Poppins-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]


def icon(size):
    big = size * 4  # draw large, scale down: smooth edges
    a, b = (0x1E, 0x66, 0xD0), (0x17, 0x37, 0x5C)
    img = Image.new("RGB", (big, big))
    px = img.load()
    for y in range(big):
        for x in range(big):
            t = (x + y) / (2 * big - 2)  # 135 deg gradient as in the sidebar badge
            px[x, y] = tuple(round(a[i] + (b[i] - a[i]) * t) for i in range(3))
    d = ImageDraw.Draw(img)
    font = next((ImageFont.truetype(f, int(big * 0.36)) for f in FONTS if os.path.exists(f)), ImageFont.load_default())
    x0, y0, x1, y1 = d.textbbox((0, 0), "AQ", font=font)
    d.text(((big - (x1 - x0)) / 2 - x0, (big - (y1 - y0)) / 2 - y0), "AQ", font=font, fill=(255, 255, 255))
    # orange underline: the nose colour of the 3D model
    w = big * 0.3
    d.rounded_rectangle([big / 2 - w / 2, big * 0.66, big / 2 + w / 2, big * 0.66 + big * 0.035], radius=big * 0.02, fill=(0xE8, 0x74, 0x2A))
    return img.resize((size, size), Image.LANCZOS)


if __name__ == "__main__":
    for s in (192, 512):
        icon(s).save(os.path.join(HERE, "icon-%d.png" % s), optimize=True)
        print("icon-%d.png" % s)
