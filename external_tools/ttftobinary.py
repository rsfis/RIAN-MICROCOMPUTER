from PIL import Image, ImageFont, ImageDraw
import struct
import math

FONT_PATH = "American Western.ttf"
SIZE = 32

CHARS = [chr(i) for i in range(32, 127)]

font = ImageFont.truetype(FONT_PATH, SIZE)
ascent, descent = font.getmetrics()

glyph_data = []

max_w = 0
max_h = 0

for ch in CHARS:
    bbox = font.getbbox(ch)
    if bbox is None:
        continue

    x0, y0, x1, y1 = bbox
    w = x1 - x0
    h = y1 - y0

    advance = int(font.getlength(ch))

    glyph_data.append({
        "code": ord(ch),
        "w": w,
        "h": h,
        "xOff": x0,
        "yOff": y0,
        "advance": advance
    })

    max_w = max(max_w, w)
    max_h = max(max_h, h)

cols = 16
rows = math.ceil(len(glyph_data) / cols)

cell_w = max_w + 4
cell_h = max_h + 4

atlas_w = cols * cell_w
atlas_h = rows * cell_h

atlas = Image.new("L", (atlas_w, atlas_h), 0)
draw = ImageDraw.Draw(atlas)

for i, g in enumerate(glyph_data):
    col = i % cols
    row = i // cols

    x = col * cell_w
    y = row * cell_h

    draw.text(
        (x - g["xOff"], y - g["yOff"]),
        chr(g["code"]),
        font=font,
        fill=255
    )

    g["atlas_x"] = x
    g["atlas_y"] = y

atlas.save("atlas_debug.png")

with open("font.bin", "wb") as f:

    f.write(b"FNT1")
    f.write(struct.pack("<HHHHH",
                        SIZE,
                        ascent,
                        len(glyph_data),
                        atlas_w,
                        atlas_h))

    for g in glyph_data:
        f.write(struct.pack("<IHHHHhhH",
                            g["code"],
                            g["atlas_x"],
                            g["atlas_y"],
                            g["w"],
                            g["h"],
                            g["xOff"],
                            g["yOff"],
                            g["advance"]))

    bitmap_bytes = atlas.tobytes()
    f.write(struct.pack("<I", len(bitmap_bytes)))
    f.write(bitmap_bytes)

print("font.bin gerado com sucesso.")