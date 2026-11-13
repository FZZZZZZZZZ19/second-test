"""Generate the embedded ASCII bold sans bitmap; requires Pillow and --font."""
import argparse
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser()
p.add_argument('--font', required=True, help='DejaVuSansMono-Bold.ttf')
a = p.parse_args()
font = ImageFont.truetype(a.font, 16)
rows = []
for code in range(32, 127):
    tile = Image.new('L', (10, 16))
    ImageDraw.Draw(tile).text((0, -3), chr(code), font=font, fill=255)
    tile = tile.resize((8, 16), Image.Resampling.LANCZOS)
    rows.append([sum((1 << (7-x)) for x in range(8) if tile.getpixel((x,y)) >= 100) for y in range(16)])
out = Path(__file__).resolve().parents[1] / 'User/integration/menu_font_data.h'
out.write_text('/* DejaVu Sans Mono Bold raster; see LICENSES/DejaVu-font.txt. */\n'
               'static const unsigned char menu_glyphs[95][16] = {\n' +
               ''.join('    {' + ','.join('0x%02x' % b for b in row) + '},\n' for row in rows) + '};\n', encoding='ascii')
