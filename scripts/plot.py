"""Draws docs/coverage-rect.png from the curve CSVs written by `poolsim curve`.
Needs Pillow. Usage: python3 scripts/plot.py"""
import csv
from PIL import Image, ImageDraw, ImageFont

W, H, L, R, T, B = 900, 460, 70, 24, 40, 56
rows = {}
for name in ("random", "lanes"):
    with open(f"docs/data/curve-rect-{name}.csv") as f:
        rows[name] = [(int(r["minute"]), float(r["p10"]), float(r["median"]), float(r["p90"])) for r in csv.DictReader(f)]

im = Image.new("RGB", (W, H), "#0f1418")
d = ImageDraw.Draw(im)
font = ImageFont.load_default()
x = lambda m: L + (W - L - R) * m / 60
y = lambda c: T + (H - T - B) * (1 - c)

for c in (0, 0.25, 0.5, 0.75, 0.9, 1.0):
    d.line([(L, y(c)), (W - R, y(c))], fill="#2a343d")
    d.text((L - 40, y(c) - 5), f"{int(c * 100)}%", fill="#8a99a6", font=font)
for m in range(0, 61, 10):
    d.text((x(m) - 6, H - B + 10), f"{m}", fill="#8a99a6", font=font)
d.text((W // 2 - 80, H - 24), "minutes since the robot was dropped in", fill="#8a99a6", font=font)
d.text((L, 14), "8 x 4 m pool, 100 seeds: median coverage, band is the 10th to 90th percentile", fill="#dbe4ea", font=font)

colors = {"random": "#d98b4a", "lanes": "#3fb6a8"}
for name, pts in rows.items():
    col = colors[name]
    band = [(x(m), y(hi)) for m, lo, med, hi in pts] + [(x(m), y(lo)) for m, lo, med, hi in reversed(pts)]
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    ImageDraw.Draw(ov).polygon(band, fill=col + "40")
    im.paste(ov, (0, 0), ov)
    d = ImageDraw.Draw(im)
    d.line([(x(m), y(med)) for m, lo, med, hi in pts], fill=col, width=3)
d.text((x(40), y(0.62)), "bounce off walls", fill=colors["random"], font=font)
d.text((x(12), y(0.97) - 18), "poolnav lanes", fill=colors["lanes"], font=font)
d.line([(L, T), (L, H - B), (W - R, H - B)], fill="#8a99a6")
im.save("docs/coverage-rect.png")
