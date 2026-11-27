"""Draw the 1200x627 link-preview card for the docs (docs/source/_static/social_card.png).

Usage: python3 tools/make_social_card.py docs/source/_static/social_card.png
Needs Pillow and the macOS system fonts (Helvetica Neue, Menlo); change HN and MONO to use other fonts.
"""
import sys
from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 627
S = 3  # supersample, then downscale for smooth text
img = Image.new("RGB", (W * S, H * S), "#2a1250")
d = ImageDraw.Draw(img)

HN = "/System/Library/Fonts/HelveticaNeue.ttc"
MONO = "/System/Library/Fonts/Menlo.ttc"


def font(path, size, index=0):
    return ImageFont.truetype(path, size * S, index=index)


# HelveticaNeue.ttc: 0 regular, 1 bold, 4 medium (indices vary; check names)
def hn(size, want):
    for i in range(12):
        try:
            f = ImageFont.truetype(HN, size * S, index=i)
        except OSError:
            break
        if f.getname()[1].lower() == want:
            return f
    return ImageFont.truetype(HN, size * S, index=0)


kicker = hn(22, "medium")
title = hn(80, "bold")
sub = hn(29, "regular")
tagf = hn(20, "medium")
foot = hn(20, "regular")
mono = font(MONO, 19)


def P(x, y):
    return (x * S, y * S)


# Left column
x0 = 72
d.text(P(x0, 64), "Taught MSc Level Open Course Material", font=hn(24, "medium"), fill="#c6a3f7")
owner = hn(48, "bold")
d.text(P(x0, 104), "(Murilo’s)", font=owner, fill="#c6a3f7")
# Largest size up to 80 at which the main title fits before the terminal window.
size = 80
while d.textlength("ROS2 Tutorial", font=hn(size, "bold")) / S > 590:
    size -= 2
d.text(P(x0, 166 + (80 - size) // 2), "ROS2 Tutorial", font=hn(size, "bold"), fill="#f4eefb")
print("title size", size)
SUBTITLE = ("Hands-on tutorial on software for robotics: making and understanding "
            "custom nodes for physical and simulated robots.")
sub = hn(27, "regular")
lines, cur = [], ""
for word in SUBTITLE.split():
    trial = (cur + " " + word).strip()
    if d.textlength(trial, font=sub) / S > 570 and cur:
        lines.append(cur)
        cur = word
    else:
        cur = trial
lines.append(cur)
for i, line in enumerate(lines):
    d.text(P(x0, 282 + i * 37), line, font=sub, fill="#d8cce9")
print("subtitle lines", len(lines))

# Tags
tx, ty = x0, 452
for t in ["Python/C++", "Topics", "Services", "Actions", "TF2", "Gazebo", "Nav2", "Cybersecurity"]:
    w = d.textlength(t, font=tagf) / S
    if tx + w + 28 > 640:
        tx, ty = x0, ty + 48
    d.rounded_rectangle([*P(tx, ty), *P(tx + w + 28, ty + 38)], radius=8 * S, fill="#46297a")
    d.text(P(tx + 14, ty + 8), t, font=tagf, fill="#ebdffc")
    tx += w + 28 + 10

d.text(P(x0, 566), "Murilo M. Marinho  ·  The University of Manchester  ·  ros2-tutorial.readthedocs.io",
       font=foot, fill="#ab97c8")

# Terminal window on the right
mono = font(MONO, 15)
LH = 30
N_LINES = 9
tx0, tx1 = 668, 1144
box_h = 38 + 20 + N_LINES * LH + 12
ty0 = round(64 + (540 - 64 - box_h) / 2)
ty1 = ty0 + box_h
d.rounded_rectangle([*P(tx0 + 6, ty0 + 14), *P(tx1 + 6, ty1 + 14)], radius=14 * S, fill="#1a0b33")
d.rounded_rectangle([*P(tx0, ty0), *P(tx1, ty1)], radius=14 * S, fill="#130a21", outline="#4b3373", width=S)
d.rounded_rectangle([*P(tx0, ty0), *P(tx1, ty0 + 38)], radius=14 * S, fill="#2c1b48")
d.rectangle([*P(tx0, ty0 + 24), *P(tx1, ty0 + 38)], fill="#2c1b48")
for i in range(3):
    cx = tx0 + 22 + i * 22
    d.ellipse([*P(cx - 6.5, ty0 + 12.5), *P(cx + 6.5, ty0 + 25.5)], fill="#5d4486")

GREEN, GREY, AMBER, TXT = "#6fd08c", "#8f7cae", "#f2b866", "#e9def5"
lines = [
    [("# get the code", GREY)],
    [("$ ", GREEN), ("git clone \\", TXT)],
    [("    https://github.com/", TXT), ("mmmarinho/ROS2_Tutorial", AMBER)],
    [("$ ", GREEN), ("cd ROS2_Tutorial/ros2_tutorial_workspace", TXT)],
    [("# build and run", GREY)],
    [("$ ", GREEN), ("colcon build", TXT)],
    [("$ ", GREEN), ("source install/setup.bash", TXT)],
    [("$ ", GREEN), ("ros2 run ", TXT), ("python_package_with_a_node", AMBER), (" \\", TXT)],
    [("    sample_python_node", AMBER)],
]
assert len(lines) == N_LINES, len(lines)
ly = ty0 + 38 + 20
widest = 0
for parts in lines:
    lx = tx0 + 22
    for text, col in parts:
        d.text(P(lx, ly), text, font=mono, fill=col)
        lx += d.textlength(text, font=mono) / S
    widest = max(widest, lx)
    ly += LH
print("terminal text right edge", round(widest), "box right", tx1)

img = img.resize((W, H), Image.LANCZOS)
img.save(sys.argv[1], optimize=True)
print("wrote", sys.argv[1])
