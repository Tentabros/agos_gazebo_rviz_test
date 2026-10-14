#!/usr/bin/env python3
"""
Generate the concrete textures used by the pipe worlds.

Writes four images into materials/textures/:
  pipe_concrete.jpg           colour of the pipe's inside (and outside)
  pipe_concrete_normal.jpg    bumps: concrete grain, cracks, the joint grooves
  chamber_concrete.jpg        colour of the manhole chambers
  chamber_concrete_normal.jpg

The pipe texture is wrapped around the bore by tools/make_pipe_mesh.py:
  across the image (u) = once around the pipe; the middle column (u = 0.5) is
                         the bottom of the pipe (the invert) and the edges are
                         the crown,
  down the image (v)   = PIPE_LENGTH_M of pipe, then it repeats.
So the bottom of the image carries the sediment and the dried flow line, and
the image has a joint every JOINT_SPACING_M, like 2.4 m concrete pipe sections.
Every section gets its own shade, stains and cracks, so the camera (and
RTAB-Map) sees a different wall every few metres instead of one repeated
pattern.

Only needs numpy and Pillow (pip install pillow). The output is deterministic
(fixed --seed):
    python3 tools/make_pipe_texture.py
Then rebuild (colcon build) so the new images are installed.
"""
import argparse
import os

import numpy as np
from PIL import Image

PIPE_LENGTH_M = 19.2        # pipe length covered by one copy of the texture
JOINT_SPACING_M = 2.4       # concrete pipe section length
PIPE_SIZE = (1024, 8192)    # (around, along) pixels: ~2.8 x 2.3 mm each on a 900 mm bore
CHAMBER_SIZE = (2048, 2048)  # covers CHAMBER_TILE_M x CHAMBER_TILE_M of wall
CHAMBER_TILE_M = 4.0

CONCRETE = np.array([0.60, 0.58, 0.54])
SEDIMENT = np.array([0.33, 0.27, 0.19])
BIOFILM = np.array([0.30, 0.31, 0.20])
JOINT = np.array([0.16, 0.15, 0.13])


def fbm(rng, shape, beta, lowest=1):
    """
    Tileable fractal noise (mean 0, std 1): white noise shaped by 1/f^beta.

    lowest: drop wavelengths longer than shape/lowest (keeps big blotches out
    of fine detail layers).
    """
    h, w = shape
    fy = np.fft.fftfreq(h)[:, None] * h
    fx = np.fft.rfftfreq(w)[None, :] * w
    f = np.sqrt((fx * h / w) ** 2 + fy ** 2)  # cycles per image height, isotropic in pixels
    f[0, 0] = 1.0
    amp = f ** (-beta / 2.0)
    amp[f < lowest] = 0.0
    spec = np.fft.rfft2(rng.standard_normal(shape)) * amp
    out = np.fft.irfft2(spec, s=shape)
    return (out - out.mean()) / (out.std() + 1e-12)


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def wrap_dist(a, b, period):
    d = np.abs(a - b) % period
    return np.minimum(d, period - d)


def draw_cracks(rng, height, colour_mask, shape, count, step_px, length_steps, prefer_axis):
    """Random-walk hairline cracks. Draws into colour_mask (darkening) and height."""
    h, w = shape
    for _ in range(count):
        y, x = rng.uniform(0, h), rng.uniform(0, w)
        heading = rng.uniform(0, 2 * np.pi)
        if prefer_axis is not None:  # mostly around the pipe, like real ring cracks
            heading = prefer_axis + rng.normal(0, 0.35) + rng.choice([0, np.pi])
        width = rng.uniform(0.8, 1.6)
        for _ in range(int(length_steps * rng.uniform(0.4, 1.0))):
            heading += rng.normal(0, 0.35)
            y = (y + step_px * np.sin(heading)) % h
            x = (x + step_px * np.cos(heading)) % w
            r = int(np.ceil(width)) + 1
            ys = (np.arange(int(y) - r, int(y) + r + 1)) % h
            xs = (np.arange(int(x) - r, int(x) + r + 1)) % w
            dy = wrap_dist(ys[:, None], y, h)
            dx = wrap_dist(xs[None, :], x, w)
            ink = np.clip(1.0 - np.sqrt(dx ** 2 + dy ** 2) / width, 0, 1)
            colour_mask[np.ix_(ys, xs)] = np.maximum(colour_mask[np.ix_(ys, xs)], ink)
            height[np.ix_(ys, xs)] -= 0.6 * ink


def normal_map(height, strength):
    """Tangent-space normal map (RGB, 0..1) from a tileable height field."""
    gy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
    gx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
    n = np.dstack([-gx * strength, gy * strength, np.ones_like(height)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return n * 0.5 + 0.5


def concrete_base(rng, shape):
    """Grey concrete: broad shading, mottling, sand grain and aggregate."""
    broad = fbm(rng, shape, 3.2)
    mottle = fbm(rng, shape, 2.0, lowest=8)
    grain = rng.standard_normal(shape)
    agg = fbm(rng, shape, 0.6, lowest=64)
    shade = 1.0 + 0.07 * broad + 0.05 * mottle + 0.035 * grain
    shade -= 0.10 * smoothstep(2.4, 3.2, agg)   # dark aggregate stones
    shade += 0.07 * smoothstep(2.6, 3.4, -agg)  # light aggregate stones
    height = 0.6 * mottle + 0.35 * grain + 0.8 * smoothstep(2.2, 3.2, np.abs(agg))
    return shade, height


def pipe_texture(rng, shape, metres_per_px_along):
    w, h = shape  # w = around, h = along
    shape = (h, w)
    shade, height = concrete_base(rng, shape)
    u = (np.arange(w) + 0.5) / w           # 0 = crown, 0.5 = invert
    v_m = (np.arange(h) + 0.5) * metres_per_px_along
    from_invert = np.abs(u - 0.5)[None, :]  # 0 at the bottom, 0.5 at the crown

    colour = shade[..., None] * CONCRETE

    # Each 2.4 m section is a slightly different pour: shade and tint.
    sections = int(round(PIPE_LENGTH_M / JOINT_SPACING_M))
    section_of_row = np.minimum((v_m / JOINT_SPACING_M).astype(int), sections - 1)
    tint = 1.0 + rng.normal(0, 0.010, (sections, 3)) + rng.normal(0, 0.06, (sections, 1))
    colour *= tint[section_of_row][:, None, :]

    # Sediment and flow staining along the bottom (about +-30 deg), ragged
    # edge, thicker in some places than others.
    edge_noise = fbm(rng, shape, 2.6) * 0.02
    band = 1.0 - smoothstep(0.06, 0.095, from_invert + edge_noise)
    band *= 0.55 + 0.25 * np.clip(fbm(rng, shape, 3.0), -1, 1)
    streak = fbm(rng, (h, w), 1.6, lowest=4)
    sediment = SEDIMENT * (1.0 + 0.12 * streak)[..., None]
    colour = colour * (1 - band[..., None]) + sediment * band[..., None]
    height += band * 0.4 * streak

    # Dried flow line (tide mark) a little above the sediment, and biofilm patches.
    level = 0.115 + 0.010 * fbm(rng, (h, 1), 3.0)
    tide = np.exp(-((from_invert - level) / 0.006) ** 2)
    colour *= (1 - 0.35 * tide)[..., None]
    film = smoothstep(0.6, 1.6, fbm(rng, shape, 2.8)) * (1 - smoothstep(0.12, 0.2, from_invert))
    colour = colour * (1 - 0.5 * film[..., None]) + BIOFILM * 0.5 * film[..., None]

    # Run-down stains from the crown towards the bottom on both sides.
    drip_rows = fbm(rng, (h, 1), 2.4, lowest=6)
    drips = smoothstep(1.0, 2.0, drip_rows) * (1 - smoothstep(0.30, 0.48, from_invert))
    drips *= smoothstep(-0.5, 1.0, fbm(rng, shape, 2.2))
    colour *= (1 - 0.15 * drips)[..., None]

    # Cracks, mostly running around the pipe.
    crack = np.zeros(shape)
    draw_cracks(rng, height, crack, shape, count=int(4 * sections), step_px=3.0,
                length_steps=220, prefer_axis=0.0)
    colour *= (1 - 0.55 * crack)[..., None]

    # Joints between sections: a dark gap with a mortar collar either side.
    for k in range(sections):
        d = wrap_dist(v_m, k * JOINT_SPACING_M, PIPE_LENGTH_M)[:, None]
        gap = 1.0 - smoothstep(0.006, 0.012, d)
        collar = (1.0 - smoothstep(0.02, 0.05, d)) * (1 - gap)
        colour = colour * (1 - gap[..., None]) + JOINT * gap[..., None]
        colour *= (1 + 0.08 * collar)[..., None]
        height -= 3.0 * gap
        height += 0.5 * collar
    return np.clip(colour, 0, 1), height


def chamber_texture(rng, shape):
    shape = (shape[1], shape[0])
    shade, height = concrete_base(rng, shape)
    colour = shade[..., None] * (CONCRETE * np.array([1.0, 1.0, 0.97]))
    stains = smoothstep(0.8, 2.0, fbm(rng, shape, 3.0))
    colour = colour * (1 - 0.35 * stains[..., None]) + SEDIMENT * 0.35 * stains[..., None]
    crack = np.zeros(shape)
    draw_cracks(rng, height, crack, shape, count=10, step_px=3.0, length_steps=300,
                prefer_axis=None)
    colour *= (1 - 0.5 * crack)[..., None]
    return np.clip(colour, 0, 1), height


def save_rgb(path, rgb, quality):
    img = Image.fromarray((rgb * 255 + 0.5).astype(np.uint8), 'RGB')
    img.save(path, quality=quality, subsampling=0 if quality >= 90 else 2, optimize=True)
    size = os.path.getsize(path) / 1e6
    print('wrote %s (%dx%d, %.1f MB)' % (os.path.normpath(path), img.width, img.height, size))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--output-dir',
                    default=os.path.join(here, '..', 'materials', 'textures'))
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    colour, height = pipe_texture(rng, PIPE_SIZE, PIPE_LENGTH_M / PIPE_SIZE[1])
    save_rgb(os.path.join(args.output_dir, 'pipe_concrete.jpg'), colour, 88)
    save_rgb(os.path.join(args.output_dir, 'pipe_concrete_normal.jpg'),
             normal_map(height, 0.35), 90)

    colour, height = chamber_texture(rng, CHAMBER_SIZE)
    save_rgb(os.path.join(args.output_dir, 'chamber_concrete.jpg'), colour, 88)
    save_rgb(os.path.join(args.output_dir, 'chamber_concrete_normal.jpg'),
             normal_map(height, 0.35), 90)


if __name__ == '__main__':
    main()
