#!/usr/bin/env python3
"""Generate meshes/agos_pipe.stl, the pipe used by worlds/agos_pipe.sdf.

The centreline is the one in Designs/STL/PipeDesign_AGOS.stl (Fusion export, mm):
straight runs and 609.6 mm-radius bends lying flat at z = 0, open at both ends.
Every vertex of that STL sits exactly 200 mm or 300 mm from this centreline,
i.e. the CAD pipe is 600 mm OD with a 100 mm wall *inward* (400 mm bore),
which is too small for the 381 x 476 mm robot. This script keeps the same path
but lets you choose the bore and wall, and writes the mesh in metres.

Only needs numpy:
    python3 tools/make_pipe_mesh.py                     # 600 mm bore, 100 mm wall
    python3 tools/make_pipe_mesh.py --bore 0.5 --wall 0.02
Then rebuild (colcon build) so the new mesh is installed.
If you change the wall, update the pipe <pose> z in worlds/agos_pipe.sdf to
bore/2 + wall so the pipe still rests on the ground.
"""
import argparse
import os
import struct

import numpy as np

BEND_R = 0.6096  # centreline bend radius (m)

# Centreline in the CAD frame (m). ('line', start, end) or
# ('arc', centre, radius, start_deg, end_deg). Runs from the open end at
# (-1.2192, -1.2192) to the open end at (-0.6096, -2.4384).
SEGMENTS = [
    ('line', (-1.2192, -1.2192), (-1.2192, -0.6096)),
    ('arc', (-0.6096, -0.6096), BEND_R, 180.0, 90.0),
    ('line', (-0.6096, 0.0), (0.6096, 0.0)),
    ('arc', (0.6096, -0.6096), BEND_R, 90.0, -90.0),
    ('line', (0.6096, -1.2192), (0.0, -1.2192)),
    ('arc', (0.0, -1.8288), BEND_R, 90.0, 180.0),
    ('line', (-0.6096, -1.8288), (-0.6096, -2.4384)),
]


def centreline(arc_steps_per_90deg):
    """Return points (N,2) and unit tangents (N,2) along the centreline."""
    pts, tans = [], []
    for seg in SEGMENTS:
        if seg[0] == 'line':
            a, b = np.array(seg[1]), np.array(seg[2])
            t = (b - a) / np.linalg.norm(b - a)
            p = np.array([a, b])
            tt = np.array([t, t])
        else:
            c, r, a0, a1 = np.array(seg[1]), seg[2], np.radians(seg[3]), np.radians(seg[4])
            n = max(2, int(round(abs(a1 - a0) / (np.pi / 2) * arc_steps_per_90deg)) + 1)
            ang = np.linspace(a0, a1, n)
            p = c + r * np.c_[np.cos(ang), np.sin(ang)]
            sgn = np.sign(a1 - a0)
            tt = sgn * np.c_[-np.sin(ang), np.cos(ang)]
        if pts:  # drop the duplicated joint point
            p, tt = p[1:], tt[1:]
        pts.append(p)
        tans.append(tt)
    return np.concatenate(pts), np.concatenate(tans)


def rings(points, tangents, radius, n_around):
    """Circle of n_around points around each centreline point, in its normal plane."""
    theta = np.linspace(0.0, 2 * np.pi, n_around, endpoint=False)
    up = np.array([0.0, 0.0, 1.0])
    side = np.c_[-tangents[:, 1], tangents[:, 0], np.zeros(len(tangents))]  # z x t
    c3 = np.c_[points, np.zeros(len(points))]
    ring = (c3[:, None, :]
            + radius * np.cos(theta)[None, :, None] * side[:, None, :]
            + radius * np.sin(theta)[None, :, None] * up[None, None, :])
    return ring  # (n_rings, n_around, 3)


def tube_faces(n_rings, n_around, offset, flip):
    f = []
    for i in range(n_rings - 1):
        for j in range(n_around):
            a = offset + i * n_around + j
            b = offset + i * n_around + (j + 1) % n_around
            c = offset + (i + 1) * n_around + j
            d = offset + (i + 1) * n_around + (j + 1) % n_around
            f += [(a, c, b), (b, c, d)] if not flip else [(a, b, c), (b, d, c)]
    return f


def build(bore, wall, n_around, arc_steps):
    pts, tans = centreline(arc_steps)
    r_in, r_out = bore / 2.0, bore / 2.0 + wall
    outer = rings(pts, tans, r_out, n_around)
    inner = rings(pts, tans, r_in, n_around)
    n_r = len(pts)
    verts = np.concatenate([outer.reshape(-1, 3), inner.reshape(-1, 3)])
    n_tube = n_r * n_around
    faces = tube_faces(n_r, n_around, 0, flip=False)
    faces += tube_faces(n_r, n_around, n_tube, flip=True)
    n_wall = len(faces)
    # annular end caps
    for i in (0, n_r - 1):
        for j in range(n_around):
            o0 = i * n_around + j
            o1 = i * n_around + (j + 1) % n_around
            i0 = n_tube + o0
            i1 = n_tube + o1
            faces += [(o0, o1, i0), (o1, i1, i0)]
    faces = np.array(faces, dtype=np.int64)

    # Point every face out of the pipe material: away from the centreline on
    # the outer wall, towards it on the bore, and along the axis on the end caps.
    # Each vertex knows its ring, so the radial direction is exact.
    ring_of = (np.arange(len(verts)) % n_tube) // n_around
    centre3 = np.c_[pts, np.zeros(len(pts))]
    radial = (verts - centre3[ring_of])[faces].mean(axis=1)
    is_inner = faces[:, 0] >= n_tube
    want = np.where(is_inner[:, None], -radial, radial)
    cap_ring = ring_of[faces[n_wall:, 0]]
    cap_sign = np.where(cap_ring == 0, -1.0, 1.0)
    want[n_wall:] = cap_sign[:, None] * np.c_[tans[cap_ring], np.zeros(len(cap_ring))]
    tri = verts[faces]
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    flip = np.einsum('ij,ij->i', normal, want) < 0
    faces[flip] = faces[flip][:, ::-1]
    return verts, faces


def write_stl(path, verts, faces):
    tri = verts[faces].astype(np.float32)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    rec = np.zeros(len(faces), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'] = n
    rec['v'] = tri
    with open(path, 'wb') as fh:
        fh.write(b'AGOS pipe (metres), generated by tools/make_pipe_mesh.py'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(faces)))
        rec.tofile(fh)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bore', type=float, default=0.6, help='inside diameter in m (default 0.6)')
    ap.add_argument('--wall', type=float, default=0.1, help='wall thickness in m (default 0.1)')
    ap.add_argument('--around', type=int, default=64, help='segments around the pipe')
    ap.add_argument('--arc-steps', type=int, default=32, help='segments per 90 deg of bend')
    ap.add_argument('-o', '--output', default=os.path.join(here, '..', 'meshes', 'agos_pipe.stl'))
    args = ap.parse_args()
    verts, faces = build(args.bore, args.wall, args.around, args.arc_steps)
    write_stl(args.output, verts, faces)
    print('wrote %s: %d triangles, bore %.3f m, wall %.3f m'
          % (os.path.normpath(args.output), len(faces), args.bore, args.wall))


if __name__ == '__main__':
    main()
