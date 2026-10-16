#!/usr/bin/env python3
"""
Generate the pipe meshes used by the pipe worlds.

Three layouts, all with a 900 mm bore (the sewer pipe size AGOS is built for):

  network -> meshes/agos_network_pipes.{obj,stl} and agos_network_chambers.{obj,stl},
             used by worlds/agos_sewer_network.sdf (the default world)
             A larger, twistier network: five manholes (M1-M5) in a 250 m loop
             with sweeping curves, S-bends, a zig-zag, a serpentine, doglegs,
             and four dead-end branches (a hairpin, a 135 deg bend, a 45 deg
             bend and a zig-zag). The robot starts in M1 with 50 m of pipe
             either side of it, so the proposal's mission (explore 25 m one way,
             come back, turn round, explore 25 m the other way) fits.
  sewer   -> meshes/agos_sewer_pipes.{obj,stl} and agos_sewer_chambers.{obj,stl},
             used by worlds/agos_sewer.sdf
             The smaller network: four manhole chambers joined in a loop of
             47 m of pipe (long straights, an S-bend made of two 45 deg
             bends) plus a 10 m dead-end branch with a 90 deg bend.
  cad     -> meshes/agos_pipe.{obj,stl}, used by worlds/agos_pipe.sdf
             The pipe in Designs/STL/PipeDesign_AGOS.stl, scaled up 2.25x so
             its 400 mm bore becomes 900 mm: same shape, same open ends, the
             straights 2.25x longer and the bends 1.37 m in radius.

The .obj files are what Gazebo draws. They carry texture coordinates for the
images from tools/make_pipe_texture.py: once around the pipe across the image
(bottom of the pipe = middle of the image) and TEXTURE_LENGTH_M of pipe down
it. The .stl files are for collisions: for the pipes only the bore (the robot
is always inside), with fewer facets around it, which keeps the physics fast.

Coordinates are metres. z = 0 is the ground: the pipes rest on it, so the
bottom inside the pipe (the invert) is at z = WALL and the pipe axis at
z = WALL + BORE / 2.

Only needs numpy:
    python3 tools/make_pipe_mesh.py              # all layouts
    python3 tools/make_pipe_mesh.py --layout network
    python3 tools/make_pipe_mesh.py --layout cad --bore 1.0
Then rebuild (colcon build) so the new meshes are installed. If you change
--bore or --wall, check spawn_z in launch/sim.launch.py (about WALL + 0.04).
"""
import argparse
import os
import struct

import numpy as np

BORE = 0.9              # inside diameter (m)
WALL = 0.1              # pipe wall (m)
CAD_SCALE = 2.25        # 400 mm CAD bore -> 900 mm
CAD_BEND_R = 0.6096     # bend radius in the CAD file (m), before scaling
TEXTURE_LENGTH_M = 19.2  # metres of pipe covered by one copy of the texture
CHAMBER_TEXTURE_M = 4.0  # metres of chamber wall covered by one copy of its texture
BEND_R = 1.35           # sewer layout: long-radius bends, 1.5 x the bore

# Manhole chambers in the sewer layout: square, inside size x height above the
# invert, wall and roof thickness, and an access shaft in the roof.
CHAMBER_SIZE = 2.4
CHAMBER_HEIGHT = 1.8
CHAMBER_WALL = 0.15
SHAFT_DIAMETER = 0.7
# The chamber floor sits this far above the pipe invert: level with where the
# tracks (0.12-0.16 m either side of the robot's centre) touch the curved
# bottom of a 900 mm pipe, so the robot rolls from pipe to chamber without
# a step. With the floor at the invert, the tracks hit a 30 mm lip.
FLOOR_LIFT = 0.024
# In front of each pipe the floor is shaped like the pipe for BENCH_LENGTH into
# the chamber: benching rises at the sides, up to BENCH_HEIGHT, and a shallow
# channel continues the bottom of the pipe in the middle (like the channel in
# a real manhole floor), both flattening out into the chamber floor. A robot
# that arrives off-centre rides up the side and slides back to the middle
# instead of hitting the edge of the opening, and one that reverses out of a
# pipe off-centre meets a ramp, not a 24 mm step its rear wheels cannot climb.
# Beside the opening the benching slopes back down to the floor over
# BENCH_FADE. It is kept low, under the robot's 78 mm belly clearance: 0.2 m
# benching faded over 0.25 m made a ridge beside each opening that caught the
# belly of a robot crossing the manhole 0.45 m off the middle of a pipe, and
# its tracks then spun without moving it, forwards or back.
BENCH_LENGTH = 0.6
BENCH_HEIGHT = 0.08
BENCH_FADE = 0.5

AROUND = 64             # segments around the bore in the visual mesh (what the camera sees)
AROUND_OUTSIDE = 24     # segments around the outside of the pipe (only seen from above)
AROUND_COLLISION = 32   # segments around the bore in the collision mesh
ARC_STEP_DEG = 3.0      # collision mesh: bend segment angle
LINE_STEP = 0.6         # collision mesh: straight segment length (short triangles collide better)
VISUAL_ARC_STEP_DEG = 6.0   # visual mesh: bend segment angle (smooth shading hides the facets)
VISUAL_LINE_STEP = 2.4  # visual mesh: straight segment length (straights look the same)


# ---------------------------------------------------------------- mesh builder

class Mesh:
    """Triangles with per-vertex normals and texture coordinates."""

    def __init__(self):
        self.v, self.n, self.uv, self.f = [], [], [], []
        self.count = 0

    def add(self, verts, normals, uvs, faces):
        verts = np.asarray(verts, float).reshape(-1, 3)
        self.v.append(verts)
        self.n.append(np.asarray(normals, float).reshape(-1, 3))
        self.uv.append(np.asarray(uvs, float).reshape(-1, 2))
        self.f.append(np.asarray(faces, np.int64).reshape(-1, 3) + self.count)
        self.count += len(verts)

    def grid(self, pts, normals, uvs):
        """Add a (rows, cols) grid of points as quads. Faces follow the normals."""
        rows, cols = pts.shape[:2]
        idx = np.arange(rows * cols).reshape(rows, cols)
        a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()
        c, d = idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
        faces = np.r_[np.c_[a, c, b], np.c_[b, c, d]]
        faces = orient(pts.reshape(-1, 3), normals.reshape(-1, 3), faces)
        self.add(pts, normals, uvs, faces)

    def arrays(self):
        return (np.concatenate(self.v), np.concatenate(self.n),
                np.concatenate(self.uv), np.concatenate(self.f))

    def write_obj(self, path, title):
        v, n, uv, f = self.arrays()
        f = drop_degenerate(v, f)
        name = os.path.splitext(os.path.basename(path))[0]
        with open(os.path.splitext(path)[0] + '.mtl', 'w') as fh:
            # Plain concrete grey for other viewers; Gazebo uses the textured
            # material in the world file instead.
            fh.write('newmtl concrete\nKa 1 1 1\nKd 0.6 0.58 0.54\nKs 0.05 0.05 0.05\n')
        with open(path, 'w') as fh:
            fh.write('# %s\n# generated by tools/make_pipe_mesh.py, metres\n' % title)
            fh.write('mtllib %s.mtl\no %s\nusemtl concrete\n' % (name, name))
            np.savetxt(fh, v, fmt='v %.5f %.5f %.5f')
            np.savetxt(fh, uv, fmt='vt %.5f %.5f')
            np.savetxt(fh, n, fmt='vn %.4f %.4f %.4f')
            g = f + 1
            np.savetxt(fh, np.repeat(g, 3, axis=1), fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')
        return len(f)

    def write_stl(self, path, title):
        v, _, _, f = self.arrays()
        f = drop_degenerate(v, f)
        tri = v[f].astype(np.float32)
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1), 1e-12)[:, None]
        rec = np.zeros(len(f), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        rec['n'], rec['v'] = nrm, tri
        with open(path, 'wb') as fh:
            fh.write(title.encode()[:80].ljust(80, b' '))
            fh.write(struct.pack('<I', len(f)))
            rec.tofile(fh)
        return len(f)


def orient(verts, normals, faces):
    """Flip faces whose winding disagrees with their vertex normals."""
    tri = verts[faces]
    geo = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    want = normals[faces].sum(axis=1)
    flip = np.einsum('ij,ij->i', geo, want) < 0
    faces = faces.copy()
    faces[flip] = faces[flip][:, ::-1]
    return faces


def drop_degenerate(v, f):
    tri = v[f]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    return f[area > 1e-10]


# ---------------------------------------------------------------- pipe runs

class Run:
    """
    A pipe centreline on the ground plane, built like a turtle.

    start: (x, y) of the run's start; heading: degrees, 0 = +x, 90 = +y.
    Then .straight(length) and .bend(degrees, radius) (positive = left).
    """

    def __init__(self, start, heading, v_offset=0.0):
        self.p = np.array(start, float)
        self.h = np.radians(heading)
        self.segments = []   # (kind, start point, start heading, length, radius, sign)
        self.v_offset = v_offset
        self.start_cap = self.end_cap = False

    def straight(self, length):
        self.segments.append(('line', self.p.copy(), self.h, length, 0.0, 0.0))
        self.p = self.p + length * np.array([np.cos(self.h), np.sin(self.h)])
        return self

    def bend(self, degrees, radius):
        sign = np.sign(degrees)
        sweep = np.radians(abs(degrees))
        self.segments.append(('arc', self.p.copy(), self.h, radius * sweep, radius, sign))
        centre = self.p + radius * sign * np.array([-np.sin(self.h), np.cos(self.h)])
        self.h += sign * sweep
        self.p = centre - radius * sign * np.array([-np.sin(self.h), np.cos(self.h)])
        return self

    def capped(self, start=False, end=True):
        """Close an open end with a flat end wall (a blanked-off pipe)."""
        self.start_cap, self.end_cap = start, end
        return self

    @property
    def length(self):
        return sum(s[3] for s in self.segments)

    def samples(self, line_step=LINE_STEP, arc_step_deg=ARC_STEP_DEG):
        """Arc length, points and unit tangents along the centreline."""
        out_s, out_p, out_t, s0 = [], [], [], 0.0
        for kind, p0, h0, length, radius, sign in self.segments:
            if kind == 'line':
                steps = np.linspace(0.0, length, max(2, int(np.ceil(length / line_step)) + 1))
                heading = np.full(len(steps), h0)
                pts = p0 + steps[:, None] * np.array([np.cos(h0), np.sin(h0)])
            else:
                n = max(2, int(np.ceil(np.degrees(length / radius) / arc_step_deg)) + 1)
                steps = np.linspace(0.0, length, n)
                heading = h0 + sign * steps / radius
                centre = p0 + radius * sign * np.array([-np.sin(h0), np.cos(h0)])
                pts = centre - radius * sign * np.c_[-np.sin(heading), np.cos(heading)]
            if out_s:
                steps, heading, pts = steps[1:], heading[1:], pts[1:]
            out_s.append(s0 + steps)
            out_p.append(pts)
            out_t.append(np.c_[np.cos(heading), np.sin(heading)])
            s0 += length
        return np.concatenate(out_s), np.concatenate(out_p), np.concatenate(out_t)


def add_run(mesh, run, bore, wall, collision=False):
    """
    One pipe run.

    Visual: the bore (inside surface), the outside surface, and flat end walls
    where capped or a rim where the pipe ends in the open.
    Collision: the bore only, with fewer facets around it but short ones along
    it, plus the inside of the end walls.
    """
    if collision:
        s, pts, tans = run.samples(LINE_STEP, ARC_STEP_DEG)
    else:
        s, pts, tans = run.samples(VISUAL_LINE_STEP, VISUAL_ARC_STEP_DEG)
    r_in, r_out = bore / 2.0, bore / 2.0 + wall
    axis_z = wall + r_in
    side = np.c_[tans[:, 1], -tans[:, 0], np.zeros(len(tans))]   # right of travel
    up = np.array([0.0, 0.0, 1.0])
    centre = np.c_[pts, np.full(len(pts), axis_z)]
    v = (s + run.v_offset) / TEXTURE_LENGTH_M

    def ring(n):
        # angle from the crown: 0 = top, pi = bottom (texture u = 0.5)
        phi = np.linspace(0.0, 2 * np.pi, n + 1)
        radial = (np.sin(phi)[None, :, None] * side[:, None, :]
                  + np.cos(phi)[None, :, None] * up[None, None, :])
        uv = np.dstack(np.broadcast_arrays(phi[None, :] / (2 * np.pi), v[:, None]))
        return radial, uv

    radial, uv = ring(AROUND_COLLISION if collision else AROUND)
    mesh.grid(centre[:, None, :] + r_in * radial, -radial, uv)          # bore, facing in
    if not collision:
        radial_out, uv_out = ring(AROUND_OUTSIDE)
        mesh.grid(centre[:, None, :] + r_out * radial_out, radial_out, uv_out)   # outside
    for i, cap, sign in ((0, run.start_cap, -1.0), (-1, run.end_cap, 1.0)):
        normal = sign * np.r_[tans[i], 0.0]
        if cap:   # flat end wall: one face towards the bore, one outside
            rim = np.linspace(0, 1, 2)[:, None, None] * r_out * radial[i][None]
            disc_uv = np.dstack([rim @ side[i], rim[..., 2]]) / (np.pi * bore)
            disc = centre[i] + rim
            mesh.grid(disc - normal * wall, np.broadcast_to(-normal, disc.shape), disc_uv)
            if not collision:
                mesh.grid(disc, np.broadcast_to(normal, disc.shape), disc_uv)
        elif not collision and not getattr(
                run, 'joined_%s' % ('start' if i == 0 else 'end'), False):
            # open end: the ring between the bore and the outside
            rim = centre[i] + np.stack([r_in * radial[i], r_out * radial[i]])
            mesh.grid(rim, np.broadcast_to(normal, rim.shape), uv[[i, i]])


# ---------------------------------------------------------------- chambers

def wall_with_holes(width, height, holes, segments=AROUND):
    """
    Triangulate a width x height rectangle with circular holes [(a, b, r)].

    a, b are local coordinates from the rectangle's lower-left corner. The
    rectangle is cut into vertical strips with at most one hole each; each
    strip is meshed by rays from the hole centre to the strip outline, so the
    outline (corners included) is exact. A hole may run past the bottom edge
    (b < 0): it is then cut off by that edge. Returns points (N,2) and faces.
    """
    holes = sorted(holes)
    cuts = [0.0] + [(h1[0] + h1[2] + h2[0] - h2[2]) / 2 for h1, h2 in zip(holes, holes[1:])] \
        + [width]
    pts, faces = [], []
    for k, (a0, a1) in enumerate(zip(cuts[:-1], cuts[1:])):
        if k >= len(holes):
            base = len(pts)
            pts += [(a0, 0), (a1, 0), (a1, height), (a0, height)]
            faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
            continue
        ca, cb, r = holes[k]
        corners = np.array([(a0, 0), (a1, 0), (a1, height), (a0, height)], float)
        ang = np.r_[np.linspace(0, 2 * np.pi, segments, endpoint=False),
                    np.arctan2(corners[:, 1] - cb, corners[:, 0] - ca) % (2 * np.pi)]
        if cb < r:  # where the hole crosses the bottom edge
            cross = np.arcsin(np.clip(-cb / r, -1, 1))
            ang = np.r_[ang, cross % (2 * np.pi), (np.pi - cross) % (2 * np.pi)]
        ang = np.unique(np.round(ang, 9))
        d = np.c_[np.cos(ang), np.sin(ang)]
        # distance along each ray to the strip outline
        with np.errstate(divide='ignore', invalid='ignore'):
            ta = np.where(d[:, 0] > 0, (a1 - ca) / d[:, 0], (a0 - ca) / d[:, 0])
            tb = np.where(d[:, 1] > 0, (height - cb) / d[:, 1], (0 - cb) / d[:, 1])
        t = np.fmin(np.where(np.isfinite(ta), ta, np.inf), np.where(np.isfinite(tb), tb, np.inf))
        inner = np.c_[ca, cb] + np.minimum(t, r)[:, None] * d
        outer = np.c_[ca, cb] + t[:, None] * d
        base, n = len(pts), len(ang)
        pts += list(map(tuple, inner)) + list(map(tuple, outer))
        for i in range(n):
            j = (i + 1) % n
            faces += [(base + i, base + n + i, base + n + j), (base + i, base + n + j, base + j)]
    return np.array(pts, float), np.array(faces, np.int64)


def add_plane(mesh, origin, a_axis, b_axis, normal, width, height, holes, uv_scale):
    p2, faces = wall_with_holes(width, height, holes)
    pts = origin + p2[:, :1] * a_axis + p2[:, 1:] * b_axis
    world_uv = np.c_[pts @ a_axis, pts @ b_axis] / uv_scale
    nrm = np.broadcast_to(normal, pts.shape)
    mesh.add(pts, nrm, world_uv, orient(pts, nrm, faces))


def add_cylinder(mesh, centre, axis, radius, length, inward, uv_scale, ring_dir):
    """Open cylinder from centre along axis (for hole sleeves and the shaft)."""
    axis = np.asarray(axis, float)
    e1 = np.asarray(ring_dir, float)
    e2 = np.cross(axis, e1)
    ang = np.linspace(0, 2 * np.pi, AROUND + 1)
    radial = np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2
    along = np.array([0.0, length])
    pts = centre + along[:, None, None] * axis + radius * radial[None]
    nrm = np.broadcast_to(-radial if inward else radial, pts.shape)
    uv = np.dstack(np.broadcast_arrays(ang[None, :] * radius / uv_scale,
                                       along[:, None] / uv_scale))
    mesh.grid(pts, nrm, uv)


class Chamber:
    """Square manhole chamber with a flat floor (FLOOR_LIFT above the invert)."""

    def __init__(self, x, y):
        self.c = np.array([x, y], float)
        self.holes = {d: [] for d in ('+x', '-x', '+y', '-y')}

    def face(self, direction):
        """Return the middle of an inside wall at floor level (direction '+x', '-y' ...)."""
        axis, sign = (0 if direction[1] == 'x' else 1), (1 if direction[0] == '+' else -1)
        p = self.c.copy()
        p[axis] += sign * CHAMBER_SIZE / 2
        return p

    def connect(self, run, at_start, direction, offset=0.0):
        """
        Open a hole for a run that starts or ends at this chamber's wall.

        The run's bore is extended through the wall.
        """
        self.holes[direction].append(offset)
        setattr(run, 'joined_%s' % ('start' if at_start else 'end'), True)


# chamber wall -> (outward direction, direction along the wall)
WALLS = {'+x': (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])),
         '-x': (np.array([-1.0, 0, 0]), np.array([0, -1.0, 0])),
         '+y': (np.array([0, 1.0, 0]), np.array([-1.0, 0, 0])),
         '-y': (np.array([0, -1.0, 0]), np.array([1.0, 0, 0]))}


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def add_floor(mesh, ch, floor, r_in, uv_scale, step=0.04):
    """
    Make the chamber floor as a height field.

    Flat, and shaped like the bottom of the pipe in front of each opening, so
    it meets the pipe exactly at the wall (benching at the sides, a shallow
    channel in the middle).
    """
    half = CHAMBER_SIZE / 2.0
    n = int(round(CHAMBER_SIZE / step)) + 1
    g = np.linspace(-half, half, n)
    x, y = np.meshgrid(g, g, indexing='ij')
    lift = np.zeros_like(x)
    dip = np.zeros_like(x)
    for d, offsets in ch.holes.items():
        out_dir, a_dir = WALLS[d]
        dist = half - (x * out_dir[0] + y * out_dir[1])     # distance from the wall
        into = smooth01(1.0 - dist / BENCH_LENGTH)          # 1 at the wall, 0 BENCH_LENGTH in
        for off in offsets:
            a = np.abs(x * a_dir[0] + y * a_dir[1] - off)   # sideways from the opening's middle
            ac = np.minimum(a, r_in)
            pipe = r_in - np.sqrt(r_in ** 2 - ac ** 2) - FLOOR_LIFT   # pipe bottom above the floor
            bump = np.clip(pipe, 0.0, BENCH_HEIGHT)
            bump *= 1.0 - smooth01((a - r_in) / BENCH_FADE)  # fade out beside the opening
            lift = np.maximum(lift, bump * into)
            # channel: the middle of the pipe bottom is below the floor
            dip = np.minimum(dip, np.minimum(pipe, 0.0) * into)
    z = floor + lift + dip
    pts = np.dstack([ch.c[0] + x, ch.c[1] + y, z])
    gx, gy = np.gradient(z, g, g)
    nrm = np.dstack([-gx, -gy, np.ones_like(z)])
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    uv = np.dstack([pts[..., 0], pts[..., 1]]) / uv_scale
    mesh.grid(pts, nrm, uv)


def add_chamber(mesh, ch, bore, wall):
    r_in, r_out = bore / 2.0, bore / 2.0 + wall
    inv = wall                      # pipe invert
    floor = inv + FLOOR_LIFT
    t = CHAMBER_WALL
    half = CHAMBER_SIZE / 2.0
    top = floor + CHAMBER_HEIGHT    # underside of the roof
    out_h = top + t                 # roof top
    cx, cy = ch.c
    s = CHAMBER_TEXTURE_M
    ez = np.array([0.0, 0.0, 1.0])
    for d, (out_dir, a_dir) in WALLS.items():
        mid = np.r_[cx, cy, 0.0]
        # inside face: from the floor to the roof, holes = pipe bores
        o_in = mid + half * out_dir - half * a_dir + floor * ez
        holes_in = [(half + off, r_in - FLOOR_LIFT, r_in) for off in ch.holes[d]]
        add_plane(mesh, o_in, a_dir, ez, -out_dir, CHAMBER_SIZE, CHAMBER_HEIGHT, holes_in, s)
        # outside face: from the ground to the roof top, holes = pipe outsides
        o_out = mid + (half + t) * out_dir - (half + t) * a_dir
        holes_out = [(half + t + off, inv + r_in, r_out) for off in ch.holes[d]]
        add_plane(mesh, o_out, a_dir, ez, out_dir, CHAMBER_SIZE + 2 * t, out_h, holes_out, s)
        # each pipe's bore continues through the wall
        for off in ch.holes[d]:
            centre = mid + half * out_dir + off * a_dir + (inv + r_in) * ez
            add_cylinder(mesh, centre, out_dir, r_in, t, True, s, ez)
    # floor: flat, with benching in front of each pipe; then the roof
    add_floor(mesh, ch, floor, r_in, s)
    shaft = [(half, half, SHAFT_DIAMETER / 2)]
    add_plane(mesh, np.r_[cx - half, cy - half, top], np.array([1.0, 0, 0]),
              np.array([0, 1.0, 0]), -ez, CHAMBER_SIZE, CHAMBER_SIZE, shaft, s)
    shaft_out = [(half + t, half + t, SHAFT_DIAMETER / 2)]
    add_plane(mesh, np.r_[cx - half - t, cy - half - t, out_h], np.array([1.0, 0, 0]),
              np.array([0, 1.0, 0]), ez, CHAMBER_SIZE + 2 * t, CHAMBER_SIZE + 2 * t,
              shaft_out, s)
    add_cylinder(mesh, np.r_[cx, cy, top], ez, SHAFT_DIAMETER / 2, t, True, s,
                 np.array([1.0, 0, 0]))


# ---------------------------------------------------------------- layouts

def cad_layout():
    """
    Return the CAD pipe's centreline (Designs/STL/PipeDesign_AGOS.stl), scaled.

    The world origin is the middle of the longest straight, where the robot
    spawns facing +x. Both ends are open.
    """
    k, r = CAD_SCALE, CAD_BEND_R * CAD_SCALE
    run = Run((-1.2192 * k, -1.2192 * k), 90.0)
    run.straight(0.6096 * k).bend(-90, r).straight(1.2192 * k).bend(-180, r)
    run.straight(0.6096 * k).bend(90, r).straight(0.6096 * k)
    return [run], []


def sewer_layout():
    r"""
    Return a loop of four manholes (A, B, C, D) and a dead-end branch off B.

            D ===== S-bend ===== C
            |                    |
            |                    |       N (+y)
            A ================== B ==\    |
                                      ||   +-- E (+x)
                                      ==  (capped)

    The robot spawns in chamber A at the origin, facing B (+x).
    """
    half = CHAMBER_SIZE / 2
    a, b, c = Chamber(0.0, 0.0), Chamber(16.0, 0.0), Chamber(16.0, 11.0)
    # C -> D: west, then an S-bend (45 deg right, 1.5 m, 45 deg left), then west
    sx = 2 * BEND_R * np.sin(np.radians(45)) + 1.5 * np.cos(np.radians(45))
    sy = 2 * BEND_R * (1 - np.cos(np.radians(45))) + 1.5 * np.sin(np.radians(45))
    d = Chamber(0.0, 11.0 + sy)
    runs = []

    def link(c0, d0, c1, d1, build, v_offset):
        run = Run(c0.face(d0), {'+x': 0, '+y': 90, '-x': 180, '-y': 270}[d0], v_offset)
        build(run)
        end = c1.face(d1)
        if np.linalg.norm(run.p - end) > 1e-6:
            raise SystemExit('layout error: run ends at %s, chamber wall at %s' % (run.p, end))
        c0.connect(run, True, d0)
        c1.connect(run, False, d1)
        runs.append(run)

    link(a, '+x', b, '-x', lambda r: r.straight(16.0 - 2 * half), 0.0)
    link(b, '+y', c, '-y', lambda r: r.straight(11.0 - 2 * half), 4.1)
    link(c, '-x', d, '+x', lambda r: r.straight(3.0).bend(-45, BEND_R).straight(1.5)
         .bend(45, BEND_R).straight(16.0 - 2 * half - 3.0 - sx), 9.3)
    link(d, '-y', a, '+y', lambda r: r.straight(11.0 + sy - 2 * half), 13.7)
    spur = Run(b.face('+x'), 0.0, 6.5)
    spur.straight(3.0).bend(-90, BEND_R).straight(5.0).capped(end=True)
    b.connect(spur, True, '+x')
    runs.append(spur)
    return runs, [a, b, c, d]


# Headings of the four chamber walls, and turtle steps for the network layout:
# ('S', length) is a straight, ('B', degrees, radius) a bend (positive = left).
HEADINGS = {'+x': 0.0, '+y': 90.0, '-x': 180.0, '-y': 270.0}


def run_from(chamber, direction, steps, v_offset=0.0):
    """Make a run that leaves `chamber` square through its `direction` wall."""
    run = Run(chamber.face(direction), HEADINGS[direction], v_offset)
    for step in steps:
        if step[0] == 'S':
            run.straight(step[1])
        else:
            run.bend(step[1], step[2])
    return run


def chamber_at_end(run, direction):
    """
    Find the chamber whose `direction` wall is where `run` ends.

    The run must arrive square to that wall, heading into the chamber.
    """
    heading = np.degrees(run.h) % 360.0
    want = (HEADINGS[direction] + 180.0) % 360.0
    if abs((heading - want + 180.0) % 360.0 - 180.0) > 1e-6:
        raise SystemExit('layout error: run arrives heading %.1f deg, the %s wall needs %.1f'
                         % (heading, direction, want))
    out = np.radians(HEADINGS[direction])
    centre = run.p - CHAMBER_SIZE / 2 * np.array([np.cos(out), np.sin(out)])
    return Chamber(*centre)


def solve_straights(c0, d0, c1, d1, steps):
    """
    Fill in the two straights given as ('S', None).

    They are solved so that the run from c0's d0 wall ends at c1's d1 wall.
    The two must not be parallel.
    """
    unknown = [i for i, step in enumerate(steps) if step[0] == 'S' and step[1] is None]
    if len(unknown) != 2:
        raise SystemExit('layout error: solve_straights needs exactly two unknown straights')

    def end(lengths):
        filled = list(steps)
        for i, length in zip(unknown, lengths):
            filled[i] = ('S', length)
        return run_from(c0, d0, filled).p, filled

    p0 = end([0.0, 0.0])[0]
    a = np.c_[end([1.0, 0.0])[0] - p0, end([0.0, 1.0])[0] - p0]
    lengths = np.linalg.solve(a, c1.face(d1) - p0)
    if lengths.min() < 0.5:
        raise SystemExit('layout error: solved straights %s are too short; move the chambers'
                         % np.round(lengths, 2))
    return end(lengths)[1]


def network_layout():
    """
    Five manholes in a 250 m loop, with a dead-end branch at four of them.

        M5 ~~~wiggle~~~ step _/~dogleg M1 ~S-curve~ zig-zag ======= M2 =hairpin
        |  `zig-zag spur                                            |
        |                                                   serpentine
        |                                                           |
        `========== gentle S ========= M4 ~~ sweeping curves ~~ M3 -135 deg spur
                                       `45 deg spur

    The robot starts in M1 at the origin, facing M2 (+x). M1 sits on a straight
    line of pipe like most manholes: 49.5 m of twisty pipe east to M2 and
    44.8 m west to M5, so it can explore 25 m each way (the proposal's
    mission) without reaching the next manhole. Pipes only meet in manholes.
    """
    r = BEND_R
    runs = []
    v = [0.0]

    def add(run):
        run.v_offset = v[0]
        v[0] += 7.3     # so neighbouring runs do not show the same stretch of texture
        runs.append(run)
        return run

    def link(c0, d0, c1, d1, run):
        c0.connect(run, True, d0)
        c1.connect(run, False, d1)
        add(run)

    m1 = Chamber(0.0, 0.0)
    # M1 -> M2: a long sweeping S-curve (R 6 m), a zig-zag of 30/60 deg bends,
    # a straight, then 90 deg right into M2
    east = run_from(m1, '+x', [
        ('S', 3.0), ('B', 60, 6.0), ('S', 2.0), ('B', -60, 6.0), ('S', 3.0),
        ('B', -30, r), ('S', 1.2), ('B', 60, r), ('S', 1.2), ('B', -60, r), ('S', 1.2),
        ('B', 30, r), ('S', 14.0), ('B', -90, r), ('S', 5.0)])
    m2 = chamber_at_end(east, '+y')
    link(m1, '+x', m2, '+y', east)
    # M2 -> M3: a serpentine of 90 deg bends heading south
    south = run_from(m2, '-y', [
        ('S', 2.0), ('B', -90, r), ('S', 2.0), ('B', 90, r), ('S', 2.0), ('B', 90, r),
        ('S', 2.0), ('B', -90, r), ('S', 3.0), ('B', -90, r), ('S', 2.0), ('B', 90, r),
        ('S', 2.5)])
    m3 = chamber_at_end(south, '+y')
    link(m2, '-y', m3, '+y', south)
    # M3 -> M4: long sweeping curves (R 8 m), then a dogleg
    southwest = run_from(m3, '-x', [
        ('S', 2.0), ('B', 30, 8.0), ('S', 3.0), ('B', -60, 8.0), ('S', 3.0), ('B', 30, 8.0),
        ('S', 6.0), ('B', -45, r), ('S', 1.5), ('B', 45, r), ('S', 4.0)])
    m4 = chamber_at_end(southwest, '+x')
    link(m3, '-x', m4, '+x', southwest)
    # M1 -> M5: a dogleg, a step (90 left, 90 right), a gentle wiggle (R 8 m)
    west = run_from(m1, '-x', [
        ('S', 2.5), ('B', -45, r), ('S', 2.0), ('B', 45, r), ('S', 4.0),
        ('B', 90, r), ('S', 3.0), ('B', -90, r), ('S', 3.0),
        ('B', -25, 8.0), ('S', 2.0), ('B', 50, 8.0), ('S', 2.0), ('B', -25, 8.0), ('S', 6.0)])
    m5 = chamber_at_end(west, '+x')
    link(m1, '-x', m5, '+x', west)
    # M4 -> M5 closes the loop: a gentle S (R 10 m), a long straight, then a
    # wide 90 deg bend (R 4 m) north into M5. The two straights are solved.
    steps = solve_straights(m4, '-x', m5, '-y', [
        ('S', 3.0), ('B', 20, 10.0), ('S', 2.0), ('B', -20, 10.0), ('S', None),
        ('B', -90, 4.0), ('S', None)])
    link(m4, '-x', m5, '-y', run_from(m4, '-x', steps))

    # dead-end branches, blanked off at the end
    for ch, d, branch in ((m2, '+x', [('S', 3.0), ('B', -180, r), ('S', 1.5)]),
                          (m3, '-y', [('S', 2.0), ('B', -135, r), ('S', 3.0)]),
                          (m4, '-y', [('S', 2.5), ('B', 45, r), ('S', 4.0)]),
                          (m5, '-x', [('S', 2.0), ('B', 30, r), ('S', 1.5), ('B', -60, r),
                                      ('S', 1.5), ('B', 30, r), ('S', 2.0)])):
        spur = run_from(ch, d, branch).capped(end=True)
        ch.connect(spur, True, d)
        add(spur)
    check_clearance(runs, [m1, m2, m3, m4, m5])
    return runs, [m1, m2, m3, m4, m5]


def check_clearance(runs, chambers, gap=0.2):
    """Stop if two pipes overlap or a pipe runs through a chamber."""
    r_out = BORE / 2 + WALL
    pts = [run.samples(0.3)[1] for run in runs]
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            d = np.linalg.norm(pts[i][:, None] - pts[j][None], axis=2)
            k = np.unravel_index(d.argmin(), d.shape)
            in_chamber = min(np.abs(pts[i][k[0]] - ch.c).max() for ch in chambers)
            if d.min() < 2 * r_out + gap and in_chamber > CHAMBER_SIZE / 2 + 1.0:
                raise SystemExit('layout error: runs %d and %d come %.2f m apart at %s'
                                 % (i, j, d.min(), np.round(pts[i][k[0]], 2)))
    for i, p in enumerate(pts):
        for ch in chambers:
            if (np.abs(p - ch.c).max(axis=1) < CHAMBER_SIZE / 2 + r_out - 0.01).sum() > 2:
                raise SystemExit('layout error: run %d passes through the chamber at %s'
                                 % (i, ch.c))


def build(layout, bore, wall):
    runs, chambers = layout()
    pipes, pipes_collision, rooms = Mesh(), Mesh(), Mesh()
    for run in runs:
        add_run(pipes, run, bore, wall)
        add_run(pipes_collision, run, bore, wall, collision=True)
    for ch in chambers:
        add_chamber(rooms, ch, bore, wall)
    return runs, chambers, pipes, pipes_collision, rooms


def main():
    global CHAMBER_SIZE
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--layout', choices=['cad', 'sewer', 'network', 'all'], default='all')
    ap.add_argument('--bore', type=float, default=BORE, help='inside diameter in m (default 0.9)')
    ap.add_argument('--wall', type=float, default=WALL, help='wall thickness in m (default 0.1)')
    ap.add_argument('-o', '--output-dir', default=os.path.join(here, '..', 'meshes'))
    args = ap.parse_args()
    if args.bore + 2 * args.wall > CHAMBER_SIZE - 0.2:
        CHAMBER_SIZE = args.bore + 2 * args.wall + 1.0

    jobs = {'cad': ('agos_pipe', cad_layout),
            'sewer': ('agos_sewer', sewer_layout),
            'network': ('agos_network', network_layout)}
    todo = list(jobs) if args.layout == 'all' else [args.layout]
    for key in todo:
        name, layout = jobs[key]
        runs, chambers, pipes, pipes_collision, rooms = build(layout, args.bore, args.wall)
        pipe_name = name if not chambers else name + '_pipes'
        parts = [(pipe_name, pipes, pipes_collision)]
        if chambers:
            parts.append((name + '_chambers', rooms, rooms))
        for part, visual, collision in parts:
            title = 'AGOS %s, bore %.3f m, wall %.3f m' % (part, args.bore, args.wall)
            base = os.path.join(args.output_dir, part)
            n_obj = visual.write_obj(base + '.obj', title)
            n_stl = collision.write_stl(base + '.stl', title + ', collision')
            print('wrote %s.obj (%d triangles) and .stl (%d triangles)'
                  % (os.path.normpath(base), n_obj, n_stl))
        print('  %s: %.1f m of pipe in %d runs, %d chambers'
              % (name, sum(r.length for r in runs), len(runs), len(chambers)))
        for i, ch in enumerate(chambers):
            print('    chamber %d at (%.2f, %.2f)' % (i + 1, ch.c[0], ch.c[1]))


if __name__ == '__main__':
    main()
