#!/usr/bin/env python3
"""Rebuild the robot meshes (meshes/agos_*.stl) from the CAD export.

The input is Designs/STL/AGOS V1.stl: a binary STL from Fusion, in mm, with the
robot facing +y and a colour stored on every triangle (the "COLOR=" convention:
bits 0-4 red, 5-9 green, 10-14 blue). At about 10 million triangles it is far
too heavy for Gazebo and RViz, so this script:
  1. splits the triangles by colour into the six parts listed in PARTS
     (one mesh per part, so each part can get its own colour in the URDF),
  2. simplifies every separate piece (each link, pin, bolt, plate...) on its own:
       - small or thin pieces (under SMALL_MM across, or under THIN_MM thick)
         become a plain box, or an 8-sided cylinder if they are round like a pin;
       - larger pieces are simplified as far as they keep their shape (size
         within 0.5 mm or 1 %, area within 6 %, volume within 5 %);
       - long chain and rail pieces that cannot be simplified become a box,
  3. leaves out the tether: the 25 mm tube that runs 0.69 m out of the back
     of the body (the communication and power cable to the base station). In
     the real robot it trails behind and bends, so it must not be a rigid part.
     The green fitting it plugs into stays. Use --keep-tether to keep it.
  4. turns the result to face +x (ROS convention), converts mm to m and writes
     meshes/agos_<part>.stl. The CAD origin stays the origin (= base_link).

Needs numpy, scipy and fast-simplification (pip install fast-simplification),
and about 3 GB of free memory for the full model:
    python3 tools/convert_robot_stl.py "/path/to/AGOS V1.stl"
    python3 tools/convert_robot_stl.py "/path/to/AGOS V1.stl" --list-colours
Then rebuild (colcon build) so the new meshes are installed.
If you recolour parts in the CAD, run with --list-colours and update PARTS.
"""
import argparse
import gzip
import os
import struct

import numpy as np

# part name -> CAD colours it is made of. The names match the
# <xacro:agos_mesh_visual part=...> lines in agos_urdf.xacro.
PARTS = {
    'body':    [20083, 22197, 15855, 19060, 24311],
    'treads':  [4311],
    'accents': [4796, 1757],
    'light':   [30686, 30653, 29595, 32767],
    'green':   [8776, 13132],
    'dark':    [2114, 3171],
}

SMALL_MM = 30.0     # pieces smaller than this (bounding-box diagonal) become a box/cylinder
THIN_MM = 1.5       # pieces thinner than this become a box
LONG_RATIO = 8.0    # length / width above which a piece counts as a chain or rail
ROD_RATIO = 1.3     # a piece whose two short sides are this close is treated as round
SIZE_TOL_MM = 0.5   # simplified piece must keep its size to within this (or 1 %) ...
AREA_TOL = 0.06     # ... its surface area to within 6 % ...
VOLUME_TOL = 0.05   # ... and its volume to within 5 %
BODY_REAR_MM = -356.0  # back face of the body (CAD y); the tether runs out behind it
TETHER_MM = 100.0      # a piece reaching this far behind the body is the tether

STL_RECORD = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
BOX_FACES = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
BOX_CORNERS = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], dtype=float)


def read_stl(path):
    """Return (triangles (N,3,3) float32 in mm, colour attribute (N,) uint16)."""
    opener = gzip.open if path.endswith('.gz') else open
    with opener(path, 'rb') as fh:
        data = fh.read()
    count = struct.unpack('<I', data[80:84])[0]
    if 84 + count * STL_RECORD.itemsize != len(data):
        raise SystemExit('%s is not a binary STL (ASCII STL has no colours)' % path)
    rec = np.frombuffer(data, dtype=STL_RECORD, count=count, offset=84)
    return rec['v'], rec['a']


def colour_rgb(value):
    return ((value & 31) / 31.0, ((value >> 5) & 31) / 31.0, ((value >> 10) & 31) / 31.0)


def list_colours(attr):
    values, counts = np.unique(attr, return_counts=True)
    known = {c: name for name, colours in PARTS.items() for c in colours}
    print('colour  triangles  rgb               part')
    for value, count in sorted(zip(values.tolist(), counts.tolist()), key=lambda vc: -vc[1]):
        if value & 0x8000:
            rgb = 'object default'
        else:
            rgb = '%.2f %.2f %.2f' % colour_rgb(value)
        print('%6d %10d  %-16s  %s' % (value, count, rgb, known.get(value, '-- not used --')))


def drop_collapsed(faces):
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return faces[ok]


def indexed_mesh(tris):
    """Triangle soup -> shared vertices (V,3) float64 and faces (F,3) int64."""
    verts, inverse = np.unique(tris.reshape(-1, 3), axis=0, return_inverse=True)
    return verts.astype(np.float64), drop_collapsed(inverse.reshape(-1, 3).astype(np.int64))


def split_pieces(verts, faces):
    """Yield (verts, faces) for each separate (connected) piece of the mesh."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    n = len(verts)
    rows = np.concatenate([faces[:, 0], faces[:, 1], faces[:, 2]])
    cols = np.concatenate([faces[:, 1], faces[:, 2], faces[:, 0]])
    graph = coo_matrix((np.ones(len(rows), dtype=np.int8), (rows, cols)), shape=(n, n))
    label = connected_components(graph, directed=False)[1][faces[:, 0]]
    order = np.argsort(label, kind='stable')
    faces, label = faces[order], label[order]
    bounds = np.r_[0, np.flatnonzero(np.diff(label)) + 1, len(faces)]
    for start, end in zip(bounds[:-1], bounds[1:]):
        used, local = np.unique(faces[start:end], return_inverse=True)
        yield verts[used], local.reshape(-1, 3)


def measure(verts, faces):
    """Surface area, signed volume and bounding box of a mesh."""
    tri = verts[faces]
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = 0.5 * np.linalg.norm(cross, axis=1).sum()
    volume = np.einsum('ij,ij->i', tri[:, 0], cross).sum() / 6.0
    return area, volume, tri.reshape(-1, 3).min(axis=0), tri.reshape(-1, 3).max(axis=0)


def outward(verts, faces):
    """Flip a closed primitive's faces if they point inwards."""
    return (verts, faces) if measure(verts, faces)[1] >= 0 else (verts, faces[:, ::-1].copy())


def bounding_box(verts, faces):
    """Tightest of the CAD-aligned and the principal-axis box around a piece.

    Returns centre, axes (rows, shortest side first) and half sizes.
    """
    tri = verts[faces]
    area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) + 1e-12
    centroid = tri.mean(axis=1)
    mean = (centroid * area[:, None]).sum(axis=0) / area.sum()
    spread = ((centroid - mean) * area[:, None]).T @ (centroid - mean)
    best = None
    for axes in (np.eye(3), np.linalg.eigh(spread)[1].T):  # rows = box directions
        local = verts @ axes.T
        lo, hi = local.min(axis=0), local.max(axis=0)
        if best is None or np.prod(hi - lo + 1e-3) < 0.95 * np.prod(best[2] + 1e-3):
            best = (axes, (lo + hi) / 2, hi - lo)
    axes, mid, extent = best
    order = np.argsort(extent)
    return mid @ axes, axes[order], extent[order] / 2


def make_box(centre, axes, half):
    return outward(centre + (BOX_CORNERS * half) @ axes, BOX_FACES.copy())


def make_cylinder(centre, axis, side_axes, radius, half_length, sides):
    angle = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    ring = radius * (np.cos(angle)[:, None] * side_axes[0] + np.sin(angle)[:, None] * side_axes[1])
    ends = [centre - half_length * axis, centre + half_length * axis]
    verts = np.concatenate([ends[0] + ring, ends[1] + ring, ends])
    i = np.arange(sides)
    j = (i + 1) % sides
    bottom, top = 2 * sides, 2 * sides + 1
    faces = np.concatenate([np.c_[i, j, sides + j], np.c_[i, sides + j, sides + i],
                            np.c_[np.full(sides, bottom), j, i],
                            np.c_[np.full(sides, top), sides + i, sides + j]])
    return outward(verts, faces)


def is_rod(half):
    return half[1] < ROD_RATIO * half[0] and half[2] > 1.5 * half[1]


def primitive(verts, faces, sides):
    """Replace a piece by a box, a rod (pin, bolt) or a disc (washer, lens)."""
    centre, axes, half = bounding_box(verts, faces)
    if 2 * half[2] >= 8.0:  # tiny pieces are simply boxes
        if is_rod(half):
            radius = (half[0] + half[1]) / 2
            return make_cylinder(centre, axes[2], axes[:2], radius, half[2], sides)
        if half[2] < ROD_RATIO * half[1] and half[0] < 0.67 * half[1]:
            radius = (half[1] + half[2]) / 2
            return make_cylinder(centre, axes[0], axes[1:], radius, half[0], sides)
    return make_box(centre, axes, half)


def simplify_piece(verts, faces):
    import fast_simplification
    n = len(faces)
    if n <= 24:
        return verts, faces
    area0, volume0, lo0, hi0 = measure(verts, faces)
    size = np.linalg.norm(hi0 - lo0)
    half = bounding_box(verts, faces)[2]
    if size < SMALL_MM or 2 * half[0] < THIN_MM:
        return primitive(verts, faces, 8)
    long_piece = half[2] > LONG_RATIO * half[1]
    closed = abs(volume0) > 1e-6 * size ** 3
    targets = [max(100, n // 50), max(100, n // 20), n // 10, n // 5]
    if not long_piece:
        targets.append(int(n / 2.5))
    for target in targets:
        if target >= n:
            break
        v, f = fast_simplification.simplify(verts, faces.astype(np.int32),
                                            target_count=int(target), agg=7)
        v, f = np.asarray(v, np.float64), drop_collapsed(np.asarray(f, np.int64))
        if not len(f):
            continue
        area, volume, lo, hi = measure(v, f)
        if (max(np.abs(lo - lo0).max(), np.abs(hi - hi0).max()) <= max(SIZE_TOL_MM, 0.01 * size)
                and abs(area / area0 - 1) <= AREA_TOL
                and (not closed or abs(volume / volume0 - 1) <= VOLUME_TOL)):
            # Bolts and standoffs under 80 mm that only keep their shape with
            # 1/5 of their triangles (threads) are cheaper as a cylinder.
            if target >= n // 5 and size < 80 and is_rod(half):
                return primitive(verts, faces, 16)
            return v, f
    if long_piece:
        return primitive(verts, faces, 16)
    return verts, faces


def is_tether(verts):
    """The tether is the only piece that sticks out far behind the body."""
    return verts[:, 1].min() < BODY_REAR_MM - TETHER_MM


def simplify_part(tris, keep_tether=False):
    verts, faces = indexed_mesh(tris)
    out_v, out_f, offset, pieces = [], [], 0, 0
    for v, f in split_pieces(verts, faces):
        if not keep_tether and is_tether(v):
            print('    left out the tether: %d triangles, %.0f mm long behind the body'
                  % (len(f), BODY_REAR_MM - v[:, 1].min()))
            continue
        v, f = simplify_piece(v, f)
        out_v.append(v)
        out_f.append(f + offset)
        offset += len(v)
        pieces += 1
    return np.concatenate(out_v), np.concatenate(out_f), pieces


def cad_to_urdf(verts):
    """CAD (mm, +y forward) -> URDF base_link (m, +x forward, +y left)."""
    return np.c_[verts[:, 1], -verts[:, 0], verts[:, 2]] * 0.001


def write_stl(path, verts, faces, title):
    tri = verts[faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1)
    keep = length > 1e-12  # drop zero-area slivers
    tri, n = tri[keep], n[keep] / length[keep, None]
    rec = np.zeros(len(tri), dtype=STL_RECORD)
    rec['n'] = n
    rec['v'] = tri
    with open(path, 'wb') as fh:
        fh.write(title.encode()[:80].ljust(80, b' '))
        fh.write(struct.pack('<I', len(tri)))
        rec.tofile(fh)
    return len(tri)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='the CAD export, e.g. "AGOS V1.stl" (.stl or .stl.gz)')
    ap.add_argument('-o', '--output-dir', default=os.path.join(here, '..', 'meshes'))
    ap.add_argument('--keep-tether', action='store_true',
                    help='keep the tether tube that runs out of the back of the robot')
    ap.add_argument('--list-colours', action='store_true',
                    help='only print the colours in the file and which part uses each')
    args = ap.parse_args()

    tris, attr = read_stl(args.input)
    print('read %s: %d triangles' % (args.input, len(tris)))
    if args.list_colours:
        list_colours(attr)
        return

    used = np.zeros(len(attr), dtype=bool)
    for name, colours in PARTS.items():
        mask = np.isin(attr, colours)
        used |= mask
        verts, faces, pieces = simplify_part(tris[mask], args.keep_tether)
        out = os.path.join(args.output_dir, 'agos_%s.stl' % name)
        written = write_stl(out, cad_to_urdf(verts), faces,
                            'AGOS %s (metres, +x forward) from tools/convert_robot_stl.py' % name)
        print('  %-8s %9d -> %7d triangles (%d pieces)  %s'
              % (name, mask.sum(), written, pieces, os.path.normpath(out)), flush=True)
    if not used.all():
        print('note: %d triangles have a colour that no part uses (see --list-colours)'
              % (~used).sum())


if __name__ == '__main__':
    main()
