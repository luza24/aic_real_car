#!/usr/bin/env python3
"""Thin the walls of world.pgm by one cell.

gmapping draws every wall 2-3 cells (10-15 cm) thick while the real boards are
2 cm, which makes the narrowest corridor (0.50 m of map free space instead of
0.64 m) barely passable for TEB. This tool erodes the occupied cells by one
cell from every side and keeps a one-cell skeleton for walls that would
otherwise disappear (the 2-cell outer boundary), so no wall gets a hole.

Run from the source tree (a one-off map maintenance tool, not a node):

    python3 src/aic26_slam/scripts/thin_map_walls.py            # dry run
    python3 src/aic26_slam/scripts/thin_map_walls.py --write    # apply

The checks printed below must pass: the free space keeps its components and the
inner free region must not reach any cell that was unknown before (that would
mean a wall was opened).
"""
import argparse
import re

import numpy as np
from scipy import ndimage
from skimage.morphology import thin

OCC, FREE, UNKNOWN = 0, 254, 205
CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)


def read_pgm(path):
    data = open(path, 'rb').read()
    idx, parts = 0, []
    while len(parts) < 4:
        m = re.match(rb'\s*(#[^\n]*\n\s*)*([^\s]+)', data[idx:])
        parts.append(m.group(2))
        idx += m.end()
    width, height = int(parts[1]), int(parts[2])
    pixels = np.frombuffer(data[idx + 1:], dtype=np.uint8).reshape(height,
                                                                    width)
    return data[:idx + 1], pixels.copy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default='src/aic26_slam/maps/world.pgm')
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()

    header, px = read_pgm(args.map)
    occ = px < 100
    free = px > 250
    unknown = ~occ & ~free

    eroded = ndimage.binary_erosion(occ, structure=CROSS)
    skeleton = thin(occ)
    print('first pass would leave %d cells (one-cell erosion + skeleton)'
          % (eroded | skeleton).sum())
    new_occ = occ.copy()
    for _ in range(5):
        nxt = ndimage.binary_erosion(new_occ, structure=CROSS) | thin(new_occ)
        if np.array_equal(nxt, new_occ):
            break
        new_occ = nxt
    removed = occ & ~new_occ

    print('occupied %d -> %d (removed %d), skeleton cells %d'
          % (occ.sum(), new_occ.sum(), removed.sum(), skeleton.sum()))

    # thickness report
    for name, mask in (('before', occ), ('after', new_occ)):
        cnt = np.zeros_like(px, dtype=int)
        cnt[1:-1, 1:-1] = (mask[:-2, 1:-1].astype(int) + mask[2:, 1:-1] +
                           mask[1:-1, :-2] + mask[1:-1, 2:])
        hist = {k: int(((cnt == k) & mask).sum()) for k in range(5)}
        print('  %s 4-neighbour histogram %s' % (name, hist))

    # free space must keep its components
    for name, f in (('before', free), ('after', free | removed)):
        lab, n = ndimage.label(f, structure=np.ones((3, 3)))
        sizes = sorted([int((lab == i).sum()) for i in range(1, n + 1)],
                       reverse=True)[:4]
        print('  %s free components %d %s' % (name, n, sizes))

    # no leak: the inner free region must not reach previously unknown cells
    lab_free, n_free = ndimage.label(free, structure=np.ones((3, 3)))
    largest = 1 + int(np.argmax([(lab_free == i).sum()
                                 for i in range(1, n_free + 1)]))
    seed = tuple(np.argwhere(lab_free == largest)[0])
    new_free = free | removed
    lab, _ = ndimage.label(new_free, structure=np.ones((3, 3)))
    inside = lab == lab[seed]
    leak = int((inside & unknown).sum())
    print('  inner free region (seed %s) reaches %d previously unknown cells %s'
          % (seed, leak, 'OK' if leak == 0 else 'LEAK!'))
    if leak:
        raise SystemExit('refusing to write: the erosion opens a wall')

    # corridor width at the tightest passage (x 0.60..1.65, y = 0.30)
    r = px.shape[0] - 1 - int((0.30 + 6) / 0.05)
    row = ''.join('#' if new_occ[r, c] else '.' for c in
                  range(int((0.60 + 6) / 0.05), int((1.65 + 6) / 0.05)))
    print('  corridor row y=0.30, x 0.60..1.65:', row)

    if args.write:
        out = px.copy()
        out[removed] = FREE
        with open(args.map, 'wb') as f:
            f.write(header)
            f.write(out.tobytes())
        print('wrote', args.map)
    else:
        print('dry run, pass --write to apply')


if __name__ == '__main__':
    main()
