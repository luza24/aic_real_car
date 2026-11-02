"""Clean a recorded map: close small wall gaps and drop isolated cells.

A plain 3x3 opening erodes the walls away: at 0.05 m resolution a wall is only
one or two cells thick. This tool instead closes gaps along walls (a line
structuring element in 0, 45, 90 and 135 degrees) and only removes cells that
have no occupied neighbour at all.

    python3 clean_map.py --map world_real.yaml --backup
    python3 clean_map.py --map world_real.yaml --close 2 --min-neighbours 1
"""

import argparse
import os
import shutil

import numpy as np

from realign_map import read_map, read_pgm, write_pgm

DIRECTIONS = ((0, 1), (1, 0), (1, 1), (1, -1))


def components(mask):
    """Return the number of 8-connected components and the largest one."""
    height, width = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    sizes = []
    for row, col in zip(*np.nonzero(mask)):
        if seen[row, col]:
            continue
        stack = [(row, col)]
        seen[row, col] = True
        size = 0
        while stack:
            r, c = stack.pop()
            size += 1
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if (0 <= rr < height and 0 <= cc < width and
                            mask[rr, cc] and not seen[rr, cc]):
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        sizes.append(size)
    return len(sizes), (max(sizes) if sizes else 0)


def close_lines(mask, length):
    """Fill gaps of up to ``length`` cells between two wall cells."""
    filled = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    for dr, dc in DIRECTIONS:
        for step in range(1, length + 1):
            a = np.zeros_like(mask)
            b = np.zeros_like(mask)
            src_rows = slice(max(0, -dr * step), height - max(0, dr * step))
            dst_rows = slice(max(0, dr * step), height - max(0, -dr * step))
            src_cols = slice(max(0, -dc * step), width - max(0, dc * step))
            dst_cols = slice(max(0, dc * step), width - max(0, -dc * step))
            a[dst_rows, dst_cols] = mask[src_rows, src_cols]
            b[src_rows, src_cols] = mask[dst_rows, dst_cols]
            filled |= a & b
    return mask | filled, int((filled & ~mask).sum())


def count_neighbours(mask):
    total = np.zeros(mask.shape, dtype=int)
    height, width = mask.shape
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            a = np.zeros_like(mask, dtype=int)
            src_rows = slice(max(0, -dr), height - max(0, dr))
            dst_rows = slice(max(0, dr), height - max(0, -dr))
            src_cols = slice(max(0, -dc), width - max(0, dc))
            dst_cols = slice(max(0, dc), width - max(0, -dc))
            a[dst_rows, dst_cols] = mask[src_rows, src_cols]
            total += a
    return total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--close', type=int, default=2,
                        help='gap length in cells to close, 0 disables')
    parser.add_argument('--min-neighbours', type=int, default=1,
                        help='drop a wall cell with fewer occupied '
                             'neighbours (1 keeps short wall ends)')
    parser.add_argument('--out', default='')
    parser.add_argument('--backup', action='store_true')
    args = parser.parse_args()

    image, resolution, origin, _ = read_map(args.map)
    base = os.path.dirname(os.path.abspath(args.map))
    image_path = image if os.path.isabs(image) else os.path.join(base, image)
    pixels, width, height = read_pgm(image_path)

    mask = pixels < 50
    parts, biggest = components(mask)
    print('input  %dx%d occupied %d, components %d (largest %d)'
          % (width, height, mask.sum(), parts, biggest))

    if args.close > 0:
        mask, added = close_lines(mask, args.close)
        print('closed gaps <= %d cells: +%d cells' % (args.close, added))

    neighbours = count_neighbours(mask)
    specks = mask & (neighbours < args.min_neighbours)
    if specks.any():
        mask = mask & ~specks
        print('removed %d isolated cells' % specks.sum())

    parts, biggest = components(mask)
    print('output %dx%d occupied %d, components %d (largest %d)'
          % (width, height, mask.sum(), parts, biggest))

    result = np.array(pixels, copy=True)
    result[(pixels < 50) & ~mask] = 205
    result[mask] = 0
    out_image = args.out or image_path
    if args.backup and out_image == image_path:
        shutil.copy2(image_path, image_path + '.orig')
        print('backup %s.orig' % image_path)
    write_pgm(out_image, result, creator='clean_map.py')
    print('wrote %s' % out_image)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
