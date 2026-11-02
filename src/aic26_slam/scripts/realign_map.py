"""Rotate and shift a recorded map so its frame matches the venue frame.

Used once after ``mapping`` mode: the recorded map inherits the error of the
pose the odometry was initialised with, so it is rotated by a few degrees
relative to the waypoint frame. This tool resamples the map onto the same grid
(same resolution, origin and size, so the virtual walls keep matching) with

    p_map' = R(yaw) * p_map + (dx, dy)

    python3 realign_map.py --map ~/aic26_ws_112/src/aic26_slam/maps/world_real.yaml \
        --yaw -3.65 --dx -0.13 --dy 0.09 --backup
"""

import argparse
import math
import os
import shutil

import numpy as np


def read_map(path):
    fields = {}
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            key, value = line.split(':', 1)
            fields[key.strip()] = value.strip()
    image = fields['image']
    resolution = float(fields['resolution'])
    origin = [float(v) for v in fields['origin'].strip('[]').split(',')]
    return image, resolution, origin, fields


def read_pgm(path):
    data = open(path, 'rb').read()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            while data[index:index + 1] not in (b'\n', b''):
                index += 1
            continue
        start = index
        while not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    index += 1
    width = int(tokens[1])
    height = int(tokens[2])
    pixels = np.frombuffer(data[index:index + width * height],
                           dtype=np.uint8).reshape(height, width)
    return pixels, width, height


def write_pgm(path, pixels, creator='realign_map.py'):
    height, width = pixels.shape
    with open(path, 'wb') as handle:
        handle.write(b'P5\n# CREATOR: %s\n%d %d\n255\n'
                     % (creator.encode('ascii'), width, height))
        handle.write(pixels.astype(np.uint8).tobytes())


def bilinear(field, col, row):
    height, width = field.shape
    inside = ((col >= 0) & (col <= width - 1) &
              (row >= 0) & (row <= height - 1))
    col = np.clip(col, 0, width - 1)
    row = np.clip(row, 0, height - 1)
    c0 = np.floor(col).astype(int)
    r0 = np.floor(row).astype(int)
    c1 = np.minimum(c0 + 1, width - 1)
    r1 = np.minimum(r0 + 1, height - 1)
    fc = col - c0
    fr = row - r0
    value = (field[r0, c0] * (1 - fc) * (1 - fr) +
             field[r0, c1] * fc * (1 - fr) +
             field[r1, c0] * (1 - fc) * fr +
             field[r1, c1] * fc * fr)
    return value, inside


def resample(pixels, resolution, origin, yaw, dx, dy):
    height, width = pixels.shape
    occupied = (pixels < 50).astype(float)
    known = (pixels != 205).astype(float)

    cols = np.arange(width)
    rows = np.arange(height)
    grid_col, grid_row = np.meshgrid(cols, rows)
    x = origin[0] + (grid_col + 0.5) * resolution
    y = origin[1] + (height - 1 - grid_row + 0.5) * resolution

    cos, sin = math.cos(-yaw), math.sin(-yaw)
    xs = x - dx
    ys = y - dy
    x_old = cos * xs - sin * ys
    y_old = sin * xs + cos * ys

    col_old = (x_old - origin[0]) / resolution - 0.5
    row_old = (height - 1) - (y_old - origin[1]) / resolution + 0.5

    occ_value, inside = bilinear(occupied, col_old, row_old)
    known_value, _ = bilinear(known, col_old, row_old)

    result = np.full(pixels.shape, 205, dtype=np.uint8)
    usable = inside & (known_value >= 0.5)
    result[usable & (occ_value >= 0.5)] = 0
    result[usable & (occ_value < 0.5)] = 254
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='degrees, applied to the map content')
    parser.add_argument('--dx', type=float, default=0.0)
    parser.add_argument('--dy', type=float, default=0.0)
    parser.add_argument('--out', default='')
    parser.add_argument('--backup', action='store_true')
    parser.add_argument('--calibration', nargs=3, type=float, default=None,
                        metavar=('X', 'Y', 'YAW'),
                        help='frame_calibrate output (venue -> map); the '
                             'inverse is applied to the map')
    args = parser.parse_args()

    if args.calibration:
        tx, ty, dyaw = args.calibration
        yaw = -dyaw
        cos, sin = math.cos(yaw), math.sin(yaw)
        dx = -(cos * tx - sin * ty)
        dy = -(sin * tx + cos * ty)
    else:
        yaw = math.radians(args.yaw)
        dx, dy = args.dx, args.dy

    image, resolution, origin, rest = read_map(args.map)
    base = os.path.dirname(os.path.abspath(args.map))
    image_path = image if os.path.isabs(image) else os.path.join(base, image)
    pixels, width, height = read_pgm(image_path)

    before = int((pixels < 50).sum())
    rotated = resample(pixels, resolution, origin, yaw, dx, dy)
    after = int((rotated < 50).sum())
    print('map %dx%d res %.3f origin (%.2f, %.2f)' % (
        width, height, resolution, origin[0], origin[1]))
    print('applying yaw %.2f deg, shift (%.3f, %.3f) m'
          % (math.degrees(yaw), dx, dy))
    print('occupied %d -> %d (%.0f%% kept)' % (
        before, after, 100.0 * after / max(before, 1)))

    out_image = args.out or image_path
    if args.backup and out_image == image_path:
        shutil.copy2(image_path, image_path + '.orig')
        print('backup %s.orig' % image_path)
    write_pgm(out_image, rotated)
    print('wrote %s' % out_image)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
