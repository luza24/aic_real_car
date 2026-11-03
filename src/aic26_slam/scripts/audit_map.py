"""Cross-check the maps, the virtual walls and the waypoints.

Prints each map's grid and how much of the track it knows, then for every
waypoint the clearance to the nearest painted line and to the nearest recorded
obstacle:

    python3 audit_map.py
    python3 audit_map.py --backend sim
"""

import argparse
import math

import numpy as np
import yaml

MAPS = '/home/mowen/aic26_ws_112/src/aic26_slam/maps/'
CONFIG = '/home/mowen/aic26_ws_112/src/aic26_decision/config/'
NAV = '/home/mowen/aic26_ws_112/src/aic26_navigation/config/'
RESOLUTION = 0.05
ORIGIN = (-6.0, -6.0)
FOOTPRINT_RADIUS = 0.196
WALL_CLEARANCE = 0.26


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


def world_cells(pixels, height):
    rows, cols = np.nonzero(pixels)
    return (ORIGIN[0] + cols * RESOLUTION,
            ORIGIN[1] + (height - 1 - rows) * RESOLUTION)


def cell_value(pixels, height, x, y):
    col = int(round((x - ORIGIN[0]) / RESOLUTION))
    row = height - 1 - int(round((y - ORIGIN[1]) / RESOLUTION))
    return int(pixels[row, col])


def segment_distance(px, py, segment):
    x1, y1, x2, y2 = segment
    dx, dy = x2 - x1, y2 - y1
    length = dx * dx + dy * dy
    t = 0.0 if length == 0 else ((px - x1) * dx + (py - y1) * dy) / length
    t = min(1.0, max(0.0, t))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', default='real', choices=('real', 'sim'))
    parser.add_argument('--track', type=float, default=2.05)
    args = parser.parse_args()

    maps = {}
    for name in ('world', 'world_sim', 'world_real'):
        pixels, width, height = read_pgm(MAPS + name + '.pgm')
        maps[name] = (pixels, height)
        xs, ys = world_cells(pixels < 50, height)
        print('%-11s %dx%d occ %4d free %5d unknown %5d '
              'extent x %.2f..%.2f y %.2f..%.2f'
              % (name, width, height, (pixels < 50).sum(),
                 (pixels == 254).sum(), (pixels == 205).sum(),
                 xs.min(), xs.max(), ys.min(), ys.max()))

    print('\nknown / free cells inside the %.2f m track:' % args.track)
    for name in ('world', 'world_sim', 'world_real'):
        pixels, height = maps[name]
        width = pixels.shape[1]
        grid_col, grid_row = np.meshgrid(np.arange(width), np.arange(height))
        x = ORIGIN[0] + (grid_col + 0.5) * RESOLUTION
        y = ORIGIN[1] + (height - 1 - grid_row + 0.5) * RESOLUTION
        inside = (np.abs(x) <= args.track) & (np.abs(y) <= args.track)
        known = inside & (pixels != 205)
        free = inside & (pixels == 254)
        print('  %-11s known %5d (%.0f%%)  free %5d (%.0f%%)  occupied %4d'
              % (name, known.sum(), 100.0 * known.sum() / inside.sum(),
                 free.sum(), 100.0 * free.sum() / inside.sum(),
                 (inside & (pixels < 50)).sum()))

    walls = yaml.safe_load(open(NAV + 'virtual_walls.yaml'))['segments']
    waypoints = yaml.safe_load(
        open(CONFIG + args.backend + '_waypoints.yaml'))['waypoints']
    other_name = 'sim' if args.backend == 'real' else 'real'
    other = {w['name']: w for w in yaml.safe_load(
        open(CONFIG + other_name + '_waypoints.yaml'))['waypoints']}

    print('\n%d wall segments, total %.2f m' % (
        len(walls), sum(math.hypot(s[2] - s[0], s[3] - s[1])
                        for s in walls)))
    print('\n%-16s %6s %6s %7s %7s %7s %s' % (
        'waypoint', 'x', 'y', 'line_m', 'real_m', 'sim_m', 'value r/s'))
    flags = []
    for wp in waypoints:
        name, x, y = wp['name'], wp['x'], wp['y']
        line = min(segment_distance(x, y, s) for s in walls)
        clearance = {}
        for key in ('world_real', 'world_sim'):
            xs, ys = world_cells(maps[key][0] < 50, maps[key][1])
            clearance[key] = float(np.min(np.hypot(xs - x, ys - y)))
        v_real = cell_value(maps['world_real'][0], maps['world_real'][1], x, y)
        v_sim = cell_value(maps['world_sim'][0], maps['world_sim'][1], x, y)
        mark = ''
        if v_real < 50:
            mark = ' GOAL-IN-WALL'
            flags.append('%s: goal cell occupied in world_real' % name)
        elif clearance['world_real'] < FOOTPRINT_RADIUS + 0.02:
            mark = ' tight'
            flags.append('%s: %.2f m from a recorded obstacle (footprint '
                         'radius %.2f)' % (name, clearance['world_real'],
                                           FOOTPRINT_RADIUS))
        if line < WALL_CLEARANCE:
            mark += ' close-to-line'
            flags.append('%s: %.2f m from a painted line (needs %.2f)'
                         % (name, line, WALL_CLEARANCE))
        print('%-16s %6.2f %6.2f %7.2f %7.2f %7.2f %4d/%d%s' % (
            name, x, y, line, clearance['world_real'], clearance['world_sim'],
            v_real, v_sim, mark))

    print('\n-- %s vs %s waypoints --' % (args.backend, other_name))
    biggest = 0.0
    for wp in waypoints:
        partner = other.get(wp['name'])
        if not partner:
            continue
        dx = wp['x'] - partner['x']
        dy = wp['y'] - partner['y']
        dyaw = wp['yaw'] - partner['yaw']
        biggest = max(biggest, abs(dx), abs(dy))
        if max(abs(dx), abs(dy)) >= 0.05 or abs(dyaw) >= 0.01:
            print('   %-16s dx %+.2f dy %+.2f dyaw %+.2f'
                  % (wp['name'], dx, dy, dyaw))
    print('   largest position difference %.2f m' % biggest)

    samples = 0
    on_obstacle = 0
    for segment in walls:
        length = math.hypot(segment[2] - segment[0], segment[3] - segment[1])
        steps = max(2, int(length / 0.025))
        for index in range(steps + 1):
            t = index / float(steps)
            x = segment[0] + (segment[2] - segment[0]) * t
            y = segment[1] + (segment[3] - segment[1]) * t
            samples += 1
            if cell_value(maps['world_real'][0], maps['world_real'][1],
                          x, y) < 50:
                on_obstacle += 1
    print('\nlines: %d of %d sampled points (%.1f%%) sit on a recorded '
          'obstacle cell' % (on_obstacle, samples,
                             100.0 * on_obstacle / max(samples, 1)))

    print('\nflags:')
    for flag in flags:
        print('   - ' + flag)
    if not flags:
        print('   none')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
