#!/usr/bin/env python3
"""Align a gmapping map to a reference map with a rigid 2-D transform.

The saved gmapping map is not in the static map frame when the robot is
placed at an arbitrary yaw.  This tool extracts occupied wall points from
both maps, estimates the rotation/translation with ICP (with a rotation
search for the initial guess uniquely, then writes the source map
resampled into the reference frame.

Usage:
  python3 map_align.py --reference maps/world.yaml \
      --source maps/world_real.yaml --output maps/world_aligned

  python3 map_align.py --selftest --reference maps/world.yaml

The self-test rotates/translates the reference by a known transform,
builds a synthetic source map, aligns it, and checks the recovered
source->reference transform.  It needs no robot and no ROS.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree


def load_map(path):
    """Load a map YAML and its PGM; return (array, meta, image_path)."""
    with open(path, 'r', encoding='utf-8') as fp:
        meta = yaml.safe_load(fp)
    image = meta.get('image', '')
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    arr = cv2.imread(image, cv2.IMREAD_UNCHANGED)
    if arr is None:
        # cv2 on Windows can fail silently on non-ASCII paths.
        data = np.fromfile(image, dtype=np.uint8)
        arr = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if arr is None:
        raise RuntimeError('cannot read image %s' % image)
    if arr.ndim == 3:
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    return arr, meta, image


def world_of(arr, meta):
    """Return the world (x, y) of every occupied cell."""
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'][:2]]
    h, w = arr.shape
    rows, cols = np.nonzero(arr < 100)
    x = ox + (cols + 0.5) * res
    y = oy + (h - 1 - rows + 0.5) * res
    return np.column_stack([x, y])


def rot2(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def apply_rt(points, R, t):
    return points.dot(R.T) + t


def icp(P_src, P_ref, R, t, iterations=30, trim=0.8):
    """Point-to-point ICP from source to reference."""
    for _ in range(iterations):
        Q = apply_rt(P_src, R, t)
        dist, idx = cKDTree(P_ref).query(Q, k=1)
        order = np.argsort(dist)
        keep = max(20, int(trim * len(order)))
        sel = order[:keep]
        src = P_src[sel]
        dst = P_ref[idx[sel]]
        cs = src.mean(axis=0)
        cr = dst.mean(axis=0)
        H = (src - cs).T.dot(dst - cr)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T.dot(U.T)
        if np.linalg.det(Rn) < 0.0:
            Vt[-1, :] *= -1.0
            Rn = Vt.T.dot(U.T)
        tn = cr - Rn.dot(cs)
        R, t = Rn, tn
    Q = apply_rt(P_src, R, t)
    dist, _ = cKDTree(P_ref).query(Q, k=1)
    keep = max(20, int(trim * len(dist)))
    score = float(np.mean(np.sort(dist)[:keep]))
    return R, t, score


def align_points(P_src, P_ref, max_rotation_deg, step_deg, trim, tries,
                 iterations):
    """Search yaw, run ICP, then refine the best candidate."""
    cs = P_src.mean(axis=0)
    cr = P_ref.mean(axis=0)
    best = None
    best_deg = 0.0
    for deg in np.arange(-max_rotation_deg, max_rotation_deg + 1e-9,
                         step_deg):
        R = rot2(math.radians(float(deg)))
        t = cr - R.dot(cs)
        R2, t2, score = icp(P_src, P_ref, R, t, iterations=tries,
                            trim=trim)
        if best is None or score < best[2]:
            best = (R2, t2, score)
            best_deg = float(deg)
    R, t, score = icp(P_src, P_ref, best[0], best[1],
                      iterations=iterations, trim=trim)
    return R, t, score, best_deg


def sample_world(arr, meta, X, Y):
    """Nearest-neighbour sample arr at world coordinates X, Y."""
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'][:2]]
    h, w = arr.shape
    cols = np.rint((X - ox) / res - 0.5).astype(np.int64)
    rows = np.rint((h - 0.5) - (Y - oy) / res).astype(np.int64)
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    out = np.full(X.shape, 205, dtype=np.uint8)  # unknown
    out[ok] = arr[rows[ok], cols[ok]]
    return out


def make_synthetic_source(ref_arr, ref_meta, deg, tx, ty):
    """Source = known rigid transform of the reference content."""
    R = rot2(math.radians(deg))
    t = np.array([tx, ty], dtype=np.float64)
    h, w = ref_arr.shape
    res = float(ref_meta['resolution'])
    ox, oy = [float(v) for v in ref_meta['origin'][:2]]
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    X = ox + (cols + 0.5) * res
    Y = oy + (h - 1 - rows + 0.5) * res
    P = np.column_stack([X.ravel(), Y.ravel()])
    # Source world = R * ref_world + t; inverse to sample the reference.
    PR = (R.T.dot((P - t).T)).T
    src = sample_world(ref_arr, ref_meta, PR[:, 0], PR[:, 1])
    return src.reshape(h, w), ref_meta


def write_aligned(src_arr, src_meta, ref_meta, ref_arr, R, t, output):
    """Resample source into the reference grid and write PGM/YAML."""
    h, w = ref_arr.shape
    res = float(ref_meta['resolution'])
    ox, oy = [float(v) for v in ref_meta['origin'][:2]]
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    X = ox + (cols + 0.5) * res
    Y = oy + (h - 1 - rows + 0.5) * res
    P = np.column_stack([X.ravel(), Y.ravel()])
    PS = (R.T.dot((P - t).T)).T
    out = sample_world(src_arr, src_meta, PS[:, 0], PS[:, 1])
    out = out.reshape(h, w)
    pgm = output + '.pgm'
    if not cv2.imwrite(pgm, out):
        data = cv2.imencode('.pgm', out)[1]
        data.tofile(pgm)
    meta = dict(ref_meta)
    meta['image'] = os.path.basename(pgm)
    meta['resolution'] = res
    meta['origin'] = [ox, oy, 0.0]
    meta.setdefault('negate', 0)
    meta.setdefault('occupied_thresh', 0.65)
    meta.setdefault('free_thresh', 0.196)
    with open(output + '.yaml', 'w', encoding='utf-8') as fp:
        yaml.safe_dump(meta, fp, sort_keys=False)
    return out, pgm, output + '.yaml'


def run_pair(source_yaml, reference_yaml, output, args):
    src_arr, src_meta, _ = load_map(source_yaml)
    ref_arr, ref_meta, _ = load_map(reference_yaml)
    P_src = world_of(src_arr, src_meta)
    P_ref = world_of(ref_arr, ref_meta)
    if len(P_src) < 20 or len(P_ref) < 20:
        raise RuntimeError('not enough occupied cells: src=%d ref=%d'
                           % (len(P_src), len(P_ref)))
    R, t, score, seed_deg = align_points(
        P_src, P_ref, args.max_rotation_deg, args.rotation_step,
        args.trim, args.tries, args.iterations)
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    print('source cells=%d reference cells=%d'
          % (len(P_src), len(P_ref)))
    print('rotation seed %.2f deg, refined %.4f deg'
          % (seed_deg, yaw))
    print('translation dx=%.4f dy=%.4f m, residual=%.4f m'
          % (t[0], t[1], score))
    print('source->reference: dx=%.4f dy=%.4f dyaw=%.4f rad (%.2f deg)'
          % (t[0], t[1], math.radians(yaw), yaw))
    if output:
        out, pgm, yml = write_aligned(
            src_arr, src_meta, ref_meta, ref_arr, R, t, output)
        both = np.zeros((out.shape[0], out.shape[1], 3), np.uint8)
        both[ref_arr < 100] = (0, 0, 255)
        both[out < 100] = (0, 255, 0)
        cv2.imwrite(output + '_overlay.png', both)
        print('wrote %s and %s' % (pgm, yml))
    return R, t, score, yaw


def selftest(reference_yaml, output, args):
    ref_arr, ref_meta, _ = load_map(reference_yaml)
    known_deg = 7.0
    known_tx, known_ty = 0.18, -0.12
    src_arr, src_meta = make_synthetic_source(
        ref_arr, ref_meta, known_deg, known_tx, known_ty)
    tmp = (output + '_source') if output else 'map_align_selftest_source'
    pgm = tmp + '.pgm'
    cv2.imwrite(pgm, src_arr)
    meta = dict(ref_meta)
    meta['image'] = os.path.basename(pgm)
    with open(tmp + '.yaml', 'w', encoding='utf-8') as fp:
        yaml.safe_dump(meta, fp, sort_keys=False)
    R, t, score, yaw = run_pair(
        tmp + '.yaml', reference_yaml, output or (tmp + '_aligned'), args)

    # The known transform maps reference -> source.  Alignment returns
    # source -> reference, whose expected values are the inverse.
    Rk = rot2(math.radians(known_deg))
    tk = np.array([known_tx, known_ty], dtype=np.float64)
    Re = Rk.T
    te = -Rk.T.dot(tk)
    yaw_e = math.degrees(math.atan2(Re[1, 0], Re[0, 0]))
    err_yaw = abs(yaw - yaw_e)
    err_t = float(np.linalg.norm(t - te))
    print('selftest known ref->src: %.2f deg, t=(%.3f, %.3f)'
          % (known_deg, known_tx, known_ty))
    print('selftest expected src->ref: %.4f deg, t=(%.4f, %.4f)'
          % (yaw_e, te[0], te[1]))
    print('selftest recovered: %.4f deg, t=(%.4f, %.4f), residual=%.4f'
          % (yaw, t[0], t[1], score))
    print('selftest error: dyaw=%.4f deg, dt=%.4f m'
          % (err_yaw, err_t))
    ok = err_yaw < 0.5 and err_t < 0.03
    print('SELFTEST %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True,
                        help='static/reference map YAML')
    parser.add_argument('--source', help='gmapping map YAML to align')
    parser.add_argument('--output', help='output prefix (PGM/YAML)')
    parser.add_argument('--selftest', action='store_true',
                        help='align a synthetic rotated copy of reference')
    parser.add_argument('--max-rotation-deg', type=float, default=30.0)
    parser.add_argument('--rotation-step', type=float, default=1.0)
    parser.add_argument('--trim', type=float, default=0.8)
    parser.add_argument('--tries', type=int, default=5)
    parser.add_argument('--iterations', type=int, default=30)
    args = parser.parse_args()
    if args.selftest:
        return selftest(args.reference, args.output, args)
    if not args.source or not args.output:
        parser.error('--source and --output are required without --selftest')
    run_pair(args.source, args.reference, args.output, args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
