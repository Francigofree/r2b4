#!/usr/bin/env python3
"""R2B4 V3 passive ASCII L4 mini-map monitor. No commands are sent to the robot."""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


def parse(argv=None):
    p = argparse.ArgumentParser(description='R2B4 V3 local costmap - passive 5 Hz terminal view')
    p.add_argument('--status', default='runtime/v3_status.json')
    p.add_argument('--hz', type=float, default=5.0)
    p.add_argument('--width', type=int, default=61)
    p.add_argument('--height', type=int, default=31)
    p.add_argument('--max-age', type=float, default=0.75)
    return p.parse_args(argv)


def number(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (ValueError, TypeError):
        return None


def pose_info(status):
    e = status.get('estimate') or {}
    p = e.get('local_pose') or {}
    # Encode_value(Pose2D) uses x_m, y_m, yaw_rad.
    return (number(p.get('x_m')), number(p.get('y_m')), number(p.get('yaw_rad')))


def cell_xy(cell):
    if isinstance(cell, dict):
        return cell.get('grid_x'), cell.get('grid_y')
    if isinstance(cell, (tuple, list)) and len(cell) >= 2:
        return cell[0], cell[1]
    return None, None


def render(status, args):
    world = status.get('world') or {}
    costmap = world.get('local_costmap') or {}
    cells = costmap.get('occupied_cells')
    px, py, yaw = pose_info(status)
    resolution = number(costmap.get('resolution_m'))
    radius = number(costmap.get('radius_m'))
    stamp = status.get('monotonic_ns')
    age = (time.monotonic_ns() - stamp) / 1e9 if isinstance(stamp, int) else None
    ready = age is not None and 0 <= age <= args.max_age
    title = f"R2B4 V3 L4 MINIMAP | {args.hz:g} Hz | tick={status.get('tick_id','?')} | age={age:.2f}s" if age is not None else 'R2B4 V3 L4 MINIMAP | no timestamp'
    out = [title, f"runtime={'FRESH' if ready else 'STALE / OFFLINE'} | safety={status.get('safety_decision','?')} | L4 revision={costmap.get('revision','?')} | cells={costmap.get('occupied_cell_count','?')}"]
    if not ready:
        out.append('NO CURRENT MEASUREMENT: map/distance are not valid.')
        return '\n'.join(out)
    if not isinstance(cells, list):
        out.append('NO GEOMETRY EXPORT: native V3 status contains cell COUNT only.')
        out.append('Install the optional diagnostic patch, restart V3; never interpret this as a free map.')
        return '\n'.join(out)
    if None in (px,py,yaw,resolution) or resolution <= 0:
        out.append('Missing pose/resolution: cannot render a geometrically valid map.')
        return '\n'.join(out)
    w = max(11, min(151, args.width | 1)); h = max(9,min(75,args.height | 1))
    grid = [[' ' for _ in range(w)] for _ in range(h)]
    cx, cy = w//2, h//2
    # One terminal cell is one map cell, with a fixed 1:1 metric grid.
    nearest = None
    front = None
    for c in cells:
        gx,gy = cell_xy(c)
        if not isinstance(gx,int) or not isinstance(gy,int):
            continue
        wx,wy = (gx+0.5)*resolution, (gy+0.5)*resolution
        dx,dy=wx-px,wy-py
        forward=dx*math.cos(yaw)+dy*math.sin(yaw)
        left=-dx*math.sin(yaw)+dy*math.cos(yaw)
        # Convert to robot-forward-up, robot-left-left screen.
        sx=int(round(cx-left/resolution)); sy=int(round(cy-forward/resolution))
        if 0 <= sx < w and 0 <= sy < h:
            grid[sy][sx]='#'
        # Center-to-center estimate; not boundary/clearance and not raw LIDAR distance.
        dist=math.hypot(dx,dy)
        nearest=dist if nearest is None else min(nearest,dist)
        if forward > 0 and abs(left) <= 0.25:
            front=forward if front is None else min(front,forward)
    grid[cy][cx]='^'
    out.append(f"pose x={px:.2f} y={py:.2f} yaw={math.degrees(yaw):.1f}deg | map cell={resolution:.2f}m radius={radius}m")
    out.append(f"Nearest occupied cell CENTER: {nearest:.2f}m" if nearest is not None else 'Nearest occupied cell CENTER: no occupied cells (NOT proof of clearance)')
    out.append(f"Front occupied cell CENTER (±0.25m corridor): {front:.2f}m" if front is not None else 'Front occupied cell CENTER: none detected in corridor')
    out.append('NOTE: map-based cell centers, NOT LiDAR ray range or robot-boundary clearance.')
    out.append('^ robot/front; # occupied; blank UNKNOWN (not guaranteed free). Ctrl+C exit.')
    out.extend(''.join(row) for row in grid)
    return '\n'.join(out)


def main(argv=None):
    args = parse(argv)
    if not (0 < args.hz <= 20):
        raise SystemExit('--hz must be 0..20')
    path = Path(args.status).expanduser()
    interval=1/args.hz
    try:
        while True:
            start=time.monotonic()
            try:
                data=json.loads(path.read_text(encoding='utf-8'))
                display=render(data,args)
            except (OSError,ValueError,TypeError) as e:
                display=f'Waiting for valid status at {path}: {e}'
            sys.stdout.write('\033[H\033[2J'+display+'\n')
            sys.stdout.flush()
            time.sleep(max(0,interval-(time.monotonic()-start)))
    except KeyboardInterrupt:
        print('\nStopped (observer only).')

if __name__=='__main__':
    main()
