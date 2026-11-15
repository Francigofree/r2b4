"""Bounded ROS-edge checks of the padded physical body in Nav2 OccupancyGrid.

100 is lethal, -1 unknown; 1..99 is inflation, not another body margin.
No ROS imports, device I/O, control-tick work or path planning.
"""
import math


def padded_footprint(bounds, padding):
    """Body Minkowski sum with a circumscribed 16-gon of apothem padding.

    Every direction retains at least the Euclidean body clearance. Axis-wise
    rectangle expansion instead demands sqrt(2) times that margin at corners.
    This same explicit polygon is supplied to both native Nav2 costmaps.
    """
    rear, front, right, left = bounds
    if (rear >= front or right >= left or padding < 0
            or not all(math.isfinite(v) for v in (*bounds, padding))):
        raise ValueError("invalid_footprint")
    if padding == 0:
        return [(front, left), (rear, left), (rear, right), (front, right)]
    radius = padding/math.cos(math.pi/16)
    return [((front if math.cos(a) > 0 else rear)+radius*math.cos(a),
             (left if math.sin(a) > 0 else right)+radius*math.sin(a))
            for a in ((2*k+1)*math.pi/16 for k in range(16))]


def _context(grid, bounds, padding):
    info = grid.info
    if (info.width <= 0 or info.height <= 0 or info.resolution <= 0
            or not math.isfinite(info.resolution) or len(grid.data) != info.width*info.height):
        raise ValueError("invalid_grid")
    q = info.origin.orientation
    if abs(q.x)+abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1.) > .01:
        raise ValueError("invalid_grid_origin")
    polygon = padded_footprint(bounds, padding)
    axes = []
    for p, following in zip(polygon, polygon[1:]+polygon[:1]):
        nx, ny = following[1]-p[1], p[0]-following[0]
        projections = [nx*x+ny*y for x, y in polygon]
        axes.append((nx, ny, min(projections), max(projections)))
    return grid, 2*math.atan2(q.z, q.w), polygon, axes


def _check(context, pose):
    grid, origin_yaw, polygon, axes = context
    info, res = grid.info, grid.info.resolution
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    dx, dy = pose[0]-info.origin.position.x, pose[1]-info.origin.position.y
    gx, gy = c*dx+s*dy, -s*dx+c*dy
    c, s = math.cos(pose[2]-origin_yaw), math.sin(pose[2]-origin_yaw)
    vertices = [(gx+c*x-s*y, gy+s*x+c*y) for x, y in polygon]
    minx, maxx = min(x for x, y in vertices), max(x for x, y in vertices)
    miny, maxy = min(y for x, y in vertices), max(y for x, y in vertices)
    rotated_axes = [(c*nx-s*ny, s*nx+c*ny, lo, hi) for nx, ny, lo, hi in axes]
    def blocked(reason, cell=None, cost=None):
        return dict(reason=reason, pose=list(pose), cell=cell, cost=cost)
    if minx < -1e-10 or miny < -1e-10 or maxx > info.width*res+1e-10 or maxy > info.height*res+1e-10:
        return blocked("outside_grid")
    x0, x1 = max(0, math.floor((minx-1e-10)/res)), min(info.width-1, math.floor((maxx+1e-10)/res))
    y0, y1 = max(0, math.floor((miny-1e-10)/res)), min(info.height-1, math.floor((maxy+1e-10)/res))
    if (x1-x0+1)*(y1-y0+1) > 16384:
        return blocked("footprint_cell_budget")
    r = res*.5
    for iy in range(y0, y1+1):
        for ix in range(x0, x1+1):
            cost = grid.data[iy*info.width+ix]
            if 0 <= cost < 100:
                continue
            dx, dy = (ix+.5)*res-gx, (iy+.5)*res-gy
            # Separating axes of the convex body and the complete square cell.
            if all(lo-r*(abs(nx)+abs(ny))-1e-10 <= nx*dx+ny*dy
                    <= hi+r*(abs(nx)+abs(ny))+1e-10 for nx, ny, lo, hi in rotated_axes):
                return blocked("unknown_cell" if cost == -1 else "lethal_cell" if cost == 100 else "invalid_cost", [ix, iy], int(cost))
    return None


def footprint_check(grid, x, y, yaw, bounds, padding):
    pose = (x, y, yaw)
    try:
        if not all(math.isfinite(v) for v in pose):
            raise ValueError("nonfinite_pose")
        return _check(_context(grid, bounds, padding), pose)
    except (ValueError, TypeError, AttributeError, OverflowError):
        return dict(reason="invalid_geometry", pose=list(pose), cell=None, cost=None)


def sweep_check(grid, start, v, w, duration, bounds, padding):
    """Exact arc, <= quarter-cell point displacement and <= 128 intervals.

    A <= eighth-cell sampling guard covers the gap between samples. The
    returned arc definition reconstructs every examined pose without payloads.
    """
    trajectory = dict(model="constant_twist_arc", start=list(start), v_mps=v,
        omega_rad_s=w, duration_s=duration, steps=0, sampling_guard_m=0.)
    result = dict(ok=False, trajectory=trajectory, first_blocking=None)
    try:
        if duration < 0 or not all(math.isfinite(x) for x in (*start, v, w, duration)):
            raise ValueError("invalid_motion")
        context = _context(grid, bounds, padding)
        radius = max(math.hypot(x, y) for x, y in context[2])
        travel = duration*(abs(v)+radius*abs(w))
        steps = max(1, math.ceil(travel/(grid.info.resolution*.25)))
        trajectory["steps"] = steps
        if steps > 128:
            raise ValueError("trajectory_sample_budget")
        guard = travel/(2*steps)
        trajectory["sampling_guard_m"] = guard
        context = _context(grid, bounds, padding+guard)
        for index in range(steps+1):
            t = duration*index/steps
            half = w*t*.5
            distance = v*t*(1. if abs(half) < 1e-12 else math.sin(half)/half)
            pose = (start[0]+distance*math.cos(start[2]+half),
                    start[1]+distance*math.sin(start[2]+half), start[2]+w*t)
            blocking = _check(context, pose)
            if blocking is not None:
                result["first_blocking"] = dict(blocking, sample=index, t_s=t)
                return result
        result["ok"] = True
    except (ValueError, TypeError, AttributeError, OverflowError) as exc:
        result["first_blocking"] = dict(reason=str(exc), pose=list(start), cell=None, cost=None)
    return result
