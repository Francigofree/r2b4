#!/usr/bin/env python3
"""Egyszer indithato, csak olvaso SLAM / tenyleges NAV2 local costmap JPG exporter.

A rovid ideju ROS megfigyeles utan a program leall; nincs service, daemon.
Az export offline is kepes a MAR ROGZITETT terkepadatokbol dolgozni;
it never substitutes a global-map crop for a missing Nav2 local costmap.

Source aligned with Francigofree/r2b4 HEAD a2b44c2 (2026-11-11):
  /map                        nav_msgs/OccupancyGrid (SLAM)
  /local_costmap/costmap      nav_msgs/OccupancyGrid (ACTUAL Nav2)
  /global_costmap/costmap     nav_msgs/OccupancyGrid (Nav2 global costs)
  /r2b4/odom                 nav_msgs/Odometry (local trajectory)
  map->base_link             TF, R2B4_BOOT_ROBOT_MAP (global trajectory)
  runtime/nav2_planner/bridge_status.json (selected goal/provider/diagnostics)
  /dev/shm/.../v3_status.json (passive V3 status)

No motion command, motor authority, or modification of the V3 control chain.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any

# Ez a fajl masolhato kozvetlenul a repo tools/ mappajaba.
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

SCHEMA = "R2B4_MAP_OBSERVER_V1"
GLOBAL_FRAME = "R2B4_BOOT_ROBOT_MAP"
LOCAL_FRAME = "R2B4_ODOM_LOCAL"
TOPICS = {
    "global": "/map",
    "local": "/local_costmap/costmap",
    "global_costmap": "/global_costmap/costmap",
}
FRAME = {"global": GLOBAL_FRAME, "local": LOCAL_FRAME, "global_costmap": GLOBAL_FRAME}
MAX_CELLS = 250_000
MAX_EVENTS = 120_000
MAX_EXPORT_GOALS = 25


def root_path(root: str | Path | None = None) -> Path:
    return Path(root).expanduser().resolve() if root else Path(__file__).resolve().parents[1]


def output_dir(root: Path) -> Path:
    path = root / "runtime" / "terkep"
    path.mkdir(parents=True, exist_ok=True)
    return path


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stamp(message: Any) -> int:
    return int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)


def utc_label(epoch_ns: int) -> str:
    if not isinstance(epoch_ns, int) or epoch_ns <= 0:
        raise ValueError("Térkép forrás-időbélyeg hiányzik")
    dt = datetime.fromtimestamp(epoch_ns // 1_000_000_000, tz=timezone.utc)
    return dt.strftime("%Y%m%dT%H%M%S") + f".{epoch_ns // 1_000_000 % 1000:03d}Z"


def utc_human(epoch_ns: int) -> str:
    return datetime.fromtimestamp(epoch_ns / 1e9, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " UTC"


def yaw_from_quat(q: Any) -> float:
    values = [float(q.x), float(q.y), float(q.z), float(q.w)]
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1.0) > .02:
        raise ValueError("érvénytelen orientáció")
    x, y, z, w = values
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def validated_grid(document: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA:
        raise ValueError("ismeretlen térkép formátum")
    typ = document.get("kind")
    if typ not in TOPICS or document.get("source_topic") != TOPICS[typ]:
        raise ValueError("térkép forrása nem bizonyított")
    w, h, res, source_ns = (document.get(x) for x in ("width", "height", "resolution_m", "source_stamp_ns"))
    if (type(w) is not int or type(h) is not int or not 0 < w*h <= MAX_CELLS
            or not finite(res) or not .001 <= res <= 1.0
            or type(source_ns) is not int or source_ns < 1_600_000_000_000_000_000
            or document.get("frame_id") != FRAME[typ]):
        raise ValueError("térkép geometria / frame / forrás-idő hibás")
    origin = document.get("origin")
    if not isinstance(origin, list) or len(origin) != 3 or not all(finite(v) for v in origin):
        raise ValueError("térképorigin hibás")
    cells = document.get("cells")
    if not isinstance(cells, list) or len(cells) != w*h:
        raise ValueError("térképcellák hiányoznak")
    if any(type(v) is not int or v < -1 or v > 100 for v in cells):
        raise ValueError("érvénytelen OccupancyGrid cellák")
    return document


def grid_from_ros(message: Any, kind: str, session_id: str) -> dict[str, Any]:
    if kind not in TOPICS or message.header.frame_id != FRAME[kind]:
        raise ValueError("nem egyező ROS térképkeret")
    origin = message.info.origin
    item = {
        "schema": SCHEMA, "kind": kind, "source_topic": TOPICS[kind],
        "frame_id": message.header.frame_id, "source_stamp_ns": stamp(message),
        "received_wall_ns": time.time_ns(), "session_id": session_id,
        "width": int(message.info.width), "height": int(message.info.height),
        "resolution_m": float(message.info.resolution),
        "origin": [float(origin.position.x), float(origin.position.y), yaw_from_quat(origin.orientation)],
        "cells": [int(v) for v in message.data],
        "source_kind": "ACTUAL_ROS_OCCUPANCYGRID",
    }
    return validated_grid(item)


def atomic_gzip_json(path: Path, item: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, name = tempfile.mkstemp(prefix=".map-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(handle, "wb") as f:
            with gzip.GzipFile(fileobj=f, mode="wb", compresslevel=4) as g:
                g.write(json.dumps(item, allow_nan=False, separators=(",", ":")).encode())
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def atomic_json(path: Path, item: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".report-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(item, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def load_cached_map(root: Path, kind: str) -> dict | None:
    p = output_dir(root) / "cache" / f"{kind}.json.gz"
    if not p.is_file() or p.is_symlink() or p.stat().st_size > 2_000_000:
        return None
    try:
        with gzip.open(p, "rt", encoding="utf-8") as stream:
            row = json.load(stream)
        return validated_grid(row)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None


def load_saved_slam(root: Path) -> dict | None:
    """Latest saved SLAM PGM: source timestamp MUST come from same-stem .json."""
    directory = root / "runtime" / "slam_toolbox" / "maps"
    candidates = []
    for meta in directory.glob("*.json") if directory.is_dir() else []:
        try:
            j = json.loads(meta.read_text(encoding="utf-8"))
            source_ns = j["stamp_sec"]*1_000_000_000 + j.get("stamp_nanosec", 0)
            if type(source_ns) is not int or source_ns <= 0:
                continue
            candidates.append((source_ns, meta.with_suffix(".pgm"), meta.with_suffix(".yaml"), j))
        except (OSError, KeyError, ValueError, TypeError):
            continue
    for source_ns, pgm, yaml, info in sorted(candidates, reverse=True):
        if not pgm.is_file() or not yaml.is_file() or pgm.is_symlink():
            continue
        try:
            # map_saver PGM P5: 0 occupied, 254 free, 205 unknown. No pseudo-cells.
            with pgm.open("rb") as file:
                if file.readline().strip() != b"P5":
                    raise ValueError("PGM P5 required")
                dims = file.readline().split()
                while dims and dims[0].startswith(b"#"):
                    dims = file.readline().split()
                w, h = map(int, dims)
                if file.readline().strip() != b"255" or not 0 < w*h <= MAX_CELLS:
                    raise ValueError("invalid saved PGM")
                b = file.read()
            if len(b) != w*h:
                raise ValueError("truncated PGM")
            # yaml dependency intentionally not required
            lines = yaml.read_text(encoding="utf-8").splitlines()
            res = next(float(s.split(":", 1)[1]) for s in lines if s.startswith("resolution:"))
            ori = next(s.split(":", 1)[1].strip() for s in lines if s.startswith("origin:"))
            origin = json.loads(ori)
            cells = [-1 if v == 205 else 0 if v == 254 else 100 if v == 0 else -1 for v in b]
            # PGM first row is TOP, OccupancyGrid first row is BOTTOM.
            cells = [x for row in range(h-1, -1, -1) for x in cells[row*w:(row+1)*w]]
            d = {"schema": SCHEMA, "kind": "global", "source_topic": "/map",
                "frame_id": info["frame_id"], "source_stamp_ns": source_ns,
                "received_wall_ns": 0, "session_id": None,
                "width": w, "height": h, "resolution_m": res, "origin": origin,
                "cells": cells, "source_kind": "SAVED_SLAM_PGM", "file": str(pgm)}
            return validated_grid(d)
        except (OSError, TypeError, ValueError, KeyError, StopIteration):
            continue
    return None


def choose_global_map(root: Path) -> dict | None:
    a, b = load_cached_map(root, "global"), load_saved_slam(root)
    if a is None:
        return b
    if b is None:
        return a
    return a if a["source_stamp_ns"] >= b["source_stamp_ns"] else b



def transform_xy(pose: list, tx: float, ty: float, yaw: float) -> list:
    """Apply verified TF target<-source to a 2-D source-frame goal."""
    if (not isinstance(pose,list) or len(pose)<2
            or not all(finite(v) for v in (pose[0],pose[1],tx,ty,yaw))):
        raise ValueError("invalid goal/TF transform")
    c,s=math.cos(yaw),math.sin(yaw)
    x=tx+c*pose[0]-s*pose[1]
    y=ty+s*pose[0]+c*pose[1]
    a=pose[2] if len(pose)>2 else None
    return [x,y,None if not finite(a) else math.atan2(math.sin(a+yaw),math.cos(a+yaw))]


def world_to_pixel(point: tuple[float, float], grid: dict, scale: int = 1) -> tuple[float, float]:
    x, y = point
    ox, oy, theta = grid["origin"]
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = x-ox, y-oy
    gx, gy = (c*dx+s*dy)/grid["resolution_m"], (-s*dx+c*dy)/grid["resolution_m"]
    return gx*scale, (grid["height"]-gy)*scale


def read_events(root: Path, session_id: str | None, until_ns: int) -> list[dict]:
    if not session_id or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in session_id):
        return []
    p = output_dir(root) / "cache" / "sessions" / f"{session_id}.jsonl"
    if not p.is_file() or p.is_symlink() or p.stat().st_size > 80_000_000:
        return []
    events = []
    try:
        with p.open(encoding="utf-8") as stream:
            for line in stream:
                if len(events) >= MAX_EVENTS:
                    break
                if len(line) > 32768:
                    continue
                try:
                    row = json.loads(line)
                    if (isinstance(row, dict) and type(row.get("source_stamp_ns")) is int
                            and row["source_stamp_ns"] <= until_ns + 1_000_000_000):
                        events.append(row)
                except (ValueError, TypeError):
                    continue
    except OSError:
        pass
    return events


def path_for_frame(events: list[dict], frame: str) -> list[dict]:
    path = [event for event in events if event.get("type") == "pose" and event.get("frame_id") == frame
            and all(finite(event.get(c)) for c in ("x_m", "y_m", "yaw_rad"))]
    path.sort(key=lambda p: p["source_stamp_ns"])
    return path


def goals_for_frame(events: list[dict], frame: str) -> list[dict]:
    goals = []
    for e in events:
        if e.get("type") != "goal" or not isinstance(e.get("goal"), list):
            continue
        if frame == GLOBAL_FRAME:
            goal = e["goal"]
        else:
            goal = e.get("local_goal")
        if isinstance(goal, list) and len(goal) >= 2 and all(finite(z) for z in goal[:2]):
            goals.append({**e, "draw_goal": goal[:2]})
    return goals[-MAX_EXPORT_GOALS:]


def clip_string(x: Any, maxlen: int = 84) -> str:
    return str(x).replace("\n", " ")[:maxlen]


def summarize(events: list[dict]) -> dict:
    decisions = [e for e in events if e.get("type") in ("v3_decision", "nav2_decision", "nav2_error")]
    nav2 = [e for e in events if e.get("type") == "nav2_observation"]
    goals = [e for e in events if e.get("type") == "goal"]
    return {"goal_count": len(goals), "decision_count": len(decisions),
            "recent_decisions": decisions[-50:], "last_nav2_observation": nav2[-1] if nav2 else None}


def source_provenance(root: Path) -> dict[str, Any]:
    """Lightweight reproducibility references, gathered only during export."""
    result={"config_sha256":{}}
    for rel in ("conf/nav2_planner.yaml", "conf/slam_toolbox.yaml",
                "conf/vezerles.json", "runtime/nav2_planner/geometry.yaml"):
        p=root/rel
        try:
            if p.is_file() and not p.is_symlink() and p.stat().st_size < 2_000_000:
                result["config_sha256"][rel]=hashlib.sha256(p.read_bytes()).hexdigest()
        except OSError:
            continue
    try:
        value=(root/"runtime/.r2b4_capture_path").read_text(encoding="utf-8")[:1024].strip()
        if value:result["capture_hint"] = value
    except OSError:
        pass
    try:
        git=subprocess.run(["git","-C",str(root),"rev-parse","HEAD"],capture_output=True,
                           text=True,timeout=2)
        if git.returncode == 0:result["git_head"]=git.stdout.strip()
    except (OSError,subprocess.TimeoutExpired):
        pass
    return result


def render(root: Path, kind: str, grid: dict, *, pixels_per_cell: int = 9) -> dict:
    """Render exact occupancy values, then add NAV2/V3 evidence as annotations."""
    from PIL import Image, ImageDraw, ImageFont
    grid = validated_grid(grid)
    events = read_events(root, grid.get("session_id"), grid["source_stamp_ns"])
    path = path_for_frame(events, grid["frame_id"])
    run_started=any(e.get("type")=="mission_start" for e in events)
    run_stopped=any(e.get("type")=="mission_stop" for e in events)
    starts=[e["source_stamp_ns"] for e in events if e.get("type")=="mission_start"]
    # Retain the latest mission's trajectory when one was actually observed.
    if starts:
        latest_start=starts[-1]
        path=[p for p in path if p["source_stamp_ns"]>=latest_start-500_000_000]
    goals = goals_for_frame(events, grid["frame_id"])
    stats = summarize(events)
    width, height = grid["width"], grid["height"]
    # Fixed image dimensions, not a recomputed local/global map.
    zoom = max(2, min(10, min(1600//width, 1400//height, pixels_per_cell)))
    im_w, im_h = width*zoom, height*zoom
    padding = 26
    panel = 470
    image = Image.new("RGB", (im_w+padding*2+panel, max(1180, im_h+padding*2+105)), (247,248,251))
    surface = Image.new("RGB", (width,height))
    rgb = []
    unknown = free = occupied = costs = 0
    for v in grid["cells"]:
        if v < 0:
            rgb.append((165,173,184)); unknown += 1
        elif v == 0:
            rgb.append((251,251,251)); free += 1
        elif v >= 99:
            rgb.append((32,40,53)); occupied += 1
        elif v >= 70:
            rgb.append((221,92,63)); costs += 1
        else:
            # actual inflated Nav2 costs: yellow to orange, NOT physical walls
            rgb.append((255, int(235-90*v/100), int(180-125*v/100))); costs += 1
    surface.putdata(rgb)
    surface = surface.transpose(Image.Transpose.FLIP_TOP_BOTTOM).resize((im_w, im_h), Image.Resampling.NEAREST)
    map_x, map_y = padding, 100
    image.paste(surface, (map_x,map_y))
    d = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 17)
        small = ImageFont.truetype("DejaVuSans.ttf", 13)
        title = ImageFont.truetype("DejaVuSans-Bold.ttf", 22)
    except OSError:
        font = small = title = ImageFont.load_default()
    kind_label = "GLOBAL / SLAM map" if kind == "global" else "LOCAL / ACTUAL NAV2 costmap"
    d.text((padding,16), kind_label, fill=(25,30,44), font=title)
    d.text((padding,49),"Map measurement: "+utc_human(grid["source_stamp_ns"]), fill=(55,65,75),font=font)
    d.text((padding,74),"Frame: "+grid["frame_id"]+" | topic: "+grid["source_topic"],fill=(65,74,83),font=small)
    panel_x = map_x+im_w+25
    def line(text, y, *, color=(40,48,61), weight=False):
        d.text((panel_x,y),clip_string(text,57),font=font if weight else small,fill=color)
    d.rectangle([map_x-1,map_y-1,map_x+im_w,map_y+im_h],outline=(73,82,94),width=2)

    # Coordinates are in the native ROS frame. No unverified cross-frame joins.
    def screen(x, y):
        px, py = world_to_pixel((x,y), grid, zoom)
        return map_x+px, map_y+py
    valid_path = [p for p in path if -30 <= screen(p["x_m"],p["y_m"])[0]-map_x <= im_w+30
                  and -30 <= screen(p["x_m"],p["y_m"])[1]-map_y <= im_h+30]
    if len(valid_path) > 1:
        d.line([screen(p["x_m"],p["y_m"]) for p in valid_path],fill=(17,111,212),width=max(2,zoom//2),joint="curve")

    def arrow(pose, color, caption):
        x, y = screen(pose["x_m"],pose["y_m"])
        if not (map_x <= x <= map_x+im_w and map_y <= y <= map_y+im_h):
            return False
        rad = max(5, zoom*1.2)
        yaw = pose["yaw_rad"]-grid["origin"][2]
        # Image y axis points down; local grid y points up.
        end = (x+1.8*rad*math.cos(yaw),y-1.8*rad*math.sin(yaw))
        d.ellipse([x-rad,y-rad,x+rad,y+rad],fill=color,outline=(255,255,255),width=2)
        d.line([(x,y),end],fill=color,width=4)
        d.text((x+rad+4,y-18),caption,font=small,fill=color,stroke_width=2,stroke_fill=(255,255,255))
        return True
    for index,g in enumerate(goals):
        x,y = screen(g["draw_goal"][0],g["draw_goal"][1])
        if map_x <= x <= map_x+im_w and map_y <= y <= map_y+im_h:
            r = max(5,zoom)
            d.line((x-r,y,x+r,y),fill=(171,55,153),width=3)
            d.line((x,y-r,x,y+r),fill=(171,55,153),width=3)
            d.text((x+r+2,y-r-10),"G"+str(index+1),fill=(171,55,153),font=small,
                   stroke_width=2,stroke_fill=(255,255,255))
    if path:
        arrow(path[0],(13,152,87),"FIRST OBSERVED")
        arrow(path[-1],(215,59,55),"LAST OBSERVED")
    # Correct ROS-grid distances, not pixel distance or SLAM-estimated footprint.
    span = min(1.0, width*grid["resolution_m"]/4)
    x1,y1 = map_x+17,map_y+im_h-22
    x2=x1+span/grid["resolution_m"]*zoom
    d.line((x1,y1,x2,y1),fill=(20,20,20),width=4)
    d.text((x1,y1-20),f"{span:g} m",fill=(0,0,0),font=small,stroke_width=2,stroke_fill=(255,255,255))

    y = 105
    line("SOURCE / COORDINATES",y,weight=True);y+=31
    for s in (f"{width} x {height} @ {grid['resolution_m']:.3f} m/cell",
              f"Origin: ({grid['origin'][0]:.2f}, {grid['origin'][1]:.2f}) m",
              f"Origin yaw: {math.degrees(grid['origin'][2]):+.2f} deg",
              f"Map stamp: {utc_label(grid['source_stamp_ns'])}",
              f"Source: {grid.get('source_kind')}",f"Session: {grid.get('session_id') or 'unknown'}"):
        line(s,y);y+=23
    y+=5
    line("CELLS (UNKNOWN IS NOT FREE)",y,weight=True);y+=28
    for s in (f"Unknown: {unknown}  |  Free: {free}",f"Occupied/lethal: {occupied}",f"Inflated costs: {costs}",
              "Dark = occupied; gray = unknown", "Yellow/red = NAV2 cost/inflation"):
        line(s,y);y+=22
    y+=7
    line("TRAJECTORY / HEADING",y,weight=True);y+=28
    line(f"Mission start observed: {run_started}",y);y+=21
    line(f"Mission stop observed: {run_stopped}",y);y+=21
    if path:
        for label,p in (("START",path[0]),("END",path[-1])):
            line(f"{label} {p['x_m']:+.2f}, {p['y_m']:+.2f} m",y);y+=21
            line(f"  yaw {math.degrees(p['yaw_rad']):+.1f} deg",y);y+=21
        meters = sum(math.dist((a["x_m"],a["y_m"]),(b["x_m"],b["y_m"]))
                     for a,b in zip(path,path[1:]))
        line(f"Route length (sampled): {meters:.2f} m",y);y+=22
        line(f"Path samples: {len(path)}",y);y+=22
    else:
        line("NO SAME-FRAME PATH EVIDENCE",y,color=(178,56,56));y+=25
    y+=9
    line("GOALS (OBSERVER TIMESTAMP)",y,weight=True);y+=28
    if not goals:
        line("No attributable goal records",y);y+=23
    for idx,g in enumerate(goals[-8:]):
        clock=datetime.fromtimestamp(g["source_stamp_ns"]/1e9,timezone.utc).strftime("%H:%M:%S")
        line(f"G{max(1,len(goals)-8)+idx} {clock} {g.get('provider','?')}",y);y+=21
        line(f"  {g['draw_goal'][0]:+.2f},{g['draw_goal'][1]:+.2f} m",y);y+=23
    y+=8
    line("NAV / DIAGNOSTIC EVENTS",y,weight=True);y+=28
    line(f"Decisions captured: {stats['decision_count']}",y);y+=22
    n=stats["last_nav2_observation"]
    if n:
        for key in ("cruise_phase", "frontier_status", "state", "sensor_tf_age_ms", "controller_hz"):
            if n.get(key) is not None:
                line(f"{key}: {n.get(key)}",y);y+=22
    for event in stats["recent_decisions"][-6:]:
        label=" / ".join(str(event.get(k, "")) for k in ("type","reason") if event.get(k) is not None)
        line("- "+label,y);y+=22
    if grid.get("session_id") is None:
        line("Saved PGM: no run association",y+8,color=(177,60,48))
    # Bottom plain-text legend, no claim about robot physical position if absent.
    d.text((padding,map_y+im_h+15),
           "Blue: measured route  |  Green: start  |  Red: end  |  Magenta: goal  |  Map remains original ROS data",
           fill=(50,59,71),font=small)
    out = output_dir(root)
    label = "global" if kind=="global" else "local"
    target = out / f"{label}-{utc_label(grid['source_stamp_ns'])}.jpg"
    image.save(target,format="JPEG",quality=94,subsampling=0)
    sidecar = target.with_suffix(".json")
    record = {"schema": SCHEMA,"report_kind":label,"map":{k:v for k,v in grid.items() if k != "cells"},
              "cell_histogram":{"unknown":unknown,"free":free,"occupied":occupied,"inflation":costs},
              "mission_start_observed":run_started,"mission_stop_observed":run_stopped,
              "provenance":source_provenance(root),
              "source_to_received_ms":(grid["received_wall_ns"]-grid["source_stamp_ns"])/1e6
                 if grid.get("received_wall_ns",0)>0 else None,
              "path":path,"goals":goals,"navigation":stats,
              "warnings": ([] if path else ["NO_PATH_SAME_FRAME_SESSION"])
                          + ([] if run_started else ["START_POSITION_ONLY_FIRST_OBSERVED_NOT_PROVEN_MISSION_START"])
                          + ([] if run_stopped else ["END_POSITION_ONLY_LAST_OBSERVED_NOT_PROVEN_MISSION_END"])
                          + (["SAVED_PGM_UNLINKED_TO_NAVIGATION_SESSION"] if not grid.get("session_id") else []),
              "timestamp_policy":"SOURCE_HEADER_STAMP_UTC_NOT_EXPORT_TIME",
              "coordinate_policy":"ONLY_NATIVE_MATCHING_FRAME; NO_FABRICATED_ALIGNMENT"}
    atomic_json(sidecar,record)
    return {"jpg":str(target),"data":str(sidecar),"source_stamp":utc_human(grid["source_stamp_ns"]),
            "source":grid.get("source_kind"),"path_samples":len(path),"goal_samples":len(goals)}


def export(root: Path, *, kinds=("local","global"), strict=False) -> dict:
    results = {}; errors = {}
    for kind in kinds:
        grid = load_cached_map(root,"local") if kind=="local" else choose_global_map(root)
        if grid is None:
            errors[kind] = ("NINCS VALÓDI NAV2 LOKÁLIS COSTMAP ARCHÍVUM: korábbi /local_costmap/costmap "
                            "rögzítés kell, a SLAM kivágatát tilos helyettesíteni." if kind=="local" else
                            "Nincs SLAM /map megfigyelés vagy PGM mentés")
            continue
        results[kind]=render(root,kind,grid)
    response={"result":results,"missing":errors,"output_dir":str(output_dir(root))}
    if strict and errors:
        raise RuntimeError(json.dumps(response,ensure_ascii=False))
    return response

class Archive:
    """Single writer for passive ROS source observations and navigation evidence.

    Caller: ONLY map-report observer, never control/ROS bridge. Map files are
    replace-atomically saved on every delivered source revision; path/event
    records are append-only for the observed Nav2 bridge generation.
    """

    def __init__(self, root: Path):
        self.root = root
        self.cache = output_dir(root) / "cache"
        (self.cache / "sessions").mkdir(parents=True, exist_ok=True)
        self.session_id = "observer_" + utc_label(time.time_ns()).replace(".", "_").replace("Z","")
        self.map_stamps = {}
        self.last_pose = {}
        self.prev_state = None
        self.prev_v3 = None
        self.last_goal = None
        self.previous_mission = None
        self.goal_to_local = None
        self.nav2_errors_seen = set()
        self.last_status_poll_ns = 0
        self.last_v3_poll_ns = 0
        self.last_observation_ns = 0
        self.last_nav2_observation_poll_ns = 0
        self.ros_map_valid_count = 0
        self.ros_local_valid_count = 0

    def event(self, item: dict) -> None:
        if type(item.get("source_stamp_ns")) is not int or item["source_stamp_ns"] <= 0:
            return
        item = {"schema":SCHEMA,"session_id":self.session_id,**item}
        dest = self.cache / "sessions" / f"{self.session_id}.jsonl"
        with dest.open("a",encoding="utf-8") as f:
            f.write(json.dumps(item,ensure_ascii=False,allow_nan=False,separators=(",",":"))+"\n")
        self.last_observation_ns=max(self.last_observation_ns,item["source_stamp_ns"])

    def map(self,message: Any,kind: str) -> bool:
        source=stamp(message)
        if source <= self.map_stamps.get(kind,0):
            return False
        try:
            item = grid_from_ros(message,kind,self.session_id)
        except (ValueError,AttributeError,TypeError,OverflowError):
            return False
        if item["source_stamp_ns"] > time.time_ns()+5_000_000_000:
            return False
        # A one-shot export and the always-on observer may run concurrently.
        # Do not let an old latched ROS frame replace a newer saved snapshot.
        with (self.cache/f".{kind}.lock").open("a+b") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            previous=load_cached_map(self.root,kind)
            if previous is not None and previous["source_stamp_ns"]>=source:
                self.map_stamps[kind]=source
                return False
            atomic_gzip_json(self.cache/f"{kind}.json.gz", item)
        self.map_stamps[kind]=source
        if kind=="global":self.ros_map_valid_count+=1
        if kind=="local":self.ros_local_valid_count+=1
        return True

    def pose(self, frame_id: str, source_ns: int, x: float, y: float, yaw: float) -> bool:
        if frame_id not in (GLOBAL_FRAME,LOCAL_FRAME) or not all(finite(v) for v in (x,y,yaw)):
            return False
        previous=self.last_pose.get(frame_id)
        if previous is not None:
            if source_ns <= previous[0]:
                return False
            if source_ns-previous[0] < 190_000_000:
                return False
            # On reset, refuse to join separate odom coordinate generations.
            if math.hypot(x-previous[1],y-previous[2])>1.5 or source_ns-previous[0] > 10_000_000_000:
                self.event({"type":"pose_gap","source_stamp_ns":source_ns,"frame_id":frame_id,
                            "reason":"TIME_GAP_OR_POSE_DISCONTINUITY"})
        self.last_pose[frame_id]=(source_ns,x,y,yaw)
        self.event({"type":"pose","frame_id":frame_id,"source_stamp_ns":source_ns,
                    "x_m":float(x),"y_m":float(y),"yaw_rad":float(yaw)})
        return True

    def switch_session(self,generation: int) -> None:
        new="nav2_"+str(generation)
        if self.session_id == new:return
        self.session_id=new
        self.map_stamps.clear()
        self.last_pose.clear()
        self.prev_state=None
        self.prev_v3=None
        self.last_goal=None
        self.previous_mission=None
        self.nav2_errors_seen.clear()
        self.last_nav2_observation_poll_ns=0
        atomic_json(self.cache/"current_session.json",{"session_id":new,"changed_wall_ns":time.time_ns()})

    def poll_status(self) -> None:
        now=time.monotonic_ns()
        if now-self.last_status_poll_ns<200_000_000:return
        self.last_status_poll_ns=now
        p=self.root/"runtime"/"nav2_planner"/"bridge_status.json"
        try:
            if p.stat().st_size >131072:return
            st=json.loads(p.read_text(encoding="utf-8"))
        except (OSError,ValueError):return
        measured=st.get("observed_monotonic_ns")
        if type(measured) is not int or not 0 <= now-measured <= 2_000_000_000:
            return  # never re-label stale last-run status as current
        generation=st.get("worker_generation")
        if type(generation) is int and generation>0:
            self.switch_session(generation)
        # Accurate source event time is NOT available from a latest-only bridge
        # file. These are explicitly observation times.
        wall=time.time_ns()
        current_mission=st.get("mission_id")
        if current_mission != self.previous_mission:
            if self.previous_mission:
                self.event({"type":"mission_stop","source_stamp_ns":wall,
                    "mission_id":self.previous_mission,"timestamp_semantics":"OBSERVER_RECEIPT"})
            if current_mission:
                self.event({"type":"mission_start","source_stamp_ns":wall,
                    "mission_id":current_mission,"timestamp_semantics":"OBSERVER_RECEIPT"})
            self.previous_mission=current_mission
        goal=st.get("selected_goal")
        goal_key=(st.get("child_goal_id"),str(goal),st.get("goal_provider"),st.get("mission_id"))
        if isinstance(goal,list) and len(goal)>=2 and all(finite(g) for g in goal[:2]) and goal_key!=self.last_goal:
            try:
                local_goal=self.goal_to_local(goal) if self.goal_to_local else None
            except (ValueError,TypeError,RuntimeError):
                local_goal=None
            self.event({"type":"goal","source_stamp_ns":wall,"timestamp_semantics":"OBSERVER_RECEIPT_NOT_SELECTION_TIME",
                "frame_id":GLOBAL_FRAME,"goal":goal[:3],"local_goal":local_goal,
                "goal_id":st.get("child_goal_id"),
                "provider":st.get("goal_provider"),"mission_id":st.get("mission_id"),
                "cruise_phase":st.get("cruise_phase"),"frontier_status":st.get("frontier_status")})
        self.last_goal=goal_key
        obs=st.get("nav2_observation") or {}
        state_key=(st.get("state"),st.get("cruise_phase"),st.get("frontier_status"),
            st.get("action_active"),st.get("cancelling"),st.get("pending"),st.get("child_goal_id"),
            st.get("frontier_goals"),st.get("coverage_reached"))
        if state_key!=self.prev_state:
            self.event({"type":"nav2_decision","source_stamp_ns":wall,
                "timestamp_semantics":"OBSERVER_RECEIPT_NOT_DECISION_TIME", "reason":"STATE_CHANGE",
                "state":st.get("state"),"cruise_phase":st.get("cruise_phase"),
                "frontier_status":st.get("frontier_status"),"action_active":st.get("action_active"),
                "cancelling":st.get("cancelling"),"pending":st.get("pending"),
                "child_goal_id":st.get("child_goal_id"),"frontier_goals":st.get("frontier_goals"),
                "coverage_reached":st.get("coverage_reached")})
            self.prev_state=state_key
        for err in obs.get("navigation_errors",[]) if isinstance(obs,dict) else []:
            if not isinstance(err,dict):continue
            key=(err.get("measurement_monotonic_ns"),err.get("kind"),err.get("code"))
            if key not in self.nav2_errors_seen:
                self.nav2_errors_seen.add(key)
                self.event({"type":"nav2_error","source_stamp_ns":wall,"reason":err.get("kind"),
                            "code":err.get("code"),"message":err.get("message")})
        if now-self.last_nav2_observation_poll_ns >= 2_000_000_000:
            self.last_nav2_observation_poll_ns = now
            rate=obs.get("controller",{}) if isinstance(obs,dict) else {}
            age=obs.get("sensor_age_ns",{}) if isinstance(obs,dict) else {}
            self.event({"type":"nav2_observation","source_stamp_ns":wall,
                "cruise_phase":st.get("cruise_phase"),"frontier_status":st.get("frontier_status"),
                "state":st.get("state"),"controller_hz":rate.get("source_hz"),
                "sensor_tf_age_ms":age.get("tf")/1e6 if type(age.get("tf")) is int else None,
                "sensor_scan_age_ms":age.get("scan")/1e6 if type(age.get("scan")) is int else None,
                "wall_clearance":obs.get("wall_clearance"),
                "errors":obs.get("navigation_errors",[])[:5],
                "coverage":obs.get("coverage")})
        self.poll_v3(now)

    def poll_v3(self, now: int) -> None:
        if now-self.last_v3_poll_ns<500_000_000:return
        self.last_v3_poll_ns=now
        try:
            from v3.live_io_paths import live_io_directory
            p=live_io_directory(self.root,create=False)/"v3_status.json"
            if p.stat().st_size>131_072:return
            s=json.loads(p.read_text(encoding="utf-8"))
        except (OSError,ValueError,TypeError):return
        measured=s.get("monotonic_ns")
        if type(measured) is not int or not 0<=now-measured<=1_500_000_000:return
        estimate=s.get("estimate") or {}
        quality=estimate.get("localization_quality") or {}
        nav=s.get("navigation") or {}
        mission=s.get("mission") or {}
        value={"tick_id":s.get("tick_id"),"safety_decision":s.get("safety_decision"),
               "safety_reason":s.get("safety_reason"),"nav_status":nav.get("status"),
               "nav_reason":nav.get("reason"),"goal_selection_reason":nav.get("goal_selection_reason"),
               "mission_mode":mission.get("mode"),"mission_stop_reason":mission.get("stop_reason"),
               "quality":{k:quality.get(k) for k in ("heading","local_translation","global_position")}}
        # Tick IDs change at 50 Hz and are not themselves navigation decisions.
        comparison={k:v for k,v in value.items() if k!="tick_id"}
        if comparison!=self.prev_v3:
            self.event({"type":"v3_decision","source_stamp_ns":time.time_ns(),
                "timestamp_semantics":"OBSERVER_RECEIPT_NOT_V3_TICK_TIME", "reason":nav.get("reason") or s.get("safety_reason"),
                **value})
            self.prev_v3=comparison


def watch_ros(root: Path, *, once_seconds: float = 0.0) -> dict:
    """Run inside existing ROS-isolated Python; no control socket used."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid, Odometry
    from tf2_ros import Buffer, TransformListener, TransformException
    from rclpy.signals import SignalHandlerOptions
    running=True
    def stopped(_sig,_frame):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stopped)
    signal.signal(signal.SIGINT,stopped)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=Node("r2b4_passive_map_export_observer")
    archive=Archive(root)
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    def convert_goal_to_local(goal):
        transform=buffer.lookup_transform(LOCAL_FRAME,GLOBAL_FRAME,Time())
        if abs(time.time_ns()-stamp(transform))>250_000_000:
            return None  # Never stamp a goal with a stale cross-frame TF.
        p=transform.transform.translation
        return transform_xy(goal,p.x,p.y,yaw_from_quat(transform.transform.rotation))
    archive.goal_to_local=convert_goal_to_local
    reliable=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    latched=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[]
    for kind,topic in TOPICS.items():
        subscriptions.append(node.create_subscription(OccupancyGrid,topic,
            lambda msg,k=kind:archive.map(msg,k),latched if kind=="global" else reliable))
    def odom_callback(msg):
        if msg.header.frame_id!=LOCAL_FRAME or msg.child_frame_id!="base_link":return
        try:
            q=msg.pose.pose.orientation
            p=msg.pose.pose.position
            archive.pose(LOCAL_FRAME,stamp(msg),p.x,p.y,yaw_from_quat(q))
        except (ValueError,TypeError):pass
    subscriptions.append(node.create_subscription(Odometry,"/r2b4/odom",odom_callback,qos_profile_sensor_data))
    start=time.monotonic()
    last_global=0
    try:
        while running and rclpy.ok() and (once_seconds<=0 or time.monotonic()-start<once_seconds):
            rclpy.spin_once(node,timeout_sec=.05)
            archive.poll_status()
            now=time.monotonic_ns()
            archive.poll_v3(now)
            if now-last_global>=200_000_000:
                last_global=now
                try:
                    tf=buffer.lookup_transform(GLOBAL_FRAME,"base_link",Time())
                    source=stamp(tf)
                    if abs(time.time_ns()-source)<1_000_000_000:
                        p=tf.transform.translation
                        archive.pose(GLOBAL_FRAME,source,p.x,p.y,yaw_from_quat(tf.transform.rotation))
                except (TransformException,ValueError,TypeError):
                    pass
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()
    return {"global_ros_revisions":archive.ros_map_valid_count,
            "local_ros_revisions":archive.ros_local_valid_count,
            "session":archive.session_id,"cache":str(archive.cache)}


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Egy alkalommal exportal SLAM global es tenyleges NAV2 local JPG terkepet. Nincs hatterfolyamat.")
    parser.add_argument("--root", type=Path, default=None, help="R2B4 repo gyokere")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="hany masodpercig figyelje a FUTÓ ROS terkepet es utvonalat (alap: 5)")
    parser.add_argument("--offline", action="store_true",
                        help="ROS lekerdezes nelkul, a korabban mentett pillanatkepekbol")
    parser.add_argument("--_ros_observe", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.seconds < 0.5 or args.seconds > 3600.0:
        parser.error("--seconds legyen 0.5 es 3600 masodperc kozott")
    root = root_path(args.root)
    if args._ros_observe:
        print(json.dumps(watch_ros(root, once_seconds=args.seconds), ensure_ascii=False))
        return 0
    if not args.offline:
        # Pontosan a repoban letezo Nav2 ROS kornyezetet hasznaljuk.
        # EGY egyszeri, rovid eletu subprocess. Nem service, nem daemon.
        try:
            from tools.nav2_planner import _paths, _navigation_environment
            paths = _paths(root)
            interpreter = paths["prefix"] / "bin" / "python"
            if not interpreter.is_file():
                raise FileNotFoundError(f"Nav2 ROS Python hianyzik: {interpreter}")
            env = _navigation_environment(paths, root)
            command = [str(interpreter), "-m", "tools.r2b4_terkep",
                       "--_ros_observe", "--root", str(root), "--seconds", str(args.seconds)]
            result = subprocess.run(command, cwd=root, env=env, capture_output=True,
                                    text=True, timeout=args.seconds + 18)
            if result.returncode != 0:
                print("ROS olvasas nem sikerult, mentett terkepre esunk vissza: " +
                      (result.stderr.strip()[-700:] or str(result.returncode)), file=sys.stderr)
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            print(f"ROS nem elerheto, mentett terkepet probalok: {exc}", file=sys.stderr)
    response = export(root, kinds=("local", "global"))
    print(json.dumps(response, indent=2, ensure_ascii=False))
    if response["missing"]:
        print("Hianyzo terkep nem helyettesitheto kitalalt adattal.", file=sys.stderr)
    return 0 if response["result"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
