"""Render a recorded local 2D path and bounded LiDAR returns from verified EVI.

This is a passive offline view, not SLAM, planner occupancy or Atlas alignment.
The original MCAP remains evidence authority. Older behavior captures can show
their estimated path, but missing obstacle geometry is never reconstructed.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
import csv
from dataclasses import dataclass
import hashlib
from html import escape
import json
import math
from pathlib import Path
import sys

from tools.mcap_evidence.reader import EvidenceBundle
from v3.navigation_map_capture import NAVIGATION_MAP_TOPIC, validate_navigation_map
from v3.local_costmap_capture import LOCAL_COSTMAP_TOPIC, validate_local_costmap

MAX_POSE_GAP_NS = 250_000_000


def _dict(value):
    return value if isinstance(value, dict) else {}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _uint(value):
    return type(value) is int and value >= 0


@dataclass(frozen=True)
class Pose:
    time_ns: int
    tick_id: int
    message_id: str
    frame_id: str
    generation: int
    x_m: float
    y_m: float
    yaw_rad: float
    qualified: bool
    segment: int
    planner_hold_sample: bool = False
    plan_reason: str | None = None

    @property
    def identity(self):
        return self.frame_id, self.generation


def _poses(bundle, quality):
    poses, previous, segment = [], None, -1
    for row in bundle.iter_view("ticks"):
        payload = _dict(row.payload)
        context = _dict(_dict(payload.get("inputs")).get("context"))
        layers = _dict(_dict(payload.get("expected")).get("layers"))
        layer = _dict(layers.get("L3"))
        pose, local = _dict(layer.get("local_pose")), _dict(layer.get("localization_quality"))
        timestamp, tick = payload.get("monotonic_ns"), payload.get("tick_id")
        generation, frame = local.get("generation"), pose.get("frame_id")
        layer_context = layer.get("context")
        valid = (_uint(timestamp) and _uint(tick) and tick == row.tick_id
                 and _uint(context.get("monotonic_ns")) and _uint(context.get("tick_id"))
                 and context.get("monotonic_ns") == timestamp and context.get("tick_id") == tick
                 and layer_context in (context, {"$ref": "input_context"})
                 and _uint(generation) and isinstance(frame, str) and bool(frame)
                 and all(_finite(pose.get(name)) for name in ("x_m", "y_m", "yaw_rad")))
        if not valid or (poses and timestamp <= poses[-1].time_ns):
            quality["invalid_pose_rows"] += 1
            previous = None
            segment += 1
            continue
        qualified = (local.get("local_translation") in ("GOOD", "DEGRADED")
                     and local.get("heading") in ("GOOD", "DEGRADED")
                     and local.get("local_pose_continuous") is True
                     and local.get("pose_discontinuity") is False)
        if (previous is None or previous.identity != (frame, generation)
                or not previous.qualified or not qualified
                or timestamp - previous.time_ns > MAX_POSE_GAP_NS):
            segment += 1
        plan, mission = _dict(layers.get("L6")), _dict(layers.get("L5"))
        reason = plan.get("reason") if isinstance(plan.get("reason"), str) else None
        hold = (mission.get("lifecycle") == "ACTIVE" and reason is not None
                and plan.get("status") in ("IDLE", "PENDING", "INVALIDATED", "NO_PATH")
                and (reason.endswith("_HOLD") or reason == "PLANNER_PENDING"))
        sample = Pose(timestamp, tick, row.message_id, frame, generation,
                      pose["x_m"], pose["y_m"], pose["yaw_rad"], qualified, segment, hold, reason)
        poses.append(sample)
        quality["qualified_pose_rows" if qualified else "unqualified_pose_rows"] += 1
        previous = sample
    return poses


def _interpolate(poses, times, stamp):
    index = bisect_left(times, stamp)
    if index < len(poses) and times[index] == stamp:
        before = after = poses[index]
    elif index == 0 or index == len(poses):
        return None, "outside_recorded_pose_interval"
    else:
        before, after = poses[index - 1], poses[index]
    if not before.qualified or not after.qualified:
        return None, "unqualified_pose_bracket"
    if before.identity != after.identity or before.segment != after.segment:
        return None, "pose_continuity_boundary"
    gap = after.time_ns - before.time_ns
    if gap > MAX_POSE_GAP_NS:
        return None, "pose_bracket_gap"
    weight = (stamp - before.time_ns) / gap if gap else 0.
    yaw_delta = math.atan2(math.sin(after.yaw_rad - before.yaw_rad),
                          math.cos(after.yaw_rad - before.yaw_rad))
    return (before, after, weight,
            before.x_m + weight * (after.x_m - before.x_m),
            before.y_m + weight * (after.y_m - before.y_m),
            before.yaw_rad + weight * yaw_delta), None


def _events(bundle):
    if bundle.has_view("events"):
        for row in bundle.iter_view("events"):
            yield row, _dict(row.payload)


def _valid_scan(payload):
    try:
        validate_navigation_map(payload)
        return True
    except (ValueError, TypeError):
        return False


def collect(bundle):
    """Project only recorded returns with qualified measurement-time brackets."""
    quality = Counter()
    poses = _poses(bundle, quality)
    times = [pose.time_ns for pose in poses]
    panels, lineage, seen, scope = {}, [], set(), {}
    for pose in poses:
        panels.setdefault(pose.identity, {"poses": [], "returns": [], "latest_returns": [],
                                          "latest_measurement_ns": -1, "map_samples": 0})["poses"].append(pose)
    for row, envelope in _events(bundle):
        if envelope.get("event_type") == "capture_finalized":
            integrity = _dict(envelope.get("integrity"))
            scope = {"capture_hz": envelope.get("capture_hz"), "capture_status": envelope.get("status"),
                     **{name: integrity.get(name) for name in
                        ("sample_complete", "raw_evidence_requested", "replay_complete", "navigation_map_requested",
                         "navigation_map_hz", "navigation_map_complete", "navigation_map_integrity_reasons",
                         "navigation_map_transport", "local_costmap_requested", "local_costmap_snapshot_count",
                         "local_costmap_available_count")}}
        if envelope.get("source_topic") != NAVIGATION_MAP_TOPIC:
            continue
        payload = _dict(envelope.get("payload"))
        quality["map_events"] += 1
        record = {"message_id": row.message_id, "source_pointer": row.source_pointer,
                  **{name: payload.get(name) for name in
                     ("source_sequence", "measurement_monotonic_ns", "scan_start_monotonic_ns",
                      "scan_end_monotonic_ns", "source_point_count", "sensor_health",
                      "lidar_x_offset_m", "lidar_y_offset_m")}}
        reason, projection = None, None
        if not _valid_scan(payload):
            reason = "invalid_map_observation"
        elif payload["sensor_health"] != "OK":
            reason = "sensor_not_ok"
        else:
            identity = (payload["source_sequence"], payload["measurement_monotonic_ns"])
            if identity in seen:
                reason = "duplicate_map_observation"
            else:
                seen.add(identity)
                projection, reason = _interpolate(poses, times, payload["measurement_monotonic_ns"])
        if projection is None:
            record["status"] = reason
            quality[reason] += 1
            lineage.append(record)
            continue
        before, after, weight, x, y, yaw = projection
        cosine, sine = math.cos(yaw), math.sin(yaw)
        # Producer coordinates already include the LiDAR -> axle translation.
        points = [(x + cosine * px - sine * py, y + sine * px + cosine * py)
                  for px, py in payload["points_xy_m"]]
        panel = panels[before.identity]
        panel["returns"].extend(points)
        if payload["measurement_monotonic_ns"] > panel["latest_measurement_ns"]:
            panel["latest_returns"] = points
            panel["latest_measurement_ns"] = payload["measurement_monotonic_ns"]
        panel["map_samples"] += 1
        quality["projected_map_samples"] += 1
        quality["projected_return_points"] += len(points)
        record.update(status="PROJECTED", frame_id=before.frame_id, generation=before.generation,
                      bracket_tick_ids=[before.tick_id, after.tick_id],
                      bracket_message_ids=[before.message_id, after.message_id],
                      bracket_times_ns=[before.time_ns, after.time_ns], weight=weight,
                      point_count=len(points))
        lineage.append(record)
    return panels, lineage, dict(quality), scope


def _timeline(bundle, panels):
    """Join exact recorded source ticks; never manufacture historical grids."""
    poses = {pose.tick_id: pose for panel in panels.values() for pose in panel["poses"]}
    states, by_tick = [], {}
    for row in bundle.iter_view("ticks"):
        payload = _dict(row.payload)
        pose = poses.get(payload.get("tick_id"))
        if pose is None or payload.get("monotonic_ns") != pose.time_ns:
            continue
        layers = _dict(_dict(payload.get("expected")).get("layers"))
        plan, mission = _dict(layers.get("L6")), _dict(layers.get("L5"))
        requested, allowed = _dict(layers.get("L8")), _dict(layers.get("L9"))
        final = _dict(layers.get("L12"))
        quality = _dict(_dict(layers.get("L3")).get("localization_quality"))
        by_tick[pose.tick_id] = len(states)
        states.append({"tick_id": pose.tick_id, "monotonic_ns": pose.time_ns,
                       "source_message_id": row.message_id, "frame_id": pose.frame_id,
                       "generation": pose.generation, "pose": [pose.x_m, pose.y_m, pose.yaw_rad],
                       "qualified": pose.qualified, "segment": pose.segment,
                       "lifecycle": mission.get("lifecycle"), "plan_status": plan.get("status"),
                       "plan_reason": plan.get("reason"), "stop_reason": requested.get("stop_reason"),
                       "requested": [requested.get("requested_v_mps"), requested.get("requested_omega_rad_s")],
                       "allowed": [allowed.get("allowed_v_mps"), allowed.get("allowed_omega_rad_s")],
                       "outputs": [final.get("left_output"), final.get("right_output")],
                       "localization_quality": quality})
    snapshots, cell_sets, unique, counts = [], [], {}, Counter()
    for row, envelope in _events(bundle):
        if envelope.get("source_topic") != LOCAL_COSTMAP_TOPIC:
            continue
        counts["recorded_snapshots"] += 1
        payload = _dict(envelope.get("payload"))
        try:
            validate_local_costmap(payload)
        except (ValueError, TypeError):
            counts["invalid_snapshots"] += 1
            continue
        cells = payload["occupied_cells"]
        digest = hashlib.sha256(json.dumps(cells, separators=(",", ":")).encode()).digest()
        if digest not in unique:
            unique[digest] = len(cell_sets)
            cell_sets.append(cells)
        state_index = by_tick.get(payload["tick_id"])
        state = states[state_index] if state_index is not None else None
        aligned = state is not None and state["monotonic_ns"] == payload["monotonic_ns"]
        if aligned and payload["available"]:
            aligned = (state["frame_id"], state["generation"]) == (payload["frame_id"], payload["generation"])
        counts["aligned_snapshots" if aligned else "unaligned_snapshots"] += 1
        counts["available_snapshots"] += int(payload["available"])
        snapshots.append({**{key: value for key, value in payload.items() if key != "occupied_cells"},
                          "source_message_id": row.message_id, "source_pointer": row.source_pointer,
                          "cells_ref": unique[digest], "state_index": state_index if aligned else None})
    snapshots.sort(key=lambda row: (row["monotonic_ns"], row["tick_id"]))
    counts["unique_cell_sets"] = len(cell_sets)
    # Old captures retain a useful path timeline, with an explicit grid gap.
    frames = snapshots or [{"tick_id": s["tick_id"], "monotonic_ns": s["monotonic_ns"],
                           "available": False, "availability_reason": "NOT_RECORDED",
                           "frame_id": s["frame_id"], "generation": s["generation"],
                           "state_index": index, "cell_count": 0} for index, s in enumerate(states)]
    return {"snapshots": frames, "states": states, "cell_sets": cell_sets,
            "actual_grid_recorded": bool(snapshots), "counts": dict(counts)}


def _html(timeline, source):
    data = json.dumps({**timeline, "source": source}, separators=(",", ":"), allow_nan=False)
    # Capture labels are data even when an HTML-looking string was recorded.
    data = data.replace("<", "\\u003c").replace("&", "\\u0026")
    return '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Recorded L4 local costmap timeline</title>
<style>body{font:15px system-ui,sans-serif;margin:24px;color:#203149;background:#f5f7fa}main{max-width:1180px;margin:auto}h1{font-size:24px}canvas{width:100%;background:white;border:1px solid #ccd4df}#controls{display:flex;gap:10px;align-items:center;margin:14px 0}input{flex:1}button{padding:7px 14px}pre{white-space:pre-wrap;background:white;padding:14px;line-height:1.5}.note{color:#4c5c70}#gap{color:#a33818;font-weight:600}</style>
<main><h1>Actual recorded L4 costmap and estimated path</h1>
<p class="note">Black squares: occupied cells in this completed L4 snapshot. Blue: qualified estimated path up to this time. Red: same-tick estimated robot pose. White means unknown, not proven free.</p>
<div id="gap"></div><div id="controls"><button id="back">◀</button><button id="play">Play</button><button id="next">▶</button><input id="slider" type="range" min="0" value="0" step="1" aria-label="Snapshot time"></div>
<canvas id="map" width="1100" height="720"></canvas><pre id="status"></pre>
<p class="note">No scan registration, map alignment or geometric correction is applied. Cell origins (temporal versus structural memory) are not present in this capture format. Native bias-warning and pose-discontinuity flags are displayed below.</p>
<p><a href="atlas2d.svg">Separate historical measured-return union (SVG)</a> · <a href="report.json">Evidence/projection report</a> · <a href="path.csv">Path CSV</a></p>
<pre id="source"></pre></main><script type="application/json" id="data">''' + data + '''</script>
<script>
"use strict";
const D=JSON.parse(document.getElementById("data").textContent), rows=D.snapshots;
const canvas=document.getElementById("map"),ctx=canvas.getContext("2d"),slider=document.getElementById("slider");
const status=document.getElementById("status"),gap=document.getElementById("gap");
let index=Math.max(0,rows.length-1),playing=false,timer=null;
slider.max=Math.max(0,rows.length-1);slider.value=index;
document.getElementById("source").textContent="Source: "+(D.source.path||"unknown")+"\nSHA256: "+(D.source.sha256||"unavailable")+"\nPassive offline view; no motion authority or exact replay claim.";
const finite=x=>typeof x==="number"&&Number.isFinite(x),show=x=>x===null||x===undefined?"unavailable":String(x);
function identity(r){const s=r.state_index===null||r.state_index===undefined?null:D.states[r.state_index];return [r.frame_id||(s&&s.frame_id),r.generation===null||r.generation===undefined?s&&s.generation:r.generation];}
const same=(a,b)=>a[0]===b[0]&&a[1]===b[1];
const bounds=new Map();
function extent(id,x,y){if(!finite(x)||!finite(y))return;const key=JSON.stringify(id),v=bounds.get(key)||[x,x,y,y];v[0]=Math.min(v[0],x);v[1]=Math.max(v[1],x);v[2]=Math.min(v[2],y);v[3]=Math.max(v[3],y);bounds.set(key,v);}
for(const s of D.states)if(s.qualified)extent([s.frame_id,s.generation],s.pose[0],s.pose[1]);
for(const r of rows)if(r.available){const id=identity(r);for(const c of D.cell_sets[r.cells_ref]){extent(id,c[0]*r.resolution_m,c[1]*r.resolution_m);extent(id,(c[0]+1)*r.resolution_m,(c[1]+1)*r.resolution_m);}}
function draw(){
 ctx.clearRect(0,0,canvas.width,canvas.height);if(!rows.length){gap.textContent="No recorded grid snapshots or usable local poses.";status.textContent="No timeline.";return;}
 const r=rows[index],s=r.state_index===null||r.state_index===undefined?null:D.states[r.state_index],id=identity(r);
 const b=bounds.get(JSON.stringify(id))||[-1,1,-1,1],margin=.3,minx=b[0]-margin,maxx=b[1]+margin,miny=b[2]-margin,maxy=b[3]+margin;
 const scale=Math.min(1000/Math.max(1,maxx-minx),610/Math.max(1,maxy-miny));
 const xy=(x,y)=>[65+(x-minx)*scale,45+(maxy-y)*scale];
 const step=Math.max(1,10**Math.ceil(Math.log10(Math.max(maxx-minx,maxy-miny)/15)));
 ctx.strokeStyle="#e8edf3";ctx.fillStyle="#64748b";ctx.lineWidth=1;ctx.font="13px system-ui";
 for(let x=Math.ceil(minx/step)*step;x<=maxx;x+=step){const a=xy(x,miny),b=xy(x,maxy);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.fillText(x.toFixed(0)+"m",a[0],a[1]+20);}
 for(let y=Math.ceil(miny/step)*step;y<=maxy;y+=step){const a=xy(minx,y),b=xy(maxx,y);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();ctx.fillText(y.toFixed(0)+"m",a[0]-45,a[1]+4);}
 if(r.available){ctx.fillStyle="#263446";const size=r.resolution_m*scale;for(const c of D.cell_sets[r.cells_ref]){const p=xy(c[0]*r.resolution_m,(c[1]+1)*r.resolution_m);ctx.fillRect(p[0],p[1],size,size);}}
 ctx.strokeStyle="#2563eb";ctx.lineWidth=2.5;ctx.beginPath();let previous=null;
 for(const p of D.states){if(p.monotonic_ns>r.monotonic_ns)break;if(!p.qualified||!same([p.frame_id,p.generation],id)){previous=null;continue;}const q=xy(p.pose[0],p.pose[1]);if(previous!==null&&previous.segment===p.segment)ctx.lineTo(...q);else ctx.moveTo(...q);previous=p;}ctx.stroke();
 if(s&&s.qualified&&same([s.frame_id,s.generation],id)){const p=xy(s.pose[0],s.pose[1]),tip=xy(s.pose[0]+.22*Math.cos(s.pose[2]),s.pose[1]+.22*Math.sin(s.pose[2]));ctx.fillStyle="#dc2626";ctx.beginPath();ctx.arc(...p,5,0,2*Math.PI);ctx.fill();ctx.strokeStyle="#dc2626";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...tip);ctx.stroke();}
 gap.textContent=!D.actual_grid_recorded?"Actual L4 grid was not recorded in this capture. This timeline shows the estimated path only.":!r.available?"Actual grid unavailable at this tick: "+show(r.availability_reason):!s?"Grid recorded; no exactly aligned sampled L3/navigation state. Robot marker is omitted.":"";
 const first=rows[0].monotonic_ns,q=s?s.localization_quality:r.localization_quality||{};
 const lines=["Snapshot "+(index+1)+" / "+rows.length+" | capture +"+((r.monotonic_ns-first)/1e9).toFixed(3)+" s | source tick "+r.tick_id,
 "Frame "+show(id[0])+" | generation "+show(id[1])+" | grid revision "+show(r.revision)+" | source scan "+show(r.source_sequence),
 "Occupied cells "+show(r.cell_count)+" | resolution "+show(r.resolution_m)+" m | geometry freshness "+(finite(r.freshness_ns)?(r.freshness_ns/1e6).toFixed(1)+" ms":"unavailable")];
 if(s)lines.push("Same-tick mission "+show(s.lifecycle)+" | L6 "+show(s.plan_status)+" / "+show(s.plan_reason)+" | L8 stop "+show(s.stop_reason),"Requested v / ω "+s.requested.map(show).join(" / ")+" | allowed "+s.allowed.map(show).join(" / ")+" | final left/right "+s.outputs.map(show).join(" / "));
 lines.push("Native accumulated translation bias warning: "+show(q.accumulated_translation_bias_warning)+" | encoder–LiDAR discrepancy "+show(q.encoder_lidar_consistency_m)+" m","Local / heading "+show(q.local_translation)+" / "+show(q.heading)+" | pose discontinuity "+show(q.pose_discontinuity)+" | continuous "+show(q.local_pose_continuous),"Grid source message: "+show(r.source_message_id)+" | state source message: "+show(s&&s.source_message_id));
 status.textContent=lines.join("\n");
}
function stop(){playing=false;clearTimeout(timer);document.getElementById("play").textContent="Play";}
function select(i){index=Math.max(0,Math.min(rows.length-1,i));slider.value=index;draw();}
function advance(){if(!playing)return;if(index>=rows.length-1){stop();return;}const wait=Math.max(20,Math.min(1000,(rows[index+1].monotonic_ns-rows[index].monotonic_ns)/1e6));timer=setTimeout(()=>{select(index+1);advance();},wait);}
slider.addEventListener("input",()=>{stop();select(Number(slider.value));});
document.getElementById("back").onclick=()=>{stop();select(index-1);};document.getElementById("next").onclick=()=>{stop();select(index+1);};
document.getElementById("play").onclick=()=>{if(playing){stop();return;}if(index>=rows.length-1)select(0);playing=true;document.getElementById("play").textContent="Pause";advance();};
draw();
</script></html>'''


def _sampled_holds(panels):
    """Longest sampled plan-hold runs; unsampled motion is not inferred."""
    runs = []
    for panel in panels.values():
        run = []
        for pose in [*panel["poses"], None]:
            if (run and (pose is None or not pose.qualified or not pose.planner_hold_sample
                         or pose.segment != run[-1].segment)):
                if len(run) >= 2:
                    first, last = run[0], run[-1]
                    runs.append({"frame_id": first.frame_id, "generation": first.generation,
                                 "tick_id": first.tick_id, "time_ns": first.time_ns,
                                 "end_tick_id": last.tick_id, "sample_count": len(run),
                                 "sampled_span_s": (last.time_ns - first.time_ns) / 1e9,
                                 "reason": first.plan_reason, "end_reason": last.plan_reason,
                                 "x_m": first.x_m, "y_m": first.y_m,
                                 "physical_stop_inferred": False})
                run = []
            if pose is not None and pose.qualified and pose.planner_hold_sample:
                run.append(pose)
    first_time = min((p.time_ns for panel in panels.values() for p in panel["poses"]), default=0)
    selected = sorted(sorted(runs, key=lambda r: r["sampled_span_s"], reverse=True)[:3], key=lambda r: r["time_ns"])
    for index, run in enumerate(selected, 1):
        run.update(label="H" + str(index), capture_elapsed_s=(run["time_ns"] - first_time) / 1e9)
    return selected


def _svg(panels, source_name, holds=()):
    width, panel_height = 1100, 790
    result = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{120 + panel_height * max(1, len(panels))}" role="img">',
              '<rect width="100%" height="100%" fill="white"/>',
              '<style>text{font:15px sans-serif;fill:#24334a}.title{font-size:22px;font-weight:bold}</style>',
              '<text x="35" y="34" class="title">Recorded local map and estimated robot path</text>',
              f'<text x="35" y="61">{escape(source_name)}</text>',
              '<text x="35" y="88">Blue: sampled L3 path · gray: measured return history · dark: latest projected scan · green/red: path start/end</text>',
              '<text x="35" y="110">Unknown space is white. Returns are not planner occupancy; no SLAM or historical Atlas alignment is inferred.</text>']
    for panel_index, ((frame, generation), panel) in enumerate(sorted(panels.items())):
        top = 120 + panel_index * panel_height
        poses = panel["poses"]
        qualified = [pose for pose in poses if pose.qualified]
        points = [*panel["returns"], *((pose.x_m, pose.y_m) for pose in qualified)]
        result.append(f'<text x="35" y="{top + 28}">{escape(frame)} · generation {generation} · {len(qualified)} qualified path samples · {panel["map_samples"]} projected scans</text>')
        if not points:
            result.append(f'<text x="35" y="{top + 58}">No qualified geometry in this coordinate frame.</text>')
            continue
        min_x, max_x = min(p[0] for p in points) - .3, max(p[0] for p in points) + .3
        min_y, max_y = min(p[1] for p in points) - .3, max(p[1] for p in points) + .3
        scale = min(980 / max(max_x - min_x, 1), 570 / max(max_y - min_y, 1))
        def xy(x, y):
            return 75 + (x - min_x) * scale, top + 65 + (max_y - y) * scale
        grid_step = max(1, 10 ** math.ceil(math.log10(max(max_x - min_x, max_y - min_y) / 20)))
        for x in range(math.ceil(min_x / grid_step) * grid_step, math.floor(max_x / grid_step) * grid_step + 1, grid_step):
            a, b = xy(x, min_y), xy(x, max_y)
            result.append(f'<path d="M{a[0]:.2f},{a[1]:.2f}L{b[0]:.2f},{b[1]:.2f}" stroke="#edf0f4"/>')
            result.append(f'<text x="{a[0]:.2f}" y="{a[1] + 20:.2f}">{x} m</text>')
        for y in range(math.ceil(min_y / grid_step) * grid_step, math.floor(max_y / grid_step) * grid_step + 1, grid_step):
            a, b = xy(min_x, y), xy(max_x, y)
            result.append(f'<path d="M{a[0]:.2f},{a[1]:.2f}L{b[0]:.2f},{b[1]:.2f}" stroke="#edf0f4"/>')
            result.append(f'<text x="{a[0] - 45:.2f}" y="{a[1] + 5:.2f}">{y} m</text>')
        for name, color, radius in (("returns", "#b7bdc5", 2), ("latest_returns", "#334155", 2.5)):
            data = " ".join(f'M{xy(x, y)[0]:.2f},{xy(x, y)[1]:.2f}h.01' for x, y in panel[name])
            if data:
                result.append(f'<path d="{data}" fill="none" stroke="{color}" stroke-width="{radius * 2}" stroke-linecap="round"/>')
        segments = {}
        for pose in qualified:
            segments.setdefault(pose.segment, []).append(pose)
        for segment in segments.values():
            data = " ".join(f'{xy(p.x_m, p.y_m)[0]:.2f},{xy(p.x_m, p.y_m)[1]:.2f}' for p in segment)
            if len(segment) > 1:
                result.append(f'<polyline points="{data}" fill="none" stroke="#2563eb" stroke-width="2.5"/>')
            for pose in (segment[0], segment[-1]):
                a, b = xy(pose.x_m, pose.y_m)
                result.append(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="2" fill="#2563eb"/>')
        if qualified:
            for pose, color in ((qualified[0], "#16803c"), (qualified[-1], "#dc2626")):
                a, b = xy(pose.x_m, pose.y_m)
                tip = xy(pose.x_m + .18 * math.cos(pose.yaw_rad), pose.y_m + .18 * math.sin(pose.yaw_rad))
                result.append(f'<circle cx="{a:.2f}" cy="{b:.2f}" r="5" fill="{color}"/><path d="M{a:.2f},{b:.2f}L{tip[0]:.2f},{tip[1]:.2f}" stroke="{color}" stroke-width="3"/>')
        panel_holds = [h for h in holds if (h["frame_id"], h["generation"]) == (frame, generation)]
        for index, hold in enumerate(panel_holds):
            a, b = xy(hold["x_m"], hold["y_m"])
            description = f'{hold["label"]}: tick {hold["tick_id"]}, capture +{hold["capture_elapsed_s"]:.2f}s, {hold["reason"]}; {hold["sample_count"]} hold samples over {hold["sampled_span_s"]:.2f}s'
            result.append(f'<g><title>{escape(description)}</title><circle cx="{a:.2f}" cy="{b:.2f}" r="8" fill="none" stroke="#9333ea" stroke-width="2"/><text x="{a + 10:.2f}" y="{b - 8:.2f}">{hold["label"]}</text></g>')
            result.append(f'<text x="75" y="{top + 694 + index * 18}">{escape(description)}</text>')
        if panel_holds:
            result.append(f'<text x="75" y="{top + 673}">Purple markers: sampled planner hold starts; intervening unsampled motion remains possible.</text>')
        if not panel["map_samples"]:
            result.append(f'<text x="75" y="{top + 765}">Obstacle geometry was not recorded/projectable. White space does not establish free space.</text>')
    if not panels:
        result.append('<text x="35" y="165">No usable sampled L3 local poses were recorded.</text>')
    result.append('</svg>')
    return "\n".join(result) + "\n"


def render(bundle, output_dir):
    """Write standalone artifacts without modifying the sealed evidence bundle."""
    if bundle.verification.get("status") != "PASS":
        raise ValueError("verified EVI evidence is required")
    output = Path(output_dir).expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("output directory already exists: " + str(output))
    output = output.resolve()
    if output.is_relative_to(bundle.root.resolve()):
        raise ValueError("output must be outside the sealed evidence bundle")
    panels, lineage, quality, scope = collect(bundle)
    timeline = _timeline(bundle, panels)
    holds = _sampled_holds(panels)
    source = _dict(bundle.manifest.get("source"))
    warnings = []
    if not quality.get("projected_map_samples"):
        warnings.append("OBSTACLE_GEOMETRY_NOT_RECORDED_OR_NOT_PROJECTABLE")
    if quality.get("map_events", 0) > quality.get("projected_map_samples", 0):
        warnings.append("SOME_MAP_OBSERVATIONS_COULD_NOT_BE_PROJECTED")
    if scope.get("navigation_map_requested") and scope.get("navigation_map_complete") is not True:
        warnings.append("NAVIGATION_MAP_TRANSPORT_INCOMPLETE")
    if scope.get("local_costmap_requested") and not timeline["actual_grid_recorded"]:
        warnings.append("REQUESTED_LOCAL_COSTMAP_SNAPSHOTS_MISSING")
    if timeline["counts"].get("invalid_snapshots") or timeline["counts"].get("unaligned_snapshots"):
        warnings.append("LOCAL_COSTMAP_SNAPSHOT_OR_STATE_ALIGNMENT_GAPS")
    report = {"schema": "R2B4_OFFLINE_ATLAS2D_V1", "source": source,
              "evidence_bundle": str(bundle.root), "verification": bundle.verification,
              "compiler_status": bundle.manifest.get("compiler_status"),
              "source_integrity": bundle.manifest.get("source_integrity"),
              "capture_scope": scope,
              "motion_authority": False, "exact_replay": "NOT_PERFORMED",
              "path": "SAMPLED_L3_LOCAL_ESTIMATE", "obstacles": "BOUNDED_MEASURED_RETURN_HISTORY",
              "planner_occupancy": ("RECORDED_L4_SNAPSHOTS" if timeline["actual_grid_recorded"] else "NOT_RECONSTRUCTED"),
              "historical_atlas_alignment": "NOT_INFERRED",
              "scan_projection": "RIGID_AT_MEASUREMENT_TIME_NO_DESKEW",
              "lidar_extrinsics": "APPLIED_BY_PRODUCER",
              "unknown_space": "NOT_FREE_SPACE_EVIDENCE", "max_pose_bracket_gap_ns": MAX_POSE_GAP_NS,
              "quality": quality, "warnings": warnings,
              "local_costmap_timeline": {"actual_grid_recorded": timeline["actual_grid_recorded"],
                                         **timeline["counts"]},
              "sampled_planner_holds": holds,
              "frames": [{"frame_id": key[0], "generation": key[1], "pose_samples": len(value["poses"]),
                          "projected_map_samples": value["map_samples"]} for key, value in sorted(panels.items())],
              "map_lineage": lineage,
              "artifacts": ["atlas2d.html", "atlas2d.svg", "path.csv", "report.json"]}
    output.mkdir(parents=True)
    (output / "atlas2d.svg").write_text(_svg(panels, Path(str(source.get("path", bundle.root.name))).name, holds))
    (output / "atlas2d.html").write_text(_html(timeline, source))
    with (output / "path.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(Pose.__dataclass_fields__)
        for panel in panels.values():
            for pose in panel["poses"]:
                writer.writerow(tuple(getattr(pose, name) for name in Pose.__dataclass_fields__))
    (output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return {"status": "PASS", "output": str(output), "quality": quality, "warnings": warnings,
            "svg": str(output / "atlas2d.svg"), "html": str(output / "atlas2d.html"),
            "local_costmap_timeline": timeline["counts"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", help="sealed MCAP Evidence Compiler bundle (r evi CAPTURE.mcap)")
    parser.add_argument("--output-dir", required=True, help="new directory for interactive HTML, SVG, CSV and provenance JSON")
    args = parser.parse_args(argv)
    try:
        result = render(EvidenceBundle.open(args.bundle), args.output_dir)
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}), file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
