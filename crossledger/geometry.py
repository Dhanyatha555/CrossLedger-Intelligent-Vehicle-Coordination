"""Path geometry for a simple two-road construction-site crossing."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]
INTENTION_STRAIGHT = "straight"
INTENTION_LEFT = "left"
INTENTION_RIGHT = "right"
APPROACHES = ("south", "west", "north", "east")
INTENTIONS = (INTENTION_STRAIGHT, INTENTION_LEFT, INTENTION_RIGHT)

HEADING_DEG = {
    "north": 90.0,
    "east": 0.0,
    "south": 270.0,
    "west": 180.0,
}


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point]) -> float:
    return sum(_dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def resample_polyline(points: Sequence[Point], step_m: float = 0.5) -> List[Point]:
    if len(points) < 2:
        return list(points)
    out: List[Point] = [points[0]]
    remaining = step_m
    i = 0
    cur = points[0]
    while i < len(points) - 1:
        nxt = points[i + 1]
        seg = _dist(cur, nxt)
        if seg < 1e-9:
            i += 1
            cur = nxt
            continue
        if seg < remaining:
            remaining -= seg
            cur = nxt
            i += 1
            continue
        t = remaining / seg
        cur = (cur[0] + t * (nxt[0] - cur[0]), cur[1] + t * (nxt[1] - cur[1]))
        out.append(cur)
        remaining = step_m
    if _dist(out[-1], points[-1]) > 1e-6:
        out.append(points[-1])
    return out


def point_on_polyline(points: Sequence[Point], s: float) -> Tuple[Point, float]:
    """Return (x, y) and heading degrees at arc length s from the start of the polyline."""
    if not points:
        return (0.0, 0.0), 0.0
    if len(points) == 1 or s <= 0:
        heading = 0.0
        if len(points) > 1:
            heading = math.degrees(math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
        return points[0], heading
    remaining = s
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        seg = _dist(a, b)
        if seg < 1e-9:
            continue
        heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        if remaining <= seg:
            t = remaining / seg
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), heading
        remaining -= seg
    a, b = points[-2], points[-1]
    heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    return points[-1], heading


def remaining_polyline(points: Sequence[Point], s: float) -> List[Point]:
    pos, _ = point_on_polyline(points, s)
    total = 0.0
    idx = 0
    for i in range(len(points) - 1):
        seg = _dist(points[i], points[i + 1])
        if total + seg >= s:
            idx = i + 1
            break
        total += seg
        idx = i + 1
    rest = [pos] + list(points[idx:])
    cleaned: List[Point] = []
    for p in rest:
        if not cleaned or _dist(cleaned[-1], p) > 1e-6:
            cleaned.append(p)
    return cleaned or [pos]


@dataclass
class PathSpec:
    approach: str
    intention: str
    waypoints: List[Point]
    length_m: float

    def position_at(self, s: float) -> Tuple[Point, float]:
        return point_on_polyline(self.waypoints, s)


def build_path(
    approach: str,
    intention: str,
    start_distance_m: float,
    lane_offset_m: float = 2.5,
    exit_m: float = 80.0,
) -> PathSpec:
    """Build a right-hand-traffic polyline through a plus-shaped crossing."""
    if approach not in APPROACHES:
        raise ValueError(f"Unknown approach: {approach}")
    if intention not in INTENTIONS:
        raise ValueError(f"Unknown intention: {intention}")

    lane = lane_offset_m
    d0 = max(start_distance_m, 12.0)
    # Waypoints keep right-turning vehicles on the near corner so two right
    # turns do not intersect, while crossing/merge paths still do.
    if approach == "south":
        start = (lane, -d0)
        if intention == INTENTION_STRAIGHT:
            pts = [start, (lane, exit_m)]
        elif intention == INTENTION_RIGHT:
            pts = [start, (lane, -lane), (exit_m, -lane)]
        else:
            pts = [start, (lane, lane), (-exit_m, lane)]
    elif approach == "west":
        start = (-d0, -lane)
        if intention == INTENTION_STRAIGHT:
            pts = [start, (exit_m, -lane)]
        elif intention == INTENTION_RIGHT:
            pts = [start, (-lane, -lane), (-lane, -exit_m)]
        else:
            pts = [start, (lane, -lane), (lane, exit_m)]
    elif approach == "north":
        start = (-lane, d0)
        if intention == INTENTION_STRAIGHT:
            pts = [start, (-lane, -exit_m)]
        elif intention == INTENTION_RIGHT:
            pts = [start, (-lane, lane), (-exit_m, lane)]
        else:
            pts = [start, (-lane, -lane), (exit_m, -lane)]
    else:  # east
        start = (d0, lane)
        if intention == INTENTION_STRAIGHT:
            pts = [start, (-exit_m, lane)]
        elif intention == INTENTION_RIGHT:
            pts = [start, (lane, lane), (lane, exit_m)]
        else:
            pts = [start, (-lane, lane), (-lane, -exit_m)]

    return PathSpec(approach=approach, intention=intention, waypoints=pts, length_m=polyline_length(pts))


def _clamp01(t: float) -> float:
    return 0.0 if t < 0.0 else 1.0 if t > 1.0 else t


def _seg_closest(a1: Point, a2: Point, b1: Point, b2: Point) -> Tuple[float, Point, Point, float, float]:
    """Closest points between two segments, with parameters in [0, 1]."""
    ax, ay = a2[0] - a1[0], a2[1] - a1[1]
    bx, by = b2[0] - b1[0], b2[1] - b1[1]
    rx, ry = a1[0] - b1[0], a1[1] - b1[1]
    aa = ax * ax + ay * ay
    bb = bx * bx + by * by
    ab = ax * bx + ay * by
    ar = ax * rx + ay * ry
    br = bx * rx + by * ry
    denom = aa * bb - ab * ab

    if aa < 1e-12 and bb < 1e-12:
        return _dist(a1, b1), a1, b1, 0.0, 0.0
    if aa < 1e-12:
        t = _clamp01(br / bb) if bb > 1e-12 else 0.0
        pb = (b1[0] + t * bx, b1[1] + t * by)
        return _dist(a1, pb), a1, pb, 0.0, t
    if bb < 1e-12:
        s = _clamp01(-ar / aa)
        pa = (a1[0] + s * ax, a1[1] + s * ay)
        return _dist(pa, b1), pa, b1, s, 0.0

    if abs(denom) < 1e-12:
        s = _clamp01(-ar / aa)
        t = _clamp01((ab * s + br) / bb) if bb > 1e-12 else 0.0
    else:
        s = _clamp01((ab * br - bb * ar) / denom)
        t = (ab * s + br) / bb
        if t < 0.0:
            t = 0.0
            s = _clamp01(-ar / aa)
        elif t > 1.0:
            t = 1.0
            s = _clamp01((ab - ar) / aa)

    pa = (a1[0] + s * ax, a1[1] + s * ay)
    pb = (b1[0] + t * bx, b1[1] + t * by)
    return _dist(pa, pb), pa, pb, s, t


def closest_approach(
    path_a: Sequence[Point],
    path_b: Sequence[Point],
    step_m: float = 0.6,
) -> Tuple[float, Point, Point, float, float]:
    """Return (min_distance, point_a, point_b, s_a, s_b) along remaining paths."""
    del step_m  # retained for call-site compatibility; segments are exact
    if not path_a or not path_b:
        origin = path_a[0] if path_a else path_b[0]
        return float("inf"), origin, origin, 0.0, 0.0
    best = float("inf")
    pa, pb = path_a[0], path_b[0]
    best_sa = best_sb = 0.0
    sa_off = 0.0
    for i in range(max(len(path_a) - 1, 0)):
        a1, a2 = path_a[i], path_a[min(i + 1, len(path_a) - 1)]
        la = _dist(a1, a2)
        sb_off = 0.0
        for j in range(max(len(path_b) - 1, 0)):
            b1, b2 = path_b[j], path_b[min(j + 1, len(path_b) - 1)]
            lb = _dist(b1, b2)
            d, qa, qb, s, t = _seg_closest(a1, a2, b1, b2)
            if d < best:
                best = d
                pa, pb = qa, qb
                best_sa = sa_off + s * la
                best_sb = sb_off + t * lb
            sb_off += lb
        sa_off += la
    if len(path_a) == 1 or len(path_b) == 1:
        return _dist(path_a[0], path_b[0]), path_a[0], path_b[0], 0.0, 0.0
    return best, pa, pb, best_sa, best_sb


def heading_unit(heading_deg: float) -> Point:
    rad = math.radians(heading_deg)
    return (math.cos(rad), math.sin(rad))


def format_heading(heading_deg: float) -> str:
    h = heading_deg % 360.0
    names = [
        (0, "east"),
        (90, "north"),
        (180, "west"),
        (270, "south"),
    ]
    closest = min(names, key=lambda item: min(abs(h - item[0]), 360 - abs(h - item[0])))
    return f"{h:.0f}° ({closest[1]})"


def iter_road_lines(half: float = 90.0, lane: float = 2.5) -> Iterable[Tuple[Point, Point]]:
    yield ((-half, 0.0), (half, 0.0))
    yield ((0.0, -half), (0.0, half))
    yield ((-half, lane), (half, lane))
    yield ((-half, -lane), (half, -lane))
    yield ((lane, -half), (lane, half))
    yield ((-lane, -half), (-lane, half))
