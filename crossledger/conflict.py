"""Conflict prediction from intended paths and estimated arrival windows."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .config import SimConfig
from .geometry import closest_approach, remaining_polyline
from .vehicles import Vehicle, eta_or_inf


@dataclass
class ConflictAssessment:
    geometric: bool
    temporal: bool
    conflict: bool
    min_path_distance_m: float
    conflict_point_a: Optional[Tuple[float, float]]
    conflict_point_b: Optional[Tuple[float, float]]
    dist_a_m: float
    dist_b_m: float
    eta_a_s: Optional[float]
    eta_b_s: Optional[float]
    window_a: Tuple[Optional[float], Optional[float]]
    window_b: Tuple[Optional[float], Optional[float]]
    overlap_s: float
    reason: str
    occupying: List[str]

    @property
    def paths_intersect(self) -> bool:
        return self.geometric


def _arrival_window(
    dist_m: float,
    speed_mps: float,
    length_m: float,
    padding_m: float,
    stopped_eps: float,
) -> Tuple[Optional[float], Optional[float]]:
    occupancy = length_m + padding_m
    if speed_mps <= stopped_eps:
        if dist_m <= occupancy:
            return (0.0, float("inf"))
        return (None, None)
    t0 = max(dist_m - 0.5 * length_m, 0.0) / speed_mps
    t1 = (dist_m + 0.5 * length_m + padding_m) / speed_mps
    return (t0, t1)


def _windows_overlap(
    a: Tuple[Optional[float], Optional[float]],
    b: Tuple[Optional[float], Optional[float]],
    margin_s: float,
) -> Tuple[bool, float]:
    if a[0] is None or a[1] is None or b[0] is None or b[1] is None:
        return False, 0.0
    a0, a1 = a[0] - margin_s, a[1] + margin_s
    b0, b1 = b[0] - margin_s, b[1] + margin_s
    lo = max(a0, b0)
    hi = min(a1, b1)
    if hi <= lo:
        return False, 0.0
    if math.isinf(hi - lo):
        return True, float("inf")
    return True, hi - lo


def assess_conflict(va: Vehicle, vb: Vehicle, cfg: SimConfig) -> ConflictAssessment:
    rest_a = remaining_polyline(va.path.waypoints, va.s_m)
    rest_b = remaining_polyline(vb.path.waypoints, vb.s_m)
    min_d, pa, pb, sa, sb = closest_approach(rest_a, rest_b)
    geometric = min_d <= cfg.path_conflict_threshold_m

    occupying = []
    if va.in_intersection(cfg.intersection_half_m) and va.speed_mps <= cfg.stopped_speed_mps:
        occupying.append(va.vehicle_id)
    if vb.in_intersection(cfg.intersection_half_m) and vb.speed_mps <= cfg.stopped_speed_mps:
        occupying.append(vb.vehicle_id)

    win_a = _arrival_window(
        sa, va.speed_mps, cfg.vehicle_length_m, cfg.occupancy_padding_m, cfg.stopped_speed_mps
    )
    win_b = _arrival_window(
        sb, vb.speed_mps, cfg.vehicle_length_m, cfg.occupancy_padding_m, cfg.stopped_speed_mps
    )
    temporal, overlap = _windows_overlap(win_a, win_b, cfg.safety_margin_s)

    eta_a = None if va.speed_mps <= cfg.stopped_speed_mps and sa > cfg.vehicle_length_m else (
        0.0 if va.speed_mps <= cfg.stopped_speed_mps else sa / max(va.speed_mps, 1e-9)
    )
    eta_b = None if vb.speed_mps <= cfg.stopped_speed_mps and sb > cfg.vehicle_length_m else (
        0.0 if vb.speed_mps <= cfg.stopped_speed_mps else sb / max(vb.speed_mps, 1e-9)
    )

    if not geometric:
        reason = (
            f"Intended paths stay {min_d:.1f} m apart (threshold {cfg.path_conflict_threshold_m:.1f} m), "
            "so proximity at the site is not treated as a crossing conflict."
        )
        conflict = False
        temporal = False
    elif occupying and geometric:
        reason = (
            f"Geometric path conflict (closest approach {min_d:.1f} m) and "
            f"stopped vehicle occupying the crossing: {', '.join(occupying)}."
        )
        conflict = True
        temporal = True
        overlap = float("inf")
    elif not temporal:
        reason = (
            f"Paths would intersect (closest approach {min_d:.1f} m) but arrival windows "
            f"are separated by more than the {cfg.safety_margin_s:.1f} s safety margin."
        )
        conflict = False
    else:
        reason = (
            f"Conflicting intended paths (closest approach {min_d:.1f} m) with overlapping "
            f"arrival windows ({overlap:.2f} s including a {cfg.safety_margin_s:.1f} s margin)."
        )
        conflict = True

    return ConflictAssessment(
        geometric=geometric,
        temporal=temporal if geometric else False,
        conflict=conflict,
        min_path_distance_m=min_d,
        conflict_point_a=pa if geometric else None,
        conflict_point_b=pb if geometric else None,
        dist_a_m=sa,
        dist_b_m=sb,
        eta_a_s=eta_a,
        eta_b_s=eta_b,
        window_a=win_a,
        window_b=win_b,
        overlap_s=overlap if geometric else 0.0,
        reason=reason,
        occupying=occupying,
    )


def assess_from_snapshots(
    snap_a: dict,
    snap_b: dict,
    path_a: Sequence[Tuple[float, float]],
    path_b: Sequence[Tuple[float, float]],
    cfg: SimConfig,
    s_a: float,
    s_b: float,
    waypoints_a,
    waypoints_b,
) -> ConflictAssessment:
    """Rebuild lightweight vehicles from last-known telemetry for conservative prediction."""
    va = Vehicle(
        vehicle_id=snap_a["vehicle_id"],
        label=snap_a["vehicle_id"],
        approach=snap_a["approach"],
        intention=snap_a["intention"],
        priority=int(snap_a["priority"]),
        speed_mps=float(snap_a["speed_mps"]),
        start_distance_m=max(float(snap_a.get("distance_to_intersection_m", 40.0)), 12.0),
        color="#000",
    )
    vb = Vehicle(
        vehicle_id=snap_b["vehicle_id"],
        label=snap_b["vehicle_id"],
        approach=snap_b["approach"],
        intention=snap_b["intention"],
        priority=int(snap_b["priority"]),
        speed_mps=float(snap_b["speed_mps"]),
        start_distance_m=max(float(snap_b.get("distance_to_intersection_m", 40.0)), 12.0),
        color="#000",
    )
    # Place each vehicle using reported remaining distance if available.
    rem_a = float(snap_a.get("path_remaining_m", va.path.length_m))
    rem_b = float(snap_b.get("path_remaining_m", vb.path.length_m))
    va.s_m = max(va.path.length_m - rem_a, 0.0)
    vb.s_m = max(vb.path.length_m - rem_b, 0.0)
    return assess_conflict(va, vb, cfg)


def ground_truth_conflict(vehicles: Sequence[Vehicle], cfg: SimConfig) -> ConflictAssessment:
    if len(vehicles) < 2:
        raise ValueError("Need two vehicles")
    return assess_conflict(vehicles[0], vehicles[1], cfg)


def finite_eta_text(eta: Optional[float]) -> str:
    if eta is None or not math.isfinite(eta_or_inf(eta)):
        return "—"
    return f"{eta:.1f} s"
