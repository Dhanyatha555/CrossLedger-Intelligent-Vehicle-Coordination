"""Construction-vehicle kinematics along intent-based paths."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .config import SimConfig
from .geometry import PathSpec, build_path, remaining_polyline


@dataclass
class Vehicle:
    vehicle_id: str
    label: str
    approach: str
    intention: str
    priority: int
    speed_mps: float
    start_distance_m: float
    color: str
    s_m: float = 0.0
    stopped: bool = False
    seq: int = 0
    path: PathSpec = field(init=False)

    def __post_init__(self) -> None:
        self.rebuild_path()

    def rebuild_path(self) -> None:
        remaining_start = max(self.start_distance_m - self.s_m, 1.0) if self.s_m else self.start_distance_m
        # Rebuild from original start so the full polyline stays consistent.
        self.path = build_path(self.approach, self.intention, self.start_distance_m)
        self.s_m = min(self.s_m, max(self.path.length_m - 0.1, 0.0))
        _ = remaining_start

    @property
    def position(self) -> Tuple[float, float]:
        p, _ = self.path.position_at(self.s_m)
        return p

    @property
    def heading_deg(self) -> float:
        _, h = self.path.position_at(self.s_m)
        return h

    @property
    def path_remaining_m(self) -> float:
        return max(self.path.length_m - self.s_m, 0.0)

    def distance_to_intersection(self, box_half: float) -> float:
        """Approximate remaining distance along path until the vehicle enters the crossing box."""
        s = self.s_m
        step = 0.5
        while s < self.path.length_m:
            (x, y), _ = self.path.position_at(s)
            if abs(x) <= box_half and abs(y) <= box_half:
                return max(s - self.s_m, 0.0)
            s += step
        return self.path_remaining_m

    def eta_to_intersection(self, cfg: SimConfig) -> Optional[float]:
        dist = self.distance_to_intersection(cfg.intersection_half_m)
        if self.speed_mps <= cfg.stopped_speed_mps:
            if dist <= 0.75:
                return 0.0
            return None
        return dist / self.speed_mps

    def in_intersection(self, box_half: float) -> bool:
        x, y = self.position
        return abs(x) <= box_half and abs(y) <= box_half

    def remaining_waypoints(self):
        return remaining_polyline(self.path.waypoints, self.s_m)

    def step(self, dt: float, cfg: SimConfig) -> None:
        if self.stopped:
            return
        speed = min(max(self.speed_mps, cfg.min_speed_mps), cfg.max_speed_mps)
        self.s_m = min(self.s_m + speed * dt, self.path.length_m)

    def snapshot(self, cfg: SimConfig) -> dict:
        x, y = self.position
        eta = self.eta_to_intersection(cfg)
        return {
            "vehicle_id": self.vehicle_id,
            "label": self.label,
            "x": x,
            "y": y,
            "speed_mps": self.speed_mps,
            "heading_deg": self.heading_deg,
            "intention": self.intention,
            "approach": self.approach,
            "priority": self.priority,
            "path_remaining_m": self.path_remaining_m,
            "distance_to_intersection_m": self.distance_to_intersection(cfg.intersection_half_m),
            "eta_s": eta,
            "stopped": self.stopped,
            "in_intersection": self.in_intersection(cfg.intersection_half_m),
        }


def eta_or_inf(eta: Optional[float]) -> float:
    if eta is None or not math.isfinite(eta):
        return float("inf")
    return eta


def default_vehicles(cfg: SimConfig) -> List[Vehicle]:
    return [
        Vehicle(
            vehicle_id="EX-101",
            label="Excavator EX-101",
            approach="south",
            intention="straight",
            priority=2,
            speed_mps=cfg.default_speed_mps,
            start_distance_m=cfg.default_start_distance_m,
            color="#e67e22",
        ),
        Vehicle(
            vehicle_id="DT-204",
            label="Dump truck DT-204",
            approach="west",
            intention="straight",
            priority=2,
            speed_mps=cfg.default_speed_mps,
            start_distance_m=cfg.default_start_distance_m,
            color="#2980b9",
        ),
    ]
