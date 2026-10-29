"""Time-stepped two-vehicle simulation with a simulated V2V layer."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .channel import Delivery, SimulatedV2V
from .config import SimConfig
from .conflict import ConflictAssessment, assess_conflict
from .coordination import CoordinationResult, coordinate
from .messages import TelemetryPayload
from .security import MessageAuthenticator
from .vehicles import Vehicle, default_vehicles


@dataclass
class LogEvent:
    t: float
    kind: str
    text: str


@dataclass
class VehicleView:
    vehicle: Vehicle
    last_delivery: Optional[Delivery] = None
    last_rx_time: Optional[float] = None


@dataclass
class TickResult:
    t: float
    truth: ConflictAssessment
    local: ConflictAssessment
    coordination: CoordinationResult
    deliveries: List[Delivery]
    comms_ok: bool
    peer_age_s: Optional[float]


class Simulation:
    def __init__(self, cfg: Optional[SimConfig] = None, seed: int = 7):
        self.cfg = cfg or SimConfig()
        self.rng = random.Random(seed)
        self.auth = MessageAuthenticator(self.cfg.hmac_key)
        self.channel = SimulatedV2V(self.auth, rng=self.rng)
        self.vehicles: List[Vehicle] = default_vehicles(self.cfg)
        self.t = 0.0
        self.events: List[LogEvent] = []
        self.position_errors: List[float] = []
        self.alert_times: List[float] = []
        self._conflict_onset: Optional[float] = None
        self._alert_logged = False
        self.last_tick: Optional[TickResult] = None
        self.history: List[TickResult] = []
        self.paused = True
        self.last_local_assessment: Optional[ConflictAssessment] = None
        self.last_coordination: Optional[CoordinationResult] = None
        self.broadcast_every = 1  # ticks
        self._tick_i = 0
        self.comms_ok = True
        self._last_good = {}
        self.log("system", "Simulation initialized. V2V layer is in-process and simulated.")

    def log(self, kind: str, text: str) -> None:
        self.events.append(LogEvent(self.t, kind, text))
        if len(self.events) > 400:
            self.events = self.events[-400:]

    def reset(self, seed: Optional[int] = None) -> None:
        if seed is not None:
            self.rng.seed(seed)
        cfg = self.cfg
        speeds = [v.speed_mps for v in self.vehicles]
        starts = [v.start_distance_m for v in self.vehicles]
        intents = [v.intention for v in self.vehicles]
        approaches = [v.approach for v in self.vehicles]
        prios = [v.priority for v in self.vehicles]
        self.__init__(cfg, seed=seed if seed is not None else 7)
        for v, sp, st, it, ap, pr in zip(self.vehicles, speeds, starts, intents, approaches, prios):
            v.speed_mps = sp
            v.start_distance_m = st
            v.intention = it
            v.approach = ap
            v.priority = pr
            v.s_m = 0.0
            v.stopped = False
            v.rebuild_path()

    def apply_vehicle_settings(
        self,
        index: int,
        speed: Optional[float] = None,
        start_distance: Optional[float] = None,
        intention: Optional[str] = None,
        approach: Optional[str] = None,
        priority: Optional[int] = None,
        reset_pose: bool = False,
    ) -> None:
        v = self.vehicles[index]
        if speed is not None:
            v.speed_mps = float(speed)
            if v.speed_mps <= self.cfg.stopped_speed_mps:
                v.stopped = True
            else:
                v.stopped = False
        path_changed = False
        if intention is not None and intention != v.intention:
            path_changed = True
            v.intention = intention
        elif intention is not None:
            v.intention = intention
        if approach is not None and approach != v.approach:
            path_changed = True
            v.approach = approach
        elif approach is not None:
            v.approach = approach
        if priority is not None:
            v.priority = int(priority)
        if start_distance is not None:
            if abs(float(start_distance) - v.start_distance_m) > 1e-9:
                path_changed = True
            v.start_distance_m = float(start_distance)
        if reset_pose or path_changed:
            if reset_pose or path_changed:
                v.s_m = 0.0
            v.rebuild_path()

    def _noisy_pose(self, v: Vehicle) -> tuple:
        nx = self.rng.gauss(0.0, self.cfg.position_noise_std_m)
        ny = self.rng.gauss(0.0, self.cfg.position_noise_std_m)
        nh = self.rng.gauss(0.0, self.cfg.heading_noise_std_deg)
        x, y = v.position
        return x + nx, y + ny, v.heading_deg + nh

    def _broadcast(self, v: Vehicle) -> None:
        v.seq += 1
        x, y, heading = self._noisy_pose(v)
        tx, ty = v.position
        eta = v.eta_to_intersection(self.cfg)
        payload = TelemetryPayload(
            vehicle_id=v.vehicle_id,
            seq=v.seq,
            ts=self.t,
            x=x,
            y=y,
            speed_mps=0.0 if v.stopped else v.speed_mps,
            heading_deg=heading,
            intention=v.intention,
            approach=v.approach,
            priority=v.priority,
            path_remaining_m=v.path_remaining_m,
            eta_s=eta,
            truth_x=tx,
            truth_y=ty,
        )
        rec = self.channel.transmit(
            payload,
            self.t,
            self.cfg.mean_latency_s,
            self.cfg.latency_jitter_s,
            self.cfg.packet_loss,
        )
        if rec.tampered:
            self.log("security", f"{v.vehicle_id} transmitted a tampered telemetry frame (demo).")
        if rec.dropped:
            self.log("radio", f"{v.vehicle_id} seq={v.seq} dropped by simulated channel.")

    def _vehicle_from_payload(self, payload: TelemetryPayload, template: Vehicle) -> Vehicle:
        clone = Vehicle(
            vehicle_id=payload.vehicle_id,
            label=template.label,
            approach=payload.approach,
            intention=payload.intention,
            priority=payload.priority,
            speed_mps=payload.speed_mps,
            start_distance_m=template.start_distance_m,
            color=template.color,
        )
        # Snap onto reconstructed path using remaining distance from the message.
        clone.s_m = max(clone.path.length_m - payload.path_remaining_m, 0.0)
        clone.stopped = payload.speed_mps <= self.cfg.stopped_speed_mps
        err = math.hypot(payload.x - payload.truth_x, payload.y - payload.truth_y)
        self.position_errors.append(err)
        return clone

    def _freshness(self, last_rx: Optional[float]) -> bool:
        if last_rx is None:
            return False
        return (self.t - last_rx) <= self.cfg.stale_after_s

    def step(self) -> TickResult:
        cfg = self.cfg
        for v in self.vehicles:
            v.step(cfg.dt, cfg)

        if self._tick_i % self.broadcast_every == 0:
            for v in self.vehicles:
                self._broadcast(v)
        self._tick_i += 1

        deliveries = self.channel.receive_due(self.t)
        rx_by_sender: Dict[str, Delivery] = {}
        for d in deliveries:
            if d.rejected:
                self.log(
                    "security",
                    f"Rejected {d.payload.vehicle_id} seq={d.payload.seq}: {d.reject_reason}",
                )
                continue
            rx_by_sender[d.payload.vehicle_id] = d
            self.log(
                "radio",
                f"RX {d.payload.vehicle_id} seq={d.payload.seq}  latency={d.latency_s * 1000:.1f} ms (simulated)",
            )

        for sender, d in rx_by_sender.items():
            self._last_good[sender] = d

        truth = assess_conflict(self.vehicles[0], self.vehicles[1], cfg)

        v0, v1 = self.vehicles
        d0 = self._last_good.get(v0.vehicle_id)
        d1 = self._last_good.get(v1.vehicle_id)
        age0 = None if d0 is None else self.t - d0.recv_time
        age1 = None if d1 is None else self.t - d1.recv_time
        fresh = self._freshness(None if d0 is None else d0.recv_time) and self._freshness(
            None if d1 is None else d1.recv_time
        )

        reject_reason = None
        recent_rejects = [d for d in deliveries if d.rejected]
        if recent_rejects:
            reject_reason = recent_rejects[-1].reject_reason
        if not fresh:
            oldest = []
            if age0 is None:
                oldest.append(f"no authenticated telemetry from {v0.vehicle_id}")
            elif age0 > cfg.stale_after_s:
                oldest.append(f"{v0.vehicle_id} stale ({age0:.2f} s)")
            if age1 is None:
                oldest.append(f"no authenticated telemetry from {v1.vehicle_id}")
            elif age1 > cfg.stale_after_s:
                oldest.append(f"{v1.vehicle_id} stale ({age1:.2f} s)")
            reject_reason = reject_reason or ("; ".join(oldest) if oldest else "stale telemetry")

        if fresh and d0 is not None and d1 is not None:
            local_a = self._vehicle_from_payload(d0.payload, v0)
            local_b = self._vehicle_from_payload(d1.payload, v1)
            local = assess_conflict(local_a, local_b, cfg)
            coord = coordinate(local_a, local_b, local, cfg, True, None)
        else:
            local = truth
            coord = coordinate(v0, v1, truth, cfg, False, reject_reason)

        if truth.conflict and self._conflict_onset is None:
            self._conflict_onset = self.t
        if (
            not self._alert_logged
            and self._conflict_onset is not None
            and any(r.action in ("YIELD", "HOLD") for r in coord.recommendations.values())
        ):
            self.alert_times.append(self.t - self._conflict_onset)
            self._alert_logged = True
            self.log("alert", f"Protective recommendation issued {self.alert_times[-1]:.2f} s after conflict onset.")

        prev = self.last_coordination
        if prev is None or any(
            prev.recommendations[k].action != coord.recommendations[k].action for k in coord.recommendations
        ):
            for rec in coord.recommendations.values():
                self.log("decision", f"{rec.vehicle_id}: {rec.action} — {rec.explanation}")

        peer_age = None
        if age0 is not None and age1 is not None:
            peer_age = max(age0, age1)
        elif age0 is not None:
            peer_age = age0
        elif age1 is not None:
            peer_age = age1

        self.comms_ok = fresh and self.cfg.packet_loss < 0.5
        result = TickResult(
            t=self.t,
            truth=truth,
            local=local,
            coordination=coord,
            deliveries=deliveries,
            comms_ok=self.comms_ok,
            peer_age_s=peer_age,
        )
        self.last_tick = result
        self.last_local_assessment = local
        self.last_coordination = coord
        self.history.append(result)
        if len(self.history) > 2500:
            self.history = self.history[-2500:]
        self.t += cfg.dt
        return result

    def run_for(self, duration_s: float) -> List[TickResult]:
        steps = max(1, int(round(duration_s / self.cfg.dt)))
        out = []
        for _ in range(steps):
            out.append(self.step())
        return out

    def inject_tamper(self, vehicle_id: str) -> None:
        self.channel.tamper_next_from = vehicle_id
        self.log("security", f"Next outbound message from {vehicle_id} will be tampered after signing.")

    def inject_replay(self, vehicle_id: str) -> None:
        self.channel.force_replay_from = vehicle_id
        self.log("security", f"Next outbound message from {vehicle_id} will replay a previous authenticated frame.")

    def sudden_stop(self, index: int = 1) -> None:
        v = self.vehicles[index]
        v.speed_mps = 0.0
        v.stopped = True
        self.log("scenario", f"{v.vehicle_id} executed a sudden stop.")
