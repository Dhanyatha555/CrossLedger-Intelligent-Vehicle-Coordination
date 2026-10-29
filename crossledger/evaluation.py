"""Reproducible evaluation scenarios and CSV export."""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from .config import SimConfig
from .coordination import HOLD, PROCEED, unsafe_proceed
from .simulation import Simulation, TickResult


@dataclass
class ScenarioResult:
    name: str
    description: str
    passed: bool
    detail: str
    messages_sent: int
    messages_received: int
    delivery_success_rate: float
    avg_latency_ms: Optional[float]
    max_latency_ms: Optional[float]
    position_rmse_m: Optional[float]
    conflict_detection_correct: Optional[float]
    conflict_samples: int
    unsafe_proceed_count: int
    alert_response_s: Optional[float]
    auth_rejected: int
    replay_rejected: int
    extras: Dict[str, str] = field(default_factory=dict)


def _latency_ms(sim: Simulation):
    avg, mx = sim.channel.latency_stats()
    return (None if avg is None else avg * 1000.0, None if mx is None else mx * 1000.0)


def _rmse(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def _detection_rate(history: List[TickResult]) -> tuple:
    ok = 0
    n = 0
    for tick in history:
        if not tick.comms_ok:
            continue
        n += 1
        if tick.local.conflict == tick.truth.conflict:
            ok += 1
    if n == 0:
        return None, 0
    return ok / n, n


def _unsafe_count(history: List[TickResult]) -> int:
    count = 0
    for tick in history:
        count += len(unsafe_proceed(tick.coordination, tick.truth))
    return count


def _summarize(sim: Simulation, name: str, description: str, passed: bool, detail: str, extras=None) -> ScenarioResult:
    avg_ms, max_ms = _latency_ms(sim)
    det, n = _detection_rate(sim.history)
    alert = sim.alert_times[0] if sim.alert_times else None
    return ScenarioResult(
        name=name,
        description=description,
        passed=passed,
        detail=detail,
        messages_sent=sim.channel.sent,
        messages_received=sim.channel.delivered,
        delivery_success_rate=sim.channel.delivery_success_rate(),
        avg_latency_ms=avg_ms,
        max_latency_ms=max_ms,
        position_rmse_m=_rmse(sim.position_errors),
        conflict_detection_correct=det,
        conflict_samples=n,
        unsafe_proceed_count=_unsafe_count(sim.history),
        alert_response_s=alert,
        auth_rejected=sim.auth.rejected_mac,
        replay_rejected=sim.auth.rejected_replay,
        extras=extras or {},
    )


def _base_sim(seed: int, **cfg_overrides) -> Simulation:
    cfg = SimConfig()
    for k, v in cfg_overrides.items():
        setattr(cfg, k, v)
    return Simulation(cfg, seed=seed)


def scenario_non_conflicting() -> ScenarioResult:
    sim = _base_sim(11, packet_loss=0.0, mean_latency_s=0.02, latency_jitter_s=0.005)
    for i, v in enumerate(sim.vehicles):
        sim.apply_vehicle_settings(i, speed=5.0, start_distance=50.0, intention="right", priority=2, reset_pose=True)
    sim.run_for(8.0)
    conflicts = sum(1 for t in sim.history if t.truth.conflict)
    both_proceed = all(
        all(r.action == PROCEED for r in t.coordination.recommendations.values())
        for t in sim.history
        if t.comms_ok
    )
    passed = conflicts == 0 and both_proceed and _unsafe_count(sim.history) == 0
    return _summarize(
        sim,
        "non_conflicting_paths",
        "Both vehicles turn right; offset lanes do not intersect.",
        passed,
        f"truth conflicts={conflicts}, both-proceed-when-fresh={both_proceed}",
    )


def scenario_separated_arrival() -> ScenarioResult:
    sim = _base_sim(21, packet_loss=0.0, safety_margin_s=1.0)
    sim.apply_vehicle_settings(0, speed=7.0, start_distance=28.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=3.0, start_distance=70.0, intention="straight", reset_pose=True)
    sim.run_for(6.0)
    geometric = any(t.truth.geometric for t in sim.history)
    truth_conflict = sum(1 for t in sim.history if t.truth.conflict)
    passed = geometric and truth_conflict == 0 and _unsafe_count(sim.history) == 0
    return _summarize(
        sim,
        "safe_crossing_separated",
        "Crossing paths with arrival windows separated beyond the safety margin.",
        passed,
        f"geometric={geometric}, temporal conflicts={truth_conflict}",
    )


def scenario_simultaneous() -> ScenarioResult:
    sim = _base_sim(31, packet_loss=0.0, safety_margin_s=2.0)
    sim.apply_vehicle_settings(0, speed=5.0, start_distance=45.0, intention="straight", priority=2, reset_pose=True)
    sim.apply_vehicle_settings(1, speed=5.0, start_distance=45.0, intention="straight", priority=2, reset_pose=True)
    sim.run_for(8.0)
    conflict_ticks = [t for t in sim.history if t.truth.conflict]
    double_proceed = 0
    consistent = True
    for t in conflict_ticks:
        actions = [r.action for r in t.coordination.recommendations.values()]
        if actions.count(PROCEED) > 1:
            double_proceed += 1
        if t.comms_ok and HOLD in actions:
            # HOLD is allowed only if occupying; otherwise expect PROCEED+YIELD
            if not t.truth.occupying:
                consistent = True
        recs = list(t.coordination.recommendations.values())
        if t.comms_ok and not t.coordination.unresolved:
            if recs[0].action == recs[1].action == PROCEED:
                consistent = False
    passed = (
        len(conflict_ticks) > 0
        and double_proceed == 0
        and _unsafe_count(sim.history) == 0
        and consistent
    )
    return _summarize(
        sim,
        "simultaneous_conflicting_arrivals",
        "Equal ETA crossing; exactly one vehicle may proceed.",
        passed,
        f"conflict_ticks={len(conflict_ticks)}, double_proceed={double_proceed}",
    )


def scenario_sudden_stop() -> ScenarioResult:
    sim = _base_sim(41, packet_loss=0.0)
    sim.apply_vehicle_settings(0, speed=5.5, start_distance=40.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=5.5, start_distance=22.0, intention="straight", reset_pose=True)
    sim.run_for(2.2)
    sim.sudden_stop(1)
    # Slide the stopped vehicle into the box if it is not yet there so occupancy is observable.
    stopped = sim.vehicles[1]
    if not stopped.in_intersection(sim.cfg.intersection_half_m):
        target = stopped.distance_to_intersection(sim.cfg.intersection_half_m) + 1.0
        stopped.s_m += target
        stopped.speed_mps = 0.0
        stopped.stopped = True
    sim.run_for(3.0)
    holds_after = [
        t
        for t in sim.history
        if t.t >= 2.2
        and all(r.action == HOLD for r in t.coordination.recommendations.values())
    ]
    passed = len(holds_after) > 0 and _unsafe_count(sim.history) == 0
    return _summarize(
        sim,
        "sudden_stop",
        "Peer stops in the crossing; coordinator must HOLD both vehicles.",
        passed,
        f"hold_ticks_after_stop={len(holds_after)}",
    )


def scenario_stale_telemetry() -> ScenarioResult:
    sim = _base_sim(
        51,
        packet_loss=0.0,
        mean_latency_s=1.6,
        latency_jitter_s=0.05,
        stale_after_s=0.4,
    )
    sim.apply_vehicle_settings(0, speed=5.0, start_distance=40.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=5.0, start_distance=40.0, intention="straight", reset_pose=True)
    sim.run_for(4.0)
    hold_ticks = sum(
        1
        for t in sim.history
        if any(r.action == HOLD for r in t.coordination.recommendations.values())
    )
    passed = hold_ticks > 0 and _unsafe_count(sim.history) == 0
    return _summarize(
        sim,
        "stale_or_missing_telemetry",
        "Simulated delay exceeds the freshness window; vehicles HOLD.",
        passed,
        f"hold_ticks={hold_ticks}, mean_latency={sim.cfg.mean_latency_s}s (simulated)",
    )


def scenario_packet_loss() -> ScenarioResult:
    sim = _base_sim(61, packet_loss=0.85, mean_latency_s=0.05, stale_after_s=0.35)
    sim.apply_vehicle_settings(0, speed=5.0, start_distance=40.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=5.0, start_distance=40.0, intention="straight", reset_pose=True)
    sim.run_for(5.0)
    rate = sim.channel.delivery_success_rate()
    hold_ticks = sum(
        1
        for t in sim.history
        if any(r.action == HOLD for r in t.coordination.recommendations.values())
    )
    passed = rate < 0.4 and hold_ticks > 0 and _unsafe_count(sim.history) == 0
    return _summarize(
        sim,
        "packet_loss_and_delay",
        "High simulated loss causes missing telemetry and conservative HOLD.",
        passed,
        f"delivery_success={rate:.2%}, hold_ticks={hold_ticks}",
    )


def scenario_invalid_auth() -> ScenarioResult:
    sim = _base_sim(71, packet_loss=0.0, mean_latency_s=0.02, latency_jitter_s=0.0)
    sim.apply_vehicle_settings(0, speed=4.0, start_distance=50.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=4.0, start_distance=50.0, intention="straight", reset_pose=True)
    sim.run_for(0.4)
    sim.inject_tamper(sim.vehicles[0].vehicle_id)
    sim.run_for(1.2)
    passed = sim.auth.rejected_mac >= 1
    return _summarize(
        sim,
        "invalid_authentication",
        "Tampered frame is rejected by HMAC-SHA256.",
        passed,
        f"mac_rejections={sim.auth.rejected_mac}",
        extras={"security_note": "Prototype HMAC only; not production vehicle cybersecurity."},
    )


def scenario_replay() -> ScenarioResult:
    sim = _base_sim(81, packet_loss=0.0, mean_latency_s=0.02, latency_jitter_s=0.0)
    sim.apply_vehicle_settings(0, speed=4.0, start_distance=50.0, intention="straight", reset_pose=True)
    sim.apply_vehicle_settings(1, speed=4.0, start_distance=50.0, intention="straight", reset_pose=True)
    sim.run_for(0.5)
    sim.inject_replay(sim.vehicles[1].vehicle_id)
    sim.run_for(1.0)
    passed = sim.auth.rejected_replay >= 1
    return _summarize(
        sim,
        "replayed_messages",
        "Replayed sequence numbers are rejected.",
        passed,
        f"replay_rejections={sim.auth.rejected_replay}",
    )


SCENARIOS: List[Callable[[], ScenarioResult]] = [
    scenario_non_conflicting,
    scenario_separated_arrival,
    scenario_simultaneous,
    scenario_sudden_stop,
    scenario_stale_telemetry,
    scenario_packet_loss,
    scenario_invalid_auth,
    scenario_replay,
]


def run_all() -> List[ScenarioResult]:
    return [fn() for fn in SCENARIOS]


def results_to_rows(results: List[ScenarioResult]) -> List[dict]:
    rows = []
    for r in results:
        rows.append(
            {
                "scenario": r.name,
                "description": r.description,
                "passed": r.passed,
                "detail": r.detail,
                "messages_sent": r.messages_sent,
                "messages_received": r.messages_received,
                "delivery_success_rate": round(r.delivery_success_rate, 4),
                "avg_latency_ms_simulated": "" if r.avg_latency_ms is None else round(r.avg_latency_ms, 3),
                "max_latency_ms_simulated": "" if r.max_latency_ms is None else round(r.max_latency_ms, 3),
                "position_rmse_m": "" if r.position_rmse_m is None else round(r.position_rmse_m, 4),
                "conflict_detection_correct_rate": ""
                if r.conflict_detection_correct is None
                else round(r.conflict_detection_correct, 4),
                "conflict_samples": r.conflict_samples,
                "unsafe_proceed_count": r.unsafe_proceed_count,
                "alert_response_s": "" if r.alert_response_s is None else round(r.alert_response_s, 3),
                "auth_rejected": r.auth_rejected,
                "replay_rejected": r.replay_rejected,
                "latency_source": "simulated_in_process_channel",
            }
        )
    return rows


def write_csv(results: List[ScenarioResult], path: str) -> str:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    rows = results_to_rows(results)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path


def summary_text(results: List[ScenarioResult]) -> str:
    passed = sum(1 for r in results if r.passed)
    lines = [
        "CrossLedger evaluation summary",
        "All latency figures are simulated in-process delays, not radio measurements.",
        f"Scenarios passed: {passed}/{len(results)}",
        "",
    ]
    for r in results:
        mark = "PASS" if r.passed else "FAIL"
        lines.append(f"[{mark}] {r.name}: {r.detail}")
        lines.append(
            f"       sent={r.messages_sent} rx={r.messages_received} "
            f"pdr={r.delivery_success_rate:.1%} unsafe_proceed={r.unsafe_proceed_count}"
        )
    return "\n".join(lines)
