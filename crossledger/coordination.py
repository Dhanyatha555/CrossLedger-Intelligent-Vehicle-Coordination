"""Deterministic crossing coordination: PROCEED, YIELD, or HOLD."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .config import SimConfig
from .conflict import ConflictAssessment
from .vehicles import Vehicle, eta_or_inf

PROCEED = "PROCEED"
YIELD = "YIELD"
HOLD = "HOLD"


@dataclass
class Recommendation:
    vehicle_id: str
    action: str
    explanation: str


@dataclass
class CoordinationResult:
    recommendations: Dict[str, Recommendation]
    winner_id: Optional[str]
    unresolved: bool
    rule: str

    def action_for(self, vehicle_id: str) -> str:
        return self.recommendations[vehicle_id].action


def _choose_winner(va: Vehicle, vb: Vehicle, assessment: ConflictAssessment) -> Tuple[str, str]:
    pa, pb = va.priority, vb.priority
    if pa != pb:
        winner = va.vehicle_id if pa > pb else vb.vehicle_id
        return winner, f"higher site priority ({max(pa, pb)} > {min(pa, pb)})"

    eta_a = assessment.eta_a_s
    eta_b = assessment.eta_b_s
    fa, fb = eta_or_inf(eta_a), eta_or_inf(eta_b)
    if abs(fa - fb) > 0.05:
        if fa < fb:
            return va.vehicle_id, f"earlier arrival at the conflict point ({fa:.1f} s vs {fb:.1f} s)"
        return vb.vehicle_id, f"earlier arrival at the conflict point ({fb:.1f} s vs {fa:.1f} s)"

    winner = min(va.vehicle_id, vb.vehicle_id)
    return winner, "equal priority and arrival time; vehicle ID tie-break"


def coordinate(
    va: Vehicle,
    vb: Vehicle,
    assessment: ConflictAssessment,
    cfg: SimConfig,
    peer_fresh: bool,
    reject_reason: Optional[str] = None,
) -> CoordinationResult:
    ids = (va.vehicle_id, vb.vehicle_id)

    if not peer_fresh:
        detail = reject_reason or (
            f"Peer telemetry is missing, invalid, or older than {cfg.stale_after_s:.2f} s"
        )
        recs = {
            vid: Recommendation(
                vid,
                HOLD,
                f"{detail}. HOLD until authenticated, fresh intent is available. "
                "The coordinator will not authorize entering an unresolved blind crossing.",
            )
            for vid in ids
        }
        return CoordinationResult(recs, None, True, "conservative-hold-stale-or-invalid")

    if assessment.occupying:
        recs = {
            vid: Recommendation(
                vid,
                HOLD,
                assessment.reason
                + " Both vehicles HOLD while a stopped vehicle occupies the conflict area.",
            )
            for vid in ids
        }
        return CoordinationResult(recs, None, True, "hold-occupied-crossing")

    if not assessment.conflict:
        recs = {
            va.vehicle_id: Recommendation(
                va.vehicle_id,
                PROCEED,
                assessment.reason + " EX-101 / this vehicle may continue on its intended path.",
            ),
            vb.vehicle_id: Recommendation(
                vb.vehicle_id,
                PROCEED,
                assessment.reason + " The peer may continue on its intended path.",
            ),
        }
        recs[va.vehicle_id].explanation = (
            assessment.reason + f" {va.vehicle_id} may continue."
        )
        recs[vb.vehicle_id].explanation = (
            assessment.reason + f" {vb.vehicle_id} may continue."
        )
        return CoordinationResult(recs, None, False, "clear-to-proceed")

    winner, rule = _choose_winner(va, vb, assessment)
    recs = {}
    for veh in (va, vb):
        if veh.vehicle_id == winner:
            recs[veh.vehicle_id] = Recommendation(
                veh.vehicle_id,
                PROCEED,
                assessment.reason
                + f" {veh.vehicle_id} is authorized to PROCEED based on {rule}. "
                "The peer must yield.",
            )
        else:
            recs[veh.vehicle_id] = Recommendation(
                veh.vehicle_id,
                YIELD,
                assessment.reason
                + f" {veh.vehicle_id} must YIELD. Right of way goes to {winner} by {rule}.",
            )
    return CoordinationResult(recs, winner, False, rule)


def unsafe_proceed(result: CoordinationResult, truth: ConflictAssessment) -> List[str]:
    """Vehicles told to PROCEED while a true unresolved or losing-side conflict exists."""
    flagged = []
    if not truth.conflict and not result.unresolved:
        return flagged
    for vid, rec in result.recommendations.items():
        if rec.action != PROCEED:
            continue
        if result.unresolved:
            flagged.append(vid)
        elif truth.conflict and result.winner_id and vid != result.winner_id:
            flagged.append(vid)
        elif truth.conflict and result.unresolved:
            flagged.append(vid)
    return flagged
