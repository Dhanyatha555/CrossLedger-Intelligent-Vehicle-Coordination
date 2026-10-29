"""Structured JSON V2V telemetry messages."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional


@dataclass
class TelemetryPayload:
    vehicle_id: str
    seq: int
    ts: float
    x: float
    y: float
    speed_mps: float
    heading_deg: float
    intention: str
    approach: str
    priority: int
    path_remaining_m: float
    eta_s: Optional[float]
    truth_x: float
    truth_y: float

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        if data["eta_s"] is None:
            data["eta_s"] = None
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TelemetryPayload":
        return cls(
            vehicle_id=str(data["vehicle_id"]),
            seq=int(data["seq"]),
            ts=float(data["ts"]),
            x=float(data["x"]),
            y=float(data["y"]),
            speed_mps=float(data["speed_mps"]),
            heading_deg=float(data["heading_deg"]),
            intention=str(data["intention"]),
            approach=str(data["approach"]),
            priority=int(data["priority"]),
            path_remaining_m=float(data["path_remaining_m"]),
            eta_s=None if data.get("eta_s") is None else float(data["eta_s"]),
            truth_x=float(data.get("truth_x", data["x"])),
            truth_y=float(data.get("truth_y", data["y"])),
        )


@dataclass
class ChannelRecord:
    sender: str
    seq: int
    send_time: float
    deliver_at: float
    received: bool
    dropped: bool
    latency_s: Optional[float]
    reject_reason: Optional[str] = None
    tampered: bool = False
