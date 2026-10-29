"""Simulated V2V messaging layer with delay, loss, and delivery accounting.

This is not a radio stack. Latency and loss are injected in-process so the
dashboard can demonstrate degraded links without a network broker.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .messages import ChannelRecord, TelemetryPayload
from .security import MessageAuthenticator, sign_payload


@dataclass
class InFlight:
    envelope: dict
    record: ChannelRecord


@dataclass
class Delivery:
    payload: TelemetryPayload
    envelope: dict
    send_time: float
    recv_time: float
    latency_s: float
    tampered: bool
    rejected: bool
    reject_reason: Optional[str]


class SimulatedV2V:
    """In-process store-and-forward channel with HMAC verification on receive."""

    def __init__(self, authenticator: MessageAuthenticator, rng: Optional[random.Random] = None):
        self.auth = authenticator
        self.rng = rng or random.Random(7)
        self.queue: List[InFlight] = []
        self.records: List[ChannelRecord] = []
        self.sent = 0
        self.delivered = 0
        self.dropped = 0
        self.tamper_next_from: Optional[str] = None
        self.force_replay_from: Optional[str] = None
        self._last_envelope: Dict[str, dict] = {}

    def reset_stats(self) -> None:
        self.queue.clear()
        self.records.clear()
        self.sent = 0
        self.delivered = 0
        self.dropped = 0
        self.tamper_next_from = None
        self.force_replay_from = None
        self._last_envelope.clear()
        self.auth.reset_counters()

    def sample_latency(self, mean_s: float, jitter_s: float) -> float:
        lat = self.rng.gauss(mean_s, jitter_s)
        return max(0.001, min(lat, mean_s + 8 * max(jitter_s, 0.001)))

    def transmit(
        self,
        payload: TelemetryPayload,
        now: float,
        mean_latency_s: float,
        latency_jitter_s: float,
        packet_loss: float,
    ) -> ChannelRecord:
        envelope = self.auth.wrap(payload.to_dict())
        tampered = False
        if self.tamper_next_from == payload.vehicle_id:
            # Change telemetry after signing so the MAC no longer matches.
            envelope["payload"] = dict(envelope["payload"])
            envelope["payload"]["speed_mps"] = float(envelope["payload"]["speed_mps"]) + 9.0
            envelope["payload"]["intention"] = "left"
            tampered = True
            self.tamper_next_from = None
        elif self.force_replay_from == payload.vehicle_id and payload.vehicle_id in self._last_envelope:
            envelope = self._last_envelope[payload.vehicle_id]
            tampered = False
            self.force_replay_from = None

        self._last_envelope[payload.vehicle_id] = {
            "payload": dict(envelope["payload"]),
            "mac": envelope["mac"],
        }

        self.sent += 1
        drop = self.rng.random() < max(0.0, min(packet_loss, 1.0))
        latency = self.sample_latency(mean_latency_s, latency_jitter_s)
        rec = ChannelRecord(
            sender=payload.vehicle_id,
            seq=payload.seq,
            send_time=now,
            deliver_at=now + latency,
            received=False,
            dropped=drop,
            latency_s=None if drop else latency,
            tampered=tampered,
        )
        self.records.append(rec)
        if drop:
            self.dropped += 1
        else:
            self.queue.append(InFlight(envelope=envelope, record=rec))
        return rec

    def receive_due(self, now: float) -> List[Delivery]:
        due: List[InFlight] = []
        keep: List[InFlight] = []
        for item in self.queue:
            if item.record.deliver_at <= now:
                due.append(item)
            else:
                keep.append(item)
        self.queue = keep
        out: List[Delivery] = []
        for item in due:
            result = self.auth.authenticate(item.envelope)
            payload_dict = item.envelope.get("payload") or {}
            try:
                payload = TelemetryPayload.from_dict(payload_dict)
            except (KeyError, TypeError, ValueError):
                payload = None
            item.record.received = result.accepted
            item.record.reject_reason = None if result.accepted else result.reason
            if result.accepted:
                self.delivered += 1
                item.record.latency_s = now - item.record.send_time
            if payload is None:
                continue
            out.append(
                Delivery(
                    payload=payload,
                    envelope=item.envelope,
                    send_time=item.record.send_time,
                    recv_time=now,
                    latency_s=now - item.record.send_time,
                    tampered=item.record.tampered,
                    rejected=not result.accepted,
                    reject_reason=result.reason if not result.accepted else None,
                )
            )
        return out

    def delivery_success_rate(self) -> float:
        if self.sent == 0:
            return 0.0
        return self.delivered / self.sent

    def latency_stats(self) -> Tuple[Optional[float], Optional[float]]:
        vals = [r.latency_s for r in self.records if r.received and r.latency_s is not None]
        if not vals:
            return None, None
        return sum(vals) / len(vals), max(vals)


def tamper_copy(envelope: dict, key: bytes, field: str, value) -> dict:
    """Utility used by tests to produce a forged message with an outdated MAC."""
    cloned = {"payload": dict(envelope["payload"]), "mac": envelope["mac"]}
    cloned["payload"][field] = value
    # Keep the old MAC unless the caller re-signs.
    _ = key
    _ = sign_payload
    return cloned
