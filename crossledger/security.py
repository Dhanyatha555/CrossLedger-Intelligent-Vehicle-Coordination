"""Prototype HMAC-SHA256 authentication and replay protection.

This is a demonstration mechanism for an academic proof of concept.
It is not production-grade vehicle cybersecurity.
"""

from __future__ import annotations

import hashlib
import hmac
import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set


def canonical_json(payload: Dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def sign_payload(payload: Dict[str, Any], key: bytes) -> str:
    return hmac.new(key, canonical_json(payload), hashlib.sha256).hexdigest()


def verify_mac(payload: Dict[str, Any], mac: str, key: bytes) -> bool:
    expected = sign_payload(payload, key)
    if not isinstance(mac, str):
        return False
    return hmac.compare_digest(expected, mac)


@dataclass
class AuthResult:
    accepted: bool
    reason: str
    replay: bool = False
    invalid_mac: bool = False


@dataclass
class ReplayWindow:
    last_seq: Dict[str, int] = field(default_factory=dict)
    seen: Dict[str, Set[int]] = field(default_factory=dict)

    def check_and_record(self, vehicle_id: str, seq: int) -> Optional[str]:
        seen = self.seen.setdefault(vehicle_id, set())
        if seq in seen:
            return f"replayed sequence number {seq} from {vehicle_id}"
        last = self.last_seq.get(vehicle_id)
        if last is not None and seq <= last:
            return f"non-increasing sequence {seq} (last accepted {last}) from {vehicle_id}"
        seen.add(seq)
        self.last_seq[vehicle_id] = seq
        return None

    def reset(self) -> None:
        self.last_seq.clear()
        self.seen.clear()


class MessageAuthenticator:
    def __init__(self, key: bytes):
        self.key = key
        self.replay = ReplayWindow()
        self.accepted = 0
        self.rejected_mac = 0
        self.rejected_replay = 0

    def wrap(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return {"payload": payload, "mac": sign_payload(payload, self.key)}

    def authenticate(self, message: Dict[str, Any], record_replay: bool = True) -> AuthResult:
        payload = message.get("payload")
        mac = message.get("mac")
        if not isinstance(payload, dict):
            self.rejected_mac += 1
            return AuthResult(False, "malformed message: missing payload", invalid_mac=True)
        if not verify_mac(payload, mac, self.key):
            self.rejected_mac += 1
            return AuthResult(False, "invalid HMAC-SHA256 authentication tag", invalid_mac=True)
        vehicle_id = str(payload.get("vehicle_id", ""))
        try:
            seq = int(payload.get("seq"))
        except (TypeError, ValueError):
            self.rejected_mac += 1
            return AuthResult(False, "malformed message: invalid sequence number", invalid_mac=True)
        if record_replay:
            replay_reason = self.replay.check_and_record(vehicle_id, seq)
            if replay_reason:
                self.rejected_replay += 1
                return AuthResult(False, replay_reason, replay=True)
        self.accepted += 1
        return AuthResult(True, "authenticated")

    def reset_counters(self) -> None:
        self.accepted = 0
        self.rejected_mac = 0
        self.rejected_replay = 0
        self.replay.reset()
