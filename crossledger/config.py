"""Shared simulation defaults for the CrossLedger proof of concept."""

from dataclasses import dataclass


@dataclass
class SimConfig:
    dt: float = 0.10
    lane_offset_m: float = 2.5
    intersection_half_m: float = 8.0
    vehicle_length_m: float = 7.0
    vehicle_width_m: float = 3.0
    path_conflict_threshold_m: float = 4.5
    safety_margin_s: float = 2.0
    stale_after_s: float = 0.80
    stopped_speed_mps: float = 0.15
    position_noise_std_m: float = 0.40
    heading_noise_std_deg: float = 1.5
    mean_latency_s: float = 0.040
    latency_jitter_s: float = 0.025
    packet_loss: float = 0.02
    hmac_key: bytes = b"crossledger-demo-key-not-for-production"
    max_speed_mps: float = 12.0
    min_speed_mps: float = 0.0
    default_start_distance_m: float = 55.0
    default_speed_mps: float = 4.5
    occupancy_padding_m: float = 6.0
    sim_duration_s: float = 25.0


DEFAULT = SimConfig()
