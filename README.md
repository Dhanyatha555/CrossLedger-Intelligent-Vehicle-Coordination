# CrossLedger

Intent-based vehicle-to-vehicle (V2V) coordination for **blind off-highway crossings**.

Two simulated construction vehicles approach a shared intersection, exchange JSON telemetry (intent, pose, speed, priority), and receive consistent **PROCEED / YIELD / HOLD** recommendations. The radio is an **in-process simulated V2V layer** with delay, packet loss, stale-data handling, and HMAC-SHA256 authentication.

This is an academic proof of concept. It does **not** guarantee real-world vehicle safety. Latency and delivery figures are **simulated**, not measured on a physical radio.

## What it demonstrates

- Blind construction-site crossing with two off-highway vehicles (excavator EX-101, dump truck DT-204).
- Intent-aware conflict prediction (paths must actually intersect; proximity alone is not a conflict).
- Deterministic right-of-way: safety first, then priority, then earlier arrival, then vehicle ID.
- Conservative HOLD when telemetry is missing, stale, tampered, or replayed.
- Dashboard with a top-down map, telemetry, communication health, event log, and evaluation metrics.

## Requirements

- Windows 10/11 laptop
- Python 3.10 or later

## Setup

```powershell
cd "d:\DHANYATHA\Projects\ADAS PS4"
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run the dashboard

```powershell
streamlit run app.py
```

Then open the local URL Streamlit prints (typically http://localhost:8501).

Use **Play** to run the simulation, or the scenario buttons:

- Normal crossing
- Simultaneous arrival
- Sudden stop
- Communication failure
- Tampered message

Sidebar controls change speed, start distance, intention, approach, priority, safety margin, simulated loss, and simulated latency.

## Run the evaluation suite

```powershell
python evaluate.py
```

This executes reproducible scenarios, prints a pass/fail summary, runs unit tests, and writes:

`results/evaluation.csv`

Metrics include messages sent/received, delivery success rate, simulated average/max latency, position RMSE against known ground truth, conflict-detection rate on fresh frames, unsafe-proceed count, alert response time, and HMAC/replay rejections.

You can also press **Run evaluation suite** on the dashboard.

## Project layout

| Path | Role |
| --- | --- |
| `app.py` | Streamlit dashboard |
| `evaluate.py` | CLI evaluation + unit tests |
| `crossledger/vehicles.py` | Vehicle kinematics |
| `crossledger/geometry.py` | Intent paths at the crossing |
| `crossledger/messages.py` | JSON telemetry schema |
| `crossledger/channel.py` | Simulated V2V delay/loss |
| `crossledger/security.py` | HMAC-SHA256 + replay window |
| `crossledger/conflict.py` | Geometric + temporal conflict |
| `crossledger/coordination.py` | PROCEED / YIELD / HOLD |
| `crossledger/simulation.py` | Time-stepped world |
| `crossledger/evaluation.py` | Scenarios and CSV export |

## Coordination rules

1. If a peer frame is missing, stale, or fails authentication → both **HOLD**.
2. If intended paths do not come within the conflict distance → both **PROCEED**.
3. If paths could cross but arrival windows (plus safety margin) do not overlap → both **PROCEED**.
4. If a stopped vehicle occupies the crossing → both **HOLD**.
5. Otherwise one vehicle **PROCEED**s and the other **YIELD**s: higher priority, then earlier ETA, then lower vehicle ID.
6. The engine never authorizes **PROCEED** into an unresolved conflict.

Stopped vehicles use a speed threshold; ETA is not computed by dividing by zero.

## Security (prototype only)

Messages are canonical JSON plus **HMAC-SHA256**. Invalid tags and repeated sequence numbers are rejected and shown in the event log. Shared-key HMAC is **not** production vehicle cybersecurity.

## License / academic use

Built as a time-limited course proof of concept for Problem Statement 4 (V2V connectivity for off-highway vehicles).
