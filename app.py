"""CrossLedger Streamlit dashboard."""

from __future__ import annotations

import time

import plotly.graph_objects as go
import streamlit as st

from crossledger.config import SimConfig
from crossledger.coordination import HOLD, PROCEED, YIELD
from crossledger.evaluation import run_all, summary_text, write_csv
from crossledger.geometry import format_heading
from crossledger.simulation import Simulation

st.set_page_config(
    page_title="CrossLedger — V2V Blind Crossing",
    page_icon="🚧",
    layout="wide",
)

ACTION_COLOR = {PROCEED: "#1e8449", YIELD: "#d35400", HOLD: "#c0392b"}


def get_sim() -> Simulation:
    if "sim" not in st.session_state:
        st.session_state.sim = Simulation(SimConfig(), seed=7)
        st.session_state.playing = False
        st.session_state.eval_rows = None
        st.session_state.eval_summary = None
    return st.session_state.sim


def apply_sidebar(sim: Simulation) -> None:
    v0, v1 = sim.vehicles
    st.sidebar.header("Vehicle 1 — EX-101")
    s0 = st.sidebar.slider("EX-101 speed (m/s)", 0.0, 12.0, float(v0.speed_mps), 0.1, key="spd0")
    d0 = st.sidebar.slider("EX-101 start distance (m)", 15.0, 90.0, float(v0.start_distance_m), 1.0, key="dst0")
    i0 = st.sidebar.selectbox("EX-101 intention", ["straight", "left", "right"], index=["straight", "left", "right"].index(v0.intention), key="int0")
    a0 = st.sidebar.selectbox("EX-101 approach", ["south", "north", "east", "west"], index=["south", "north", "east", "west"].index(v0.approach), key="ap0")
    p0 = st.sidebar.selectbox("EX-101 priority (higher wins)", [1, 2, 3], index=v0.priority - 1, key="pr0")

    st.sidebar.header("Vehicle 2 — DT-204")
    s1 = st.sidebar.slider("DT-204 speed (m/s)", 0.0, 12.0, float(v1.speed_mps), 0.1, key="spd1")
    d1 = st.sidebar.slider("DT-204 start distance (m)", 15.0, 90.0, float(v1.start_distance_m), 1.0, key="dst1")
    i1 = st.sidebar.selectbox("DT-204 intention", ["straight", "left", "right"], index=["straight", "left", "right"].index(v1.intention), key="int1")
    a1 = st.sidebar.selectbox("DT-204 approach", ["west", "east", "south", "north"], index=["west", "east", "south", "north"].index(v1.approach), key="ap1")
    p1 = st.sidebar.selectbox("DT-204 priority (higher wins)", [1, 2, 3], index=v1.priority - 1, key="pr1")

    st.sidebar.header("Safety & simulated radio")
    margin = st.sidebar.slider("Safety margin (s)", 0.5, 5.0, float(sim.cfg.safety_margin_s), 0.1, key="margin")
    loss = st.sidebar.slider("Packet loss", 0.0, 1.0, float(sim.cfg.packet_loss), 0.01, key="loss")
    lat = st.sidebar.slider("Mean latency (s, simulated)", 0.0, 2.0, float(sim.cfg.mean_latency_s), 0.01, key="lat")
    jit = st.sidebar.slider("Latency jitter (s, simulated)", 0.0, 0.5, float(sim.cfg.latency_jitter_s), 0.005, key="jit")
    stale = st.sidebar.slider("Stale after (s)", 0.2, 3.0, float(sim.cfg.stale_after_s), 0.05, key="stale")

    sim.cfg.safety_margin_s = margin
    sim.cfg.packet_loss = loss
    sim.cfg.mean_latency_s = lat
    sim.cfg.latency_jitter_s = jit
    sim.cfg.stale_after_s = stale
    sim.apply_vehicle_settings(0, speed=s0, start_distance=d0, intention=i0, approach=a0, priority=p0)
    sim.apply_vehicle_settings(1, speed=s1, start_distance=d1, intention=i1, approach=a1, priority=p1)


def load_preset(name: str) -> None:
    sim = Simulation(SimConfig(), seed=7)
    st.session_state.playing = False
    if name == "normal":
        sim.apply_vehicle_settings(0, speed=6.0, start_distance=32.0, intention="straight", approach="south", priority=2, reset_pose=True)
        sim.apply_vehicle_settings(1, speed=3.5, start_distance=70.0, intention="straight", approach="west", priority=2, reset_pose=True)
        sim.cfg.packet_loss = 0.02
        sim.log("scenario", "Preset: normal crossing with separated arrival times.")
    elif name == "simul":
        sim.apply_vehicle_settings(0, speed=5.0, start_distance=48.0, intention="straight", approach="south", priority=2, reset_pose=True)
        sim.apply_vehicle_settings(1, speed=5.0, start_distance=48.0, intention="straight", approach="west", priority=2, reset_pose=True)
        sim.log("scenario", "Preset: simultaneous conflicting arrivals.")
    elif name == "stop":
        sim.apply_vehicle_settings(0, speed=5.5, start_distance=40.0, intention="straight", approach="south", priority=2, reset_pose=True)
        sim.apply_vehicle_settings(1, speed=5.5, start_distance=18.0, intention="straight", approach="west", priority=2, reset_pose=True)
        sim.log("scenario", "Preset: sudden stop — DT-204 will halt near the box after a short run.")
        sim._pending_stop = True
    elif name == "fail":
        sim.apply_vehicle_settings(0, speed=5.0, start_distance=45.0, intention="straight", approach="south", reset_pose=True)
        sim.apply_vehicle_settings(1, speed=5.0, start_distance=45.0, intention="straight", approach="west", reset_pose=True)
        sim.cfg.packet_loss = 1.0
        sim.log("scenario", "Preset: communication failure (100% simulated packet loss).")
    elif name == "tamper":
        sim.apply_vehicle_settings(0, speed=5.0, start_distance=45.0, intention="straight", approach="south", reset_pose=True)
        sim.apply_vehicle_settings(1, speed=5.0, start_distance=45.0, intention="straight", approach="west", reset_pose=True)
        sim.cfg.packet_loss = 0.0
        sim.inject_tamper(sim.vehicles[0].vehicle_id)
        sim.log("scenario", "Preset: next EX-101 frame is tampered after HMAC signing.")
    st.session_state.sim = sim
    # Sync widgets
    v0, v1 = sim.vehicles
    st.session_state.spd0, st.session_state.spd1 = v0.speed_mps, v1.speed_mps
    st.session_state.dst0, st.session_state.dst1 = v0.start_distance_m, v1.start_distance_m
    st.session_state.int0, st.session_state.int1 = v0.intention, v1.intention
    st.session_state.ap0, st.session_state.ap1 = v0.approach, v1.approach
    st.session_state.pr0, st.session_state.pr1 = v0.priority, v1.priority
    st.session_state.loss = sim.cfg.packet_loss
    st.session_state.lat = sim.cfg.mean_latency_s
    st.session_state.margin = sim.cfg.safety_margin_s


def draw_map(sim: Simulation) -> go.Figure:
    fig = go.Figure()
    half = 80
    lane = sim.cfg.lane_offset_m
    box = sim.cfg.intersection_half_m
    fig.add_shape(type="rect", x0=-half, y0=-12, x1=half, y1=12, fillcolor="#3d3d3d", line_width=0)
    fig.add_shape(type="rect", x0=-12, y0=-half, x1=12, y1=half, fillcolor="#3d3d3d", line_width=0)
    fig.add_shape(type="rect", x0=-box, y0=-box, x1=box, y1=box, fillcolor="#5d4e37", line=dict(color="#c9a227", width=2))
    for y in (-lane, lane):
        fig.add_trace(go.Scatter(x=[-half, half], y=[y, y], mode="lines", line=dict(color="#f1c40f", width=1, dash="dash"), showlegend=False, hoverinfo="skip"))
    for x in (-lane, lane):
        fig.add_trace(go.Scatter(x=[x, x], y=[-half, half], mode="lines", line=dict(color="#f1c40f", width=1, dash="dash"), showlegend=False, hoverinfo="skip"))

    tick = sim.last_tick
    if tick and tick.truth.conflict_point_a and tick.truth.geometric:
        fig.add_trace(
            go.Scatter(
                x=[tick.truth.conflict_point_a[0], tick.truth.conflict_point_b[0]],
                y=[tick.truth.conflict_point_a[1], tick.truth.conflict_point_b[1]],
                mode="markers",
                marker=dict(size=10, color="#e74c3c", symbol="x"),
                name="Closest approach",
            )
        )

    for v in sim.vehicles:
        path = v.path.waypoints
        fig.add_trace(
            go.Scatter(
                x=[p[0] for p in path],
                y=[p[1] for p in path],
                mode="lines",
                line=dict(color=v.color, width=2, dash="dot"),
                name=f"{v.vehicle_id} intent",
            )
        )
        x, y = v.position
        fig.add_trace(
            go.Scatter(
                x=[x],
                y=[y],
                mode="markers+text",
                marker=dict(size=18, color=v.color, symbol="arrow-right", angle=v.heading_deg),
                text=[v.vehicle_id],
                textposition="top center",
                name=v.vehicle_id,
            )
        )
    fig.update_layout(
        template="plotly_dark",
        height=520,
        margin=dict(l=10, r=10, t=30, b=10),
        xaxis=dict(range=[-70, 70], scaleanchor="y", scaleratio=1, title="East (m)"),
        yaxis=dict(range=[-70, 70], title="North (m)"),
        legend=dict(orientation="h", y=1.08),
        title="Construction-site crossing (top-down, simulated)",
    )
    return fig


def rec_card(rec, vehicle) -> None:
    color = ACTION_COLOR.get(rec.action, "#7f8c8d")
    st.markdown(
        f"""
        <div style="border:2px solid {color};border-radius:12px;padding:14px 16px;margin-bottom:10px;background:#111;">
          <div style="font-size:13px;color:#bbb;">{vehicle.label}</div>
          <div style="font-size:28px;font-weight:700;color:{color};letter-spacing:1px;">{rec.action}</div>
          <div style="font-size:14px;color:#eee;margin-top:6px;">{rec.explanation}</div>
        </div>
        """,
        unsafe_allow_html=True,
    )


def main() -> None:
    sim = get_sim()

    st.title("CrossLedger")
    st.caption(
        "Intent-based vehicle-to-vehicle coordination for blind off-highway crossings. "
        "Two construction vehicles share telemetry and movement intentions. "
        "The coordinator predicts path conflicts and issues PROCEED / YIELD / HOLD. "
        "The radio layer is simulated in-process (delay, loss, HMAC-SHA256). "
        "This prototype does not guarantee real-world vehicle safety."
    )

    c1, c2, c3, c4, c5 = st.columns(5)
    if c1.button("Normal crossing", use_container_width=True):
        load_preset("normal")
        st.rerun()
    if c2.button("Simultaneous arrival", use_container_width=True):
        load_preset("simul")
        st.rerun()
    if c3.button("Sudden stop", use_container_width=True):
        load_preset("stop")
        st.rerun()
    if c4.button("Communication failure", use_container_width=True):
        load_preset("fail")
        st.rerun()
    if c5.button("Tampered message", use_container_width=True):
        load_preset("tamper")
        st.rerun()

    apply_sidebar(sim)

    b1, b2, b3, b4, b5 = st.columns([1, 1, 1, 1, 2])
    if b1.button("Play", type="primary", use_container_width=True):
        st.session_state.playing = True
    if b2.button("Pause", use_container_width=True):
        st.session_state.playing = False
    if b3.button("Step", use_container_width=True):
        st.session_state.playing = False
        sim.step()
        if getattr(sim, "_pending_stop", False) and sim.t >= 1.6:
            sim.sudden_stop(1)
            sim._pending_stop = False
    if b4.button("Reset pose", use_container_width=True):
        st.session_state.playing = False
        for i in range(2):
            sim.apply_vehicle_settings(i, reset_pose=True)
        sim.t = 0.0
        sim.history.clear()
        sim.events.clear()
        sim.channel.reset_stats()
        sim._last_good = {}
        sim.last_tick = None
        sim.last_coordination = None
        sim.log("system", "Pose reset. Simulated V2V counters cleared.")
    b5.metric("Simulation time", f"{sim.t:.1f} s")

    if st.session_state.playing:
        tick = sim.step()
        if getattr(sim, "_pending_stop", False) and sim.t >= 1.6:
            sim.sudden_stop(1)
            sim._pending_stop = False
        _ = tick

    if sim.last_tick is None:
        sim.step()

    left, right = st.columns([1.15, 0.85])
    with left:
        st.plotly_chart(draw_map(sim), use_container_width=True)

    with right:
        tick = sim.last_tick
        st.subheader("Live crossing recommendations")
        if tick:
            for v in sim.vehicles:
                rec_card(tick.coordination.recommendations[v.vehicle_id], v)

        age = tick.peer_age_s if tick else None
        if tick and tick.comms_ok:
            health, hcolor = "Healthy (fresh authenticated telemetry)", "#1e8449"
        elif age is None:
            health, hcolor = "No peer frame yet — HOLD", "#c0392b"
        else:
            health, hcolor = f"Degraded / stale ({age:.2f} s since last good RX)", "#d35400"
        st.markdown(
            f"<div style='padding:10px 12px;border-radius:8px;background:{hcolor};color:white;font-weight:600;'>"
            f"Communication health: {health}</div>",
            unsafe_allow_html=True,
        )
        st.caption("Delivery statistics use the simulated in-process channel, not a physical radio.")

    st.subheader("Vehicle telemetry")
    cols = st.columns(2)
    for col, v in zip(cols, sim.vehicles):
        snap = v.snapshot(sim.cfg)
        eta = snap["eta_s"]
        with col:
            st.markdown(f"**{v.label}**")
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("Speed", f"{v.speed_mps:.1f} m/s")
            m2.metric("Distance to box", f"{snap['distance_to_intersection_m']:.1f} m")
            m3.metric("ETA", "stopped" if eta is None else f"{eta:.1f} s")
            m4.metric("Priority", str(v.priority))
            st.write(
                f"ID `{v.vehicle_id}` · pos ({snap['x']:.1f}, {snap['y']:.1f}) m · "
                f"heading {format_heading(v.heading_deg)} · intent **{v.intention}** from **{v.approach}**"
            )

    st.subheader("Simulated V2V performance")
    ch = sim.channel
    avg, mx = ch.latency_stats()
    k1, k2, k3, k4, k5 = st.columns(5)
    k1.metric("Messages sent", ch.sent)
    k2.metric("Authenticated RX", ch.delivered)
    k3.metric("Delivery success", f"{ch.delivery_success_rate():.0%}")
    k4.metric("Avg latency (sim.)", "—" if avg is None else f"{avg*1000:.1f} ms")
    k5.metric("Max latency (sim.)", "—" if mx is None else f"{mx*1000:.1f} ms")
    s1, s2, s3 = st.columns(3)
    s1.metric("HMAC rejected", sim.auth.rejected_mac)
    s2.metric("Replay rejected", sim.auth.rejected_replay)
    rmse = None
    if sim.position_errors:
        rmse = (sum(e * e for e in sim.position_errors) / len(sim.position_errors)) ** 0.5
    s3.metric("Position RMSE vs truth", "—" if rmse is None else f"{rmse:.2f} m")

    if tick:
        st.info(tick.local.reason if tick.comms_ok else tick.coordination.recommendations[sim.vehicles[0].vehicle_id].explanation)

    st.subheader("Event log")
    rows = list(reversed(sim.events[-40:]))
    if rows:
        st.dataframe(
            {
                "t_s": [f"{e.t:.2f}" for e in rows],
                "kind": [e.kind for e in rows],
                "event": [e.text for e in rows],
            },
            use_container_width=True,
            hide_index=True,
            height=280,
        )

    st.subheader("Evaluation")
    st.caption(
        "Automated scenarios write measured (simulated-channel) metrics to CSV. "
        "Values are computed from the run, not hardcoded."
    )
    if st.button("Run evaluation suite"):
        with st.spinner("Running reproducible scenarios..."):
            results = run_all()
            path = write_csv(results, "results/evaluation.csv")
            st.session_state.eval_summary = summary_text(results)
            st.session_state.eval_rows = results
            st.success(f"Wrote {path}")
    if st.session_state.eval_summary:
        st.text(st.session_state.eval_summary)
    if st.session_state.eval_rows:
        import pandas as pd
        from crossledger.evaluation import results_to_rows

        st.dataframe(pd.DataFrame(results_to_rows(st.session_state.eval_rows)), use_container_width=True)

    st.markdown("---")
    st.markdown(
        "**Security note.** HMAC-SHA256 with a shared demo key authenticates JSON telemetry and "
        "rejects replays. This is a teaching prototype, not production-grade vehicle cybersecurity. "
        "**Safety note.** Recommendations are explainable simulation outputs. They do not certify "
        "off-highway vehicles for field use."
    )

    if st.session_state.playing:
        time.sleep(max(sim.cfg.dt * 0.35, 0.03))
        st.rerun()


if __name__ == "__main__":
    main()
