"""
Interactive visualisation for hydrofoil simulation results.

Generates self-contained HTML files (no server needed) that open in your
Windows browser from WSL2.

Usage:
    from hydrofoil.viz import dashboard, animation_2d
    dashboard(log, title="My run")        # time-series plots → browser
    animation_2d(log)                     # animated side-view → browser
"""

from __future__ import annotations
import subprocess
import webbrowser
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

_OUT_DIR = Path(__file__).parent.parent / "data"


# ---------------------------------------------------------------------------
# HTML dashboard
# ---------------------------------------------------------------------------

def dashboard(log: dict, title: str = "Hydrofoil Simulation", z_ref: float = 0.5):
    """
    Build and open an interactive Plotly dashboard.

    Parameters
    ----------
    log    : dict returned by sim.run_simulation()
    title  : window title
    z_ref  : heave reference depth [m NED]
    """
    t = log["t"]

    fig = make_subplots(
        rows=3, cols=2,
        subplot_titles=(
            "Heave depth (NED, z↓)",  "Main foil command",
            "Pitch angle",             "Elevator command",
            "Roll angle",              "Roll differential command",
        ),
        vertical_spacing=0.10,
        horizontal_spacing=0.08,
    )

    # ── left column: states ──────────────────────────────────────────────
    fig.add_trace(go.Scatter(x=t, y=log["z"], name="depth z", line=dict(color="#1f77b4", width=2)), row=1, col=1)
    fig.add_hline(y=z_ref, line=dict(color="red", dash="dash", width=1.5), row=1, col=1)
    fig.add_annotation(x=t[-1]*0.95, y=z_ref + 0.015, text="ref", font=dict(color="red", size=11), showarrow=False, row=1, col=1)

    fig.add_trace(go.Scatter(x=t, y=log["theta"], name="pitch θ", line=dict(color="#2ca02c", width=2)), row=2, col=1)
    fig.add_hline(y=0, line=dict(color="red", dash="dash", width=1), row=2, col=1)

    fig.add_trace(go.Scatter(x=t, y=log["phi"], name="roll φ", line=dict(color="#9467bd", width=2)), row=3, col=1)
    fig.add_hline(y=0, line=dict(color="red", dash="dash", width=1), row=3, col=1)

    # ── right column: commands ───────────────────────────────────────────
    fig.add_trace(go.Scatter(x=t, y=log["d_main"], name="δ main foil", line=dict(color="#1f77b4", width=2)), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=log["d_elev"], name="δ elevator",  line=dict(color="#ff7f0e", width=2)), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=log["d_roll"], name="δ roll diff", line=dict(color="#2ca02c", width=2)), row=3, col=2)

    # ── axis labels ──────────────────────────────────────────────────────
    for row, ylabel in enumerate(["z [m]", "θ [deg]", "φ [deg]"], start=1):
        fig.update_yaxes(title_text=ylabel, row=row, col=1)
    for row in range(1, 4):
        fig.update_yaxes(title_text="δ [deg]", row=row, col=2)
    for col in range(1, 3):
        fig.update_xaxes(title_text="Time [s]", row=3, col=col)

    # ── summary stats in title ───────────────────────────────────────────
    z_final = log["z"][-1]
    pitch_max = np.max(np.abs(log["theta"]))
    cmd_max = np.max(np.abs(log["d_main"]))
    subtitle = (f"z_final={z_final:.3f} m (ref={z_ref} m)  |  "
                f"|θ|_max={pitch_max:.2f}°  |  δ_main_max={cmd_max:.2f}°")

    fig.update_layout(
        title=dict(text=f"<b>{title}</b><br><sup>{subtitle}</sup>", x=0.5),
        height=780,
        showlegend=True,
        legend=dict(orientation="h", y=-0.07),
        template="plotly_white",
        hovermode="x unified",
    )

    # ── write HTML and open ──────────────────────────────────────────────
    slug = title.lower().replace(" ", "_").replace("—", "").replace("(", "").replace(")", "").strip("_")
    out = _OUT_DIR / f"{slug}.html"
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"Dashboard saved → {out}")
    _open(out)
    return out


# ---------------------------------------------------------------------------
# Browser-based 2D side-view animation (Plotly frames, no GUI needed)
# ---------------------------------------------------------------------------

def animation_2d(
    log: dict,
    vehicle_length: float = 2.0,
    foil_span: float = 0.6,
    elev_span: float = 0.3,
    z_ref: float = 0.5,
    water_depth: float = 2.5,
    n_frames: int = 200,
    title: str = "T-Foil — Side View Animation",
):
    """
    Animated 2D side-view rendered in Plotly → opens in browser.

    Shows the hull pitching and heaving, foil flap angles responding to
    the controller, water surface, and live state readout.
    Use the play/pause button and slider to scrub through the run.

    Parameters
    ----------
    log           : dict returned by sim.run_simulation()
    vehicle_length: hull length [m], used for display scaling
    foil_span     : main foil half-span [m] (tip-to-tip / 2)
    elev_span     : elevator half-span [m]
    z_ref         : reference depth line [m NED]
    water_depth   : scene depth for y-axis [m]
    n_frames      : number of animation frames (downsampled from sim steps)
    """
    t      = log["t"]
    z      = log["z"]
    theta  = np.radians(log["theta"])
    d_main = log["d_main"]
    d_elev = log["d_elev"]

    # Downsample to n_frames
    idx    = np.linspace(0, len(t) - 1, n_frames, dtype=int)
    t_ds   = t[idx];  z_ds = z[idx];  th_ds = theta[idx]
    dm_ds  = d_main[idx];  de_ds = d_elev[idx]

    MAIN_Z = 0.5          # main foil strut length below hull CoG [m body]
    ELEV_X = -vehicle_length * 0.45
    ELEV_Z = 0.4

    def body_to_world(bx, bz, zi, thi):
        c, s = np.cos(thi), np.sin(thi)
        return bx * c - bz * s, bx * s + bz * c + zi

    def make_vehicle_traces(zi, thi, dmi, dei):
        """Return list of Plotly traces for one frame."""
        traces = []

        # Hull: four corners of a rectangle in body frame, closed polygon
        hw, hh = vehicle_length / 2, 0.14
        corners_b = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh)]
        hx = [body_to_world(bx, bz, zi, thi)[0] for bx, bz in corners_b]
        hz = [body_to_world(bx, bz, zi, thi)[1] for bx, bz in corners_b]
        traces.append(go.Scatter(
            x=hx, y=hz, fill="toself", fillcolor="#90caf9",
            line=dict(color="#1565c0", width=2),
            name="hull", showlegend=False,
        ))

        # Main foil strut
        sx, sz = zip(*[body_to_world(0, bz, zi, thi) for bz in [0, MAIN_Z]])
        traces.append(go.Scatter(x=list(sx), y=list(sz),
            line=dict(color="#546e7a", width=3), name="strut", showlegend=False))

        # Main foil bar (angled by flap)
        da = np.radians(dmi)
        mf = [body_to_world(-foil_span * np.cos(da), MAIN_Z + foil_span * np.sin(da), zi, thi),
              body_to_world( foil_span * np.cos(da), MAIN_Z - foil_span * np.sin(da), zi, thi)]
        traces.append(go.Scatter(x=[p[0] for p in mf], y=[p[1] for p in mf],
            line=dict(color="#1565c0", width=5), name="main foil", showlegend=False))

        # Elevator strut
        ex, ez = zip(*[body_to_world(ELEV_X, bz, zi, thi) for bz in [0, ELEV_Z]])
        traces.append(go.Scatter(x=list(ex), y=list(ez),
            line=dict(color="#546e7a", width=3), name="elev strut", showlegend=False))

        # Elevator bar
        dae = np.radians(dei)
        ef = [body_to_world(ELEV_X - elev_span * np.cos(dae), ELEV_Z + elev_span * np.sin(dae), zi, thi),
              body_to_world(ELEV_X + elev_span * np.cos(dae), ELEV_Z - elev_span * np.sin(dae), zi, thi)]
        traces.append(go.Scatter(x=[p[0] for p in ef], y=[p[1] for p in ef],
            line=dict(color="#e65100", width=5), name="elevator", showlegend=False))

        # CoG dot
        traces.append(go.Scatter(x=[0], y=[zi],
            mode="markers", marker=dict(color="black", size=10),
            name="CoG", showlegend=False))

        return traces

    # ── static scene elements (water surface, ref line, full depth trace) ──
    xscene = [-vehicle_length * 2, vehicle_length * 2]

    static_traces = [
        go.Scatter(x=xscene, y=[0, 0],
            line=dict(color="#4fc3f7", width=3), name="water surface"),
        go.Scatter(x=xscene, y=[z_ref, z_ref],
            line=dict(color="red", width=1.5, dash="dash"), name=f"depth ref {z_ref} m"),
        go.Scatter(x=[xscene[0], xscene[0], xscene[1], xscene[1]],
            y=[0, -0.3, -0.3, 0],
            fill="toself", fillcolor="rgba(200,230,255,0.35)",
            line=dict(width=0), name="above water", showlegend=False),
        # Depth history trace
        go.Scatter(x=np.zeros(len(t)), y=z,
            line=dict(color="rgba(100,100,200,0.3)", width=1.5),
            name="depth history", showlegend=True),
    ]

    # Initial frame vehicle traces
    init_vehicle = make_vehicle_traces(z_ds[0], th_ds[0], dm_ds[0], de_ds[0])
    all_init = static_traces + init_vehicle

    # ── build frames ──────────────────────────────────────────────────────
    frames = []
    for i in range(n_frames):
        zi, thi, dmi, dei, ti = z_ds[i], th_ds[i], dm_ds[i], de_ds[i], t_ds[i]
        vehicle_traces = make_vehicle_traces(zi, thi, dmi, dei)

        annotation_text = (
            f"<b>t = {ti:.2f} s</b><br>"
            f"depth z = {zi:.3f} m<br>"
            f"pitch θ = {np.degrees(thi):.2f}°<br>"
            f"δ main = {dmi:.2f}°<br>"
            f"δ elev = {dei:.2f}°"
        )
        frames.append(go.Frame(
            data=static_traces + vehicle_traces,
            name=str(i),
            layout=go.Layout(
                annotations=[dict(
                    x=vehicle_length * 0.85, y=-water_depth * 0.85,
                    text=annotation_text, showarrow=False,
                    align="left", bgcolor="rgba(255,255,255,0.85)",
                    bordercolor="#ccc", borderwidth=1,
                    font=dict(size=12, family="monospace"),
                )]
            ),
        ))

    # ── layout ────────────────────────────────────────────────────────────
    duration_ms = int(t[-1] * 1000 / n_frames)

    layout = go.Layout(
        title=dict(text=f"<b>{title}</b>", x=0.5),
        xaxis=dict(title="Surge x [m]", range=[-vehicle_length * 1.6, vehicle_length * 1.6],
                   scaleanchor="y", scaleratio=1, zeroline=False),
        yaxis=dict(title="Depth z [m NED]  (down = deeper)",
                   range=[-0.4, water_depth],
                   autorange=False),
        height=620,
        plot_bgcolor="#f0f8ff",
        legend=dict(x=0.01, y=0.01),
        updatemenus=[dict(
            type="buttons", showactive=False,
            y=1.08, x=0.1,
            buttons=[
                dict(label="▶  Play",
                     method="animate",
                     args=[None, dict(frame=dict(duration=duration_ms, redraw=True),
                                      fromcurrent=True, mode="immediate")]),
                dict(label="⏸  Pause",
                     method="animate",
                     args=[[None], dict(frame=dict(duration=0, redraw=False),
                                        mode="immediate")]),
            ],
        )],
        sliders=[dict(
            active=0, currentvalue=dict(prefix="Frame: "),
            pad=dict(t=50),
            steps=[dict(args=[[f.name], dict(frame=dict(duration=0, redraw=True), mode="immediate")],
                        method="animate", label=f"{t_ds[i]:.1f}s")
                   for i, f in enumerate(frames)],
        )],
    )

    fig = go.Figure(data=all_init, layout=layout, frames=frames)

    slug = title.lower().replace(" ", "_").replace("—", "").replace("(", "").replace(")", "").strip("_")
    out = _OUT_DIR / f"{slug}_animation.html"
    fig.write_html(str(out), include_plotlyjs="cdn", auto_play=False)
    print(f"Animation saved → {out}")
    _open(out)
    return out


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _open(path: Path):
    """Open a file in the system browser/viewer, WSL2-aware."""
    # Try explorer.exe — works with WSL UNC paths directly
    try:
        win_path = subprocess.run(
            ["wslpath", "-w", str(path)], capture_output=True, text=True
        ).stdout.strip()
        if win_path:
            subprocess.Popen(["explorer.exe", win_path])
            return
    except FileNotFoundError:
        pass
    # Fallback: xdg-open or Python webbrowser
    try:
        subprocess.Popen(["xdg-open", str(path)])
    except FileNotFoundError:
        webbrowser.open(path.as_uri())
