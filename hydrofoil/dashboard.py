"""
Interactive browser dashboard for the hydrofoil simulation.

Run:
    python -m hydrofoil.dashboard
    # Then open http://localhost:8000 in your Windows browser

Controls all hardcoded parameters from sim.py via sliders.
Streams live state via WebSocket and renders attitude/depth visuals.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from typing import Any

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from hydrofoil.waves.airy import CalmWater, AiryWave
from hydrofoil.vehicle import VehiclePhysics

app = FastAPI()

DEFAULT_PARAMS: dict[str, Any] = {
    "mass": 25.0,
    "ixx": 2.0, "iyy": 5.0, "izz": 5.0,
    "heave_damping": 50.0,
    "main_chord": 0.15, "main_span": 0.6,
    "elev_chord": 0.10, "elev_span": 0.3,
    "main_pos_x": 0.0,  "main_pos_z": 0.5, "main_lateral_offset": 0.35,
    "elev_pos_x": -1.2, "elev_pos_z": 0.4,
    # Heave gains are tuned higher than sim.py's — here the vehicle starts at rest
    # and must generate real lift to take off, not just trim near an already-flying point.
    "kp_heave": 15.0, "ki_heave": 1.0, "kd_heave": 5.0,
    "kp_pitch": 2.0, "ki_pitch": 0.0, "kd_pitch": 2.0,
    # Ki must stay 0 here — any nonzero integral term reliably diverges under
    # wave disturbance (verified: Ki=0.05 blew up to 1000+ deg within 90s).
    "kp_roll":  3.5, "ki_roll":  0.0, "kd_roll":  2.5,
    # Target height of the HULL above the water once foilborne [m] (negative =
    # hull still below the surface). Internally converted to a CoG depth
    # reference via hull_offset — this is the number you actually care about.
    # Default = half the strut length (hull_offset + main_pos_z) / 2, so the
    # main strut's midpoint sits right at the waterline once flying — half
    # the shaft above water, half below.
    "flying_height": 0.55,
    "phi_ref": 0.0, "theta_ref": 0.0,
    "waves_enabled": False,
    "wave_amplitude": 0.05, "wave_wavelength": 5.0, "wave_period": 2.0,
    "dt": 0.01, "t_end": 60.0,

    # Velocity (surge) controller: closed-loop speed control, thrust in Newtons.
    "target_speed": 4.0,
    "kp_speed": 30.0, "ki_speed": 5.0, "kd_speed": 0.0,
    "max_thrust": 50.0,   # matches the Unity TwinMotorController placeholder (50N)

    # Hull buoyancy / drag — a simplified wall-sided hull near the waterline.
    # hull_offset is how far above the CoG the hull rides (roughly the strut
    # length); waterplane_area/max_draft set how buoyant it is at rest and how
    # much draft it has before the linear approximation caps out.
    "waterplane_area": 0.6, "max_draft": 0.3, "hull_offset": 0.6,
    "hull_drag_coeff": 20.0,
}


class SimRunner:
    def __init__(self):
        self.params = DEFAULT_PARAMS.copy()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._state: dict = {"running": False, "t": 0.0, "z": 0.0,
                             "phi": 0.0, "theta": 0.0, "psi": 0.0,
                             "u": 0.0, "d_main": 0.0, "d_elev": 0.0, "d_roll": 0.0,
                             "draft": 0.0, "thrust": 0.0, "foilborne": False,
                             "wave_elev": 0.0, "wave_phase": 0.0}

    def get_state(self) -> dict:
        with self._lock:
            return self._state.copy()

    def set_state(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.set_state(running=False)

    def restart_with(self, params: dict):
        self.stop()
        self.params.update(params)
        self.start()

    def set_live_params(self, updates: dict):
        """
        Update setpoints on the RUNNING sim without stopping/resetting it —
        this is the "stick input" path (height, bank angle) versus the
        stop/restart path used for gains and vehicle config.
        """
        with self._lock:
            self.params.update(updates)

    def _run(self):
        p = self.params
        dt = p["dt"]

        vp = VehiclePhysics(p)

        wave = (AiryWave(amplitude=p["wave_amplitude"],
                         wavelength=p["wave_wavelength"],
                         period=p["wave_period"])
                if p["waves_enabled"] else CalmWater())

        # Start at rest, floating at the hull's buoyant equilibrium draft —
        # not already at cruise speed — so the takeoff transition is visible.
        state = vp.rest_state()
        t = 0.0

        self.set_state(running=True)

        real_start = time.time()

        while not self._stop_event.is_set() and t <= p["t_end"]:
            eta, nu = state[:6], state[6:]
            state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
            t += dt

            self.set_state(
                t=round(t, 3),
                z=round(float(eta[2]), 4),
                phi=round(float(np.degrees(eta[3])), 3),
                theta=round(float(np.degrees(eta[4])), 3),
                psi=round(float(np.degrees(eta[5])), 3),
                u=round(float(nu[0]), 3),
                d_main=round(float(cmds["delta_main"]), 3),
                d_elev=round(float(cmds["delta_elev"]), 3),
                d_roll=round(float(cmds["delta_roll"]), 3),
                draft=round(draft, 4),
                thrust=round(float(thrust), 2),
                foilborne=draft <= 1e-6,
                phi_ref_live=round(float(p["phi_ref"]), 3),
                wave_elev=round(float(wave_elev), 4),
                wave_phase=round(float(wave.k * (eta[0]*np.cos(wave.direction) + eta[1]*np.sin(wave.direction))
                                        - wave.omega * t + wave.phase), 5),
                running=True,
            )

            # Real-time pacing: sleep to match wall clock
            elapsed = time.time() - real_start
            if t > elapsed:
                time.sleep(t - elapsed)

        self.set_state(running=False)


runner = SimRunner()
clients: list[WebSocket] = []


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(content=DASHBOARD_HTML)


@app.get("/params")
async def get_params():
    return runner.params


@app.post("/params")
async def set_params(body: dict):
    runner.restart_with(body)
    return {"ok": True}


@app.post("/manual")
async def manual_control(body: dict):
    """Live stick input (flying_height, phi_ref, ...) — no restart, no reset."""
    runner.set_live_params(body)
    return {"ok": True}


@app.post("/run")
async def run_sim():
    runner.start()
    return {"ok": True}


@app.post("/stop")
async def stop_sim():
    runner.stop()
    return {"ok": True}


@app.post("/reset")
async def reset_sim():
    runner.stop()
    runner.params = DEFAULT_PARAMS.copy()
    return {"ok": True, "params": runner.params}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    try:
        while True:
            state = runner.get_state()
            await ws.send_text(json.dumps(state))
            await asyncio.sleep(1 / 30)
    except (WebSocketDisconnect, Exception):
        clients.remove(ws)


# ---------------------------------------------------------------------------
# HTML / JS dashboard (self-contained, no build step)
# ---------------------------------------------------------------------------

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Hydrofoil Sim Dashboard</title>
<script src="https://cdn.plot.ly/plotly-2.32.0.min.js"></script>
<style>
  :root {
    --bg: #0d1117; --panel: #161b22; --border: #30363d;
    --accent: #58a6ff; --green: #3fb950; --red: #f85149;
    --yellow: #d29922; --text: #e6edf3; --muted: #8b949e;
    --font: 'Segoe UI', system-ui, sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: var(--font); font-size: 13px; height: 100vh; display: flex; flex-direction: column; overflow: hidden; }

  header { background: var(--panel); border-bottom: 1px solid var(--border); padding: 10px 18px; display: flex; align-items: center; gap: 16px; flex-shrink: 0; }
  header h1 { font-size: 15px; font-weight: 600; color: var(--accent); letter-spacing: .5px; }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--red); transition: background .3s; }
  .status-dot.running { background: var(--green); }
  .btn { padding: 5px 14px; border-radius: 6px; border: 1px solid var(--border); background: var(--panel); color: var(--text); cursor: pointer; font-size: 12px; transition: background .15s; }
  .btn:hover { background: #21262d; }
  .btn.primary { background: var(--accent); color: #000; border-color: var(--accent); }
  .btn.danger  { border-color: var(--red); color: var(--red); }

  .layout { display: grid; grid-template-columns: 300px 1fr; flex: 1; overflow: hidden; }

  /* ── Left: tabbed params panel ── */
  #params-panel { background: var(--panel); border-right: 1px solid var(--border); display: flex; flex-direction: column; overflow: hidden; }

  #tab-bar { display: flex; border-bottom: 1px solid var(--border); flex-shrink: 0; }
  .tab-btn { flex: 1; padding: 9px 6px; background: transparent; border: none; border-bottom: 2px solid transparent; color: var(--muted); cursor: pointer; font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: .8px; transition: color .15s, border-color .15s; }
  .tab-btn.active { color: var(--accent); border-bottom-color: var(--accent); }
  .tab-btn:hover:not(.active) { color: var(--text); }

  .tab-pane { display: none; flex-direction: column; flex: 1; overflow: hidden; }
  .tab-pane.active { display: flex; }
  .groups-wrap { flex: 1; overflow-y: auto; padding: 6px 0; }
  .apply-wrap { padding: 8px 14px; border-top: 1px solid var(--border); flex-shrink: 0; }

  .group { border-bottom: 1px solid var(--border); }
  .group-header { padding: 8px 14px; font-weight: 600; font-size: 11px; text-transform: uppercase; letter-spacing: .8px; color: var(--muted); cursor: pointer; display: flex; justify-content: space-between; user-select: none; }
  .group-header:hover { color: var(--text); }
  .group-body { padding: 6px 14px 10px; display: flex; flex-direction: column; gap: 6px; }
  .param-row { display: grid; grid-template-columns: 1fr 72px; gap: 8px; align-items: center; }
  .param-row label { color: var(--muted); font-size: 11px; }
  .slider-wrap { display: flex; flex-direction: column; gap: 2px; }
  input[type=range] { width: 100%; accent-color: var(--accent); cursor: pointer; }
  input[type=number] { width: 68px; background: var(--bg); border: 1px solid var(--border); border-radius: 4px; color: var(--text); padding: 2px 6px; font-size: 12px; text-align: right; }
  input[type=number]:focus { outline: none; border-color: var(--accent); }
  input[type=checkbox] { accent-color: var(--accent); cursor: pointer; }

  /* ── Right: visuals ── */
  #right { display: grid; grid-template-rows: 300px 1fr 44px; overflow: hidden; }

  #visuals { display: grid; grid-template-columns: 1fr 2.4fr 1fr; gap: 1px; background: var(--border); border-bottom: 1px solid var(--border); }
  .vis-card { background: var(--bg); display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 8px; gap: 4px; }
  .vis-title { font-size: 10px; text-transform: uppercase; color: var(--muted); letter-spacing: .6px; }
  canvas { display: block; }

  #charts { display: grid; grid-template-columns: repeat(4, 1fr); gap: 1px; background: var(--border); overflow: hidden; }
  .chart-cell { background: var(--bg); min-height: 0; }

  #telem { background: var(--panel); border-top: 1px solid var(--border); display: flex; align-items: center; gap: 24px; padding: 0 18px; flex-shrink: 0; }
  .telem-item { display: flex; flex-direction: column; align-items: center; }
  .telem-label { font-size: 9px; text-transform: uppercase; color: var(--muted); }
  .telem-value { font-size: 15px; font-weight: 600; font-variant-numeric: tabular-nums; color: var(--accent); }
</style>
</head>
<body>

<header>
  <h1>HYDROFOIL SIM</h1>
  <div class="status-dot" id="dot"></div>
  <span id="status-text" style="color:var(--muted);font-size:11px">Stopped</span>
  <button class="btn primary" onclick="runSim()">&#9654; Run</button>
  <button class="btn" onclick="stopSim()">&#9632; Stop</button>
  <button class="btn danger" onclick="resetSim()">&#8635; Reset</button>
  <span style="color:var(--muted);font-size:10.5px" title="Up/Down (or W/S) = height, Left/Right (or A/D) = bank/turn">🕹 W/S = height, A/D = turn</span>
  <span id="manual-indicator" style="color:var(--green);font-size:11px;font-weight:600;opacity:0;transition:opacity .1s"></span>
  <span style="flex:1"></span>
  <span id="sim-time" style="color:var(--muted);font-size:11px;font-variant-numeric:tabular-nums">t = 0.000 s</span>
</header>

<div class="layout">
  <!-- PARAMS PANEL -->
  <div id="params-panel">
    <div id="tab-bar">
      <button class="tab-btn active" data-tab="live" onclick="setTab('live')">LIVE CONTROLS</button>
      <button class="tab-btn" data-tab="config" onclick="setTab('config')">VEHICLE CONFIG</button>
    </div>

    <!-- LIVE tab: setpoints, PID, speed, waves -->
    <div class="tab-pane active" id="tab-live">
      <div class="groups-wrap" id="live-groups"></div>
      <div class="apply-wrap">
        <button class="btn primary" style="width:100%" onclick="applyParams()">Apply &amp; Restart</button>
      </div>
    </div>

    <!-- CONFIG tab: foil geometry, vehicle mass/inertia, sim settings -->
    <div class="tab-pane" id="tab-config">
      <div class="groups-wrap" id="config-groups"></div>
      <div class="apply-wrap">
        <button class="btn" style="width:100%;border-color:#444" onclick="applyParams()">Apply &amp; Restart</button>
      </div>
    </div>
  </div>

  <!-- RIGHT -->
  <div id="right">
    <div id="visuals">
      <div class="vis-card">
        <div class="vis-title">Roll — Front View</div>
        <canvas id="c-roll" width="180" height="280"></canvas>
      </div>
      <div class="vis-card">
        <div class="vis-title">Side Profile</div>
        <canvas id="c-pitch" width="460" height="280"></canvas>
      </div>
      <div class="vis-card">
        <div class="vis-title">Depth</div>
        <canvas id="c-depth" width="160" height="280"></canvas>
      </div>
    </div>

    <div id="charts">
      <div class="chart-cell" id="chart-depth"></div>
      <div class="chart-cell" id="chart-speed"></div>
      <div class="chart-cell" id="chart-angles"></div>
      <div class="chart-cell" id="chart-foils"></div>
    </div>

    <div id="telem">
      <div class="telem-item"><div class="telem-label">Depth (m)</div><div class="telem-value" id="tv-depth">—</div></div>
      <div class="telem-item"><div class="telem-label">Pitch (°)</div><div class="telem-value" id="tv-pitch">—</div></div>
      <div class="telem-item"><div class="telem-label">Roll (°)</div><div class="telem-value" id="tv-roll">—</div></div>
      <div class="telem-item"><div class="telem-label">Heading (°)</div><div class="telem-value" id="tv-heading">—</div></div>
      <div class="telem-item"><div class="telem-label">Speed (m/s)</div><div class="telem-value" id="tv-speed">—</div></div>
      <div class="telem-item"><div class="telem-label">Thrust (N)</div><div class="telem-value" id="tv-thrust">—</div></div>
      <div class="telem-item"><div class="telem-label">Draft (m)</div><div class="telem-value" id="tv-draft">—</div></div>
      <div class="telem-item"><div class="telem-label">Mode</div><div class="telem-value" id="tv-mode" style="font-size:12px">—</div></div>
      <div class="telem-item"><div class="telem-label">d_main (°)</div><div class="telem-value" id="tv-dmain">—</div></div>
      <div class="telem-item"><div class="telem-label">d_elev (°)</div><div class="telem-value" id="tv-delev">—</div></div>
      <div class="telem-item"><div class="telem-label">d_roll (°)</div><div class="telem-value" id="tv-droll">—</div></div>
    </div>
  </div>
</div>

<script>
// ── Parameter groups ───────────────────────────────────────────────────────
// LIVE: things you change to test and tune the running model
// Grouped in the order you'd actually reason about the vehicle: where do I
// want it to fly, how fast, then the gains for each axis, then disturbances.
const LIVE_GROUPS = [
  { label: "① Flight Target", params: [
    { key:"flying_height", label:"Hull Height Over Water (m)", min:-0.3, max:1.0,  step:0.02 },
    { key:"theta_ref",     label:"Target Pitch (°)",           min:-10,  max:10,   step:0.5  },
    { key:"phi_ref",       label:"Target Roll (°)",            min:-10,  max:10,   step:0.5  },
  ]},
  { label: "② Propulsion", params: [
    { key:"target_speed", label:"Target Speed (m/s)",  min:0, max:15,  step:0.1 },
    { key:"kp_speed",     label:"Speed Kp",             min:0, max:100, step:1   },
    { key:"ki_speed",     label:"Speed Ki",             min:0, max:20,  step:0.5 },
    { key:"kd_speed",     label:"Speed Kd",             min:0, max:20,  step:0.5 },
    { key:"max_thrust",   label:"Max Thrust (N)",       min:5, max:200, step:5   },
  ]},
  { label: "③ Height Hold (heave PID)", params: [
    { key:"kp_heave", label:"Kp", min:0, max:20, step:0.1  },
    { key:"ki_heave", label:"Ki", min:0, max:5,  step:0.01 },
    { key:"kd_heave", label:"Kd", min:0, max:20, step:0.1  },
  ]},
  { label: "④ Pitch Hold", params: [
    { key:"kp_pitch", label:"Kp", min:0, max:20, step:0.1  },
    { key:"ki_pitch", label:"Ki", min:0, max:5,  step:0.01 },
    { key:"kd_pitch", label:"Kd", min:0, max:20, step:0.1  },
  ]},
  { label: "⑤ Roll Hold", params: [
    { key:"kp_roll",  label:"Kp", min:0, max:20, step:0.1  },
    { key:"ki_roll",  label:"Ki", min:0, max:5,  step:0.01 },
    { key:"kd_roll",  label:"Kd", min:0, max:20, step:0.1  },
  ]},
  { label: "⑥ Waves (disturbance)", params: [
    { key:"waves_enabled",   label:"Enable Waves",   type:"checkbox" },
    { key:"wave_amplitude",  label:"Amplitude (m)",  min:0.01, max:1.0, step:0.01 },
    { key:"wave_wavelength", label:"Wavelength (m)", min:1,    max:20,  step:0.5  },
    { key:"wave_period",     label:"Period (s)",     min:0.5,  max:10,  step:0.25 },
  ]},
];

// CONFIG: physical constants — rarely changed, require sim restart
const CONFIG_GROUPS = [
  { label: "Vehicle", params: [
    { key:"mass", label:"Mass (kg)",    min:1,   max:100, step:0.5 },
    { key:"ixx",  label:"Ixx (kg·m²)", min:0.1, max:20,  step:0.1 },
    { key:"iyy",  label:"Iyy (kg·m²)", min:0.1, max:20,  step:0.1 },
    { key:"izz",  label:"Izz (kg·m²)", min:0.1, max:20,  step:0.1 },
  ]},
  { label: "Hull (buoyancy & drag)", params: [
    { key:"waterplane_area", label:"Waterplane Area (m²)", min:0.05, max:3,   step:0.05 },
    { key:"max_draft",       label:"Max Draft (m)",        min:0.05, max:1,   step:0.01 },
    { key:"hull_offset",     label:"Hull Above CoG (m)",   min:0,    max:1.5, step:0.05 },
    { key:"hull_drag_coeff", label:"Hull Drag Coeff",      min:0,    max:100, step:1    },
    { key:"heave_damping",   label:"Heave Damping",        min:0,    max:200, step:1    },
  ]},
  { label: "Main Foil", params: [
    { key:"main_chord",  label:"Chord (m)",  min:0.05, max:0.5, step:0.01 },
    { key:"main_span",   label:"Span (m)",   min:0.1,  max:2,   step:0.05 },
    { key:"main_pos_x",  label:"X pos (m)",  min:-2,   max:2,   step:0.05 },
    { key:"main_pos_z",  label:"Z pos (m)",  min:0.1,  max:1.5, step:0.05 },
    { key:"main_lateral_offset", label:"Lateral Offset (m)", min:0.05, max:1.0, step:0.05 },
  ]},
  { label: "Elevator Foil", params: [
    { key:"elev_chord",  label:"Chord (m)",  min:0.05, max:0.4, step:0.01 },
    { key:"elev_span",   label:"Span (m)",   min:0.1,  max:1.5, step:0.05 },
    { key:"elev_pos_x",  label:"X pos (m)",  min:-3,   max:0,   step:0.05 },
    { key:"elev_pos_z",  label:"Z pos (m)",  min:0.1,  max:1.5, step:0.05 },
  ]},
  { label: "Simulation", params: [
    { key:"dt",    label:"Timestep (s)",  min:0.001, max:0.05, step:0.001 },
    { key:"t_end", label:"Duration (s)",  min:5,     max:120,  step:5     },
  ]},
];

let currentParams = {};

// Mirrors the backend's z_ref = hull_offset - flying_height conversion,
// so charts/canvases reference the same target the controller is chasing.
function effectiveZRef() {
  const hullOffset = currentParams.hull_offset ?? 0.6;
  const flyingHeight = currentParams.flying_height ?? 0.1;
  return hullOffset - flyingHeight;
}

// ── Tab switching ──────────────────────────────────────────────────────────
function setTab(name) {
  document.querySelectorAll('.tab-btn').forEach(b =>
    b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tab-pane').forEach(p =>
    p.classList.toggle('active', p.id === 'tab-' + name));
}

// ── Build param UI ─────────────────────────────────────────────────────────
async function loadParams() {
  const r = await fetch("/params");
  currentParams = await r.json();
  buildParamUI();
}

function buildParamUI() {
  buildGroups("live-groups",   LIVE_GROUPS);
  buildGroups("config-groups", CONFIG_GROUPS);
}

function buildGroups(containerId, groups) {
  const container = document.getElementById(containerId);
  container.innerHTML = "";
  groups.forEach(group => {
    const div = document.createElement("div");
    div.className = "group";
    const hdr = document.createElement("div");
    hdr.className = "group-header";
    hdr.innerHTML = `<span>${group.label}</span><span>&#9662;</span>`;
    const body = document.createElement("div");
    body.className = "group-body";

    group.params.forEach(p => {
      const row = document.createElement("div");
      row.className = "param-row";
      const val = currentParams[p.key];
      if (p.type === "checkbox") {
        row.innerHTML = `
          <label for="p-${p.key}">${p.label}</label>
          <input type="checkbox" id="p-${p.key}" ${val ? "checked" : ""}
            onchange="currentParams['${p.key}']=this.checked">`;
      } else {
        row.innerHTML = `
          <div class="slider-wrap">
            <label>${p.label}</label>
            <input type="range" id="slider-${p.key}"
              min="${p.min}" max="${p.max}" step="${p.step}" value="${val}"
              oninput="syncParam('${p.key}', parseFloat(this.value))">
          </div>
          <input type="number" id="p-${p.key}"
            min="${p.min}" max="${p.max}" step="${p.step}" value="${val}"
            oninput="syncParam('${p.key}', parseFloat(this.value))">`;
      }
      body.appendChild(row);
    });

    hdr.onclick = () => { body.style.display = body.style.display === "none" ? "" : "none"; };
    div.appendChild(hdr);
    div.appendChild(body);
    container.appendChild(div);
  });
}

// Setpoints the pilot can nudge live, without stopping/resetting the sim —
// same idea as a stick input feeding an autopilot's outer loop.
const LIVE_CONTROL_KEYS = new Set(["flying_height", "phi_ref"]);

function syncParam(key, val) {
  currentParams[key] = val;
  const slider = document.getElementById("slider-" + key);
  const input  = document.getElementById("p-" + key);
  if (slider) slider.value = val;
  if (input)  input.value  = val;
  if (LIVE_CONTROL_KEYS.has(key)) sendManualUpdate();
}

let manualSendPending = false;
function sendManualUpdate() {
  if (manualSendPending) return;
  manualSendPending = true;
  setTimeout(() => {
    manualSendPending = false;
    fetch("/manual", { method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify({ flying_height: currentParams.flying_height, phi_ref: currentParams.phi_ref }) });
  }, 50); // ~20Hz max — coalesces bursts from held keys or fast slider drags
}

async function applyParams() {
  await fetch("/params", { method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(currentParams) });
}
async function runSim()  { await fetch("/run",  { method:"POST" }); }
async function stopSim() { await fetch("/stop", { method:"POST" }); }

// ── Manual "stick" control: Up/Down (or W/S) = height, Left/Right (or A/D) = bank/turn ──
const MANUAL_KEYS = { up:false, down:false, left:false, right:false };
const KEY_MAP = {
  ArrowUp:"up", w:"up", W:"up",
  ArrowDown:"down", s:"down", S:"down",
  ArrowLeft:"left", a:"left", A:"left",
  ArrowRight:"right", d:"right", D:"right",
};
function updateManualIndicator() {
  const el = document.getElementById("manual-indicator");
  if (!el) return;
  const active = Object.entries(MANUAL_KEYS).filter(([,v]) => v).map(([k]) => k.toUpperCase());
  el.textContent = active.length ? `PILOTING: ${active.join(" + ")}` : "";
  el.style.opacity = active.length ? "1" : "0";
}
window.addEventListener("keydown", e => {
  const k = KEY_MAP[e.key];
  if (!k || MANUAL_KEYS[k]) return;
  MANUAL_KEYS[k] = true; updateManualIndicator();
  if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)) e.preventDefault();
});
window.addEventListener("keyup", e => {
  const k = KEY_MAP[e.key];
  if (!k) return;
  MANUAL_KEYS[k] = false; updateManualIndicator();
});

const HEIGHT_RATE = 0.15;       // m/s while held
const HEIGHT_MIN = -0.3, HEIGHT_MAX = 1.0;
const ROLL_RATE = 20.0;         // deg/s while held (turning)
const ROLL_CENTER_RATE = 15.0;  // deg/s spring-back to level when released
const ROLL_MAX = 10.0;
let lastManualTick = null;

function manualControlTick(now) {
  if (lastManualTick === null) lastManualTick = now;
  const dt = Math.min((now - lastManualTick) / 1000, 0.2);
  lastManualTick = now;

  let fh = currentParams.flying_height ?? 0.1;
  let pr = currentParams.phi_ref ?? 0.0;
  let changed = false;

  if (MANUAL_KEYS.up !== MANUAL_KEYS.down) {
    fh = MANUAL_KEYS.up ? Math.min(fh + HEIGHT_RATE*dt, HEIGHT_MAX)
                         : Math.max(fh - HEIGHT_RATE*dt, HEIGHT_MIN);
    changed = true;
  }
  if (MANUAL_KEYS.left !== MANUAL_KEYS.right) {
    pr = MANUAL_KEYS.left ? Math.max(pr - ROLL_RATE*dt, -ROLL_MAX)
                           : Math.min(pr + ROLL_RATE*dt, ROLL_MAX);
    changed = true;
  } else if (Math.abs(pr) > 0.05) {
    const step = ROLL_CENTER_RATE * dt;
    pr = pr > 0 ? Math.max(0, pr - step) : Math.min(0, pr + step);
    changed = true;
  }

  if (changed) {
    currentParams.flying_height = fh;
    currentParams.phi_ref = pr;
    const fhSlider = document.getElementById("slider-flying_height"), fhInput = document.getElementById("p-flying_height");
    const prSlider = document.getElementById("slider-phi_ref"), prInput = document.getElementById("p-phi_ref");
    if (fhSlider) fhSlider.value = fh; if (fhInput) fhInput.value = fh.toFixed(2);
    if (prSlider) prSlider.value = pr; if (prInput) prInput.value = pr.toFixed(1);
    sendManualUpdate();
  }

  requestAnimationFrame(manualControlTick);
}
requestAnimationFrame(manualControlTick);

function clearCharts() {
  tArr.length = 0; zArr.length = 0;
  phiArr.length = 0; thetaArr.length = 0;
  dmArr.length = 0; deArr.length = 0; drArr.length = 0;
  uArr.length = 0; thrustArr.length = 0;
  Plotly.restyle("chart-depth",  { x: [[], []], y: [[], []] });
  Plotly.restyle("chart-speed",  { x: [[], [], []], y: [[], [], []] });
  Plotly.restyle("chart-angles", { x: [[], []], y: [[], []] });
  Plotly.restyle("chart-foils",  { x: [[], [], []], y: [[], [], []] });
}

async function resetSim() {
  const r = await fetch("/reset", { method:"POST" });
  const data = await r.json();
  currentParams = data.params;
  buildParamUI();
  clearCharts();
}

// ── Charts ─────────────────────────────────────────────────────────────────
const WINDOW = 20; // seconds of history
const tArr=[], zArr=[], phiArr=[], thetaArr=[], dmArr=[], deArr=[], drArr=[], uArr=[], thrustArr=[];

const chartLayout = (title, yLabel) => ({
  title:{ text:title, font:{color:"#8b949e", size:11}, pad:{t:4} },
  paper_bgcolor:"#0d1117", plot_bgcolor:"#0d1117",
  font:{ color:"#8b949e", size:10 },
  margin:{ t:28, b:28, l:44, r:8 },
  xaxis:{ color:"#30363d", gridcolor:"#21262d", title:"t (s)" },
  yaxis:{ color:"#30363d", gridcolor:"#21262d", title:yLabel },
  legend:{ bgcolor:"rgba(0,0,0,0)", font:{size:9} },
  autosize:true,
});

Plotly.newPlot("chart-depth",
  [{ x:[], y:[], name:"z", line:{color:"#58a6ff",width:1.5} },
   { x:[], y:[], name:"z_ref", line:{color:"#f85149",width:1,dash:"dash"} }],
  chartLayout("Depth", "m (NED)"), { responsive:true, displayModeBar:false });

Plotly.newPlot("chart-speed",
  [{ x:[], y:[], name:"speed", line:{color:"#3fb950",width:1.5} },
   { x:[], y:[], name:"target", line:{color:"#f85149",width:1,dash:"dash"} },
   { x:[], y:[], name:"thrust/5", line:{color:"#d29922",width:1,dash:"dot"} }],
  chartLayout("Speed & Thrust", "m/s"), { responsive:true, displayModeBar:false });

Plotly.newPlot("chart-angles",
  [{ x:[], y:[], name:"pitch θ", line:{color:"#d29922",width:1.5} },
   { x:[], y:[], name:"roll φ",  line:{color:"#3fb950",width:1.5} }],
  chartLayout("Angles", "deg"), { responsive:true, displayModeBar:false });

Plotly.newPlot("chart-foils",
  [{ x:[], y:[], name:"d_main", line:{color:"#79c0ff",width:1.5} },
   { x:[], y:[], name:"d_elev", line:{color:"#a371f7",width:1.5} },
   { x:[], y:[], name:"d_roll", line:{color:"#56d364",width:1.5} }],
  chartLayout("Foil Deflections", "deg"), { responsive:true, displayModeBar:false });

let frameCount = 0;
function pushCharts(s) {
  tArr.push(s.t); zArr.push(s.z);
  phiArr.push(s.phi); thetaArr.push(s.theta);
  dmArr.push(s.d_main); deArr.push(s.d_elev); drArr.push(s.d_roll);
  uArr.push(s.u); thrustArr.push(s.thrust);

  // trim to window
  const zRef = effectiveZRef();
  const targetSpeed = currentParams.target_speed ?? 4.0;
  const tMin = s.t - WINDOW;
  while (tArr.length > 1 && tArr[0] < tMin) {
    [tArr,zArr,phiArr,thetaArr,dmArr,deArr,drArr,uArr,thrustArr].forEach(a=>a.shift());
  }

  if (frameCount++ % 3 !== 0) return; // update charts at ~10Hz

  Plotly.restyle("chart-depth", { x:[tArr,tArr], y:[zArr, tArr.map(()=>zRef)] });
  Plotly.restyle("chart-speed", { x:[tArr,tArr,tArr],
    y:[uArr, tArr.map(()=>targetSpeed), thrustArr.map(v=>v/5)] });
  Plotly.restyle("chart-angles", { x:[tArr,tArr], y:[thetaArr, phiArr] });
  Plotly.restyle("chart-foils",  { x:[tArr,tArr,tArr], y:[dmArr, deArr, drArr] });
}

// ── Canvas drawing ─────────────────────────────────────────────────────────
const cRoll  = document.getElementById("c-roll").getContext("2d");
const cPitch = document.getElementById("c-pitch").getContext("2d");
const cDepth = document.getElementById("c-depth").getContext("2d");

function drawRoll(phi_deg) {
  const c = cRoll, w=180, h=280, cx=w/2, cy=h/2, r=70;
  c.clearRect(0,0,w,h);

  // Sky/water split rotated by roll
  const phi = phi_deg * Math.PI/180;
  c.save();
  c.translate(cx, cy);
  c.rotate(phi);

  // Sky
  c.fillStyle="#0a2a4a"; c.fillRect(-r,-r,2*r,r);
  // Water
  c.fillStyle="#0d3b6e"; c.fillRect(-r,0,2*r,r);
  // Horizon line
  c.strokeStyle="#58a6ff"; c.lineWidth=1.5;
  c.beginPath(); c.moveTo(-r,0); c.lineTo(r,0); c.stroke();

  c.restore();

  // Circle mask clip
  c.save();
  c.beginPath(); c.arc(cx,cy,r,0,2*Math.PI); c.clip();
  c.restore();

  // Outer ring
  c.strokeStyle="#30363d"; c.lineWidth=2;
  c.beginPath(); c.arc(cx,cy,r,0,2*Math.PI); c.stroke();

  // Aircraft symbol (fixed)
  c.strokeStyle="#f0c040"; c.lineWidth=2;
  c.beginPath(); c.moveTo(cx-30,cy); c.lineTo(cx-10,cy); c.stroke();
  c.beginPath(); c.moveTo(cx+10,cy); c.lineTo(cx+30,cy); c.stroke();
  c.beginPath(); c.moveTo(cx,cy-6); c.lineTo(cx,cy+6); c.stroke();

  // Label
  c.fillStyle="#8b949e"; c.font="10px monospace"; c.textAlign="center";
  c.fillText(`φ = ${phi_deg.toFixed(1)}°`, cx, h-6);
}

// Airfoil body (teardrop) + a separately-hinged trailing flap, drawn in a
// local frame where +x = forward (chord-wise) and +y = down. `flapDeg` is
// the flap's OWN deflection relative to the foil chord, independent of
// whatever the outer transform (vehicle pitch) is doing.
function drawFoilWithFlap(c, chordPx, thickPx, flapDeg, bodyColor, strokeColor) {
  const flapFrac = 0.38;
  const mainChord = chordPx * (1 - flapFrac);
  const flapChord = chordPx * flapFrac;
  const t = thickPx / 2;
  const le = -chordPx / 2;
  const hinge = le + mainChord;

  c.fillStyle = bodyColor; c.strokeStyle = strokeColor; c.lineWidth = 1.3;
  c.beginPath();
  c.moveTo(le, 0);
  c.quadraticCurveTo(le + mainChord * 0.35, -t, hinge, -t * 0.35);
  c.lineTo(hinge, t * 0.35);
  c.quadraticCurveTo(le + mainChord * 0.35, t, le, 0);
  c.closePath();
  c.fill(); c.stroke();

  c.save();
  c.translate(hinge, 0);
  c.rotate(flapDeg * Math.PI / 180);
  c.beginPath();
  c.moveTo(0, -t * 0.35);
  c.lineTo(flapChord, 0);
  c.lineTo(0, t * 0.35);
  c.closePath();
  c.fill(); c.stroke();
  c.restore();
}

function drawPitch(theta_deg, z_m, draft_m, foilborne, d_main, d_elev, u_speed, wave_phase) {
  const c = cPitch, w=460, h=280;
  c.clearRect(0,0,w,h);

  const surfY = 130;
  const timeNow = performance.now() / 1000;   // wall-clock, only used for the wake's decorative animation

  // Scale + geometry, driven by the actual configured vehicle — not fixed
  // pixel guesses — so this picture matches whatever's in the Config tab.
  // +x screen = forward (bow), matching body-frame +x convention directly.
  const scale = 110; // px/m
  const cogY = surfY + z_m * scale;
  const boatX = w * 0.44;

  // Real wave surface, not decoration: elev(x) = A*cos(k*(x-x_boat) + wave_phase),
  // matching exactly what the physics uses for hull buoyancy — the server sends
  // wave_phase (already the argument of that cos() at the boat's position), so
  // a screen offset from the boat converts straight to a phase offset.
  const wavesOn = currentParams.waves_enabled ?? false;
  const waveA = wavesOn ? (currentParams.wave_amplitude ?? 0) : 0;
  const waveK = 2*Math.PI / (currentParams.wave_wavelength ?? 5.0);
  const phase = wave_phase ?? 0;
  const surfaceYAt = (screenX) => surfY - waveA*scale * Math.cos(waveK*(screenX-boatX)/scale + phase);

  // Sky / water gradients
  const sky = c.createLinearGradient(0,0,0,surfY);
  sky.addColorStop(0,"#071527"); sky.addColorStop(1,"#123354");
  c.fillStyle = sky; c.fillRect(0,0,w,surfY);
  const water = c.createLinearGradient(0,surfY,0,h);
  water.addColorStop(0,"#0d3b6e"); water.addColorStop(1,"#051423");
  c.fillStyle = water; c.fillRect(0,surfY,w,h-surfY);

  // Ripple texture beneath the surface, riding on the real wave shape
  c.strokeStyle="rgba(255,255,255,0.07)"; c.lineWidth=1;
  for (let i=0;i<4;i++){
    c.beginPath();
    let started=false;
    for (let x=0;x<=w;x+=8){
      const yy = surfaceYAt(x) + 20 + i*22;
      if (!started){ c.moveTo(x,yy); started=true; } else c.lineTo(x,yy);
    }
    c.stroke();
  }

  // Real surface line
  c.strokeStyle="#58a6ff"; c.lineWidth=1.8;
  c.beginPath();
  let started=false;
  for (let x=0;x<=w;x+=4){
    const yy = surfaceYAt(x);
    if (!started){ c.moveTo(x,yy); started=true; } else c.lineTo(x,yy);
  }
  c.stroke();
  c.fillStyle="#58a6ff99"; c.font="10px monospace"; c.textAlign="left";
  c.fillText(wavesOn ? `waves: A=${waveA.toFixed(2)}m` : "surface (calm)", 8, surfY-8);

  const hullOffsetPx  = (currentParams.hull_offset ?? 0.6) * scale;
  const mainXPx    = (currentParams.main_pos_x ?? 0.0) * scale;
  const mainZPx    = (currentParams.main_pos_z ?? 0.5) * scale;
  const mainChordPx = (currentParams.main_chord ?? 0.15) * scale * 3.2; // exaggerated for visibility
  const elevXPx    = (currentParams.elev_pos_x ?? -1.2) * scale;
  const elevZPx    = (currentParams.elev_pos_z ?? 0.4) * scale;
  const elevChordPx = (currentParams.elev_chord ?? 0.10) * scale * 3.2;

  const theta = theta_deg * Math.PI/180;

  // Wake: turbulent hull wake while wetted+moving, cleaner foil wake once flying.
  const wetted = draft_m > 0.001;
  const speedFrac = Math.min(Math.abs(u_speed ?? 0) / 6.0, 1.0);
  if (speedFrac > 0.03) {
    const wakeX = boatX - mainChordPx*0.4;
    if (wetted) {
      c.strokeStyle = `rgba(255,255,255,${0.10+0.25*speedFrac})`;
      for (let i=0;i<3;i++){
        c.lineWidth = 1.5 - i*0.3;
        c.beginPath();
        let started=false;
        for (let x=wakeX; x>=0; x-=6){
          const decay = (wakeX-x)/wakeX;
          const yy = surfY + Math.sin((wakeX-x)*0.09 - timeNow*3 + i)*3*(1-decay*0.6);
          if (!started){ c.moveTo(x,yy); started=true; } else c.lineTo(x,yy);
        }
        c.stroke();
      }
    } else {
      c.strokeStyle = `rgba(120,200,255,${0.12+0.2*speedFrac})`;
      c.lineWidth = 1;
      [mainZPx-4, mainZPx+4].forEach(dy=>{
        c.beginPath();
        c.moveTo(boatX+mainXPx-mainChordPx*0.5, cogY+dy);
        c.lineTo(0, cogY+dy);
        c.stroke();
      });
    }
  }

  c.save();
  c.translate(boatX, cogY);
  c.rotate(-theta); // pitch: nose up = negative screen rotation

  const hullY = -hullOffsetPx;

  // Struts: main strut down to the main foil, elevator strut down to the elevator
  c.strokeStyle="#58a6ff"; c.lineWidth=1;
  c.fillStyle="#3d5a80";
  c.beginPath();
  c.moveTo(mainXPx-3, hullY); c.lineTo(mainXPx+3, hullY);
  c.lineTo(mainXPx+1.2, mainZPx); c.lineTo(mainXPx-1.2, mainZPx);
  c.closePath(); c.fill(); c.stroke();

  c.beginPath();
  c.moveTo(elevXPx-2.5, hullY); c.lineTo(elevXPx+2.5, hullY);
  c.lineTo(elevXPx+1, elevZPx); c.lineTo(elevXPx-1, elevZPx);
  c.closePath(); c.fill(); c.stroke();

  // Hull silhouette: pointed bow (+x), flatter transom stern (-x)
  const hullColor   = foilborne ? "#2d6a4f" : "#4a6fa5";
  const hullStroke  = foilborne ? "#52b788" : "#79c0ff";
  c.fillStyle = hullColor; c.strokeStyle = hullStroke; c.lineWidth = 1.6;
  c.beginPath();
  c.moveTo( 66, hullY);            // bow tip
  c.quadraticCurveTo(40, hullY-13, 0, hullY-14);
  c.lineTo(-58, hullY-14);
  c.quadraticCurveTo(-70, hullY-14, -70, hullY-4); // transom
  c.lineTo(-70, hullY+4);
  c.quadraticCurveTo(-10, hullY+12, 30, hullY+8);
  c.closePath();
  c.fill(); c.stroke();
  // Cockpit
  c.fillStyle = "#1c2b3a"; c.strokeStyle = hullStroke; c.lineWidth=1;
  c.beginPath(); c.roundRect(-20, hullY-24, 40, 12, 3); c.fill(); c.stroke();
  // Waterline mark on the hull, so partial submersion reads clearly even when rotated
  if (wetted) {
    c.strokeStyle="rgba(255,255,255,0.5)"; c.lineWidth=1; c.setLineDash([3,2]);
    c.beginPath(); c.moveTo(-70, hullY+4-14*(1-Math.min(draft_m/(currentParams.max_draft??0.3),1)));
    c.lineTo(66, hullY+4-14*(1-Math.min(draft_m/(currentParams.max_draft??0.3),1)));
    c.stroke(); c.setLineDash([]);
  }

  // Foils, each with its own hinged flap showing actual commanded deflection
  c.save(); c.translate(mainXPx, mainZPx);
  drawFoilWithFlap(c, mainChordPx, 7, -d_main, "#2d6a4f", "#52b788");
  c.restore();

  c.save(); c.translate(elevXPx, elevZPx);
  drawFoilWithFlap(c, elevChordPx, 5, -d_elev, "#2d6a4f", "#52b788");
  c.restore();

  c.restore();

  // Labels
  c.fillStyle="#8b949e"; c.font="11px monospace"; c.textAlign="left";
  c.fillText(`θ = ${theta_deg.toFixed(1)}°   z = ${z_m.toFixed(2)} m`, 10, h-38);
  c.fillText(`draft = ${draft_m.toFixed(3)} m   u = ${(u_speed ?? 0).toFixed(2)} m/s`, 10, h-22);
  c.fillText(`δ_main = ${d_main.toFixed(1)}°   δ_elev = ${d_elev.toFixed(1)}°`, 10, h-6);
  c.fillStyle = foilborne ? "#3fb950" : "#d29922";
  c.font="bold 13px monospace"; c.textAlign="right";
  c.fillText(foilborne ? "◆ FOILBORNE" : "◆ DISPLACEMENT", w-10, h-10);
}

function drawDepth(z_m, z_ref) {
  const c = cDepth, w=160, h=280;
  c.clearRect(0,0,w,h);

  const maxDepth = 3.0;
  const scale = (h-40) / maxDepth;

  // Background
  c.fillStyle="#0d1117"; c.fillRect(0,0,w,h);

  // Water column
  c.fillStyle="#0d2b47"; c.fillRect(30, 20, 60, h-40);
  // Surface
  c.strokeStyle="#58a6ff"; c.lineWidth=1.5;
  c.beginPath(); c.moveTo(30,20); c.lineTo(90,20); c.stroke();

  // Ref line
  const refY = 20 + z_ref * scale;
  c.strokeStyle="#f85149"; c.lineWidth=1; c.setLineDash([4,3]);
  c.beginPath(); c.moveTo(20, refY); c.lineTo(100, refY); c.stroke();
  c.setLineDash([]);
  c.fillStyle="#f85149"; c.font="9px monospace"; c.textAlign="left";
  c.fillText(`ref ${z_ref.toFixed(2)}m`, 102, refY+3);

  // Current depth bar
  const depthY = 20 + Math.min(z_m, maxDepth) * scale;
  c.fillStyle="#58a6ff55"; c.fillRect(30, 20, 60, depthY-20);

  // Boat marker
  c.fillStyle="#79c0ff"; c.beginPath(); c.arc(60, depthY, 7, 0, 2*Math.PI); c.fill();

  // Depth axis ticks
  c.fillStyle="#8b949e"; c.font="9px monospace"; c.textAlign="right";
  for(let d=0; d<=maxDepth; d+=0.5) {
    const y = 20 + d * scale;
    c.beginPath(); c.moveTo(28,y); c.lineTo(30,y); c.strokeStyle="#30363d"; c.lineWidth=1; c.stroke();
    c.fillText(d.toFixed(1), 26, y+3);
  }

  c.fillStyle="#8b949e"; c.font="10px monospace"; c.textAlign="center";
  c.fillText(`${z_m.toFixed(3)} m`, w/2, h-4);
}

// ── WebSocket ──────────────────────────────────────────────────────────────
function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onmessage = e => {
    const s = JSON.parse(e.data);

    // Status
    const dot = document.getElementById("dot");
    const txt = document.getElementById("status-text");
    if (s.running) { dot.classList.add("running"); txt.textContent = "Running"; }
    else           { dot.classList.remove("running"); txt.textContent = "Stopped"; }
    document.getElementById("sim-time").textContent = `t = ${s.t.toFixed(3)} s`;

    // Telemetry
    document.getElementById("tv-depth").textContent  = s.z.toFixed(3);
    document.getElementById("tv-pitch").textContent  = s.theta.toFixed(2);
    document.getElementById("tv-roll").textContent   = s.phi.toFixed(2);
    document.getElementById("tv-heading").textContent = s.psi.toFixed(1);
    document.getElementById("tv-speed").textContent  = s.u.toFixed(2);
    document.getElementById("tv-dmain").textContent  = s.d_main.toFixed(2);
    document.getElementById("tv-delev").textContent  = s.d_elev.toFixed(2);
    document.getElementById("tv-droll").textContent  = s.d_roll.toFixed(2);
    document.getElementById("tv-thrust").textContent = s.thrust.toFixed(1);
    document.getElementById("tv-draft").textContent  = s.draft.toFixed(3);
    const modeEl = document.getElementById("tv-mode");
    modeEl.textContent = s.foilborne ? "FOILBORNE" : "DISPLACEMENT";
    modeEl.style.color = s.foilborne ? "var(--green)" : "var(--yellow)";

    // Visuals
    drawRoll(s.phi);
    drawPitch(s.theta, s.z, s.draft, s.foilborne, s.d_main, s.d_elev, s.u, s.wave_phase);
    drawDepth(s.z, effectiveZRef());

    // Charts
    if (s.running) pushCharts(s);
  };

  ws.onclose = () => setTimeout(connect, 1000);
}

// ── Init ───────────────────────────────────────────────────────────────────
loadParams().then(() => connect());
</script>
</body>
</html>"""


if __name__ == "__main__":
    import socket as _socket
    try:
        _s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        _s.connect(("8.8.8.8", 80))
        _wsl_ip = _s.getsockname()[0]
        _s.close()
    except Exception:
        _wsl_ip = None

    print("=" * 55)
    print("  Hydrofoil Dashboard")
    print("  Open in your Windows browser:")
    print("    http://localhost:8000")
    if _wsl_ip and not _wsl_ip.startswith("127."):
        print(f"    http://{_wsl_ip}:8000  (WSL2 direct IP)")
    print("=" * 55)
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
