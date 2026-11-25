# 07 — Interfaces

> **Current Viewer & Web UI Interfaces:** The active application serves `hydrofoil/assets/launcher/page.html` (side elevation + top-down turning & oblique-wave canvas, hydro/cavitation/encounter/heading telemetry cards, Boat Editor, and Phase-Aware Autotuner) via `hydrofoil/sim_server.py` (with `/api/design`, `/api/design/preview`, `/api/design/save`, `/api/load_cad`, `/api/relaunch`, and `/api/tune/*`), backed by the MuJoCo 3D viewer's HTTP `ControlAPI` in `hydrofoil/viewer.py`.
> - **`GET /state`** returns live telemetry including `t`, `mode`, `preset`, `mass`, `x`, `speed`, `speed_ref`, `height`, `height_ref`, `roll`, `pitch`, `bank_ref`, `yaw`, `yaw_rate`, `yaw_rate_cmd`, `sideslip`, `bank_max`, `thrust`, `power_kw`, `draft`, `foil_depth_hc`, `cavitation_sigma`, `cp_min_max`, `ventilation`, `surfaces`, `stops`, `drives`, `alloc`, `turn_envelope`, `height_status`, `controller`, `controller_detail`, `geom` (hull, struts, flaps, `cockpit_box`, `cockpit_buoyancy`), and `wave` (`type`, `amp`, `len`, `dir`, `rel_dir`, `omega_e`, `period_e`, `regime`).
> - **`POST /set`** accepts `safety_margin`, `allocator`, `reverse_drive`, `yaw_rate_cmd`, `controller_mode`, `unit`, slider setpoints, and `action` commands.
> - **`POST /api/load_cad`** accepts optional `{form}` settings from the launcher form so launching/resetting the CAD preset preserves controller, sensor, wave, and allocator selections while clearing saved `cad` overrides.
> - **CLI flags (`python -m hydrofoil.viewer`):** defaults to `--preset cad` (`cad|model|manned`), alongside `--cockpit-buoyancy on|off`, `--controller`, `--sensors`, `--supervisor`, `--allocator`, `--safety-margin`, `--wave-type`, `--wave-dir`, `--waves`, `--overrides`, and `--speed`. Removed 2026-11-05 Unity UDP/FastAPI specs remain archived below.

## 1. (Removed) UDP bridge to Unity

> Removed on branch `mujoco-viewer`: `stream.py`, `control_uplink.py`, `sim_realtime.py`. The formats below are kept for history (git commit `841d501`).

### 1a. UDP: Python → Unity (state), port 5005

Sender: `SimStreamer` (`stream.py`), one JSON datagram per stream tick (~30 Hz: every `int(0.033/dt)` steps).
Destination: `--host` (default: WSL gateway via `ip route show default`, else `127.0.0.1`).

```json
{"t":1.23,"x":0,"y":0,"z":0.5,"phi":0,"theta":0.02,"psi":0,
 "u":4,"v":0,"w":0,"d_main":2.1,"d_elev":0.5,"d_roll":0,
 "thrust":35.2,"draft":0.0,"foilborne":true,"wave_elev":0.01}
```

| Field | Unit | Meaning |
|---|---|---|
| `t` | s | sim time |
| `x,y,z` | m | NED position (z down) |
| `phi,theta,psi` | **rad** | roll, pitch, yaw |
| `u,v,w` | m/s | body-frame linear velocity (no p,q,r sent) |
| `d_main,d_elev,d_roll` | **deg** | actuator-filtered flap angles |
| `thrust` | N | surge-PID thrust |
| `draft` | m | hull draft below local surface |
| `foilborne` | bool | draft ≤ 1e-6 |
| `wave_elev` | m | wave elevation at vehicle |

Unity's `SimPacket` mirrors these fields exactly (`JsonUtility`).

### 1b. UDP: Unity → Python (pilot), port 5006

Receiver: `PilotUplink` bound to `0.0.0.0:5006`, 1 KB max datagram, 0.5 s socket timeout loop.

```json
{"flying_height":0.55,"target_speed":4.0,"phi_ref":0.0}
```
Any subset of keys. Values are clamped (height −0.3…1.0 m; speed 0…10 m/s; bank ±8°). Bad JSON / non-numeric values are ignored. Sent at ~20 Hz from `PythonSimDriver`.

## 3. Dashboard HTTP / WebSocket (port 8000)

> **Removed 2026-11-05** with the dashboard (doc 08). Kept below for history.

| Method | Path | Body | Effect |
|---|---|---|---|
| GET | `/` | – | Dashboard HTML |
| GET | `/params` | – | Current params dict |
| POST | `/params` | dict | Merge into params and **restart** the sim |
| POST | `/manual` | dict | Merge into params, **no restart** (stick input: `flying_height`, `phi_ref`) |
| POST | `/run` | – | Start (no-op if already running) |
| POST | `/stop` | – | Stop |
| POST | `/reset` | – | Stop, restore `DEFAULT_PARAMS`; returns them |
| WS | `/ws` | – | Pushes the state JSON at 30 Hz |

WebSocket state keys: `running, t, z, phi, theta, psi` (degrees for angles), `u, d_main, d_elev, d_roll, draft, thrust, foilborne, phi_ref_live, wave_elev, wave_phase`.
Note these are **not** the same units as the UDP packet (angles in degrees here, radians there). No authentication; the server binds `0.0.0.0` (`uvicorn.run(app, host="0.0.0.0", port=8000)`), so anyone on the network can drive it.

## 4. CLI

```
python -m hydrofoil.sim
python -m hydrofoil.viewer [--speed X] [--duration 600] [--waves]      # the app (doc 08)
python tools/actuator_budget.py [--ratios 10 20 30] [--vbus 24]
pytest tests/
```
Viewer keys: Up/Down height, Left/Right bank, [ ] speed, R reset.

## 5. Python API (for scripts and a future linearisation/LQR driver)

```python
from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater, AiryWave

p  = dict(DEFAULT_PARAMS)
vp = VehiclePhysics(p)
s  = vp.rest_state()                       # 12-vector
s, cmds, thrust, draft, wave_elev = vp.step(s, t, CalmWater())
```
`vp.physics_rhs(state, cmds, thrust, water_surface_z, wave_vel_body)` is the pure nonlinear dynamics, suitable for finite-difference linearisation
(`cmds` = dict with `delta_main`, `delta_elev`, `delta_roll` in degrees).
`DEFAULT_PARAMS` now lives in `hydrofoil/params.py` (no FastAPI import).
