# Hydrofoil Sim — Project State

Snapshot of the Python simulation as of 2026-09-17. Written to orient
control-theory work (PID is done; next is LQR/MPC) — not a tutorial, a map
of what exists, what physics it encodes, and what's still a stub.

## What this is

A 6-DOF marine-vehicle physics sim (Fossen notation) for a fully-submerged
T-foil hydrofoil, closed-loop via cascade PID, with a live browser dashboard
and a real-time UDP bridge to a Unity visualizer. Runs in WSL2; Unity runs on
the Windows host. No hardware/firmware code exists yet — that's a separate,
currently-deprioritized track (see `estimation/`/`sensors/` below and the
project memory `project_hydrofoil_hardware.md`).

Three ways to run it:
- `python -m hydrofoil.dashboard` — FastAPI/WebSocket app at `localhost:8000`,
  sliders for every param, canvas + Plotly visuals, keyboard "stick" control.
  This is the actively-developed entry point.
- `python -m hydrofoil.sim` — headless batch run, saves a static PNG and
  Plotly HTML dashboards to `../data/`. Simpler, older model (see below);
  used for quick trim/plotting checks, not kept in sync with `vehicle.py`.
- `python -m hydrofoil.sim_realtime` — runs `vehicle.py`'s physics at
  wall-clock speed, streams state to Unity over UDP, and accepts pilot
  climb/dive/bank/throttle input back from Unity.

## Repo layout

```
hydrofoil/
  vehicle.py           canonical physics — shared by dashboard.py & sim_realtime.py
  dashboard.py          FastAPI app + self-contained HTML/JS dashboard
  sim.py                standalone headless batch sim (older, simpler, DIVERGED model)
  sim_realtime.py       wall-clock loop -> UDP -> Unity
  stream.py             UDP JSON sender (state -> Unity)
  control_uplink.py     UDP JSON receiver (pilot stick -> Python)
  viz.py                Plotly HTML report/animation generator (used by sim.py)
  dynamics/
    rigid_body.py       M_RB, C_RB, nu_dot  (Fossen ch. 3-4)
    kinematics.py       J(eta): body-rate -> world-rate transform
  forces/
    foil.py             strip-theory lift/drag from a CL/CD polar table
    buoyancy.py          constant-volume buoyancy + gravity (used by sim.py only)
    hull.py               draft-dependent buoyancy/drag (used by vehicle.py)
  control/
    pid.py               PID class + HydrofoilController (heave/pitch/roll cascade)
    surge.py              closed-loop speed -> thrust PID
    actuator.py            per-flap delay + rate-limit + first-order lag model
  waves/
    airy.py               linear (Airy) regular wave: elevation + orbital velocity
  estimation/           EMPTY — no state estimator exists (sim feeds ground truth)
  sensors/              EMPTY — no sensor noise/model exists
data/foil_tables/naca0009_xflr5_re2700000.dat   the only polar in the repo
```

`estimation/` and `sensors/` are empty stub packages — placeholders for when
a real vehicle needs an observer (IMU/depth-sensor fusion → state estimate)
instead of reading exact state directly. Not started.

No test suite exists (`find -iname "test_*"` turns up nothing). Verification
so far has been ad-hoc headless scripts run during development sessions, not
checked-in tests.

## Physics model — what's actually simulated

**State**: `state = [eta; nu]`, 12-vector.
`eta = [x, y, z, phi, theta, psi]` — NED world position + ZYX Euler angles.
`nu = [u, v, w, p, q, r]` — body-frame linear + angular velocity.
`z` positive **down** (NED); `phi`/`theta`/`psi` in radians internally, UI/UDP
uses degrees at the boundary.

**Rigid body** (`dynamics/rigid_body.py`): `M_RB * nu_dot + C_RB(nu) * nu = tau`,
exactly Fossen eq. 3.46. `M_RB` is diagonal mass + full 3x3 inertia tensor
(off-diagonal products of inertia are zero — no measured inertia tensor
exists yet, `ixx/iyy/izz` are round-number guesses: 2.0/5.0/5.0 kg·m²).
CoG offset `r_g` defaults to zero (body origin = CoG assumed).
**No added mass, no linear/quadratic hydrodynamic damping matrix** — the only
damping in the system comes from foil drag, hull drag, and an ad-hoc heave
damping term (`heave_damping * nu[2]`, tuned by feel, not derived).

**Kinematics** (`dynamics/kinematics.py`): standard ZYX Euler `J(eta)`.
Singular at `theta = ±90°` (gimbal lock) — raises `ValueError` in the raw
transform; `vehicle.py` clamps attitude to ±70° specifically to stay clear
of this (see Safety net below).

**Foils** (`forces/foil.py`): strip theory, one `Foil` object per lifting
surface, no spanwise discretization (whole foil treated as one strip at its
attachment point). AoA = `atan2(w_local, u_local)` plus a fixed geometric
offset plus the commanded flap deflection. Force/moment computed from a
`FoilTables` CL/CD lookup (linear interpolation, **clamped** at the table
edges — no stall recovery, no post-stall model). The only polar in the repo
is NACA 0009 from XFLR5 at `Re=2.70e6`, used for every surface (main foil
halves and elevator) regardless of their actual chord/local Re. Water
density fixed at 1025 kg/m³.

Vehicle has three foil surfaces:
- Left + right main foil halves, laterally offset (`main_lateral_offset`),
  each independently flappable — `delta_main ∓ delta_roll` on left/right.
  This differential-lift-times-offset is the *only* roll-moment mechanism;
  there's no separate rudder or roll thruster.
- One elevator (pitch authority, `delta_elev`), aft and below.

**Hull buoyancy/drag** (`forces/hull.py`, used by `vehicle.py`): wall-sided
linear approximation — buoyancy grows linearly with draft
(`waterplane_area * draft`) up to `max_draft`, then the hull is "foilborne"
(draft = 0, zero buoyancy contribution, foils carry 100% of weight). Hull
drag is quadratic in surge speed, scaled by wetted fraction
(`draft / max_draft`), coefficient `hull_drag_coeff` is a placeholder
constant (20.0), not derived from hull geometry or towing data. This module
is what makes the water-to-foilborne "takeoff" transition possible; the
older `forces/buoyancy.py::buoyancy_force()` (constant-volume, always fully
submerged) is only used by `sim.py`'s simpler standalone model and cannot
represent takeoff.

**Waves** (`waves/airy.py`): single regular (monochromatic) Airy wave —
amplitude/wavelength/period/direction, deep-water orbital velocity decaying
exponentially with depth. No wave spectrum (e.g. JONSWAP/Pierson-Moskowitz),
no irregular sea state, no current. `CalmWater` subclass zeroes everything
for the no-disturbance case. Wave orbital velocity is subtracted from body
velocity before it reaches the foils (relative flow), so waves disturb the
vehicle only through the foils' AoA — there's no direct wave force on the
hull itself beyond what's captured in buoyancy via the wave-displaced local
surface (`water_surface_z`).

**Propulsion**: single closed-loop surge PID (`control/surge.py`) maps speed
error → thrust `[N]`, clamped to `±max_thrust`, applied as a pure `+x`
body-frame force. No propeller model, no motor dynamics, no two-motor
differential thrust for yaw — thrust is a scalar force at the CoG. This is a
placeholder for the real twin-BLDC propulsion (see hardware memory); the
`max_thrust: 50.0` default is explicitly labeled a guess matching Unity's
arcade-mode `TwinMotorController` placeholder, not a real number.

## Control architecture

Three independent SISO PID loops (`control/pid.py::HydrofoilController`),
cascaded onto flap actuator models, plus one more PID for surge:

| Loop  | Error                     | Output        | Actuator            |
|-------|---------------------------|----------------|---------------------|
| heave | `z_meas - z_ref`          | `delta_main`   | main flap (symmetric) |
| pitch | `theta_meas - theta_ref`  | `delta_elev`   | elevator flap        |
| roll  | `phi_meas - phi_ref`      | `delta_roll`   | main flap differential |
| surge | `speed_ref - speed_meas`  | thrust [N]     | (none — direct force) |

Each `PID` is discrete-time, **derivative-on-measurement** (not
derivative-on-error — deliberately, to avoid a "derivative kick" when a
pilot stick input changes the setpoint discontinuously; see the docstring in
`pid.py` for the full rationale), with output clamping and anti-windup
(integrator step undone when the clamp is active).

Gains are **not coupled** — each loop is tuned independently against its own
axis in isolation. There is no MIMO/state-space controller anywhere in the
repo; LQR/MPC (the stated Phase 3 roadmap item) has not been started.

Current tuned gains (`dashboard.py::DEFAULT_PARAMS`, mirrored in `sim.py`):
```
heave: Kp=15.0  Ki=1.0  Kd=5.0
pitch: Kp=2.0   Ki=0.0  Kd=2.0
roll:  Kp=3.5   Ki=0.0  Kd=2.5   <- roll Ki MUST stay 0 (see below)
speed: Kp=30.0  Ki=5.0  Kd=0.0
```
`sim.py` uses different heave gains (`Kp=1.0, Kd=3.0`) — it starts the
vehicle already near-trim rather than from rest, so it doesn't need
`dashboard.py`'s more aggressive takeoff-authority gains. The two files are
not meant to be kept identical; `vehicle.py` is the one source of truth for
anything beyond a quick trim check.

**Actuator model** (`control/actuator.py`): every flap channel gets a pure
delay (`delay_steps`, default 2 steps of `dt`), a rate limit
(`rate_limit_deg_s`, default 720°/s), and a first-order lag (`tau=0.02s`).
These are explicitly labeled placeholders for a MyActuator RMD-X8-class
CAN-bus direct-drive BLDC actuator, not measured hardware numbers.

## Known stability limits (empirically verified, not theoretical)

These came out of headless stress testing during the roll-unlock work and
are load-bearing constraints for anything built on top of this sim:

- **Roll gains** `Kp=3.5, Kd=2.5, Ki=0` were reached by sweeping — the old
  `Kp=1, Kd=1` gains didn't diverge, they converged to a *wrong* ~40° list
  under disturbance. Any nonzero roll `Ki` reliably diverges under wave
  disturbance (verified: `Ki=0.05` → 1000+° within 90s) — Fossen gyroscopic
  roll↔yaw↔pitch coupling (`C_RB(nu)`, real physics, not a bug) outruns a
  naive integral term.
- **Sustained commanded bank** is a materially harder stability question
  than *recovering from* a disturbance. A held bank converges to roughly
  1.3–1.5x the commanded angle (expected for a P-dominant loop with no
  integral) and yaw builds continuously — a real coordinated turn, correct
  behavior. But commanded bank ≥ 12° reliably diverges to overflow within
  ~6s even in calm water. This is why pilot bank input is hard-clamped to
  **±8°** in both `control_uplink.py` (`BANK_MIN/MAX`) and `dashboard.py`'s
  JS stick handler (`ROLL_MAX`) — do not widen without re-running the sweep.
- **Rapid multi-axis combined pilot input** (bank + height + speed all
  changing chaotically at once) can still diverge to NaN/Inf even within the
  ±8° bank cap — found via randomized fuzz testing after a real Unity crash
  report (`NaN` position). This is a genuine control-authority gap: three
  independent SISO loops can't always track gyroscopically-coupled dynamics
  under adversarial combined input. **Not fixed at the controller level** —
  explicitly scoped as a Phase 3 (coupled/MIMO, i.e. LQR/MPC) problem.

**Mitigation in place** (`vehicle.py::step()`, not a controller fix, a
safety net): after each RK4 step, if the new state isn't finite, roll back
to the last valid state with zero velocity/rates and reset both controllers;
otherwise clamp attitude to ±70° (Euler-singularity margin), linear velocity
to ±15 m/s, angular velocity to ±3 rad/s (~172°/s). This guarantees Unity
never receives NaN, at the cost of the sim looking like an aggressive, ugly
maneuver in the extreme-mashing case instead of crashing the receiver. The
user explicitly chose to leave this residual behavior as-is rather than add
rate-limiting or auto-recovery logic on top.

## Unity integration

- `sim_realtime.py` → `stream.py::SimStreamer` sends one UDP JSON packet per
  stream tick (~30 Hz) to the Windows host (auto-detected via
  `ip route show default` from inside WSL2), port 5005 by default.
- `control_uplink.py::PilotUplink` listens on UDP port 5006 for
  `{"flying_height", "target_speed", "phi_ref"}` packets sent back from
  Unity's `PythonSimDriver.cs`. Values clamp on receipt.
- Unity-side controls: A/D → bank target (spring-centered to 0 on release),
  W/S → speed target, Up/Down arrows → climb/dive target. A top-right HUD
  shows front-left/front-right/back flap angles. Model is rotated -90°
  (`modelYawOffsetDeg`) to correct a facing mismatch discovered earlier.
- This is a one-way physics authority split: Python owns all dynamics,
  Unity is purely a renderer + input source, never simulates anything
  itself.

## What's unfinished / not started

- **State estimation** (`estimation/`) and **sensor modeling** (`sensors/`):
  both empty packages. The sim currently feeds every controller exact
  ground-truth state — no IMU noise, no depth-sensor model, no observer
  (Kalman filter or otherwise). Needed before this sim can meaningfully
  validate anything that will run on real sensors.
- **LQR/MPC** (Phase 3 of the stated roadmap): not started. This is the
  actual "control theory" work the user wants to start — the sim's current
  bottleneck for that is the stability limits above (a MIMO controller is
  the natural fix for the combined-input divergence) and the lack of a
  linearized state-space model of the plant anywhere in the codebase (no
  `A`/`B` matrices exist yet — `vehicle.py::physics_rhs` is nonlinear and
  has never been linearized about a trim point).
- **No test suite.** Stability/regression claims above come from throwaway
  scripts run ad-hoc, not from anything checked into the repo.
- **Hardware/firmware**: explicitly deprioritized by the user (see
  `project_hydrofoil_hardware.md` memory — target is a NUCLEO-H723ZG +
  two Maytech MTSPF75200 VESC ESCs over CAN). No firmware code exists.
  Actuator and thrust models in the sim remain labeled placeholders
  pending real datasheet/bench numbers.
- **No measured vehicle parameters** anywhere: mass, inertia, foil
  geometry, hull waterplane area/drag coefficient, actuator dynamics are
  all engineering guesses, not measurements or CAD-derived numbers.
- **`sim.py` has drifted from `vehicle.py`** (older buoyancy model, no
  surge control, no wave coupling in the same form, no safety-net clamp) —
  kept intentionally as a lightweight trim-check tool, not meant to be
  merged back.
