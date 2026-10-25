# Hydrofoil Sim

Python 6-DOF physics simulation (Fossen notation) of a fully-submerged T-foil
hydrofoil, with a live browser dashboard and a real-time Unity visualizer
driven over UDP. Built for UBC Okanagan Marine Robotics Hydrofoil Club.

## Missing file — read this first

`unity-visualizer/Assets/Foil/Mockup4_JacobRig_Merged.blend` is **not** in
this repo — it's 709MB, over GitHub's 100MB file limit. Jacob is sending it
to you separately. Once you have it, drop it in at exactly that path
(`unity-visualizer/Assets/Foil/`) before opening the Unity project, or the
boat hull won't render (everything else — scripts, scene, materials, physics
— works fine without it).

## What's here

- `hydrofoil/` — the Python simulation. `vehicle.py` is the canonical physics
  (6-DOF rigid body, strip-theory foils, hull buoyancy/drag, wave coupling,
  cascade PID control). `dashboard.py` is the interactive browser UI.
  `sim_realtime.py` streams state to Unity over UDP.
- `unity-visualizer/` — the Unity project (trimmed: Unity's default
  HDRP/terrain template sample content is left out, only the actual
  hydrofoil scene/scripts/materials are here).
- `tests/` — pytest unit tests (`test_forces.py`, `test_actuator.py`).
- `notebooks/` — currently empty, reserved for analysis notebooks.
- `requirements.txt` — Python deps.

## Running it

```bash
pip install -r requirements.txt

# Browser dashboard (sliders for every param, live plots) → localhost:8000
python -m hydrofoil.dashboard

# Tests
pytest tests/

# Stream to Unity over UDP (run this, then hit Play in the Unity project)
python -m hydrofoil.sim_realtime
```

Open `unity-visualizer/` as a Unity project (Unity Hub → Add → select this
folder) to get the live visualizer. `Assets/HydrofoilSim/PythonSimDriver.cs`
is what receives state from `sim_realtime.py` over UDP and drives the scene;
`SimModeController.cs` switches between that and a built-in arcade mode.

## Known state / heads-up for anyone picking this up

- Control is three independent PID loops (heave/pitch/roll), not yet a MIMO
  controller — see `hydrofoil/control/pid.py`. Roll `Ki` must stay `0`
  (verified to diverge otherwise under wave disturbance). Pilot bank input
  is clamped to ±8° (`hydrofoil/control_uplink.py`) — commanded bank beyond
  that has been verified to diverge.
- The foil force model (`hydrofoil/forces/foil.py`) includes flap
  effectiveness, finite-span, free-surface, and induced-drag corrections.
  PID gains have **not** been retuned against these corrections yet — expect
  the response to feel under-authoritative until that happens.
- No state estimation or sensor noise modeling exists yet (`estimation/`,
  `sensors/` are empty packages) — the sim feeds every controller exact
  ground-truth state.
- No linearized state-space (A/B) model of the plant exists yet — that's the
  prerequisite for the planned LQR/MPC work and hasn't been started.
