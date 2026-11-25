# 26 — CAD boat preset (`--preset cad`)

Team summary in plain language: [27-cad-boat-summary-for-the-team.md](27-cad-boat-summary-for-the-team.md).

The `cad` preset (`hydrofoil/params.py: CAD_PARAMS`) is our boat as drawn in SolidWorks, so the sim shows what
**that** design does, not a tuned stand-in. It is the dashboard's default boat; **Load our CAD boat** (top bar)
or **Load our CAD values** (Boat editor) clears any saved edits for it and relaunches. Nothing in it was tuned to
make it fly.

Sources: `Hydrofoil Assembly+wings_not_finished.STL` (full boat, wings added roughly) and `Split Wing
Assembly.STL` (main wing without flaps), measured 2026-11-24 by slicing the meshes. Body frame origin = assumed CG;
CAD → body: `x = (X_cad − 1716 mm)` forward, `z_down = (863 mm − Y_cad)`.

## Measured from CAD

| Item | CAD | Sim key(s) |
|---|---|---|
| Main wing section | NACA 0009 (9.0 % thick, symmetric) | polar (unchanged) |
| Main wing span / area (flaps in) | 1.353 m / 0.3665 m², AR 5.0, rounded tips, straight LE | `main_span` 0.6765 (per half), `main_chord` 0.271 (area-equivalent) |
| Half-wing area centroid | 0.31 m from centreline | `main_lateral_offset` |
| Mounting angle, both wings | 0.0° (level with the pontoons) | `main_incidence_deg` 0 (slider 0–5°) |
| Flaps | two, 0–0.24 m from the centreline each side, 100 mm of the 300 mm chord | `main_flap_frac` 0.333, `main_flap_span_frac` 0.355, `main_flap_center_y` 0.12 |
| Rear wing | 0.149 × 0.677 m, NACA 0009, all-moving | `elev_chord`, `elev_span` |
| Wing spacing (quarter chords) | 2.391 m | `elev_pos_x` −2.391 |
| Strut length (pontoon bottom → wing) | 0.749 m; rear wing 19 mm lower | `main_pos_z` 1.099, `elev_pos_z` 1.118 |
| Front masts | two, ±0.35 m, 0.28 m chord, 51 mm thick | `main_strut_count` 2, `main_strut_y`, `main_strut_chord` |
| Rear mast | one, centreline, 102 × 76 mm | `elev_strut_chord` 0.10 |
| Hull | twin pontoons, 2.54 m × Ø0.265 m, 1.308 m apart, centre 0.483 m aft of the main wing | `hull_twin_spacing`, `hull_length`, `hull_beam` 0.208 (= π/4 × Ø, same volume), `max_draft` 0.265, `hull_x` |
| Cockpit box | sealed 48 × 32 × 24 in (1.219 × 0.813 × 0.610 m, 0.605 m³ ≈ 620 kg buoyancy), centred in the frame (0.347 m aft of the main wing), bottom level with the pontoon bottoms | `cockpit_box` |
| Propellers | two, Ø ≈ 0.21 m, 0.386 m aft of and 0.139 m above the main wing | `prop_pos_z` 0.96 (thrust line below the CG) |

## Assumed (not in the STL: it has no mass)

* **Mass 275 kg** and inertias: the manned preset's placeholders.
* **CG** longitudinally over the main wing's quarter chord, 0.35 m above the pontoon bottoms.
* Elevator pivot (22 % chord; the rear mast is at 49 %), flap and elevator travel, drive train, hull drag: manned values.
* Flying height 0.35 m (wing 0.40 m = 1.5 chords deep).
* Controllers: the manned preset's PID gains and LQR weights, unchanged.

## Model changes this needed

* `Foil.flap_dy`: a part-span flap's extra force acts at the flap, so roll authority uses the real (short) lever
  arm; ours is about 0.39× of a flap centred on the half.
* `HullHydrostatics(twin_spacing=…)`: catamaran footprint (supervisor clearances use the outer pontoon edge).
* `main_strut_count`: the two front masts.
* `HullHydrostatics(boxes=…)`: extra buoyant boxes (the cockpit) with their own height, plus `box_drag`, a
  flat-faced box's water drag (C_D 1 on the submerged face) so the buoyancy doesn't come for free.
* `VehiclePhysics.set_main_incidence()` and the viewer's **Main wing mounting angle** slider: physics changes at
  once; trim feed-forward and LQR are re-derived in the background (PID flies meanwhile).
* The LQR skips speeds where the boat has no level trim inside the surface limits, and falls back to the PID loops
  (with a dashboard message) if there are none.

## Cockpit buoyancy toggle

Dashboard launch bar → **Cockpit box buoyancy (CAD boat)** (viewer: `--cockpit-buoyancy on|off`,
params: `cockpit_buoyancy`). `params.resolve_cad_hull()` turns it into `hull_beam` / `max_draft` / `hull_boxes`:

* **On** (default): CAD pontoons (Ø 0.265 m, 287 kg) + the floating, dragging cockpit box.
* **Off**: the box is out of the water calculation (drawn see-through) and the pontoon **diameter** is scaled so
  the two pontoons alone hold `pontoon_reserve` × the boat's weight when fully under (default 2.0 = floats about
  half under; never smaller than CAD). At 275 kg that is Ø 0.367 m (550 kg). Length and spacing stay as drawn; the
  frame is drawn on top of the larger pontoons, the strut length is still measured from the pontoon bottoms.

## What the sim shows (2026-11-24, with the cockpit box)

At 0° the boat **does not fly**:

1. **At rest it noses under** (pitch −70° after 30 s, no thrust). The pontoons + box hold 900 kg, so it is not
   too heavy any more, but the assumed CG (over the main wing) is 0.48 m ahead of where they hold it up. With the
   CG 0.25 m ahead it rests at −4.6°; over the pontoon centre at +1.6°. A foiler wants the CG near the main wing; a
   floating boat wants it over its buoyancy: the layout puts those 0.48 m apart.
2. **Under power it floats level but stalls at 2.5 m/s** with full thrust (1000 N), pontoons 0.13 m deep and the
   cockpit box ploughing (the hull drag hump plus the box's own drag).
3. **Even ignoring the hull, the wing is too small/too shallow at 0°.** Level trim needs about 15.5 m/s with the
   flap inside ±15° (surface effect at 1.5 chords costs about 35 % of the lift). At a 4–5° mount it would trim from
   about 8.5 m/s, but item 2 still stops it at 2.6 m/s.
4. **Roll control is weak.** The flaps are 35 % of the span and next to the centreline.

With the box **off** (Ø 0.367 m pontoons): at rest it floats half under but 11.5° nose-down (the same CG
offset); under full thrust it runs level but stalls at 3.8 m/s, still far short of the 8 m/s cruise.

Items 1–2 hinge on the assumed mass, CG and the placeholder hull drag: get the assembly's mass properties
(SolidWorks → Tools → Evaluate → Mass Properties) before drawing conclusions from them.

---

## Integration, Default Boot & Verified 5° Flight / Turn Findings (2026-11-25)

### What Was Changed From `origin/jacob/cad-boat`
- **Boot-up default (`cad`):** Switched the default preset to `cad` across [viewer.py](../hydrofoil/viewer.py) (`--preset cad`), [sim_server.py](../hydrofoil/sim_server.py), [run_sim.bat](../run_sim.bat), [run_sim.sh](../run_sim.sh), and added `--preset` support to [wave_study.py](../tools/wave_study.py).
- **Physics & geometry fixes:**
  - Replaced the single scaled front strut with true dual front struts (`main_strut_l` and `main_strut_r` at `±main_strut_y`) in [vehicle.py](../hydrofoil/vehicle.py) so oblique waves, breach/ventilation, sideslip drag, and differential yaw damping $M_z$ act independently on port and starboard masts.
  - Replaced mutable `self.flap_dy` assignment in `Foil._eval_force_and_deriv` ([foil.py](../hydrofoil/forces/foil.py)) with a thread-safe `_apply_flap_dy` keyword flag.
  - Added `HullHydrostatics.wetted_draft()`, optional `rot_damping` in `box_drag()`, and fixed a trailing parenthesis syntax error in `buoyancy()` ([hull.py](../hydrofoil/forces/hull.py)); added piecewise pontoon+box static equilibrium in `rest_state()` ([vehicle.py](../hydrofoil/vehicle.py)) and placed rider deck height above buoyant cockpit boxes in [cad_geometry.py](../hydrofoil/cad_geometry.py).
- **Actuation, allocation, estimation & supervisor fixes:**
  - Added explicit `"main_flap_chord": 0.100` to `CAD_PARAMS` ([params.py](../hydrofoil/params.py)) and wired it into [loads.py](../hydrofoil/actuation/loads.py) and [consistency.py](../hydrofoil/consistency.py) (`flap_inertia` check).
  - Weighted `ControlAllocator` roll error using the effective flap arm (`main_flap_center_y or main_lateral_offset`) and exposed `cp_min_max` and `hc_min` telemetry ([allocator.py](../hydrofoil/control/allocator.py)); updated turn-envelope lens clearance in [supervisor.py](../hydrofoil/control/supervisor.py).
  - Updated `KalmanEstimator.__init__` ([kalman.py](../hydrofoil/estimation/kalman.py)) to skip untrimmable speeds rather than aborting when at least one trim speed exists, with clean `HeaveFilter`/PID fallback when `estimator` or `lqr` is unavailable.
  - Fixed `set_main_incidence()` / `_trim_ff()` analytical feedforward shift in [vehicle.py](../hydrofoil/vehicle.py) and upgraded live `main_incidence_deg` background rebuilds in [viewer.py](../hydrofoil/viewer.py) to re-derive both `LQRController` and `KalmanEstimator` with bumpless handover.
- **Design scaling, tuning & dashboard UI:**
  - Moved `resolve_cad_hull()` before mass/inertia scaling in [design.py](../hydrofoil/design.py), preserved explicit `hull_beam`/`max_draft` overrides, proportionally scaled `main_lateral_offset` and `main_flap_center_y` with span edits, and allowed `cockpit_buoyancy` across `design_info`, `design_preview`, and `tune_start` ([sim_server.py](../hydrofoil/sim_server.py), plus form-preserving `/api/load_cad`).
  - Made hump/turn envelope handling in [scoring.py](../hydrofoil/tuning/scoring.py) and [classical.py](../hydrofoil/tuning/classical.py) safe for CAD runs.
  - Upgraded [page.html](../hydrofoil/assets/launcher/page.html) with a split **Side Elevation (`#side`) + Top-Down Turning & Oblique Waves (`#topView`)** hero view, secondary telemetry strip (`#gd` depth $h/c$, `#gcav` cavitation $\sigma$ vs $|C_{p,\min}|$, `#gwav` encounter period $T_e / \mu$, `#gnav` heading $\psi$ / sideslip $\beta$), Boat Editor **Cockpit box buoyancy** toggle, and `cockpit_box` planform preview.

### What Worked vs. What Didn't Work (Verified Results)
- **Worked:**
  - `pytest tests/ -q` passes all **`161/161`** tests (including `13` tests in [test_cad_preset.py](../tests/test_cad_preset.py)).
  - `python -m hydrofoil.consistency` passes (`0 FAIL`) across both `model` and `manned` presets; `HTMLParser` and `node --check` pass cleanly on [page.html](../hydrofoil/assets/launcher/page.html).
  - `CAD_PARAMS` at `0°` under `sensors="noisy"` & `controller="auto"` initializes and steps finitely without crashing.
  - With `main_incidence_deg=5.0` and `cockpit_buoyancy=False` (`controller="auto"`, `supervisor=True`, calm water), the CAD boat takes off cleanly (`t_foil = 26.37 s`, `u_end = 8.10 m/s`, `0` faults; slower than the pre-fix `17.84 s` single-strut run due to dual-strut drag).
  - Routing config changes through `apply_design(..., {"main_incidence_deg": 5.0, "cockpit_buoyancy": False})` reschedules `lqr_speeds` from `(15.5..16.0)` to `(8.0, 8.75, 9.5, 10.25, 11.0)` and builds `LQRController` cleanly.
  - Moving `main_flap_center_y` outboard to `0.50 m` (at `5°`, box OFF, `8.5 m/s`, `apply_design()`) holds a steady `8°` turn for `10.0 s` (`7.16°` roll, `2.18°/s` yaw rate, `0` faults) with zero gain retuning.
- **Didn't Work / Physical & Configuration Limits:**
  - Raw `0°` `CAD_PARAMS` still cannot trim at `8.0 m/s` (minimum trim speed `~15.5 m/s`), so `python -m hydrofoil.consistency --preset all` fails only on `cad` `0°` trim at `8.0 m/s`. At rest it noses down (`0.48 m` CG-to-buoyancy offset), and under placeholder `1000 N` thrust it stalls in displacement at `~2.5 m/s` (box ON) or `~3.8 m/s` (box OFF).
  - With the measured inboard flaps (`main_flap_center_y = 0.12 m`) at `5°` box OFF and `8.5 m/s`, commanding `3°–8°` banked turns still overbanks to `25.5°` (`attitude` fault) in `2.18–2.74 s` because free-surface asymmetry and dual-strut sideslip roll overpower the cavitation-limited inboard flap torque.
  - Overriding `main_incidence_deg=5.0` directly on a raw copy of `CAD_PARAMS` without `apply_design()` leaves `0°` `lqr_speeds = (15.5..16.0)`, causing LQR initialization to fail (falling back to PID) because a `5°` wing cannot trim inside flap limits at `15.5–16.0 m/s`. Route static edits through `apply_design()` or use the viewer's live mounting-angle slider.

### What Is Still Missing
- **SolidWorks Mass Properties:** True dry + rider mass, 3D CG coordinates, and moments of inertia (still using `275 kg`, CG over main wing).
- **CFD, tow-tank & bench data:** Validated 2D/3D foil & flap polars, hinge moments, cavitation bucket, bare-hull & cockpit-box drag curves (replacing `C_D = 1.0`), and real motor/prop thrust curves (replacing `1000 N`) — swap instructions catalogued in [22-assumptions-and-provenance.md](22-assumptions-and-provenance.md) §13.
- **CAD hardware layout decisions:** Choosing a `4–5°` main-wing incidence (or larger/deeper wing), moving flaps outboard toward `~0.50 m` centroid for roll authority, and reconciling the `0.48 m` CG vs. pontoon buoyancy offset.
- **Live interactive GUI smoke-test:** Running [run_sim.bat](../run_sim.bat) / [run_sim.sh](../run_sim.sh) options `[1]` and `[2]` interactively to inspect the 3D MuJoCo window, split `#side` + `#topView` dashboard, and Boat Editor toggles visually (headless CI tests only covered markup and JS syntax).
- **CAD-specific autotuning:** Running `python -m hydrofoil.tuning.autotune --preset cad --target auto` after physical mass, incidence, and flap geometry are finalized (`CAD_PARAMS` currently inherits `MANNED_PARAMS` PID/LQR gains).

### What Needs To Be Done Next
1. **Live GUI smoke-test:** Run [run_sim.bat](../run_sim.bat) `[1]` and `[2]`, inspect the CAD boat in MuJoCo (cockpit buoyancy ON/OFF), move the `Main wing mounting angle` slider (`0° → 5°`), and check `#side` + `#topView` + secondary gauges in the browser.
2. **Export SolidWorks Mass Properties:** Replace the placeholder mass, 3D CG, and inertia tensor in `CAD_PARAMS` ([params.py](../hydrofoil/params.py)).
3. **Swap in CFD & drivetrain data:** Follow [22-assumptions-and-provenance.md](22-assumptions-and-provenance.md) §13 to populate foil polars, hull/cockpit drag curves, and propulsion thrust curves.
4. **Update physical CAD geometry:** Apply the chosen `4–5°` mounting offset, outboard flap placement (`~0.50 m`), and CG/pontoon alignment.
5. **Re-run verification & tuning:** Verify `python -m hydrofoil.consistency --preset all`, run `python -m hydrofoil.tuning.autotune --preset cad --target auto`, and re-evaluate directional waves via `python tools/wave_study.py --preset cad`.
