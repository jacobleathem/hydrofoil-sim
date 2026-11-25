# 12 — Progress Log

Append new entries at the bottom. Keep them dated.

## Timeline reconstructed from the repository

| Date | Source | Event |
|---|---|---|
| 2026-09-10 | Unity screenshot file names | Visual-checks of the Unity scene (6 screenshots, 15:25–15:45) |
| 2026-09-17 | `hydrofoil/PROJECT_STATE.md` | Snapshot written to orient LQR/MPC work: PID done, roll unlocked, safety net, UDP bridge |
| 2026-10-24 | commit `3170acd` (jleathem) | Initial commit: Python sim + Unity visualiser |
| 2026-10-25 | commit `cc314c2` (jleathem) | `.gitignore` fix so the foil polar data file is tracked |
| 2026-11-04 | commit `2af5157` (jacobleathem) | README update ("missing file" note for the 709 MB hull model) |

Capabilities present in the code at this commit that post-date the 2026-09-17 snapshot: diagonal added mass,
flap effectiveness (0.21/1.0), finite-span AR/(AR+2), free-surface lift loss, induced drag with Oswald e, actuator model
(delay + rate limit + lag), pilot bank/speed uplink, and the unit tests.

## Milestone status

| Milestone | Status |
|---|---|
| 6-DOF rigid body + kinematics | done |
| Foil model with corrections | done (placeholders for free-surface; single polar) |
| Hull buoyancy / takeoff | done (linear wall-sided) |
| Waves | regular Airy only |
| PID (heave/pitch/roll, three parallel loops) + surge | done, **not retuned**; roll loop unstable and rad/deg unit bug (doc 13) |
| Actuator model | done, placeholder numbers |
| Dashboard | done |
| Unity bridge + pilot uplink | done, then **removed** 2026-11-04; replaced by MuJoCo viewer (branch `mujoco-viewer`, not yet merged/pushed) |
| Safety net | done |
| Unit tests | partial (forces + actuator only) |
| Linearisation (A, B) | not started |
| LQR / MPC | not started |
| Sensors / estimator | empty packages |
| Hardware / firmware | not started (deprioritised) |
| Measured vehicle parameters | none |

## Log

### 2026-11-04 — Documentation pass
- Cloned `https://github.com/jacobleathem/hydrofoil-sim` (git was not initialised in `D:\Coding_Projects\Hydrofoil Sim`; the folder was empty so it was cloned in place, equivalent to the requested "pull").
- Read every Python module, both test files, the polar, and all Unity C# scripts.
- Created `docs/` (this folder), 12 documents.
- Attempted to run tests/headless probe; blocked by the permission check, so no runtime verification was done.
- Findings recorded in doc 11 (stale `PROJECT_STATE.md`, missing `plotly` in requirements, surge-D sign, etc.).
- No source files were modified.

### 2026-11-04 — Math audit, MuJoCo viewer
- Ran the code (it had been blocked earlier): 23/23 tests pass; numerical audit in doc 13.
- Findings: rad/deg gain mismatch in roll/pitch PIDs; roll loop unstable at current gains; waves cannot excite roll; no feed-forward (heave settles in 60-90 s); Munk moment missing.
- New branch `mujoco-viewer`: removed `unity-visualizer/`, `stream.py`, `control_uplink.py`, `sim_realtime.py`; added `hydrofoil/viewer.py` (MuJoCo, render-only) and `hydrofoil/params.py`; requirements now list `mujoco`, `plotly`, `pytest`.
- Physics files were not changed. Docs updated across 01, 04, 05, 06, 09, 10, 11, 12 with the findings; new doc 13.
- Follow-ups: fix PID units and retune; add committed regression tests for bank/disturbance; merge or push `mujoco-viewer`.

### 2026-11-04 — Method/unit/math fix pass and retune
- Fixed (doc 14): PID rad/deg mismatch; anti-windup; surge Kd sign; actuator delay in seconds; Munk (added-mass Coriolis); consistent finite-span/Oswald; spanwise flow in foil force; hull draft vs attitude and buoyancy moment arm; per-foil wave sampling; wave direction + deep-water dispersion; stale reporting; `sim.py` rebuilt on `VehiclePhysics`; shared constants.
- Added: vertical struts (side force; placeholder chords), `WaveSample`, `constants.py`, `tests/test_method_fixes.py` (19 new tests), performance work (`np.cross`, closed-form rotation) ≈ 8.6 → 3.6 ms/step.
- Retuned: heave 40/6/10, pitch 1.0/0/0.1, roll 2.0/0/0.2 (was 15/1/5, 2/0/2, 3.5/0/2.5 with the wrong units). Takeoff 9.9 s → 2.9 s; 2 cm height settling 62 s → ~17 s; roll kick rejected in 0.7 s; 8° bank tracked; oblique-wave roll ≤ 1.3°.
- Verified: 42/42 tests; sweep results and robustness to doubled actuator delay/lag in doc 14 §H. Not re-run: randomised combined-input fuzz.
- Follow-ups: trim feed-forward; wing–tail downwash; flap section moment; real strut/hull/actuator measurements; merge or push `mujoco-viewer`.

### 2026-11-04 — Buoyancy cross-reference
- Compared the hull against Fossen ch.4 metacentric theory: roll stiffness was 23 % low and pitch 14× too low (no waterplane-inertia term). Replaced the lumped model with `HullHydrostatics` (footprint grid, full attitude kinematics, per-cell wave elevation); stiffness now within 1 % of theory (doc 15, E20).
- Added placeholder hull roll/pitch damping so the correctly stiff hull does not ring at rest; new params `hull_length`, `hull_beam`, `hull_x`, `hull_rot_damping` (`waterplane_area` now derived).
- 46 tests pass; foilborne metrics unchanged; gains did not need retuning. Everything remains local (no push), commits on branch `mujoco-viewer`.

### 2026-11-04 — Review packaging
- Added `tools/audit.py` (portable, reproducible: hydrostatics, linearize, checks, evaluate, sweep-angles, sweep-heave) and `docs/16-review-guide.md` (conventions, equations→code map, assumption ledger, evidence matrix, open questions, unexplained observations).
- Post-fix cruise values re-measured: trim flap 15.1°, thrust 17.1 N, pitch −0.30°; docs updated (the as-found doc 13 keeps the old figures).
- Open-loop poles of the corrected plant: +0.25 s⁻¹ lateral spiral-type, +0.029 s⁻¹ undiagnosed slow x-mode.

### 2026-11-05 — One app (MuJoCo) and the actuator drive model
- Folded the browser dashboard into the MuJoCo app: sliders (zero-gain actuators in the Control panel) for references, waves, live PID gains and drive settings; rolling plots via `set_figures`; keys and sliders share one value. Removed `dashboard.py`, fastapi/uvicorn, and legacy IDE launch config (no web server left).
- Added `hydrofoil/actuation/`: thin-airfoil hinge coefficients (validated vs 2π, c/4, Glauert τ), quasi-steady Theodorsen damping and added inertia, moteus r4.11 + BLDC + gearbox electrical model, per-drive loads after the mixer. `tools/actuator_budget.py` runs 5 scenarios and a gear-ratio sweep.
- Findings (doc 17): the trimmed hinge moment is ≈ constant with speed (≈ 2.1–2.4 N m per main flap) while the moment per degree grows with V². 20:1 sits at the peak-current optimum, with ≈ 4 A rms / 19 A peak against 12 A / 100 A. The drive can make 29 N m, 4.5× the worst water load, so cap torque in firmware. The mixer sits ahead of the limits (a flap reached 35°). Flap effectiveness 0.21 conflicts with thin-airfoil 0.75.
- All local, no push.

### 2026-11-05 — Drives closed into the loop
- `actuation/servo.py`: per physical drive moteus PD + current/back-EMF/firmware-cap limits + gearbox + rotor/water/structure inertia + hinge moment, 1 ms sub-steps, mechanical stops; mixer moved ahead of the limits. Default `actuator_model = "servo"`; legacy `"kinematic"` kept. Optional q gain schedule (off).
- Flight metrics essentially unchanged with the gains as they were (doc 17 §7); a 4× weaker drive degrades gracefully; the limp-drive test confirms the fail-safe. The app shows the drives' own telemetry. 57 tests pass. Local only.

### 2026-11-06 — Audit verified against the original code
- New doc 18 and `tools/verify_original_audit.py`: every claim in docs 13/14/17 re-checked on commit 841d501 (original code) in an isolated interpreter.
- Confirmed: 7 errors (E1, E2, E3, E12, E13, E15, F1), 2 omissions (E7, E10), 2 latent errors (E4, E5), 4 minor (E8, E9, E11, E16); 5 items were not errors or not in the original.
- My claims corrected: E7 "energy not conserved" (wrong: νᵀCν = 0), E14 (AiryWave had `direction`; the original "Ki 0.05 blew up" comment is not reproducible), doc 13 W7 (no common limit; the opposite problem), the flap-effectiveness "conflict" (inboard half-span), E2's before-figures (8.5 s / 70.6 s).
- Found and fixed my own error: the hinge model used the full 0.6 m span for a 0.3 m inboard flap (`main_flap_span_frac`). Hinge loads and currents halved; the current-optimal ratio is ≈ 15:1 (10–20 flat); doc 17 refreshed; the viewer draws the flap inboard. 57 tests pass. Local only.

### 2026-11-09 — Trim feed-forward (roadmap Phase 0 item 2)
- `vehicle._trim_ff`: δ_ff = `trim_ff_deg` (15.1°) · (`trim_ff_speed` (4 m/s) / u)², clamped ±20°, added to the heave PID output; the heave PID's clamp is shifted by the FF so anti-windup judges the total flap command. `trim_ff_deg = 0` disables it. New params in `params.py`; test added.
- Verified (`tools/audit.py evaluate`, calm water): height-step 2 cm settling 8.6 s → 2.7 s; settling after takeoff 17.7 s → 12.2 s; takeoff 2.95 s, bank/roll/pitch metrics unchanged. 58 tests pass. Without the clamp shift the post-takeoff settling was worse (32 s: integrator wound up on top of the FF). Not re-run: wave cases, the doubled-delay robustness block.
- Follow-ups: the trim angle is a measured constant, not computed from W/(2qA·CLα·k); the remaining 12 s post-takeoff settle is the speed ramp (surge loop).

### 2026-11-09 — Wing–tail downwash and flap section moment (roadmap Phase 0 item 4, rest)
- Flap moment: `thin_airfoil_coeffs` now returns `cm_alpha` (≈ 0, AC at c/4) and `cm_delta` (−0.588/rad for a 40 % flap, equals Glauert −½ sinθ(1−cosθ)). `Foil(flap_cm_delta=…)` adds q·A·c·Cm_δ·δ to the pitch moment of each main half; scaled by `main_flap_span_frac` (0.5) and `flap_cm_factor` (0.8, same viscous placeholder as the hinge model). Switch: `flap_moment`.
- Downwash: tail AoA reduced by ε = `downwash_factor` · 2·CL_wing / (π·AR_wing), AR on the full 1.2 m span, using the main foils' corrected CL (`Foil.last_CL`). Quasi-steady: NO transport lag (l/V ≈ 0.3 s), and no strut/hull wake effect.
- Effect: cruise trim flap 15.1° → 16.1° (so `trim_ff_deg` 15.1 → 16.1), elevator −0.30° → −0.36°, thrust 17.1 → 18.0 N. Calm-water metrics: takeoff 2.95 → 3.10 s, post-takeoff 2 cm settle 12.2 → 14.6 s, height step 2.7 → 3.6 s, bank/roll/pitch unchanged, no touchdowns, wave roll unchanged.
- 60 tests pass (2 new: Glauert match, downwash/moment signs). Note: the "delay and lag doubled" block of `audit.py evaluate` printed identical numbers to the nominal block, so it does not appear to alter the servo-model path — not investigated.
- Follow-ups: transport-lagged downwash (affects short-period damping), retune if the lag is added, wing-wake on the elevator strut.

### 2026-11-09 — Downwash transport lag and drive-load consistency
- Tail downwash is now delayed by l/u (wing-to-tail distance / forward speed) from a history buffer in `VehiclePhysics` (`downwash_lag`, default on). The elevator drive's hinge-load flow (`actuation/loads.flow_models`) now subtracts the same downwash, so elevator hinge moment/current are consistent with the force model.
- Flight metrics barely change with the lag (z-settle 14.7 → 14.6 s, height step 3.58 → 3.55 s). `physics_rhs` called directly (audit linearize) sees the frozen last delayed value, so the linearised model omits the downwash dynamics.
- Drive budget re-run (`actuator_budget.py --model servo`, N = 20, cruise 4 and 6 m/s head seas), new model vs old (no flap moment/downwash, trim 15.1°): peak current 13.7 A and peak voltage 5.4 V unchanged (set by the takeoff step to the 20° stop); rms current 1.9 → 2.0 A (4 m/s) and 2.0 → 2.2 A (6 m/s); peak output torque 1.40 → 1.47 and 1.83 → 1.89 N m; elevator unchanged. The flap section moment reaches the drives only through the achieved flap angle; hinge loads scale with V². All within the moteus/BLDC limits. Not swept: other gear ratios with the new model.

### 2026-11-10 — 40 V / 25 A supply and rail-current check
- Supply is now 40 V from battery through the PDB to the motors, limited to 25 A (user-stated): `v_bus` 24 → 40 (params, `ControllerSpec`, `actuator_budget.py --vbus`), new `supply_i_max` = 25 A. The budget tool sums the three drives' bus power (time-aligned) into a rail current and prints it with a rough propulsion estimate thrust·u/η (η = 0.35 PLACEHOLDER) against the 25 A limit.
- Results (servo model, N = 20): actuators peak 3.1 A (mean 0.13 A); with propulsion ≈ 8.4 A at 4 m/s and 15.5 A at 6 m/s the rail totals 11.5 A / 18.6 A vs 25 A, all scenarios OK. Gear sweep (worst rail for actuators only): N = 5 → 2.8 A, 20 → 3.1 A, 40 → 11.4 A; peak phase current 13.8 A at N = 15, 14.1 A at 20, 27 A at N = 5 and 40; rate limit at N = 20 rises 2138 → 3564 deg/s. 60 tests pass.
- Caveats: the propulsion current is a guess (real motor/ESC efficiency, takeoff thrust, regen not modelled); the rail limit is checked in the budget tool, not enforced in the flight loop; actuator regen is not netted against propulsion.
- Not done: flight metrics re-measured at 40 V (tests pass; `audit.py evaluate` not re-run).

### 2026-11-10 — Robustness block of the audit fixed
- `audit.py evaluate`'s "delay and lag doubled" block printed the nominal numbers because it only set `actuator_params`, which only the kinematic actuator reads (the default is now the servo). It now flies the kinematic actuator nominal (20 ms delay/lag) and doubled (40 ms).
- Results: doubled delay/lag changes almost nothing (foilborne 3.05 → 3.06 s, 2 cm settle 18.84 → 18.87 s, roll 0.69 → 0.64 s, pitch 0.60 → 0.57 s, height step 3.85 → 3.83 s). Servo model for reference: 3.11 s / 14.6 s / 0.70 s / 0.62 s / 3.55 s. Wave cases not re-run in this block.
- Not done: flight metrics at 40 V were not isolated from this run's servo numbers beyond the above (they match the 24 V figures, so 40 V made no visible difference to flight).

### 2026-11-11 — Linearisation, LQR, sensors and Kalman estimator
- New: `linearize.py` (trim + Jacobian), `control/lqr.py` (scheduled MIMO LQR with integral action, PID takeoff with bumpless latched handover), `sensors/models.py`, `estimation/kalman.py`; params `controller` ("pid" default / "lqr") and `sensors` ("ideal" default / "noisy"). Doc 19.
- Verified: 67 tests pass (7 new). LQR vs PID on the audit scenarios: height step 3.55 → 1.18 s, pitch kick 0.62 → 0.21 s, wave roll (beam/oblique) 0.99/1.30° → 0.39/0.26°, bank height loss 17.7 → 4.8 mm, roll kick 0.70 → 0.63 s; bank settle 0.56 → 0.61 s. Roll weight tuned 2° → 0.8° (roll kick 1.38 → 0.62 s). With noisy sensors the boat still flies (3.3 mm rms height).
- Defaults unchanged (PID, ideal sensors). Trim finding: the PID's 16.1° flap at 4 m/s is the closed-loop value at θ = −0.37°; level trim is 14.1°.
- Full noisy-sensor evaluate (doc 19): mostly mild degradation, but the 3° pitch kick settles in 4.36 s vs 0.21 s with ideal sensors (undiagnosed); bank height loss 4.8 → 15.3 mm. Not done: LQR in the viewer UI; drive dynamics in the LQR model; rail-current enforcement.

### 2026-11-11 — Estimator retune, PID vs LQR decision, LQR default
- Pitch-kick slowdown under noisy sensors: Kalman process noise on z/φ/θ was far too small (1e-4); retuned to 3e-3 / 5e-3 (`estimation/kalman.py`, override `kf_process`). Kick 4.4 s → 0.3 s with elevator jitter 0.5°.
- Compared PID and LQR with ideal and noisy sensors, and on four perturbed plants with the LQR designed on the nominal model (doc 19). LQR equal or better almost everywhere, much better on perturbed plants (PID never settles height at mass ×1.3 or tail arm −25 %).
- `controller` default PID → LQR (PID selectable; non-servo actuator model falls back to PID with a warning). 67 tests pass with the new default. Vehicle construction ≈ 3.7 s.
- Not covered: larger/combined model errors, noise plus model error, thrust/surge under the LQR, viewer UI for the controller choice.

### 2026-11-15 — Delay-aware LQR, main-foil incidence
- Changed: `control/lqr.py` (drive model + command-delay chain + command-rate penalty in a discrete design; controller feeds back drive angles/rates and past commands), `vehicle.py` (incidence on the main foil, trim feed-forward formula, drive state passed to the LQR), `params.py`, tests. Doc 19, last section.
- Why: the old LQR limit-cycled once the command delay reached 30 ms (2.2° rms flap in calm water); the flap trimmed at 14.1° of ±20° and clipped in 10 cm waves.
- Verified: 71 tests pass (3 new: 80 ms delay with nominal gains stays quiet, design stable at every scheduled speed, trim flap near neutral). Delay/drive mismatch sweep, wave sweep and `audit.py evaluate` in doc 19.
- Parameter changes: `main_incidence_deg` new = 2.5; `trim_ff_deg` 14.1 → 1.63; `lqr_delay_s` new = None; LQR weights `du_main/du_elev/du_roll` new = 250 °/s.
- Follow-ups: true head seas in `evaluate`; height sensor relative to the local surface; Kalman update only on fresh samples; wave disturbance state; kinematic-actuator PID slowdown not investigated.

### 2026-11-15 — Realistic height sensor, heave filter, CG offset, true head seas
- Changed: `sensors/models.py` (height relative to the local wave surface, `fresh` flags, accelerometer), `estimation/kalman.py` (`HeaveFilter`), `vehicle.py` (heave filter feeds height and heave velocity; `set_cg_offset`), `params.py` (`cg_offset`), `tools/audit.py` (following / head / beam / oblique with height rms). Doc 19, last section.
- Why: the height sensor measured depth relative to still water (not measurable); the Kalman filter reused held samples; "head" in the audit was a following sea; rider weight and CG shifts were untested.
- Verified: 74 tests pass (3 new). Noisy-sensor wave sweep, `evaluate` with ideal and noisy sensors, mass/CG sweep: doc 19.
- Findings: without a wave state the boat chases waves (flaps ±20°, 17 A peaks). Following and quartering seas remain weak with noisy sensors (59 mm rms at 5 cm amplitude). A 100 kg all-up weight cannot take off at 4 m/s / 50 N on these foils.
- Follow-ups: following-sea height estimation; irregular seas; sensor position and tilt; re-size foils/thrust for the real all-up weight.

### 2026-11-15 — Assessment, sensor list and carried-forward findings noted
- Changed: doc 19, end ("Is it good enough?", "Sensors required on the real boat", "Other findings to carry forward").
- Open decision: real all-up weight (boat plus rider); it drives foil, speed and thrust sizing.

### 2026-11-15 — Manned sizing (250–300 kg)
- Changed: `params.py` (`MANNED_PARAMS`), `vehicle.py` (LQR and Kalman designed on the nominal CG; `lqr_speeds`, `lqr_engage_speed`, `est_engage_speed`, `kf_heave` params), `linearize.py` (thrust step clip), `tests/test_manned.py` (4 new). Doc 19, last section.
- Why: real all-up weight is about 250 kg, up to 300 kg; the 25 kg sizing cannot take off at that weight.
- Result: 0.62 m² main foil at 3° incidence, 8 m/s cruise, hull clears at 5.6–6.3 m/s, about 190 N cruise thrust. Flies at 250 and 300 kg, CG ±25 cm fore/aft and 20 cm lateral, in-flight rider shifts absorbed.
- Findings: the drive torque cap must rise from 10 to 28 N·m (it rolled over with a 10 cm lateral CG at 10 N·m). Following / quartering seas with noisy sensors are 69–118 mm rms; needs GNSS vertical velocity or a better IMU.

### 2026-11-15 — Manned sizing verified; scale issues and failure cases
- Changed: `control/lqr.py` (`hinge_stiffness`; drive model with the restoring hinge moment). Doc 19, "Verification", "What changes with weight", "Failure cases".
- Verified: hand trim check, dt 0.005, three sensor seeds, heave-damping and roll-added-inertia sensitivity; 78 tests pass. Corrected takeoff speed to 5.6–6.3 m/s.
- Findings: hinge stiffness is 40–60 % of the servo kp (was 2 %); delay margin halves to 80 ms; unstable mode is 2.5x faster. A main flap jammed off trim capsizes the boat and a limp elevator drives the nose in within about 1 s. No fault handling exists.

### 2026-11-15 — Operating protocol (flight-mode supervisor)
- Changed: new `control/supervisor.py` (modes DISPLACEMENT / TAKEOFF / FOILING / LANDING / EMERGENCY_LANDING, speed and height envelope, drive and sensor fault detection, jammed-flap mixer reconfiguration); `vehicle.py` hooks (`params["supervisor"]`, off by default; `_last_target`, `_last_flows`); `tests/test_supervisor.py` (4). Doc 19, "Operating protocol".
- Why: safe takeoff, cruise and return to displacement at a pilot-chosen speed, for the foiling energy saving (less than half the power above 4.5 m/s).
- Result: full cycle lands at 0.1 m/s sink; flap jams that capsized the boat now land level (roll ≤ 3°); elevator loss nose-down −13° → −4° (−1° with a fail-safe drive).
- Follow-ups: fail-safe elevator drive (hardware); takeoff in waves; turns; irregular seas.

### 2026-11-15 — Autotuner, flight observer, boat editor; manned gains retuned
- Added: `hydrofoil/tuning/` (scoring over 5 scenarios incl. noisy sensors; staged autotuner: relay test + Ziegler-Nichols / Tyreus-Luyben, Bryson + re-weighting, cross-entropy search), `hydrofoil/monitor.py` (flight observer), `hydrofoil/design.py` (boat editor layer), `tests/test_tools.py` (6). Viewer: `--overrides`, observer in the instrument strip, strut/foil geometry fixed. Doc 19, last section.
- Changed: `MANNED_PARAMS` PID gains and LQR weights from the autotuner; `default_payloads` (75 kg rider).
- Results: LQR head / following sea 27 / 36 to 18 / 18 mm rms; PID following sea 95 to 64 mm. LQR delay margin at cruise 80 to 60 ms.
- Fixed: LQR handover now blends PID to LQR over 0.5 s (the tuned weights gave a 14 deg elevator step); supervisor 'stuck' detection needs the error to persist for its window (it fired on that step).
- Findings: tuning on perfect sensors over-drives the flaps; classical rules from the relay test are too aggressive for a noisy height signal; the PID cannot reach the LQR.

### 2026-11-16 — Flap geometry and drive design study (manned)
- Changed: `forces/foil.py` (`main_flap_effectiveness`: lift effectiveness scales with flap chord and span, 0.21 at the 40 % / half-span reference), `vehicle.py` (uses it; `main_flap_effectiveness` param override), `actuation/hinge.py` and `loads.py` (set-back hinge, `main_flap_setback`, default 0), new `tools/flap_design.py`, `tests/test_actuation.py` (3 new). New doc 20.
- Why: the flap's lift effectiveness was a constant while its hinge moment followed the geometry, so a geometry change was inconsistent; and the question was which flap and gear ratio the real drive needs, and what mechanism has to be built.
- Verified: the defaults are unchanged (effectiveness exactly 0.21 on both presets, test). Static sweep, plus 10 closed-loop manned runs (LQR, servo drives, takeoff / head / following sea): doc 20.
- Findings:
  - The current 40 % flap needs 44 N·m at 9.5 m/s against the drive's 28.9, and 17–20 A to hold trim against the 12 A rating.
  - A 25 % chord, full-span flap with a 20 % set-back hinge needs 4× less moment and 3× less current, with 3 mm less height error in waves.
  - Gear ratio is set by torque and reflected inertia, never by speed; keep 20:1.
  - The 8 mm output shaft placeholder is overstressed 4× (needs 12.8 mm or more).
  - A pushrod or torque tube down the 1.65 m strut buckles or is too soft, so the drives belong in a nacelle at the strut–foil junction.
  - Takeoff peaks (48 A, 17–21 A rail) come from the takeoff command step to the 20° limit, not the water.
- Parameter changes: none to defaults (`main_flap_setback` = 0 and `main_flap_effectiveness` = derived are new).
- Follow-ups: slew-limit the takeoff flap command; adopt the balanced flap geometry in `MANNED_PARAMS` and retune; model plain-flap separation above 15–20°, gearbox backlash and flap torsion; bench or CFD check of the set-back.

### 2026-11-16 (later) — Corrections, backlash and twist, co-dependency checker, balanced flap adopted
- Corrected doc 20's first version (its section 1 lists each error):
  - "the 40 % flap does not fit the drive" was a static assumption; in flight it peaks at 17.3 N·m at 9.5 m/s;
  - the 12 A controller "limit" was the no-heat-sink figure;
  - the "hinge moment under 14 N·m" claim was false for the old flap (21.1 N·m);
  - the rail-peak range was wrong, and so was the loss factor.
- Changed:
  - `actuation/servo.py`: compliant drive train with backlash, gear stiffness, spanwise twist, and motor / output / dual encoder.
  - `vehicle.py`: twist stiffness from GJ, and a command slew limit.
  - `actuation/hinge.py`: `flap_inertia`.
  - new `hydrofoil/consistency.py`.
  - `actuation/drive.py`: shaft 8 → 13 mm.
  - `design.py`: the trim re-derives on the new flap keys.
  - `tools/flap_design.py`: 9.5 m/s head sea, compliance and slew options.
  - `MANNED_PARAMS`: balanced flap and the compliant drive.
  - tests: new `tests/test_drive_compliance.py` (8); `test_actuation` and `test_tools` now derive geometry-dependent expectations.
- Why: the user asked that every number and co-dependency be checked so these errors cannot recur, that backlash and twist be modelled, and that the balanced flap and takeoff slew limit be adopted. The controller can run more than 12 A, and a firmware co-dependency limit is planned (doc 20 section 10).
- Verified: 99 tests pass, including the supervisor missions. The checker shows no FAIL on either preset. Closed-loop runs and drive hand checks are in doc 20.
- Bugs found and fixed:
  - the slew limiter let the first takeoff step through, and its test skipped that step;
  - a test hard-coded the old cruise trim;
  - output-only encoder feedback rang at takeoff, with 72 N·m through the gear (dual feedback fixes it).
- Parameter changes, `MANNED_PARAMS`:
  - flap 0.4 / 0.5 / 0 → 0.25 / 1.0 / 0.15;
  - `main_surface_inertia` 9e-3 → 2.7e-3;
  - `trim_ff_deg` 6.57 → 4.11;
  - new: `cmd_slew_deg_s` 150, backlash 0.25°, gear stiffness 4000 N·m/rad, encoder "dual", GJ 250 / 1000, `drive_i_cont` 20 A, `tuned_for`.
- Parameter changes, `DriveSpec.shaft_diameter`: 0.008 → 0.013.
- Gains: PID and LQR re-checked with the autotuner and kept. The PID retune stalled the supervised takeoff; the LQR retune was worse on noisy sensors.
- Results: takeoff peak 48 → 11 A, rail 17.6 → 1.1 A; head-sea hinge moment 7.65 → 2.6 N·m; current 9.8 → 3.4 A rms; height 18.2 → 15.2 mm.
- Findings: the balanced flap's near-zero hinge load lets the gear knock across its backlash twice per wave (needs a preloaded stage); the static twist and torque bounds are pessimistic.
- Follow-ups:
  - model the firmware co-dependency limit once its rules exist;
  - add a supervised takeoff to the autotuner;
  - choose gearbox, encoder and flap structure, then replace the placeholders;
  - model plain-flap separation and gear tooth impact.

### 2026-11-16 (later) — Supervised takeoff in the autotuner; manned PID re-tuned
- Changed:
  - `tuning/scoring.py`: new `supervised` scenario (a full supervised takeoff; an aborted takeoff is a failure) and `scenarios_for`.
  - `tuning/autotune.py`: uses the preset's scenario list.
  - `MANNED_PARAMS`: `supervisor_tuning` and the new PID gains.
  - `tests/test_tools.py`: the stalling gain set must fail the scenario.
- Why: the earlier PID search found gains that stall the supervised takeoff on the drag hump, because its takeoff ran without the supervisor.
- Result: PID 44.9 / 6.5 / 7.9 → 62.4 / 6.0 / 6.8 (heave); every scenario improves (doc 20 section 8).
- Verified: 100 tests pass, including the supervisor missions.

### 2026-11-16 (evening) — Safety, estimation, actuation, and polar bug-fix pass
- Changed:
  - `control/supervisor.py`: `_update_wave_estimate` now subtracts `heave.z` and updates only in `FOILING` (stops calm takeoff climbs from fabricating ~0.27 m wave estimates); `_detect` delays the reference target by `d.delay_steps`, detects frozen-but-fresh height samples and persistent `HeaveFilter` gate rejections, and tags `"limp"` vs `"jam"` drives; `mix()` tracks the live failed-drive angle `drives[side].delta`; `_fault_response` cuts thrust on `height_lost` and allows `EMERGENCY_LANDING -> DISPLACEMENT` exit once slowed below `v_disp_max`.
  - `estimation/kalman.py`: `KalmanEstimator` keeps its internal `z` state relative to its fixed linearisation depth datum `_z_trim0` so `flying_height` changes never perturb the model or inject fake innovations; `HeaveFilter` uses the symmetric Joseph covariance update and records `reject_events`.
  - `vehicle.py`, `control/pid.py`, `control/lqr.py`, `monitor.py`, `tools/phase_trials.py`: fixed missing `v cos(theta) sin(phi)` in `w`/`vz` conversions, removed oracle `v`/`w` below `est_engage_speed`, froze standby PID integrators when LQR is active, held LQR heave integration during `DISPLACEMENT` overrides, and synced `LQR.hist` to post-shaping/post-slew targets (`sync_sent`).
  - `actuation/servo.py`: unilateral gear-backlash flank contact (no tensile pulling across the gap), full substep `_peak_spring` tracking, per-step frozen flow linearization in `_update_compliant`, and scalar clipping speedup (1000 manned steps ~10.1 s → ~6.6 s).
  - `data/foil_tables/naca0009_xflr5_re2700000.dat` and `forces/foil.py`: mirrored symmetric negative side (−17.8°…−10.2°) and added smooth flat-plate post-stall blending in `generalized_force` beyond polar bounds (eliminates the reverse-flow snap at `V_n = 0, u_rel < 0`).
  - `viewer.py`, `tests/conftest.py`, `.gitignore`, `README.md`, `docs/11-known-issues.md`: preserved `v_bus` overrides, enabled bare `pytest` collection, ignored `phase_trials.jsonl`, synced `README.md` and `docs/11` C4.
- Verified: full `pytest -v` suite (existing 101 + 7 new regression tests = 108 tests) and `python -m hydrofoil.consistency` pass.
- Open situational model choices echoed for confirmation before pushing: continuous-wing `AR` (`3.93` vs `7.86`), Wadlin free-surface / ventilation model, unmodeled roll added inertia (`~0.8 * ixx`), hull heave added mass / drag hump, plain-flap separation past 15°, and wave-takeoff / oblique-turn clearance envelope.

### 2026-11-16 (late evening) — Added physics, dual height + GNSS fallback, autoscaling, viewer `--supervisor`, and `phase_trials.py` sweep (doc 21)
- Physics & hardware upgrades:
  - `dynamics/rigid_body.py`, `vehicle.py`: added analytical 2D strip `strip_added_mass_roll` + `added_inertia_roll` into `M[3,3]` and passive `C_A`, and precomputed `_M_inv`.
  - `forces/hull.py`, `forces/foil.py`, `vehicle.py`: added dimensionless displacement Froude drag hump peaking around `Fr_L ~ 0.48`, smooth plain-flap separation effectiveness roll-off + profile drag past `15°`, and per-foil shallow-breach ventilation hysteresis state (`_vent`).
  - `params.py`, `consistency.py`: upgraded `MANNED_PARAMS` to low-backlash precision planetary defaults (`backlash_deg=0.10`, `gear_stiffness=5000.0`), `shaft_diameter=0.015 m` (manned shaft shear `64.9 -> 42.3 MPa` vs `70.0 MPa` allowable), propulsion spool lag `prop_tau_s=0.12 s`, and power-capped thrust (`prop_power_max=6500.0 W`); added `dual_height`, `gnss_fallback`, and `gear_rated_torque` wiring.
- Estimation, redundancy & supervisor upgrades:
  - `sensors/models.py`, `estimation/kalman.py`, `vehicle.py`: added dual ranging channels (`height` + staggered `height_aux`) with health voting/outlier quarantine (preventing frozen samples from poisoning accelerometer bias `b_a`) and latency-compensated GNSS vertical-velocity (`gnss_vz`) `[vz, b_a]` fallback when both height channels fail.
  - `control/supervisor.py`: raised `takeoff_pitch_limit=14.0°` separately from `pitch_limit=10.0°` in `FOILING`, gated `TAKEOFF -> FOILING` inside `FOILING` attitude limits, added rate-compensated `(kd + B_hyd) / kp * delta_dot` FDI residual so healthy `150°/s` slews in `0.25 m` waves don't false-trip `elev_loss`, and added dynamic bank clearance bump `0.5 * hull_beam * sin|phi|`.
- Design autoscaling & viewer integration:
  - `design.py`: `apply_design` now auto-scales empty-boat dry inertias, wetted hull drag, critical hull damping ratio, flap structural inertia, composite `GJ`, output shaft diameter, thrust/power envelope, and first-order PID gains from physical dimensions/mass.
  - `viewer.py`: added `--supervisor`, `F` key / `toggle_foil` API, live supervisor status/alerts on HUD and HTTP telemetry.
- Sweep & docs: ran all 108 runs of `tools/phase_trials.py` (`92/108` pass; `pid, auto, lqr, lqr` optimal across calm/head/following seas) and documented in `docs/21-per-phase-controller-trials.md` (indexed in `docs/README.md`).

- Launchers & packaging: added `run_sim.bat` / `run_sim.sh`, promoted `local/sim_server.py` -> `hydrofoil/sim_server.py` (and `page.html` + web fonts to `hydrofoil/assets/launcher/`) with supervisor toggle while keeping `local/` gitignored for runtime logs/overrides, and wired `viewer.py` (`D` key) to auto-start `sim_server` on `127.0.0.1:8765` if not already running.

### 2026-11-18 — Three-tier code/logic audit, full 3D split-front-flap turning, and assumptions ledger (docs 22 & 23)
- Audit fixes (module, subgroup, and project level):
  - `waves/airy.py`: clamped `z_eff = min(z, 0.0)` in `orbital_velocity` (eliminated `12.3x` exponential blowup above SWL) and added positive `wavelength`/`period` validation.
  - `forces/foil.py`: eliminated emerged-foil phantom water drag and pitching moment (`h/c <= 0` now returns zero generalized force with smooth emergence wetting over `0 < h/c < 0.25` and ventilated `flap_cm_delta`).
  - `forces/buoyancy.py`: hoisted `R_body_to_ned` import out of the inner RK4 call path.
  - `sensors/models.py`, `estimation/kalman.py`, `control/lqr.py`, `control/pid.py`: fixed `Channel.step` schedule drift when `rate_hz > 1/dt`, single-speed `0/0 = NaN` in `KalmanEstimator._interp` and `LQRController._sched`, and inverted roll-sign comments in `pid.py`.
  - `design.py`, `sim_server.py`, `viz.py`: guarded `mass <= sum(default_payloads)` in `design.py`, fixed Windows path-traversal validation and log-file leaks in `sim_server.py`, and fixed inverted pitch sign + NED depth axis in `viz.py`.
  - `vehicle.py`: switched servo `q_schedule_speed` to measured `cnu[0]` and routed the legacy `kinematic` actuator branch through `ceta` and `supervisor.shape_raw`.
- 3D lateral-directional turning physics & control (split front flaps, single-piece rear elevator, no rudder):
  - `dynamics/rigid_body.py`, `vehicle.py`, `forces/hull.py`: added analytical strut sway/yaw added mass (`strip_added_mass_sway_yaw`, `27.82 kg` / `37.10 kg·m²` on manned) with exact passive destabilizing yaw Munk moment, displacement hull lateral crossflow drag & yaw rotational damping, geometrically tilted wetted strut length, wingtip submergence (`_foil_tip_submergence_over_chord`) + 3-station span-averaged free-surface lift loss, wingtip-first breach ventilation, and strut sideslip ventilation hysteresis (`8.5°` onset / `4.5°` recovery).
  - `control/lqr.py`: added steady-turn equilibrium feedforward (`steady_turn_sensitivity` + `_turn_reference` solving linear `(v_ref, r_ref, du_roll)` plus exact level-turn `w_ref`, `q_ref`, and second-order `(cos(phi)-1)` body-z gravity shift) and roll-prioritized split-flap diamond allocator (`allocate_split_flaps`) with saturation-aware anti-windup.
  - `control/supervisor.py`, `monitor.py`, `viewer.py`: added dynamic `safe_bank_limit` (`15°` calm cruise, `12°` at `6 m/s`, narrowing automatically with wave height and ride height to protect both outer wingtip submergence `>= 1.25 c` and inner chine clearance `>= 0.10 m`), smooth `12°/s` bank ramp (`24°/s` emergency), `roll_limit = 20°`, `TAKEOFF → FOILING` attitude/pitch-rate hysteresis + `0.25 s` soft attitude persistence (`14°` pitch / `25°` roll hard ceiling instantaneous), wingtip-aware breach alert + sideslip alert in `FlightMonitor`, and `yaw`/`yaw_rate`/`sideslip`/`bank_max` viewer telemetry.
- Documentation & tests:
  - Created `docs/22-assumptions-and-provenance.md` (complete ledger of all assumed/placeholder/derived/tuned values across the codebase) and `docs/23-code-audit-and-3d-turning.md` (three-tier break-test report and 3D turning engineering study).
  - Added `tests/test_3d_turning_and_audit.py` (13 regression tests).
- Follow-ups:
  - Replace placeholder structural/hydrodynamic/drive values per `docs/22-assumptions-and-provenance.md` once CAD `.STL`, Flow Simulation polars, and bench tests arrive.
  - Decide whether to tighten `LQR_WEIGHTS` `iphi` coupling in `autotune.py` (currently causes ~`2°` transient bank overshoot because `phi` was relaxed to `4.52°` while `iphi` stayed at `0.8°·s`) and whether a steering rudder or smaller aft strut chord is desired if tighter turn radii than `R ≈ 78 m` (`15°` bank at `8 m/s`) are required.

### 2026-11-20 — Ergonomics, Auto Combined PID/LQR Mode, Interactive 3D CG Boat Editor & Persistent Autotuner
- Launchers & cross-platform reliability:
  - Added `.gitattributes` enforcing LF for `*.sh` and CRLF for `*.bat`.
  - Fixed `run_sim.bat` dependency check (`if not errorlevel 1` so missing deps properly install into `.venv`), added `PIL` to both `run_sim.bat` and `run_sim.sh` import checks, added macOS `mjpython` auto-detection in `run_sim.sh` and `sim_server.py::_viewer_python()`, and updated options 2–3 to `--controller auto`.
  - Expanded `viewer.py` keybinds (`Space` level wings, `-`/`+` speed aliases, `F` foil/land in both supervised and unsupervised modes, `C` cycle `Auto Combined -> LQR -> PID` smoothly, `U` cycle `m/s -> km/h -> mph`), cross-platform `_focus_window`, 3D Rider/payload visualization at edited `(x, y)` coordinates, and `ki_pitch` live gain updates.
- Auto Combined PID/LQR controller (`controller="auto"`):
  - Added `auto_handback` and `status_dict()` to `control/lqr.py`: engages LQR only once foilborne and steady, and smoothly hands back (`0.5 s` fade) to PID when slowing down or returning onto the hull.
  - Updated `control/supervisor.py::_phase_choice()` and `vehicle.py` (`set_controller_mode`, `controller_status`, `draft` passing) so `controller="auto"` uses PID on the water and for landing touchdown and LQR for foilborne flight and turns.
- Interactive Boat Editor & 3D Center-of-Gravity / Inertia Auto-Calculation:
  - Updated `design.py` (`apply_design`, `mass_properties`, `derived`, `design_summary`): idempotent `_base_mass`/`_base_inertia`/`_base_payloads` preservation across sequential edits, explicit `empty_mass` + placed `payloads` 3D parallel-axis CG `(cg_x, cg_y, cg_z)` and inertia calculation, and plain-English qualitative assessments (`verdict`, `balance`, `wings`, `power`).
  - Added `HullHydrostatics.set_cg_offset()` and wired it through `VehiclePhysics.set_cg_offset()` so CG shifts consistently move hull buoyancy as well as foils/struts.
- Web Dashboard (`page.html` & `sim_server.py`):
  - Added 3-way `m/s -> km/h -> mph` speed unit selector synced between header, Fly gauges/helm, Boat Editor fields/chart/table, and the 3D viewer (`U` key).
  - Added live controller mode bar + newcomer-friendly explanation cards comparing Auto Combined, LQR, and PID, plus `Foil / Land (F)` helm button.
  - Made payloads draggable on the Boat Editor SVG blueprint with real-time CG/inertia recalculation, empty hull mass field, quick-add mass buttons, plain-English field descriptions, and qualitative design summary cards.
  - Fixed autotuner tab-switching & backend persistence: atomic `.tmp` JSON writes in `tuning/autotune.py`, cached last progress in `sim_server.py::tune_status`, automatic backend saving of best PID/LQR results to `local/overrides.json` with a live "Stored Backend Controller Parameters" readout, background progress badge on the Autotuner tab button, and live PID gain sync to the running viewer.

### 2026-11-20 (evening) — Phase-Aware Combined Autotuning (Rising + Cruising/Turning + Falling)
- `hydrofoil/control/lqr.py`: added module-level `_PLANT_LIN_CACHE` keyed on plant/geometry/servo/speed parameters (excluding PID gains, LQR weights, wave/sensor/supervisor flags) so repeated `LQRController` constructions during tuning reuse pre-linearized matrices bit-for-bit without redundant Newton trim solves.
- `hydrofoil/tuning/scoring.py`: added `turn` (12° banked 3D split-flap maneuver: `roll_overshoot`, `roll_rms_err`, `sideslip_peak`) and `landing` (descent to touchdown + LQR→PID handback: `vz_touch`, `touch_pitch`, `touch_roll`, `handback_ok`, post-touchdown bobbing `z_rms`) scenarios, plus `PHASE_SCENARIOS`, `phase_scenarios()`, and `phase_costs()` separating costs into Rising, Cruising/Turning, and Falling phases.
- `hydrofoil/tuning/classical.py`: fixed `relay_test`'s monkeypatched `relay(error, meas, integrate=True)` signature to accept the `integrate` keyword passed by `HydrofoilController.pid_commands`, excluded landing/rising runs from foilborne `bryson_update`, and measured `turn` roll excursion relative to the commanded 12° bank angle rather than zero.
- `hydrofoil/tuning/autotune.py`: added default `--target auto` ("Both — Phase-Aware Combined") pipeline: Stage 1 tunes PID gains specifically on Rising (`takeoff`, `supervised`) and Falling (`landing`) in `controller="auto"`, while Stage 2 locks those PID gains and tunes LQR weights (with `iphi`/`itheta` coupled proportionally to `phi`/`theta`) across Cruising & 3D Turning (`head`, `following`, `shift`, `noisy`, `turn`) + full-envelope handovers.
- `hydrofoil/sim_server.py`, `hydrofoil/assets/launcher/page.html`: `_persist_tuned` now saves both PID gains and `lqr_weights` together for `target="auto"`; dashboard Autotuner defaults to Phase-Aware Both and displays per-phase score breakdown cards (`Rising`, `Cruising & Turning`, `Falling`) alongside all 8 scenario cards.

### 2026-11-21 — UI Anti-Clipping, Backend Cleanup, and Null / Collapsing Edge-Case Hardening
- `hydrofoil/viewer.py`: added adaptive font scaling & ellipsis fitting (`Hud._fit_text`) across the key bar, alert banner, cell text, and heel dial readout so narrow viewer windows never clip text across cell borders; removed dead variables (`lqr_spare`, `ctl`, `sea`) in `run()`.
- `hydrofoil/assets/launcher/page.html`: widened/wrapped `.bar` grids, added horizontal scroll wrappers on `.storebox` and `#cmp` tables, word-wrapped preformatted blocks, scaled `drawSide`/`drawPlan` bounds to include aft tails and outboard payloads, and flipped right-edge SVG payload/CG labels + tick anchors cleanly.
- `hydrofoil/forces/foil.py`, `hydrofoil/actuation/hinge.py`, `hydrofoil/forces/hull.py`, `hydrofoil/vehicle.py`, `hydrofoil/consistency.py`, `hydrofoil/estimation/kalman.py`, `hydrofoil/monitor.py`, `hydrofoil/design.py`: hardened against zero/collapsing foil chords & spans, stale `last_CL`, zero vector axes, extreme attitude cosines, zero `w0` in `HeaveFilter`, duplicate timestamps in `FlightMonitor`, `None` scalar overrides, missing payload coordinates, and sub-minimum slider inputs.
- `tests/test_ergonomics_and_auto.py`: added `test_null_collapsing_physics_and_hud_anticlipping`.

### 2026-11-22 — Independent Dual Front Flaps, Jacobian QP Control Allocation, Top-Down Turn Safety & System Architecture
- Baseline commit: committed all verified prior work (`dca58ca`, sole human author Josh) before building out the dual-flap extension.
- Singularity & hardcode audit fixes (`control/supervisor.py`, `control/lqr.py`, `linearize.py`, `vehicle.py`, `monitor.py`, `consistency.py`): clamped `arccos(rhs/r)` inputs, added `lstsq`/`pinv`/Levenberg–Marquardt fallbacks, parameterized `bank_v_min_deg` and `flap_cmd_max_deg`, and replaced static flap-authority/sideslip alerts in `FlightMonitor` with live allocator and turn-envelope bounds.
- Independent dual front flaps hydrodynamics & speed optimization (`forces/foil.py`, `params.py`, `vehicle.py`, `actuation/loads.py`): added `bisect`-indexed `FoilTables` slopes, `cavitation_number`, `cp_min_estimate`, `effective_flap_and_slope`, `Foil.local_derivatives()`, explicit `delta_left`/`delta_right` evaluation with symmetric-preserving `split_flap_cross_coupling`, and scalar kinematics in `VehiclePhysics`.
- Physics × Control-Stability Jacobian QP Allocator (`control/allocator.py`): built `ControlAllocator`, `SAFETY_PRESETS` (`conservative`/`standard`/`sport`), exact 3-variable box QP (`solve_box_qp_3d`, `0.341 ms/step` avg), closed-form quadratic cavitation bounds, stability-gated reverse drive (lift floor + elevator pitch reserve), and Roll+Pitch jammed-flap reconfiguration.
- Top-down turning safety envelope (`control/supervisor.py`): added `compute_turn_envelope()` computing live `bank_max_deg`, `yaw_rate_max_deg_s`, `turn_radius_min_m`, lateral-g ceiling, sideslip CBF rate damping, and optional `pilot["yaw_rate"]` command support.
- Frontend visibility & UI upgrade (`viewer.py`, `sim_server.py`, `assets/launcher/page.html`): exposed `alloc`, `turn_envelope`, `ventilation`, `cavitation_sigma`, `height_status`, `drives` (`M`, `I`, `P`, `limited`, `cap`), and live toggles (`safety_margin`, `allocator`, `reverse_drive`) across `/state`, `/set`, CLI/launcher selectors, and three new live dashboard cards + top-down `#turnSvg` radius overlay.
- Documentation & verification: added [docs/00-master-system-architecture.md](00-master-system-architecture.md) (Mermaid + ASCII block diagrams) and [docs/24-dual-front-flaps-and-jacobian-allocation.md](24-dual-front-flaps-and-jacobian-allocation.md); verified `134/134` existing tests pass, `11/11` tests in `tests/test_dual_flaps_allocator.py` pass, `python -m hydrofoil.consistency` exits `0` (`0 FAIL`), and `HTMLParser` + `node --check` pass cleanly.

### 2026-11-22 (late) — Red-Team Audit Hardening, 3D CG/Cavitation Physics, UI Telemetry Fixes & Directional JONSWAP Waves
- Tier-1 Control & Logic Fixes:
  - Enforced strict no-cheat separation in `VehiclePhysics._servo_commands` and `ControlAllocator` (`use_truth=False` in `sensors="noisy"` routes estimated `cstate`, `calm_sample()`, `vent={}`, and `_r_nominal` to the allocator while `ServoDrive.update()` still integrates against true `flows_true`).
  - Removed the rank-1 collinear T-foil yaw offset `dw_des[3]` from `ControlAllocator.allocate`, evaluated exact nonlinear lift (`_eval_nl`), gated `reverse_drive_enabled=False` around aerodynamic zero-lift (`cl_L * cl_R < -1e-4` or chordline split `> 9°`), added slew-before-QP rate clamping on `u_des` before computing `dw_des`, added T-foil elevator-heave QP decoupling (`B[0][2] = B_nom[0][2] = 0.0`) to prevent pitch-for-heave inversion on `300 kg` off-center takeoff, optimized hot-path buffer reuse (`~0.341 ms/call`), and fixed LQR `(ch, j)` channel-to-integrator indexing in `update()` and `sync_sent(..., sat_flags=...)`.
  - Replaced hardcoded `eta_turn` with lazily cached plant-derived `VehiclePhysics.turning_efficiency(u)`, and upgraded `Supervisor.update()` sideslip CBF to actively unwind `bank_tgt` when `|beta| > 0.80 * beta_max`.
- Tier-2 Physics & UI / Caching Fixes:
  - Wired 3D vertical CG (`cg_z`) through `HullHydrostatics.set_cg_offset`, `VehiclePhysics.set_cg_offset` / `hull_offset`, `linearize.py`, `monitor.py`, `supervisor.py`, and `viewer.py`, plus thrust-line pitching moment `tau[4] += thrust * (prop_pos_z + cg_z)`.
  - Activated $C^1$-smooth physical cavitation degradation (`cavitation_model=True`, `cav_fac` + `cd_cav` + `cp_min_and_slope` capped at `18° / 30°`) inside `Foil._eval_force_and_deriv` and enabled across `VehiclePhysics` foils/struts.
  - Fixed `_last_sigma` updates across `allocator="legacy"`, `kinematic`, and `cmds_override`, updated `design.py` `flap_margin` & `_CONTROL_KEYS`, replaced hardcoded `stopMap` in `page.html`, and surfaced `yaw_rate_cmd`, `sup.log`, `drives[d].cap`, `alloc.solve_ms`, and heading `ψ` (`yaw`) in the dashboard UI.
- Documentation, Wave Quantification & Final Verification:
  - Updated [docs/00](00-master-system-architecture.md), [docs/01](01-overview-and-status.md), [docs/02](02-code-reference.md), [docs/05](05-constraints-and-limits.md), [docs/06](06-control-system.md), [docs/07](07-interfaces.md), [docs/10](10-roadmap.md), [docs/11](11-known-issues.md), [docs/22 §13](22-assumptions-and-provenance.md) (CFD & bench drop-in checklist), and [docs/24](24-dual-front-flaps-and-jacobian-allocation.md).
  - Implemented `encounter_frequency()` and vectorized `IrregularWave` (JONSWAP $\gamma=3.3$ + $\cos^{2s}$ directional spreading) in `waves/airy.py`, wired `--wave-type` through `viewer.py`, `sim_server.py`, and `page.html`, ran `tools/wave_study.py` across `0°–180°`, and documented findings in [docs/25-wave-direction-and-irregular-sea-impact.md](25-wave-direction-and-irregular-sea-impact.md).
  - Final verification: `pytest tests/ -q` `148/148` passed (`0` failed), `python -m hydrofoil.consistency` `0 FAIL`, and `HTMLParser` / `node --check` passed cleanly.

### 2026-11-25 — CAD Branch Integration, Boot Default, Interconnectivity Fixes & Side + Top-Down UI
- What changed:
  - Integrated the CAD boat branch (`cad` SolidWorks geometry preset, `HullHydrostatics(..., boxes=...)` & `box_drag`, `resolve_cad_hull`, `flap_dy`, live `main_incidence_deg` slider, CAD MuJoCo assembly) onto `mujoco-viewer`.
  - Switched boot default across [viewer.py](../hydrofoil/viewer.py), [sim_server.py](../hydrofoil/sim_server.py), [run_sim.bat](../run_sim.bat), and [run_sim.sh](../run_sim.sh) to `--preset cad` (keeping `model` and `manned` selectable).
  - Fixed physics, estimation, control, and tooling disconnects across [foil.py](../hydrofoil/forces/foil.py) (thread-safe `_apply_flap_dy`), [hull.py](../hydrofoil/forces/hull.py) (`wetted_draft`, `rot_damping` on `box_drag`, `buoyancy` syntax fix), [vehicle.py](../hydrofoil/vehicle.py) (true dual front struts at `±main_strut_y`, `KalmanEstimator` wiring & graceful fallback, smooth analytical `_trim_ff` incidence shift, piecewise pontoon+box rest equilibrium), [kalman.py](../hydrofoil/estimation/kalman.py) (skip untrimmable speeds cleanly), [loads.py](../hydrofoil/actuation/loads.py) & [consistency.py](../hydrofoil/consistency.py) (explicit `main_flap_chord`), [allocator.py](../hydrofoil/control/allocator.py) (`main_flap_center_y` roll normalization, `cp_min_max` & `hc_min` telemetry), [supervisor.py](../hydrofoil/control/supervisor.py) (turn envelope lens clearance), [design.py](../hydrofoil/design.py) (`resolve_cad_hull` ordering, override preservation, proportional flap arm scaling), [scoring.py](../hydrofoil/tuning/scoring.py) / [classical.py](../hydrofoil/tuning/classical.py), [cad_geometry.py](../hydrofoil/cad_geometry.py), and [tools/wave_study.py](../tools/wave_study.py) (`--preset`).
  - Modernized [page.html](../hydrofoil/assets/launcher/page.html) with a split Fly Hero (`#side` + Top-Down Turning & Oblique Wave view `#topView`), secondary hydro/cavitation/encounter/heading KPIs (`#gd`, `#gcav`, `#gwav`, `#gnav`), Boat Editor `cockpit_buoyancy` toggle, `cockpit_box` planform preview, and non-destructive `/api/load_cad` form handling; updated `docs/` (`README`, `00`, `07`, `12`, `22`, `26`) and expanded [test_cad_preset.py](../tests/test_cad_preset.py).
- Verified how: `pytest tests/ -q` (`160 passed`), CAD `5°` box-OFF takeoff & outboard-flap turn trials, `python -m hydrofoil.consistency`, `HTMLParser` + `node --check`, link check, and zero-attribution check.

### Template for future entries
```
### YYYY-MM-DD — Title
- What changed (files)
- Why
- Verified how (test/run/sweep) — or "not verified"
- Parameter changes (old → new)
- Follow-ups
```
