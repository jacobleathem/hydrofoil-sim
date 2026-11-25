# Hydrofoil Sim — Documentation

Written 2026-11-04 as a record of the repo at commit `2af5157`
(<https://github.com/jacobleathem/hydrofoil-sim>), by reading every script.
Docs 01–12 were written by reading; doc 13 and the updates it triggered were produced by running the code
(see [11-known-issues.md](11-known-issues.md) → "Verification status").

| Doc | What's in it |
|-----|--------------|
| [00-master-system-architecture.md](00-master-system-architecture.md) | **Master System Architecture:** Mermaid & ASCII closed-loop block diagrams, module map, coordinate frames/sign conventions, singularity hardening, and telemetry contract |
| [01-overview-and-status.md](01-overview-and-status.md) | What the project is, what exists, progress, what's stub/missing |
| [02-code-reference.md](02-code-reference.md) | Every file and class/function, one by one |
| [03-math-and-physics.md](03-math-and-physics.md) | Equations of motion, foil model, hull, waves, integration |
| [04-constants-and-parameters.md](04-constants-and-parameters.md) | Every constant, default parameter, gain, with units and provenance |
| [05-constraints-and-limits.md](05-constraints-and-limits.md) | Clamps, saturation limits, stability limits, modelling assumptions |
| [06-control-system.md](06-control-system.md) | PID loops, actuator model, surge loop, sign conventions |
| [07-interfaces.md](07-interfaces.md) | UDP packet formats, REST/WebSocket API, CLI flags |
| [08-mujoco-viewer.md](08-mujoco-viewer.md) | The MuJoCo app (replaced Unity and the dashboard): sliders, plots, drive read-out, frames, keys |
| [09-manual.md](09-manual.md) | How to install, run, operate, and tune |
| [10-roadmap.md](10-roadmap.md) | Planned and suggested future development |
| [11-known-issues.md](11-known-issues.md) | Bugs, inconsistencies, stale docs, unverified items |
| [12-progress-log.md](12-progress-log.md) | Dated record of project history and this documentation pass |
| [13-math-audit-and-pid-critique.md](13-math-audit-and-pid-critique.md) | Math audit as found: PID method and flaws, physics omissions (pre-fix state) |
| [18-audit-verification.md](18-audit-verification.md) | **Every audit claim re-checked against the original code**: verdicts, evidence, corrections to my own claims |
| [17-actuators-moteus-drive.md](17-actuators-moteus-drive.md) | Control-surface drives: hinge moments, moteus/BLDC/gearbox current-voltage-power budget, gear-ratio sweep, stress, failure modes, next steps |
| [15-buoyancy-hydrostatics-cross-reference.md](15-buoyancy-hydrostatics-cross-reference.md) | Hull buoyancy checked against metacentric theory; distributed hydrostatics model; before/after stiffness |
| [16-review-guide.md](16-review-guide.md) | **Start here for review:** conventions, equations→code map, assumptions ledger, evidence/commands, open questions |
| [19-linearisation-lqr-estimation.md](19-linearisation-lqr-estimation.md) | Trim, linearisation, scheduled / delay-aware LQR, sensors and Kalman, manned sizing, supervisor, autotuner |
| [20-flap-and-drive-design.md](20-flap-and-drive-design.md) | Main-flap geometry and gear ratio vs hinge torque, current and mechanics; what mechanism has to be designed |
| [21-per-phase-controller-trials.md](21-per-phase-controller-trials.md) | 108-run sweep of controller choice per supervisor phase (`DISPLACEMENT`, `TAKEOFF`, `FOILING`, `LANDING`) across calm, head, and following seas |
| [22-assumptions-and-provenance.md](22-assumptions-and-provenance.md) | **Master ledger of all assumed, placeholder, derived, tuned, and physical values** across every module, with origin, risk, and hardware replacement plan |
| [23-code-audit-and-3d-turning.md](23-code-audit-and-3d-turning.md) | **Three-tier adversarial code/logic audit and 3D split-front-flap turning analysis**: module/subgroup/project break tests, sideways forces, adverse yaw, spiral overbanking, safe bank envelope, and coordinated-turn LQR/supervisor design |
| [24-dual-front-flaps-and-jacobian-allocation.md](24-dual-front-flaps-and-jacobian-allocation.md) | **Dual Front Flaps, Jacobian QP Control Allocation & Top-Down Turn Envelope**: $4\times 3$ Jacobian + exact 3-var QP, 3-layer Physics × Control-Stability bounds, stability-gated reverse drive, top-down turn safety envelope, audit fixes, and verification |
| [25-wave-direction-and-irregular-sea-impact.md](25-wave-direction-and-irregular-sea-impact.md) | **Wave Direction & Irregular JONSWAP Sea Impact**: encounter-frequency physics ($\omega_e(\mu)$), directional load mechanisms, quantitative `0°–180°` regular vs. directional JONSWAP sweep results, and control/supervisor guidance |
| [26-cad-boat-preset.md](26-cad-boat-preset.md) | **Our Boat from the CAD (`CAD_PARAMS`)**: measured SolidWorks dimensions, part-span inboard flap placement, twin pontoons & cockpit box toggle, dual front struts, assumptions, and flight behavior |
| [27-cad-boat-summary-for-the-team.md](27-cad-boat-summary-for-the-team.md) | **CAD Boat Plain-Language Team Summary**: overview of the CAD preset, wing mounting angle requirement, inboard roll-arm findings, and design next steps |
| [14-error-register-and-fixes.md](14-error-register-and-fixes.md) | **Every error expanded: cause, evidence, fix, test; retuned gains; unit/conversion map; still-open items** |

Conventions used across the code (and these docs): Fossen (2011) notation,
NED world frame (z **down**), body frame x-forward / y-starboard / z-down,
ZYX Euler angles in radians internally, degrees at UI/UDP-input boundaries.
