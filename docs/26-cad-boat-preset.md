# 26 — CAD boat preset (`--preset cad`)

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
* `VehiclePhysics.set_main_incidence()` and the viewer's **Main wing mounting angle** slider: physics changes at
  once; trim feed-forward and LQR are re-derived in the background (PID flies meanwhile).
* The LQR skips speeds where the boat has no level trim inside the surface limits, and falls back to the PID loops
  (with a dashboard message) if there are none.

## What the sim shows (2026-11-24)

At 0° the boat **does not fly**:

1. **It cannot float level at rest.** 275 kg is 96 % of the pontoons' full volume (287 kg), and the assumed CG is
   0.48 m ahead of their centre, so the bows go under (pitch −70° after 30 s with no thrust). The pontoon model
   checks out: 150 kg centred over them floats level at the expected 0.138 m draft.
2. **Under power it stalls at 2.1 m/s** against the hull's drag hump with the pontoons fully submerged, then
   nose-dives.
3. **Even ignoring the hull, the wing is too small/too shallow at 0°.** Level trim needs about 15.5 m/s with the
   flap inside ±15° (surface effect at 1.5 chords costs about 35 % of the lift). At a 4–5° mount it would trim from
   about 8.5 m/s.
4. **Roll control is weak.** The flaps are 35 % of the span and next to the centreline.

Items 1–2 hinge on the assumed mass and CG: get the assembly's mass properties (SolidWorks → Tools → Evaluate →
Mass Properties) before drawing conclusions from them.
