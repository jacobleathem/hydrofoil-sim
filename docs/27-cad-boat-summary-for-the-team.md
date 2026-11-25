# 27 — Our CAD boat in the sim: what we did and what it shows

Plain-language summary for the team. The detailed measurement table and every assumption are in
[26-cad-boat-preset.md](26-cad-boat-preset.md).

**Branch:** `jacob/cad-boat`. **Try it:** run `./run_sim.sh` (or `run_sim.bat`), then click **Load our CAD boat**.

---

## 1. What we did

### Put our real numbers into the sim
Josh's sim had two made-up boats (a 25 kg model and a 275 kg "manned" sizing). We added a third, **"Our boat
(CAD)"**, built from our SolidWorks files (`Hydrofoil Assembly+wings_not_finished` and `Split Wing Assembly`).
We exported them as STL and measured them with a script that slices the 3D mesh:

* **Main wing:** NACA 0009 (confirmed: 9.0 % thick), 1.353 m span, 300 mm chord, rounded tips, 0.367 m² area.
* **Flaps:** two, in a notch on the trailing edge, from the centreline out to 0.24 m each side, 100 mm deep.
* **Rear wing:** 677 × 149 mm, NACA 0009, rotates as one piece (no flap), 2.39 m behind the main wing.
* **Both wings mounted at 0°.**
* **Struts:** 0.75 m from the pontoon bottoms to the wing.
* **Hull:** two pontoons (2.54 m × Ø0.265 m, 1.31 m apart) plus a 48 × 32 × 24 in cockpit box.
* **Props:** two, on the front masts, just above the wing.

The preset is in `hydrofoil/params.py` (`CAD_PARAMS`). We did **not** tune anything to make it fly: the point is
to see what our design does.

### Adjusted the wing's lift to match our wing
The sim calculates lift from strips of wing, each a rectangle. Our wing isn't a rectangle, so:

* **Same area, same span.** Each half is a rectangle 0.6765 m long with a 0.271 m "average" chord, which gives the
  same 0.367 m² as the real rounded-tip wing. The lift slope follows the real aspect ratio (5.0).
* **Lift acts where the area is.** Each half's lift acts at its area centre, 0.31 m from the centreline (measured).
* **Flap strength from the real flap.** The flaps are 33 % of the chord over 35.5 % of the span, so each flap adds
  less lift than a full-span flap; the sim scales that.
* **Flap lift acts at the flap (new physics).** The extra lift a flap makes now acts where the flap actually is,
  0.12 m from the centreline, not at the middle of the wing half. This matters most for banking (section 3).
* **Mounting angle slider.** A new **Main wing mounting angle** lever (0–5°, default 0° like the CAD) tilts the
  whole wing in the 3D view and changes the lift immediately.
* **Surface effect is automatic.** The sim already loses lift when a wing is close to the surface; with our short
  struts the wing flies only about 0.40 m (1.5 chords) deep, which costs about a third of its lift.

### Dual struts, twin pontoons, cockpit box, new 3D model
* **Two front struts** (±0.35 m) instead of one in the middle, for side force and drag.
* **Twin pontoons** instead of one hull: much stiffer against rolling over when floating.
* **Cockpit box buoyancy**, with a dashboard toggle: **on** = the box floats (and drags through the water);
  **off** = the box doesn't count and the pontoons are made fatter (Ø0.367 m at 275 kg) so they alone hold the
  boat with 2× reserve.
* **New 3D model** in MuJoCo of our layout: pontoons, frame, cockpit box, both masts, motors and props, the rear
  mast, the rounded-tip wing with its two flaps, and the rear wing.
* **"Load our CAD boat"** buttons on the dashboard reset everything to the CAD values.

---

## 2. Limitation: our wing is not strong enough at 0°

To fly, the wing has to make lift equal to the boat's weight (275 kg ≈ 2700 N, an assumption, see section 4).
Lift grows with speed², wing area, and angle of attack. With the wing mounted flat (0°) and the boat level, the
only angle comes from the flaps, and those only cover a third of the span.

Slowest speed at which the boat can fly level with the flaps inside ±15° (from the sim's own trim solver):

| Main wing mounting angle | Slowest level-flight speed |
|---|---|
| **0° (our CAD)** | **≈ 15.5 m/s (56 km/h)** |
| 3° | ≈ 9.5 m/s |
| 4° | ≈ 8.5 m/s |
| 5° | ≈ 8.0 m/s (our planned cruise) |

**So we need an offset (mounting) angle of about 4–5°** on the main wing, or more wing area, or a deeper wing
(longer struts), for the boat to fly at our planned 8 m/s. Demo: launch the CAD boat and move the **Main wing
mounting angle** lever; the trim feed-forward and LQR are recalculated for the new angle in about a second.

Two more things currently stop it before the wing even matters (both depend on assumptions, section 4):

* **It can't get fast enough.** With full thrust (1000 N, a placeholder) the hull's drag stops it at about
  2.5 m/s with the box on, 3.8 m/s with the box off and bigger pontoons.
* **It sits nose-down.** We assumed the centre of gravity (CG) is over the main wing, but the pontoons and box
  hold the boat up about 0.48 m further back, so the bow sinks at rest.

---

## 3. Limitation: banking (roll) is very weak

A boat banks (rolls) by pushing one flap down and the other up. The roll it gets is **force × distance from the
centreline**. Our flaps sit right next to the centreline, so their lever arm is tiny: about **0.12 m**, against
0.31 m if they were in the middle of each wing half, or about 0.6 m on Josh's full-span design.

Measured in the sim at 8 m/s:

| | Roll moment from 1° of opposite flap | Roll acceleration with 10° of opposite flap |
|---|---|---|
| **Our CAD flaps (inboard)** | **9.3 N·m** | **≈ 58 °/s²** |
| Same flaps centred on each half | 24.0 N·m | — |
| Josh's manned design (full-span flaps) | 227 N·m | ≈ 900 °/s² |

Our boat has about **1/24 of the roll control** of Josh's design and banks roughly **15× slower**. In practice it
will be slow to correct a gust or a wave hitting one side, and slow to bank into turns. Ways to fix it: move the
flaps **outboard** (toward the tips), make them longer, or both.

---

## 4. What is still a guess

The STL files have shapes, not masses. Before trusting the "can't float level" and "can't get fast enough"
results, we need from SolidWorks (**Tools → Evaluate → Mass Properties**, with materials set):

* total mass (and rider mass) — we assumed 275 kg;
* CG position — we assumed over the main wing, 0.35 m above the pontoon bottoms;
* moments of inertia.

Also still placeholders: thrust (1000 N), hull drag, the rear wing's pivot point and travel, flap travel, and the
controller gains (Josh's, tuned for his boat).

## 5. Suggested next steps for the CAD

1. Get mass properties and put them in the sim.
2. Add a 4–5° mounting angle to the main wing (or more area / deeper wing) and re-check.
3. Move the flaps outboard for roll control.
4. Decide where the CG goes: over the main wing (good for flying) or over the pontoons (good for floating); they
   are 0.48 m apart now.
5. Real thrust and drag numbers for the motors and pontoons.
