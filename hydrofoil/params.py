"""Default simulation parameters, shared by the dashboard and the viewer."""

from __future__ import annotations

from typing import Any

DEFAULT_PARAMS: dict[str, Any] = {
    "mass": 25.0,
    "ixx": 2.0, "iyy": 5.0, "izz": 5.0,
    "heave_damping": 50.0,
    "main_chord": 0.15, "main_span": 0.6,
    "elev_chord": 0.10, "elev_span": 0.3,
    "main_pos_x": 0.0,  "main_pos_z": 0.5, "main_lateral_offset": 0.35,
    # Fixed mounting incidence of the main foil [deg]. Carries most of the weight at cruise so the flap trims
    # near neutral and keeps authority both ways (at 0 the flap sat at 14 deg of a 20 deg limit at 4 m/s and
    # clipped in 10 cm waves). 2.5 deg puts the trim flap at about +12 deg at 3 m/s and -7 deg at 7 m/s.
    "main_incidence_deg": 2.5,
    # Centre-of-gravity shift from the nominal layout (dx forward, dy to starboard) [m], e.g. a rider's position.
    # Vertical CG shift cg_z [m, +ve UP] shifts vertical foil/strut/hull/prop lever arms; VehiclePhysics.set_cg_offset() updates mid-flight.
    "cg_offset": (0.0, 0.0), "cg_z": 0.0, "prop_pos_z": 0.0,
    "elev_pos_x": -1.2, "elev_pos_z": 0.4,
    # Vertical struts: give the side force that makes bank turn the boat instead of letting it
    # skid (without them a banked boat slides sideways and yaw has the wrong sign). Chord is a
    # PLACEHOLDER; the wetted length is computed from depth below the surface every step.
    "main_strut_chord": 0.12, "elev_strut_chord": 0.08,
    # Heave gains are tuned higher than sim.py's — here the vehicle starts at rest
    # and must generate real lift to take off, not just trim near an already-flying point.
    "kp_heave": 40.0, "ki_heave": 6.0, "kd_heave": 10.0,
    "kp_pitch": 1.0, "ki_pitch": 1.0, "kd_pitch": 0.1,   # ki_pitch: holds theta = theta_ref (P-only sagged nose-down and stalled takeoff on short-tail / aft-foil plants)
    # Ki must stay 0 here — any nonzero integral term reliably diverges under
    # wave disturbance (verified: Ki=0.05 blew up to 1000+ deg within 90s).
    "kp_roll":  2.0, "ki_roll":  0.0, "kd_roll":  0.2,
    # Target height of the HULL above the water once foilborne [m] (negative =
    # hull still below the surface). Internally converted to a CoG depth
    # reference via hull_offset — this is the number you actually care about.
    # Default = half the strut length (hull_offset + main_pos_z) / 2, so the
    # main strut's midpoint sits right at the waterline once flying — half
    # the shaft above water, half below.
    "flying_height": 0.55,
    "trim_ff_deg": 1.63, "trim_ff_speed": 4.0,   # speed-scheduled trim flap feed-forward (0 = off); = linearize.trim(theta=0) main flap at trim_ff_speed (a test checks it)
    "sensors": "ideal",                  # "ideal" (controllers read ground truth) or "noisy" (sensor models + Kalman estimator)
    # "lqr" (default): scheduled MIMO LQR after takeoff (control/lqr.py, doc 19), needs the servo actuator model (falls
    # back to PID otherwise); "pid": the three SISO loops, which also fly takeoff. The PID gains below stay in use for takeoff.
    "controller": "lqr",
    "lqr_delay_s": None,                 # loop latency the LQR is designed for [s]; None = the drive model's delay_s
    "phi_ref": 0.0, "theta_ref": 0.0,
    "waves_enabled": False,
    # Period is derived from wavelength by the deep-water dispersion relation. Direction is the
    # wave TRAVEL heading in degrees (0 = +x, the way the boat faces); oblique/beam seas (e.g. 90)
    # are what load the left and right foils differently and excite roll.
    "wave_amplitude": 0.05, "wave_wavelength": 5.0, "wave_direction": 0.0,
    "dt": 0.01, "t_end": 60.0,

    # Velocity (surge) controller: closed-loop speed control, thrust in Newtons.
    "target_speed": 4.0,
    "kp_speed": 30.0, "ki_speed": 5.0, "kd_speed": 0.0,
    "max_thrust": 50.0,   # matches the Unity TwinMotorController placeholder (50N)

    # Hull buoyancy / drag — a simplified wall-sided hull near the waterline.
    # hull_offset is how far above the CoG the hull rides (roughly the strut
    # length); waterplane_area/max_draft set how buoyant it is at rest and how
    # much draft it has before the linear approximation caps out.
    # Hull footprint (rectangle, PLACEHOLDER): waterplane area = length x beam = 0.6 m^2 as before. The footprint
    # (not just its area) sets roll/pitch hydrostatic stiffness through the waterplane inertia.
    "hull_length": 2.0, "hull_beam": 0.30, "hull_x": 0.0,    # m; footprint centre relative to the CoG (0 = floats level with the CoG over the CB)
    "max_draft": 0.3, "hull_offset": 0.6,
    "hull_drag_coeff": 20.0,
    # Roll/pitch damping of the floating hull [N*m*s/rad at full draft], scaled by wetted fraction. PLACEHOLDER
    # standing in for radiation damping (~0.2 of critical in pitch at rest draft).
    "hull_rot_damping": 400.0,

    # Control-surface drives (moteus r4.11 + BLDC + gearbox), see hydrofoil/actuation and docs/17.
    # Layout: "flap" = 40 % plain flap on each main-foil half (as drawn in the viewer);
    # "all_moving" = whole surface rotates about pivot_frac * chord from the LE.
    "main_actuation": "flap", "main_flap_frac": 0.4, "main_pivot_frac": 0.22,
    # The flap covers only the INBOARD half of each main-foil half-span (per the MAIN_FOIL_FLAP_EFFECTIVENESS
    # comment in forces/foil.py), so the hinge carries main_span * main_flap_span_frac of flap.
    "main_flap_span_frac": 0.5,
    "elev_actuation": "all_moving", "elev_pivot_frac": 0.22,   # PLACEHOLDER pivot just ahead of the AC (weathervane-stable)
    "main_surface_inertia": 4e-4, "elev_surface_inertia": 4e-4, # kg m^2 about the hinge, PLACEHOLDER (composite)
    "gear_ratio": 20.0,
    "v_bus": 40.0,                       # moteus supply [V] (10-44 V allowed); battery -> PDB -> motors is 40 V
    "supply_i_max": 25.0,                # PDB output limit [A] shared by everything on the rail
    # "servo": drives with dynamics, current/voltage limits and hinge moments in the loop (default);
    # "kinematic": legacy delay -> rate limit -> lag on the virtual channels (control/actuator.py).
    "actuator_model": "servo",
    "servo_params": None,                # overrides of actuation.servo.DEFAULT_SERVO_PARAMS (kp, kd, torque_cap, delay_s, ...)
    "flap_stop_deg": 30.0, "elev_stop_deg": 20.0,   # mechanical stops of the PHYSICAL surfaces, PLACEHOLDER
    "flap_cmd_max_deg": 20.0,                        # control-surface aerodynamic command ceiling [deg] (reversible/gated inside stop)
    "allocator": "jacobian",                         # "jacobian" (physics-constrained 3-var QP) or "legacy" (fixed angle mixer)
    "safety_margin": "standard",                     # "conservative" | "standard" | "sport" (scales Layer 2/3 reserves)
    "reverse_drive_enabled": True,                   # allow stability-gated opposing front flap deflections for roll/turn moment
    "split_flap_cross_coupling": 0.08,               # centerline root-vortex wash factor when left != right flap [0 = independent]
    "cavitation_model": True,                        # enable physical cavity lift loss + drag when -Cp_min > sigma
    "wave_type": "regular",                          # "regular" (monochromatic Airy) or "irregular" (JONSWAP directional sea)
    "q_schedule_speed": 0.0,             # >0: scale flap commands by (v/u)^2 above this speed (gain scheduling on q); 0 = off
    "prop_tau_s": 0.08,                  # motor/ESC/prop spool lag [s]
    "prop_power_max": 300.0,             # shaft thrust power limit [W] (caps thrust at high speed)
    "dual_height": True,                 # dual redundant ranging sensors + health voting
    "gnss_fallback": True,               # GNSS vertical-velocity fallback if both height channels fail
}


# Manned boat, 250-300 kg all-up (boat + rider), sized 2026-11-15 (docs/19, "Manned sizing"). Same layout as the
# model (T-foil, split flapped main foil at the CG, all-moving elevator aft), scaled: the main foil is 0.62 m^2
# (2 x 0.28 x 1.1 m) at 3 deg incidence so the trim flap is near neutral at the 8 m/s cruise and takeoff is
# about 5 m/s. The CG sits in the hull, 0.35 m above the hull bottom (hull_offset < 0), the foil 0.9 m under
# the surface and the hull 0.5 m clear of it when flying (1.4 m struts). Inertias, hull, damping, thrust and
# drive figures are PLACEHOLDERS scaled from the model, not measured.
MANNED_PARAMS: dict[str, Any] = dict(DEFAULT_PARAMS)
MANNED_PARAMS.update({
    "mass": 275.0,
    "ixx": 80.0, "iyy": 260.0, "izz": 280.0,
    # what the 275 kg is made of, for the boat editor (hydrofoil/design.py): the empty boat plus these point masses,
    # positions relative to the nominal CG [m] (x forward, y starboard, z up). PLACEHOLDER rider position.
    "default_payloads": [{"name": "Rider", "mass": 75.0, "x": -0.43, "y": 0.0, "z": 0.33}],
    "heave_damping": 600.0,
    "main_chord": 0.28, "main_span": 1.1, "main_lateral_offset": 0.60,
    "main_incidence_deg": 3.0,
    # Balanced flap (docs/20, adopted 2026-11-16): 25 % chord over the FULL half-span with the hinge set back 15 % of
    # the flap chord (thin theory overbalances near 33 %; 0.15 keeps more than 2x margin). Against the old 40 % chord,
    # half-span plain flap: about 4x less hinge moment and 3x less drive current in waves, deflections inside +-13 deg.
    "main_flap_frac": 0.25, "main_flap_span_frac": 1.0, "main_flap_setback": 0.15,
    # Drive train compliance and flap twist (docs/20 section 7). PLACEHOLDERS until the gearbox and flap are chosen:
    # flap GJ of a 70 x 13 mm carbon closed section (1.5 mm wall), elevator GJ of a 170 x 15 mm section (2 mm wall).
    "main_flap_GJ": 250.0, "elev_GJ": 1000.0,
    # Takeoff steps the flap to its 20 deg limit; unshaped that is 550-610 deg/s and saturates the drives' torque
    # (48 A, 17-21 A rail). Cruise in waves needs at most about 40 deg/s.
    "cmd_slew_deg_s": 150.0,
    "main_pos_x": 0.0, "main_pos_z": 1.75,
    "elev_chord": 0.17, "elev_span": 0.5, "elev_pos_x": -2.0, "elev_pos_z": 1.65,
    "main_strut_chord": 0.16, "elev_strut_chord": 0.12,
    "hull_offset": -0.35, "flying_height": 0.5,
    "hull_length": 3.6, "hull_beam": 1.2, "hull_x": 0.0, "max_draft": 0.3,
    "hull_drag_coeff": 120.0, "hull_rot_damping": 6000.0,
    "target_speed": 8.0, "max_thrust": 1000.0,
    "kp_speed": 360.0, "ki_speed": 60.0,
    # flap structural inertia about its hinge scales as span x c_f^3; the 9e-3 was for the 40 % half-span flap. Balanced
    # flap: x 0.488 for the geometry, x 0.617 for the axis 15 % into the flap -> 2.7e-3 (PLACEHOLDER until CAD)
    "main_surface_inertia": 2.7e-3, "elev_surface_inertia": 6e-3,
    # hinge moments reach 12-13 N m (takeoff, waves): the 10 N m firmware cap left the flap short and the boat
    # rolled over with a 10 cm lateral CG; 28 N m is just under the mj5208 x 20:1 peak (1.7 x 20 x 0.85)
    # Continuous drive current used by the sizing checks. The moteus runs above its 12 A no-heat-sink figure (20-32 A
    # quoted with heat sinking; the drives sit in water). PLACEHOLDER: the firmware co-dependency limit (coupled
    # current / torque / rail / temperature limits across the drives) will set the real value.
    "drive_i_cont": 20.0,
    # Precision low-backlash preloaded planetary stage (6 arcmin) + dual encoder feedback; 15 mm 316SS shaft gives >1.6x yield margin.
    "servo_params": {"torque_cap": 28.0, "backlash_deg": 0.10, "gear_stiffness": 5000.0, "encoder": "dual"},
    "shaft_diameter": 0.015,
    "prop_tau_s": 0.12, "prop_power_max": 6500.0,
    "dual_height": True, "gnss_fallback": True,
    # heave filter for 8 m/s: encounter frequencies are lower (following / quartering 15 m waves about 1-1.5 rad/s);
    # a slower, larger wave band and a slower bias trade 13 mm calm wander (6 mm at the defaults) for 67-117 mm
    # instead of 90-260 mm rms in following seas (docs/19, "Manned sizing")
    "kf_wave": {"w0": 2.0, "zeta": 1.0, "sigma": 0.15}, "kf_heave": {"bias_walk": 0.005},
    "trim_ff_deg": 4.11, "trim_ff_speed": 6.0,   # = trim flap at 6 m/s for the balanced flap (was 6.57 on the 40 % flap)
    # The geometry the PID gains and LQR weights below were autotuned on; hydrofoil/consistency.py warns when the
    # boat no longer matches (re-run hydrofoil.tuning.autotune and update this).
    # 2026-11-16, balanced flap + compliant drives + slew limit: both sets re-tuned / re-checked on this geometry. The
    # LQR search (9.55 -> 9.16) only won on ideal sensors and was WORSE with noisy ones (25 -> 28 mm, flap rate 31 ->
    # 47 deg/s), so the LQR weights were kept; the PID gains were re-tuned with the supervised takeoff. docs/20 s8.
    # the autotuner also flies the supervised takeoff (tuning/scoring.py): the 2026-11-16 PID search, which lacked
    # it, found gains that stall the supervised takeoff on the drag hump
    "supervisor_tuning": True,
    "tuned_for": {"main_flap_frac": 0.25, "main_flap_span_frac": 1.0, "main_flap_setback": 0.15, "gear_ratio": 20.0},
    # PID gains from the autotuner (2026-11-15): these fly takeoff (and everything in PID mode). The relay-test rule
    # sets (Ziegler-Nichols etc.) scored best with perfect sensors but held the drives at their torque limit on noisy
    # ones; with drive effort charged properly the search kept gains near the model's. Following sea 95 -> 64 mm rms,
    # rider shift peak 183 -> 113 mm, takeoff settle 15.1 -> 13.8 s.
    # 2026-11-16, balanced flap: autotuned WITH the supervised takeoff scenario (tuning/scoring.py), cost 23.4 -> 18.2;
    # every scenario improves (takeoff settle 8.6 -> 7.4 s, following 61 -> 48 mm, noisy head 48 -> 40 mm rms, rider
    # shift 36 -> 17 mm) at +8 % flap rate on noisy sensors. Was kp/ki/kd_heave 44.94 / 6.488 / 7.906, pitch 1.818 /
    # 1.038 / 0.0987, roll 2.0 / 0.191. The search WITHOUT that scenario had found 340 / 264 / 19, which aborts the
    # supervised takeoff on the drag hump (docs/20 section 8). The classical rule sets all abort it too.
    "kp_heave": 62.38, "ki_heave": 6.015, "kd_heave": 6.793,
    "kp_pitch": 3.019, "ki_pitch": 1.447, "kd_pitch": 0.0920,
    "kp_roll": 1.915, "kd_roll": 0.2135,
    # LQR weights from the autotuner (hydrofoil/tuning, docs/19 "Autotuner"), 2026-11-15: Bryson's rule, 4 rounds of
    # re-weighting, then 6 rounds of search over 5 scenarios incl. noisy sensors. Acceptable excursions, overriding
    # control.lqr.DEFAULT_WEIGHTS: tighter on height, heave rate and the height integral, looser on roll and pitch
    # rate, and a 2.3x larger command-rate allowance. Head / following sea height rms 27 / 36 -> 18 / 18 mm.
    "lqr_weights": {"z": 0.01084, "phi": 0.07882, "theta": 0.0348, "w": 0.08196, "q": 0.2391, "iz": 0.01292,
                    "du_main": 566.2, "du_elev": 566.2, "du_roll": 566.2},
    "lqr_speeds": (4.5, 5.5, 6.5, 8.0, 9.5), "lqr_engage_speed": 4.8, "est_engage_speed": 4.5,
})


# Our boat as drawn in CAD (SolidWorks "Hydrofoil Assembly+wings_not_finished" and "Split Wing Assembly", measured
# from their STL exports on 2026-11-24; full measurement notes in docs/26-cad-boat-preset.md). Starts from the
# manned preset (controller gains, drives, estimator), then replaces every dimension the CAD defines.
#
# MEASURED from CAD: wing/elevator geometry and positions, NACA 0009 sections (9.0 % thick), 0 deg mounting on
# both wings, flap notch (inboard 0-240 mm each side, 100 mm of the 300 mm chord), two front masts at +-0.35 m,
# one rear mast, twin pontoons, propeller position.
# ASSUMED (not in the STL, which has no mass): mass 275 kg and inertias (manned preset), CG longitudinally over the
# main wing's quarter chord and 0.35 m above the pontoon bottoms, elevator pivot and both surfaces' travel limits.
# Body frame origin = CG. CAD -> body: x = (X_cad - 1716) mm forward, z_down = (863 - Y_cad) mm.
CAD_PARAMS: dict[str, Any] = dict(MANNED_PARAMS)
CAD_PARAMS.update({
    "visual": "cad",                       # viewer draws the CAD layout (hydrofoil/cad_geometry.py)
    # Main wing: 1.353 m span, 0.3665 m^2 with the flaps in, AR 5.0, rounded tips. Each half is modelled as one
    # rectangular strip of the same area (0.6765 m x 0.271 m mean chord) whose lift acts at the half's area
    # centroid, 0.31 m from the centreline.
    "main_chord": 0.271, "main_span": 0.6765, "main_lateral_offset": 0.31,
    "main_incidence_deg": 0.0,             # measured 0.0 deg; the dashboard can demonstrate 0-5 deg
    # Flaps: two independent plain flaps meeting at the centreline, each 0.24 m of span (35.5 % of the half) and
    # 100 mm of the local 300 mm chord. Their extra lift acts 0.12 m from the centreline (main_flap_center_y), not at
    # the half's centroid, which cuts roll authority to about 0.39x of a centred flap.
    "main_flap_frac": 0.333, "main_flap_span_frac": 0.355, "main_flap_setback": 0.0, "main_flap_center_y": 0.12,
    "tuned_for": {"main_flap_frac": 0.25, "main_flap_span_frac": 1.0, "main_flap_setback": 0.15, "gear_ratio": 20.0},
    # Positions relative to the CG: wing quarter chord 1.099 m below, elevator quarter chord 2.391 m aft and 1.118 m
    # below (0.149 x 0.677 m, all-moving). Hull bottom 0.35 m below the CG, so the struts are 0.749 m long.
    "main_pos_x": 0.0, "main_pos_z": 1.099,
    "elev_chord": 0.149, "elev_span": 0.677, "elev_pos_x": -2.391, "elev_pos_z": 1.118,
    "elev_pivot_frac": 0.22,               # PLACEHOLDER: the rear mast sits at 49 % chord; pivot not drawn yet
    # Two streamlined front masts (0.28 m chord, 51 mm thick) at +-0.35 m, one 0.10 m rear mast on the centreline.
    "main_strut_chord": 0.28, "main_strut_count": 2, "main_strut_y": 0.35, "elev_strut_chord": 0.10,
    # Twin pontoons: 2.54 m long, 265 mm diameter, 1.308 m apart, centred 0.483 m aft of the CG. Modelled as
    # wall-sided hulls of equal volume (beam = pi/4 x 0.265 m) with the full diameter as draft.
    "hull_offset": -0.35, "hull_length": 2.54, "hull_beam": 0.208, "hull_twin_spacing": 1.308, "hull_x": -0.483,
    "max_draft": 0.265,
    # Cockpit: a sealed 48 x 32 x 24 in box (1.219 x 0.813 x 0.610 m, 0.605 m^3, ~620 kg of buoyancy when fully
    # under) centred in the frame between the pontoons, its bottom level with the pontoon bottoms. 1.219 m along the
    # boat. Its water drag is a flat-faced box (C_D 1) on whatever is submerged.
    "hull_boxes": [{"name": "cockpit", "x": -0.347, "y": 0.0, "length": 1.219, "beam": 0.813, "height": 0.610}],
    # Wing 0.40 m (1.5 chords) under the surface when the pontoons are 0.35 m clear.
    "flying_height": 0.35,
    # Two propellers (about 0.21 m diameter) on the front masts, 0.386 m aft of and 0.96 m below the CG: the thrust
    # line pitches the nose up by thrust x 0.96 m. The sim applies the total thrust on the centreline.
    "prop_pos_z": 0.96, "prop_count": 2,
    "default_payloads": [{"name": "Rider", "mass": 75.0, "x": -0.35, "y": 0.0, "z": 0.0}],
    # Cruise target: the team's planned 8 m/s (as the manned sizing). NOT tuned to make this geometry fly.
    "target_speed": 8.0,
    # Trim schedule DERIVED for the 0 deg wing by hydrofoil.design._reschedule (flap inside +-15 deg): level flight
    # first trims at 15.5 m/s, so the LQR only exists there. Re-derived live when the mounting angle changes.
    "lqr_speeds": (15.5, 15.62, 15.75, 15.88, 16.0), "lqr_engage_speed": 15.8, "est_engage_speed": 15.5,
    "trim_ff_deg": 13.54, "trim_ff_speed": 16.0,
})

PRESETS: dict[str, dict[str, Any]] = {"model": DEFAULT_PARAMS, "manned": MANNED_PARAMS, "cad": CAD_PARAMS}
PRESET_LABELS = {"cad": "Our boat (CAD)", "manned": "Manned (Josh's sizing)", "model": "25 kg model"}


def preset_params(name: str) -> dict[str, Any]:
    """A fresh copy of the named preset ("model", "manned" or "cad"); unknown names give the model."""
    return dict(PRESETS.get(str(name), DEFAULT_PARAMS))
