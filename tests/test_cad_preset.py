"""CAD boat preset (docs/26): part-span flap position, twin pontoons, live mounting angle, LQR fallback, viewer model."""

import warnings

import numpy as np
import pytest

from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables
from hydrofoil.forces.hull import HullHydrostatics
from hydrofoil.params import CAD_PARAMS, PRESETS, preset_params
from hydrofoil.vehicle import VehiclePhysics

NU = np.array([8.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def _foil(dy=0.0, y=-0.31):
    return Foil(FoilGeometry(chord=0.271, span=0.6765), FoilTables.naca0009(), r_attach=np.array([0.0, y, 1.1]),
                flap_effectiveness=0.25, flap_dy=dy)


def test_flap_increment_acts_at_flap_centroid():
    """The flap's extra lift makes roll moment about the flap's own lever arm, not the panel's."""
    base, moved = _foil(0.0), _foil(0.19)            # panel at y = -0.31, flap centroid at -0.12
    d_base = base.generalized_force(NU, delta=8.0, submergence_over_chord=3.0) - base.generalized_force(NU, 0.0, 3.0)
    d_moved = moved.generalized_force(NU, delta=8.0, submergence_over_chord=3.0) - moved.generalized_force(NU, 0.0, 3.0)
    assert np.allclose(d_base[:3], d_moved[:3])                       # same force
    assert d_moved[3] == pytest.approx(d_base[3] * 0.12 / 0.31, rel=1e-6)   # roll moment scales with the lever arm
    # flap at zero: no change from the lumped model at all
    assert np.allclose(moved.generalized_force(NU, 0.0, 3.0), base.generalized_force(NU, 0.0, 3.0))


def test_flap_offset_derivative_matches_finite_difference():
    f = _foil(0.19)
    d = f.local_derivatives(NU, delta=5.0, submergence_over_chord=3.0)["dtau_ddelta"]
    h = 1e-4
    fd = (f.generalized_force(NU, 5.0 + h, 3.0) - f.generalized_force(NU, 5.0 - h, 3.0)) / (2 * h)
    assert np.allclose(d, fd, rtol=2e-3, atol=1e-6)


def test_twin_pontoons_float_at_volume_draft():
    hull = HullHydrostatics(2.54, 0.208, 0.0, 0.35, 0.265, twin_spacing=1.308)
    assert hull.waterplane_area == pytest.approx(2 * 2.54 * 0.208)
    assert set(np.round(np.unique(np.abs(hull.xy[:, 1])) // 0.5, 0)) == {1.0}   # all cells out on the pontoons
    eta = np.array([0.0, 0.0, 0.35 + 0.10, 0.0, 0.0, 0.0])           # CG 0.45 m deep, hull bottom 0.35 above: 0.10 m draft
    tau = hull.buoyancy(eta, 0.0)
    assert -tau[2] == pytest.approx(1025 * 9.81 * hull.waterplane_area * 0.10, rel=1e-9)
    assert abs(tau[3]) < 1e-6 and abs(tau[4]) < 1e-6
    mono = HullHydrostatics(2.54, 2 * 0.208, 0.0, 0.35, 0.265)
    heel = eta.copy(); heel[3] = np.radians(2.0)
    assert abs(hull.buoyancy(heel, 0.0)[3]) > 5 * abs(mono.buoyancy(heel, 0.0)[3])    # catamaran: far stiffer in roll (~8.7x)


def test_cad_preset_uses_measured_geometry():
    p = CAD_PARAMS
    assert PRESETS["cad"] is CAD_PARAMS and preset_params("cad") is not CAD_PARAMS
    assert p["main_incidence_deg"] == 0.0
    assert 2 * p["main_chord"] * p["main_span"] == pytest.approx(0.3665, abs=2e-3)   # CAD planform area
    assert p["hull_offset"] + p["main_pos_z"] == pytest.approx(0.749, abs=1e-3)      # strut length
    assert p["main_strut_count"] == 2 and p["hull_twin_spacing"] > 0


def test_live_mounting_angle_changes_lift():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp = VehiclePhysics(dict(CAD_PARAMS, controller="pid"))
    f0 = vp.left_foil.generalized_force(NU, 0.0, 3.0)
    vp.set_main_incidence(4.0)
    assert vp.params["main_incidence_deg"] == 4.0 and vp.right_foil.geometry.aoa_offset == 4.0
    assert -vp.left_foil.generalized_force(NU, 0.0, 3.0)[2] > -f0[2] + 50.0   # more lift (NED z up = negative)


def test_lqr_falls_back_to_pid_when_boat_cannot_trim():
    p = dict(CAD_PARAMS, controller="auto", lqr_speeds=(5.0, 6.0, 7.0))     # far below the 0 deg wing's flying speed
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp = VehiclePhysics(p)
    assert vp.lqr is None and vp.lqr_error
    assert "unavailable" in vp.controller_status(6.0)["reason"]
    from hydrofoil.waves.airy import CalmWater
    state, *_ = vp.step(vp.rest_state(), 0.0, CalmWater())           # still steps on the PID loops
    assert np.all(np.isfinite(state))


def test_viewer_model_compiles_for_every_preset():
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco.viewer")
    from hydrofoil.viewer import build_mjcf, SLIDERS
    for name in PRESETS:
        m = mujoco.MjModel.from_xml_string(build_mjcf(preset_params(name)))
        for j in ("main_mount", "left_foil_hinge", "right_foil_hinge", "elev_hinge"):
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0
    inc = [s for s in SLIDERS if s[1] == "main_incidence_deg"]
    assert inc and inc[0][2] == 0.0 and inc[0][3] == 5.0


def test_dashboard_offers_the_cad_boat():
    from hydrofoil import sim_server
    assert sim_server.FIELDS["preset"][1] == "cad"
    info = sim_server.design_info("cad")
    assert info["base"]["main_incidence_deg"] == 0.0 and info["geom"]["hull_twin_spacing"] > 0
    assert 'value="cad"' in sim_server.page() and 'id="loadCad"' in sim_server.page()


def test_cockpit_box_buoys_to_its_own_height_and_drags_when_wet():
    box = {"name": "cockpit", "x": 0.0, "y": 0.0, "length": 1.219, "beam": 0.813, "height": 0.610}
    hull = HullHydrostatics(2.54, 0.208, 0.0, 0.35, 0.265, twin_spacing=1.308, boxes=[box])
    assert hull.waterplane_area == pytest.approx(2 * 2.54 * 0.208 + 1.219 * 0.813)
    deep = np.array([0.0, 0.0, 0.35 + 0.50, 0.0, 0.0, 0.0])          # 0.50 m deep: pontoons under, box not yet
    f_box = 1025 * 9.81 * 1.219 * 0.813 * 0.50
    f_pont = 1025 * 9.81 * 2 * 2.54 * 0.208 * 0.265
    assert -hull.buoyancy(deep, 0.0)[2] == pytest.approx(f_box + f_pont, rel=1e-9)
    assert hull.mean_draft(deep, 0.0) == pytest.approx(0.265)        # main-hull draft only
    nu = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert hull.box_drag(nu, deep, 0.0)[0] == pytest.approx(-0.5 * 1025 * 0.813 * 0.50 * 9.0)
    dry = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])                    # hull 0.35 m above the water
    assert np.allclose(hull.box_drag(nu, dry, 0.0), 0.0)


def test_cad_preset_has_the_cockpit_box():
    b = CAD_PARAMS["hull_boxes"][0]
    assert (b["length"], b["beam"], b["height"]) == pytest.approx((48 * 0.0254, 32 * 0.0254, 24 * 0.0254), abs=1e-3)


def test_cockpit_toggle_off_scales_pontoons_to_hold_the_boat():
    from hydrofoil.params import resolve_cad_hull
    on = resolve_cad_hull(dict(CAD_PARAMS, cockpit_buoyancy=True))
    off = resolve_cad_hull(dict(CAD_PARAMS, cockpit_buoyancy=False))
    assert on["max_draft"] == pytest.approx(0.265) and len(on["hull_boxes"]) == 1
    assert off["hull_boxes"] == []
    cap = 2 * np.pi * (off["max_draft"] / 2) ** 2 * off["hull_length"] * 1025.0
    assert cap == pytest.approx(off["pontoon_reserve"] * off["mass"], rel=1e-6)   # pontoons alone hold 2x the boat
    assert resolve_cad_hull(dict(off))["max_draft"] == off["max_draft"]            # idempotent
    heavy = resolve_cad_hull(dict(CAD_PARAMS, cockpit_buoyancy=False, mass=400.0))
    assert heavy["max_draft"] > off["max_draft"]                                    # follows the mass
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp = VehiclePhysics(dict(CAD_PARAMS, cockpit_buoyancy=False, controller="pid"))
    assert vp.hull.boxes == [] and vp.hull.max_draft == pytest.approx(off["max_draft"])


def test_cockpit_toggle_reaches_viewer_and_dashboard():
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("mujoco.viewer")
    from hydrofoil.params import resolve_cad_hull
    from hydrofoil.viewer import build_mjcf
    from hydrofoil import sim_server
    m = mujoco.MjModel.from_xml_string(build_mjcf(resolve_cad_hull(dict(CAD_PARAMS, cockpit_buoyancy=False))))
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pontoon_0")
    assert m.geom_size[g][0] == pytest.approx(0.5 * 0.3672, abs=2e-3)
    assert 'name="cockpit_buoyancy"' in sim_server.page()


def test_cad_integration_fixes_and_top_down_ui():
    import argparse
    import inspect
    from hydrofoil.actuation.loads import surface_models
    from hydrofoil.control.allocator import ControlAllocator
    from hydrofoil.design import apply_design
    from hydrofoil.params import resolve_cad_hull
    from hydrofoil import sim_server, viewer
    from hydrofoil.waves.airy import AiryWave, CalmWater

    # (a) Thread-safe _apply_flap_dy does not mutate self.flap_dy
    f = _foil(0.19)
    f._eval_force_and_deriv(NU, delta=5.0, submergence_over_chord=3.0, _apply_flap_dy=False)
    assert f.flap_dy == pytest.approx(0.19)

    # (b) Real dual front struts at +-main_strut_y produce extra yaw damping and feel oblique waves asymmetrically
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp = VehiclePhysics(dict(CAD_PARAMS, controller="pid"))
        vp_single = VehiclePhysics(dict(CAD_PARAMS, main_strut_count=1, controller="pid"))
    assert vp.main_strut_l.r_attach[1] == pytest.approx(-CAD_PARAMS["main_strut_y"])
    assert vp.main_strut_r.r_attach[1] == pytest.approx(+CAD_PARAMS["main_strut_y"])
    env_calm = vp.calm_sample()
    d_zero = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    ds0 = vp.physics_rhs(vp.rest_state(), d_zero, 0.0, env_calm)
    s_yaw = np.zeros(12)
    s_yaw[2] = vp.hull_offset - 0.3
    s_yaw[6] = 8.0
    s_yaw[11] = 0.5
    ds_dual = vp.physics_rhs(s_yaw, d_zero, 0.0, env_calm)
    ds_single = vp_single.physics_rhs(s_yaw, d_zero, 0.0, vp_single.calm_sample())
    assert ds_dual[11] < ds_single[11] and np.all(np.isfinite(ds0))
    wave_beam = AiryWave(amplitude=0.12, wavelength=4.0, direction=np.radians(90.0))
    w_env = vp.wave_sample(s_yaw[:6], 0.0, wave_beam)
    assert not np.allclose(w_env.foil_vel_body["main_strut_l"], w_env.foil_vel_body["main_strut_r"])

    # (c) main_flap_chord sizes actuator hinge torque directly
    models = surface_models(CAD_PARAMS)
    assert models["left"].g.chord * models["left"].g.flap_frac == pytest.approx(CAD_PARAMS["main_flap_chord"])

    # (d) KalmanEstimator initializes gracefully even when CAD 0 deg has few/no trimmed speeds
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp_noisy = VehiclePhysics(dict(CAD_PARAMS, sensors="noisy", controller="auto"))
    s_n, *_ = vp_noisy.step(vp_noisy.rest_state(), 0.0, CalmWater())
    assert np.all(np.isfinite(s_n))

    # (e) Smooth set_main_incidence analytical trim_ff update shifts trim_ff_deg without saturating at 9.0 m/s
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp_inc = VehiclePhysics(dict(CAD_PARAMS, controller="pid"))
    ff_before = vp_inc._trim_ff(9.0)
    vp_inc.set_main_incidence(5.0)
    assert vp_inc.params["trim_ff_deg"] < CAD_PARAMS["trim_ff_deg"] - 20.0
    assert -15.0 < vp_inc._trim_ff(9.0) < ff_before

    # (f) design.py preserves explicit hull_beam override and scales inboard flap arm proportionally
    base_cad = resolve_cad_hull(dict(CAD_PARAMS))
    beam_ed = apply_design(base_cad, {"hull_beam": 0.40})
    assert beam_ed["hull_beam"] == pytest.approx(0.40)
    wide_ed = apply_design(base_cad, {"main_span": base_cad["main_span"] * 1.5})
    assert wide_ed["main_flap_center_y"] == pytest.approx(base_cad["main_flap_center_y"] * 1.5, rel=1e-4)

    # (g) ControlAllocator weights roll with main_flap_center_y and emits cp_min_max & hc_min
    assert "main_flap_center_y" in inspect.getsource(ControlAllocator.allocate)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        vp_c = VehiclePhysics(dict(CAD_PARAMS, main_incidence_deg=4.0, controller="pid"))
    s_c = np.zeros(12)
    s_c[2] = vp_c.hull_offset - 0.35
    s_c[6] = 8.0
    vp_c.step(s_c, 0.0, CalmWater())
    alloc_st = getattr(vp_c, "_last_alloc_status", {}) or {}
    assert "cp_min_max" in alloc_st and "hc_min" in alloc_st

    # (h) Viewer CLI defaults to cad, design_preview forwards cockpit_buoyancy, load_cad preserves form
    assert sim_server.FIELDS["preset"][1] == "cad"
    src_main = inspect.getsource(viewer.main)
    assert 'default="cad"' in src_main
    prev = sim_server.design_preview("cad", {"cockpit_buoyancy": False})
    assert prev["edited_geom"]["cockpit_buoyancy"] is False
    assert prev["edited_geom"]["hull_boxes"] == []

    # (i) Modernized UI includes top-down tactical/oblique wave canvas and secondary telemetry readouts
    html = sim_server.page()
    for hook in ('id="topView"', "drawTopTactical", 'id="gd"', 'id="gcav"', 'id="gwav"', 'id="gnav"'):
        assert hook in html
