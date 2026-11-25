"""
The hydrofoil simulator app: native MuJoCo window with live 3-D view, sliders, plots and actuator loads.
(Replaces both the old Unity visualizer and the browser dashboard.)

MuJoCo is used purely as a renderer and UI: hydrofoil.vehicle.VehiclePhysics remains the only
physics authority. Each frame we write its pose into the MuJoCo model (qpos) and call
mj_forward -- mj_step is never called.

Run:
    python -m hydrofoil.viewer [--waves] [--wave-dir 90] [--speed 1.0] [--duration 600]
                               [--controller lqr|pid] [--sensors ideal|noisy] [--preset model|manned]
                               [--supervisor] [--windowed] [--api-port 8766]

The window opens maximised (Windows) with MuJoCo's own menus hidden: 3-D view, a status panel
(bottom left; shortened in small windows), the flight mode (top left) and four rolling plots (right).
The water is a heightfield following the boat, refilled from the wave field every frame.

Live controls: a local HTTP API on 127.0.0.1:<api-port> (GET /state, GET /sliders, POST /set) that a
dashboard page can use (`python -m hydrofoil.sim_server` serves one on :8765). Slider values are kept in
zero-gain MuJoCo actuators (ctrl), shared by the keys and the API, and read every frame:
    height_ref, bank_ref, speed_ref                 pilot references (bank holds when set here)
    wave_amp, wave_len, wave_dir                    Airy wave (amp 0 = calm); applied live
    kp/ki/kd_heave, kp/kd_pitch, kp/kd_roll         PID gains, applied live (no restart)
    gear_ratio, v_bus                               drive model used for the load read-out

Keys (click the viewer window first):
    Up / Down        climb / dive   (height_ref, holds)
    Left / Right     bank left / right (bank_ref, +-15 deg, springs back to 0)
    Space            level wings immediately (bank_ref -> 0 deg)
    [  /  ] (- / +)  speed_ref down / up (holds)
    R                reset to rest
    C                cycle controller mode: Auto Combined -> LQR -> PID
    F                toggle Foil / Land request (or cruise / land height without --supervisor)
    U                cycle speed units: m/s -> km/h -> mph
    D                open / bring up the web dashboard (http://127.0.0.1:8765)

The PID gain sliders act on the PID loops, which fly takeoff in both modes and everything in PID mode.

Per drive the status panel shows the hinge moment, motor current and bus power (docs/17).
With actuator_model = "servo" (default) these are the simulated drives' own values and the drives'
limits act on the flight; with "kinematic" they are estimated from the flap trajectory.

Frames: the sim is NED/body (x fwd, y stbd, z down). MuJoCo is Z-up, so world
coordinates are mapped with D = diag(1,-1,-1) (a proper 180 deg rotation about x):
    p_mj = D p_ned,   R_mj = D R_ned B   with B = diag(1,-1,-1) mapping the
MuJoCo body frame (x fwd, y left, z up) onto the sim's body frame.
"""

from __future__ import annotations

from pathlib import Path

import argparse
import json
import time
from collections import deque

import numpy as np
import mujoco
import mujoco.viewer

from hydrofoil.params import PRESETS, preset_params
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.waves.airy import AiryWave, CalmWater, IrregularWave
from hydrofoil.actuation.drive import DriveSpec, electrical_demand
from hydrofoil.actuation.loads import DRIVES, surface_models, drive_angles, steady_moments

# Pilot limits -- keep identical to the verified/capped ranges used elsewhere.
DASHBOARD_URL = "http://127.0.0.1:8765"
HEIGHT_MIN, HEIGHT_MAX, HEIGHT_STEP = -0.3, 1.0, 0.02   # m, per key event
SPEED_MIN, SPEED_MAX, SPEED_STEP = 0.0, 16.0, 0.25       # m/s, per key event (the CAD boat at 0 deg needs ~15 m/s)
BANK_MIN, BANK_MAX, BANK_STEP = -15.0, 15.0, 1.0         # deg, per key event (supervisor enforces dynamic safe_bank_limit)
BANK_RETURN_RATE = 10.0                                   # deg/s spring-back

# UI sliders: (name, param key or None, lo, hi). Order = MuJoCo actuator / ctrl index.
SLIDERS = [
    ("height_ref", "flying_height", HEIGHT_MIN, HEIGHT_MAX),
    ("bank_ref", "phi_ref", BANK_MIN, BANK_MAX),
    ("speed_ref", "target_speed", SPEED_MIN, SPEED_MAX),
    ("wave_amp", "wave_amplitude", 0.0, 0.30),
    ("wave_len", "wave_wavelength", 1.0, 20.0),
    ("wave_dir", "wave_direction", -180.0, 180.0),
    ("kp_heave", "kp_heave", 0.0, 100.0),
    ("ki_heave", "ki_heave", 0.0, 20.0),
    ("kd_heave", "kd_heave", 0.0, 30.0),
    ("kp_pitch", "kp_pitch", 0.0, 5.0),
    ("kd_pitch", "kd_pitch", 0.0, 1.0),
    ("kp_roll", "kp_roll", 0.0, 10.0),
    ("kd_roll", "kd_roll", 0.0, 1.0),
    ("gear_ratio", "gear_ratio", 5.0, 50.0),
    ("v_bus", "v_bus", 10.0, 44.0),
    ("main_incidence", "main_incidence_deg", 0.0, 5.0),   # main wing mounting angle [deg], live (docs/26)
]
IDX = {name: i for i, (name, *_rest) in enumerate(SLIDERS)}

WATER_N, WATER_HALF, WATER_RANGE = 128, 30.0, 0.6   # water heightfield: grid points, half-size [m], +- elevation [m]

_D = np.diag([1.0, -1.0, -1.0])   # NED <-> Z-up world
_B = np.diag([1.0, -1.0, -1.0])   # sim body frame <-> MuJoCo body frame


def _hull_vertices(L: float, B: float, D: float) -> str:
    """A simple planing hull as a convex point set (MuJoCo takes the convex hull): square transom, hard chines,
    a raked bow. Origin at the middle of the bottom, x forward, z up."""
    pts = [(-L / 2, y * B / 2, D) for y in (-1, 1)] + [(-L / 2, y * 0.36 * B, 0.0) for y in (-1, 1)]
    pts += [(0.18 * L, y * B / 2, D) for y in (-1, 1)] + [(0.18 * L, y * 0.36 * B, 0.0) for y in (-1, 1)]
    pts += [(0.38 * L, y * 0.33 * B, D) for y in (-1, 1)] + [(0.30 * L, y * 0.16 * B, 0.12 * D) for y in (-1, 1)]
    pts += [(L / 2, 0.0, 1.08 * D), (0.44 * L, 0.0, 0.3 * D)]
    return " ".join(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in pts)


def build_mjcf(p: dict) -> str:
    """Generate the MJCF for hull + struts + main foil halves (with flaps) + elevator + UI sliders."""
    hull_off = p["hull_offset"]
    mc, ms = p["main_chord"], p["main_span"]
    ec, es = p["elev_chord"], p["elev_span"]
    lat = p["main_lateral_offset"]
    mx, mz = p["main_pos_x"], p["main_pos_z"]
    ex, ez = p["elev_pos_x"], p["elev_pos_z"]
    flap_frac = p.get("main_flap_frac", 0.4)

    fs = ms * p.get("main_flap_span_frac", 1.0)      # flap span (inboard part of each half)
    cad = None
    if p.get("visual") == "cad":
        from hydrofoil.cad_geometry import cad_parts
        cad = cad_parts(p)

    def half(name, y):
        # y is the half's centre (MuJoCo y = left); the flap sits on the INBOARD side (toward y = 0)
        inboard = -1.0 if y > 0 else 1.0
        fy = inboard * (ms - fs) / 2                   # flap centre relative to the half's centre
        oy = -inboard * fs / 2                         # fixed trailing-edge strip on the outboard part
        return f"""
        <body name="{name}" pos="0 {y} 0">
          <geom type="box" size="{mc*(1-flap_frac)/2} {ms/2} 0.008" pos="{mc*flap_frac/2} 0 0" material="foil"/>
          <geom type="box" size="{mc*flap_frac/2} {(ms-fs)/2 + 1e-4} 0.008" pos="{-mc*(1-flap_frac)/2} {oy} 0" material="foil"/>
          <body name="{name}_flap" pos="{-mc*(1-flap_frac)/2 + mc*flap_frac/2} {fy} 0">
            <joint name="{name}_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
            <geom type="box" size="{mc*flap_frac/2} {fs/2} 0.008" pos="{-mc*flap_frac/2} 0 0" material="flap"/>
          </body>
        </body>"""

    # Zero-gain actuators: they only exist to put sliders in the viewer's Control panel.
    sliders = "\n".join(
        f'    <general name="{n}" joint="elev_hinge" gainprm="0" biasprm="0 0 0" ctrllimited="true" ctrlrange="{lo} {hi}"/>'
        for n, _k, lo, hi in SLIDERS)

    strut_len = hull_off + mz
    L, B = p["hull_length"], p["hull_beam"]
    D = max(0.12, 0.09 * L)                              # hull depth, visual only
    # Struts drawn at their real chord (fore-aft) and ~12 % thickness, meeting each foil at a streamlined pod;
    # the two main-foil halves are joined across the centre gap. If the elevator strut sits aft of the transom
    # (as on the manned preset: elevator 2.0 m aft, transom 1.8 m), a bracket carries it from the transom.
    mc_s, ec_s = p.get("main_strut_chord", 0.12), p.get("elev_strut_chord", 0.08)
    gap = max(lat - ms / 2, 0.0)
    pod_r = max(0.025, 0.16 * mc)
    transom = p["hull_x"] - L / 2
    struts = f"""<geom name="main_strut" type="box" size="{mc_s / 2} {0.06 * mc_s} {strut_len / 2}" pos="{mx} 0 {hull_off - strut_len / 2}" material="strut"/>
      <geom name="main_pod" type="capsule" fromto="{mx - 0.9 * mc} 0 {-mz} {mx + 0.7 * mc} 0 {-mz}" size="{pod_r}" material="strut"/>
      <geom name="main_root" type="box" size="{mc / 2} {gap + 0.01} 0.008" pos="{mx} 0 {-mz}" material="foil"/>
      <geom name="elev_strut" type="box" size="{ec_s / 2} {0.06 * ec_s} {(hull_off + ez) / 2}" pos="{ex} 0 {hull_off - (hull_off + ez) / 2}" material="strut"/>
      <geom name="elev_pod" type="capsule" fromto="{ex - 0.8 * ec} 0 {-ez} {ex + 0.6 * ec} 0 {-ez}" size="{max(0.02, 0.16 * ec)}" material="strut"/>"""
    if ex - ec_s / 2 < transom:
        struts += f"""
      <geom name="elev_bracket" type="box" size="{(transom - ex) / 2 + ec_s / 2} {0.08 * B} 0.03" pos="{(transom + ex) / 2} 0 {hull_off + 0.03}" material="strut"/>"""
    rider_p = next((q for q in p.get("default_payloads", []) if str(q.get("name", "")).lower() == "rider"), None)
    deck = hull_off + D if cad is None else cad["deck"]
    rider = ""
    if p["mass"] > 100.0 or rider_p is not None:         # seated rider positioned at Rider payload (x fwd, y_mj = -y stbd)
        rx = float(rider_p["x"]) if rider_p is not None else p["hull_x"] - 0.12 * L
        ry = -float(rider_p["y"]) if rider_p is not None else 0.0
        rider = f"""
      <geom name="rider_thigh_l" type="capsule" fromto="{rx + 0.02} {ry + 0.10} {deck + 0.10} {rx + 0.40} {ry + 0.11} {deck + 0.30}" size="0.06" material="rider_dark" contype="0" conaffinity="0"/>
      <geom name="rider_thigh_r" type="capsule" fromto="{rx + 0.02} {ry - 0.10} {deck + 0.10} {rx + 0.40} {ry - 0.11} {deck + 0.30}" size="0.06" material="rider_dark" contype="0" conaffinity="0"/>
      <geom name="rider_shin_l" type="capsule" fromto="{rx + 0.40} {ry + 0.11} {deck + 0.30} {rx + 0.56} {ry + 0.12} {deck + 0.05}" size="0.05" material="rider_dark" contype="0" conaffinity="0"/>
      <geom name="rider_shin_r" type="capsule" fromto="{rx + 0.40} {ry - 0.11} {deck + 0.30} {rx + 0.56} {ry - 0.12} {deck + 0.05}" size="0.05" material="rider_dark" contype="0" conaffinity="0"/>
      <geom name="rider_arm_l" type="capsule" fromto="{rx - 0.02} {ry + 0.19} {deck + 0.58} {rx + 0.30} {ry + 0.17} {deck + 0.36}" size="0.04" material="rider_vest" contype="0" conaffinity="0"/>
      <geom name="rider_arm_r" type="capsule" fromto="{rx - 0.02} {ry - 0.19} {deck + 0.58} {rx + 0.30} {ry - 0.17} {deck + 0.36}" size="0.04" material="rider_vest" contype="0" conaffinity="0"/>
      <geom name="rider_body" type="capsule" fromto="{rx} {ry} {deck + 0.14} {rx - 0.05} {ry} {deck + 0.62}" size="0.15" material="rider_vest" contype="0" conaffinity="0"/>
      <geom name="rider_head" type="sphere" pos="{rx - 0.05} {ry} {deck + 0.86}" size="0.11" material="rider_dark" contype="0" conaffinity="0"/>"""
    extra_loads = ""
    for i, q in enumerate(p.get("default_payloads", [])):
        if str(q.get("name", "")).lower() == "rider" and (p["mass"] > 100.0 or rider_p is not None):
            continue
        sz = float(np.clip(0.03 + 0.008 * np.sqrt(max(float(q.get("mass", 1.0)), 0.1)), 0.025, 0.12))
        qx, qy = float(q.get("x", 0.0)), -float(q.get("y", 0.0))
        extra_loads += f'\n      <geom name="payload_{i}" type="box" size="{sz:.3f} {sz:.3f} {0.6 * sz:.3f}" pos="{qx:.3f} {qy:.3f} {deck + 0.6 * sz:.3f}" material="flap" contype="0" conaffinity="0"/>'
    if cad is not None:
        hull_xml = cad["geoms"]
        struts = ""
        wing_xml = cad["wing"]
        elev_xml = cad["elev"]
    else:
        hull_xml = (f'<geom name="hull" type="mesh" mesh="hull" pos="{p["hull_x"]} 0 {hull_off}" material="hull" contype="0" conaffinity="0"/>\n'
                    f'      <geom name="cockpit" type="box" size="{0.22 * L} {0.36 * B} 0.006" pos="{p["hull_x"] - 0.08 * L} 0 {hull_off + D}" material="deck" contype="0" conaffinity="0"/>')
        wing_xml = half("left_foil", lat) + half("right_foil", -lat)
        elev_xml = f'<geom type="box" size="{ec/2} {es/2} 0.006" material="foil"/>'
    cg_off = p.get("cg_offset", (0.0, 0.0))
    if abs(float(cg_off[0])) > 1e-4 or abs(float(cg_off[1])) > 1e-4 or p.get("default_payloads"):
        extra_loads += f'\n      <geom name="cg_deck_dot" type="sphere" size="0.03" pos="{float(cg_off[0]):.3f} {-float(cg_off[1]):.3f} {deck + 0.025:.3f}" material="foil" contype="0" conaffinity="0"/>'
    return f"""
<mujoco model="hydrofoil">
  <option gravity="0 0 0"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="40"/>
    <headlight ambient="0.35 0.37 0.40" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="500" haze="0.25"/>
    <quality shadowsize="4096"/>
    <rgba haze="0.70 0.80 0.82 1" fog="0.70 0.80 0.82 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.72 0.82 0.84" rgb2="0.10 0.27 0.33" width="512" height="512"/>
    <texture name="seabed" type="2d" builtin="checker" rgb1=".05 .17 .21" rgb2=".06 .20 .24" width="256" height="256"/>
    <material name="seabed" texture="seabed" texrepeat="12 12" texuniform="true"/>
    <material name="hull" rgba="0.94 0.95 0.97 1" specular="0.4" shininess="0.6"/>
    <material name="deck" rgba="0.06 0.23 0.28 1"/>
    <material name="rider_vest" rgba="0.95 0.76 0.19 1"/>
    <material name="rider_dark" rgba="0.08 0.12 0.14 1"/>
    <mesh name="hull" vertex="{_hull_vertices(L, B, D)}"/>
    <material name="strut" rgba="0.62 0.72 0.75 1" specular="0.3"/>
    <material name="foil" rgba="0.95 0.76 0.19 1" specular="0.5"/>
    <material name="flap" rgba="0.36 0.81 0.76 1" specular="0.5"/>
    <material name="water" rgba="0.09 0.36 0.44 0.42" specular="0.7" shininess="0.9"/>
    <material name="prop" rgba="0.15 0.17 0.19 0.55"/>
    {cad["assets"] if cad else ""}
    <hfield name="water" nrow="{WATER_N}" ncol="{WATER_N}" size="{WATER_HALF} {WATER_HALF} {2 * WATER_RANGE} 0.02"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0.3 0.2 -1" directional="true" castshadow="true" diffuse="0.7 0.7 0.7"/>
    <geom name="seabed" type="plane" pos="0 0 -4" size="0 0 0.1" material="seabed" contype="0" conaffinity="0"/>
    <geom name="far_water" type="plane" pos="0 0 -0.03" size="0 0 0.1" material="water" contype="0" conaffinity="0"/>
    <body name="water" mocap="true" pos="0 0 {-WATER_RANGE}">
      <geom type="hfield" hfield="water" material="water" contype="0" conaffinity="0"/>
    </body>
    <body name="vehicle" pos="0 0 0">
      <freejoint name="root"/>
      {hull_xml}{rider}{extra_loads}
      {struts}
      <body name="main_wing" pos="{mx} 0 {-mz}">
        <joint name="main_mount" type="hinge" axis="0 1 0" limited="true" range="-30 30"/>
        {wing_xml}
      </body>
      <body name="elevator" pos="{ex} 0 {-ez}">
        <joint name="elev_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
        {elev_xml}
      </body>
    </body>
  </worldbody>
  <actuator>
{sliders}
  </actuator>
</mujoco>"""


class MujocoPose:
    """Maps a sim state + flap commands onto the MuJoCo model's qpos."""

    def __init__(self, params: dict):
        self.params = params                              # live dict: the mounting angle can change in flight
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(params))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.vehicle_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "vehicle")
        j = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        self.q_left, self.q_right, self.q_elev = j("left_foil_hinge"), j("right_foil_hinge"), j("elev_hinge")
        self.q_mount = j("main_mount")
        self.water_mocap = m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "water")]
        self.hfield_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_HFIELD, "water")
        g = np.linspace(-WATER_HALF, WATER_HALF, WATER_N)
        self._gx, self._gy = np.meshgrid(g, g)            # rows along MuJoCo y, columns along x
        self._calm_drawn = False

    def apply(self, state: np.ndarray, cmds: dict, wave_elev: float = 0.0):
        eta = state[:6]
        R_mj = _D @ R_body_to_ned(eta[3], eta[4], eta[5]) @ _B
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_mj.ravel())
        d = self.data
        d.qpos[0:3] = _D @ eta[:3]
        d.qpos[3:7] = quat
        # Positive sim flap deflection = more AoA = trailing edge down. In the MuJoCo
        # body frame (y left) a positive hinge rotation lowers the *leading* edge, so negate.
        ang = drive_angles(cmds)
        d.qpos[self.q_left] = -np.radians(ang["left"])
        d.qpos[self.q_right] = -np.radians(ang["right"])
        d.qpos[self.q_elev] = -np.radians(ang["elev"])
        d.qpos[self.q_mount] = -np.radians(float(self.params.get("main_incidence_deg", 0.0)))   # + = leading edge up
        mujoco.mj_forward(self.model, d)

    def water(self, state, wave, t: float) -> bool:
        """Move the water patch under the boat and refill its heights from the wave field. Returns True if
        the heightfield changed (the caller then uploads it with the viewer's update_hfield)."""
        x0, y0 = round(float(state[0]) / 0.5) * 0.5, round(float(-state[1]) / 0.5) * 0.5   # snapped
        self.data.mocap_pos[self.water_mocap] = [x0, y0, -WATER_RANGE]
        calm = isinstance(wave, CalmWater)
        if calm and self._calm_drawn:
            return False
        self._calm_drawn = calm
        eta = 0.0 if calm else wave.elevation(x0 + self._gx, -(y0 + self._gy), t)
        eta = np.broadcast_to(eta, self._gx.shape)
        a = self.model.hfield_adr[self.hfield_id]
        self.model.hfield_data[a:a + WATER_N * WATER_N] = np.clip((eta + WATER_RANGE) / (2 * WATER_RANGE), 0, 1).ravel()
        return True


class Pilot:
    """Keyboard stick. Writes into the slider values (ctrl) so keys and sliders stay one source."""

    UNITS = (("m/s", 1.0), ("km/h", 3.6), ("mph", 2.2369362920544))

    def __init__(self, ctrl: np.ndarray):
        self.ctrl = ctrl
        self.reset_flag = False
        self.toggle_ctrl = False
        self.next_ctrl_mode: str | None = None
        self.toggle_foil = False
        self.open_dashboard = False
        self.unit_idx = 0          # U key: cycle m/s -> km/h -> mph (display only)
        self.focus = False         # set by the dashboard's "Show 3D view"
        self.bank_spring = False   # True after a key bank input; a slider-set bank holds
        self.key_bank_value = None
        self.gain_updates: dict = {}
        self.safety_margin_req: str | None = None
        self.allocator_req: str | None = None
        self.reverse_drive_req: bool | None = None
        self.yaw_rate_cmd: float | None = None
        self.wave_type_req: str | None = None

    @property
    def kmh(self) -> bool:
        return self.unit_idx == 1

    def unit_info(self) -> tuple[str, float]:
        return self.UNITS[self.unit_idx]

    def set_unit(self, name: str) -> str:
        for i, (u, _scale) in enumerate(self.UNITS):
            if u == name:
                self.unit_idx = i
                return u
        return self.UNITS[self.unit_idx][0]

    def _nudge(self, name, step):
        _n, _k, lo, hi = SLIDERS[IDX[name]]
        self.ctrl[IDX[name]] = float(np.clip(self.ctrl[IDX[name]] + step, lo, hi))

    def key(self, code: int):
        if code == 265:      # up
            self._nudge("height_ref", HEIGHT_STEP)
        elif code == 264:    # down
            self._nudge("height_ref", -HEIGHT_STEP)
        elif code in (262, 263):    # right / left -> bank right / left (springs back)
            self._nudge("bank_ref", BANK_STEP if code == 262 else -BANK_STEP)
            self.bank_spring = True
            self.key_bank_value = float(self.ctrl[IDX["bank_ref"]])
        elif code == 32:     # Space -> level wings immediately
            self.ctrl[IDX["bank_ref"]] = 0.0
            self.bank_spring = False
            self.key_bank_value = None
        elif code in (93, 61, 334):   # ], =, numpad + -> speed up
            self._nudge("speed_ref", SPEED_STEP)
        elif code in (91, 45, 333):   # [, -, numpad - -> speed down
            self._nudge("speed_ref", -SPEED_STEP)
        elif code in (ord("R"), ord("r")):
            self.reset_flag = True
        elif code in (ord("C"), ord("c")):
            self.next_ctrl_mode = None
            self.toggle_ctrl = True
        elif code in (ord("F"), ord("f")):
            self.toggle_foil = True
        elif code in (ord("U"), ord("u")):
            self.unit_idx = (self.unit_idx + 1) % len(self.UNITS)
        elif code in (ord("D"), ord("d")):        # open the dashboard (auto-starting hydrofoil.sim_server if needed)
            self.open_dashboard = True

    def spring_back(self, dt: float):
        if not self.bank_spring:
            return
        i = IDX["bank_ref"]
        phi, step = self.ctrl[i], BANK_RETURN_RATE * dt
        self.ctrl[i] = max(0.0, phi - step) if phi > 0 else min(0.0, phi + step)
        if self.ctrl[i] == 0.0:
            self.bank_spring = False


class DriveMonitor:
    """Per-step hinge moment / motor current / bus power for the three physical drives (docs/17)."""

    def __init__(self, params: dict):
        self.hinges = surface_models(params)
        self.spec = DriveSpec()
        self.prev = None          # (angles rad, rates rad/s) of the previous step
        self.out = {d: {"M": 0.0, "I": 0.0, "P": 0.0, "tau": 0.0} for d in DRIVES}

    def reset(self):
        self.prev = None

    def update(self, vp, state, cmds, env, dt: float, gear_ratio: float, v_bus: float):
        self.spec.gear.ratio, self.spec.ctrl.v_bus = gear_ratio, v_bus
        ang = {k: np.radians(v) for k, v in drive_angles(cmds).items()}
        m = steady_moments(vp, state, cmds, env, self.hinges)
        if self.prev is None:
            rates = {d: 0.0 for d in DRIVES}
            acc = {d: 0.0 for d in DRIVES}
        else:
            a0, r0 = self.prev
            rates = {d: (ang[d] - a0[d]) / dt for d in DRIVES}
            acc = {d: (rates[d] - r0[d]) / dt for d in DRIVES}
        self.prev = (ang, rates)
        for d in DRIVES:
            h = self.hinges[d]
            tau = ((h.g.j_struct + h.i_added) * acc[d] + h.damping(m[d + "_V"]) * rates[d]
                   + self.spec.gear.friction_out * np.sign(rates[d]) - m[d])
            e = electrical_demand(self.spec, rates[d], acc[d], tau)
            self.out[d] = {"M": m[d], "tau": float(tau), "I": float(e["i_q"]), "P": float(e["p_bus"]),
                           "V": float(e["v_needed"])}
        return self.out

    def flags(self) -> str:
        c = self.spec.ctrl
        bad = [d for d in DRIVES if self.out[d].get("limited")
               or abs(self.out[d]["I"]) > c.i_peak or self.out[d].get("V", 0) > c.k_mod * c.v_bus]
        return "AT LIMIT: " + ",".join(bad) if bad else "within limits"


class Plots:
    """Rolling MuJoCo figures drawn on the left edge of the window."""

    SPECS = [
        ("Hull height above water [m]", ("height", "target")),
        ("Roll / pitch [deg]", ("roll", "pitch")),
        ("Surface angle [deg]", ("left", "right", "elev")),
        ("Drive current [A]", ("I_left", "I_right", "I_elev")),
    ]
    COLORS = [(0.95, 0.76, 0.19), (0.36, 0.81, 0.76), (0.89, 0.93, 0.94)]   # signal yellow, aqua, spray (launcher/page.html)

    def __init__(self, window_s: float = 15.0, rate_hz: float = 30.0, ranges=None):
        self.n = int(window_s * rate_hz)
        self.min_dt = 1.0 / rate_hz
        self.last_t = -1e9
        self.buf = {k: deque(maxlen=self.n) for _t, keys in self.SPECS for k in keys}
        self.t = deque(maxlen=self.n)
        self.figs = []
        for title, keys in self.SPECS:
            f = mujoco.MjvFigure()
            mujoco.mjv_defaultFigure(f)
            f.title = title
            f.flg_extend = 1
            f.flg_legend = 1
            f.figurergba[:] = (0.04, 0.17, 0.22, 0.90)
            f.panergba[:] = (0.03, 0.13, 0.16, 0.95)
            f.gridrgb[:] = (0.12, 0.32, 0.38)
            f.textrgb[:] = (0.89, 0.93, 0.94)
            f.gridsize = [4, 5]
            f.gridwidth = 1.0
            f.linewidth = 2.0
            f.legendrgba[:] = (0.04, 0.17, 0.22, 0.7)
            for i, k in enumerate(keys):
                f.linename[i] = k
                f.linergb[i] = self.COLORS[i]
            if ranges is not None:                      # fixed y range (no jumping axes); x follows time
                f.range[1][:] = ranges[len(self.figs)]
                f.flg_extend = 0
            self.figs.append(f)

    def clear(self):
        for b in self.buf.values():
            b.clear()
        self.t.clear()
        self.last_t = -1e9

    def push(self, t: float, values: dict):
        if t - self.last_t < self.min_dt:
            return
        self.last_t = t
        self.t.append(t)
        for k, v in values.items():
            self.buf[k].append(v)

    @staticmethod
    def width(viewport) -> int:
        return int(min(460, max(280, viewport.width * 0.27)))

    def figures(self, viewport):
        if viewport is None or len(self.t) < 2:
            return []
        w = Plots.width(viewport)
        h = viewport.height // len(self.figs)
        tt = np.asarray(self.t)
        out = []
        span = self.n * self.min_dt
        for j, ((_title, keys), f) in enumerate(zip(self.SPECS, self.figs)):
            f.range[0][:] = (tt[-1] - span, tt[-1])          # scrolling time window
            f.flg_legend = int(h >= 170)                      # legends cover the lines in short plots
            for i, k in enumerate(keys):
                y = np.asarray(self.buf[k])
                n = len(y)
                f.linepnt[i] = n
                f.linedata[i][0:2 * n:2] = tt[-n:]
                f.linedata[i][1:2 * n:2] = y
            rect = mujoco.MjrRect(viewport.width - w, viewport.height - (j + 1) * h, w, h)
            out.append((rect, f))
        return out


def _maximize_window(title_prefix: str):
    """Windows only: maximise the viewer window once it exists (best effort, silently skipped elsewhere)."""
    import sys
    if sys.platform != "win32":
        return
    import ctypes
    import threading

    def go():
        u = ctypes.windll.user32
        found = []
        cb_t = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

        def cb(hwnd, _):
            n = u.GetWindowTextLengthW(hwnd)
            if n:
                b = ctypes.create_unicode_buffer(n + 1)
                u.GetWindowTextW(hwnd, b, n + 1)
                if b.value.startswith(title_prefix) and u.IsWindowVisible(hwnd):
                    found.append(hwnd)
            return True
        for _ in range(50):
            u.EnumWindows(cb_t(cb), 0)
            if found:
                u.ShowWindow(found[0], 3)         # SW_MAXIMIZE
                u.SetForegroundWindow(found[0])
                return
            time.sleep(0.1)
    threading.Thread(target=go, daemon=True).start()


_FONT_DIR = Path(__file__).resolve().parent / "assets" / "fonts"
_C = {"deep": (10, 44, 55), "trench": (7, 32, 41), "rule": (30, 81, 97), "mist": (142, 178, 187), "spray": (227, 238, 240),
      "signal": (242, 194, 48), "aqua": (92, 207, 194), "flag": (215, 38, 61)}


class Hud:
    """Instrument strip along the bottom of the 3-D view, drawn with Pillow in the dashboard's type and colours
    (MuJoCo's own overlay text is a single fixed font). Saira fonts: hydrofoil/assets/fonts (SIL OFL)."""

    def __init__(self):
        from PIL import ImageFont
        self._IF = ImageFont
        self._cache = {}

    def _font(self, kind: str, size: int):
        key = (kind, size)
        if key not in self._cache:
            name = {"num": "SairaExtraCondensed-SemiBold.ttf", "ui": "SairaSemiCondensed-Regular.ttf",
                    "ui5": "SairaSemiCondensed-Medium.ttf"}[kind]
            try:
                self._cache[key] = self._IF.truetype(str(_FONT_DIR / name), size)
            except OSError:
                self._cache[key] = self._IF.load_default()
        return self._cache[key]

    def _fit_text(self, g, text: str, kind: str, size: int, max_w: int, min_size: int = 9):
        """Scale down font size until `text` fits inside `max_w`; truncate with ellipsis if still wider."""
        max_w = max(int(max_w), 16)
        s = max(int(size), min_size)
        font = self._font(kind, s)
        while s > min_size and g.textlength(text, font=font) > max_w:
            s = max(min_size, s - max(1, (s - min_size) // 3))
            font = self._font(kind, s)
        if g.textlength(text, font=font) > max_w:
            t = text
            while len(t) > 1 and g.textlength(t + "…", font=font) > max_w:
                t = t[:-1]
            text = (t + "…") if len(t) > 0 else ""
        return text, font

    def render(self, w: int, h: int, d: dict) -> np.ndarray:
        from PIL import Image, ImageDraw
        w, h = max(int(w), 160), max(int(h), 72)
        im = Image.new("RGB", (w, h), _C["deep"])
        g = ImageDraw.Draw(im)
        bar = max(18, h // 6)                                    # key-hint bar along the top of the strip
        al = d.get("alert")
        if al:                                                   # the observer's worst alert replaces the key help
            col = _C["flag"] if al[0] == "danger" else _C["signal"]
            g.rectangle([0, 0, w, bar], fill=col)
            msg = ("Danger: " if al[0] == "danger" else "Caution: ") + str(al[1])
            atxt, afnt = self._fit_text(g, msg, "ui5", int(bar * 0.66), w - 28)
            g.text((14, bar // 2), atxt, font=afnt, fill=(10, 30, 36), anchor="lm")
        else:
            g.rectangle([0, 0, w, bar], fill=_C["trench"])
            ktxt, kfnt = self._fit_text(g, str(d["keys"]), "ui", int(bar * 0.62), w - 28)
            g.text((14, bar // 2), ktxt, font=kfnt, fill=_C["mist"], anchor="lm")
        body = h - bar
        cells = [0.22, 0.2, 0.2, 0.2, 0.18]
        x = 0
        lab_sz, big_sz, sml_sz = int(body * 0.17), int(body * 0.42), int(body * 0.15)
        pad = 12 if w < 900 else 16
        for i, frac in enumerate(cells):
            cw = (w - x) if i == len(cells) - 1 else int(w * frac)
            avail = max(24, cw - 2 * pad)
            if i:
                g.line([x, bar + 8, x, h - 8], fill=_C["rule"], width=1)
            y0 = bar + int(body * 0.08)
            if i == 0:
                ttxt, tfnt = self._fit_text(g, str(d["time"]), "ui", lab_sz, avail)
                mtxt, mfnt = self._fit_text(g, str(d["mode"]), "ui5", int(body * 0.30), avail)
                ctxt, cfnt = self._fit_text(g, str(d["ctl"]), "ui", sml_sz, avail)
                g.text((x + pad, y0), ttxt, font=tfnt, fill=_C["mist"])
                g.text((x + pad, y0 + int(body * 0.2)), mtxt, font=mfnt,
                       fill=_C["signal"] if d["mode"] == "Foilborne" else _C["spray"])
                g.text((x + pad, y0 + int(body * 0.62)), ctxt, font=cfnt, fill=_C["mist"])
            elif i == 3:                                          # heel dial + numbers
                r = min(int(body * 0.34), max(16, int(cw * 0.19)))
                cx, cy = x + pad + r, bar + body // 2
                g.ellipse([cx - r, cy - r, cx + r, cy + r], fill=_C["trench"], outline=_C["rule"])
                for a in (-30, -20, -10, 0, 10, 20, 30):
                    t = np.radians(a)
                    g.line([cx + np.sin(t) * r * 0.8, cy - np.cos(t) * r * 0.8, cx + np.sin(t) * r, cy - np.cos(t) * r],
                           fill=_C["mist"], width=2 if a == 0 else 1)
                tb = np.radians(d["bank_ref"])
                g.ellipse([cx + np.sin(tb) * r * 0.9 - 3, cy - np.cos(tb) * r * 0.9 - 3, cx + np.sin(tb) * r * 0.9 + 3,
                           cy - np.cos(tb) * r * 0.9 + 3], fill=_C["aqua"])
                t = np.radians(d["roll"])
                dx, dy = np.cos(t) * r * 0.65, np.sin(t) * r * 0.65
                g.line([cx - dx, cy - dy, cx + dx, cy + dy], fill=_C["signal"], width=3)
                g.line([cx, cy, cx + np.sin(t) * r * 0.6, cy - np.cos(t) * r * 0.6], fill=_C["signal"], width=2)
                tx = cx + r + 10
                avail_r = max(24, x + cw - tx - 8)
                htxt, hfnt = self._fit_text(g, "Heel / trim", "ui", lab_sz, avail_r)
                rtxt, rfnt = self._fit_text(g, f"{d['roll']:+.1f}°", "num", big_sz, avail_r)
                ptxt, pfnt = self._fit_text(g, f"trim {d['pitch']:+.1f}°", "ui", sml_sz, avail_r)
                g.text((tx, y0), htxt, font=hfnt, fill=_C["mist"])
                g.text((tx, y0 + int(body * 0.18)), rtxt, font=rfnt, fill=_C["signal"])
                g.text((tx, y0 + int(body * 0.66)), ptxt, font=pfnt, fill=_C["mist"])
            else:
                k, v, unit, sub_, warn = d["cells"][i]
                ktxt, kfnt = self._fit_text(g, str(k), "ui", lab_sz, avail)
                sml_f = self._font("ui", max(9, sml_sz))
                uw = g.textlength(str(unit), font=sml_f) if unit else 0.0
                vtxt, vfnt = self._fit_text(g, str(v), "num", big_sz, max(20, avail - int(uw) - 8))
                stxt, sfnt = self._fit_text(g, str(sub_), "ui", sml_sz, avail)
                g.text((x + pad, y0), ktxt, font=kfnt, fill=_C["mist"])
                g.text((x + pad, y0 + int(body * 0.18)), vtxt, font=vfnt, fill=_C["signal"])
                vw = g.textlength(vtxt, font=vfnt)
                g.text((x + pad + vw + 6, y0 + int(body * 0.18) + int(body * 0.2)), str(unit), font=sml_f, fill=_C["spray"])
                g.text((x + pad, y0 + int(body * 0.66)), stxt, font=sfnt, fill=_C["flag"] if warn else _C["mist"])
            x += cw
        return np.ascontiguousarray(np.asarray(im))               # rows top-down: set_images flips them itself


def _find_window(title_prefix: str):
    """Windows only: handle of the first visible top-level window whose title starts with title_prefix."""
    import ctypes
    u = ctypes.windll.user32
    found = []
    cb_t = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

    def cb(hwnd, _):
        n = u.GetWindowTextLengthW(hwnd)
        if n:
            b = ctypes.create_unicode_buffer(n + 1)
            u.GetWindowTextW(hwnd, b, n + 1)
            if b.value.startswith(title_prefix) and u.IsWindowVisible(hwnd):
                found.append(hwnd)
        return True
    u.EnumWindows(cb_t(cb), 0)
    return found[0] if found else None


def _focus_window(title_prefix: str):
    """Bring the viewer window to the front (dashboard's "Show 3D view"), best effort across platforms."""
    import subprocess
    import sys
    try:
        if sys.platform == "win32":
            import ctypes
            u = ctypes.windll.user32
            h = _find_window(title_prefix)
            if h:
                u.ShowWindow(h, 9 if u.IsIconic(h) else 5)       # SW_RESTORE / SW_SHOW
                u.SetForegroundWindow(h)
        elif sys.platform == "darwin":
            script = 'tell application "System Events" to set frontmost of (first process whose name contains "Python" or name contains "mjpython") to true'
            subprocess.run(["osascript", "-e", script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
        else:
            subprocess.run(["wmctrl", "-a", title_prefix], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
    except Exception:
        pass


class ControlAPI:
    """Local HTTP API (127.0.0.1 only) so the launcher page can show live telemetry and move the sliders.
    GET /state -> telemetry JSON; GET /sliders -> slider ranges; POST /set {"name": value, ...} or
    {"action": "reset" | "toggle_ctrl"}. The slider values live in the MuJoCo ctrl array, as for the keys."""

    def __init__(self, port: int, ctrl, pilot, lock):
        import json
        import threading
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        self.telemetry: dict = {}
        api = self

        class H(BaseHTTPRequestHandler):
            def _json(self, obj, code=200):
                b = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

            def do_OPTIONS(self):
                self._json({})

            def do_GET(self):
                if self.path.startswith("/state"):
                    self._json(api.telemetry)
                elif self.path.startswith("/sliders"):
                    self._json([{"name": n, "lo": lo, "hi": hi, "value": float(ctrl[i])}
                                for i, (n, _k, lo, hi) in enumerate(SLIDERS)])
                else:
                    self._json({"error": "not found"}, 404)

            def do_POST(self):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except ValueError:
                    return self._json({"error": "bad json"}, 400)
                with lock():
                    act = body.pop("action", None)
                    if act == "reset":
                        pilot.reset_flag = True
                    elif act == "toggle_ctrl":
                        pilot.next_ctrl_mode = None
                        pilot.toggle_ctrl = True
                    elif act == "toggle_foil":
                        pilot.toggle_foil = True
                    elif act == "focus":
                        pilot.focus = True
                    if "unit" in body:
                        pilot.set_unit(str(body.pop("unit")))
                    if "controller_mode" in body:
                        pilot.next_ctrl_mode = str(body.pop("controller_mode")).lower()
                        pilot.toggle_ctrl = True
                    if "safety_margin" in body:
                        pilot.safety_margin_req = str(body.pop("safety_margin")).lower()
                    if "allocator" in body:
                        pilot.allocator_req = str(body.pop("allocator")).lower()
                    if "reverse_drive" in body:
                        pilot.reverse_drive_req = bool(body.pop("reverse_drive"))
                    if "yaw_rate_cmd" in body:
                        v = body.pop("yaw_rate_cmd")
                        pilot.yaw_rate_cmd = None if (v is None or abs(float(v)) < 1e-6) else float(v)
                    if "wave_type" in body:
                        wt = str(body.pop("wave_type")).lower()
                        if wt in ("regular", "irregular"):
                            pilot.wave_type_req = wt
                    for name, val in body.items():
                        if name in IDX:
                            _n, _k, lo, hi = SLIDERS[IDX[name]]
                            ctrl[IDX[name]] = float(np.clip(float(val), lo, hi))
                            if name == "bank_ref":
                                pilot.bank_spring = False          # a set bank holds
                                pilot.yaw_rate_cmd = None          # manual bank clears yaw-rate hold
                        elif name == "ki_pitch":
                            pilot.gain_updates["ki_pitch"] = float(val)
                self._json({"ok": True})

            def log_message(self, *a):
                pass

        try:
            self.srv = ThreadingHTTPServer(("127.0.0.1", port), H)
        except OSError:
            self.srv = None                                         # port busy: run without the API
            return
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        if self.srv is not None:
            self.srv.shutdown()


def _wave_from(p: dict):
    if p["wave_amplitude"] <= 1e-4:
        return CalmWater()
    if str(p.get("wave_type", "regular")).lower() == "irregular":
        return IrregularWave(
            significant_height=2.0 * float(p["wave_amplitude"]),
            peak_wavelength=float(p["wave_wavelength"]),
            direction=float(np.radians(p["wave_direction"])),
        )
    return AiryWave(amplitude=p["wave_amplitude"], wavelength=p["wave_wavelength"],
                    direction=np.radians(p["wave_direction"]))


def _apply_gains(vp: VehiclePhysics, p: dict):
    c = vp.controller
    c.heave_pid.Kp, c.heave_pid.Ki, c.heave_pid.Kd = p["kp_heave"], p["ki_heave"], p["kd_heave"]
    c.pitch_pid.Kp, c.pitch_pid.Ki, c.pitch_pid.Kd = p["kp_pitch"], p.get("ki_pitch", c.pitch_pid.Ki), p["kd_pitch"]
    c.roll_pid.Kp, c.roll_pid.Kd = p["kp_roll"], p["kd_roll"]


def _ctrl_label(vp: VehiclePhysics, u: float = 0.0) -> str:
    st = vp.controller_status(u)
    if st["mode"] == "auto":
        return f"Auto ({st['active']})"
    if st["mode"] == "lqr" and not st["engaged"] and st["active"].startswith("PID"):
        return "LQR (PID takeoff)"
    return st["active"]


def _ensure_dashboard(api_port: int = 8766, port: int = 8765, open_browser: bool = True):
    """Start `python -m hydrofoil.sim_server` on `127.0.0.1:port` if not already listening, and open it."""
    import socket
    import subprocess
    import sys
    import webbrowser
    url = f"http://127.0.0.1:{port}"
    proc = None
    try:
        with socket.create_connection(("127.0.0.1", port), 0.15):
            pass
    except OSError:
        cmd = [sys.executable, "-m", "hydrofoil.sim_server", "--no-browser",
               "--port", str(port), "--api-port", str(api_port)]
        proc = subprocess.Popen(
            cmd,
            cwd=str(Path(__file__).resolve().parent.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        deadline = time.perf_counter() + 4.0
        while time.perf_counter() < deadline:
            if proc.poll() is not None:
                break
            try:
                with socket.create_connection(("127.0.0.1", port), 0.1):
                    break
            except OSError:
                time.sleep(0.05)
    if open_browser:
        webbrowser.open(url)
    return proc


def run(speed: float = 1.0, wave_cfg: dict | None = None, duration: float = 600.0,
        controller: str | None = None, sensors: str | None = None, preset: str = "model", api_port: int = 8766, maximize: bool = True,
        overrides: dict | None = None, supervisor: bool = False,
        safety_margin: str | None = None, allocator: str | None = None, wave_type: str | None = None):
    from hydrofoil.design import apply_design
    from hydrofoil.monitor import FlightMonitor
    params = apply_design(preset_params(preset), overrides or {})
    params.setdefault("v_bus", DriveSpec().ctrl.v_bus)
    if controller:
        params["controller"] = controller
    if sensors:
        params["sensors"] = sensors
    if supervisor:
        params["supervisor"] = True
    if safety_margin:
        params["safety_margin"] = str(safety_margin).lower()
    if allocator:
        params["allocator"] = str(allocator).lower()
    if wave_type:
        params["wave_type"] = str(wave_type).lower()
    if wave_cfg:
        params.update(wave_cfg)
    else:
        params["wave_amplitude"] = 0.0
    dt = params["dt"]

    vp = VehiclePhysics(params)
    pose = MujocoPose(params)
    ctrl = pose.data.ctrl
    for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
        ctrl[i] = params[key]
    if vp.supervisor is not None:
        vp.supervisor.pilot["foil"] = True
        ctrl[IDX["height_ref"]] = float(vp.supervisor.pilot["height"])
    pilot = Pilot(ctrl)
    drives = DriveMonitor(params)
    fh = params["flying_height"]
    monitor = FlightMonitor(params)
    alerts = []
    plots = Plots(ranges=[(min(-0.1, fh - 0.6), fh + 0.4), (-6.0, 6.0), (-25.0, 25.0), (-40.0, 40.0)])

    wave = _wave_from(params)
    wave_key = (params.get("wave_type", "regular"), params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
    state = vp.rest_state()
    t = 0.0
    cmds = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    thrust, draft, wave_elev = 0.0, 0.0, 0.0
    pose.apply(state, cmds)
    last_bank_written = ctrl[IDX["bank_ref"]]

    size = max(params["hull_length"], 2 * params["main_lateral_offset"] + params["main_span"])

    # Main wing mounting angle, live from the slider (docs/26). The physics changes at once; the trim feed-forward
    # and the LQR gains depend on it, so they are re-derived in the background ~0.6 s after the slider stops,
    # with the PID loops flying meanwhile.
    inc_applied = float(params.get("main_incidence_deg", 0.0))
    inc_changed_at = None
    inc_job = {"gen": 0, "result": None}

    def _rebuild_for_incidence(inc: float, gen: int, base: dict):
        from hydrofoil.design import _reschedule
        q = dict(base, main_incidence_deg=inc)
        try:
            _reschedule(q)
        except Exception:
            pass
        sched = {k: q[k] for k in ("trim_ff_deg", "trim_ff_speed", "lqr_speeds", "lqr_engage_speed", "est_engage_speed")
                 if k in q}
        lqr, err = None, None
        if str(q.get("controller", "pid")).lower() in ("lqr", "auto") and vp.actuator_model == "servo":
            lqr, err = vp.make_lqr(q)
        inc_job["result"] = (gen, inc, sched, lqr, err)
    hud, hud_img, hud_n = Hud(), None, 0
    import threading
    with mujoco.viewer.launch_passive(pose.model, pose.data, key_callback=pilot.key,
                                      show_left_ui=False, show_right_ui=False) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = pose.vehicle_id
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.6 * size, 145.0, -12.0
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_LIGHT] = 0
        import threading
        api = ControlAPI(api_port, ctrl, pilot, v.lock)
        dash_procs: list = []
        dash_lock = threading.Lock()
        pose.water(state, wave, t)
        v.update_hfield(pose.hfield_id)
        api_line = "http://127.0.0.1:8765" if api.srv else "off (API port busy)"
        if maximize:
            _maximize_window("MuJoCo : hydrofoil")

        wall0 = time.perf_counter()
        last_wall = wall0
        while v.is_running() and t < duration:
            now = time.perf_counter()
            with v.lock():
                # A bank value that changed without a key press came from the slider: hold it.
                b = float(ctrl[IDX["bank_ref"]])
                if abs(b - last_bank_written) > 1e-9 and b != pilot.key_bank_value:
                    pilot.bank_spring = False
                pilot.spring_back(now - last_wall)
                last_bank_written = ctrl[IDX["bank_ref"]]
                pilot_keys = {"flying_height", "phi_ref", "target_speed"}
                for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
                    if vp.supervisor is not None and key in pilot_keys:
                        continue
                    params[key] = float(ctrl[i])
                if pilot.gain_updates:
                    params.update(pilot.gain_updates)
                    pilot.gain_updates.clear()
                if vp.supervisor is not None:
                    vp.supervisor.pilot["height"] = float(ctrl[IDX["height_ref"]])
                    vp.supervisor.pilot["bank"] = float(ctrl[IDX["bank_ref"]])
                    vp.supervisor.pilot["speed"] = float(ctrl[IDX["speed_ref"]])
                    vp.supervisor.pilot["yaw_rate"] = pilot.yaw_rate_cmd
                    if pilot.toggle_foil:
                        pilot.toggle_foil = False
                        vp.supervisor.pilot["foil"] = not vp.supervisor.pilot["foil"]
                else:
                    if pilot.toggle_foil:
                        pilot.toggle_foil = False
                        ctrl[IDX["height_ref"]] = -0.10 if ctrl[IDX["height_ref"]] > 0.0 else float(fh)
                        params["flying_height"] = float(ctrl[IDX["height_ref"]])
            last_wall = now
            _apply_gains(vp, params)
            inc_now = float(params.get("main_incidence_deg", 0.0))
            if abs(inc_now - inc_applied) > 1e-6:
                vp.set_main_incidence(inc_now)
                inc_applied, inc_changed_at = inc_now, now
                if vp.lqr is not None:
                    vp.lqr.force = False            # fade to PID until gains for the new angle are ready
            if inc_changed_at is not None and now - inc_changed_at > 0.6:
                inc_changed_at = None
                inc_job["gen"] += 1
                threading.Thread(target=_rebuild_for_incidence, args=(inc_applied, inc_job["gen"], dict(params)),
                                 daemon=True).start()
            if inc_job["result"] is not None:
                gen, inc, sched, new_lqr, err = inc_job["result"]
                inc_job["result"] = None
                if gen == inc_job["gen"] and abs(inc - inc_applied) < 1e-6:
                    params.update(sched)
                    cm = str(params.get("controller", "pid")).lower()
                    if new_lqr is not None:
                        new_lqr.auto_handback = (cm == "auto")
                    if cm == "pid":
                        vp._lqr_spare = new_lqr
                    else:
                        vp.lqr = new_lqr
                    vp.lqr_error = err
            if pilot.wave_type_req is not None:
                params["wave_type"] = pilot.wave_type_req
                pilot.wave_type_req = None
            wk = (params.get("wave_type", "regular"), params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
            if wk != wave_key:
                wave, wave_key = _wave_from(params), wk

            if pilot.toggle_ctrl:          # cycle auto -> lqr -> pid smoothly (or explicit controller_mode)
                pilot.toggle_ctrl = False
                cycle = ("auto", "lqr", "pid")
                if pilot.next_ctrl_mode in cycle:
                    target_mode = pilot.next_ctrl_mode
                else:
                    cur_m = str(params.get("controller", "lqr")).lower()
                    target_mode = cycle[(cycle.index(cur_m) + 1) % len(cycle)] if cur_m in cycle else "auto"
                pilot.next_ctrl_mode = None
                vp.set_controller_mode(target_mode)
                params["controller"] = vp.params["controller"]
            if pilot.safety_margin_req is not None:
                params["safety_margin"] = vp.set_safety_margin(pilot.safety_margin_req)
                pilot.safety_margin_req = None
            if pilot.allocator_req is not None:
                try:
                    params["allocator"] = vp.set_allocator_mode(pilot.allocator_req)
                except ValueError:
                    pass
                pilot.allocator_req = None
            if pilot.reverse_drive_req is not None:
                params["reverse_drive_enabled"] = bool(pilot.reverse_drive_req)
                pilot.reverse_drive_req = None
            if pilot.reset_flag:
                pilot.reset_flag = False
                pilot.yaw_rate_cmd = None
                vp.reset(); drives.reset(); plots.clear(); monitor.reset()
                state, t = vp.rest_state(), 0.0
                wall0 = now

            target_t = (now - wall0) * speed
            steps = 0
            while t < target_t and steps < 50:     # cap catch-up so a slow frame can't spiral
                state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
                t += dt
                steps += 1
                if "drive" in cmds:     # servo model: the drives' own simulated current/power
                    dm = drives.out = cmds["drive"]
                else:                   # kinematic model: estimate loads from the trajectory
                    env = vp.wave_sample(state[:6], t, wave)
                    dm = drives.update(vp, state, cmds, env, dt, params["gear_ratio"], params["v_bus"])
                ang = drive_angles(cmds)
                plots.push(t, {
                    "height": -state[2] + vp.hull_offset, "target": params["flying_height"],
                    "roll": np.degrees(state[3]), "pitch": np.degrees(state[4]),
                    "left": ang["left"], "right": ang["right"], "elev": ang["elev"],
                    "I_left": dm["left"]["I"], "I_right": dm["right"]["I"], "I_elev": dm["elev"]["I"],
                })
            if steps == 50:
                wall0 = now - t / speed             # drop the backlog instead of racing to catch up
            if steps:
                alerts = monitor.update(t, vp, state, cmds, wave, cmds.get("drive"), draft <= 1e-6)

            with v.lock():
                pose.apply(state, cmds, wave_elev)
                hf = pose.water(state, wave, t)
            if hf:
                v.update_hfield(pose.hfield_id)
            dm = drives.out
            mode = vp.supervisor.mode if vp.supervisor is not None else (
                "FOILBORNE" if draft <= 1e-6 else ("TAKING OFF" if state[6] > 1.0 else "DISPLACEMENT")
            )
            sup_stat = vp.supervisor.status if vp.supervisor is not None else None
            hud_alert = (alerts[0]["level"], alerts[0]["text"]) if alerts else None
            if sup_stat is not None:
                if sup_stat["faults"]:
                    hud_alert = ("danger", "SUP FAULT: " + ", ".join(sup_stat["faults"])
                                 + (" (GNSS vz)" if sup_stat["using_gnss"] else ""))
                elif sup_stat["warnings"] and hud_alert is None:
                    hud_alert = ("caution", "SUP WARN: " + ", ".join(sup_stat["warnings"]))
            height = -state[2] + vp.hull_offset
            limits = drives.flags()
            ctrl_str = _ctrl_label(vp, float(state[6]))
            if pilot.focus:
                pilot.focus = False
                _focus_window("MuJoCo : hydrofoil")
            if pilot.open_dashboard:
                pilot.open_dashboard = False
                def _open_dash(ap=api.srv.server_address[1] if api.srv else api_port):
                    with dash_lock:
                        pr = _ensure_dashboard(api_port=ap, open_browser=True)
                        if pr is not None:
                            dash_procs.append(pr)
                threading.Thread(target=_open_dash, daemon=True).start()
            vp_ = v.viewport
            su, k = pilot.unit_info()
            if vp_ is not None and vp_.width > 0:
                pw = Plots.width(vp_)
                sw, sh = max(320, vp_.width - pw), int(np.clip(vp_.height * 0.17, 104, 168))
                hud_n += 1
                akey = hud_alert
                if hud_img is None or hud_n % 6 == 0 or hud_img.shape[:2] != (sh, sw) or akey != getattr(hud, "_akey", None):
                    hud._akey = akey    # ~10 Hz redraw (a draw takes ~14 ms)
                    ok = limits == "within limits"
                    key_bar = ("Up/Down height  Left/Right bank  Space level  [ ] speed  F foil/land  R rest  C mode  U units"
                               + ("  D dashboard" if api.srv else ""))
                    hud_img = hud.render(sw, sh, {
                        "time": f"{t:6.1f} s", "mode": mode.capitalize(),
                        "ctl": f"{ctrl_str}, {'noisy' if vp.sensors is not None else 'ideal'} sensors"
                               + (" [sup]" if vp.supervisor is not None else ""),
                        "roll": float(np.degrees(state[3])), "pitch": float(np.degrees(state[4])), "bank_ref": params["phi_ref"],
                        "alert": hud_alert,
                        "keys": key_bar,
                        "cells": {1: ("Speed", f"{k * state[6]:.2f}", su,
                                      f"{state[6] * 1.944:.1f} kn, target {k * params['target_speed']:.2f} {su}", False),
                                  2: ("Ride height", f"{height:.2f}", "m", f"target {params['flying_height']:.2f} m", False),
                                  4: ("Thrust", f"{thrust:.0f}", "N",
                                      "drives within limits" if ok else limits.lower(), not ok)},
                    })
                v.set_images([(mujoco.MjrRect(0, 0, sw, sh), hud_img)])
            v.set_texts([])
            v.set_figures(plots.figures(v.viewport))
            v.sync()
            te = dict(getattr(vp, "_last_turn_envelope", None) or {})
            api.telemetry = {
                "t": t, "mode": mode, "preset": preset, "mass": params["mass"], "x": float(state[0]),
                "supervisor": sup_stat,
                "geom": {k: params[k] for k in ("hull_length", "hull_x", "hull_offset", "main_pos_x", "main_pos_z",
                                                "main_chord", "elev_pos_x", "elev_pos_z", "elev_chord")}
                        | {"hull_offset": float(vp.hull_offset), "main_pos_z": float(vp.left_foil.r_attach[2])},
                "wave": {"type": str(params.get("wave_type", "regular")), "amp": params["wave_amplitude"], "len": params["wave_wavelength"], "dir": params["wave_direction"]},
                "speed": float(state[6]), "speed_ref": params["target_speed"], "unit": su,
                "height": float(height), "height_ref": params["flying_height"],
                "roll": float(np.degrees(state[3])), "pitch": float(np.degrees(state[4])), "bank_ref": params["phi_ref"],
                "yaw": float(np.degrees(state[5])), "yaw_rate": float(np.degrees(state[11])),
                "yaw_rate_cmd": pilot.yaw_rate_cmd,
                "sideslip": float(np.degrees(np.arctan2(state[7], max(float(state[6]), 0.1)))),
                "bank_max": float(sup_stat["bank_max"]) if sup_stat is not None else float(te.get("bank_max_deg", BANK_MAX)),
                "thrust": float(thrust), "draft": float(draft), "controller": ctrl_str,
                "controller_detail": vp.controller_status(float(state[6])),
                "sensors": "noisy" if vp.sensors is not None else "ideal", "limits": limits,
                "surfaces": {k: float(a) for k, a in drive_angles(cmds).items()},
                "stops": {"left": float(params["flap_stop_deg"]), "right": float(params["flap_stop_deg"]),
                          "elev": float(params["elev_stop_deg"])},
                "drives": {
                    d: {k: float(dm[d].get(k, 0.0)) for k in ("M", "I", "P")}
                       | {"limited": bool(dm[d].get("limited", False)),
                          "cap": float(vp.drives[d].cap) if d in vp.drives else 0.0}
                    for d in DRIVES
                },
                "alloc": dict(getattr(vp, "_last_alloc_status", None) or {}),
                "turn_envelope": te,
                "ventilation": {k: float(v) for k, v in vp._vent.items()},
                "cavitation_sigma": float(getattr(vp, "_last_sigma", 99.0)),
                "height_status": dict(getattr(vp, "height_status", None) or {}),
                "safety_margin": str(params.get("safety_margin", "standard")),
                "allocator_mode": str(params.get("allocator", "jacobian")),
                "reverse_drive_enabled": bool(params.get("reverse_drive_enabled", True)),
                "sliders": {n: float(ctrl[i]) for i, (n, *_r) in enumerate(SLIDERS)},
                "alerts": alerts, "alert_log": monitor.log[-40:],
                "design": {k: params[k] for k in ("main_chord", "main_span", "main_incidence_deg", "elev_chord", "elev_span",
                                                  "elev_pos_x", "flying_height", "target_speed")} | {"cg_offset": list(params.get("cg_offset", (0, 0)))},
            }
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))
        for pr in dash_procs:
            if pr.poll() is None:
                pr.terminate()
        api.close()


def main():
    ap = argparse.ArgumentParser(description="Hydrofoil simulator (native MuJoCo window)")
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed multiplier")
    ap.add_argument("--duration", type=float, default=600.0, help="run length [sim s]")
    ap.add_argument("--waves", action="store_true", help="start with waves enabled (amp/len/dir also on sliders)")
    ap.add_argument("--wave-type", choices=("regular", "irregular"), help="wave spectrum model (default: params)")
    ap.add_argument("--wave-dir", type=float, default=0.0, help="wave travel direction [deg from +x]; try 90 for beam seas")
    ap.add_argument("--controller", choices=("auto", "lqr", "pid"), help="flight controller (default: params)")
    ap.add_argument("--sensors", choices=("ideal", "noisy"), help="ideal truth, or noisy sensors + Kalman")
    ap.add_argument("--windowed", action="store_true", help="do not maximise the window on start")
    ap.add_argument("--overrides", help="JSON file of design edits (boat editor), see hydrofoil/design.py")
    ap.add_argument("--api-port", type=int, default=8766, help="local live-control API port (launcher page)")
    ap.add_argument("--preset", choices=tuple(PRESETS), default="model",
                    help="model = 25 kg DEFAULT_PARAMS, manned = 250-300 kg MANNED_PARAMS (docs/19), "
                         "cad = our boat from the CAD (CAD_PARAMS, docs/26)")
    ap.add_argument("--supervisor", action="store_true",
                    help="enable flight-mode supervisor (takeoff/landing envelope & fault protection; press F to foil/land)")
    ap.add_argument("--safety-margin", choices=("conservative", "standard", "sport"),
                    help="Physics x Control safety envelope preset (default: standard)")
    ap.add_argument("--allocator", choices=("jacobian", "legacy"),
                    help="control surface allocator (default: jacobian)")
    a = ap.parse_args()
    wave_cfg = (dict(wave_amplitude=0.15, wave_wavelength=10.0, wave_direction=a.wave_dir) if a.preset in ("manned", "cad")
                else dict(wave_amplitude=0.05, wave_wavelength=5.0, wave_direction=a.wave_dir)) if a.waves else None
    run(speed=a.speed, wave_cfg=wave_cfg, duration=a.duration, controller=a.controller, sensors=a.sensors,
        preset=a.preset, api_port=a.api_port, maximize=not a.windowed, supervisor=a.supervisor,
        safety_margin=a.safety_margin, allocator=a.allocator, wave_type=a.wave_type,
        overrides=json.loads(Path(a.overrides).read_text()) if a.overrides and Path(a.overrides).exists() else None)


if __name__ == "__main__":
    main()
