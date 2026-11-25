"""
MuJoCo drawing of our boat as modelled in CAD (SolidWorks "Hydrofoil Assembly+wings_not_finished" and "Split Wing
Assembly", measured from their STL exports on 2026-11-24, see docs/26-cad-boat-preset.md). Visual only: the physics
uses the numbers in params.CAD_PARAMS. Everything is placed from the params, so boat-editor edits (strut length,
wing size, elevator position) still move the drawing.

Frames: MuJoCo body frame, origin at the CG, x forward, y to port (left), z up. Lengths in metres.
"""

from __future__ import annotations

import numpy as np

# Main-wing planform of one half, measured from the CAD (Split Wing Assembly). Spanwise stations from the
# centreline [m] with the leading and trailing edges measured from the straight leading edge [m, + = aft].
# 0-0.24 m is the flap notch: the fixed wing ends 0.200 m aft of the leading edge and the flap fills 0.200-0.300.
CAD_HALF_SPAN = 0.6765
CAD_ROOT_CHORD = 0.300
CAD_MEAN_CHORD = 0.271
CAD_NOTCH_Y = 0.240
CAD_NOTCH_TE = 0.200
WING_STATIONS = [   # (y, LE aft, TE aft)
    (0.000, 0.0000, 0.3000), (0.380, 0.0000, 0.3000), (0.400, 0.0004, 0.2987), (0.420, 0.0010, 0.2972),
    (0.440, 0.0018, 0.2947), (0.460, 0.0030, 0.2910), (0.480, 0.0046, 0.2861), (0.500, 0.0067, 0.2799),
    (0.520, 0.0092, 0.2724), (0.540, 0.0122, 0.2634), (0.560, 0.0158, 0.2527), (0.580, 0.0199, 0.2402),
    (0.600, 0.0249, 0.2255), (0.620, 0.0309, 0.2073), (0.640, 0.0390, 0.1832), (0.660, 0.0500, 0.1502),
    (0.670, 0.0591, 0.1229), (0.6765, 0.0830, 0.0990),
]
THICKNESS = 0.09            # NACA 0009

# Layout measured from the full assembly, relative to the pontoon bottoms (z) and the main wing's quarter chord (x).
PONTOON_DIAMETER = 0.265
FRAME_Z = 0.265             # top of the pontoons / underside of the deck frame, above the pontoon bottoms
BEAM_Z = 0.341              # cross and side beams (centre), above the pontoon bottoms
COCKPIT = dict(dx=0.619, z=0.260, size=(1.406, 0.705, 0.319))      # dx: centre forward of the pontoon centre
CROSS_BEAMS_DX = (-0.618, 0.889)                                     # forward of the pontoon centre
SIDE_BEAM_Y, SIDE_BEAM_DX = 0.400, 0.136
REAR_BOOMS = ((-2.082, 0.00), (-0.622, 0.44))                        # (dx from pontoon centre, y) ends
MAST_CHORD, MAST_THICK = 0.28, 0.051
MAST_DX = 0.060             # mast centre aft of the wing quarter chord
REAR_MAST = dict(dx=-0.036, chord=0.102, thick=0.076)                # dx: forward of the elevator quarter chord
PROP = dict(dx=-0.386, dz=0.139, d=0.21, y=0.343)                    # from the wing quarter chord; dz above the wing
MOTOR = dict(x0=-0.374, x1=-0.132, r=0.033)


def _f(v: float) -> str:
    return f"{float(v):.4f}"


def _prism(name: str, pts2d, z0: float, z1: float) -> str:
    """Mesh asset: a convex prism from a planform polygon (x, y) between heights z0 and z1."""
    verts = " ".join(f"{_f(x)} {_f(y)} {_f(z)}" for z in (z0, z1) for x, y in pts2d)
    return f'<mesh name="{name}" vertex="{verts}"/>'


def cad_parts(p: dict) -> dict:
    """MJCF fragments for the CAD boat: assets, hull/frame/struts geoms, the two wing halves (with hinged flaps)
    for inside the main-wing body, the elevator geom, and the deck height for the rider."""
    hull_bot = float(p["hull_offset"])              # z of the pontoon bottoms (hull_offset < 0: below the CG)
    hx, L = float(p["hull_x"]), float(p["hull_length"])
    spacing = float(p.get("hull_twin_spacing", 1.308) or 1.308)
    r = 0.5 * float(p.get("max_draft", PONTOON_DIAMETER))
    mx, mz = float(p["main_pos_x"]), float(p["main_pos_z"])
    ex, ez = float(p["elev_pos_x"]), float(p["elev_pos_z"])
    ec, es = float(p["elev_chord"]), float(p["elev_span"])
    strut_y = float(p.get("main_strut_y", 0.35))
    sy = float(p["main_span"]) / CAD_HALF_SPAN                       # boat-editor scaling of the drawn planform
    sx = float(p["main_chord"]) / CAD_MEAN_CHORD
    frame_z = hull_bot + 2 * r                      # the frame sits on top of the pontoons (CAD: 0.265 m)
    assets, geoms = [], []

    # pontoons, deck frame, cockpit
    for i, y in enumerate((spacing / 2, -spacing / 2)):
        geoms.append(f'<geom name="pontoon_{i}" type="capsule" fromto="{_f(hx - L / 2 + r)} {_f(y)} {_f(hull_bot + r)} '
                     f'{_f(hx + L / 2 - r)} {_f(y)} {_f(hull_bot + r)}" size="{_f(r)}" material="hull" contype="0" conaffinity="0"/>')
    bz = frame_z + (BEAM_Z - FRAME_Z)
    for i, dx in enumerate(CROSS_BEAMS_DX):
        geoms.append(f'<geom name="cross_beam_{i}" type="box" size="0.051 {_f(spacing / 2 + 0.1)} 0.076" '
                     f'pos="{_f(hx + dx)} 0 {_f(bz)}" material="strut" contype="0" conaffinity="0"/>')
    for i, y in enumerate((SIDE_BEAM_Y, -SIDE_BEAM_Y)):
        geoms.append(f'<geom name="side_beam_{i}" type="box" size="0.75 0.051 0.076" pos="{_f(hx + SIDE_BEAM_DX)} {_f(y)} {_f(bz)}" '
                     f'material="strut" contype="0" conaffinity="0"/>')
    (bx0, by0), (bx1, by1) = REAR_BOOMS
    for i, s in enumerate((1, -1)):
        geoms.append(f'<geom name="rear_boom_{i}" type="capsule" fromto="{_f(hx + bx0)} {_f(s * by0)} {_f(bz)} '
                     f'{_f(hx + bx1)} {_f(s * by1)} {_f(bz)}" size="0.045" material="strut" contype="0" conaffinity="0"/>')
    boxes = [p["cockpit_box"]] if p.get("cockpit_box") else (p.get("hull_boxes") or [])
    if not boxes:                                   # the CAD's own (unfinished) cockpit shape
        cx, cy, cz = COCKPIT["size"]
        geoms.append(f'<geom name="cockpit" type="box" size="{_f(cx / 2)} {_f(cy / 2)} {_f(cz / 2)}" '
                     f'pos="{_f(hx + COCKPIT["dx"])} 0 {_f(hull_bot + COCKPIT["z"])}" material="deck" contype="0" conaffinity="0"/>')
    for i, b in enumerate(boxes):                   # buoyant boxes (cockpit), bottom level with the pontoon bottoms
        geoms.append(f'<geom name="{"cockpit" if i == 0 else f"box_{i}"}" type="box" '
                     f'size="{_f(b["length"] / 2)} {_f(b["beam"] / 2)} {_f(b["height"] / 2)}" '
                     f'pos="{_f(b.get("x", 0.0))} {_f(-float(b.get("y", 0.0)))} {_f(hull_bot + b["height"] / 2)}" '
                     f'material="{"deck" if p.get("cockpit_buoyancy", True) else "deck_dry"}" contype="0" conaffinity="0"/>')

    # two front masts with motor pods and propellers, one rear mast
    mast_len = frame_z + mz
    for i, y in enumerate((strut_y, -strut_y)):
        geoms.append(f'<geom name="front_mast_{i}" type="box" size="{_f(MAST_CHORD / 2)} {_f(MAST_THICK / 2)} {_f(mast_len / 2)}" '
                     f'pos="{_f(mx - MAST_DX)} {_f(y)} {_f(frame_z - mast_len / 2)}" material="strut"/>')
        pz = -mz + PROP["dz"]
        py = np.sign(y) * PROP["y"]
        geoms.append(f'<geom name="motor_{i}" type="capsule" fromto="{_f(mx + MOTOR["x0"])} {_f(py)} {_f(pz)} '
                     f'{_f(mx + MOTOR["x1"])} {_f(py)} {_f(pz)}" size="{_f(MOTOR["r"])}" material="strut"/>')
        geoms.append(f'<geom name="motor_arm_{i}" type="box" size="0.06 {_f(abs(abs(y) - PROP["y"]) / 2 + 0.02)} 0.025" '
                     f'pos="{_f(mx - 0.15)} {_f((y + py) / 2)} {_f(pz)}" material="strut"/>')
        geoms.append(f'<geom name="prop_{i}" type="cylinder" fromto="{_f(mx + PROP["dx"] - 0.01)} {_f(py)} {_f(pz)} '
                     f'{_f(mx + PROP["dx"] + 0.01)} {_f(py)} {_f(pz)}" size="{_f(PROP["d"] / 2)}" material="prop"/>')
    rm_len = frame_z + ez
    geoms.append(f'<geom name="rear_mast" type="box" size="{_f(REAR_MAST["chord"] / 2)} {_f(REAR_MAST["thick"] / 2)} {_f(rm_len / 2)}" '
                 f'pos="{_f(ex + REAR_MAST["dx"])} 0 {_f(frame_z - rm_len / 2)}" material="strut"/>')

    # main wing halves, drawn about the wing's quarter chord (the main-wing body origin)
    le = 0.25 * CAD_ROOT_CHORD * sx                    # leading edge is this far forward of the quarter chord

    def half(name: str, side: int) -> str:
        segs = []
        for k in range(len(WING_STATIONS) - 1):
            (y0, le0, te0), (y1, le1, te1) = WING_STATIONS[k], WING_STATIONS[k + 1]
            pieces = [(y0, y1, le0, le1, te0, te1)]
            if y1 <= CAD_NOTCH_Y + 1e-9:
                continue
            if y0 < CAD_NOTCH_Y:
                pieces = [(CAD_NOTCH_Y, y1, le0, le1, te0, te1)]
            for a, b, la, lb, ta, tb in pieces:
                t = THICKNESS * (tb - lb) * sx
                pts = [(le - la * sx, side * a * sy), (le - ta * sx, side * a * sy),
                       (le - tb * sx, side * b * sy), (le - lb * sx, side * b * sy)]
                mesh = f"{name}_seg{k}"
                assets.append(_prism(mesh, pts, -max(t, 0.004) / 2, max(t, 0.004) / 2))
                segs.append(f'<geom type="mesh" mesh="{mesh}" material="foil"/>')
        # fixed part of the notch section, then the hinged flap behind it
        n_y = CAD_NOTCH_Y * sy
        t_root = THICKNESS * CAD_ROOT_CHORD * sx
        segs.append(f'<geom type="box" size="{_f(CAD_NOTCH_TE * sx / 2)} {_f(n_y / 2)} {_f(t_root / 2)}" '
                    f'pos="{_f(le - CAD_NOTCH_TE * sx / 2)} {_f(side * n_y / 2)} 0" material="foil"/>')
        fc = (CAD_ROOT_CHORD - CAD_NOTCH_TE) * sx
        flap = (f'<body name="{name}_flap" pos="{_f(le - CAD_NOTCH_TE * sx)} {_f(side * n_y / 2)} 0">'
                f'<joint name="{name}_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>'
                f'<geom type="box" size="{_f(fc / 2)} {_f(n_y / 2 - 0.002)} {_f(0.45 * t_root / 2)}" pos="{_f(-fc / 2)} 0 0" material="flap"/>'
                f'</body>')
        return f'<body name="{name}" pos="0 0 0">{"".join(segs)}{flap}</body>'

    wing = half("left_foil", 1) + half("right_foil", -1)
    elev = (f'<geom type="box" size="{_f(ec / 2)} {_f(es / 2)} {_f(THICKNESS * ec / 2)}" pos="{_f(-ec / 4)} 0 0" '
            f'material="foil"/>')
    return {"assets": "\n    ".join(assets), "geoms": "\n      ".join(geoms), "wing": wing, "elev": elev,
            "deck": hull_bot + 0.12}
