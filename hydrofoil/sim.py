"""
Main simulation loop.

Integrates 6-DOF rigid-body dynamics with foil forces, buoyancy, and gravity.
Runs a cascade PID controller and logs state history for plotting.

Quick start:
    python -m hydrofoil.sim

State layout (Fossen notation):
    eta = [x, y, z, phi, theta, psi]   world-frame position + ZYX Euler angles
    nu  = [u, v, w, p, q, r]           body-frame linear + angular velocity
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path

from hydrofoil.dynamics.rigid_body import RigidBody
from hydrofoil.dynamics.kinematics import J
from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables
from hydrofoil.forces.buoyancy import buoyancy_force, gravity_force
from hydrofoil.waves.airy import CalmWater, AiryWave
from hydrofoil.control.pid import HydrofoilController
from hydrofoil.control.actuator import DEFAULT_ACTUATOR_PARAMS


# ---------------------------------------------------------------------------
# Vehicle parameters — edit these for your vehicle
# ---------------------------------------------------------------------------

MASS = 25.0                     # kg
INERTIA = np.diag([2.0, 5.0, 5.0])   # kg·m² (Ixx, Iyy, Izz) — rough estimate
DISPLACED_VOLUME = MASS / 1025.0     # m³, neutral buoyancy

FOIL_GEOMETRY = FoilGeometry(chord=0.15, span=0.6)   # each main foil half (left/right)
ELEV_GEOMETRY = FoilGeometry(chord=0.10, span=0.3)   # elevator

# Foil polar — NACA 0009, XFLR5 Re=2.70e6
# Replace with FoilTables.from_dat("path/to/your_polar.dat") when you have
# an experimental polar or a polar at your actual operating Reynolds number.
FOIL_POLAR = FoilTables.naca0009()

# Foil attachment points in body frame [forward, lateral, depth] (m)
# Positive x = forward, positive y = right/starboard, positive z = downward.
MAIN_FOIL_POS = np.array([ 0.0,  0.0, 0.5])   # main foil centerline, below CoG
ELEV_FOIL_POS = np.array([-1.2,  0.0, 0.4])   # rear + below

# Lateral distance from centerline to each main-foil half (roughly the
# post/mast spacing). The main wing is modeled as two independently-flapped
# halves — this offset is what lets delta_roll actually produce a rolling
# moment: left/right lift is asymmetric, and the resulting moment (r x F)
# banks the vehicle, which the closed-loop dynamics turn into an actual
# turn — no separate "yaw" mechanism needed.
MAIN_FOIL_LATERAL_OFFSET = 0.35   # m
LEFT_FOIL_POS  = MAIN_FOIL_POS + np.array([0.0, -MAIN_FOIL_LATERAL_OFFSET, 0.0])
RIGHT_FOIL_POS = MAIN_FOIL_POS + np.array([0.0,  MAIN_FOIL_LATERAL_OFFSET, 0.0])

# PID gains — tune these first in simulation
PID_GAINS = {
    'heave': {'Kp': 1.0,  'Ki': 0.0, 'Kd': 3.0},
    'pitch': {'Kp': 2.0,  'Ki': 0.0, 'Kd': 2.0},
    'roll':  {'Kp': 3.5,  'Ki': 0.0, 'Kd': 2.5},
}

# Flap actuator dynamics — placeholder BLDC (MyActuator RMD-X8-class) numbers.
# Swap in real datasheet/bench-measured values once known.
ACTUATOR_PARAMS = dict(DEFAULT_ACTUATOR_PARAMS)

# Simulation settings
DT = 0.01       # integration timestep [s]
T_END = 20.0    # simulation duration [s]
CRUISE_SPEED = 4.0   # m/s forward speed (held constant for now)

# Reference (desired) state.  NED convention: z positive DOWN.
Z_REF = 0.5     # 0.5 m below surface (positive z = deeper in NED)
PHI_REF = 0.0
THETA_REF = 0.0


def build_vehicle() -> RigidBody:
    return RigidBody(mass=MASS, I_b=INERTIA)


def build_foils():
    left = Foil(FOIL_GEOMETRY, FOIL_POLAR, r_attach=LEFT_FOIL_POS)
    right = Foil(FOIL_GEOMETRY, FOIL_POLAR, r_attach=RIGHT_FOIL_POS)
    elev = Foil(ELEV_GEOMETRY, FOIL_POLAR, r_attach=ELEV_FOIL_POS)
    return left, right, elev


def physics_rhs(state, cmds, vehicle, left_foil, right_foil, elev_foil):
    """
    Pure physics ODE right-hand side with fixed control commands.

    Separating physics from control means RK4 can call this freely
    without corrupting the PID integrator state.
    """
    eta = state[:6]
    nu  = state[6:]

    tau = np.zeros(6)

    nu_with_speed = nu.copy()
    nu_with_speed[0] = CRUISE_SPEED
    # delta_roll differentially biases the two halves — this is the actual
    # mechanism (not a separate "roll force") that produces the rolling
    # moment: left and right halves fly at different AoA, so their lift
    # differs, and that asymmetry times the lateral offset is the moment.
    tau += left_foil.generalized_force(nu_with_speed, delta=cmds['delta_main'] - cmds['delta_roll'])
    tau += right_foil.generalized_force(nu_with_speed, delta=cmds['delta_main'] + cmds['delta_roll'])
    tau += elev_foil.generalized_force(nu_with_speed, delta=cmds['delta_elev'])
    tau += buoyancy_force(DISPLACED_VOLUME, eta)
    tau += gravity_force(MASS, eta)
    tau[2] -= 50.0 * nu[2]   # linear heave damping

    eta_dot = J(eta) @ nu
    nu_dot  = vehicle.nu_dot(nu, tau)
    return np.concatenate([eta_dot, nu_dot])


def run_simulation(wave=None, open_loop=False):
    if wave is None:
        wave = CalmWater()

    vehicle = build_vehicle()
    left_foil, right_foil, elev_foil = build_foils()
    controller = HydrofoilController(PID_GAINS, dt=DT, actuator_params=ACTUATOR_PARAMS)

    # Initial state: 0.3m depth (too shallow vs 0.5m ref), small pitch perturbation.
    # NED: z=0.3 means 0.3m below surface (positive = deeper).
    eta0 = np.array([0.0, 0.0, 0.3, 0.0, np.radians(2.0), 0.0])
    nu0  = np.array([CRUISE_SPEED, 0.0, 0.0, 0.0, 0.0, 0.0])
    state = np.concatenate([eta0, nu0])

    t = 0.0
    log = {'t': [], 'z': [], 'phi': [], 'theta': [],
           'd_main': [], 'd_elev': [], 'd_roll': []}

    while t <= T_END:
        # 1. Evaluate controller once with current state (advances PID integrator once).
        eta = state[:6]
        nu  = state[6:]
        if open_loop:
            cmds = {'delta_main': 0.0, 'delta_elev': 0.0, 'delta_roll': 0.0}
        else:
            cmds = controller.update(
                Z_REF, eta[2],
                PHI_REF, eta[3],
                THETA_REF, eta[4],
            )

        # 2. RK4 with fixed commands — safe to call physics_rhs multiple times.
        def rhs(s):
            return physics_rhs(s, cmds, vehicle, left_foil, right_foil, elev_foil)

        k1 = rhs(state)
        k2 = rhs(state + DT / 2 * k1)
        k3 = rhs(state + DT / 2 * k2)
        k4 = rhs(state + DT * k3)
        state = state + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        log['t'].append(t)
        log['z'].append(eta[2])
        log['phi'].append(np.degrees(eta[3]))
        log['theta'].append(np.degrees(eta[4]))
        log['d_main'].append(cmds['delta_main'])
        log['d_elev'].append(cmds['delta_elev'])
        log['d_roll'].append(cmds['delta_roll'])

        t += DT

    return {k: np.array(v) for k, v in log.items()}


def plot_results(log, title="Hydrofoil Simulation"):
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle(title)
    gs = gridspec.GridSpec(3, 2, figure=fig)

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(log['t'], log['z'])
    ax1.axhline(Z_REF, color='r', linestyle='--', label='reference')
    ax1.set_ylabel('Depth z [m] (NED)')
    ax1.set_title('Heave (altitude)')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(log['t'], log['phi'])
    ax2.axhline(0, color='r', linestyle='--')
    ax2.set_ylabel('Roll φ [deg]')
    ax2.set_title('Roll')
    ax2.grid(True)

    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(log['t'], log['theta'])
    ax3.axhline(0, color='r', linestyle='--')
    ax3.set_ylabel('Pitch θ [deg]')
    ax3.set_xlabel('Time [s]')
    ax3.set_title('Pitch')
    ax3.grid(True)

    ax4 = fig.add_subplot(gs[0, 1])
    ax4.plot(log['t'], log['d_main'], label='main foil')
    ax4.set_ylabel('Delta [deg]')
    ax4.set_title('Main foil command')
    ax4.legend()
    ax4.grid(True)

    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(log['t'], log['d_elev'], color='orange', label='elevator')
    ax5.set_ylabel('Delta [deg]')
    ax5.set_title('Elevator command')
    ax5.legend()
    ax5.grid(True)

    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(log['t'], log['d_roll'], color='green', label='roll diff')
    ax6.set_ylabel('Delta [deg]')
    ax6.set_xlabel('Time [s]')
    ax6.set_title('Roll differential command')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()

    # Save to file; also attempt interactive display if a backend supports it
    slug = title.lower().replace(' ', '_').replace('—', '').replace('(', '').replace(')', '').strip('_')
    out = Path(__file__).parent.parent / "data" / f"{slug}.png"
    fig.savefig(out, dpi=120)
    print(f"Plot saved → {out}")

    try:
        plt.show()
    except Exception:
        pass
    plt.close(fig)


if __name__ == '__main__':
    from hydrofoil.viz import dashboard, animation_2d

    print("=== CLOSED-LOOP (PID controller) ===")
    log_cl = run_simulation()

    # Static PNG
    plot_results(log_cl, "T-Foil — Closed Loop (PID)")

    # Interactive time-series dashboard → opens in browser
    dashboard(log_cl, title="T-Foil — Closed Loop (PID)", z_ref=Z_REF)

    # Animated 2D side-view → opens in browser
    animation_2d(log_cl, z_ref=Z_REF, title="T-Foil — Closed Loop Side View")

    print("\n=== OPEN-LOOP (no controller — should diverge) ===")
    log_ol = run_simulation(open_loop=True)
    plot_results(log_ol, "T-Foil — Open Loop (should diverge)")
    dashboard(log_ol, title="T-Foil — Open Loop (diverges)", z_ref=Z_REF)
    animation_2d(log_ol, z_ref=Z_REF, title="T-Foil — Open Loop Side View")
