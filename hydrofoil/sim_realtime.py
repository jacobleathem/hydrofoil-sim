"""
Real-time simulation loop with Unity streaming.

Runs the full vehicle physics (hull buoyancy/takeoff transition, surge/
thrust control, wave coupling — see hydrofoil/vehicle.py) at wall-clock
speed (or a configurable multiple), streams state to Unity via UDP after
each step, and listens for pilot climb/dive/bank/throttle input sent back
from Unity (see hydrofoil/control_uplink.py). Python stays the sole physics
authority; Unity is a view + control surface.

Roll is closed-loop via the roll PID (see control/pid.py and vehicle.py) —
stable for the tuned default gains (Kp=3.5, Ki=0, Kd=2.5); Ki must stay 0.

Usage:
    python3 -m hydrofoil.sim_realtime
    python3 -m hydrofoil.sim_realtime --speed 2   # 2x real-time
    python3 -m hydrofoil.sim_realtime --host 172.26.32.1 --port 5005
    python3 -m hydrofoil.sim_realtime --waves
"""

import argparse
import time

from hydrofoil.waves.airy import CalmWater, AiryWave
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.stream import SimStreamer
from hydrofoil.control_uplink import PilotUplink, ThrottleFailsafe
from hydrofoil.dashboard import DEFAULT_PARAMS


def run_realtime(speed: float = 1.0, streamer: SimStreamer | None = None,
                 wave=None, duration: float = 60.0, pilot_port: int = 5006,
                 link_timeout: float = 0.5, failsafe_ramp: float = 1.0,
                 require_pilot: bool = False):
    """
    Run the simulation in real time, stream state to Unity, and accept
    pilot climb/dive input from Unity over UDP.

    Parameters
    ----------
    speed      : simulation speed multiplier (1.0 = real-time, 2.0 = 2x faster)
    streamer   : SimStreamer instance (created automatically if None)
    wave       : wave model (CalmWater() if None)
    duration   : total run duration [s]
    pilot_port : UDP port to listen for pilot input on
    link_timeout  : seconds without a pilot packet before thrust is cut
    failsafe_ramp : seconds for thrust to ramp to zero when cut
    require_pilot : keep thrust cut until the first pilot packet arrives
    """
    if wave is None:
        wave = CalmWater()
    if streamer is None:
        streamer = SimStreamer()

    params = dict(DEFAULT_PARAMS)
    dt = params["dt"]

    vp = VehiclePhysics(params)
    state = vp.rest_state()   # start at rest, floating — the takeoff transition is the point

    pilot = PilotUplink(port=pilot_port, initial_flying_height=params["flying_height"],
                        initial_target_speed=params["target_speed"],
                        initial_phi_ref=params["phi_ref"])
    pilot.start()
    failsafe = ThrottleFailsafe(link_timeout=link_timeout, ramp_time=failsafe_ramp,
                                require_pilot=require_pilot)

    t = 0.0
    step_wall = dt / speed          # wall-clock seconds per sim step
    stream_every = max(1, int(0.033 / dt))   # ~30 Hz stream rate
    step_count = 0

    print(f"Streaming to Unity at ~{1/step_wall*stream_every:.0f} packets/s  "
          f"(sim {speed}x real-time, {duration}s total)")
    print("Press Ctrl+C to stop.\n")

    try:
        while t <= duration:
            wall_start = time.perf_counter()

            params["flying_height"] = pilot.flying_height   # read fresh, same pattern as dashboard.py
            params["target_speed"] = pilot.target_speed     # throttle — feeds the surge PID, not a direct force
            params["phi_ref"] = pilot.phi_ref                # bank target — feeds the roll PID (clamped +-8deg)
            was_cut = failsafe.reason
            params["thrust_scale"] = failsafe.update(dt, pilot.armed, pilot.deadman,
                                                     pilot.seconds_since_packet())
            if failsafe.reason != was_cut:
                if failsafe.reason:
                    action = "thrust cut" if failsafe.scale <= 0.0 else f"thrust ramping to 0 over {failsafe_ramp:.1f}s"
                    print(f"t={t:5.1f}s  FAILSAFE: {failsafe.reason} - {action}")
                else:
                    print(f"t={t:5.1f}s  failsafe cleared ({was_cut}) - thrust live")
            state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
            t += dt
            step_count += 1

            foilborne = draft <= 1e-6

            if step_count % stream_every == 0:
                streamer.send(t, state[:6], state[6:], cmds,
                              thrust=thrust, draft=draft, foilborne=foilborne, wave_elev=wave_elev,
                              thrust_scale=params["thrust_scale"], failsafe=failsafe.reason)

            if step_count % int(1.0 / dt) == 0:
                z = state[2]
                th = (state[4] * 180.0 / 3.14159265)
                mode = "FOILBORNE" if foilborne else "DISPLACEMENT"
                u = state[6]
                print(f"t={t:5.1f}s  z={z:.3f}m  draft={draft:.3f}m [{mode}]  "
                      f"u={u:.2f}m/s (target {params['target_speed']:.1f})  "
                      f"theta={th:.2f} deg  thrust={thrust:.1f}N  "
                      f"delta_main={cmds['delta_main']:.1f} deg")

            elapsed = time.perf_counter() - wall_start
            sleep_time = step_wall - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pilot.stop()
        streamer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hydrofoil sim -> Unity streamer")
    parser.add_argument("--speed",      type=float, default=1.0,   help="Sim speed multiplier")
    parser.add_argument("--host",       type=str,   default=None,  help="Unity host IP")
    parser.add_argument("--port",       type=int,   default=5005,  help="Unity UDP port")
    parser.add_argument("--pilot-port", type=int,   default=5006,  help="UDP port for pilot input from Unity")
    parser.add_argument("--duration",   type=float, default=60.0,  help="Run duration [s]")
    parser.add_argument("--waves",      action="store_true",        help="Enable wave disturbance")
    parser.add_argument("--link-timeout", type=float, default=0.5, help="Seconds without pilot input before thrust is cut")
    parser.add_argument("--failsafe-ramp", type=float, default=1.0, help="Seconds for thrust to ramp to zero when cut")
    parser.add_argument("--require-pilot", action="store_true", help="Keep thrust cut until the first pilot packet arrives")
    args = parser.parse_args()

    wave = AiryWave(amplitude=0.05, wavelength=5.0, period=2.0) if args.waves else CalmWater()
    streamer = SimStreamer(host=args.host, port=args.port)
    run_realtime(speed=args.speed, streamer=streamer, wave=wave, duration=args.duration,
                 pilot_port=args.pilot_port, link_timeout=args.link_timeout,
                 failsafe_ramp=args.failsafe_ramp, require_pilot=args.require_pilot)
