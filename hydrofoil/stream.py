"""
UDP state streamer — sends simulation state to Unity in real time.

Python (WSL2) → UDP JSON → Unity (Windows, port 5005)

Usage:
    from hydrofoil.stream import SimStreamer, get_windows_host_ip
    streamer = SimStreamer()
    streamer.send(t, eta, nu, cmds)
"""

from __future__ import annotations
import json
import socket
import subprocess


def get_windows_host_ip() -> str:
    """Return the Windows host IP as seen from WSL2 (the default gateway)."""
    try:
        result = subprocess.run(
            "ip route show default",
            shell=True, capture_output=True, text=True,
        )
        for line in result.stdout.splitlines():
            parts = line.split()
            if "default" in parts:
                return parts[parts.index("via") + 1]
    except Exception:
        pass
    return "127.0.0.1"


class SimStreamer:
    """
    Sends one UDP datagram per simulation step.

    Packet format — flat JSON matching Unity's SimState.cs:
        {"t":1.23, "x":0,"y":0,"z":0.5, "phi":0,"theta":0.02,"psi":0,
         "u":4,"v":0,"w":0, "d_main":2.1,"d_elev":0.5,"d_roll":0}

    Coordinate convention: NED (z positive down, angles in radians).
    Unity-side conversion is handled in HydrofoilScene.cs.

    Parameters
    ----------
    host : IP of Windows Unity instance (auto-detected from WSL2 gateway)
    port : UDP port Unity is listening on (default 5005)
    """

    def __init__(self, host: str | None = None, port: int = 5005):
        self.host = host or get_windows_host_ip()
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[SimStreamer] Sending to {self.host}:{self.port}")

    def send(self, t: float, eta, nu, cmds: dict,
              thrust: float = 0.0, draft: float = 0.0,
              foilborne: bool = False, wave_elev: float = 0.0):
        """
        Encode and send one state packet.

        Parameters
        ----------
        t         : simulation time [s]
        eta       : (6,) [x, y, z, phi, theta, psi] in NED [m, m, m, rad, rad, rad]
        nu        : (6,) [u, v, w, p, q, r] body-frame velocity [m/s, rad/s]
        cmds      : HydrofoilController output dict — keys delta_main,
                    delta_elev, delta_roll [deg] (NOT d_main/d_elev/d_roll;
                    those are only the outbound JSON field names below)
        thrust    : commanded thrust [N]
        draft     : hull draft below the local water surface [m]
        foilborne : True once draft has reached zero (hull clear of water)
        wave_elev : wave surface elevation at the vehicle's position [m]
        """
        packet = {
            "t":     round(float(t), 4),
            "x":     round(float(eta[0]), 4),
            "y":     round(float(eta[1]), 4),
            "z":     round(float(eta[2]), 4),
            "phi":   round(float(eta[3]), 5),
            "theta": round(float(eta[4]), 5),
            "psi":   round(float(eta[5]), 5),
            "u":     round(float(nu[0]), 3),
            "v":     round(float(nu[1]), 3),
            "w":     round(float(nu[2]), 3),
            "d_main": round(float(cmds["delta_main"]), 3),
            "d_elev": round(float(cmds["delta_elev"]), 3),
            "d_roll": round(float(cmds["delta_roll"]), 3),
            "thrust": round(float(thrust), 2),
            "draft": round(float(draft), 4),
            "foilborne": bool(foilborne),
            "wave_elev": round(float(wave_elev), 4),
        }
        try:
            self._sock.sendto(json.dumps(packet).encode(), (self.host, self.port))
        except OSError:
            pass

    def close(self):
        self._sock.close()
