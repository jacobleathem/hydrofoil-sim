"""
Pilot-input UDP uplink: Unity -> Python.

Mirrors dashboard.py's live "stick" semantics (POST /manual mutating the
running sim's setpoints without a restart) but over UDP, since sim_realtime.py
isn't a web server. A background daemon thread listens for small JSON packets
and exposes the latest values; the main sim loop reads them each iteration,
exactly like dashboard.py reads runner.params fresh every step.

Three axes: climb/dive (flying_height, an autopilot-style height target the
heave PID chases), throttle (target_speed, what the surge controller
chases), and bank (phi_ref, what the roll PID chases). All are targets fed
to the existing closed-loop PIDs, not raw direct forces — the pilot flies
"through" the autopilot, same architecture on every axis.

BANK_MIN/MAX are deliberately tight (+-8 deg), not a UI nicety: the
closed-loop roll axis is only verified stable for a *commanded* bank up to
~10 deg (headless swept 0-30 deg — 12 deg reliably diverges to overflow
within ~6s, even in calm water). A held bank converges to roughly 1.3-1.5x
the commanded angle (P-only roll loop, no integral — see vehicle.py/pid.py
for why Ki must stay 0) and yaw builds continuously, i.e. a real coordinated
turn — that part is correct, not a bug. Do not widen this range without
re-running that sweep.
"""

import json
import socket
import threading

HEIGHT_MIN, HEIGHT_MAX = -0.3, 1.0    # same clamp range as the dashboard's slider
SPEED_MIN, SPEED_MAX = 0.0, 10.0      # m/s — 0 lets you feel it sink with no lift
BANK_MIN, BANK_MAX = -8.0, 8.0        # deg — see module docstring for why this is tight


class PilotUplink:
    """
    UDP listener for `{"flying_height": float, "target_speed": float, "phi_ref": float}`
    packets (any subset of keys per-packet). Thread-safe, lossy-tolerant
    (UDP) by design — a dropped packet just means the next one supersedes it,
    fine for a live stick input.
    """

    def __init__(self, port: int = 5006, initial_flying_height: float = 0.55,
                 initial_target_speed: float = 4.0, initial_phi_ref: float = 0.0):
        self.port = port
        self._flying_height = initial_flying_height
        self._target_speed = initial_target_speed
        self._phi_ref = initial_phi_ref
        self._lock = threading.Lock()
        self._running = False
        self._sock = None
        self._thread = None

    @property
    def flying_height(self) -> float:
        with self._lock:
            return self._flying_height

    @property
    def target_speed(self) -> float:
        with self._lock:
            return self._target_speed

    @property
    def phi_ref(self) -> float:
        with self._lock:
            return self._phi_ref

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self.port))
        self._sock.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print(f"[PilotUplink] Listening for pilot input on UDP port {self.port}")

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _listen(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except json.JSONDecodeError:
                continue

            with self._lock:
                if "flying_height" in msg:
                    try:
                        fh = float(msg["flying_height"])
                        self._flying_height = max(HEIGHT_MIN, min(HEIGHT_MAX, fh))
                    except (ValueError, TypeError):
                        pass
                if "target_speed" in msg:
                    try:
                        ts = float(msg["target_speed"])
                        self._target_speed = max(SPEED_MIN, min(SPEED_MAX, ts))
                    except (ValueError, TypeError):
                        pass
                if "phi_ref" in msg:
                    try:
                        br = float(msg["phi_ref"])
                        self._phi_ref = max(BANK_MIN, min(BANK_MAX, br))
                    except (ValueError, TypeError):
                        pass
