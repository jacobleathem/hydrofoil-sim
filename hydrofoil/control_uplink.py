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

BANK_MIN/MAX (+-20 deg) are a safety margin, not a UI nicety. With strut
side force in the plant and the retuned roll gains (dashboard.DEFAULT_PARAMS),
a held bank was swept headless from 8 to 40 deg, in calm water and 5 cm
waves, at 6 and 9 m/s: everything up to 30 deg settles on the command, and
40 deg ran away to the attitude clamp in waves at 9 m/s. 20 deg keeps half
of that margin. (The old cap was +-8 deg: on the strut-less plant with the
old P-only gains, 12 deg diverged.) Do not widen this range without
re-running that sweep.

Failsafe: packets may also carry "armed" and "deadman" booleans (Unity's
PilotInput sends both). ThrottleFailsafe turns those, plus pilot-link loss
(no packet for link_timeout seconds), into a thrust scale that ramps the
motors down to zero, like a kill cord or RC-loss failsafe. A client that
never sends the flags (the dashboard, older Unity builds) is treated as
armed with the deadman held, so its behaviour is unchanged.
"""

import json
import socket
import threading
import time

HEIGHT_MIN, HEIGHT_MAX = -0.3, 1.0    # same clamp range as the dashboard's slider
SPEED_MIN, SPEED_MAX = 0.0, 10.0      # m/s — 0 lets you feel it sink with no lift
BANK_MIN, BANK_MAX = -20.0, 20.0      # deg — see module docstring for where this comes from


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
        self._armed = True
        self._deadman = True
        self._last_packet = None        # time.monotonic() of the last valid packet
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

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def deadman(self) -> bool:
        with self._lock:
            return self._deadman

    def seconds_since_packet(self, now: float | None = None) -> float | None:
        """Seconds since the last valid pilot packet, or None if none has arrived yet."""
        with self._lock:
            last = self._last_packet
        if last is None:
            return None
        return (time.monotonic() if now is None else now) - last

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
            if not isinstance(msg, dict):
                continue

            with self._lock:
                self._last_packet = time.monotonic()
                for key in ("armed", "deadman"):
                    if isinstance(msg.get(key), bool):
                        setattr(self, "_" + key, msg[key])
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


class ThrottleFailsafe:
    """
    Turns the pilot link's safety state into a thrust scale in [0, 1].

    Thrust is live while the pilot is armed with the deadman held and packets
    keep arriving. Any cut (disarmed, deadman released, or no packet for
    link_timeout seconds) ramps the scale down to zero over ramp_time
    instead of commanding speed 0, which would make the surge PID brake with
    reverse thrust. When the cut clears, thrust is live again at once: arming
    already requires the throttle at zero.

    Before the first packet arrives the link isn't treated as lost, so the
    sim still runs on its own (headless, or with a client that never connects).
    With require_pilot, thrust instead stays cut until the first packet, so
    the boat doesn't set off at the initial target speed before the pilot's
    client has connected.
    """

    def __init__(self, link_timeout: float = 0.5, ramp_time: float = 1.0,
                 require_pilot: bool = False):
        self.link_timeout = link_timeout
        self.ramp_time = ramp_time
        self.require_pilot = require_pilot
        self.scale = 0.0 if require_pilot else 1.0   # waiting for the pilot starts cut, not ramping
        self.reason = ""        # why thrust is cut, "" while live

    def cut_reason(self, armed: bool, deadman: bool, seconds_since_packet: float | None) -> str:
        if seconds_since_packet is None and self.require_pilot:
            return "waiting for pilot"
        if seconds_since_packet is not None and seconds_since_packet > self.link_timeout:
            return "pilot link lost"
        if not armed:
            return "disarmed"
        if not deadman:
            return "deadman released"
        return ""

    def update(self, dt: float, armed: bool, deadman: bool, seconds_since_packet: float | None) -> float:
        """Advance one step and return the thrust scale."""
        self.reason = self.cut_reason(armed, deadman, seconds_since_packet)
        if self.reason:
            step = dt / self.ramp_time if self.ramp_time > 0 else 1.0
            self.scale = max(0.0, self.scale - step)
        else:
            self.scale = 1.0
        return self.scale
