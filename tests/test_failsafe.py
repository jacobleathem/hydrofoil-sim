import json
import socket
import time

import pytest

from hydrofoil.control_uplink import PilotUplink, ThrottleFailsafe
from hydrofoil.dashboard import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


# ── ThrottleFailsafe ─────────────────────────────────────────────────────

def test_failsafe_live_when_armed_with_deadman():
    fs = ThrottleFailsafe()
    assert fs.update(0.01, armed=True, deadman=True, seconds_since_packet=0.0) == 1.0
    assert fs.reason == ""


def test_failsafe_ramps_to_zero_over_ramp_time_when_disarmed():
    fs = ThrottleFailsafe(ramp_time=1.0)
    scale = fs.update(0.25, armed=False, deadman=True, seconds_since_packet=0.0)
    assert scale == pytest.approx(0.75)
    assert fs.reason == "disarmed"
    for _ in range(10):
        scale = fs.update(0.25, armed=False, deadman=True, seconds_since_packet=0.0)
    assert scale == 0.0


def test_failsafe_deadman_release_cuts():
    fs = ThrottleFailsafe()
    fs.update(0.01, armed=True, deadman=False, seconds_since_packet=0.0)
    assert fs.reason == "deadman released"
    assert fs.scale < 1.0


def test_failsafe_link_loss_cuts_and_wins_over_other_reasons():
    fs = ThrottleFailsafe(link_timeout=0.5)
    fs.update(0.01, armed=True, deadman=True, seconds_since_packet=0.4)
    assert fs.reason == ""
    fs.update(0.01, armed=False, deadman=False, seconds_since_packet=0.6)
    assert fs.reason == "pilot link lost"


def test_failsafe_no_packet_yet_is_not_link_loss():
    fs = ThrottleFailsafe()
    assert fs.update(0.01, armed=True, deadman=True, seconds_since_packet=None) == 1.0


def test_failsafe_require_pilot_cuts_until_first_packet():
    fs = ThrottleFailsafe(require_pilot=True)
    assert fs.update(0.01, armed=True, deadman=True, seconds_since_packet=None) == 0.0
    assert fs.reason == "waiting for pilot"
    assert fs.update(0.01, armed=True, deadman=True, seconds_since_packet=0.0) == 1.0


def test_failsafe_clears_at_once():
    fs = ThrottleFailsafe(ramp_time=1.0)
    for _ in range(200):
        fs.update(0.01, armed=False, deadman=True, seconds_since_packet=0.0)
    assert fs.scale == 0.0
    assert fs.update(0.01, armed=True, deadman=True, seconds_since_packet=0.0) == 1.0


# ── PilotUplink flags over real UDP ──────────────────────────────────────

def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _send_and_wait(uplink: PilotUplink, port: int, msg: dict, done) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", port))
            time.sleep(0.02)
            if done():
                return
        raise AssertionError(f"uplink never applied {msg}")
    finally:
        sock.close()


def test_uplink_reads_armed_and_deadman_flags():
    port = _free_udp_port()
    up = PilotUplink(port=port)
    up.start()
    try:
        assert up.armed and up.deadman
        assert up.seconds_since_packet() is None
        _send_and_wait(up, port, {"target_speed": 3.0, "armed": False, "deadman": False},
                       lambda: not up.armed)
        assert not up.deadman
        assert up.target_speed == pytest.approx(3.0)
        assert up.seconds_since_packet() is not None
    finally:
        up.stop()


def test_uplink_without_flags_keeps_them_set():
    port = _free_udp_port()
    up = PilotUplink(port=port)
    up.start()
    try:
        _send_and_wait(up, port, {"target_speed": 2.0},
                       lambda: up.target_speed == pytest.approx(2.0))
        assert up.armed and up.deadman
    finally:
        up.stop()


# ── VehiclePhysics thrust_scale ──────────────────────────────────────────

def _thrust_after_one_step(**overrides) -> float:
    params = dict(DEFAULT_PARAMS, **overrides)
    vp = VehiclePhysics(params)
    _, _, thrust, _, _ = vp.step(vp.rest_state(), 0.0, CalmWater())
    return thrust


def test_thrust_scale_defaults_to_live():
    assert _thrust_after_one_step() == pytest.approx(_thrust_after_one_step(thrust_scale=1.0))
    assert _thrust_after_one_step() > 0.0


def test_thrust_scale_scales_and_cuts_thrust():
    full = _thrust_after_one_step(thrust_scale=1.0)
    assert _thrust_after_one_step(thrust_scale=0.5) == pytest.approx(0.5 * full)
    assert _thrust_after_one_step(thrust_scale=0.0) == 0.0


def test_cut_never_brakes_with_reverse_thrust():
    # Unity's cut also sends target_speed 0. Moving at 5 m/s, the surge PID
    # alone would brake with reverse thrust; during a failsafe ramp it mustn't.
    params = dict(DEFAULT_PARAMS, target_speed=0.0)
    vp = VehiclePhysics(params)
    state = vp.rest_state()
    state[6] = 5.0
    assert vp.step(state, 0.0, CalmWater())[2] < 0.0      # live: PID brakes
    params["thrust_scale"] = 0.99
    vp = VehiclePhysics(params)
    assert vp.step(state, 0.0, CalmWater())[2] == 0.0     # cut: no reverse thrust
