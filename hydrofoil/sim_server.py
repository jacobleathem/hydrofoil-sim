"""
Web launcher, boat editor, and autotuner dashboard for the sim: a page at http://127.0.0.1:8765 that starts / stops
the MuJoCo viewer (`python -m hydrofoil.viewer`) with the chosen options.

Run:  python -m hydrofoil.sim_server [--port 8765]
"""

from __future__ import annotations

import argparse
import html
import json
import shutil
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ASSETS = Path(__file__).resolve().parent / "assets" / "launcher"
PAGE = ASSETS / "page.html"
FONTS = ASSETS / "fonts"

STATE_DIR = ROOT / "local"                     # gitignored runtime directory for logs & saved overrides
LOG = STATE_DIR / "viewer.log"
OVERRIDES = STATE_DIR / "overrides.json"       # boat-editor edits and tuned gains, per preset: {"manned": {...}, "model": {...}}
TUNE_PROGRESS = STATE_DIR / "tune_progress.json"

tune_proc: subprocess.Popen | None = None
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
proc: subprocess.Popen | None = None
last_cmd: list[str] = []
_last_tune_data: dict = {"status": "idle"}
_auto_saved_sig: tuple | None = None

FIELDS = {   # name: (choices or None for free numeric, default)
    "preset": (("cad", "manned", "model"), "cad"),
    "cockpit_buoyancy": (("on", "off"), "on"),
    "supervisor": (("off", "on"), "on"),
    "controller": (("auto", "lqr", "pid"), "auto"),
    "allocator": (("jacobian", "legacy"), "jacobian"),
    "safety_margin": (("conservative", "standard", "sport"), "standard"),
    "sensors": (("ideal", "noisy"), "ideal"),
    "waves": (("off", "on"), "off"),
    "wave_type": (("regular", "irregular"), "regular"),
    "wave_dir": (None, "0"),
    "speed": (None, "1.0"),
}

LABELS = {
    "preset": "Boat",
    "cockpit_buoyancy": "Cockpit box buoyancy (CAD boat)",
    "supervisor": "Supervisor",
    "controller": "Flight controller",
    "allocator": "Flap allocator",
    "safety_margin": "Safety margin",
    "sensors": "Sensors",
    "waves": "Waves",
    "wave_type": "Wave spectrum",
    "wave_dir": "Wave direction (deg)",
    "speed": "Sim speed",
}

API_PORT = 8766
last_form: dict = {}


def _ensure_state_dir() -> Path:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    return STATE_DIR


def running() -> bool:
    return proc is not None and proc.poll() is None


def _viewer_python() -> str:
    """Use mjpython beside sys.executable or on PATH when running MuJoCo viewer on macOS."""
    if sys.platform == "darwin":
        cand = Path(sys.executable).with_name("mjpython")
        if cand.is_file():
            return str(cand)
        found = shutil.which("mjpython")
        if found:
            return found
    return sys.executable


def start(form: dict) -> None:
    global proc, last_cmd
    stop()
    _ensure_state_dir()
    v = {k: form.get(k, [d])[0] for k, (_c, d) in FIELDS.items()}
    for k, (choices, d) in FIELDS.items():
        if choices and v[k] not in choices:
            v[k] = d
        if not choices:
            try:
                v[k] = str(float(v[k]))
            except ValueError:
                v[k] = d
    cmd = [_viewer_python(), "-m", "hydrofoil.viewer", "--preset", v["preset"], "--controller", v["controller"],
           "--allocator", v["allocator"], "--safety-margin", v["safety_margin"],
           "--sensors", v["sensors"], "--speed", v["speed"], "--wave-dir", v["wave_dir"],
           "--wave-type", v["wave_type"]]
    if v["supervisor"] == "on":
        cmd.append("--supervisor")
    if v["preset"] == "cad":
        cmd += ["--cockpit-buoyancy", v["cockpit_buoyancy"]]
    if v["waves"] == "on":
        cmd.append("--waves")
    cmd += ["--duration", "36000", "--api-port", str(API_PORT)]
    ov = load_overrides().get(v["preset"])
    if ov:
        f = STATE_DIR / f"overrides_{v['preset']}.json"
        f.write_text(json.dumps(ov))
        cmd += ["--overrides", str(f)]
    last_cmd = cmd
    last_form.update(v)
    with open(LOG, "w") as log_fh:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log_fh, stderr=subprocess.STDOUT, creationflags=NO_WINDOW)


def stop() -> None:
    global proc
    if running():
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    proc = None


def load_overrides() -> dict:
    try:
        return json.loads(OVERRIDES.read_text())
    except (OSError, ValueError):
        return {}


def save_overrides(d: dict):
    _ensure_state_dir()
    OVERRIDES.write_text(json.dumps(d, indent=1))


EDIT_KEYS = ["main_chord", "main_span", "main_incidence_deg", "main_flap_frac", "elev_chord", "elev_span", "elev_pos_x",
             "flying_height", "target_speed", "max_thrust"]


def design_info(preset: str) -> dict:
    """Preset values for the editor fields, and what is saved for this preset."""
    from hydrofoil.params import preset_params
    p = preset_params(preset)
    payloads = p.get("default_payloads", [])
    base = {k: p[k] for k in EDIT_KEYS}
    base["strut_length"] = p["hull_offset"] + p["main_pos_z"]
    base["empty_mass"] = float(p["mass"] - sum(float(q["mass"]) for q in payloads))
    geom = {k: p[k] for k in ("hull_length", "hull_beam", "hull_x", "main_lateral_offset")}
    geom["hull_twin_spacing"] = float(p.get("hull_twin_spacing", 0.0) or 0.0)
    return {"preset": preset, "base": base, "payloads": payloads, "geom": geom,
            "saved": load_overrides().get(preset, {})}


def design_preview(preset: str, overrides: dict) -> dict:
    from hydrofoil.params import preset_params
    from hydrofoil.design import apply_design, derived, design_summary
    p = preset_params(preset)
    design_only = {k: v for k, v in overrides.items() if k not in ("lqr_weights",) and not k.startswith(("kp_", "ki_", "kd_"))}
    e = apply_design(p, design_only)
    base_d = derived(p, include_summary=False)
    edit_d = derived(e, include_summary=False)
    base_d["summary"] = design_summary(None, base_d, p)
    edit_d["summary"] = design_summary(base_d, edit_d, e)
    return {"base": base_d, "edited": edit_d,
            "edited_geom": {k: e[k] for k in ("main_chord", "main_span", "main_lateral_offset", "elev_chord", "elev_span",
                                               "elev_pos_x", "hull_length", "hull_beam", "hull_x")}
                           | {"hull_twin_spacing": float(e.get("hull_twin_spacing", 0.0) or 0.0)}}


def _persist_tuned(preset: str, vals: dict) -> None:
    allov = load_overrides()
    ov = allov.setdefault(preset, {})
    ov.update(vals)
    save_overrides(allov)
    (_ensure_state_dir() / f"overrides_{preset}.json").write_text(json.dumps(ov, indent=1), encoding="utf-8")


def stored_controller_values(preset: str) -> dict:
    """Return both preset defaults and persisted backend overrides for PID gains and LQR weights."""
    from hydrofoil.params import preset_params
    from hydrofoil.control.lqr import DEFAULT_WEIGHTS
    from hydrofoil.tuning.autotune import PID_KEYS
    p = preset_params(preset)
    ov = load_overrides().get(preset, {})
    defaults_pid = {k: float(p[k]) for k in PID_KEYS}
    stored_pid = {k: float(ov.get(k, p[k])) for k in PID_KEYS}
    defaults_lqr = {k: float(v) for k, v in dict(DEFAULT_WEIGHTS, **(p.get("lqr_weights") or {})).items()}
    stored_lqr = {k: float(v) for k, v in dict(defaults_lqr, **(ov.get("lqr_weights") or {})).items()}
    return {
        "preset": preset,
        "has_tuned_pid": any(k in ov for k in PID_KEYS),
        "has_tuned_lqr": "lqr_weights" in ov,
        "defaults": {"pid": defaults_pid, "lqr_weights": defaults_lqr},
        "stored": {"pid": stored_pid, "lqr_weights": stored_lqr},
        "overrides_path": str(OVERRIDES),
    }


def tune_start(preset: str, target: str):
    global tune_proc, _last_tune_data, _auto_saved_sig
    if tune_proc is not None and tune_proc.poll() is None:
        return False
    _ensure_state_dir()
    _auto_saved_sig = None
    ov = load_overrides().get(preset)
    f = STATE_DIR / f"overrides_{preset}.json"
    f.write_text(json.dumps(ov or {}))
    _last_tune_data = {"status": "starting", "preset": preset, "target": target}
    TUNE_PROGRESS.write_text(json.dumps(_last_tune_data))
    cmd = [sys.executable, "-m", "hydrofoil.tuning.autotune", "--preset", preset, "--target", target,
           "--overrides", str(f), "--progress", str(TUNE_PROGRESS)]
    with open(STATE_DIR / "tune.log", "w") as log_fh:
        tune_proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log_fh, stderr=subprocess.STDOUT,
                                     creationflags=NO_WINDOW)
    return True


def tune_status(preset_hint: str | None = None) -> dict:
    global _last_tune_data, _auto_saved_sig
    running_now = tune_proc is not None and tune_proc.poll() is None
    d = None
    try:
        if TUNE_PROGRESS.exists():
            d = json.loads(TUNE_PROGRESS.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        d = None
    if d is None:
        d = dict(_last_tune_data) if (running_now or _last_tune_data.get("status") != "idle") else {"status": "idle"}
    else:
        _last_tune_data = dict(d)
    if tune_proc is not None and tune_proc.poll() not in (None, 0) and d.get("status") != "done":
        d["status"] = "failed"
        log_path = STATE_DIR / "tune.log"
        d["log"] = log_path.read_text(errors="replace")[-1500:] if log_path.exists() else ""
    preset = d.get("preset") or preset_hint or last_form.get("preset", "manned")
    best = d.get("best")
    stages = d.get("stages") or []
    improved = bool(best and (d.get("status") == "done" or not stages or best["cost"] <= stages[0]["cost"]))
    if best and improved:
        sig = (preset, d.get("target"), round(float(best.get("cost", 0.0)), 6), best.get("source"))
        if sig != _auto_saved_sig:
            _persist_tuned(preset, best["values"])
            _auto_saved_sig = sig
        d["auto_saved"] = True
    d["stored"] = stored_controller_values(preset_hint or preset)
    return d


def tune_apply() -> dict:
    d = tune_status()
    if not d.get("best"):
        return {"ok": False}
    preset = d.get("preset") or last_form.get("preset", "manned")
    vals = d["best"]["values"]
    _persist_tuned(preset, vals)
    return {"ok": True, "applied": vals, "stored": stored_controller_values(preset)}


def page() -> str:
    def select(name):
        choices, d = FIELDS[name]
        cur = last_form.get(name, d)
        from hydrofoil.params import PRESET_LABELS
        labels = (PRESET_LABELS if name == "preset" else
                  {"on": "On (box floats)", "off": "Off (pontoons scaled to hold the boat)"} if name == "cockpit_buoyancy"
                  else {})
        opts = "".join(f'<option value="{c}"{" selected" if c == cur else ""}>{html.escape(labels.get(c, c))}</option>'
                       for c in choices)
        return f'<label>{LABELS[name]}<select name="{name}">{opts}</select></label>'

    def number(name):
        return (f'<label>{LABELS[name]}<input type="text" name="{name}" '
                f'value="{html.escape(last_form.get(name, FIELDS[name][1]))}" inputmode="decimal"></label>')

    status = "running" if running() else ("stopped" if proc is None else f"exited ({proc.returncode})")
    log = html.escape(LOG.read_text(errors="replace")[-3000:]) if LOG.exists() else ""
    fields = "".join(select(n) if FIELDS[n][0] else number(n) for n in FIELDS)
    return (PAGE.read_text(encoding="utf-8").replace("__STATUS__", status).replace("__FIELDS__", fields)
            .replace("__LOG__", log or "no output yet").replace("__CMD__", html.escape(" ".join(last_cmd)))
            .replace("__APIPORT__", str(API_PORT)))


class Handler(BaseHTTPRequestHandler):
    def _send(self, body: str, code: int = 200):
        b = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _json(self, obj, code: int = 200):
        b = json.dumps(obj, default=float).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._send(page())
        if self.path.startswith("/api/design"):
            preset = parse_qs(self.path.partition("?")[2]).get("preset", [last_form.get("preset", "manned")])[0]
            return self._json(design_info(preset))
        if self.path.startswith("/api/tune/status"):
            preset = parse_qs(self.path.partition("?")[2]).get("preset", [None])[0]
            return self._json(tune_status(preset))
        name = self.path.split("?")[0].removeprefix("/fonts/")
        f = (FONTS / name).resolve()
        if (self.path.startswith("/fonts/") and "/" not in name and "\\" not in name
                and f.is_relative_to(FONTS.resolve()) and f.is_file()):   # bundled fonts, no internet needed
            b = f.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", {".css": "text/css", ".woff2": "font/woff2"}.get(f.suffix, "text/plain"))
            self.send_header("Cache-Control", "max-age=86400")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        self._send("not found", 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n).decode()
        if self.path.startswith("/api/"):
            try:
                body = json.loads(raw or "{}")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            preset = body.get("preset") or last_form.get("preset", "manned")
            try:
                if self.path == "/api/design/preview":
                    return self._json(design_preview(preset, body.get("overrides", {})))
                if self.path == "/api/design/save":
                    allov = load_overrides()
                    keep = {k: v for k, v in allov.get(preset, {}).items()
                            if k == "lqr_weights" or k.startswith(("kp_", "ki_", "kd_"))}      # tuned gains survive
                    allov[preset] = {**keep, **body.get("overrides", {})}
                    if body.get("reset"):
                        allov[preset] = keep
                    save_overrides(allov)
                    return self._json({"ok": True})
                if self.path == "/api/load_cad":
                    # "Load our CAD boat": drop every saved edit and tuned gain for the CAD preset so the boat is
                    # exactly CAD_PARAMS (hydrofoil/params.py), then (re)launch the 3D view with it.
                    allov = load_overrides()
                    allov.pop("cad", None)
                    save_overrides(allov)
                    f = STATE_DIR / "overrides_cad.json"
                    if f.exists():
                        f.unlink()
                    form = {k: [v] for k, v in last_form.items()}
                    form["preset"] = ["cad"]
                    start(form)
                    return self._json({"ok": True, "preset": "cad"})
                if self.path == "/api/relaunch":
                    form = {k: [v] for k, v in last_form.items()} or {"preset": [preset]}
                    form["preset"] = [preset]
                    start(form)
                    return self._json({"ok": True})
                if self.path == "/api/tune/start":
                    return self._json({"ok": tune_start(preset, body.get("target", "auto"))})
                if self.path == "/api/tune/apply":
                    return self._json(tune_apply())
                if self.path == "/api/tune/clear":
                    global _last_tune_data, _auto_saved_sig
                    allov = load_overrides()
                    allov[preset] = {k: v for k, v in allov.get(preset, {}).items()
                                     if not (k == "lqr_weights" or k.startswith(("kp_", "ki_", "kd_")))}
                    save_overrides(allov)
                    if (STATE_DIR / f"overrides_{preset}.json").exists():
                        (STATE_DIR / f"overrides_{preset}.json").write_text(json.dumps(allov[preset], indent=1))
                    if tune_proc is None or tune_proc.poll() is not None:
                        _last_tune_data = {"status": "idle"}
                        _auto_saved_sig = None
                        if TUNE_PROGRESS.exists():
                            TUNE_PROGRESS.unlink()
                    return self._json({"ok": True, "stored": stored_controller_values(preset)})
            except Exception as e:                                  # report, do not kill the launcher
                return self._json({"error": f"{type(e).__name__}: {e}"}, 500)
            return self._json({"error": "not found"}, 404)
        form = parse_qs(raw)
        if self.path == "/start":
            start(form)
        elif self.path == "/stop":
            stop()
        self.send_response(303)
        self.send_header("Location", "/")
        self.end_headers()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--api-port", type=int, default=8766)
    ap.add_argument("--no-browser", action="store_true", help="do not open the dashboard in the default browser")
    a = ap.parse_args()
    API_PORT = a.api_port
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"Hydrofoil sim launcher on http://127.0.0.1:{a.port}", flush=True)
    try:
        if not a.no_browser:
            import webbrowser
            webbrowser.open(f"http://127.0.0.1:{a.port}")
        srv.serve_forever()
    finally:
        stop()
        if tune_proc is not None and tune_proc.poll() is None:
            tune_proc.terminate()
