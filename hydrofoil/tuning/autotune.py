"""
Autotuner: searches controller settings that minimise `scoring.cost` across all flight phases.

    python -m hydrofoil.tuning.autotune --preset manned --target auto [--overrides file.json] [--generations 6]
    python -m hydrofoil.tuning.autotune --preset manned --target pid
    python -m hydrofoil.tuning.autotune --preset manned --target lqr

target "auto": phase-aware combined cruise tuning (default):
               Stage 1 tunes the 8 PID gains in `controller="auto"` on Rising (takeoff, supervised climb)
               and Falling (landing & LQR->PID handback), where PID owns control.
               Stage 2 locks in those PID gains and tunes LQR weights (with `iphi`/`itheta` coupled to `phi`/`theta`)
               on Cruising & 3D Banked Turning above water (`head`, `following`, `shift`, `noisy`, `turn`, plus
               post-liftoff settle & landing transition). Returns both tuned PID gains and tuned LQR weights.
target "pid":  the eight PID gains flown on the PID alone across all scenarios.
target "lqr":  multipliers on the LQR's Bryson weights flown on the LQR across all scenarios.

Progress and the best result so far are written atomically to `--progress` (JSON) after each stage/generation.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import time
from pathlib import Path

import numpy as np

from hydrofoil.tuning.scoring import cost, phase_costs, phase_scenarios, run_scenario

PID_KEYS = ["kp_heave", "ki_heave", "kd_heave", "kp_pitch", "ki_pitch", "kd_pitch", "kp_roll", "kd_roll"]
LQR_KEYS = ["z", "theta", "phi", "w", "q", "iz", "du"]          # multipliers on control.lqr.DEFAULT_WEIGHTS


def base_params(preset: str, overrides: dict | None = None) -> dict:
    from hydrofoil.params import preset_params
    from hydrofoil.design import apply_design
    p = preset_params(preset)
    return apply_design(p, overrides or {})


def candidate_params(base: dict, target: str, x: np.ndarray, pid_override: dict | None = None) -> dict:
    """Parameter set for log-space vector x (relative to the base values / default weights)."""
    from hydrofoil.control.lqr import DEFAULT_WEIGHTS
    p = dict(base)
    if pid_override:
        p.update(pid_override)
    if target in ("pid", "auto_pid"):
        p["controller"] = "auto" if target == "auto_pid" else "pid"
        for k, v in zip(PID_KEYS, x):
            p[k] = float(np.exp(v))
        if "lqr_weights" not in p:
            p["lqr_weights"] = dict(DEFAULT_WEIGHTS)
    else:
        p["controller"] = "auto" if target == "auto_lqr" else "lqr"
        w = dict(DEFAULT_WEIGHTS, **(base.get("lqr_weights") or {}))
        mult = dict(zip(LQR_KEYS, np.exp(x)))
        # Bryson weight = acceptable excursion: a larger multiplier means LESS weight on that state
        for k in ("z", "theta", "phi", "w", "q", "iz"):
            w[k] = float(w[k] * mult[k])
        # Couple integral roll & pitch weights proportionally to angle weights so banked turns stay well-damped
        w["iphi"] = float(w["iphi"] * mult["phi"])
        w["itheta"] = float(w["itheta"] * mult["theta"])
        for k in ("du_main", "du_elev", "du_roll"):
            w[k] = float(w[k] * mult["du"])
        p["lqr_weights"] = w
    return p


def start_vector(base: dict, target: str) -> np.ndarray:
    if target in ("pid", "auto_pid"):
        return np.log(np.array([max(base[k], 1e-3) for k in PID_KEYS]))
    return np.zeros(len(LQR_KEYS))


def _ce_search(evaluate_fn, init_x: np.ndarray, gens: int, pop: int, sigma: float,
               rng, phase_label: str | None, hist: list, on_candidate, extra_cost: float = 0.0):
    """Cross-entropy log-space refinement shared across all tuning targets and flight phases."""
    mean = np.array(init_x, float)
    sd = np.full_like(mean, sigma)
    for g in range(gens):
        xs = [mean + sd * rng.standard_normal(len(mean)) for _ in range(pop - 1)] + [mean]
        res = evaluate_fn(xs)
        order = np.argsort([r[0] for r in res])
        elite = [xs[i] for i in order[:max(2, pop // 4)]]
        mean = 0.7 * np.mean(elite, axis=0) + 0.3 * mean
        sd = np.maximum(0.7 * np.std(elite, axis=0) + 0.3 * sd, 0.03)
        c, mets, pp = res[order[0]]
        state_best = on_candidate(g + 1, xs[order[0]], c, mets, pp)
        row = {"generation": len(hist) + 1, "best_cost": state_best["cost"],
               "best_phase_costs": state_best["phase_costs"],
               "gen_best": float(c + extra_cost),
               "gen_median": float(np.median([r[0] for r in res])) + extra_cost}
        if phase_label:
            row["phase"] = phase_label
        hist.append(row)
        state_best["write"]()


def tune(preset: str, target: str, overrides: dict | None = None, generations: int = 6, pop: int = 10,
         sigma: float = 0.35, progress: Path | None = None, workers: int | None = None, seed: int = 0,
         bryson_rounds: int = 4) -> dict:
    """Staged tuning loop supporting:
        target='auto': Phase-aware combined tuning (1: Baseline -> 2: Rising/Falling PID -> 3: Cruising/Turning LQR)
        target='pid':  Relay rules + CE refinement on PID gains alone
        target='lqr':  Bryson re-weighting + CE refinement on LQR weights alone"""
    if target not in ("auto", "pid", "lqr"):
        raise ValueError(f"unknown tune target {target!r}; expected 'auto', 'pid', or 'lqr'")
    from hydrofoil.tuning.classical import pid_candidates, bryson_update
    base = base_params(preset, overrides)
    all_sc = phase_scenarios(base)
    rf_sc = tuple(s for s in all_sc if s in (*phase_scenarios(base, "rising"), *phase_scenarios(base, "falling")))
    cr_sc = tuple(s for s in all_sc if s in phase_scenarios(base, "cruising"))
    scale = base["flying_height"]
    rng = np.random.default_rng(seed)
    workers = workers or max(1, (mp.cpu_count() or 2) - 1)
    stages, hist = [], []
    t0 = time.time()
    total_gens = (max(1, generations // 2) + max(1, generations - generations // 2)) if target == "auto" else generations
    state = {"best": None, "status": "running", "relay": None}

    def write():
        if progress is None:
            return
        b = state["best"]
        payload = json.dumps({
            "status": state["status"], "preset": preset, "target": target, "elapsed_s": round(time.time() - t0, 1),
            "stages": stages, "relay": state["relay"], "history": hist, "generations": total_gens,
            "best": b and {"cost": b["cost"], "phase_costs": b["phase_costs"], "values": b["values"],
                           "metrics": b["metrics"], "source": b["source"]}},
            indent=1, default=float)
        progress.parent.mkdir(parents=True, exist_ok=True)
        tmp = progress.with_name(progress.name + ".tmp")
        tmp.write_text(payload, encoding="utf-8")
        os.replace(tmp, progress)

    with mp.get_context("spawn").Pool(workers) as pool:
        def evaluate(xs, sub_target: str | None = None, scen_list: tuple[str, ...] = all_sc,
                     pid_override: dict | None = None):
            st = sub_target or target
            params = [candidate_params(base, st, x, pid_override=pid_override) for x in xs]
            jobs = [(pp, n) for pp in params for n in scen_list]
            res = pool.starmap(run_scenario, jobs)
            out = []
            k = len(scen_list)
            for i, pp in enumerate(params):
                mets = dict(zip(scen_list, res[i * k:(i + 1) * k]))
                out.append((cost(mets, pp["flying_height"]), mets, pp))
            return out

        def record_best(x, c, mets, pp, source, add_stage=True):
            pc = phase_costs(mets, scale)
            vals = _values(pp, target)
            if add_stage:
                stages.append({"name": source, "cost": c, "phase_costs": pc, "values": vals, "metrics": mets})
            if state["best"] is None or c < state["best"]["cost"]:
                state["best"] = {"cost": c, "phase_costs": pc, "x": np.array(x, float),
                                 "metrics": mets, "values": vals, "source": source}
            write()

        def std_on_cand(prefix: str):
            def _cb(gen, x, c, mets, pp):
                if c < state["best"]["cost"]:
                    record_best(x, c, mets, pp, f"{prefix}, round {gen}", add_stage=False)
                return {"cost": state["best"]["cost"], "phase_costs": state["best"]["phase_costs"], "write": write}
            return _cb

        write()
        init_sub = "auto_lqr" if target == "auto" else target
        x0 = start_vector(base, init_sub)
        (c0, m0, pp0), = evaluate([x0], sub_target=init_sub, scen_list=all_sc)
        record_best(x0, c0, m0, pp0, "current values" if target != "auto" else "current values (auto baseline)")
        baseline = {"cost": c0, "phase_costs": phase_costs(m0, scale), "values": _values(pp0, target), "metrics": m0}

        if target == "auto":
            # Stage 1: Tune Rising (takeoff, supervised) + Falling (landing) PID in auto mode
            cr_mets = {k: v for k, v in m0.items() if k in cr_sc}
            cr_base_cost = cost(cr_mets, scale)
            best_pid = {"cost": cost({k: m0[k] for k in rf_sc}, scale),
                        "x": start_vector(base, "auto_pid"), "values": _values(pp0, "auto_pid")}

            cands = pid_candidates(base)
            state["relay"] = cands.pop("_relay")
            names = list(cands)
            xs_p = [np.log(np.array([max(cands[n].get(k, base[k]), 1e-4) for k in PID_KEYS])) for n in names]
            for n, x, (c_rf, m_rf, pp_rf) in zip(names, xs_p, evaluate(xs_p, sub_target="auto_pid", scen_list=rf_sc)):
                full_m = {**cr_mets, **m_rf}
                full_c = cost(full_m, scale)
                if c_rf < best_pid["cost"]:
                    best_pid.update(cost=c_rf, x=x.copy(), values=_values(pp_rf, "auto_pid"))
                record_best(x, full_c, full_m, pp_rf, f"Rising/Falling PID: {n}")

            pid_gens = max(1, generations // 2)

            def on_pid_gen(gen, x, c_rf, m_rf, pp_rf):
                full_m = {**cr_mets, **m_rf}
                full_c = cost(full_m, scale)
                if c_rf < best_pid["cost"]:
                    best_pid.update(cost=c_rf, x=x.copy(), values=_values(pp_rf, "auto_pid"))
                if full_c < state["best"]["cost"]:
                    record_best(x, full_c, full_m, pp_rf, f"Rising/Falling PID search, round {gen}", add_stage=False)
                return {"cost": state["best"]["cost"], "phase_costs": state["best"]["phase_costs"], "write": write}

            _ce_search(lambda xs: evaluate(xs, sub_target="auto_pid", scen_list=rf_sc),
                       best_pid["x"], pid_gens, pop, sigma, rng, "rising_falling_pid", hist,
                       on_pid_gen, extra_cost=cr_base_cost)

            # Stage 2: Lock best Rising/Falling PID, tune Cruising & 3D Turning LQR across all phases
            mult = dict.fromkeys(LQR_KEYS, 1.0)
            last_m = state["best"]["metrics"]
            best_lqr_x = np.zeros(len(LQR_KEYS))
            for r in range(bryson_rounds):
                mult = bryson_update(mult, last_m, scale)
                x_l = np.log(np.array([mult[k] for k in LQR_KEYS]))
                (c_l, last_m, pp_l), = evaluate([x_l], sub_target="auto_lqr", scen_list=all_sc,
                                                pid_override=best_pid["values"])
                if c_l <= state["best"]["cost"]:
                    best_lqr_x = x_l.copy()
                record_best(x_l, c_l, last_m, pp_l, f"Cruising/Turn LQR Bryson, round {r + 1}")

            _ce_search(lambda xs: evaluate(xs, sub_target="auto_lqr", scen_list=all_sc, pid_override=best_pid["values"]),
                       best_lqr_x, max(1, generations - pid_gens), pop, sigma, rng, "cruising_turn_lqr",
                       hist, std_on_cand("Cruising/Turn LQR search"))
        elif target == "pid":
            cands = pid_candidates(base)
            state["relay"] = cands.pop("_relay")
            names = list(cands)
            xs = [np.log(np.array([max(cands[n].get(k, base[k]), 1e-4) for k in PID_KEYS])) for n in names]
            for n, x, (c, m, pp) in zip(names, xs, evaluate(xs)):
                record_best(x, c, m, pp, n)
            _ce_search(evaluate, state["best"]["x"], generations, pop, sigma, rng, None, hist, std_on_cand("search"))
        else:
            mult = dict.fromkeys(LQR_KEYS, 1.0)
            last_m = m0
            for r in range(bryson_rounds):
                mult = bryson_update(mult, last_m, scale)
                x = np.log(np.array([mult[k] for k in LQR_KEYS]))
                (c, last_m, pp), = evaluate([x])
                record_best(x, c, last_m, pp, f"Bryson re-weighting, round {r + 1}")
            _ce_search(evaluate, state["best"]["x"], generations, pop, sigma, rng, None, hist, std_on_cand("search"))
    state["status"] = "done"
    write()
    b = state["best"]
    return {"baseline": baseline,
            "best": {"cost": b["cost"], "phase_costs": b["phase_costs"], "values": b["values"],
                     "metrics": b["metrics"], "source": b["source"]},
            "stages": stages, "relay": state["relay"]}


def _values(p: dict, target: str) -> dict:
    from hydrofoil.control.lqr import DEFAULT_WEIGHTS
    if target in ("pid", "auto_pid"):
        return {k: round(float(p[k]), 4) for k in PID_KEYS}
    if target == "auto":
        vals = {k: round(float(p[k]), 4) for k in PID_KEYS}
        w = p.get("lqr_weights") or DEFAULT_WEIGHTS
        vals["lqr_weights"] = {k: float(v) for k, v in w.items()}
        return vals
    w = p.get("lqr_weights") or DEFAULT_WEIGHTS
    return {"lqr_weights": {k: float(v) for k, v in w.items()}}


def main():
    ap = argparse.ArgumentParser(description="Autotune the flight controller across all flight phases")
    ap.add_argument("--preset", choices=("model", "manned", "cad"), default="manned")
    ap.add_argument("--target", choices=("auto", "pid", "lqr"), default="auto")
    ap.add_argument("--overrides", type=Path, help="JSON design overrides (boat editor)")
    ap.add_argument("--generations", type=int, default=6)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--progress", type=Path, help="JSON progress file, rewritten every generation")
    ap.add_argument("--out", type=Path, help="write the best values here (JSON)")
    a = ap.parse_args()
    ov = json.loads(a.overrides.read_text()) if a.overrides and a.overrides.exists() else None
    res = tune(a.preset, a.target, ov, a.generations, a.pop, progress=a.progress)
    b0, b1 = res["baseline"], res["best"]
    print(f"cost {b0['cost']:.2f} -> {b1['cost']:.2f} (best from: {b1['source']})")
    for st in res["stages"]:
        print(f"  {st['name']:36s} cost {st['cost']:7.2f}")
    print(json.dumps(b1["values"], indent=1))
    for n in b0["metrics"]:
        print(n, "before", {k: round(v, 3) if isinstance(v, float) else v for k, v in b0["metrics"][n].items()})
        print(n, "after ", {k: round(v, 3) if isinstance(v, float) else v for k, v in b1["metrics"][n].items()})
    if a.out:
        a.out.write_text(json.dumps(b1["values"], indent=1))


if __name__ == "__main__":
    main()
