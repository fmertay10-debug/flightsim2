"""Batch DATCOM dataset generation for surrogate ML models (ROADMAP I5).

Workflow (see D2/D8/D9 in docs/ROADMAP.md):

1. ``sample_params`` — Latin Hypercube sample of geometry parameters.
2. ``rocket_from_params`` — map a parameter dict to a consistent `Vehicle`
   (lengths and CG derived so sampled values can't contradict each other).
3. ``generate_dataset`` — run the pipeline once per sample, each in its own
   ``runs/<id>/`` directory (raw, resumable ground truth: for005/for006,
   ``aero.npz`` incl. fill masks, ``params.json``). Failures are logged to
   ``failures.jsonl`` and skipped, never fatal.
4. ``build_dataset`` — flatten every successful run into training tables:
   ``dataset_static.csv``  one row per (run, mach, alpha): geometry params +
   static/dynamic coefficients + quality flags — ``<name>_filled`` (D9) and
   ``transonic`` (0.6 < M < 1.4: extended-method data, low confidence, D10);
   ``dataset_control.csv`` one row per (run, mach, delta): pitch/roll
   control increments (only when a deflection schedule was run).
   The 3-D tables (``dcdi_sym``, ``cn_asy``) stay in the per-run ``aero.npz``.

Runs are serial (start-small, D2); ``_run_one`` is self-contained so a
process pool can drop in later without redesign.
"""
from __future__ import annotations

import csv
import json
import warnings
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .importer import _STAT_COEFFS, import_datcom
from .pipeline import run_pipeline
from .vehicle import Vehicle

# Control increments flattened into dataset_control.csv ([ndelta x nmach])
_CTRL_2D = ["dcl_sym", "dcm_sym", "dclmax_sym", "dcdmin_sym", "clroll"]


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------
def sample_params(ranges: Mapping[str, Tuple[float, float]], n: int,
                  seed: int = 0) -> List[Dict[str, float]]:
    """Latin Hypercube sample: ``n`` dicts drawn from ``{name: (lo, hi)}``.

    Each parameter's range is split into ``n`` equal bins with exactly one
    sample per bin (stratified), bins shuffled independently per parameter.
    Deterministic for a given ``seed``.
    """
    if n < 1:
        raise ValueError("n must be >= 1")
    rng = np.random.default_rng(seed)
    names = list(ranges)
    cols = {}
    for name in names:
        lo, hi = ranges[name]
        if hi < lo:
            raise ValueError(f"range for {name!r} has hi < lo")
        u = (rng.permutation(n) + rng.random(n)) / n   # one point per bin
        cols[name] = lo + u * (hi - lo)
    return [{name: float(cols[name][i]) for name in names} for i in range(n)]


def rocket_from_params(p: Mapping[str, float]) -> Vehicle:
    """Default factory: parameter dict -> consistent slender-body `Vehicle`.

    Any `Vehicle` field name may appear. Derived afterwards so sampled
    values can never contradict each other:

    * ``L``   = ``L_nc + L_af + L_bt`` (total length is never sampled)
    * ``xcg`` = ``xcg_frac * L`` if the pseudo-parameter ``xcg_frac`` is
      given (recommended over sampling ``xcg`` directly)
    * ``fin_num`` is rounded to an int.
    """
    v = Vehicle()
    p = dict(p)
    xcg_frac = p.pop("xcg_frac", None)
    for key, val in p.items():
        if not hasattr(v, key):
            raise KeyError(f"unknown Vehicle field {key!r}")
        setattr(v, key, val)
    v.fin_num = int(round(v.fin_num))
    v.L = v.L_nc + v.L_af + v.L_bt
    if xcg_frac is not None:
        v.xcg = xcg_frac * v.L
    return v


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------
def _run_one(run_dir: Path, params: Mapping[str, float],
             factory: Callable[[Mapping[str, float]], Vehicle],
             M, alpha, delta, alt, exe_path, timeout) -> None:
    """One sample: build vehicle, run pipeline, save aero.npz + params.json."""
    vehicle = factory(params)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")   # transonic warning is per-campaign
        aero = run_pipeline(vehicle, M, alpha, delta, outdir=str(run_dir),
                            alt=alt, exe_path=exe_path, timeout=timeout)
    aero.save_npz(str(run_dir / "aero.npz"))
    meta = {"params": dict(params), "alt": alt,
            "delta": list(delta) if delta else None}
    (run_dir / "params.json").write_text(json.dumps(meta, indent=1))


def generate_dataset(outdir: str,
                     params_list: Optional[Sequence[Mapping[str, float]]] = None,
                     *,
                     ranges: Optional[Mapping[str, Tuple[float, float]]] = None,
                     n: Optional[int] = None,
                     seed: int = 0,
                     factory: Callable[[Mapping[str, float]], Vehicle] = rocket_from_params,
                     M: Optional[Sequence[float]] = None,
                     alpha: Optional[Sequence[float]] = None,
                     delta: Optional[Sequence[float]] = None,
                     alt: float = 1000.0,
                     resume: bool = True,
                     exe_path: Optional[str] = None,
                     timeout: float = 60.0,
                     verbose: bool = True) -> Dict[str, int]:
    """Run the pipeline for every parameter set; return a summary dict.

    Provide either ``params_list`` or (``ranges`` + ``n``) to sample here.
    Each sample runs in ``<outdir>/runs/<id>/``. With ``resume=True``
    (default) runs whose ``aero.npz`` already exists are skipped, so an
    interrupted campaign continues where it stopped. Failures (invalid
    geometry, DATCOM crash/timeout/empty output, parse error) are appended
    to ``<outdir>/failures.jsonl`` and do not stop the campaign.
    """
    if params_list is None:
        if ranges is None or n is None:
            raise ValueError("provide params_list, or ranges and n")
        params_list = sample_params(ranges, n, seed)

    out = Path(outdir)
    runs = out / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    failures = out / "failures.jsonl"

    summary = {"total": len(params_list), "ok": 0, "failed": 0, "skipped": 0}
    for idx, params in enumerate(params_list):
        run_dir = runs / f"{idx:05d}"
        if resume and (run_dir / "aero.npz").is_file():
            summary["skipped"] += 1
            continue
        try:
            _run_one(run_dir, params, factory, M, alpha, delta, alt,
                     exe_path, timeout)
            summary["ok"] += 1
        except Exception as exc:                       # log & continue
            summary["failed"] += 1
            with open(failures, "a") as f:
                f.write(json.dumps({"run": run_dir.name,
                                    "params": dict(params),
                                    "error": f"{type(exc).__name__}: {exc}"})
                        + "\n")
        if verbose and (idx + 1) % 25 == 0:
            print(f"  [{idx + 1}/{len(params_list)}] "
                  f"ok={summary['ok']} failed={summary['failed']} "
                  f"skipped={summary['skipped']}")
    return summary


# ---------------------------------------------------------------------------
# Flattening
# ---------------------------------------------------------------------------
def build_dataset(outdir: str,
                  static_csv: str = "dataset_static.csv",
                  control_csv: str = "dataset_control.csv") -> List[str]:
    """Flatten all successful runs under ``outdir`` into training tables.

    Returns the list of files written (control table only when at least
    one run has a deflection schedule). Regenerable at any time from the
    raw per-run archives — changing this schema never loses data (D8).
    """
    out = Path(outdir)
    run_dirs = sorted(d for d in (out / "runs").glob("*")
                      if (d / "aero.npz").is_file())
    if not run_dirs:
        raise FileNotFoundError(f"no completed runs under {out / 'runs'}")

    # Union of parameter names across runs (sorted for stable columns)
    metas = {d.name: json.loads((d / "params.json").read_text())
             for d in run_dirs}
    param_names = sorted({k for m in metas.values() for k in m["params"]})

    static_path = out / static_csv
    control_path = out / control_csv
    wrote: List[str] = []

    with open(static_path, "w", newline="") as f_st:
        st = csv.writer(f_st)
        st.writerow(["run", "alt", *param_names, "mach", "alpha", "transonic",
                     *_STAT_COEFFS,
                     *[c + "_filled" for c in _STAT_COEFFS]])
        ctrl_rows: List[list] = []
        for d in run_dirs:
            meta = metas[d.name]
            pvals = [meta["params"].get(k, "") for k in param_names]
            with np.load(d / "aero.npz") as z:
                mach, alpha = z["mach"], z["alpha"]
                coeff = {c: z[c] for c in _STAT_COEFFS}
                filled = {c: z["fill_" + c] for c in _STAT_COEFFS}
                for j, m in enumerate(mach):
                    # STMACH/TSMACH-extended methods, no transonic physics
                    tr = int(0.6 < m < 1.4)
                    for i, a in enumerate(alpha):
                        st.writerow([d.name, meta["alt"], *pvals,
                                     float(m), float(a), tr,
                                     *[float(coeff[c][i, j]) for c in _STAT_COEFFS],
                                     *[int(filled[c][i, j]) for c in _STAT_COEFFS]])
                if "delta" in z and z["delta"].size:
                    delta = z["delta"]
                    for j, m in enumerate(mach):
                        for k, dl in enumerate(delta):
                            ctrl_rows.append(
                                [d.name, meta["alt"], *pvals,
                                 float(m), float(dl),
                                 *[float(z[c][k, j]) for c in _CTRL_2D]])
    wrote.append(str(static_path))

    if ctrl_rows:
        with open(control_path, "w", newline="") as f_ct:
            ct = csv.writer(f_ct)
            ct.writerow(["run", "alt", *param_names, "mach", "delta",
                         *_CTRL_2D])
            ct.writerows(ctrl_rows)
        wrote.append(str(control_path))
    return wrote
