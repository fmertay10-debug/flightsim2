"""Batch dataset generator tests (I5)."""
import csv
import json

import numpy as np
import pytest

from pydatcom import (build_dataset, datcom_exe_path, generate_dataset,
                      rocket_from_params, sample_params)

RANGES = {
    "D": (1.0, 1.6),
    "L_nc": (3.0, 5.0),
    "L_af": (18.0, 26.0),
    "fin_root": (2.5, 4.0),
    "fin_tip": (1.2, 2.2),
    "fin_sweep": (25.0, 50.0),
    "fin_height": (1.2, 2.2),
    "xcg_frac": (0.45, 0.65),
}


# ---------------------------------------------------------------------------
# Sampling / factory (fast)
# ---------------------------------------------------------------------------
def test_sample_params_bounds_and_shape():
    samples = sample_params(RANGES, 20, seed=1)
    assert len(samples) == 20
    for s in samples:
        assert set(s) == set(RANGES)
        for k, (lo, hi) in RANGES.items():
            assert lo <= s[k] <= hi, k


def test_sample_params_is_stratified():
    """LHS: exactly one sample per 1/n bin in every dimension."""
    n = 10
    samples = sample_params(RANGES, n, seed=2)
    for k, (lo, hi) in RANGES.items():
        bins = sorted(int((s[k] - lo) / (hi - lo) * n) for s in samples)
        assert bins == list(range(n)), k


def test_sample_params_deterministic():
    assert sample_params(RANGES, 5, seed=7) == sample_params(RANGES, 5, seed=7)
    assert sample_params(RANGES, 5, seed=7) != sample_params(RANGES, 5, seed=8)


def test_rocket_from_params_consistency():
    p = dict(D=1.2, L_nc=4.0, L_af=20.0, xcg_frac=0.5,
             fin_root=3.0, fin_tip=1.5, fin_sweep=30.0, fin_height=1.5)
    v = rocket_from_params(p)
    assert v.L == pytest.approx(24.0)          # derived, not sampled
    assert v.xcg == pytest.approx(12.0)        # xcg_frac * L
    assert v.D == 1.2 and v.fin_sweep == 30.0


def test_rocket_from_params_rejects_unknown_field():
    with pytest.raises(KeyError, match="wingspan"):
        rocket_from_params({"wingspan": 3.0})


# ---------------------------------------------------------------------------
# Mini campaign through real DATCOM (slow)
# ---------------------------------------------------------------------------
M_SMALL = [2.0, 3.0]
ALPHA_SMALL = [0.0, 2.0, 4.0]


@pytest.mark.slow
def test_generate_and_build_dataset(tmp_path):
    if not datcom_exe_path().is_file():
        pytest.skip("bundled DATCOM.exe not found")
    params = sample_params(RANGES, 2, seed=3)
    params.append({**params[0], "xcg_frac": 1.5})   # invalid: xcg beyond L

    summary = generate_dataset(str(tmp_path), params, M=M_SMALL,
                               alpha=ALPHA_SMALL, delta=[-10, 0, 10],
                               verbose=False)
    assert summary == {"total": 3, "ok": 2, "failed": 1, "skipped": 0}

    # failure logged with its parameters, campaign not aborted
    failures = [json.loads(line) for line in
                (tmp_path / "failures.jsonl").read_text().splitlines()]
    assert len(failures) == 1
    assert failures[0]["run"] == "00002"
    assert "ValueError" in failures[0]["error"]
    assert not (tmp_path / "runs" / "00002" / "aero.npz").exists()

    # resume skips completed runs
    summary2 = generate_dataset(str(tmp_path), params, M=M_SMALL,
                                alpha=ALPHA_SMALL, delta=[-10, 0, 10],
                                verbose=False)
    assert summary2["skipped"] == 2 and summary2["failed"] == 1

    # raw archives contain data + fill masks
    with np.load(tmp_path / "runs" / "00000" / "aero.npz") as z:
        assert z["cn"].shape == (2 * len(ALPHA_SMALL) - 1, len(M_SMALL))
        assert "fill_cn" in z and "dcdi_sym" in z

    # flatten: static + control tables
    files = build_dataset(str(tmp_path))
    assert len(files) == 2
    with open(files[0], newline="") as f:
        rows = list(csv.DictReader(f))
    n_pts = (2 * len(ALPHA_SMALL) - 1) * len(M_SMALL)
    assert len(rows) == 2 * n_pts                    # 2 ok runs
    r0 = rows[0]
    assert set(RANGES) <= set(r0)                    # geometry columns present
    assert "cn" in r0 and "cn_filled" in r0
    # spot-check one value against the raw npz
    with np.load(tmp_path / "runs" / "00000" / "aero.npz") as z:
        want = float(z["cn"][0, 0])
    got = [float(r["cn"]) for r in rows
           if r["run"] == "00000"
           and float(r["mach"]) == 2.0 and float(r["alpha"]) == -4.0]
    assert got == [pytest.approx(want)]

    with open(files[1], newline="") as f:
        crows = list(csv.DictReader(f))
    assert len(crows) == 2 * len(M_SMALL) * 3        # 2 runs x mach x delta
    assert "dcl_sym" in crows[0] and "clroll" in crows[0]
