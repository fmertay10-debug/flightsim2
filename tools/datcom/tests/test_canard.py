"""Canard + tail configuration tests (I6)."""
import warnings

import numpy as np
import pytest

from pydatcom import (build_deck, datcom_exe_path, define_example_canard_missile,
                      define_example_rocket, run_pipeline)

M_SMALL = [2.0, 3.0]
ALPHA_SMALL = [0.0, 2.0, 4.0]


# ---------------------------------------------------------------------------
# Deck construction (fast)
# ---------------------------------------------------------------------------
def test_canard_deck_layout():
    text = build_deck(define_example_canard_missile(), M_SMALL, ALPHA_SMALL)
    # canard is the forward "wing", tail fins become the horizontal tail
    assert "NACA-W-S-3-10.0-2.5-80.0" in text     # canard airfoil (default)
    assert "NACA-H-S-3-10.0-2.5-80.0" in text     # tail airfoil
    assert "$WGPLNF" in text and "$HTPLNF" in text
    assert "XH=" in text and "ALIH=" in text
    # canard apex forward of tail apex
    xw = float(text.split("XW=")[1].split(",")[0])
    xh = float(text.split("XH=")[1].split(",")[0])
    assert xw < xh
    # vertical panels still present (tail fin set)
    assert "$VTPLNF" in text and "$VFPLNF" in text


def test_no_canard_deck_unchanged():
    text = build_deck(define_example_rocket(), M_SMALL, ALPHA_SMALL)
    assert "$HTPLNF" not in text and "XH=" not in text


def test_canard_pitch_control_targets_tail_and_skips_roll():
    with pytest.warns(UserWarning, match="ASYFLP"):
        text = build_deck(define_example_canard_missile(), M_SMALL,
                          ALPHA_SMALL, delta=[-10, 0, 10])
    assert "$SYMFLP" in text          # pitch cases on the aft surface (tail)
    assert "$ASYFLP" not in text      # roll cases skipped (would hit canard)


def test_partial_canard_fields_raise(tmp_path):
    v = define_example_rocket()
    v.canard_x = 5.0                  # missing root/tip/height
    with pytest.raises(ValueError, match="together"):
        build_deck(v, M_SMALL, ALPHA_SMALL)


def test_canard_on_nose_raises():
    v = define_example_canard_missile()
    v.canard_x = 1.0                  # forward of L_nc=4.0
    with pytest.raises(ValueError, match="cylindrical"):
        build_deck(v, M_SMALL, ALPHA_SMALL)


def test_canard_overlapping_tail_raises():
    v = define_example_canard_missile()
    v.canard_x = v.L - v.fin_root - 0.5
    v.canard_root = 2.0
    with pytest.raises(ValueError, match="overlaps"):
        build_deck(v, M_SMALL, ALPHA_SMALL)


# ---------------------------------------------------------------------------
# Through real DATCOM (slow)
# ---------------------------------------------------------------------------
@pytest.mark.slow
def test_canard_pipeline_end_to_end(tmp_path):
    if not datcom_exe_path().is_file():
        pytest.skip("bundled DATCOM.exe not found")
    delta = [-10, 0, 10]
    plain = run_pipeline(define_example_rocket(), M_SMALL, ALPHA_SMALL,
                         delta, outdir=str(tmp_path / "plain"))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        canard = run_pipeline(define_example_canard_missile(), M_SMALL,
                              ALPHA_SMALL, delta,
                              outdir=str(tmp_path / "canard"))

    n_a = 2 * len(ALPHA_SMALL) - 1
    assert canard.cn.shape == (n_a, len(M_SMALL))
    assert np.isfinite(canard.cn).all()
    # the canard changes the aerodynamics (lift ahead of CG)
    assert not np.allclose(canard.cma, plain.cma)
    assert not np.allclose(canard.cn, plain.cn)
    # canard adds lift ahead of the CG -> pitch stability decreases
    mid = n_a // 2
    assert canard.cma[mid, 0] > plain.cma[mid, 0]
    # pitch control (tail SYMFLP) present; roll cases skipped by design
    assert canard.dcm_sym is not None and np.isfinite(canard.dcm_sym).all()
    assert canard.dcm_sym.any()
    assert canard.clroll is None and canard.cn_asy is None
    # fill masks cover the parsed tables
    assert canard.fill and canard.fill["cn"].shape == canard.cn.shape