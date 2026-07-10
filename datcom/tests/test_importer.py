"""Importer regression tests.

Golden test: parsing the committed ``for006.dat`` must reproduce every
array in the committed ``aero_ctrl.npz`` exactly. Unit tests: the small
MATLAB-semantics helpers.
"""
import math

import numpy as np
import pytest

from pydatcom import import_datcom
from pydatcom.importer import _after_float, _scan_floats, _to_double
from tests.conftest import golden_value


# ---------------------------------------------------------------------------
# Golden parse
# ---------------------------------------------------------------------------
def test_golden_parse(golden_dir):
    """import_datcom(for006.dat) reproduces the committed aero_ctrl.npz."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    with np.load(golden_dir / "aero_ctrl.npz") as npz:
        assert npz.files, "golden npz is empty"
        for key in npz.files:
            got = golden_value(aero, key)
            assert got is not None, f"field {key} missing from parse"
            np.testing.assert_array_equal(
                np.asarray(got), npz[key],
                err_msg=f"field '{key}' differs from golden")


def test_golden_parse_shapes(golden_dir):
    """Shape contract: static [nalpha x nmach], control tables sized by ndelta."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    na, nm, nd = aero.nalpha, aero.nmach, aero.ndelta
    assert aero.alpha.shape == (2 * na - 1,)  # mirrored about zero
    assert aero.cn.shape == (2 * na - 1, nm)
    assert aero.cmq.shape == (2 * na - 1, nm)
    if nd:
        assert aero.dcl_sym.shape == (nd, nm)
        assert aero.dcdi_sym.shape == (2 * na - 1, nd, nm)
        assert aero.cn_asy.shape == (2 * na - 1, len(aero.deltal), nm)


def test_golden_parse_invariants(golden_dir):
    """Physical invariants of the post-processing."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    # alpha mirrored and monotonic, endpoints +-180
    assert aero.alpha[0] == -180.0 and aero.alpha[-1] == 180.0
    assert np.all(np.diff(aero.alpha) > 0)
    # mach[0] repaired to 0.0
    assert aero.mach[0] == 0.0
    # antisymmetric coefficients are odd in alpha at the mirror point
    mid = len(aero.alpha) // 2
    np.testing.assert_array_equal(aero.cn[mid - 1, :], -aero.cn[mid + 1, :])
    np.testing.assert_array_equal(aero.cm[mid - 1, :], -aero.cm[mid + 1, :])
    # symmetric coefficient is even
    np.testing.assert_array_equal(aero.cd[mid - 1, :], aero.cd[mid + 1, :])
    # no NaNs survive post-processing
    for name in ("cn", "cd", "cm", "ca", "cl", "xcp", "cla", "cma",
                 "cmq", "clp"):
        assert not np.isnan(getattr(aero, name)).any(), name


# ---------------------------------------------------------------------------
# Fill mask (ROADMAP D9)
# ---------------------------------------------------------------------------
def test_fill_mask_present_and_aligned(golden_dir):
    """Every coefficient table has a boolean mask of identical shape."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    assert aero.fill, "fill mask missing"
    from pydatcom.importer import _CTRL_COEFFS, _STAT_COEFFS
    for name in _STAT_COEFFS + _CTRL_COEFFS:
        A = getattr(aero, name)
        if A is None:
            assert name not in aero.fill
            continue
        mask = aero.fill[name]
        assert mask.dtype == bool, name
        assert mask.shape == A.shape, name


def test_fill_mask_content(golden_dir):
    """Semantic checks on what is (and is not) marked as repaired."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    mid = len(aero.alpha) // 2  # alpha = 0 (mirror pivot = old first row)
    # the xcp alpha=0 row was repaired (copied from the second alpha row)
    assert aero.fill["xcp"][mid, :].all()
    # cma has interpolated cells (supersonic columns, between alpha batches)
    assert aero.fill["cma"].any()
    # cn near alpha=0 is directly measured everywhere
    assert not aero.fill["cn"][mid, :].any()
    # nothing is 100% repaired
    for name, mask in aero.fill.items():
        assert not mask.all(), f"{name} entirely repaired?"
    # masks are mirror-symmetric in alpha like the data
    np.testing.assert_array_equal(aero.fill["cd"][mid - 1, :],
                                  aero.fill["cd"][mid + 1, :])


def test_fill_mask_roundtrips_npz(golden_dir, tmp_path):
    """save_npz flattens masks to fill_<name> keys."""
    aero = import_datcom(str(golden_dir / "for006.dat"))
    out = tmp_path / "aero.npz"
    aero.save_npz(str(out))
    with np.load(out) as npz:
        for name, mask in aero.fill.items():
            np.testing.assert_array_equal(npz["fill_" + name], mask)


def test_printed_zero_dropout_repair():
    """D11: literal-0.0 method dropouts are interpolated and masked."""
    from pydatcom.importer import AeroData, _STAT_COEFFS, _postprocess

    aero = AeroData()
    aero.mach = np.array([2.0, 3.0])
    aero.alpha = np.array([0.0, 10.0, 20.0, 25.0, 30.0])
    for name in _STAT_COEFFS:
        setattr(aero, name, np.zeros((5, 2)))
    # column 0: dropout at 25 deg between two large same-sign values,
    # plus a trailing-edge dropout at 30 deg in column 1
    aero.cm[:, 0] = [0.0, -1.0, -2.0, 0.0, -3.0]
    aero.cm[:, 1] = [0.0, -1.0, -2.0, -2.5, 0.0]
    # a genuine zero crossing must NOT be treated as a dropout
    aero.cn[:, 0] = [0.0, -1.0, 0.0, 1.0, 2.0]
    _postprocess(aero)

    mid = len(aero.alpha) // 2                     # alpha = 0 after mirror
    i25, i30 = mid + 3, mid + 4
    assert aero.cm[i25, 0] == pytest.approx(-2.5)  # interpolated
    assert aero.fill["cm"][i25, 0]                 # and masked
    assert aero.fill["cm"][i30, 1]                 # trailing dropout masked
    i20 = mid + 2
    assert aero.cn[i20, 0] == 0.0                  # crossing left intact
    assert not aero.fill["cn"][i20, 0]


# ---------------------------------------------------------------------------
# Helper semantics (MATLAB sscanf / str2double mirrors)
# ---------------------------------------------------------------------------
def test_scan_floats_basic():
    assert _scan_floats("0.01,0.10,0.20,") == [0.01, 0.10, 0.20]


def test_scan_floats_dollar_terminator():
    # '$' abutting the last value, as in namelist echoes: "4.60,5.00$"
    assert _scan_floats("4.60,5.00$") == [4.60, 5.00]


def test_scan_floats_leading_spaces():
    assert _scan_floats("   1.5, 2.5,3.5") == [1.5, 2.5, 3.5]


def test_scan_floats_stops_at_non_numeric():
    assert _scan_floats("1.0,2.0,FOO,3.0") == [1.0, 2.0]


def test_scan_floats_empty_and_text():
    assert _scan_floats("") == []
    assert _scan_floats("NDM") == []


def test_scan_floats_scientific():
    assert _scan_floats("1.5e-3,2E+2") == [0.0015, 200.0]


def test_to_double_nan_semantics():
    assert _to_double("1.25") == 1.25
    assert math.isnan(_to_double("NDM"))
    assert math.isnan(_to_double(""))
    assert math.isnan(_to_double(None))


def test_after_float():
    assert _after_float(" $SYNTHS XCG=15.0, ZCG=0.0,", "XCG=") == 15.0
    assert _after_float(" $SYNTHS XCG=15.0, ZCG=0.0,", "ZCG=") == 0.0
    assert _after_float("no key here", "XCG=") == 0.0
