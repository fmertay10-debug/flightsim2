"""End-to-end regression: deck -> DATCOM.exe -> parse -> golden npz.

Marked ``slow`` (runs the bundled executable). Deselect with -m "not slow".
"""
import numpy as np
import pytest

from pydatcom import (datcom_exe_path,
                      define_example_rocket, run_pipeline)
from tests.conftest import GOLDEN_DELTA, golden_value

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def _exe_available():
    if not datcom_exe_path().is_file():
        pytest.skip("bundled DATCOM.exe not found")


def test_pipeline_end_to_end(tmp_path, golden_dir, _exe_available):
    """Full pipeline reproduces the committed aero_ctrl.npz exactly."""
    aero = run_pipeline(define_example_rocket(), None, None,
                        GOLDEN_DELTA, outdir=str(tmp_path))
    assert (tmp_path / "for006.dat").is_file()
    with np.load(golden_dir / "aero_ctrl.npz") as npz:
        for key in npz.files:
            np.testing.assert_array_equal(
                np.asarray(golden_value(aero, key)), npz[key],
                err_msg=f"field '{key}' differs from golden")


def test_pipeline_baseline_runs(tmp_path, _exe_available):
    """Baseline (no delta) case runs and yields control-free tables."""
    aero = run_pipeline(define_example_rocket(), [2.0, 3.0], [0, 2, 4, 6],
                        None, outdir=str(tmp_path))
    assert aero.ndelta == 0
    assert aero.dcl_sym is None
    assert aero.nmach == 2
    # 4 alphas mirrored -> 7
    assert aero.cn.shape == (7, 2)
