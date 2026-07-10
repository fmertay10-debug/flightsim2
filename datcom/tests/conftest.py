"""Shared fixtures for the pydatcom regression suite.

The committed files in ``vehicles/example_rocket/`` are the golden baseline:
they were produced by the validated toolchain (see docs/ROADMAP.md, I1) and
freeze its behavior. Deliberate behavior changes (e.g. the BNOSE fix, D6)
must regenerate the goldens with a reviewed diff.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent

# Make the package importable when running from a source checkout
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

GOLDEN_DIR = ROOT / "vehicles" / "example_rocket"

# The exact arguments the goldens were generated with (example.py, control case)
GOLDEN_DELTA = [-20, -10, 0, 10, 20]


@pytest.fixture(scope="session")
def golden_dir() -> Path:
    if not GOLDEN_DIR.is_dir():
        pytest.skip(f"golden directory missing: {GOLDEN_DIR}")
    return GOLDEN_DIR


def golden_value(aero, npz_key: str):
    """Map an npz key to the corresponding AeroData value.

    ``fill_<name>`` keys address entries of the ``aero.fill`` mask dict;
    everything else is a plain attribute.
    """
    if npz_key.startswith("fill_"):
        return aero.fill[npz_key[len("fill_"):]]
    return getattr(aero, npz_key)
