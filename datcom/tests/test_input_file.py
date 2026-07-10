"""Deck-writer regression tests.

Golden test: the writer must reproduce the committed ``for005.dat``
byte-for-byte (modulo newline convention). Unit tests: every validation
rule fires with the documented message class.
"""
import warnings

import pytest

from pydatcom import (DEFAULT_MACH, Vehicle, define_example_rocket,
                      write_input_file)
from tests.conftest import GOLDEN_DELTA


# ---------------------------------------------------------------------------
# Golden deck
# ---------------------------------------------------------------------------
def test_golden_deck(tmp_path, golden_dir):
    """Writer output is identical to the committed golden for005.dat."""
    out = tmp_path / "for005.dat"
    write_input_file(define_example_rocket(), None, None,
                     GOLDEN_DELTA, out_path=str(out))
    # read_text() normalizes newlines on both sides
    assert out.read_text() == (golden_dir / "for005.dat").read_text()


def test_baseline_deck_has_no_flap_cards(tmp_path):
    """Without delta, no SYMFLP/ASYFLP cards are emitted."""
    out = tmp_path / "for005.dat"
    write_input_file(define_example_rocket(), None, None,
                     None, out_path=str(out))
    text = out.read_text()
    assert "$SYMFLP" not in text
    assert "$ASYFLP" not in text


def test_deck_is_deterministic(tmp_path):
    """Two runs with identical inputs produce identical decks."""
    a = tmp_path / "a.dat"
    b = tmp_path / "b.dat"
    for p in (a, b):
        write_input_file(define_example_rocket(), None,
                         None, GOLDEN_DELTA, out_path=str(p))
    assert a.read_text() == b.read_text()


# ---------------------------------------------------------------------------
# Validation rules
# ---------------------------------------------------------------------------
def _write(tmp_path, **kwargs):
    """Helper: write a deck with small default schedules."""
    kwargs.setdefault("vehicle", define_example_rocket())
    kwargs.setdefault("M", [2.0])
    kwargs.setdefault("alpha", [0.0, 2.0, 4.0])
    kwargs.setdefault("delta", None)
    v = kwargs.pop("vehicle")
    return write_input_file(v, kwargs.pop("M"), kwargs.pop("alpha"),
                            kwargs.pop("delta"),
                            out_path=str(tmp_path / "for005.dat"), **kwargs)


def test_too_many_mach_raises(tmp_path):
    with pytest.raises(ValueError, match="17 Mach"):
        _write(tmp_path, M=[0.1 * i for i in range(1, 19)])


def test_negative_mach_raises(tmp_path):
    with pytest.raises(ValueError, match="negative"):
        _write(tmp_path, M=[-0.5, 2.0])


def test_negative_alpha_raises(tmp_path):
    with pytest.raises(ValueError, match="flip alpha"):
        _write(tmp_path, alpha=[-2.0, 0.0, 2.0])


def test_zcg_beyond_radius_raises(tmp_path):
    v = define_example_rocket()
    v.zcg = v.D  # > D/2
    with pytest.raises(ValueError, match="offset"):
        _write(tmp_path, vehicle=v)


def test_xcg_beyond_length_raises(tmp_path):
    v = define_example_rocket()
    v.xcg = v.L + 1.0
    with pytest.raises(ValueError, match="longer than the length"):
        _write(tmp_path, vehicle=v)


def test_too_many_deltas_raises(tmp_path):
    with pytest.raises(ValueError, match="9 control deflection"):
        _write(tmp_path, delta=list(range(-25, 26, 5)))  # 11 entries


def test_flap_chord_fraction_raises(tmp_path):
    v = define_example_rocket()
    v.fin_flap_c = 1.0
    with pytest.raises(ValueError, match="fin_flap_c"):
        _write(tmp_path, vehicle=v, delta=[-10, 0, 10])


def test_transonic_mach_warns(tmp_path):
    with pytest.warns(UserWarning, match="transonic"):
        _write(tmp_path, M=[0.8])


def test_default_mach_covers_transonic_without_warning(tmp_path):
    """The curated default schedule includes 0.6-1.4 points, no warning."""
    assert any(0.6 < m < 1.4 for m in DEFAULT_MACH)
    assert len(DEFAULT_MACH) <= 17
    with warnings.catch_warnings():
        warnings.simplefilter("error")          # any warning -> failure
        write_input_file(define_example_rocket(), None, [0.0, 2.0, 4.0],
                         None, out_path=str(tmp_path / "for005.dat"))


def test_mach_zero_becomes_001(tmp_path):
    """M=0 entries are silently promoted to 0.01."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        path = _write(tmp_path, M=[0.0, 2.0])
    text = (tmp_path / "for005.dat").read_text()
    assert "MACH(1)=0.01," in text
    assert path.endswith("for005.dat")


def test_vehicle_defaults_are_consistent():
    """The example rocket satisfies its own validation rules."""
    v = define_example_rocket()
    assert v.xcg <= v.L
    assert abs(v.zcg) <= v.D / 2
    assert v.fin_num in (3, 4)
