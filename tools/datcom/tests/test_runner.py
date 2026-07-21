"""Runner hardening tests (I2).

Failure paths are simulated by monkeypatching ``subprocess.run`` so no real
executable is needed; the happy path is covered by the slow E2E tests.
"""
import subprocess

import pytest

from pydatcom import datcom_exe_path
from pydatcom.runner import run_datcom


@pytest.fixture
def workdir(tmp_path):
    """A workdir containing a (dummy) for005.dat."""
    (tmp_path / "for005.dat").write_text("CASEID DUMMY\n")
    return tmp_path


def _fake_exe(tmp_path):
    exe = tmp_path / "FAKE_DATCOM.exe"
    exe.write_bytes(b"MZ")  # existence is all the runner checks pre-run
    return exe


def test_missing_exe_raises(workdir):
    with pytest.raises(FileNotFoundError, match="executable"):
        run_datcom(str(workdir), exe_path=str(workdir / "nope.exe"))


def test_missing_deck_raises(tmp_path):
    with pytest.raises(FileNotFoundError, match="for005"):
        run_datcom(str(tmp_path), exe_path=str(_fake_exe(tmp_path)))


def test_timeout_raises(workdir, monkeypatch):
    def hang(*a, **k):
        raise subprocess.TimeoutExpired(cmd="datcom", timeout=k["timeout"])
    monkeypatch.setattr(subprocess, "run", hang)
    with pytest.raises(TimeoutError, match="did not finish"):
        run_datcom(str(workdir), exe_path=str(_fake_exe(workdir)),
                   timeout=0.5)


def test_nonzero_exit_raises(workdir, monkeypatch):
    monkeypatch.setattr(subprocess, "run", lambda *a, **k:
                        subprocess.CompletedProcess(a, 2, "boom", "err"))
    with pytest.raises(RuntimeError, match="exit 2"):
        run_datcom(str(workdir), exe_path=str(_fake_exe(workdir)))


def test_missing_for006_raises(workdir, monkeypatch):
    """Exit 0 but no for006.dat produced -> RuntimeError, not silence."""
    monkeypatch.setattr(subprocess, "run", lambda *a, **k:
                        subprocess.CompletedProcess(a, 0, "", ""))
    with pytest.raises(RuntimeError, match="no for006"):
        run_datcom(str(workdir), exe_path=str(_fake_exe(workdir)))


def test_stale_for006_removed(workdir, monkeypatch):
    """A leftover for006.dat from a previous run can't fake success."""
    (workdir / "for006.dat").write_text("STALE OUTPUT")
    monkeypatch.setattr(subprocess, "run", lambda *a, **k:
                        subprocess.CompletedProcess(a, 0, "", ""))
    with pytest.raises(RuntimeError, match="no for006"):
        run_datcom(str(workdir), exe_path=str(_fake_exe(workdir)))
    assert not (workdir / "for006.dat").exists()


def test_empty_for006_raises(workdir, monkeypatch):
    def fake_run(*a, **k):
        (workdir / "for006.dat").write_text("")
        return subprocess.CompletedProcess(a, 0, "", "")
    monkeypatch.setattr(subprocess, "run", fake_run)
    with pytest.raises(RuntimeError, match="no for006"):
        run_datcom(str(workdir), exe_path=str(_fake_exe(workdir)))


def test_junk_files_cleaned(workdir, monkeypatch):
    def fake_run(*a, **k):
        (workdir / "for006.dat").write_text("1  OUTPUT")
        for n in ("for008.dat", "for009.dat", "for013.dat"):
            (workdir / n).write_text("junk")
        return subprocess.CompletedProcess(a, 0, "", "")
    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_datcom(str(workdir), exe_path=str(_fake_exe(workdir))) == 0
    assert not (workdir / "for008.dat").exists()
    assert not (workdir / "for013.dat").exists()
    assert (workdir / "for006.dat").exists()


def test_bundled_exe_is_found():
    assert datcom_exe_path().name == "DATCOM.exe"
