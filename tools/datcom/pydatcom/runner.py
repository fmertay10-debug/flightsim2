"""Run the USAF Digital DATCOM executable.

Python port of MATLAB ``datcom_run.m``. DATCOM reads ``for005.dat`` and
writes ``for006.dat`` in its working directory, so the executable is run
with ``cwd=workdir`` and the throwaway ``for0xx.dat`` files are cleaned up
there afterwards.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Optional

# bin/DATCOM.exe sits next to the package root (pyParserForDatcom/bin/)
_DEFAULT_EXE = Path(__file__).resolve().parent.parent / "bin" / "DATCOM.exe"

_JUNK = ["for008.dat", "for009.dat", "for010.dat", "for011.dat",
         "for012.dat", "for013.dat", "for014.dat"]


def datcom_exe_path() -> Path:
    """Full path to the bundled DATCOM executable."""
    return _DEFAULT_EXE


def run_datcom(workdir: str = ".", exe_path: Optional[str] = None,
               timeout: float = 60.0) -> int:
    """Run DATCOM in ``workdir`` (must contain ``for005.dat``).

    Returns the process exit code. Raises ``FileNotFoundError`` if the
    executable or input deck is missing, ``TimeoutError`` if DATCOM hangs
    beyond ``timeout`` seconds, and ``RuntimeError`` if DATCOM reports a
    runtime error or fails to produce a usable ``for006.dat``.
    """
    exe = Path(exe_path) if exe_path else _DEFAULT_EXE
    if not exe.is_file():
        raise FileNotFoundError(f"DATCOM executable not found: {exe}")

    workdir = os.path.abspath(workdir)
    if not os.path.isfile(os.path.join(workdir, "for005.dat")):
        raise FileNotFoundError(
            f"for005.dat not found in {workdir}; write the input deck first.")

    # Remove any stale output so a failed run can't be mistaken for success
    out_file = os.path.join(workdir, "for006.dat")
    try:
        os.remove(out_file)
    except OSError:
        pass

    try:
        proc = subprocess.run([str(exe)], cwd=workdir,
                              capture_output=True, text=True,
                              timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"DATCOM.exe did not finish within {timeout:.0f}s in {workdir} "
            "(malformed deck can make it wait on console input).") from exc

    # Delete throwaway output files (ignore if absent)
    for name in _JUNK:
        try:
            os.remove(os.path.join(workdir, name))
        except OSError:
            pass

    if proc.returncode != 0:
        raise RuntimeError(
            "DATCOM.exe returned a runtime error "
            f"(exit {proc.returncode}).\n{proc.stdout}\n{proc.stderr}")

    # DATCOM can exit 0 without producing usable output (e.g. input errors
    # flagged by CONERR); an existing, non-trivial for006.dat is the real
    # success signal.
    if not os.path.isfile(out_file) or os.path.getsize(out_file) == 0:
        raise RuntimeError(
            f"DATCOM.exe exited 0 but produced no for006.dat in {workdir}."
            f"\n{proc.stdout}\n{proc.stderr}")
    return proc.returncode
