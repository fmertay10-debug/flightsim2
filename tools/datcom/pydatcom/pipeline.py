"""End-to-end DATCOM aero pipeline for one vehicle.

Python port of MATLAB ``datcom_pipeline.m``. Writes the input deck, runs
DATCOM and parses the output, keeping every working file (``for005.dat`` /
``for006.dat``) together in the vehicle's output directory.
"""
from __future__ import annotations

import os
from typing import Optional, Sequence

from .importer import AeroData, import_datcom
from .input_file import write_input_file
from .runner import run_datcom
from .vehicle import Vehicle


def run_pipeline(vehicle: Vehicle,
                 M: Optional[Sequence[float]] = None,
                 alpha: Optional[Sequence[float]] = None,
                 delta: Optional[Sequence[float]] = None,
                 outdir: str = ".",
                 exe_path: Optional[str] = None,
                 alt: float = 1000.0,
                 timeout: float = 60.0) -> AeroData:
    """Run input -> DATCOM -> import inside ``outdir`` and return the aero data.

    ``delta`` (symmetric fin-deflection schedule, degrees) enables the
    fin-control case. ``alt`` sets the flight altitude in the deck. The
    DATCOM working files are written to ``outdir``. ``timeout`` bounds the
    DATCOM run in seconds.
    """
    os.makedirs(outdir, exist_ok=True)
    write_input_file(vehicle, M, alpha, delta,
                     out_path=os.path.join(outdir, "for005.dat"), alt=alt)
    run_datcom(outdir, exe_path=exe_path, timeout=timeout)
    return import_datcom(os.path.join(outdir, "for006.dat"))
