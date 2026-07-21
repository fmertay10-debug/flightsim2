"""pydatcom - a Python port of the MATLAB USAF Digital DATCOM toolchain.

Generates a DATCOM input deck from a vehicle geometry, runs DATCOM, and
parses the output into aerodynamic coefficient tables (including the fin
control increments). Mirrors the MATLAB ``datcom_input_file`` /
``datcom_run`` / ``datcom_import`` / ``datcom_pipeline`` functions.
"""
from .vehicle import (Vehicle, define_example_rocket,
                      define_example_canard_missile)
from .input_file import write_input_file, build_deck, DEFAULT_MACH, DEFAULT_ALPHA
from .runner import run_datcom, datcom_exe_path
from .importer import import_datcom, AeroData
from .pipeline import run_pipeline
from .dataset import (sample_params, rocket_from_params, generate_dataset,
                      build_dataset)
from .geometry import vehicle_mesh, vehicle_to_obj, plot_vehicle
from .viewer_html import vehicle_to_html

__all__ = [
    "Vehicle", "define_example_rocket", "define_example_canard_missile",
    "write_input_file", "build_deck", "DEFAULT_MACH", "DEFAULT_ALPHA",
    "run_datcom", "datcom_exe_path",
    "import_datcom", "AeroData",
    "run_pipeline",
    "sample_params", "rocket_from_params", "generate_dataset",
    "build_dataset",
    "vehicle_mesh", "vehicle_to_obj", "plot_vehicle", "vehicle_to_html",
]
