# pyParserForDatcom

A Python toolchain for the USAF Digital DATCOM (1976): generates an input
deck from a vehicle geometry, runs DATCOM, parses the output into
aerodynamic coefficient tables (including fin-control increments), and
batch-generates datasets for surrogate ML models.

The deck writer is built on a namelist-emitter foundation (validated by
running decks through DATCOM.exe and comparing every coefficient table
exactly), ogive noses use DATCOM's ogive method (`BNOSE=2.0`), and a canard
configuration is supported. A pytest regression suite locks behavior via
golden files.

This is a trimmed vendored copy inside flightsim2: the `ml/` tree, the docs
(including the Digital DATCOM user's manual PDF), and caches are excluded.
flightsim2 uses it to parse DATCOM output into the aero tables that
`tools/make_fleet.py` and `tools/make_missiles.py` build vehicles from.

## Layout

```
tools/datcom/
├── pyproject.toml        pip install -e .[test,plot]
├── bin/DATCOM.exe        USAF Digital DATCOM 1976 executable (Windows; wine on Linux)
├── examples/             Parsed aero databases the flightsim2 fleet is built from
├── vehicles/             Golden baseline files (regression tests)
├── tools/                playground.py / view_vehicle.py
├── tests/                pytest suite (fast; -m slow runs DATCOM.exe)
└── pydatcom/
    ├── vehicle.py        Vehicle dataclass (+ example rocket / canard missile)
    ├── deck.py           DATCOM namelist/card emission rules
    ├── input_file.py     build_deck() / write_input_file() -> for005.dat
    ├── runner.py         run_datcom() (timeout, output verification)
    ├── importer.py       import_datcom() -> AeroData (+ fill masks)
    ├── pipeline.py       run_pipeline() = write -> run -> import
    ├── dataset.py        LHS sampling + batch campaigns + flat ML tables
    ├── geometry.py       3-D mesh: vehicle_to_obj() / plot_vehicle()
    ├── viewer_html.py    self-contained HTML vehicle viewer
    └── plotting.py       plot_aero() (matplotlib, optional)
```

## Install

```bash
pip install -e .            # numpy only
pip install -e .[test,plot] # + pytest, matplotlib
```

`bin/DATCOM.exe` is bundled and located automatically (editable install
recommended; the exe is resolved relative to the source tree).

## Usage

```python
from pydatcom import define_example_rocket, run_pipeline

vehicle = define_example_rocket()

# Baseline aerodynamics over the default Mach/alpha envelope
aero = run_pipeline(vehicle, outdir="vehicles/example_rocket")

# Fin-control case: symmetric pitch + differential roll increments
aero = run_pipeline(vehicle, delta=[-20, -10, 0, 10, 20],
                    outdir="vehicles/example_rocket", alt=1000.0)
```

`AeroData` fields are NumPy arrays `[nalpha x nmach]` (alpha mirrored to
±180°): static `cn, cd, cm, ca, cl, xcp, cla, cma, cyb, cnb, clb`, dynamic
`cmq, clq, clp, cyp, cnp, cnr, clr, clad, cmad`, and fin-control tables
`dcl_sym, dcm_sym, dcdi_sym, clroll, cn_asy, ...`. `aero.fill[name]` is a
boolean mask flagging cells that were repaired (interpolated/filled) rather
than read from DATCOM output. `save_npz()` archives everything.

### Canard configuration

```python
from pydatcom import Vehicle

v = Vehicle(canard_x=5.0, canard_root=1.2, canard_tip=0.6,
            canard_height=0.8, canard_sweep=30.0)
```

The canard is a fixed forward surface (DATCOM "wing"); the tail fins move
to the horizontal-tail slot. Pitch control (`delta`) stays on the tail.
Differential-roll cases are skipped for canard vehicles (DATCOM would
attach the ailerons to the canard) — `clroll`/`cn_asy` are absent.

### Dataset generation for surrogate ML

```python
from pydatcom import generate_dataset, build_dataset

ranges = {"D": (1.0, 1.6), "L_nc": (3.0, 5.0), "L_af": (18.0, 26.0),
          "fin_root": (2.5, 4.0), "fin_tip": (1.2, 2.2),
          "fin_sweep": (25.0, 50.0), "fin_height": (1.2, 2.2),
          "xcg_frac": (0.45, 0.65)}

generate_dataset("campaigns/c1", ranges=ranges, n=300, seed=0,
                 delta=[-20, -10, 0, 10, 20])
build_dataset("campaigns/c1")   # -> dataset_static.csv, dataset_control.csv
```

Latin-Hypercube samples the geometry, runs one DATCOM case per sample in
`runs/<id>/` (resumable — rerun to continue an interrupted campaign),
logs failures to `failures.jsonl` without stopping, and flattens results
into training tables: one row per (run, mach, alpha) with geometry
parameters, all coefficients, and `<name>_filled` quality flags.

## Tests

```bash
python -m pytest              # fast suite (goldens, validation, sampling)
python -m pytest -m slow      # + end-to-end runs through DATCOM.exe
```

Golden files in `vehicles/example_rocket/` freeze validated behavior.
Deliberate physics changes regenerate them with a reviewed diff
(process in [docs/ROADMAP.md](docs/ROADMAP.md)).

## Notes

- DATCOM accepts at most 17 Mach numbers and 9 control deflections per run.
- DATCOM has no transonic methods. The deck sets `STMACH=0.99`/`TSMACH=1.01`
  so the default Mach schedule covers 0.6–1.4 with extended sub/supersonic
  methods (0.8, 0.95, 1.05, 1.2) — usable data, but no drag-rise physics;
  those rows carry a `transonic=1` flag in the ML tables, and a warning is
  emitted when a user-supplied Mach list enters the band.
- DATCOM 1976 has no all-moving-fin method; articulated fins are
  approximated as a large-chord plain trailing-edge flap (~2x underestimate
  of control power vs an all-moving fin; trends correct — ROADMAP D1).
- Altitude enters DATCOM only through Reynolds number; pass `alt=` per run
  and sweep it in a campaign if drag fidelity across altitude matters.
