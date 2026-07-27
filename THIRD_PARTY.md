# Third-party components

flightsim2 itself is MIT licensed (see [LICENSE](LICENSE)). The C++ simulator
core has no external dependencies — C++17 and the standard library only. The
third-party code that ships in this repository is listed below.

## Vendored JavaScript (`tools/vendor/`)

Inlined into the self-contained offline HTML viewers produced by
`tools/visualize.py`. This is the only place third-party code is vendored into
the visualization output.

| Component | Version | License |
|---|---|---|
| [three.js](https://github.com/mrdoob/three.js) (`three.min.js`) | r147 | MIT — © 2010-2022 three.js Authors |
| three.js `OrbitControls.js` | r147 | MIT — © 2010-2022 three.js Authors |
| [uPlot](https://github.com/leeoniya/uPlot) (`uPlot.iife.min.js`, `uPlot.min.css`) | 1.6.32 | MIT — © Leon Sorokin |

## USAF Digital DATCOM (`tools/datcom/bin/DATCOM.exe`)

Digital DATCOM (1976) is an aerodynamic-coefficient prediction program
developed for the United States Air Force. As a work of the U.S. Government it
is in the public domain and is freely redistributable. The executable shipped
here is the stock Windows binary (runs under wine on Linux); it is **not**
required to build or run the simulator, only to run the DATCOM solver for new
airframe geometries.

`tools/datcom/pydatcom/` is a deck generator, runner, and output parser for
that program, written for this project and covered by the repository's MIT
license.

## Reference data and published sources

The aerodynamic, propulsion, and validation data in this repository are derived
from published, publicly available sources. See the "Scope and data sources"
section of the [README](README.md) for the full list and for what the vehicle
models do and do not represent.
