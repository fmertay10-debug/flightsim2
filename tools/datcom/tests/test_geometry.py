"""Native 3-D mesh generation tests (pydatcom.geometry)."""
import numpy as np
import pytest

from pydatcom import (Vehicle, define_example_canard_missile,
                      define_example_rocket, vehicle_mesh, vehicle_to_obj)


def _check_parts(parts):
    for name, verts, faces in parts:
        assert np.isfinite(verts).all(), name
        assert faces.min() >= 0 and faces.max() < len(verts), name
        assert len(faces) > 0, name


def test_rocket_mesh_parts():
    parts = vehicle_mesh(define_example_rocket())
    names = [p[0] for p in parts]
    assert names == ["body", "fin_1", "fin_2", "fin_3", "fin_4"]
    _check_parts(parts)
    # body spans the full vehicle length
    body = parts[0][1]
    assert body[:, 0].min() == pytest.approx(0.0)
    assert body[:, 0].max() == pytest.approx(27.0)
    # ogive nose: radius at mid-nose exceeds the conical (linear) value
    xs = body[:, 0]
    rr = np.hypot(body[:, 1], body[:, 2])
    mid = np.abs(xs - 2.0) < 0.2
    assert rr[mid].max() > (16 / 12 / 2) * (2.0 / 4.0) * 1.1


def test_three_fin_mesh():
    parts = vehicle_mesh(Vehicle(fin_num=3))
    assert [p[0] for p in parts] == ["body", "fin_1", "fin_2", "fin_3"]
    _check_parts(parts)


def test_boattail_mesh_base_radius():
    parts = vehicle_mesh(Vehicle(L_af=21.0, L_bt=2.0, D_bt=0.8))
    body = parts[0][1]
    base = body[np.abs(body[:, 0] - 27.0) < 1e-9]
    r_base = np.hypot(base[:, 1], base[:, 2]).max()
    assert r_base == pytest.approx(0.4, abs=1e-6)


def test_canard_mesh_parts():
    parts = vehicle_mesh(define_example_canard_missile())
    names = [p[0] for p in parts]
    assert names[:5] == ["body", "fin_1", "fin_2", "fin_3", "fin_4"]
    assert names[5:] == ["canard_1", "canard_2", "canard_3", "canard_4"]
    _check_parts(parts)
    # canard sits forward of the tail fins
    canard_x = parts[5][1][:, 0]
    fin_x = parts[1][1][:, 0]
    assert canard_x.max() < fin_x.min()


def test_html_viewer_export(tmp_path):
    import json

    from pydatcom import vehicle_to_html

    path = tmp_path / "v.html"
    vehicle_to_html(define_example_canard_missile(), str(path), title="t")
    text = path.read_text(encoding="utf-8")
    assert "<canvas" in text and "<script>" in text
    mesh = json.loads(text.split("const MESH = ", 1)[1].split(";\n", 1)[0])
    assert len(mesh["parts"]) == 9              # body + 4 fins + 4 canards
    for part in mesh["parts"]:
        n_v = len(part["v"])
        assert n_v > 0 and part["f"]
        assert all(0 <= i < n_v for f in part["f"] for i in f), part["name"]


def test_obj_export(tmp_path):
    path = tmp_path / "rocket.obj"
    vehicle_to_obj(define_example_canard_missile(), str(path))
    text = path.read_text()
    assert text.count("o ") == 9                    # body + 4 fins + 4 canards
    n_v = sum(1 for ln in text.splitlines() if ln.startswith("v "))
    n_f = sum(1 for ln in text.splitlines() if ln.startswith("f "))
    assert n_v > 1000 and n_f > 2000
    # all face indices resolve to vertices (1-based)
    max_ref = max(int(tok) for ln in text.splitlines() if ln.startswith("f ")
                  for tok in ln.split()[1:])
    assert max_ref == n_v
