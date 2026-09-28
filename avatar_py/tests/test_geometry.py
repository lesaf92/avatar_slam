import numpy as np
import pytest

from avatar import geometry as G


def random_pose(rng):
    return np.array([*rng.uniform(-50, 50, 3), rng.uniform(-np.pi, np.pi)])


def test_wrap_angle_range():
    a = np.linspace(-20, 20, 1001)
    w = G.wrap_angle(a)
    assert np.all(w > -np.pi - 1e-12) and np.all(w <= np.pi + 1e-12)
    assert np.allclose(np.sin(w), np.sin(a)) and np.allclose(np.cos(w), np.cos(a))
    assert G.wrap_angle(-np.pi) == pytest.approx(np.pi)


def test_compose_inverse_identity(rng):
    for _ in range(20):
        a = random_pose(rng)
        assert np.allclose(G.compose(a, G.inverse(a)), 0.0, atol=1e-12)
        assert np.allclose(G.compose(G.inverse(a), a), 0.0, atol=1e-12)


def test_between_and_transform_consistent(rng):
    a, b = random_pose(rng), random_pose(rng)
    rel = G.between(a, b)
    assert np.allclose(G.compose(a, rel)[:3], b[:3])
    p = rng.normal(size=(5, 3))
    assert np.allclose(G.inverse_transform_points(a, G.transform_points(a, p)), p)
    # transform_points agrees with pose composition
    assert np.allclose(G.transform_points(a, b[:3]), G.compose(a, b)[:3])


def test_align_4dof_recovers_transform(rng):
    T = np.array([3.0, -7.0, 0.4, 0.9])
    src = rng.uniform(-20, 20, (30, 3))
    dst = G.transform_points(T, src)
    est = G.align_4dof(src, dst)
    assert np.allclose(est, T, atol=1e-9)
    est0 = G.align_4dof(src, dst, estimate_z=False)
    assert est0[2] == 0.0 and np.allclose(est0[[0, 1, 3]], T[[0, 1, 3]], atol=1e-9)


def test_ned_enu_round_trip():
    v = np.array([1.0, 2.0, 3.0])  # north, east, down
    enu = G.ned_to_enu(v)
    assert np.allclose(enu, [2.0, 1.0, -3.0])
    assert np.allclose(G.enu_to_ned(enu), v)
    assert np.allclose(G.frd_to_flu([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0])
    # heading north (0 in NED) is +pi/2 in ENU; heading east (pi/2 NED) is 0 in ENU
    assert G.yaw_ned_to_enu(0.0) == pytest.approx(np.pi / 2)
    assert G.yaw_ned_to_enu(np.pi / 2) == pytest.approx(0.0)
