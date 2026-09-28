import numpy as np
import pytest

from avatar.backend.graph import FactorGraph, VarType
from avatar.geometry import between, compose, inverse_transform_points, transform_points


def numeric_jacobian(g: FactorGraph, eps=1e-6):
    x0 = g._state()
    _, r0, _ = g._linearize(x0, jacobian=False)
    J = np.zeros((len(r0), len(x0)))
    for i in range(len(x0)):
        dx = np.zeros_like(x0)
        dx[i] = eps
        _, rp, _ = g._linearize(x0 + dx, jacobian=False)
        _, rm, _ = g._linearize(x0 - dx, jacobian=False)
        J[:, i] = (rp - rm) / (2 * eps)
    return J


def rand_pose(rng):
    return np.array([*rng.uniform(-5, 5, 3), rng.uniform(-3, 3)])


@pytest.mark.parametrize(
    "factor",
    [
        "pose_prior",
        "between",
        "point_obs",
        "z_prior",
        "point_prior",
        "linked_point",
        "linked_point_h",
        "coaxial",
        "range",
    ],
)
def test_analytic_jacobians_match_finite_differences(rng, factor):
    g = FactorGraph(robust_k=1e9)  # disable robust reweighting for the check
    a, b = rand_pose(rng), rand_pose(rng)
    g.add_variable("a", VarType.POSE4, a)
    g.add_variable("b", VarType.POSE4, b)
    g.add_variable("l", VarType.POINT3, rng.normal(size=3))
    g.add_variable("m", VarType.POINT3, rng.normal(size=3))
    sig4 = rng.uniform(0.1, 1.0, 4)
    sig3 = rng.uniform(0.1, 1.0, 3)
    if factor == "pose_prior":
        g.add_pose_prior("a", rand_pose(rng), sig4)
    elif factor == "between":
        g.add_between("a", "b", rand_pose(rng), sig4)
    elif factor == "point_obs":
        g.add_point_obs("a", "l", rng.normal(size=3), sig3)
    elif factor == "z_prior":
        g.add_z_prior("a", 0.3, 0.2)
    elif factor == "point_prior":
        g.add_point_prior("l", rng.normal(size=3), sig3)
    elif factor in ("linked_point", "linked_point_h"):
        g.add_linked_point(
            "l", "b", rng.normal(size=3), sig3, horizontal_only=factor.endswith("_h")
        )
    elif factor == "coaxial":
        g.add_coaxial("l", "m", 0.3)
    elif factor == "range":
        g.add_range("a", "b", 2.0, 0.5)
    _, _, J = g._linearize(g._state())
    assert np.allclose(J.toarray(), numeric_jacobian(g), atol=1e-6)


def test_small_slam_problem_converges(rng):
    """Noise-free square loop with landmarks: LM recovers the ground truth."""
    gt = [np.array([0, 0, 0, 0.0])]
    for _ in range(12):
        gt.append(compose(gt[-1], [2.0, 0.0, 0.0, np.pi / 6]))
    lms = rng.uniform(-6, 6, (8, 3))
    g = FactorGraph()
    for k, p in enumerate(gt):
        g.add_variable(("x", k), VarType.POSE4, p + rng.normal(0, 0.3, 4))
    for j, lm in enumerate(lms):
        g.add_variable(("l", j), VarType.POINT3, lm + rng.normal(0, 0.5, 3))
    g.add_pose_prior(("x", 0), gt[0], [1e-3] * 4)
    for k in range(1, len(gt)):
        g.add_between(("x", k - 1), ("x", k), between(gt[k - 1], gt[k]), [0.05] * 4)
    for k, p in enumerate(gt):
        for j, lm in enumerate(lms):
            g.add_point_obs(("x", k), ("l", j), inverse_transform_points(p, lm), [0.1] * 3)
    res = g.optimize()
    assert res.converged and res.final_cost < 1e-10
    for k, p in enumerate(gt):
        assert np.allclose(g.value(("x", k)), p, atol=1e-5)


def test_marginal_covariance_matches_linear_analytic():
    """Point prior + point prior on a POINT3: posterior variance is the harmonic sum."""
    g = FactorGraph()
    g.add_variable("l", VarType.POINT3, np.zeros(3))
    g.add_point_prior("l", [0, 0, 0], [1.0, 2.0, 3.0])
    g.add_point_prior("l", [0, 0, 0], [1.0, 2.0, 3.0])
    C = g.marginal_covariances(["l"])["l"]
    assert np.allclose(np.diag(C), np.array([1.0, 4.0, 9.0]) / 2, rtol=1e-6)


def test_robust_linked_point_downweights_outlier(rng):
    """One wrong inter-agent correspondence must not drag the frame estimate."""
    T_true = np.array([4.0, -2.0, 0.0, 0.7])
    pts_remote = rng.uniform(-10, 10, (10, 3))
    pts_mine = transform_points(T_true, pts_remote)
    g = FactorGraph(robust_k=2.0)
    g.add_variable("T", VarType.POSE4, T_true + np.array([0.5, -0.5, 0.0, 0.1]))
    for i, (pm, pr) in enumerate(zip(pts_mine, pts_remote, strict=True)):
        g.add_variable(("l", i), VarType.POINT3, pm)
        g.add_point_prior(("l", i), pm, [0.05] * 3)
        g.add_linked_point(("l", i), "T", pr, [0.05] * 3)
    g.add_variable(("l", 99), VarType.POINT3, pts_mine[0] + [20.0, 0.0, 0.0])
    g.add_point_prior(("l", 99), pts_mine[0] + [20.0, 0.0, 0.0], [0.05] * 3)
    g.add_linked_point(("l", 99), "T", pts_remote[3], [0.05] * 3)  # outlier
    g.optimize(max_iters=50)
    T = g.value("T")
    assert np.hypot(*(T[:2] - T_true[:2])) < 0.2
    assert abs(T[3] - T_true[3]) < 0.02


def test_variable_type_checks():
    g = FactorGraph()
    g.add_variable("p", VarType.POSE4, np.zeros(4))
    with pytest.raises(TypeError):
        g.add_point_prior("p", np.zeros(3), [1, 1, 1])
    with pytest.raises(KeyError):
        g.add_variable("p", VarType.POSE4, np.zeros(4))
    with pytest.raises(ValueError):
        g.add_pose_prior("p", np.zeros(4), [0, 1, 1, 1])


@pytest.mark.parametrize("which", ["landmarks", "poses", "mixed"])
def test_schur_marginals_match_dense_inverse(rng, which):
    """Schur-complement and direct marginals equal blocks of the dense H⁻¹."""
    gt = [np.array([0, 0, 0, 0.0])]
    for _ in range(9):
        gt.append(compose(gt[-1], [1.5, 0.2, 0.0, np.pi / 8]))
    lms = rng.uniform(-6, 6, (6, 3))
    g = FactorGraph()
    for k, p in enumerate(gt):
        g.add_variable(("x", k), VarType.POSE4, p)
    for j, lm in enumerate(lms):
        g.add_variable(("l", j), VarType.POINT3, lm)
    g.add_pose_prior(("x", 0), gt[0], [1e-2] * 4)
    for k in range(1, len(gt)):
        g.add_between(("x", k - 1), ("x", k), between(gt[k - 1], gt[k]), [0.05, 0.05, 0.1, 0.02])
    for k, p in enumerate(gt):
        for j, lm in enumerate(lms):
            if (k + j) % 3:  # partial visibility
                g.add_point_obs(("x", k), ("l", j), inverse_transform_points(p, lm), [0.1] * 3)
    keys = {
        "landmarks": [("l", j) for j in range(6)],
        "poses": [("x", 4), ("x", 9)],
        "mixed": [("x", 2), ("l", 3), ("l", 0)],
    }[which]
    _, _, J = g._linearize(g._state())
    Sigma = np.linalg.inv((J.T @ J).toarray())
    schur = g.marginal_covariances(keys, method="schur")
    direct = g.marginal_covariances(keys, method="direct")
    for k in keys:
        i = g._index[k]
        o, d = g._offsets[i], int(g._types[i])
        want = Sigma[o : o + d, o : o + d]
        assert np.allclose(schur[k], want, rtol=1e-8, atol=1e-12)
        assert np.allclose(direct[k], want, rtol=1e-8, atol=1e-12)
    with pytest.raises(ValueError):
        g.marginal_covariances(keys, method="qr")


def test_copy_is_independent():
    g = FactorGraph()
    g.add_variable("a", VarType.POINT3, np.zeros(3))
    g.add_point_prior("a", [1.0, 2.0, 3.0], [1.0, 1.0, 1.0])
    g.optimize()
    h = g.copy()
    h.add_variable("b", VarType.POINT3, np.zeros(3))
    h.add_point_prior("b", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
    h.set_value("a", [9.0, 9.0, 9.0])
    assert not g.has("b") and g.num_factors() == 1
    assert np.allclose(g.value("a"), [1.0, 2.0, 3.0])


def _point_with_outliers(rng, kernel, n_in=30, n_out=20):
    g = FactorGraph()
    g.add_variable("p", VarType.POINT3, np.zeros(3))
    truth = np.array([2.0, -1.0, 0.5])
    for _ in range(n_in):
        g.add_point_prior("p", truth + rng.normal(0, 0.1, 3), [0.1, 0.1, 0.1])
    for _ in range(n_out):  # gross, one-sided outliers
        g.add_point_prior("p", truth + np.array([4.0, 3.0, 0.0]) + rng.normal(0, 1.0, 3),
                          [0.1, 0.1, 0.1])  # fmt: skip
    g.set_kernel("point_prior", kernel, 4.03)  # sqrt(chi2_3(0.999))
    g.optimize()
    return g, truth


def test_gnc_tls_rejects_gross_outliers_that_bias_huber(rng):
    g_gnc, truth = _point_with_outliers(rng, "gnc")
    g_hub, _ = _point_with_outliers(np.random.default_rng(1), "huber")
    assert np.linalg.norm(g_gnc.value("p") - truth) < 0.05
    assert np.linalg.norm(g_hub.value("p") - truth) > 0.2  # Huber is pulled by 40 % outliers
    mask = g_gnc.outlier_mask("point_prior")
    assert mask[:30].sum() <= 1 and mask[30:].all()  # TLS may drop a 3-sigma inlier


def test_gnc_is_plain_least_squares_without_outliers(rng):
    g1 = FactorGraph()
    g2 = FactorGraph()
    for g in (g1, g2):
        g.add_variable("p", VarType.POINT3, np.zeros(3))
    meas = [rng.normal(0, 0.1, 3) for _ in range(20)]
    for g in (g1, g2):
        for m in meas:
            g.add_point_prior("p", m, [0.1, 0.1, 0.1])
    g2.set_kernel("point_prior", "gnc", 4.03)
    g1.optimize()
    g2.optimize()
    assert np.allclose(g1.value("p"), g2.value("p"), atol=1e-8)
    assert not g2.outlier_mask("point_prior").any()
    with pytest.raises(ValueError):
        g2.set_kernel("point_prior", "cauchy", 1.0)
