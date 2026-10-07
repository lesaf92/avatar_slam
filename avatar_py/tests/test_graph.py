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
        "between_bias",
        "between_bias_scale",
        "scalar_prior",
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
    elif factor == "between_bias":
        g.add_variable("bias", VarType.SCALAR, [rng.normal(0, 0.01)])
        g.add_between_bias("a", "b", "bias", rand_pose(rng), 3.7, sig4)
    elif factor == "between_bias_scale":
        for k in ("bias", "s", "g"):
            g.add_variable(k, VarType.SCALAR, [rng.normal(0, 0.01)])
        g.add_between_bias("a", "b", "bias", rand_pose(rng), 3.7, sig4, "s", "g")
    elif factor == "scalar_prior":
        g.add_variable("bias", VarType.SCALAR, [0.02])
        g.add_scalar_prior("bias", 0.0, 0.01)
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


def test_heading_bias_is_recovered_on_a_loop(rng):
    """A square loop driven with a constant heading bias: estimating the bias
    (b ~ 1.5 mrad/m) closes the loop that a bias-free model cannot explain."""
    b_true = 1.5e-3
    gt = [np.zeros(4)]
    for _ in range(40):
        gt.append(compose(gt[-1], [2.0, 0.0, 0.0, np.pi / 20]))
    lm = np.array([3.0, 4.0, 0.0])
    g = FactorGraph()
    for k, p in enumerate(gt):
        g.add_variable(("x", k), VarType.POSE4, p)
    g.add_variable("bias", VarType.SCALAR, [0.0])
    g.add_scalar_prior("bias", 0.0, 2e-3)
    g.add_pose_prior(("x", 0), gt[0], [1e-3] * 4)
    g.add_variable("l", VarType.POINT3, lm)
    for k in range(1, len(gt)):
        inc = between(gt[k - 1], gt[k])
        d = float(np.linalg.norm(inc[:3]))
        meas = inc + np.array([0, 0, 0, b_true * d])
        g.add_between_bias(("x", k - 1), ("x", k), "bias", meas, d, [0.01, 0.01, 0.01, 1e-4])
    for k in (0, 20, 40):  # the same landmark seen three times closes the loop
        g.add_point_obs(("x", k), "l", inverse_transform_points(gt[k], lm), [0.01] * 3)
    g.optimize()
    assert g.value("bias")[0] == pytest.approx(b_true, rel=0.05)


def test_odometry_scale_errors_are_recovered_on_a_loop(rng):
    """A rectangle driven with a 2 % translation scale error and a 3 % gyro scale error (plus a
    heading bias), seen against landmarks at known positions: the three states recover them."""
    b_true, s_true, g_true = 1e-3, 0.02, 0.03
    gt = [np.zeros(4)]
    for leg in range(8):
        for _ in range(10):
            gt.append(compose(gt[-1], [2.0, 0.0, 0.0, 0.0]))
        gt.append(compose(gt[-1], [0.0, 0.0, 0.0, np.pi / 2 * (1 if leg < 4 else -1)]))
    g = FactorGraph()
    for k, p in enumerate(gt):
        g.add_variable(("x", k), VarType.POSE4, p)
    for key, std in (("b", 2e-3), ("s", 0.02), ("g", 0.02)):
        g.add_variable(key, VarType.SCALAR, [0.0])
        g.add_scalar_prior(key, 0.0, std)
    g.add_pose_prior(("x", 0), gt[0], [1e-3] * 4)
    for k in range(1, len(gt)):
        inc = between(gt[k - 1], gt[k])
        d = float(np.linalg.norm(inc[:3]))
        meas = np.array([*(1 + s_true) * inc[:3], (1 + g_true) * inc[3] + b_true * d])
        g.add_between_bias(("x", k - 1), ("x", k), "b", meas, d, [0.01, 0.01, 0.01, 1e-4], "s", "g")
    lms = rng.uniform(-5, 25, (12, 3)) * [1, 1, 0]
    for n, lm in enumerate(lms):
        g.add_variable(("l", n), VarType.POINT3, lm)
        g.add_point_prior(("l", n), lm, [0.01] * 3)
        for k in range(0, len(gt), 3):
            if np.hypot(*(lm[:2] - gt[k][:2])) < 12:
                g.add_point_obs(("x", k), ("l", n), inverse_transform_points(gt[k], lm), [0.01] * 3)
    g.optimize(max_iters=50)
    assert g.value("s")[0] == pytest.approx(s_true, abs=2e-3)
    assert g.value("g")[0] == pytest.approx(g_true, abs=3e-3)
    assert g.value("b")[0] == pytest.approx(b_true, abs=3e-4)
