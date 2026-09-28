import numpy as np

from avatar.frontend.association import AssociationParams, LandmarkSet, align
from avatar.geometry import inverse, transform_points
from avatar.types import LandmarkFlags


def make_set(ids, pos, above, extents=None, classes=None, dim=0):
    n = len(ids)
    flags = np.where(above, int(LandmarkFlags.ABOVE), int(LandmarkFlags.BELOW))
    return LandmarkSet(
        ids=np.asarray(ids),
        positions=np.asarray(pos, float),
        sigma_xy=np.full(n, 0.1),
        sigma_z=np.full(n, 0.2),
        extents=np.ones((n, 3)) if extents is None else np.asarray(extents, float),
        class_ids=np.zeros(n, int) if classes is None else np.asarray(classes),
        flags=flags,
        descriptors=np.zeros((n, dim)),
    )


def scene(rng, n=15):
    xy = rng.uniform(-30, 30, (n, 2))
    fp = rng.uniform(0.4, 3.0, n)
    return xy, fp


def test_same_medium_alignment_recovers_transform(rng):
    xy, fp = scene(rng)
    pos = np.column_stack([xy, rng.uniform(1, 3, len(xy))])
    ext = np.column_stack([fp, fp, np.full(len(xy), 2.0)])
    T = np.array([12.0, -5.0, 0.0, 1.1])  # T_mine_from_remote
    remote_pos = transform_points(inverse(T), pos)
    remote = make_set(np.arange(len(xy)), remote_pos, np.ones(len(xy), bool), ext)
    mine = make_set(
        np.arange(len(xy)) + 100, pos + rng.normal(0, 0.05, pos.shape), np.ones(len(xy), bool), ext
    )
    res = align(mine, remote, AssociationParams())
    assert res is not None and res.n_inliers >= 12
    assert np.hypot(*(res.T_mine_from_remote[:2] - T[:2])) < 0.1
    assert abs(res.T_mine_from_remote[3] - T[3]) < 0.01
    assert all(i == j for i, j, cross in res.pairs) and not any(c for *_, c in res.pairs)


def test_cross_medium_alignment_uses_horizontal_only(rng):
    """Above-water parts (mine) vs below-water parts (remote) of the same piles."""
    xy, fp = scene(rng, 12)
    above = np.column_stack([xy, np.full(len(xy), 1.0)])
    below = np.column_stack([xy, np.full(len(xy), -6.0)])
    ext_a = np.column_stack([fp, fp, np.full(len(xy), 2.0)])
    ext_b = np.column_stack([fp, fp, np.full(len(xy), 12.0)])
    T = np.array([-3.0, 8.0, 0.0, -2.0])
    mine = make_set(np.arange(12), above, np.ones(12, bool), ext_a)
    remote_pos = transform_points(inverse(T), below)
    remote = make_set(np.arange(12), remote_pos, np.zeros(12, bool), ext_b)
    res = align(mine, remote, AssociationParams())
    assert res is not None and res.n_inliers >= 10
    assert all(cross for *_, cross in res.pairs)
    assert np.hypot(*(res.T_mine_from_remote[:2] - T[:2])) < 0.05
    assert abs(res.T_mine_from_remote[2]) < 1e-9  # no same-medium pairs: datum prior z = 0


def test_association_is_id_invariant(rng):
    xy, fp = scene(rng)
    pos = np.column_stack([xy, np.full(len(xy), 2.0)])
    ext = np.column_stack([fp, fp, np.ones(len(xy))])
    T = np.array([5.0, 5.0, 0.0, 0.3])
    remote_pos = transform_points(inverse(T), pos)
    mine = make_set(np.arange(len(xy)), pos, np.ones(len(xy), bool), ext)
    r1 = align(
        mine,
        make_set(np.arange(len(xy)), remote_pos, np.ones(len(xy), bool), ext),
        AssociationParams(),
    )
    r2 = align(
        mine,
        make_set(rng.permutation(60000)[: len(xy)], remote_pos, np.ones(len(xy), bool), ext),
        AssociationParams(),
    )
    assert np.allclose(r1.T_mine_from_remote, r2.T_mine_from_remote)


def test_no_alignment_without_enough_support(rng):
    mine = make_set([1, 2, 3], rng.uniform(-5, 5, (3, 3)), np.ones(3, bool))
    remote = make_set([4, 5, 6], rng.uniform(-5, 5, (3, 3)), np.ones(3, bool))
    assert align(mine, remote, AssociationParams(min_inliers=4)) is None


def test_class_gating_blocks_incompatible_pairs(rng):
    xy, _ = scene(rng, 10)
    pos = np.column_stack([xy, np.full(10, 2.0)])
    mine = make_set(np.arange(10), pos, np.ones(10, bool), classes=np.full(10, 1))
    remote = make_set(np.arange(10), pos, np.ones(10, bool), classes=np.full(10, 2))
    assert align(mine, remote, AssociationParams()) is None
