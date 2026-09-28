"""VoI-per-byte digest scheduler (T-C2-01): information algebra, medium weighting, runs."""

import numpy as np
import pytest

from avatar.agent import AvatarAgent, AvatarParams
from avatar.comm import codec
from avatar.comm.scheduler import (
    AlignmentInformation,
    match_probability,
    record_information,
    select_voi,
)
from avatar.runner import link_receivers, make_sim, run_decentralized
from avatar.semantics import CLASS_ID
from avatar.sim.scenarios import harbor_fleet
from avatar.types import Domain, LandmarkFlags, LinkType


def test_gain_matches_dense_log_det_oracle():
    rng = np.random.default_rng(3)
    info = AlignmentInformation.empty()
    for _ in range(4):
        info.add(rng.uniform(-30, 30, 2), rng.uniform(0.05, 1.0), rng.uniform(0.2, 1.0))
    xy = rng.uniform(-40, 40, (25, 2))
    sig = rng.uniform(0.01, 2.0, 25)
    got = info.gains(xy, sig)
    for k in range(25):
        J1 = info.J + record_information(xy[k], sig[k])
        want = np.linalg.slogdet(J1)[1] - np.linalg.slogdet(info.J)[1]
        assert got[k] == pytest.approx(want, rel=1e-8, abs=1e-9)
        assert info.gain(xy[k], sig[k]) == pytest.approx(want, rel=1e-8, abs=1e-9)


def test_greedy_prefers_spread_records_for_yaw():
    # Four precise records at the same spot, one farther away: after the first,
    # the lever arm (yaw) is worth more than repeating the same spot.
    cands = [(i, np.array([0.1 * i, 0.0]), 0.1, (1.0,)) for i in range(4)]
    cands.append((9, np.array([25.0, 10.0]), 0.1, (1.0,)))
    order = select_voi(cands, [AlignmentInformation.empty()], max_records=2)
    assert len(order) == 2 and 9 in order


def test_land_only_records_are_not_sent_to_underwater_receivers():
    container = CLASS_ID["container"]
    pile = CLASS_ID["pile"]
    rho_under = match_probability(LandmarkFlags.ABOVE, container, 10, Domain.UNDERWATER)
    rho_air = match_probability(LandmarkFlags.ABOVE, container, 10, Domain.AERIAL)
    rho_pile = match_probability(LandmarkFlags.ABOVE, pile, 10, Domain.UNDERWATER)
    assert rho_under == 0.0 and rho_air > 0.95 and 0.0 < rho_pile < rho_air
    assert match_probability(LandmarkFlags.BELOW, 0, 1, Domain.UNDERWATER) < match_probability(
        LandmarkFlags.BELOW, 0, 9, Domain.UNDERWATER
    )  # well-observed records are more likely to be re-detected
    cands = [
        (1, np.array([0.0, 0.0]), 0.1, (rho_under,)),  # container: useless below
        (2, np.array([20.0, 0.0]), 0.1, (rho_pile,)),  # pile top: coaxial match
    ]
    assert select_voi(cands, [AlignmentInformation.empty()], max_records=5) == [2]


def test_value_sums_over_receiver_domains():
    # Same geometry; the record both domains can use beats the one only air can.
    cands = [
        (1, np.array([10.0, 0.0]), 0.1, (1.0, 0.0)),
        (2, np.array([-10.0, 0.0]), 0.1, (1.0, 1.0)),
    ]
    infos = [AlignmentInformation.empty(), AlignmentInformation.empty()]
    assert select_voi(cands, infos, max_records=1) == [2]
    assert infos[1].J[0, 0] > 1.0  # the selection was credited to both domains


@pytest.mark.parametrize("budget", [0, 30, 64, 200, 1000, 5000])
@pytest.mark.parametrize("dim", [0, 8])
def test_capacity_fits_the_budget(budget, dim):
    per_pkt = 12
    n = AvatarAgent._capacity(budget, per_pkt, dim)
    full, rest = divmod(n, per_pkt)
    size = full * codec.digest_size(per_pkt, dim) + (codec.digest_size(rest, dim) if rest else 0)
    assert size <= budget
    more = n + 1
    full, rest = divmod(more, per_pkt)
    bigger = full * codec.digest_size(per_pkt, dim) + (codec.digest_size(rest, dim) if rest else 0)
    assert bigger > budget  # and one more record would not fit


def test_link_receivers_follow_the_gateway():
    sc = harbor_fleet(np.random.default_rng(0))
    uuv = next(a for a in sc.agents if a.name == "uuv_0")
    ugv = next(a for a in sc.agents if a.name == "ugv_0")
    # Acoustic reaches the other UUV and, through the quay gateway, the RF agents.
    assert set(link_receivers(sc, uuv.agent_id)[LinkType.ACOUSTIC]) == {
        Domain.UNDERWATER,
        Domain.AERIAL,
        Domain.GROUND,
    }
    assert Domain.UNDERWATER in link_receivers(sc, ugv.agent_id)[LinkType.RF]


@pytest.mark.parametrize("scheduler", ["fifo", "quality", "voi"])
def test_both_schedulers_align_the_fleet(scheduler):
    params = AvatarParams(scheduler=scheduler)
    sc, sim = make_sim("harbor_fleet", 0, 400.0, params)
    res = run_decentralized(sc, sim, params, 0)
    assert res.metrics["team_connected_s"] is not None
    assert res.metrics["ate_team_m"] < 0.5


def test_unknown_scheduler_is_rejected():
    params = AvatarParams(scheduler="lifo")
    sc, sim = make_sim("harbor_fleet", 0, 60.0, params)
    with pytest.raises(ValueError):
        run_decentralized(sc, sim, params, 0)


def test_acoustic_rate_override_keeps_the_rest_of_the_profile():
    base = harbor_fleet(np.random.default_rng(0)).channels[LinkType.ACOUSTIC]
    slow = harbor_fleet(np.random.default_rng(0), acoustic_bps=16).channels[LinkType.ACOUSTIC]
    assert slow.bandwidth_bps == 16.0 and base.bandwidth_bps == 64.0
    assert slow.mtu_B == base.mtu_B and slow.max_range_m == base.max_range_m
