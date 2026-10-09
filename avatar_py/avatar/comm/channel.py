"""Physical link models: RF above water, acoustic below water.

The numbers are order-of-magnitude defaults for a harbour scenario:

* **RF** (Wi-Fi/mesh class): Mbps, ms latency, a few hundred metres, blocked by
  water (an endpoint must be at or above the surface).
* **Acoustic** (modem class): 10²–10⁴ bps, ~0.2 s fixed latency plus
  propagation at ≈ 1500 m/s, loss that grows with range, and a shared
  half-duplex medium (modelled as a TDMA share among the nodes in water).

Sweeping these parameters is part of the experiment protocol (PLAN §6).

``CHANNEL_PROFILES`` also holds the **reference-fleet stack** (ADR-0006,
``docs/hardware.md``): a 5 GHz Wi-Fi mesh above water, and a Water Linked
Modem-M64 (64 bps, 200 m, 1.5–2.5 s latency, half duplex) or a Blueprint Subsea
SeaTrac X150 (100 bps class, 1000 m) under water. Wire packets on those modems
are capped at 64 B; the M64 cuts them into 8-byte modem packets (``avatar.comm.m64``,
task T-C6-01, +25 % on the air), which is not modelled here yet (T-C6-02).
"""

from __future__ import annotations

from dataclasses import dataclass

from avatar.types import WATER_SURFACE_TOL_M, LinkType


@dataclass(frozen=True)
class ChannelModel:
    """Point-to-point/broadcast link class."""

    link_type: LinkType
    bandwidth_bps: float
    base_latency_s: float
    propagation_speed_mps: float
    max_range_m: float
    loss_near: float  # packet loss probability at zero range
    loss_far: float  # packet loss probability at max range
    mtu_B: int
    utilization: float  # fraction of channel time the map-sharing layer may use

    def __post_init__(self) -> None:
        if self.bandwidth_bps <= 0 or self.max_range_m <= 0 or self.mtu_B <= 16:
            raise ValueError("invalid channel parameters")
        if not (0 <= self.loss_near <= 1 and 0 <= self.loss_far <= 1):
            raise ValueError("loss probabilities must be in [0, 1]")
        if not 0 < self.utilization <= 1:
            raise ValueError("utilization must be in (0, 1]")

    def medium_ok(self, z_a: float, z_b: float) -> bool:
        """Whether both endpoints are in a medium this link propagates through."""
        if self.link_type == LinkType.RF:
            return z_a >= -WATER_SURFACE_TOL_M and z_b >= -WATER_SURFACE_TOL_M
        return z_a <= WATER_SURFACE_TOL_M and z_b <= WATER_SURFACE_TOL_M

    def loss_prob(self, distance_m: float) -> float:
        """Packet loss probability at ``distance_m`` (1 beyond range)."""
        if distance_m > self.max_range_m:
            return 1.0
        f = (distance_m / self.max_range_m) ** 2
        return self.loss_near + (self.loss_far - self.loss_near) * f

    def airtime_s(self, size_B: int, share: float = 1.0) -> float:
        """Channel time to transmit ``size_B`` bytes with a fraction ``share`` of the rate."""
        return 8.0 * size_B / (self.bandwidth_bps * max(share, 1e-9))

    def latency_s(self, distance_m: float) -> float:
        """Fixed plus propagation latency (excluding airtime)."""
        return self.base_latency_s + distance_m / self.propagation_speed_mps

    def budget_B(self, period_s: float, share: float = 1.0) -> int:
        """Bytes the sharing layer may send per ``period_s`` given its utilization."""
        return int(self.bandwidth_bps * share * self.utilization * period_s / 8.0)


RF_DEFAULT = ChannelModel(
    link_type=LinkType.RF,
    bandwidth_bps=2e6,
    base_latency_s=0.005,
    propagation_speed_mps=3e8,
    max_range_m=300.0,
    loss_near=0.01,
    loss_far=0.3,
    mtu_B=1400,
    utilization=0.1,
)

ACOUSTIC_DEFAULT = ChannelModel(
    link_type=LinkType.ACOUSTIC,
    bandwidth_bps=1000.0,
    base_latency_s=0.2,
    propagation_speed_mps=1500.0,
    max_range_m=1500.0,
    loss_near=0.05,
    loss_far=0.4,
    mtu_B=256,
    utilization=0.5,
)

WIFI_MESH = ChannelModel(
    link_type=LinkType.RF,
    bandwidth_bps=10e6,  # conservative effective throughput of a 5 GHz mesh hop
    base_latency_s=0.005,
    propagation_speed_mps=3e8,
    max_range_m=250.0,
    loss_near=0.01,
    loss_far=0.3,
    mtu_B=1400,
    utilization=0.1,
)

ACOUSTIC_M64 = ChannelModel(
    link_type=LinkType.ACOUSTIC,
    bandwidth_bps=64.0,  # Water Linked Modem-M64 datasheet
    base_latency_s=2.0,  # datasheet: 1.5-2.5 s
    propagation_speed_mps=1500.0,
    max_range_m=200.0,
    loss_near=0.05,
    loss_far=0.4,
    mtu_B=64,
    utilization=0.5,
)

ACOUSTIC_X150 = ChannelModel(
    link_type=LinkType.ACOUSTIC,
    bandwidth_bps=100.0,  # SeaTrac: "100 baud" data rate (verify effective payload rate)
    base_latency_s=1.0,  # UNVERIFIED
    propagation_speed_mps=1500.0,
    max_range_m=1000.0,
    loss_near=0.05,
    loss_far=0.4,
    mtu_B=64,
    utilization=0.5,
)

CHANNEL_PROFILES: dict[str, ChannelModel] = {
    "rf_generic": RF_DEFAULT,
    "acoustic_generic": ACOUSTIC_DEFAULT,
    "wifi_mesh": WIFI_MESH,
    "m64": ACOUSTIC_M64,
    "x150": ACOUSTIC_X150,
}
