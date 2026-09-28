"""Shared enumerations with stable integer codes (docs/conventions.md §5)."""

from __future__ import annotations

from enum import IntEnum, IntFlag

WATER_SURFACE_TOL_M = 0.3
"""|z| [m] below which a vehicle counts as being at the water surface."""


class Domain(IntEnum):
    """Operating domain of an agent. Codes are part of the wire format."""

    AERIAL = 0
    GROUND = 1
    SURFACE = 2
    UNDERWATER = 3


class Medium(IntEnum):
    """Medium of an observed landmark *part* relative to the waterline datum."""

    ABOVE = 0
    BELOW = 1


class LandmarkFlags(IntFlag):
    """Bit flags carried by each landmark record on the wire (spec §3)."""

    NONE = 0
    ABOVE = 1 << 0
    BELOW = 1 << 1
    CAMERA = 1 << 2
    LIDAR = 1 << 3
    SONAR = 1 << 4


class LinkType(IntEnum):
    """Physical communication link class."""

    RF = 0
    ACOUSTIC = 1


def medium_flag(medium: Medium) -> LandmarkFlags:
    """Return the wire flag for a medium."""
    return LandmarkFlags.ABOVE if medium == Medium.ABOVE else LandmarkFlags.BELOW


def medium_from_flags(flags: int) -> Medium:
    """Recover the medium of a landmark record from its flags.

    Raises
    ------
    ValueError
        If neither or both medium bits are set.
    """
    above = bool(flags & LandmarkFlags.ABOVE)
    below = bool(flags & LandmarkFlags.BELOW)
    if above == below:
        raise ValueError(f"landmark flags {flags:#04x} must set exactly one medium bit")
    return Medium.ABOVE if above else Medium.BELOW
