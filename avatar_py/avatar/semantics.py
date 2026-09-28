"""Semantic class vocabulary and simulated open-vocabulary descriptors.

The class list is part of the wire contract (``class_id`` byte): append only,
never reorder (ADR-0005).

In the fast simulator, open-vocabulary embeddings (e.g. CLIP-family features
of a detected object) are *abstracted* as low-dimensional unit vectors: one
prototype per class plus a per-instance offset plus per-observation noise.
Only optical sensors (cameras) produce them; LiDAR and sonar return zeros.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

CLASS_NAMES: tuple[str, ...] = (
    "unknown",  # 0
    "pile",  # 1
    "hull",  # 2
    "buoy",  # 3
    "quay_wall",  # 4
    "bollard",  # 5
    "container",  # 6
    "crane_leg",  # 7
    "light_pole",  # 8
    "tree",  # 9
    "rock",  # 10
    "pipeline",  # 11
    "mooring_block",  # 12
)

CLASS_ID: dict[str, int] = {name: i for i, name in enumerate(CLASS_NAMES)}


def class_id(name: str) -> int:
    """Return the stable integer id of a class name."""
    try:
        return CLASS_ID[name]
    except KeyError as exc:
        raise KeyError(f"unknown semantic class {name!r}; add it to CLASS_NAMES") from exc


def class_prototypes(dim: int, seed: int = 7) -> NDArray[np.float64]:
    """Deterministic unit-norm prototype embedding per class, shape ``(n_classes, dim)``.

    Row 0 (``unknown``) is all zeros: "no semantic information".
    """
    if dim <= 0:
        return np.zeros((len(CLASS_NAMES), 0))
    rng = np.random.default_rng(seed)
    protos = rng.normal(size=(len(CLASS_NAMES), dim))
    protos /= np.linalg.norm(protos, axis=1, keepdims=True)
    protos[0] = 0.0
    return protos


def normalize(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return ``v / ||v||`` (zeros stay zeros)."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v)
