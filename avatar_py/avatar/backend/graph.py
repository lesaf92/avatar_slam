"""Sparse 4-DoF factor graph with Levenberg–Marquardt (reference back-end).

Variables
---------
* ``POSE4``: ``[x, y, z, yaw]``, a gravity-aligned pose (also used for
  inter-agent frame transforms ``T_i_from_j``).
* ``POINT3``: ``[x, y, z]``, a landmark part.

Factors (all with diagonal Gaussian noise, given as 1-σ vectors)
----------------------------------------------------------------
============== ================================================================
pose_prior     ``[p - p0, wrap(ψ - ψ0)]``
between        ``[R(-ψi)(pj - pi) - dp, wrap(ψj - ψi - dψ)]``
point_obs      ``R(-ψi)(l - pi) - m``           (gravity-aligned body measurement)
z_prior        ``z_i - z0``                     (depth, surface, barometer)
point_prior    ``l - l0``
linked_point   ``l - (t + R(θ) q)``              (``q`` fixed, in the remote frame;
               ``T = [t, θ]`` is a POSE4 frame variable). With
               ``horizontal_only`` only the x/y rows are used (cross-medium).
coaxial        ``(la - lb)[:2]``                 (parts of one structure)
range          ``||pa - pb|| - d``
============== ================================================================

Robust factors use a Huber kernel on the whitened residual norm, applied by
iteratively reweighted least squares. The solver is Levenberg–Marquardt on
the sparse normal equations, with multiplicative diagonal damping.
"""

from __future__ import annotations

from collections.abc import Hashable, Iterable
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


class VarType(IntEnum):
    """Variable kinds and their tangent dimensions."""

    POSE4 = 4
    POINT3 = 3


def _wrap(a: FloatArray) -> FloatArray:
    return np.arctan2(np.sin(a), np.cos(a))


@dataclass
class _FactorBlock:
    """Growable storage for one factor type (converted to arrays when solving)."""

    vars: list[tuple[int, ...]] = field(default_factory=list)
    meas: list[FloatArray] = field(default_factory=list)
    sigmas: list[FloatArray] = field(default_factory=list)
    flags: list[bool] = field(default_factory=list)  # per-factor boolean option
    robust_k: float | None = None
    _arrays: tuple | None = field(default=None, repr=False)  # cache, reset by add()

    def add(self, var_idx: tuple[int, ...], meas, sigmas, flag: bool = False) -> None:
        sig = np.asarray(sigmas, dtype=float)
        if np.any(sig <= 0) or not np.all(np.isfinite(sig)):
            raise ValueError("noise sigmas must be positive and finite")
        self.vars.append(var_idx)
        self.meas.append(np.atleast_1d(np.asarray(meas, dtype=float)))
        self.sigmas.append(sig)
        self.flags.append(flag)
        self._arrays = None

    def arrays(self) -> tuple[NDArray[np.int64], FloatArray, FloatArray, NDArray[np.bool_]]:
        """(variable indices, measurements, sigmas, flags) stacked; cached, read-only."""
        if self._arrays is None:
            out = (
                np.asarray(self.vars, dtype=np.int64),
                np.vstack(self.meas),
                np.vstack(self.sigmas),
                np.asarray(self.flags, dtype=bool),
            )
            for a in out:
                a.setflags(write=False)
            self._arrays = out
        return self._arrays

    def copy(self) -> _FactorBlock:
        """Copy with its own lists; the (never mutated) arrays and cache are shared."""
        return _FactorBlock(
            list(self.vars), list(self.meas), list(self.sigmas), list(self.flags),
            self.robust_k, self._arrays,
        )  # fmt: skip

    def __len__(self) -> int:
        return len(self.vars)


@dataclass
class OptimizeResult:
    """Summary of a solve."""

    initial_cost: float
    final_cost: float
    iterations: int
    converged: bool


def _factor(A: sp.csc_matrix):
    """Sparse LU of a symmetric positive (semi)definite matrix.

    A symmetric minimum-degree ordering (on AᵀA + A) gives ~4x less fill-in and
    ~2x faster solves than SuperLU's default COLAMD on SLAM normal equations
    (docs/LOG.md L10).
    """
    return spla.splu(A, permc_spec="MMD_AT_PLUS_A")


class FactorGraph:
    """A 4-DoF factor graph. Keys are arbitrary hashables, e.g. ``("x", agent, k)``."""

    FACTOR_TYPES = (
        "pose_prior",
        "between",
        "point_obs",
        "z_prior",
        "point_prior",
        "linked_point",
        "coaxial",
        "range",
    )

    def __init__(self, robust_k: float = 2.0) -> None:
        self._index: dict[Hashable, int] = {}
        self._keys: list[Hashable] = []
        self._types: list[VarType] = []
        self._offsets: list[int] = []
        self._dim = 0
        self._values: list[FloatArray] = []
        self._blocks: dict[str, _FactorBlock] = {t: _FactorBlock() for t in self.FACTOR_TYPES}
        for name in ("linked_point", "range"):
            self._blocks[name].robust_k = robust_k

    # ------------------------------------------------------------------ variables
    def add_variable(self, key: Hashable, vtype: VarType, initial) -> int:
        """Add a variable with an initial value; returns its index."""
        if key in self._index:
            raise KeyError(f"variable {key!r} already exists")
        val = np.asarray(initial, dtype=float).copy()
        if val.shape != (int(vtype),):
            raise ValueError(f"initial value for {vtype.name} must have shape ({int(vtype)},)")
        idx = len(self._keys)
        self._index[key] = idx
        self._keys.append(key)
        self._types.append(vtype)
        self._offsets.append(self._dim)
        self._dim += int(vtype)
        self._values.append(val)
        return idx

    def has(self, key: Hashable) -> bool:
        return key in self._index

    def value(self, key: Hashable) -> FloatArray:
        """Current estimate of a variable (copy)."""
        return self._values[self._index[key]].copy()

    def set_value(self, key: Hashable, value) -> None:
        """Overwrite a variable's current estimate (e.g. warm start)."""
        idx = self._index[key]
        val = np.asarray(value, dtype=float)
        if val.shape != self._values[idx].shape:
            raise ValueError("shape mismatch")
        self._values[idx] = val.copy()

    def keys(self) -> list[Hashable]:
        return list(self._keys)

    @property
    def dim(self) -> int:
        """Total tangent-space dimension."""
        return self._dim

    def num_factors(self) -> int:
        return sum(len(b) for b in self._blocks.values())

    def _idx(self, key: Hashable, vtype: VarType) -> int:
        idx = self._index[key]
        if self._types[idx] != vtype:
            raise TypeError(f"variable {key!r} is {self._types[idx].name}, expected {vtype.name}")
        return idx

    # ------------------------------------------------------------------ factors
    def add_pose_prior(self, key, mean, sigmas) -> None:
        self._blocks["pose_prior"].add((self._idx(key, VarType.POSE4),), mean, sigmas)

    def add_between(self, key_i, key_j, meas, sigmas) -> None:
        i = self._idx(key_i, VarType.POSE4)
        j = self._idx(key_j, VarType.POSE4)
        self._blocks["between"].add((i, j), meas, sigmas)

    def add_point_obs(self, pose_key, point_key, meas, sigmas) -> None:
        i = self._idx(pose_key, VarType.POSE4)
        l_ = self._idx(point_key, VarType.POINT3)
        self._blocks["point_obs"].add((i, l_), meas, sigmas)

    def add_z_prior(self, pose_key, z, sigma) -> None:
        self._blocks["z_prior"].add((self._idx(pose_key, VarType.POSE4),), [z], [sigma])

    def add_point_prior(self, point_key, mean, sigmas) -> None:
        self._blocks["point_prior"].add((self._idx(point_key, VarType.POINT3),), mean, sigmas)

    def add_linked_point(
        self, point_key, frame_key, remote_point, sigmas, horizontal_only: bool = False
    ) -> None:
        """Robust factor tying a local landmark to a remote one through a frame variable."""
        l_ = self._idx(point_key, VarType.POINT3)
        t = self._idx(frame_key, VarType.POSE4)
        self._blocks["linked_point"].add((l_, t), remote_point, sigmas, horizontal_only)

    def add_coaxial(self, point_a, point_b, sigma_xy: float) -> None:
        a = self._idx(point_a, VarType.POINT3)
        b = self._idx(point_b, VarType.POINT3)
        self._blocks["coaxial"].add((a, b), [0.0, 0.0], [sigma_xy, sigma_xy])

    def add_range(self, pose_a, pose_b, range_m: float, sigma: float) -> None:
        a = self._idx(pose_a, VarType.POSE4)
        b = self._idx(pose_b, VarType.POSE4)
        self._blocks["range"].add((a, b), [range_m], [sigma])

    def copy(self) -> FactorGraph:
        """Independent copy (values and factor lists).

        Measurement arrays are shared: the graph never mutates them in place.
        """
        g = FactorGraph.__new__(FactorGraph)
        g._index = dict(self._index)
        g._keys = list(self._keys)
        g._types = list(self._types)
        g._offsets = list(self._offsets)
        g._dim = self._dim
        g._values = [v.copy() for v in self._values]
        g._blocks = {name: blk.copy() for name, blk in self._blocks.items()}
        return g

    # ------------------------------------------------------------------ linearization
    def _state(self) -> FloatArray:
        return np.concatenate(self._values) if self._values else np.zeros(0)

    def _set_state(self, x: FloatArray) -> None:
        for i, off in enumerate(self._offsets):
            self._values[i] = x[off : off + int(self._types[i])].copy()

    def _yaw_mask(self) -> NDArray[np.bool_]:
        mask = np.zeros(self._dim, dtype=bool)
        for t, off in zip(self._types, self._offsets, strict=True):
            if t == VarType.POSE4:
                mask[off + 3] = True
        return mask

    def _linearize(
        self, x: FloatArray, jacobian: bool = True
    ) -> tuple[float, FloatArray, sp.csr_matrix | None]:
        """Return (robust cost, weighted whitened residual, weighted whitened Jacobian)."""
        off = np.asarray(self._offsets, dtype=np.int64)
        res_parts: list[FloatArray] = []
        rows: list[NDArray[np.int64]] = []
        cols: list[NDArray[np.int64]] = []
        vals: list[FloatArray] = []
        cost = 0.0
        row0 = 0
        for name in self.FACTOR_TYPES:
            blk = self._blocks[name]
            if not blk.vars:
                continue
            r, J_entries = getattr(self, f"_lin_{name}")(x, off, blk, jacobian)
            # r: (n_factors, m) whitened; J_entries: (row_in_factor, global_col, value) arrays
            n, m = r.shape
            valid = np.isfinite(r).all(axis=1)
            r = np.where(np.isfinite(r), r, 0.0)
            e = np.linalg.norm(r, axis=1)
            if blk.robust_k is not None:
                k = blk.robust_k
                w = np.where(e <= k, 1.0, k / np.maximum(e, 1e-12))
                cost += float(np.sum(np.where(e <= k, 0.5 * e**2, k * (e - 0.5 * k))[valid]))
            else:
                w = np.ones(n)
                cost += float(0.5 * np.sum(e[valid] ** 2))
            sw = np.sqrt(w) * valid
            res_parts.append((r * sw[:, None]).ravel())
            if jacobian:
                base = row0 + np.arange(n) * m
                for rr, cc, vv in J_entries:
                    rows.append(base + rr)
                    cols.append(cc)
                    vals.append(vv * sw)
            row0 += n * m
        residual = np.concatenate(res_parts) if res_parts else np.zeros(0)
        if not jacobian:
            return cost, residual, None
        J = sp.csr_matrix(
            (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
            shape=(row0, self._dim),
        )
        return cost, residual, J

    @staticmethod
    def _arr(blk: _FactorBlock):
        idx, meas, sig, _ = blk.arrays()
        return idx, meas, sig

    def _lin_pose_prior(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        o = off[idx[:, 0]]
        v = x[o[:, None] + np.arange(4)]
        r = v - meas
        r[:, 3] = _wrap(r[:, 3])
        r /= sig
        J = []
        if jac:
            for c in range(4):
                J.append((c, o + c, 1.0 / sig[:, c]))
        return r, J

    def _lin_between(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        oi, oj = off[idx[:, 0]], off[idx[:, 1]]
        pi = x[oi[:, None] + np.arange(3)]
        pj = x[oj[:, None] + np.arange(3)]
        yi, yj = x[oi + 3], x[oj + 3]
        c, s = np.cos(yi), np.sin(yi)
        v = pj - pi
        rp = np.column_stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], v[:, 2]])
        r = np.empty((len(idx), 4))
        r[:, :3] = rp - meas[:, :3]
        r[:, 3] = _wrap(yj - yi - meas[:, 3])
        r /= sig
        J = []
        if jac:
            dyaw = np.column_stack([-s * v[:, 0] + c * v[:, 1], -c * v[:, 0] - s * v[:, 1]])
            # rows 0,1: R(-yi) entries [[c, s], [-s, c]]
            R = [[c, s], [-s, c]]
            for row in range(2):
                for col in range(2):
                    J.append((row, oi + col, -R[row][col] / sig[:, row]))
                    J.append((row, oj + col, R[row][col] / sig[:, row]))
                J.append((row, oi + 3, dyaw[:, row] / sig[:, row]))
            J.append((2, oi + 2, -1.0 / sig[:, 2]))
            J.append((2, oj + 2, 1.0 / sig[:, 2]))
            J.append((3, oi + 3, -1.0 / sig[:, 3]))
            J.append((3, oj + 3, 1.0 / sig[:, 3]))
        return r, J

    def _lin_point_obs(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        oi, ol = off[idx[:, 0]], off[idx[:, 1]]
        pi = x[oi[:, None] + np.arange(3)]
        lm = x[ol[:, None] + np.arange(3)]
        yi = x[oi + 3]
        c, s = np.cos(yi), np.sin(yi)
        v = lm - pi
        pred = np.column_stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], v[:, 2]])
        r = (pred - meas) / sig
        J = []
        if jac:
            dyaw = np.column_stack([-s * v[:, 0] + c * v[:, 1], -c * v[:, 0] - s * v[:, 1]])
            R = [[c, s], [-s, c]]
            for row in range(2):
                for col in range(2):
                    J.append((row, oi + col, -R[row][col] / sig[:, row]))
                    J.append((row, ol + col, R[row][col] / sig[:, row]))
                J.append((row, oi + 3, dyaw[:, row] / sig[:, row]))
            J.append((2, oi + 2, -1.0 / sig[:, 2]))
            J.append((2, ol + 2, 1.0 / sig[:, 2]))
        return r, J

    def _lin_z_prior(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        o = off[idx[:, 0]]
        r = (x[o + 2][:, None] - meas) / sig
        J = [(0, o + 2, 1.0 / sig[:, 0])] if jac else []
        return r, J

    def _lin_point_prior(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        o = off[idx[:, 0]]
        r = (x[o[:, None] + np.arange(3)] - meas) / sig
        J = [(c, o + c, 1.0 / sig[:, c]) for c in range(3)] if jac else []
        return r, J

    def _lin_linked_point(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        horiz = blk.arrays()[3]
        ol, ot = off[idx[:, 0]], off[idx[:, 1]]
        lm = x[ol[:, None] + np.arange(3)]
        t = x[ot[:, None] + np.arange(3)]
        th = x[ot + 3]
        c, s = np.cos(th), np.sin(th)
        q = meas
        rq = np.column_stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2]])
        r = (lm - t - rq) / sig
        r[horiz, 2] = 0.0  # cross-medium: vertical row disabled
        J = []
        if jac:
            drq = np.column_stack([-s * q[:, 0] - c * q[:, 1], c * q[:, 0] - s * q[:, 1]])
            zmask = (~horiz).astype(float)
            for row in range(3):
                scale = 1.0 / sig[:, row]
                if row == 2:
                    scale = scale * zmask
                J.append((row, ol + row, scale))
                J.append((row, ot + row, -scale))
                if row < 2:
                    J.append((row, ot + 3, -drq[:, row] * scale))
        return r, J

    def _lin_coaxial(self, x, off, blk, jac):
        idx, _, sig = self._arr(blk)
        oa, ob = off[idx[:, 0]], off[idx[:, 1]]
        r = (x[oa[:, None] + np.arange(2)] - x[ob[:, None] + np.arange(2)]) / sig
        J = []
        if jac:
            for c in range(2):
                J.append((c, oa + c, 1.0 / sig[:, c]))
                J.append((c, ob + c, -1.0 / sig[:, c]))
        return r, J

    def _lin_range(self, x, off, blk, jac):
        idx, meas, sig = self._arr(blk)
        oa, ob = off[idx[:, 0]], off[idx[:, 1]]
        d = x[oa[:, None] + np.arange(3)] - x[ob[:, None] + np.arange(3)]
        n = np.maximum(np.linalg.norm(d, axis=1), 1e-9)
        r = ((n - meas[:, 0]) / sig[:, 0])[:, None]
        J = []
        if jac:
            u = d / n[:, None]
            for c in range(3):
                J.append((0, oa + c, u[:, c] / sig[:, 0]))
                J.append((0, ob + c, -u[:, c] / sig[:, 0]))
        return r, J

    # ------------------------------------------------------------------ solve
    def cost(self) -> float:
        """Robust cost at the current estimate."""
        c, _, _ = self._linearize(self._state(), jacobian=False)
        return c

    def optimize(
        self,
        max_iters: int = 30,
        rel_tol: float = 1e-8,
        step_tol: float = 1e-8,
        lambda0: float = 1e-4,
    ) -> OptimizeResult:
        """Levenberg–Marquardt with IRLS robust weights; updates values in place."""
        if self._dim == 0:
            return OptimizeResult(0.0, 0.0, 0, True)
        x = self._state()
        yaw = self._yaw_mask()
        cost, r, J = self._linearize(x)
        initial = cost
        lam = lambda0
        converged = False
        it = 0
        for it in range(1, max_iters + 1):  # noqa: B007 - iteration count reported below
            H = (J.T @ J).tocsc()
            g = J.T @ r
            diag = H.diagonal()
            accepted = False
            while lam < 1e12:
                A = H + sp.diags(lam * np.maximum(diag, 1e-9), format="csc")
                try:
                    delta = -_factor(A).solve(g)
                except RuntimeError:
                    lam *= 10.0
                    continue
                if not np.all(np.isfinite(delta)):
                    lam *= 10.0
                    continue
                x_new = x + delta
                x_new[yaw] = _wrap(x_new[yaw])
                new_cost, _, _ = self._linearize(x_new, jacobian=False)
                if new_cost < cost:
                    accepted = True
                    lam = max(lam / 10.0, 1e-12)
                    break
                lam *= 10.0
            if not accepted:
                converged = True  # no descent direction left
                break
            rel = (cost - new_cost) / max(cost, 1e-12)
            step = np.linalg.norm(delta) / max(np.linalg.norm(x), 1.0)
            x = x_new
            cost, r, J = self._linearize(x)
            if rel < rel_tol or step < step_tol:
                converged = True
                break
        self._set_state(x)
        return OptimizeResult(initial, cost, it, converged)

    SCHUR_MAX_DIM = 3000  # dense Schur complement limit (requested dimensions)

    def marginal_covariances(
        self, keys: Iterable[Hashable], method: str = "auto"
    ) -> dict[Hashable, FloatArray]:
        """Marginal covariance blocks at the current estimate (Laplace approximation).

        ``method="schur"`` factors only the block of the *other* variables
        (``H_aa``, typically the pose chain, which factorises with little
        fill-in) and inverts the dense Schur complement of the requested
        block, ``Σ_bb = (H_bb − H_ba H_aa⁻¹ H_ab)⁻¹``. This is much cheaper than
        ``method="direct"`` (one sparse LU of the full H, solved for every
        requested column) when many landmarks are requested, because the
        landmarks cause the fill-in. ``"auto"`` uses Schur when the requested
        block has at most :attr:`SCHUR_MAX_DIM` dimensions.
        """
        keys = list(keys)
        if not keys:
            return {}
        _, _, J = self._linearize(self._state())
        H = (J.T @ J).tocsc()
        H = H + sp.identity(self._dim, format="csc") * 1e-12
        cols: list[int] = []
        spans = []
        for k in keys:
            idx = self._index[k]
            o, d = self._offsets[idx], int(self._types[idx])
            spans.append((len(cols), d))
            cols.extend(range(o, o + d))
        if method == "auto":
            method = "schur" if len(cols) <= self.SCHUR_MAX_DIM else "direct"
        if method == "schur":
            b = np.asarray(cols, dtype=np.int64)
            mask = np.ones(self._dim, dtype=bool)
            mask[b] = False
            a = np.flatnonzero(mask)
            H_bb = H[b][:, b].toarray()
            if len(a):
                H_ab = H[a][:, b]
                X = _factor(H[a][:, a].tocsc()).solve(H_ab.toarray())
                S = H_bb - H_ab.T @ X
            else:
                S = H_bb
            Sigma = np.linalg.inv(0.5 * (S + S.T))
            X = Sigma  # columns/rows in request order
        elif method == "direct":
            lu = _factor(H)
            E = np.zeros((self._dim, len(cols)))
            E[cols, np.arange(len(cols))] = 1.0
            X = lu.solve(E)[cols]
        else:
            raise ValueError(f"unknown method {method!r}")
        out = {}
        for k, (start, d) in zip(keys, spans, strict=True):
            block = X[start : start + d, start : start + d]
            out[k] = 0.5 * (block + block.T)
        return out
