"""Safety bounds — the configurable envelope the gate checks against.

Loaded from ``config/safety.json``. The values there are PLACEHOLDERS (rough /
theoretical) to be replaced with measured ones in Phase 2b; the gate *logic*
(in :mod:`robot_core.safety.gate`) is what this layer delivers now. Pure data,
no I/O beyond reading the JSON.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

DEFAULT_BOUNDS_PATH = (
    Path(__file__).resolve().parent.parent.parent / "config" / "safety.json"
)

# Tiny tolerance so values exactly on a limit are accepted (float-safety).
_EPS = 1e-9


@dataclass(frozen=True)
class CouplingConstraint:
    """A linear J2/J3 half-plane: ``j2_coeff*J2 + j3_coeff*J3 <= max_value``."""

    j2_coeff: float
    j3_coeff: float
    max_value: float
    label: str = ""

    def is_violated(self, j2: float, j3: float) -> bool:
        return self.j2_coeff * j2 + self.j3_coeff * j3 > self.max_value + _EPS


@dataclass(frozen=True)
class SafetyBounds:
    """The allowed workspace annulus, z band, J1 dead-zone, joint ranges, coupling."""

    annulus_inner_mm: float
    annulus_outer_mm: float
    z_min_mm: float
    z_max_mm: float
    j1_rear_dead_zone_deg: float
    joint_ranges_deg: "dict[str, tuple[float, float]]"
    coupling: "tuple[CouplingConstraint, ...]" = ()

    @classmethod
    def from_dict(cls, raw: dict) -> "SafetyBounds":
        ws = raw["workspace"]
        ranges = {
            axis: (float(v[0]), float(v[1]))
            for axis, v in raw["joint_ranges_deg"].items()
            if not axis.startswith("_")
        }
        coupling = tuple(
            CouplingConstraint(
                j2_coeff=float(c["j2_coeff"]),
                j3_coeff=float(c["j3_coeff"]),
                max_value=float(c["max_value"]),
                label=c.get("label", ""),
            )
            for c in raw.get("j2_j3_coupling", [])
        )
        return cls(
            annulus_inner_mm=float(ws["annulus_inner_radius_mm"]),
            annulus_outer_mm=float(ws["annulus_outer_radius_mm"]),
            z_min_mm=float(ws["z_min_mm"]),
            z_max_mm=float(ws["z_max_mm"]),
            j1_rear_dead_zone_deg=float(ws["j1_rear_dead_zone_deg"]),
            joint_ranges_deg=ranges,
            coupling=coupling,
        )

    @classmethod
    def load(cls, path: "str | os.PathLike[str] | None" = None) -> "SafetyBounds":
        bounds_path = Path(path) if path is not None else DEFAULT_BOUNDS_PATH
        with open(bounds_path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))


@lru_cache(maxsize=1)
def default_bounds() -> SafetyBounds:
    """The default bounds, loaded once from :data:`DEFAULT_BOUNDS_PATH` and cached."""
    return SafetyBounds.load()
