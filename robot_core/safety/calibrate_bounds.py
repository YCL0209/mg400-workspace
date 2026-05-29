"""Offline calibration tool to derive safety bounds from measured limit points.

This script processes limit observations collected by probe_limits.py to derive
the actual safety boundaries of the robot. It fits J2/J3 coupling constraints,
computes workspace limits via forward kinematics, and outputs configuration
ready to paste into config/safety.json.

Pure computation — no hardware connection, no socket I/O. Reuses the kinematics
layer for FK calculations.

Usage::

    python -m robot_core.safety.calibrate_bounds outputs/limits_*.json
    python -m robot_core.safety.calibrate_bounds outputs/limits_20240101_120000.json --inner-margin 20

The output is a complete bounds configuration with updated provenance that can
replace the placeholder values in config/safety.json.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from robot_core.kinematics import KinematicsConfig, forward_kinematics
from robot_core.safety.bounds import CouplingConstraint

# Default safety margins (mm)
DEFAULT_INNER_MARGIN_MM = 20.0  # Buffer inside the singularity column
DEFAULT_OUTER_MARGIN_MM = 10.0  # Buffer from maximum reach
DEFAULT_Z_MARGIN_MM = 5.0       # Buffer from z limits


@dataclass
class CalibrationResult:
    """The derived safety bounds with metadata."""
    
    annulus_inner_mm: float
    annulus_outer_mm: float
    z_min_mm: float
    z_max_mm: float
    j1_rear_dead_zone_deg: float
    joint_ranges_deg: dict[str, tuple[float, float]]
    coupling_constraints: list[CouplingConstraint]
    provenance: str
    source_file: str
    point_count: int
    
    def to_safety_json(self) -> dict:
        """Format as config/safety.json structure."""
        return {
            "provenance": self.provenance,
            "workspace": {
                "annulus_inner_radius_mm": self.annulus_inner_mm,
                "annulus_outer_radius_mm": self.annulus_outer_mm,
                "z_min_mm": self.z_min_mm,
                "z_max_mm": self.z_max_mm,
                "j1_rear_dead_zone_deg": self.j1_rear_dead_zone_deg,
            },
            "joint_ranges_deg": self.joint_ranges_deg,
            "j2_j3_coupling": [
                {
                    "j2_coeff": c.j2_coeff,
                    "j3_coeff": c.j3_coeff,
                    "max_value": c.max_value,
                    "label": c.label,
                }
                for c in self.coupling_constraints
            ],
            "coupling_note": f"Fitted from {self.point_count} measured limit points. "
                           f"Linear half-plane constraints derived from observed J2/J3 coupling boundaries.",
        }


def load_limit_points(path: Path) -> list[dict]:
    """Load limit points from a probe_limits output file."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return data["points"]


def derive_joint_ranges(points: list[dict]) -> dict[str, tuple[float, float]]:
    """Extract min/max for each joint axis from the observations."""
    if not points:
        # Return default ranges if no points
        return {
            "J1": (-160.0, 160.0),
            "J2": (-25.0, 85.0),
            "J3": (-25.0, 105.0),
            "J4": (-180.0, 180.0),
        }
    
    # Find observed extremes
    j1_values = [p["j1"] for p in points]
    j2_values = [p["j2"] for p in points]
    j3_values = [p["j3"] for p in points]
    j4_values = [p["j4"] for p in points]
    
    return {
        "J1": (min(j1_values), max(j1_values)),
        "J2": (min(j2_values), max(j2_values)),
        "J3": (min(j3_values), max(j3_values)),
        "J4": (min(j4_values), max(j4_values)),
    }


def fit_j2_j3_coupling(points: list[dict]) -> list[CouplingConstraint]:
    """Fit linear half-plane constraints for J2/J3 coupling.
    
    The coupling manifests as reduced J3 range when J2 is extended. We fit
    constraints of the form: a*J2 + b*J3 <= c
    
    For simplicity, we identify key boundary patterns:
    1. Upper J3 limit decreases with J2 (positive J2 coefficient)
    2. Lower J3 limit may increase with J2 (negative J2 coefficient)
    """
    constraints = []
    
    # Filter points that seem to be at J2/J3 limits
    # (where label suggests coupling or where J2 and J3 are both non-zero and significant)
    coupling_points = [
        p for p in points
        if ("j2" in p.get("label", "").lower() and "j3" in p.get("label", "").lower())
        or ("coupl" in p.get("label", "").lower())
        or (abs(p["j2"]) > 30 and abs(p["j3"]) > 30)  # Both joints significantly displaced
    ]
    
    if len(coupling_points) >= 2:
        # Fit upper constraint (J3 decreases as J2 increases)
        # Find points that seem to be at upper J3 boundary
        upper_points = [p for p in coupling_points if p["j3"] > 50]
        if len(upper_points) >= 2:
            # Simple linear fit: find the line through extreme points
            # For upper bound: maximize J2 + J3 weighted
            j2_max = max(p["j2"] for p in upper_points)
            j3_at_j2_max = max(p["j3"] for p in upper_points if abs(p["j2"] - j2_max) < 5)
            
            j2_min = min(p["j2"] for p in upper_points)
            j3_at_j2_min = max(p["j3"] for p in upper_points if abs(p["j2"] - j2_min) < 5)
            
            if j2_max - j2_min > 10:  # Meaningful range
                # Fit line: J3 = m*J2 + b, convert to a*J2 + b*J3 <= c
                # Two points: (j2_min, j3_at_j2_min), (j2_max, j3_at_j2_max)
                if j3_at_j2_min > j3_at_j2_max:  # J3 decreases with J2
                    # Approximate the constraint
                    # We want: J2/j2_range + J3/j3_range <= 1 (normalized)
                    # Or: J3 <= j3_max - k*(J2 - j2_0)
                    slope = (j3_at_j2_max - j3_at_j2_min) / (j2_max - j2_min)
                    # Convert to half-plane: -slope*J2 + J3 <= intercept
                    intercept = j3_at_j2_min - slope * j2_min
                    constraints.append(
                        CouplingConstraint(
                            j2_coeff=-slope,
                            j3_coeff=1.0,
                            max_value=intercept,
                            label="J3_upper_coupling"
                        )
                    )
    
    # If no coupling found, return empty list (only per-axis limits apply)
    return constraints


def compute_workspace_limits(
    points: list[dict],
    kinematics_config: Optional[KinematicsConfig] = None,
    inner_margin: float = DEFAULT_INNER_MARGIN_MM,
    outer_margin: float = DEFAULT_OUTER_MARGIN_MM,
    z_margin: float = DEFAULT_Z_MARGIN_MM,
) -> tuple[float, float, float, float]:
    """Derive workspace annulus and z limits using FK on observed joints.
    
    Returns: (inner_radius, outer_radius, z_min, z_max)
    """
    if not points:
        # Return conservative defaults
        return 150.0, 420.0, -160.0, 180.0
    
    # Use FK to compute reachable positions
    radii = []
    z_values = []
    
    for point in points:
        x, y, z, _ = forward_kinematics(
            point["j1"], point["j2"], point["j3"], point["j4"],
            config=kinematics_config
        )
        radius = math.hypot(x, y)
        radii.append(radius)
        z_values.append(z)
    
    # Also scan a grid of joint configurations for better coverage
    joint_ranges = derive_joint_ranges(points)
    j1_min, j1_max = joint_ranges["J1"]
    j2_min, j2_max = joint_ranges["J2"]
    j3_min, j3_max = joint_ranges["J3"]
    
    # Sample joint space (coarse grid)
    for j1 in [j1_min, 0, j1_max]:
        for j2 in [j2_min, 0, j2_max]:
            for j3 in [j3_min, 0, j3_max]:
                x, y, z, _ = forward_kinematics(j1, j2, j3, 0, config=kinematics_config)
                radius = math.hypot(x, y)
                radii.append(radius)
                z_values.append(z)
    
    # Determine limits with safety margins
    inner_radius = min(radii) + inner_margin
    outer_radius = max(radii) - outer_margin
    z_min = min(z_values) + z_margin
    z_max = max(z_values) - z_margin
    
    # Sanity check
    inner_radius = max(inner_radius, 100.0)  # Minimum safe distance from center
    outer_radius = min(outer_radius, 440.0)  # Maximum spec'd reach
    
    return inner_radius, outer_radius, z_min, z_max


def derive_j1_dead_zone(points: list[dict]) -> float:
    """Compute J1 rear dead zone from observations.
    
    The dead zone is typically around +/-20 degrees from the rear (180°).
    """
    # Look for points labeled as J1 rear limits or with J1 near ±180
    rear_points = [
        p for p in points
        if "j1" in p.get("label", "").lower() and "rear" in p.get("label", "").lower()
        or abs(abs(p["j1"]) - 180) < 45  # Within 45° of rear
    ]
    
    if rear_points:
        # Find the closest approach to ±180°
        j1_values = [p["j1"] for p in rear_points]
        min_angle_from_rear = min(180 - abs(j1) for j1 in j1_values)
        # Dead zone is twice this distance (symmetric)
        dead_zone = 2 * min_angle_from_rear
        return min(dead_zone, 40.0)  # Cap at known maximum
    
    # Default conservative value
    return 40.0


def calibrate_from_file(
    limits_file: Path,
    inner_margin: float = DEFAULT_INNER_MARGIN_MM,
    outer_margin: float = DEFAULT_OUTER_MARGIN_MM,
    z_margin: float = DEFAULT_Z_MARGIN_MM,
) -> CalibrationResult:
    """Main calibration routine."""
    points = load_limit_points(limits_file)
    
    if not points:
        print("Warning: No limit points found in file")
    
    # Derive parameters
    joint_ranges = derive_joint_ranges(points)
    coupling_constraints = fit_j2_j3_coupling(points)
    inner_r, outer_r, z_min, z_max = compute_workspace_limits(
        points, inner_margin=inner_margin, outer_margin=outer_margin, z_margin=z_margin
    )
    j1_dead_zone = derive_j1_dead_zone(points)
    
    # Build result
    return CalibrationResult(
        annulus_inner_mm=inner_r,
        annulus_outer_mm=outer_r,
        z_min_mm=z_min,
        z_max_mm=z_max,
        j1_rear_dead_zone_deg=j1_dead_zone,
        joint_ranges_deg=joint_ranges,
        coupling_constraints=coupling_constraints,
        provenance=f"Empirically calibrated from {len(points)} measured limit points "
                   f"via probe_limits.py on {datetime.now().strftime('%Y-%m-%d')}. "
                   f"Workspace derived via FK scan with safety margins "
                   f"(inner={inner_margin}mm, outer={outer_margin}mm, z={z_margin}mm).",
        source_file=limits_file.name,
        point_count=len(points),
    )


def main():
    """CLI entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Derive safety bounds from measured limit points"
    )
    parser.add_argument(
        "limits_file",
        type=Path,
        help="Path to limits_*.json file from probe_limits.py",
    )
    parser.add_argument(
        "--inner-margin",
        type=float,
        default=DEFAULT_INNER_MARGIN_MM,
        help=f"Safety margin inside singularity (mm, default={DEFAULT_INNER_MARGIN_MM})",
    )
    parser.add_argument(
        "--outer-margin",
        type=float,
        default=DEFAULT_OUTER_MARGIN_MM,
        help=f"Safety margin from max reach (mm, default={DEFAULT_OUTER_MARGIN_MM})",
    )
    parser.add_argument(
        "--z-margin",
        type=float,
        default=DEFAULT_Z_MARGIN_MM,
        help=f"Safety margin from z limits (mm, default={DEFAULT_Z_MARGIN_MM})",
    )
    
    args = parser.parse_args()
    
    if not args.limits_file.exists():
        print(f"Error: File not found: {args.limits_file}")
        sys.exit(1)
    
    print(f"Calibrating from {args.limits_file}...")
    result = calibrate_from_file(
        args.limits_file,
        inner_margin=args.inner_margin,
        outer_margin=args.outer_margin,
        z_margin=args.z_margin,
    )
    
    # Display results
    print(f"\n=== Calibration Results ({result.point_count} points) ===")
    print(f"Workspace annulus: {result.annulus_inner_mm:.1f} - {result.annulus_outer_mm:.1f} mm")
    print(f"Z limits: {result.z_min_mm:.1f} - {result.z_max_mm:.1f} mm")
    print(f"J1 rear dead zone: ±{result.j1_rear_dead_zone_deg/2:.1f}° from 180°")
    
    print("\nJoint ranges (deg):")
    for axis, (low, high) in result.joint_ranges_deg.items():
        print(f"  {axis}: [{low:.1f}, {high:.1f}]")
    
    if result.coupling_constraints:
        print(f"\nJ2/J3 coupling constraints ({len(result.coupling_constraints)}):")
        for c in result.coupling_constraints:
            print(f"  {c.label}: {c.j2_coeff:.3f}*J2 + {c.j3_coeff:.3f}*J3 <= {c.max_value:.1f}")
    else:
        print("\nNo J2/J3 coupling constraints fitted")
    
    # Output JSON config
    output = result.to_safety_json()
    print("\n=== config/safety.json format ===")
    print(json.dumps(output, indent=2))
    
    # Save to file
    output_path = Path("outputs") / f"calibrated_bounds_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    output_path.parent.mkdir(exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(f"\n已存到 {output_path}")
    print("可直接複製內容到 config/safety.json")


if __name__ == "__main__":
    main()