"""Reproducible non-singular four-bar solver benchmark with independent reference.

The independent reference is the law of cosines at the grounded rocker pivot,
not the library's circle-intersection solver. This tests position closure,
not passive dynamics, spring forces, hardware controls, or robot safety.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

from opencad.robotics import PlanarFourBar, solve_four_bar_sweep

TOLERANCE_MM = 1e-7
REFERENCE_TOLERANCE_MM = 1e-8
SAMPLE_COUNT = 181


def independent_rocker_endpoint(
    model: PlanarFourBar, input_angle: float
) -> tuple[float, float]:
    """Compute endpoint by cosine law at D, independent from circle intersection."""
    bx = model.crank_mm * math.cos(input_angle)
    by = model.crank_mm * math.sin(input_angle)
    d = math.hypot(bx - model.ground_mm, by)
    gamma = math.acos(
        (d*d + model.rocker_mm**2 - model.coupler_mm**2)
        / (2.0 * d * model.rocker_mm)
    )
    beta = math.atan2(by, bx - model.ground_mm)
    solution_angle = beta - gamma if model.assembly_branch == "upper" else beta + gamma
    return (
        model.ground_mm + model.rocker_mm * math.cos(solution_angle),
        model.rocker_mm * math.sin(solution_angle),
    )


def run_benchmark() -> dict[str, Any]:
    """Verify both assembly branches across a full revolution, recording residuals."""
    models = [
        PlanarFourBar(
            id=f"bench-{branch}", driver_joint="driver", ground_mm=100,
            crank_mm=40, coupler_mm=90, rocker_mm=70, assembly_branch=branch,
            provenance="dimensioned synthetic crank-rocker benchmark (not Quaddle)",
        )
        for branch in ("upper", "lower")
    ]
    angles = [2 * math.pi * i / (SAMPLE_COUNT - 1) for i in range(SAMPLE_COUNT)]
    results: list[dict[str, Any]] = []
    for model in models:
        solutions = solve_four_bar_sweep(model, angles, tolerance_mm=TOLERANCE_MM)
        max_residual = max(item.residual_mm for item in solutions)
        max_independent_error = max(
            math.dist(item.coupler_rocker_endpoint_mm, independent_rocker_endpoint(model, item.input_angle_rad))
            for item in solutions
        )
        if max_residual > TOLERANCE_MM or max_independent_error > REFERENCE_TOLERANCE_MM:
            raise AssertionError(
                f"{model.id}: closure residual {max_residual} mm; "
                f"independent position error {max_independent_error} mm"
            )
        if math.dist(solutions[0].coupler_rocker_endpoint_mm, solutions[-1].coupler_rocker_endpoint_mm) > REFERENCE_TOLERANCE_MM:
            raise AssertionError(f"{model.id}: full-turn endpoint drift")
        results.append({
            "model": model.model_dump(mode="json"),
            "sample_count": len(solutions),
            "max_closure_residual_mm": max_residual,
            "max_law_of_cosines_disagreement_mm": max_independent_error,
            "status": "pass",
        })
    return {
        "schema_version": 1,
        "solver": "opencad.planar_four_bar.circle_intersection",
        "reference": "independent_law_of_cosines",
        "tolerances_mm": {"loop_closure": TOLERANCE_MM, "reference": REFERENCE_TOLERANCE_MM},
        "verification": {
            "position_constraints": "pass",
            "physically_validated": False,
            "closed_loop_kinematics_supported": "only declared planar four-bar positions",
            "spring_dynamics": "not_performed",
            "actuation": "not_performed",
            "quaddle_hardware": "not_performed",
        },
        "branches": results,
        "geometry_spec_sha256": hashlib.sha256(
            json.dumps([model.model_dump(mode="json") for model in models], sort_keys=True).encode()
        ).hexdigest(),
    }


def main() -> None:
    """Fail the job on a disagreement; publish an evidence JSON only on success."""
    report = run_benchmark()
    output = Path(os.environ.get("OPENCAD_FOUR_BAR_REPORT", "reports/four-bar-verification.json"))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(f"Planar four-bar positional benchmark passed: {output}")
    for branch in report["branches"]:
        print(
            f"{branch['model']['assembly_branch']}: {branch['sample_count']} poses, "
            f"max loop residual={branch['max_closure_residual_mm']:.3g} mm, "
            f"independent error={branch['max_law_of_cosines_disagreement_mm']:.3g} mm"
        )
    print("Physical/dynamic validation: NOT PERFORMED.")


if __name__ == "__main__":
    main()
