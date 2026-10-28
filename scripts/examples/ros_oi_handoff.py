"""Runnable OpenCAD -> ROS2 URDF package -> OpenIndustries asset example.

    uv run --package opencad python scripts/examples/ros_oi_handoff.py /tmp/bench_robot

This creates portable ROS package files and oi.asset.json with static neutral
geometry. It does not simulate or validate hardware motion.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import (
    RobotBuilder,
    RobotGeometry,
    RobotVisual,
    export_ros_package,
    import_urdf,
)


def run(output: Path) -> None:
    """Author a small mechanically descriptive two-link robot and export it."""
    # Native OpenCAD units are millimeters; URDF prismatic travel is meters.
    builder = RobotBuilder("bench_robot")
    builder.add_link(
        "base",
        visuals=[
            RobotVisual(geometry=RobotGeometry(kind="box", size_mm=(100, 80, 30)),
                        material={"name": "base_gray", "rgba": [0.6, 0.6, 0.6, 1]}),
        ],
    )
    builder.add_link(
        "slider",
        visuals=[
            RobotVisual(geometry=RobotGeometry(kind="cylinder", radius_mm=10, length_mm=120),
                        material={"name": "slider_blue", "rgba": [0.2, 0.4, 0.9, 1]}),
        ],
    )
    builder.add_joint(
        "linear_axis", "base", "slider", joint_type="prismatic",
        origin=RigidTransform(translation_mm=(0, 0, 30)), axis=(0, 0, 1),
        limits={"lower": 0, "upper": 0.05, "effort": 20, "velocity": 0.1},
    )
    robot = builder.build()
    exported = export_ros_package(
        robot, output, package_name="bench_robot_description"
    )
    # Explicit package mapping resolves package:// without author-specific paths.
    imported = import_urdf(
        exported.urdf_path,
        package_roots={"bench_robot_description": output},
    ).robot
    assert imported.root_link == "base"
    assert set(imported.links) == {"base", "slider"}
    artifact = json.loads(Path(exported.oi_asset_path).read_text())
    assert artifact["asset"]["source"]["generator"] == "opencad-robot"
    assert artifact["asset"]["hierarchy"]["children"][0]["id"] == "urdf:slider"
    sidecar = json.loads(Path(exported.manifest_path).read_text())
    assert sidecar["verification"]["physical_validation"] == "not_performed"
    print("ROS package:", exported.package_directory)
    print("URDF:", exported.urdf_path)
    print("OpenIndustries asset:", exported.oi_asset_path)
    print("Handoff sidecar:", exported.manifest_path)
    print("Physical validation: NOT PERFORMED")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path, help="New ROS package directory")
    arguments = parser.parse_args()
    run(arguments.output)
