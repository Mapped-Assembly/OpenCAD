"""ROS package and OpenIndustries geometry contract regression tests."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import (
    RobotBuilder,
    RobotGeometry,
    RobotVisual,
    export_ros_package,
    import_urdf,
)


def make_robot(tmp_path: Path):
    """Author a robot with independent visual/collision meshes and real frames."""
    mesh = tmp_path / "link.stl"
    mesh.write_text(
        "solid triangle\n"
        "facet normal 0 0 1\n"
        "outer loop\n"
        "vertex 0 0 0\nvertex 10 0 0\nvertex 0 20 0\n"
        "endloop\nendfacet\nendsolid triangle\n", encoding="utf-8",
    )
    return (
        RobotBuilder("DemoRobot")
        .add_link("base", visuals=[
            RobotVisual(
                geometry=RobotGeometry(kind="box", size_mm=(100, 50, 20)),
                material={"name": "blue", "rgba": [0.2, 0.3, 0.9, 1.0]},
            ),
        ])
        .add_link("tip", visuals=[
            RobotVisual(
                origin=RigidTransform(translation_mm=(0, 20, 0)),
                geometry=RobotGeometry(
                    kind="mesh", resolved_path=str(mesh),
                    mesh_vertex_unit_to_mm=1.0,
                ),
                material={"name": "steel", "rgba": [0.4, 0.5, 0.6, 1.0]},
            ),
        ], collisions=[
            RobotVisual(geometry=RobotGeometry(
                kind="mesh", resolved_path=str(mesh), mesh_vertex_unit_to_mm=1.0,
            )),
        ])
        .add_joint(
            "hinge", "base", "tip", joint_type="revolute",
            origin=RigidTransform(
                translation_mm=(0, 0, 100),
                rotation_quaternion_xyzw=(0, 0, math.sqrt(0.5), math.sqrt(0.5)),
            ),
            axis=(0, 0, 1),
            limits={"lower": -1, "upper": 1, "effort": 1, "velocity": 1},
        )
        .build()
    )


def test_ros_package_import_and_oi_handoff(tmp_path: Path) -> None:
    """Package URIs, assets, stable part IDs and geometry frames must survive."""
    robot = make_robot(tmp_path)
    destination = tmp_path / "demo_robot_description"
    result = export_ros_package(robot, destination, package_name="demo_robot_description")
    assert Path(result.urdf_path).exists()
    assert (destination / "package.xml").exists()
    assert (destination / "CMakeLists.txt").exists()
    assert len(result.mesh_paths) == 1
    original_hash = hashlib.sha256((tmp_path / "link.stl").read_bytes()).hexdigest()
    assert Path(result.mesh_paths[0]).name == f"{original_hash}.stl"

    source = Path(result.urdf_path).read_text()
    assert f"package://demo_robot_description/meshes/{original_hash}.stl" in source
    assert str(tmp_path) not in source
    root = ET.fromstring(source)
    for mesh in root.findall(".//mesh"):
        assert mesh.get("filename", "").startswith("package://demo_robot_description/meshes/")
    assert root.find("./joint[@name='hinge']") is not None
    imported = import_urdf(
        result.urdf_path,
        package_roots={"demo_robot_description": destination},
    ).robot
    assert set(imported.links) == {"base", "tip"}
    assert imported.joints["hinge"].parent == "base"
    assert imported.joints["hinge"].child == "tip"
    assert imported.links["tip"].visuals[0].geometry.resolved_path == result.mesh_paths[0]
    assert imported.links["tip"].collisions[0].geometry.resolved_path == result.mesh_paths[0]

    handoff_text = Path(result.manifest_path).read_text()
    assert str(tmp_path) not in handoff_text
    handoff = json.loads(handoff_text)
    assert handoff["schema"] == "opencad.robot_handoff.v1"
    assert handoff["version"] == 1
    assert handoff["root_link"] == "base"
    assert handoff["source"]["ros_package"] == "demo_robot_description"
    assert handoff["source"]["urdf_sha256"] == hashlib.sha256(source.encode()).hexdigest()
    assert handoff["mesh_sha256"][f"{original_hash}.stl"] == original_hash
    assert handoff["links"]["tip"]["parent_joint"] == "hinge"
    assert handoff["links"]["base"]["children"] == ["tip"]
    assert handoff["links"]["tip"]["visuals"]["urdf:tip:visual:0"]["material"]["name"] == "steel"
    assert handoff["links"]["tip"]["collisions"]["urdf:tip:collision:0"]["geometry"]["sha256"] == original_hash
    assert handoff["joints"]["hinge"]["origin"]["translation_mm"] == [0, 0, 100]
    assert handoff["verification"]["dynamics_verified"] is False
    assert handoff["verification"]["hardware_validated"] is False
    assert handoff["verification"]["physical_validation"] == "not_performed"

    asset_text = Path(result.oi_asset_path).read_text()
    assert str(tmp_path) not in asset_text
    bundle = json.loads(asset_text)
    assert bundle["schemaVersion"] == 1
    asset = bundle["asset"]
    assert asset["schemaVersion"] == 1
    assert asset["source"]["kind"] == "generated"
    assert asset["source"]["generator"] == "opencad-robot"
    assert asset["source"]["digest"] == handoff["source"]["urdf_sha256"]
    assert asset["units"] == "m" and asset["upAxis"] == "Y"
    assert asset["hierarchy"]["id"] == "urdf:base"
    assert asset["hierarchy"]["children"][0]["id"] == "urdf:tip"
    assert asset["hierarchy"]["children"][0]["partIds"] == ["urdf:tip:visual:0"]
    part_ids = {part["id"] for part in asset["parts"]}
    assert part_ids == {"urdf:base:visual:0", "urdf:tip:visual:0"}
    tip = next(part for part in asset["parts"] if part["id"] == "urdf:tip:visual:0")
    assert tip["metadata"]["mesh_sha256"] == original_hash
    assert tip["metadata"]["kinematic_status"] == "neutral_unvalidated"
    assert tip["color"] == [0.4, 0.5, 0.6]
    # First STL vertex + local Y20 mm, then joint +90 deg Z and Z100 mm,
    # transformed from OpenCAD Z-up (x,y,z) to OI Y-up (x,z,-y).
    center = asset["originOffset"]
    first = [tip["vertices"][i] + center[i] for i in range(3)]
    assert first == pytest.approx((-0.02, 0.1, 0.0), abs=1e-9)
    assert all(float(v) >= 0 for v in asset["dimensions"])
    assert handoff["openindustries"]["asset_id"] == asset["id"]
    assert handoff["openindustries"]["status"] == "static_neutral_geometry_only"


def test_ros_package_is_reproducible_across_local_folders(tmp_path: Path) -> None:
    """Identical robot emits identical XML, OI asset and path-independent sidecar."""
    robot = make_robot(tmp_path)
    a = export_ros_package(robot, tmp_path / "a", package_name="same_robot")
    b = export_ros_package(robot, tmp_path / "b", package_name="same_robot")
    for relative in ("urdf/DemoRobot.urdf", "oi.asset.json", "robot.handoff.json"):
        assert (Path(a.package_directory) / relative).read_bytes() == (
            Path(b.package_directory) / relative
        ).read_bytes()


def test_failed_or_unsafe_export_does_not_publish_partial_package(tmp_path: Path) -> None:
    """Reject overwrite, unsafe package names, and missing local mesh geometry."""
    robot = make_robot(tmp_path)
    destination = tmp_path / "ok_package"
    export_ros_package(robot, destination, package_name="ok_package")
    with pytest.raises(FileExistsError, match="already exists"):
        export_ros_package(robot, destination, package_name="ok_package")
    with pytest.raises(ValueError, match="snake_case"):
        export_ros_package(robot, tmp_path / "not_created", package_name="../escape")
    assert not (tmp_path / "not_created").exists()
    (tmp_path / "link.stl").unlink()
    with pytest.raises(ValueError, match="existing local STL"):
        export_ros_package(robot, tmp_path / "broken", package_name="broken")
    assert not (tmp_path / "broken").exists()


def test_metadata_only_ros_export_is_explicitly_opt_in(tmp_path: Path) -> None:
    """Pure URDF packages may omit OI geometry when no renderable model exists."""
    robot = RobotBuilder("bare").add_link("base").build()
    result = export_ros_package(robot, tmp_path / "bare", include_oi=False)
    assert result.oi_asset_path == ""
    assert not (tmp_path / "bare" / "oi.asset.json").exists()
    manifest = json.loads(Path(result.manifest_path).read_text())
    assert manifest["openindustries"]["asset_id"] is None
    assert Path(result.urdf_path).exists()


def test_cli_uses_public_ros_export_contract(tmp_path: Path) -> None:
    """The CLI must accept a typed OpenCAD robot document and produce the package."""
    from opencad.cli import main

    robot = make_robot(tmp_path)
    serialized = tmp_path / "robot.opencad.json"
    serialized.write_text(robot.model_dump_json(), encoding="utf-8")
    output = tmp_path / "from_cli"
    assert main(["export", "ros-package", str(serialized),
                 "--output", str(output), "--package-name", "cli_bot"]) == 0
    assert (output / "urdf" / "DemoRobot.urdf").exists()
    assert json.loads((output / "oi.asset.json").read_text())["asset"]["source"]["generator"] == "opencad-robot"


def test_quaddle_style_unresolved_state_is_not_inferred_as_solved(tmp_path: Path) -> None:
    """A closing link must stay flagged as potential loop closure, not simulated."""
    robot = (RobotBuilder("closing_test")
             .add_link("base", visuals=[RobotVisual(
                 geometry=RobotGeometry(kind="box", size_mm=(10, 10, 10)))])
             .add_link("closing_frame")
             .add_joint("close", "base", "closing_frame", joint_type="fixed")
             .build())
    result = export_ros_package(robot, tmp_path / "closing")
    sidecar = json.loads(Path(result.manifest_path).read_text())
    assert sidecar["verification"]["constraints_unresolved"] is True
    assert sidecar["verification"]["hardware_validated"] is False
