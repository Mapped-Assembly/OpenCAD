"""End-to-end and safety checks for native OpenCAD robot -> URDF generation."""

from __future__ import annotations

import hashlib
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import (
    RobotBuilder,
    RobotDescription,
    RobotGeometry,
    RobotInertial,
    RobotVisual,
    URDFImportResult,
    export_urdf,
    import_urdf,
)


def _pose() -> RigidTransform:
    """Return a genuinely nontrivial yaw/roll frame in OpenCAD mm."""
    a = math.pi / 6
    b = math.pi / 4
    return RigidTransform(
        translation_mm=(125.0, -67.5, 12.25),
        rotation_quaternion_xyzw=(
            math.sin(a / 2) * math.cos(b / 2),
            math.sin(a / 2) * math.sin(b / 2),
            math.cos(a / 2) * math.sin(b / 2),
            math.cos(a / 2) * math.cos(b / 2),
        ),
    )


def _robot() -> RobotDescription:
    """Create a native robot without importing or authoring XML manually."""
    inertial = RobotInertial(
        origin=RigidTransform.identity(),
        mass_kg=0.4,
        inertia_kg_m2={
            "ixx": 0.003, "ixy": 0.0, "ixz": 0.0,
            "iyy": 0.004, "iyz": 0.0, "izz": 0.005,
        },
    )
    return (
        RobotBuilder("demo_robot", metadata={"designer": "OpenCAD"})
        .add_link("base", visuals=[
            RobotVisual(geometry=RobotGeometry(kind="box", size_mm=(20.0, 30.0, 40.0))),
        ], inertial=inertial)
        .add_link("end_effector", visuals=[
            RobotVisual(
                name="tool",
                origin=_pose(),
                geometry=RobotGeometry(kind="cylinder", radius_mm=5.0, length_mm=25.0),
                material={"name": "red", "rgba": [1.0, 0.0, 0.0, 1.0]},
            )
        ], collisions=[
            RobotVisual(geometry=RobotGeometry(kind="sphere", radius_mm=6.5))
        ])
        .add_joint(
            "wrist", "base", "end_effector", joint_type="revolute",
            origin=_pose(), axis=(0.0, 0.0, 1.0),
            limits={"lower": -1.2, "upper": 1.4, "effort": 3.0, "velocity": 2.0},
        )
        .build()
    )


def _same_rotation(a: RigidTransform, b: RigidTransform) -> bool:
    """Compare quaternion frame orientation up to equivalent opposite signs."""
    return abs(sum(x * y for x, y in zip(a.rotation_quaternion_xyzw, b.rotation_quaternion_xyzw))) == pytest.approx(1.0, abs=1e-10)


def test_native_robot_round_trip(tmp_path: Path) -> None:
    """New OpenCAD models survive a URDF export and re-import without unit drift."""
    robot = _robot()
    artifact = export_urdf(robot, tmp_path / "robot.urdf")
    assert Path(artifact.urdf_path).is_file()
    assert Path(artifact.manifest_path).is_file()
    assert not artifact.mesh_paths
    root = ET.parse(artifact.urdf_path).getroot()
    assert root.attrib["name"] == "demo_robot"
    assert len(root.findall("link")) == 2
    assert len(root.findall("joint")) == 1
    joint = root.find("./joint[@name='wrist']")
    assert joint is not None
    assert float(joint.find("origin").get("xyz").split()[0]) == pytest.approx(0.125)
    imported = import_urdf(artifact.urdf_path).robot
    assert imported.root_link == "base"
    assert imported.joints["wrist"].limits == robot.joints["wrist"].limits
    assert imported.links["base"].visuals[0].geometry.size_mm == pytest.approx((20, 30, 40))
    assert imported.links["end_effector"].visuals[0].geometry.radius_mm == pytest.approx(5)
    assert imported.links["end_effector"].collisions[0].geometry.radius_mm == pytest.approx(6.5)
    assert imported.links["base"].inertial.inertia_kg_m2 == robot.links["base"].inertial.inertia_kg_m2
    assert imported.joints["wrist"].origin.translation_mm == pytest.approx(_pose().translation_mm)
    assert _same_rotation(imported.joints["wrist"].origin, _pose())
    assert _same_rotation(imported.links["end_effector"].visuals[0].origin, _pose())


def test_deterministic_export(tmp_path: Path) -> None:
    """Equivalent models produce byte-for-byte reproducible XML and sidecars."""
    robot = _robot()
    first = export_urdf(robot, tmp_path / "first" / "robot.urdf")
    second = export_urdf(robot, tmp_path / "second" / "robot.urdf")
    assert Path(first.urdf_path).read_bytes() == Path(second.urdf_path).read_bytes()
    assert Path(first.manifest_path).read_bytes() == Path(second.manifest_path).read_bytes()


def test_mesh_bundle_and_mm_scale(tmp_path: Path) -> None:
    """Source STL coordinates in native mm are scaled to URDF meter coordinates."""
    source = tmp_path / "source.stl"
    source.write_text("solid demo\nendsolid demo\n", encoding="utf-8")
    mesh = RobotGeometry(kind="mesh", resolved_path=str(source), mesh_vertex_unit_to_mm=1.0)
    builder = RobotBuilder("stl_bot")
    builder.add_link("base", visuals=[RobotVisual(geometry=mesh)])
    builder.add_link("foot", visuals=[RobotVisual(geometry=mesh)],
                     collisions=[RobotVisual(geometry=mesh)])
    builder.add_joint("slide", "base", "foot", joint_type="prismatic",
                      limits={"lower": 0.0, "upper": 0.03, "effort": 5, "velocity": 0.2})
    robot = builder.build()
    out = export_urdf(robot, tmp_path / "model.urdf")
    assert len(out.mesh_paths) == 1  # deduplicated by content hash
    assert Path(out.mesh_paths[0]).read_bytes() == source.read_bytes()
    assert hashlib.sha256(source.read_bytes()).hexdigest() in out.mesh_paths[0]
    root = ET.parse(out.urdf_path).getroot()
    meshes = root.findall(".//mesh")
    assert len(meshes) == 3
    for element in meshes:
        assert element.get("scale") == "0.001 0.001 0.001"
        assert element.get("filename").startswith("meshes/")
    imported = import_urdf(out.urdf_path).robot
    assert imported.joints["slide"].limits["upper"] == 0.03
    # Importer assumes URDF mesh coordinates in meters; scale restores native mm.
    assert imported.links["foot"].visuals[0].geometry.mesh_scale == (0.001,) * 3


def test_export_normalizes_joint_axis_for_independent_prismatic_fk(tmp_path: Path) -> None:
    """Non-unit joint axes must not silently change prismatic displacement."""
    robot = (
        RobotBuilder("axis_robot")
        .add_link("root")
        .add_link("slider")
        .add_joint(
            "travel", "root", "slider", joint_type="prismatic",
            axis=(0.3, 0, 0.8),
            limits={"lower": 0.0, "upper": 0.1, "effort": 1.0, "velocity": 1.0},
        )
        .build()
    )
    export = export_urdf(robot, tmp_path / "axis_robot.urdf")
    joint = ET.parse(export.urdf_path).getroot().find("./joint[@name='travel']")
    assert joint is not None
    axis = tuple(float(v) for v in joint.find("axis").get("xyz").split())
    assert math.sqrt(sum(v * v for v in axis)) == pytest.approx(1.0, abs=1e-12)
    imported = import_urdf(export.urdf_path).robot
    assert imported.joints["travel"].axis == pytest.approx(axis)


def test_continuous_joint_has_no_fabricated_bounds(tmp_path: Path) -> None:
    """Unlimited joints keep semantics; extraneous imported bounds are diagnosed."""
    robot = (RobotBuilder("rotor").add_link("body").add_link("wheel")
             .add_joint("spin", "body", "wheel", joint_type="continuous",
                        limits={"lower": -3, "upper": 3, "effort": 1.5, "velocity": 10})
             .build())
    out = export_urdf(robot, tmp_path / "rotor.urdf")
    element = ET.parse(out.urdf_path).getroot().find("./joint/limit")
    assert element is not None
    assert "lower" not in element.attrib and "upper" not in element.attrib
    assert any("bounds omitted" in warning for warning in out.warnings)
    parsed = import_urdf(out.urdf_path).robot
    assert parsed.joints["spin"].joint_type == "continuous"
    assert parsed.joints["spin"].limits == {"effort": 1.5, "velocity": 10.0}


@pytest.mark.parametrize("invalid", [
    lambda b: b.add_link("bad name"),
    lambda b: b.add_link("bad<name"),
    lambda b: b.add_link("bad&name"),
])
def test_rejects_invalid_ros_names(tmp_path: Path, invalid) -> None:
    """Refuse invalid ROS names before writing the destination file."""
    builder = RobotBuilder("test")
    invalid(builder)
    with pytest.raises(ValueError, match="Invalid ROS"):
        export_urdf(builder.build(), tmp_path / "should-not-exist.urdf")
    assert not (tmp_path / "should-not-exist.urdf").exists()


def test_material_without_name_gets_stable_identifier(tmp_path: Path) -> None:
    """Inline RGBA uses an explicit deterministic material name for ROS."""
    robot = (RobotBuilder("colors").add_link(
        "body", visuals=[RobotVisual(
            geometry=RobotGeometry(kind="sphere", radius_mm=5),
            material={"rgba": [0.2, 0.3, 0.4, 1.0]},
        )],
    ).build())
    out = export_urdf(robot, tmp_path / "colors.urdf")
    material = ET.parse(out.urdf_path).getroot().find("./link/visual/material")
    assert material is not None
    assert material.get("name", "").startswith("opencad_color_")
    assert [float(v) for v in material.find("color").get("rgba").split()] == pytest.approx((0.2, 0.3, 0.4, 1.0))
    assert import_urdf(out.urdf_path).robot.links["body"].visuals[0].material["rgba"] == pytest.approx((0.2, 0.3, 0.4, 1.0))


def test_rejects_unsafe_and_unresolved_meshes(tmp_path: Path) -> None:
    """Do not publish broken URDF references or silently fetch assets."""
    robot = (RobotBuilder("broken").add_link("base", visuals=[
        RobotVisual(geometry=RobotGeometry(kind="mesh", resolved_path=str(tmp_path / "missing.stl")))
    ]).build())
    with pytest.raises(ValueError, match="existing local STL"):
        export_urdf(robot, tmp_path / "broken.urdf")
    assert not (tmp_path / "broken.urdf").exists()
    with pytest.raises(ValueError, match="meshes_dir"):
        export_urdf(_robot(), tmp_path / "out.urdf", meshes_dir="../outside")


def test_invalid_graph_and_required_joint_limits_fail(tmp_path: Path) -> None:
    """Prevent broken robot exports rather than inventing valid-looking joints."""
    with pytest.raises(ValueError, match="exactly one root"):
        RobotBuilder("disconnected").add_link("one").add_link("two").build()
    robot = (RobotBuilder("bad_limits").add_link("base").add_link("tip")
             .add_joint("bad", "base", "tip", joint_type="revolute").build())
    with pytest.raises(ValueError, match="mandatory URDF limit"):
        export_urdf(robot, tmp_path / "bad.urdf")
    assert not (tmp_path / "bad.urdf").exists()


def test_cli_export_accepts_import_envelope(tmp_path: Path) -> None:
    """Issue #2 import documents can be used directly in the export CLI."""
    from opencad.cli import main

    original = export_urdf(_robot(), tmp_path / "source.urdf")
    imported = import_urdf(original.urdf_path)
    source_json = tmp_path / "source.opencad.json"
    source_json.write_text(imported.model_dump_json(), encoding="utf-8")
    destination = tmp_path / "cli.urdf"
    assert main(["export", "urdf", str(source_json), "--output", str(destination)]) == 0
    assert destination.exists()
    assert len(import_urdf(destination).robot.joints) == 1


@pytest.mark.skipif(importlib.util.find_spec("cadquery") is None, reason="Native OCCT is not installed")
def test_native_part_link_exports_stl(tmp_path: Path) -> None:
    """An actual OpenCAD Part produces a portable per-link mesh bundle."""
    from opencad.kernel.core.backend_factory import create_backend
    from opencad.part import Part
    from opencad.runtime import RuntimeContext

    context = RuntimeContext(backend=create_backend("occt"))
    part = Part(context=context).box(20, 10, 5)
    with pytest.raises(ValueError, match="Invalid ROS"):
        RobotBuilder("cad_bot").add_part_link("../escape", part, assets_dir=tmp_path / "cad_assets")
    assert not (tmp_path / "escape.stl").exists()
    robot = (RobotBuilder("cad_bot")
             .add_part_link("body", part, assets_dir=tmp_path / "cad_assets")
             .add_link("end")
             .add_joint("fixed_joint", "body", "end", joint_type="fixed")
             .build())
    export = export_urdf(robot, tmp_path / "cad_bot.urdf")
    assert len(export.mesh_paths) == 1
    assert Path(export.mesh_paths[0]).stat().st_size > 100
    assert import_urdf(export.urdf_path).robot.links["body"].visuals[0].geometry.mesh_scale == (0.001,) * 3
