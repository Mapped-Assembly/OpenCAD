"""Regression and security tests for the URDF robot description importer."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from opencad.robotics import URDFImportResult, import_urdf


def _write(tmp_path: Path, xml: str, *, create_mesh: bool = False) -> Path:
    """Write a small local URDF fixture and optionally a package-style STL."""
    path = tmp_path / "robot.urdf"
    path.write_text(xml, encoding="utf-8")
    if create_mesh:
        assets = tmp_path / "assets"
        assets.mkdir()
        (assets / "tip.stl").write_text("solid tip\nendsolid tip\n", encoding="utf-8")
    return path


def _robot(*, tip_geometry: str = '<box size="0.02 0.03 0.04"/>', joint_type: str = "revolute") -> str:
    """Return a self-contained two-link robot with nontrivial RPY and units."""
    return f"""
    <robot name="mini">
      <material name="blue"><color rgba="0.1 0.2 0.9 1"/></material>
      <link name="base">
        <inertial>
          <origin xyz="0.001 0 0" rpy="0 0 0"/>
          <mass value="0.5"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.03"/>
        </inertial>
        <visual><geometry><sphere radius="0.02"/></geometry></visual>
      </link>
      <link name="tip">
        <visual name="tip_shape">
          <origin xyz="0.01 0.02 0.03" rpy="0 0 1.5707963267948966"/>
          <geometry>{tip_geometry}</geometry>
          <material name="blue"/>
        </visual>
        <collision><geometry><cylinder radius="0.01" length="0.04"/></geometry></collision>
      </link>
      <joint name="hinge" type="{joint_type}">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0.1 -0.2 0.3" rpy="0 0 1.5707963267948966"/>
        <axis xyz="0 0 1"/>
        <limit effort="2" velocity="3" lower="-1.2" upper="1.2"/>
      </joint>
    </robot>
    """


def test_import_preserves_transforms_units_and_semantics(tmp_path: Path) -> None:
    """Frame rotation and translation must match URDF's meter/radian conventions."""
    result = import_urdf(_write(tmp_path, _robot()))
    assert isinstance(result, URDFImportResult)
    robot = result.robot
    assert robot.root_link == "base"
    assert set(robot.links) == {"base", "tip"}
    assert robot.links["base"].inertial.mass_kg == 0.5
    assert robot.links["base"].inertial.inertia_kg_m2["ixx"] == 0.01
    assert robot.links["base"].inertial.origin.translation_mm == (1.0, 0.0, 0.0)
    assert robot.links["tip"].visuals[0].geometry.size_mm == (20.0, 30.0, 40.0)
    assert robot.links["tip"].visuals[0].origin.translation_mm == (10.0, 20.0, 30.0)
    assert robot.links["tip"].visuals[0].material["rgba"] == (0.1, 0.2, 0.9, 1.0)
    assert robot.links["tip"].collisions[0].geometry.length_mm == 40.0
    joint = robot.joints["hinge"]
    assert joint.limits["lower"] == -1.2
    assert joint.origin.translation_mm == (100.0, -200.0, 300.0)
    assert joint.origin.rotation_quaternion_xyzw[2] == pytest.approx(math.sqrt(0.5))
    component = robot.assembly.components["urdf:tip"]
    assert component.transform == joint.origin
    assert component.geometry_refs == ["urdf:tip:visual:0"]
    assert robot.assembly.root_ids == ["urdf:base"]
    assert robot.metadata["verification"]["dynamics_unverified"] is True


def test_import_resolves_quaddle_style_package_uri(tmp_path: Path) -> None:
    """Local package://assets paths resolve under the explicit allowed root."""
    xml = _robot(tip_geometry='<mesh filename="package://assets/tip.stl" scale="1 2 3"/>')
    path = _write(tmp_path, xml, create_mesh=True)
    robot = import_urdf(path).robot
    mesh = robot.links["tip"].visuals[0].geometry
    assert mesh.kind == "mesh"
    assert mesh.source_uri == "package://assets/tip.stl"
    assert mesh.mesh_scale == (1.0, 2.0, 3.0)
    assert mesh.mesh_vertex_unit_to_mm == 1000.0
    assert mesh.resolved_path == str((tmp_path / "assets" / "tip.stl").resolve())


def test_explicit_package_map_and_missing_asset_reporting(tmp_path: Path) -> None:
    """Missing assets are explicit; a package mapping can target an allowed path."""
    xml = _robot(tip_geometry='<mesh filename="package://robot_description/meshes/tip.stl"/>')
    path = _write(tmp_path, xml)
    with pytest.raises(FileNotFoundError, match="Missing URDF mesh"):
        import_urdf(path)
    partial = import_urdf(path, require_assets=False)
    assert partial.unresolved_assets == ["package://robot_description/meshes/tip.stl"]
    assert partial.robot.links["tip"].visuals[0].geometry.resolved_path is None
    assert partial.robot.metadata["verification"]["geometry_loaded"] is False
    mapped = tmp_path / "robot_description"
    (mapped / "meshes").mkdir(parents=True)
    (mapped / "meshes" / "tip.stl").write_text("solid tip\nendsolid tip\n")
    result = import_urdf(path, package_roots={"robot_description": mapped})
    assert not result.unresolved_assets


@pytest.mark.parametrize("uri", [
    "../outside.stl",
    "package://assets/../../outside.stl",
    "package://../outside.stl",
    "https://example.com/mesh.stl",
    "file:///etc/passwd",
    "/etc/passwd",
    "C:\\unsafe.stl",
])
def test_rejects_unsafe_mesh_uris(tmp_path: Path, uri: str) -> None:
    """No path traversal, absolute path, Windows-drive path, or remote fetch."""
    path = _write(tmp_path, _robot(tip_geometry=f'<mesh filename="{uri}"/>'))
    with pytest.raises(ValueError, match="Unsafe|Unsupported|Invalid"):
        import_urdf(path, require_assets=False)


def test_rejects_symlink_escape(tmp_path: Path) -> None:
    """A symlink beneath the asset root cannot escape the permitted directory."""
    outside = tmp_path.parent / "outside_urdf_asset.stl"
    outside.write_text("solid stolen\nendsolid stolen\n")
    (tmp_path / "escape.stl").symlink_to(outside)
    path = _write(tmp_path, _robot(tip_geometry='<mesh filename="escape.stl"/>'))
    with pytest.raises(ValueError, match="outside"):
        import_urdf(path)


@pytest.mark.parametrize("payload", [
    '<!DOCTYPE robot [<!ENTITY test SYSTEM "file:///etc/passwd">]><robot name="x"><link name="a"/></robot>',
    '<!ENTITY attack "text"><robot name="x"><link name="a"/></robot>',
    '<robot name="x"><link name="a">',
])
def test_rejects_unsafe_or_malformed_xml(tmp_path: Path, payload: str) -> None:
    """Reject DTD/entity declarations as well as malformed XML."""
    with pytest.raises(ValueError):
        import_urdf(_write(tmp_path, payload))


def test_invalid_graph_is_rejected(tmp_path: Path) -> None:
    """The importer must not turn disconnected or cyclic robots into assemblies."""
    for xml in (
        '<robot name="x"><link name="a"/><link name="b"/></robot>',
        '<robot name="x"><link name="a"/><link name="b"/>'
        '<joint name="j" type="fixed"><parent link="a"/><child link="missing"/></joint></robot>',
        '<robot name="x"><link name="a"/><link name="b"/>'
        '<joint name="a_to_b" type="fixed"><parent link="a"/><child link="b"/></joint>'
        '<joint name="b_to_a" type="fixed"><parent link="b"/><child link="a"/></joint></robot>',
    ):
        with pytest.raises(ValueError):
            import_urdf(_write(tmp_path, xml))


def test_preserves_continuous_and_unsupported_joint_types(tmp_path: Path) -> None:
    """Do not silently force arbitrary joints into limited revolute motion."""
    result = import_urdf(_write(tmp_path, _robot(joint_type="continuous")))
    assert result.robot.joints["hinge"].joint_type == "continuous"
    assert result.robot.metadata["verification"]["kinematics_supported"] is False
    assert any("continuous" in warning for warning in result.warnings)
    result = import_urdf(_write(tmp_path, _robot(joint_type="planar")))
    assert result.robot.joints["hinge"].joint_type == "planar"
    assert any("not evaluated" in warning for warning in result.warnings)


def test_mimic_joint_not_falsely_solved(tmp_path: Path) -> None:
    """Retain mimic relationships as metadata rather than simulating a coupling."""
    xml = _robot().replace(
        '<axis xyz="0 0 1"/>',
        '<axis xyz="0 0 1"/><mimic joint="missing" multiplier="-1" offset="0.5"/>',
    )
    result = import_urdf(_write(tmp_path, xml))
    assert result.robot.joints["hinge"].mimic == {
        "joint": "missing", "multiplier": -1.0, "offset": 0.5,
    }
    assert result.robot.metadata["verification"]["constraints_unresolved"]
    assert any("missing joint" in warning for warning in result.warnings)


def test_snapshot_model_round_trip_is_lossless(tmp_path: Path) -> None:
    """Typed robot metadata and canonical assembly survive JSON serialization."""
    result = import_urdf(_write(tmp_path, _robot()))
    restored = URDFImportResult.model_validate_json(result.model_dump_json())
    assert restored.model_dump(mode="json") == result.model_dump(mode="json")
    assert restored.robot.assembly.components["urdf:tip"].metadata["parent_joint"] == "hinge"


def test_persistent_quaddle_fixture_if_supplied() -> None:
    """Optional integration check against a separately obtained official Quaddle checkout."""
    import os

    source = os.environ.get("OPENCAD_TEST_QUADDLE_URDF")
    if source is None:
        pytest.skip("Set OPENCAD_TEST_QUADDLE_URDF for the official Petoi fixture.")
    result = import_urdf(source)
    assert result.robot.name == "quaddle"
    assert len(result.robot.links) == 46
    assert len(result.robot.joints) == 45
    assert sum(j.joint_type == "revolute" for j in result.robot.joints.values()) == 16
    assert sum(j.joint_type == "continuous" for j in result.robot.joints.values()) == 8
    assert sum(j.joint_type == "fixed" for j in result.robot.joints.values()) == 21
    assert any(name.startswith("closing_") for name in result.robot.links)
    assert result.robot.metadata["verification"]["dynamics_unverified"]
