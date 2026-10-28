"""Independent URDF interoperability and kinematics acceptance harness.

Run with yourdfpy==0.0.60 and huggingface-hub==1.12.0 in a dedicated CI job.
The reference solver (yourdfpy) is not OpenCAD's own FK implementation.

Pass criteria cover generated supported robot trees and the real, pinned Petoi
Quaddle URDF. A serial URDF tree cannot certify passive four-bar constraints.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from huggingface_hub import snapshot_download
from yourdfpy import URDF

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import (
    RobotBuilder,
    RobotGeometry,
    RobotInertial,
    RobotVisual,
    diagnose_mechanism,
    export_urdf,
    import_urdf,
)
from opencad.robotics.poses import evaluate_robot_poses

PETOI_REPO = "petoi/quaddle"
PETOI_COMMIT = "d5ed8ccfd238f98c2c47eb47658ed23232b1a74d"
YOURDFPY_VERSION = "0.0.60"
TRANSLATION_TOLERANCE_M = 1e-7
ROTATION_TOLERANCE_RAD = 1e-6
EXPECTED_QUADDLE_COUNTS = {"links": 46, "joints": 45, "revolute": 16, "continuous": 8, "fixed": 21}


def _sha256(path: Path) -> str:
    """Calculate SHA-256 without loading large mesh assets into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fixture_pose(
    xyz_mm: tuple[float, float, float], rpy: tuple[float, float, float]
) -> RigidTransform:
    """Create a quaternion from independent, standard URDF fixed-axis XYZ RPY."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return RigidTransform(
        translation_mm=xyz_mm,
        rotation_quaternion_xyzw=(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ),
    )


def _generate_native_robot(stl_path: Path) -> Any:
    """Generate a multi-joint model entirely through OpenCAD's public authoring API."""
    stl_path.write_text(
        "solid sample\n"
        " facet normal 0 0 1\n"
        "  outer loop\n"
        "   vertex 0 0 0\n"
        "   vertex 10 0 0\n"
        "   vertex 0 20 0\n"
        "  endloop\n"
        " endfacet\n"
        "endsolid sample\n", encoding="utf-8",
    )
    valid_inertia = RobotInertial(
        origin=RigidTransform.identity(),
        mass_kg=0.5,
        inertia_kg_m2={
            "ixx": 0.001, "ixy": 0.0, "ixz": 0.0,
            "iyy": 0.0015, "iyz": 0.0, "izz": 0.002,
        },
    )
    return (
        RobotBuilder("independent_fk")
        .add_link("base", visuals=[
            RobotVisual(geometry=RobotGeometry(kind="box", size_mm=(40, 30, 15)))
        ], inertial=valid_inertia)
        .add_link("fixed_link", visuals=[
            RobotVisual(
                origin=_fixture_pose((15, -5, 10), (0.2, 0.1, -0.3)),
                geometry=RobotGeometry(kind="cylinder", radius_mm=7, length_mm=25),
            )
        ])
        .add_link("revolute_link", visuals=[
            RobotVisual(
                origin=_fixture_pose((5, -8, 12), (-0.23, 0.21, 0.45)),
                geometry=RobotGeometry(kind="sphere", radius_mm=6),
            )
        ])
        .add_link("prismatic_link", visuals=[
            RobotVisual(geometry=RobotGeometry(
                kind="mesh", resolved_path=str(stl_path),
                mesh_vertex_unit_to_mm=1.0,
            )),
        ])
        .add_link("continuous_link", visuals=[
            RobotVisual(
                origin=_fixture_pose((25, 6, -13), (0.4, -0.2, 0.05)),
                geometry=RobotGeometry(kind="box", size_mm=(12, 9, 15)),
            )
        ])
        .add_joint(
            "fixed_mount", "base", "fixed_link", joint_type="fixed",
            origin=_fixture_pose((80, -25, 10), (0.34, -0.2, 0.8)),
        )
        .add_joint(
            "hinge", "fixed_link", "revolute_link", joint_type="revolute",
            origin=_fixture_pose((70, 25, 30), (0.1, -0.3, 0.15)),
            axis=(0.4, 0.5, 0.7),
            limits={"lower": -1.1, "upper": 1.2, "effort": 2.0, "velocity": 3.0},
        )
        .add_joint(
            "slide", "revolute_link", "prismatic_link", joint_type="prismatic",
            origin=_fixture_pose((10, -20, 40), (-0.14, 0.21, -0.18)),
            axis=(0.3, 0.0, 0.8),
            limits={"lower": -0.08, "upper": 0.09, "effort": 2.0, "velocity": 0.5},
        )
        .add_joint(
            "wheel", "prismatic_link", "continuous_link", joint_type="continuous",
            origin=_fixture_pose((8, 25, -4), (0.1, -0.45, 0.2)),
            axis=(0.0, 0.6, 0.3),
            limits={"effort": 2.0, "velocity": 20.0},
        )
        .build()
    )


def _quaternion_rotation(quaternion: tuple[float, float, float, float]) -> np.ndarray:
    """Convert OpenCAD XYZW quaternion to a right-handed rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(float(x*x + y*y + z*z + w*w))
    if norm <= 1e-12:
        raise AssertionError("Invalid OpenCAD quaternion.")
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _opencad_matrix(pose: RigidTransform) -> np.ndarray:
    """Represent OpenCAD transforms as homogeneous matrices in SI meters."""
    matrix = np.eye(4)
    matrix[:3, :3] = _quaternion_rotation(pose.rotation_quaternion_xyzw)
    matrix[:3, 3] = np.asarray(pose.translation_mm, dtype=float) / 1000.0
    return matrix


def _error(reference: np.ndarray, candidate: np.ndarray) -> tuple[float, float]:
    """Measure translation norm (m) and geodesic SO(3) error (radians)."""
    pos = float(np.linalg.norm(reference[:3, 3] - candidate[:3, 3]))
    relative = reference[:3, :3].T @ candidate[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    return pos, angle


def _independent_load(path: Path, *, scene: bool = True) -> URDF:
    """Load using a distinct library without importing referenced geometry."""
    model = URDF.load(
        str(path), load_meshes=False, build_scene_graph=scene,
        load_collision_meshes=False,
    )
    assert model is not None, f"Independent parser returned no robot: {path}"
    return model


def _require_external_validity(model: URDF, model_name: str) -> None:
    """Enforce strict independent URDF validation for generated model fixtures."""
    if not model.validate():
        raise AssertionError(
            f"Independent {model_name} URDF validation failed: "
            + "; ".join(map(str, model.errors))
        )


def _compare_tree_fk(
    path: Path,
    cases: list[dict[str, float]],
) -> dict[str, Any]:
    """Compare all supported links and visual frames against yourdfpy FK."""
    imported = import_urdf(path)
    assert not imported.unresolved_assets
    assert not imported.robot.metadata["verification"]["constraints_unresolved"]
    original = _independent_load(path)
    _require_external_validity(original, "generated")
    assert set(original.link_map) == set(imported.robot.links)
    assert set(original.joint_map) == set(imported.robot.joints)

    max_pos = 0.0
    max_rot = 0.0
    samples = 0
    visual_samples = 0
    for positions in cases:
        # Recreate the reference model for each test to avoid stale state.
        reference = _independent_load(path)
        reference.update_cfg(positions)
        opencad = evaluate_robot_poses(imported.robot, positions)
        for name in sorted(imported.robot.links):
            expect = np.asarray(reference.get_transform(name), dtype=float)
            actual = _opencad_matrix(opencad.link_transforms[name])
            pos, rot = _error(expect, actual)
            if pos > TRANSLATION_TOLERANCE_M or rot > ROTATION_TOLERANCE_RAD:
                raise AssertionError(
                    f"Independent FK mismatch for {name} at {positions}: "
                    f"translation={pos:.10g} m, rotation={rot:.10g} rad."
                )
            max_pos, max_rot = max(pos, max_pos), max(rot, max_rot)
            samples += 1
            for i, visual in enumerate(imported.robot.links[name].visuals):
                # The visual placement matrix is parsed by yourdfpy, not OpenCAD.
                other_visual = reference.link_map[name].visuals[i]
                visual_origin = (
                    np.eye(4) if other_visual.origin is None
                    else np.asarray(other_visual.origin, dtype=float)
                )
                expected_visual = expect @ visual_origin
                visual_id = f"urdf:{name}:visual:{i}"
                actual_visual = _opencad_matrix(opencad.shape_transforms[visual_id])
                vp, vr = _error(expected_visual, actual_visual)
                if vp > TRANSLATION_TOLERANCE_M or vr > ROTATION_TOLERANCE_RAD:
                    raise AssertionError(
                        f"Independent visual FK mismatch for {visual_id}: "
                        f"{vp:.10g} m, {vr:.10g} rad."
                    )
                max_pos, max_rot = max(vp, max_pos), max(vr, max_rot)
                visual_samples += 1
    return {
        "status": "pass",
        "positions_tested": cases,
        "link_pose_comparisons": samples,
        "visual_pose_comparisons": visual_samples,
        "max_translation_error_m": max_pos,
        "max_rotation_error_rad": max_rot,
        "source_sha256": _sha256(path),
    }


def _verify_generated(output_dir: Path) -> dict[str, Any]:
    """Check native → URDF → import → URDF → import, all with yourdfpy."""
    original = _generate_native_robot(output_dir / "native_part.stl")
    first = export_urdf(original, output_dir / "native.urdf")
    source = Path(first.urdf_path)
    restored = import_urdf(source).robot
    for joint_name, joint in original.joints.items():
        again = restored.joints[joint_name]
        assert again.joint_type == joint.joint_type
        assert again.limits == joint.limits
        assert np.allclose(again.origin.translation_mm, joint.origin.translation_mm)
    assert set(restored.links) == set(original.links)
    # The STL contains mm coordinates; emitted scale must be exactly 0.001.
    geometry = restored.links["prismatic_link"].visuals[0].geometry
    assert geometry.mesh_scale == (0.001, 0.001, 0.001)
    assert geometry.mesh_vertex_unit_to_mm == 1000
    assert geometry.resolved_path is not None

    cases = [
        {"hinge": 0.0, "slide": 0.0, "wheel": 0.0},
        {"hinge": 0.65, "slide": 0.046, "wheel": 2.7},
        {"hinge": -0.77, "slide": -0.038, "wheel": -9.1},
        {"hinge": 1.13, "slide": 0.082, "wheel": 18.0},
    ]
    first_fk = _compare_tree_fk(source, cases)
    second_export = export_urdf(restored, output_dir / "reexport" / "native.urdf")
    reloaded = import_urdf(second_export.urdf_path).robot
    assert set(reloaded.links) == set(restored.links)
    assert set(reloaded.joints) == set(restored.joints)
    assert reloaded.joints["slide"].limits["upper"] == 0.09
    second_fk = _compare_tree_fk(Path(second_export.urdf_path), cases)
    assert first_fk["link_pose_comparisons"] >= 20
    return {
        "structural_agreement": "pass",
        "kinematic_agreement": "pass",
        "physical_validation": "not_performed",
        "first_export": first_fk,
        "second_export": second_fk,
        "stl_source_sha256": _sha256(output_dir / "native_part.stl"),
        "first_mesh_hashes": [
            {"file": Path(file).name, "sha256": _sha256(Path(file))}
            for file in first.mesh_paths
        ],
        "second_mesh_hashes": [
            {"file": Path(file).name, "sha256": _sha256(Path(file))}
            for file in second_export.mesh_paths
        ],
    }


def _verify_pinned_quaddle(output_dir: Path) -> dict[str, Any]:
    """Fetch **real** official MIT-licensed Quaddle source at a fixed SHA."""
    checkout = Path(snapshot_download(
        repo_id=PETOI_REPO,
        revision=PETOI_COMMIT,
        local_dir=str(output_dir / "petoi-quaddle"),
        allow_patterns=["robot.urdf", "LICENSE", "assets/*.stl"],
    ))
    urdf_path = checkout / "robot.urdf"
    license_path = checkout / "LICENSE"
    if not urdf_path.is_file() or not license_path.is_file():
        raise AssertionError("Pinned official Quaddle URDF or license missing.")
    license_text = license_path.read_text(encoding="utf-8")
    if "Permission is hereby granted" not in license_text:
        raise AssertionError("Official Quaddle license must be verified before using fixture.")
    imported = import_urdf(urdf_path, asset_root=checkout, require_assets=True)
    if imported.unresolved_assets:
        raise AssertionError(f"Unresolved Quaddle geometry: {imported.unresolved_assets}")
    robot = imported.robot
    breakdown = {
        "links": len(robot.links), "joints": len(robot.joints),
        "revolute": sum(j.joint_type == "revolute" for j in robot.joints.values()),
        "continuous": sum(j.joint_type == "continuous" for j in robot.joints.values()),
        "fixed": sum(j.joint_type == "fixed" for j in robot.joints.values()),
    }
    if breakdown != EXPECTED_QUADDLE_COUNTS:
        raise AssertionError(f"Pinned Quaddle fixture topology drift: {breakdown}")
    independent = _independent_load(urdf_path)
    if set(independent.link_map) != set(robot.links):
        raise AssertionError("Quaddle independent link names disagree.")
    if set(independent.joint_map) != set(robot.joints):
        raise AssertionError("Quaddle independent joint names disagree.")
    for name, joint in robot.joints.items():
        independent_joint = independent.joint_map[name]
        if independent_joint.type != joint.joint_type:
            raise AssertionError(f"Quaddle joint type disagrees: {name}")
        if independent_joint.parent != joint.parent or independent_joint.child != joint.child:
            raise AssertionError(f"Quaddle joint parent/child disagrees: {name}")
    independent_valid = bool(independent.validate())
    independent_errors = [str(err) for err in independent.errors]

    # Exercise full real-asset import → export → re-import. Never count a
    # synthetic or skipped official fixture as a passing Quaddle integration.
    rebuilt = export_urdf(robot, output_dir / "quaddle-export" / "robot.urdf")
    reimported = import_urdf(rebuilt.urdf_path, require_assets=True).robot
    if set(robot.links) != set(reimported.links) or set(robot.joints) != set(reimported.joints):
        raise AssertionError("Quaddle import→export→reimport lost links or joints.")
    independent_exported = _independent_load(Path(rebuilt.urdf_path))
    if set(independent_exported.link_map) != set(robot.links):
        raise AssertionError("External validator could not parse regenerated Quaddle links.")
    if set(independent_exported.joint_map) != set(robot.joints):
        raise AssertionError("External validator could not parse regenerated Quaddle joints.")
    if not robot.metadata["verification"]["constraints_unresolved"]:
        raise AssertionError("Quaddle passive/closing constraints must be flagged unresolved.")
    mechanism = diagnose_mechanism(robot)
    if mechanism.state != "constraints_unresolved" or not mechanism.possible_closure_markers:
        raise AssertionError("Real Quaddle must be diagnosed as having possible closure markers.")
    if mechanism.supported_solvers:
        raise AssertionError("Quaddle must not be assigned a fabricated dimensioned four-bar solver.")
    try:
        evaluate_robot_poses(robot, {})
    except ValueError as exc:
        if "closed-loop" not in str(exc):
            raise AssertionError(f"Unexpected Quaddle unsupported-motion diagnosis: {exc}") from exc
    else:
        raise AssertionError("Quaddle tree was wrongly marked as physically valid.")

    assets = sorted((checkout / "assets").glob("*.stl"))
    if not assets:
        raise AssertionError("Official Quaddle fixture did not include STL mesh assets.")
    return {
        "repository": PETOI_REPO,
        "pinned_revision": PETOI_COMMIT,
        "license": "MIT",
        "license_sha256": _sha256(license_path),
        "source_sha256": _sha256(urdf_path),
        "structural_agreement": "pass",
        "independent_source_valid": independent_valid,
        "independent_source_diagnostics": independent_errors,
        "independent_regenerated_valid": bool(independent_exported.validate()),
        "independent_regenerated_diagnostics": [
            str(err) for err in independent_exported.errors
        ],
        "kinematic_agreement": "not_asserted_closed_loop_constraints",
        "physical_validation": "not_performed",
        "constraints_unresolved": True,
        "mechanism_diagnostic": mechanism.model_dump(mode="json"),
        "mesh_assets_verified": len(assets),
        "mesh_asset_sha256": {
            mesh.name: _sha256(mesh) for mesh in assets
        },
        "regenerated_urdf_sha256": _sha256(Path(rebuilt.urdf_path)),
        "topology": breakdown,
        "unsupported": [
            "passive_four_bar_loop",
            "spring_dynamics",
            "foot_contact",
            "hardware_actuation",
        ],
    }


def main() -> None:
    """Run mandatory independent tests and persist evidence only on success."""
    import yourdfpy

    output = Path(os.environ.get("OPENCAD_ROBOT_VERIFICATION_REPORT", "reports/robot-verification.json"))
    output.parent.mkdir(parents=True, exist_ok=True)
    workspace = output.parent / "robot-verification-fixtures"
    workspace.mkdir(parents=True, exist_ok=True)
    data: dict[str, Any] = {
        "schema_version": 1,
        "independent_parser": "yourdfpy",
        "independent_parser_version": YOURDFPY_VERSION,
        "reference_model": PETOI_REPO,
        "reference_revision": PETOI_COMMIT,
        "tolerances": {
            "translation_m": TRANSLATION_TOLERANCE_M,
            "rotation_rad": ROTATION_TOLERANCE_RAD,
        },
        "capability_labels": {
            "structural": "tested",
            "kinematics": "supported_serial_tree_only",
            "physical": "not_performed",
        },
    }
    # No try/except that would turn failed network, parser or physics checks
    # into a green CI run. Missing official assets must fail the dedicated job.
    data["synthetic"] = _verify_generated(workspace)
    data["petoi_quaddle"] = _verify_pinned_quaddle(workspace)
    output.write_text(json.dumps(data, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    print(f"Independent URDF verification passed: {output}")
    print(
        f"Reference: {PETOI_REPO}@{PETOI_COMMIT}; "
        f"synthetic link comparisons: {data['synthetic']['first_export']['link_pose_comparisons']}"
    )
    print("Physical validation: NOT PERFORMED (Quaddle constraints unresolved).")


if __name__ == "__main__":
    main()
