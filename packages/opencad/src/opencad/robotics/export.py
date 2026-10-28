"""Generate standard URDF robot descriptions and portable STL bundles.

This module uses the typed RobotDescription shared with the URDF importer. It
never estimates physical properties or infers missing actuators/constraints.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from opencad.kernel.core.models import RigidTransform
from opencad.robotics.urdf import (
    RobotDescription,
    RobotGeometry,
    RobotInertial,
    RobotJoint,
    RobotVisual,
    _validate_graph,
)

_ROS_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")
_INERTIA_NAMES = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
_MOVING = ("revolute", "continuous", "prismatic")


class URDFExportResult(BaseModel):
    """Paths and diagnostics produced by one successfully completed export."""

    urdf_path: str
    manifest_path: str
    mesh_paths: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


def _name(value: str, label: str) -> str:
    """Require portable ROS robot/link/joint names to avoid XML/ROS ambiguity."""
    if not _ROS_NAME.fullmatch(value):
        raise ValueError(f"Invalid ROS {label} name {value!r}.")
    return value


def _finite(value: float, label: str) -> float:
    """Reject non-finite output values without mutating or rounding units."""
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite.")
    return number


def _number(value: float, label: str = "value") -> str:
    """Format a validated finite number deterministically."""
    number = _finite(value, label)
    return "0" if number == 0 else format(number, ".17g")


def _vector(values: tuple[float, ...] | list[float], label: str) -> str:
    """Format a vector as standard space-separated floating-point values."""
    return " ".join(_number(value, label) for value in values)


def _rpy(transform: RigidTransform) -> tuple[float, float, float]:
    """Convert an XYZW quaternion to URDF fixed-axis XYZ roll/pitch/yaw."""
    x, y, z, w = (_finite(v, "quaternion") for v in transform.rotation_quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("A joint/geometry frame has an invalid zero quaternion.")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def _origin(parent: ET.Element, pose: RigidTransform) -> None:
    """Append URDF origin in meters and radians from OpenCAD mm/XYZW."""
    xyz = tuple(_finite(value, "translation") / 1000.0 for value in pose.translation_mm)
    ET.SubElement(parent, "origin", {"xyz": _vector(xyz, "translation"), "rpy": _vector(_rpy(pose), "rotation")})


def _validate_inertia(inertia: RobotInertial) -> None:
    """Check SI inertial matrix for finite, symmetric positive-semidefinite form."""
    mass = _finite(inertia.mass_kg, "mass")
    if mass < 0:
        raise ValueError("Mass cannot be negative.")
    values = inertia.inertia_kg_m2
    if set(values) != set(_INERTIA_NAMES):
        raise ValueError("Inertia must contain ixx, ixy, ixz, iyy, iyz, izz.")
    ixx, ixy, ixz, iyy, iyz, izz = (_finite(values[key], key) for key in _INERTIA_NAMES)
    scale = max(1.0, abs(ixx), abs(iyy), abs(izz))
    tol = 1e-12 * scale
    if min(ixx, iyy, izz) < -tol:
        raise ValueError("Inertia has a negative diagonal moment.")
    if (ixx * iyy - ixy * ixy < -tol or
        ixx * izz - ixz * ixz < -tol or
        iyy * izz - iyz * iyz < -tol):
        raise ValueError("Inertia fails positive-semidefinite principal minors.")
    determinant = (
        ixx * iyy * izz + 2 * ixy * ixz * iyz
        - ixx * iyz * iyz - iyy * ixz * ixz - izz * ixy * ixy
    )
    if determinant < -1e-12 * scale ** 3:
        raise ValueError("Inertia tensor determinant is negative.")
    _rpy(inertia.origin)


def _validate_robot(robot: RobotDescription) -> None:
    """Validate a complete robot without allowing an inconsistent assembly export."""
    _name(robot.name, "robot")
    if _validate_graph(robot.links, robot.joints) != robot.root_link:
        raise ValueError("Robot root_link does not match its kinematic tree.")
    if len(robot.assembly.root_ids) != 1:
        raise ValueError("OpenCAD assembly must have exactly one root.")
    by_link = {component.name: component for component in robot.assembly.components.values()}
    if set(by_link) != set(robot.links) or len(by_link) != len(robot.links):
        raise ValueError("OpenCAD assembly and URDF link names are out of sync.")
    if robot.assembly.components[robot.assembly.root_ids[0]].name != robot.root_link:
        raise ValueError("OpenCAD assembly root does not match the robot root.")
    expected_children: dict[str, set[str]] = {name: set() for name in robot.links}
    for joint in robot.joints.values():
        expected_children[joint.parent].add(joint.child)
    for link_name, component in by_link.items():
        children = {robot.assembly.components[child].name for child in component.child_ids}
        if children != expected_children[link_name]:
            raise ValueError(f"OpenCAD assembly edges are inconsistent for {link_name!r}.")
    for name, link in robot.links.items():
        _name(name, "link")
        if name != link.name:
            raise ValueError(f"Link map key/name mismatch: {name!r}.")
    for name, joint in robot.joints.items():
        _name(name, "joint")
        if joint.name != name:
            raise ValueError(f"Joint map key/name mismatch: {name!r}.")
        if joint.joint_type in ("planar", "floating"):
            raise ValueError(f"Joint {name!r} has unsupported export type {joint.joint_type!r}.")
        _rpy(joint.origin)
        axis = tuple(_finite(v, f"axis of {name}") for v in joint.axis)
        if joint.joint_type != "fixed" and sum(v * v for v in axis) < 1e-24:
            raise ValueError(f"Joint {name!r} has a zero-length axis.")
        for key, value in joint.limits.items():
            _finite(value, f"{name}.{key}")
        if joint.joint_type in _MOVING:
            required = {"effort", "velocity"}
            if joint.joint_type != "continuous":
                required.update(("lower", "upper"))
            missing = required - set(joint.limits)
            if missing:
                raise ValueError(f"Joint {name!r} lacks mandatory URDF limit values: {sorted(missing)}.")
            if joint.joint_type != "continuous" and joint.limits["lower"] > joint.limits["upper"]:
                raise ValueError(f"Joint {name!r} has inverted limits.")
        if joint.mimic is not None and joint.mimic.get("joint") not in robot.joints:
            raise ValueError(f"Joint {name!r} mimics an unknown joint.")
        for value in joint.dynamics.values():
            _finite(value, f"dynamics of {name}")


def _visual(
    parent: ET.Element,
    appearance: RobotVisual,
    *,
    tag: str,
    mesh_uris: dict[str, str],
    warnings: list[str],
) -> None:
    """Append one independently positioned URDF visual/collision geometry."""
    attrs: dict[str, str] = {}
    if appearance.name:
        attrs["name"] = _name(appearance.name, f"{tag} occurrence")
    element = ET.SubElement(parent, tag, attrs)
    _origin(element, appearance.origin)
    geom = ET.SubElement(element, "geometry")
    shape = appearance.geometry
    if shape.kind == "mesh":
        if not shape.resolved_path:
            raise ValueError(f"Mesh visual {appearance.name!r} has no resolved_path.")
        source = str(Path(shape.resolved_path).resolve())
        if source not in mesh_uris:
            raise ValueError(f"Mesh {source!r} was not staged for URDF export.")
        factor = _finite(shape.mesh_vertex_unit_to_mm, "mesh units") / 1000.0
        if factor <= 0:
            raise ValueError("Mesh source units must be positive.")
        scale = tuple(_finite(v, "mesh scale") * factor for v in shape.mesh_scale)
        if any(v <= 0 for v in scale):
            raise ValueError("Mesh scale must be positive.")
        ET.SubElement(geom, "mesh", {"filename": mesh_uris[source], "scale": _vector(scale, "mesh scale")})
    elif shape.kind == "box":
        if shape.size_mm is None or any(_finite(v, "box size") <= 0 for v in shape.size_mm):
            raise ValueError("Box must have three positive dimensions.")
        ET.SubElement(geom, "box", {"size": _vector([v / 1000 for v in shape.size_mm], "box size")})
    elif shape.kind == "cylinder":
        if shape.radius_mm is None or shape.length_mm is None:
            raise ValueError("Cylinder needs radius and length.")
        if min(_finite(shape.radius_mm, "radius"), _finite(shape.length_mm, "length")) <= 0:
            raise ValueError("Cylinder dimensions must be positive.")
        ET.SubElement(geom, "cylinder", {"radius": _number(shape.radius_mm / 1000),
                                          "length": _number(shape.length_mm / 1000)})
    elif shape.kind == "sphere":
        if shape.radius_mm is None or _finite(shape.radius_mm, "sphere radius") <= 0:
            raise ValueError("Sphere radius must be positive.")
        ET.SubElement(geom, "sphere", {"radius": _number(shape.radius_mm / 1000)})
    else:
        raise ValueError(f"Unsupported geometry type {shape.kind!r}.")
    if tag == "visual" and appearance.material:
        mat = appearance.material
        attributes = {"name": _name(str(mat["name"]), "material")} if mat.get("name") else {}
        if not attributes and "rgba" in mat:
            color_values = _vector(mat["rgba"], "material rgba")
            attributes["name"] = "opencad_color_" + hashlib.sha256(color_values.encode()).hexdigest()[:12]
        if not attributes:
            warnings.append("Unnamed material without RGBA omitted; full metadata remains in manifest.")
            return
        material = ET.SubElement(element, "material", attributes)
        if "rgba" in mat:
            rgba = mat["rgba"]
            if len(rgba) != 4 or any(not 0 <= _finite(v, "material rgba") <= 1 for v in rgba):
                raise ValueError("Material RGBA must contain four values in [0, 1].")
            ET.SubElement(material, "color", {"rgba": _vector(rgba, "material rgba")})
        if "texture_uri" in mat:
            warnings.append("Material texture reference omitted; companion manifest retains source metadata.")


def _link(
    root: ET.Element,
    name: str,
    robot: RobotDescription,
    mesh_uris: dict[str, str],
    warnings: list[str],
) -> None:
    """Append one link including inertial, visual, and collision descriptions."""
    link = robot.links[name]
    element = ET.SubElement(root, "link", {"name": name})
    if link.inertial is not None:
        _validate_inertia(link.inertial)
        inertia = ET.SubElement(element, "inertial")
        _origin(inertia, link.inertial.origin)
        ET.SubElement(inertia, "mass", {"value": _number(link.inertial.mass_kg, "mass")})
        ET.SubElement(inertia, "inertia", {key: _number(link.inertial.inertia_kg_m2[key], key) for key in _INERTIA_NAMES})
    else:
        warnings.append(f"Link {name!r} lacks inertial data; dynamics are unverified.")
    for visual in link.visuals:
        _visual(element, visual, tag="visual", mesh_uris=mesh_uris, warnings=warnings)
    for collision in link.collisions:
        _visual(element, collision, tag="collision", mesh_uris=mesh_uris, warnings=warnings)


def _joint(root: ET.Element, joint: RobotJoint, warnings: list[str]) -> None:
    """Append one supported URDF joint without manufacturing fake constraints."""
    element = ET.SubElement(root, "joint", {"name": joint.name, "type": joint.joint_type})
    ET.SubElement(element, "parent", {"link": joint.parent})
    ET.SubElement(element, "child", {"link": joint.child})
    _origin(element, joint.origin)
    if joint.joint_type != "fixed":
        ET.SubElement(element, "axis", {"xyz": _vector(joint.axis, "joint axis")})
    if joint.joint_type in _MOVING:
        limits = {key: _number(joint.limits[key], f"joint {joint.name} {key}")
                  for key in ("lower", "upper", "effort", "velocity")
                  if key in joint.limits and not (joint.joint_type == "continuous" and key in ("lower", "upper"))}
        ET.SubElement(element, "limit", limits)
        if joint.joint_type == "continuous" and any(key in joint.limits for key in ("lower", "upper")):
            warnings.append(f"Continuous joint {joint.name!r}: meaningless lower/upper bounds omitted.")
    if joint.dynamics:
        ET.SubElement(element, "dynamics", {
            key: _number(joint.dynamics[key]) for key in ("damping", "friction") if key in joint.dynamics
        })
    if joint.mimic is not None:
        target = str(joint.mimic["joint"])
        ET.SubElement(element, "mimic", {
            "joint": target,
            "multiplier": _number(joint.mimic.get("multiplier", 1.0)),
            "offset": _number(joint.mimic.get("offset", 0.0)),
        })
    if joint.metadata:
        warnings.append(f"Joint {joint.name!r} carries non-URDF metadata in the companion manifest.")


def export_urdf(
    robot: RobotDescription,
    output_path: str | Path,
    *,
    meshes_dir: str = "meshes",
) -> URDFExportResult:
    """Export valid URDF XML, a portable STL bundle, and a provenance sidecar.

    The XML is committed last after staging and validating all referenced assets.
    Imported mesh files retain their original coordinates, with explicit URDF
    scaling. Native OpenCAD STL vertices are in millimeters and get scale .001.
    No dynamics, actuator specifications, or collision proxies are inferred.
    """
    _validate_robot(robot)
    destination = Path(output_path).resolve()
    if destination.suffix.lower() != ".urdf":
        raise ValueError("URDF export destination must end with .urdf.")
    if not _ROS_NAME.fullmatch(meshes_dir):
        raise ValueError("meshes_dir must be a safe single relative directory name.")
    destination.parent.mkdir(parents=True, exist_ok=True)
    sources: dict[str, Path] = {}
    for link in robot.links.values():
        for occurrence in (*link.visuals, *link.collisions):
            geometry = occurrence.geometry
            if geometry.kind != "mesh":
                continue
            if geometry.resolved_path is None:
                raise ValueError(f"Unresolved mesh in link {link.name!r}; cannot export.")
            source = Path(geometry.resolved_path).resolve()
            if not source.is_file() or source.suffix.lower() != ".stl":
                raise ValueError(f"Mesh must be an existing local STL file: {source}")
            sources[str(source)] = source

    warnings: list[str] = []
    mesh_uris: dict[str, str] = {}
    all_meshes: list[str] = []
    manifest = destination.with_suffix(".opencad.json")
    with tempfile.TemporaryDirectory(prefix=".opencad-urdf-", dir=destination.parent) as stage_dir:
        stage = Path(stage_dir)
        staged_meshes = stage / meshes_dir
        staged_meshes.mkdir()
        for source_name, source in sorted(sources.items()):
            data = source.read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            filename = f"{digest}.stl"
            staged_file = staged_meshes / filename
            if not staged_file.exists():
                staged_file.write_bytes(data)
            mesh_uris[source_name] = f"{meshes_dir}/{filename}"
            all_meshes.append(str(destination.parent / meshes_dir / filename))

        root = ET.Element("robot", {"name": robot.name})
        for link_name in sorted(robot.links):
            _link(root, link_name, robot, mesh_uris, warnings)
        for joint_name in sorted(robot.joints):
            _joint(root, robot.joints[joint_name], warnings)
        if robot.metadata:
            warnings.append("OpenCAD metadata/provenance retained in companion JSON, not injected into standard URDF.")
        ET.indent(root, space="  ")
        xml_bytes = ET.tostring(root, encoding="utf-8", xml_declaration=True)
        # Validate our own output with the same parser used by the importer,
        # without relying on XML serialization alone as proof of compatibility.
        ET.fromstring(xml_bytes)
        (stage / "robot.urdf").write_bytes(xml_bytes)
        payload = {
            "version": 1,
            "robot": robot.model_dump(mode="json"),
            "warnings": warnings,
            "mesh_files": sorted(set(mesh_uris.values())),
        }
        (stage / "robot.opencad.json").write_text(
            json.dumps(payload, sort_keys=True, indent=2) + "\n", encoding="utf-8"
        )
        final_mesh_dir = destination.parent / meshes_dir
        final_mesh_dir.mkdir(exist_ok=True)
        for staged_file in sorted(staged_meshes.iterdir()):
            target = final_mesh_dir / staged_file.name
            if not target.exists():
                os.replace(staged_file, target)
        os.replace(stage / "robot.opencad.json", manifest)
        os.replace(stage / "robot.urdf", destination)
    return URDFExportResult(
        urdf_path=str(destination), manifest_path=str(manifest),
        mesh_paths=sorted(set(all_meshes)), warnings=warnings,
    )
