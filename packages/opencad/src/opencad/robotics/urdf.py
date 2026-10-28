"""Safely import URDF geometry and kinematic semantics into OpenCAD.

The importer is deliberately *not* a dynamics engine. RobotDescription retains
URDF semantics that the current OpenCAD kinematics evaluator cannot yet execute,
including continuous, floating, planar, and mimic joints.
"""

from __future__ import annotations

import hashlib
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, Literal, Mapping

from pydantic import BaseModel, Field

from opencad.assembly.models import AssemblyComponent, AssemblyTree
from opencad.kernel.core.models import RigidTransform

URDFJointType = Literal["fixed", "revolute", "continuous", "prismatic", "planar", "floating"]
URDFGeometryType = Literal["mesh", "box", "cylinder", "sphere"]
MAX_URDF_BYTES = 8 * 1024 * 1024
_UNSAFE_XML = re.compile(br"<!\s*(?:DOCTYPE|ENTITY)\b", flags=re.IGNORECASE)


class RobotGeometry(BaseModel):
    """Geometry in a URDF link-local frame; length values use millimeters.

    A mesh's own vertex coordinates conventionally use meters per URDF. Apply
    mesh_vertex_unit_to_mm and the dimensionless mesh_scale when rendering.
    """

    kind: URDFGeometryType
    source_uri: str | None = None
    resolved_path: str | None = None
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    mesh_vertex_unit_to_mm: float = 1000.0
    size_mm: tuple[float, float, float] | None = None
    radius_mm: float | None = None
    length_mm: float | None = None


class RobotVisual(BaseModel):
    """One visual or collision occurrence with its own transform and geometry."""

    name: str | None = None
    origin: RigidTransform = Field(default_factory=RigidTransform.identity)
    geometry: RobotGeometry
    material: dict[str, Any] = Field(default_factory=dict)


class RobotInertial(BaseModel):
    """URDF inertial parameters in SI units, not converted to OpenCAD mm."""

    origin: RigidTransform
    mass_kg: float
    inertia_kg_m2: dict[str, float]


class RobotLink(BaseModel):
    """Stable URDF link identity, appearances, and original physical metadata."""

    name: str
    visuals: list[RobotVisual] = Field(default_factory=list)
    collisions: list[RobotVisual] = Field(default_factory=list)
    inertial: RobotInertial | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class RobotJoint(BaseModel):
    """URDF joint semantics, including types not yet animated by OpenCAD."""

    name: str
    joint_type: URDFJointType
    parent: str
    child: str
    origin: RigidTransform
    axis: tuple[float, float, float]
    # Revolute angular limits are radians; prismatic limits are meters,
    # as in the source URDF; never imply all fields have the same dimension.
    limits: dict[str, float] = Field(default_factory=dict)
    dynamics: dict[str, float] = Field(default_factory=dict)
    mimic: dict[str, Any] | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class RobotDescription(BaseModel):
    """Canonical imported robot document coupled to OpenCAD's AssemblyTree."""

    name: str
    root_link: str
    links: dict[str, RobotLink]
    joints: dict[str, RobotJoint]
    assembly: AssemblyTree
    metadata: dict[str, Any] = Field(default_factory=dict)


class URDFImportResult(BaseModel):
    """Imported robot plus explicit warnings and asset-resolution diagnostics."""

    robot: RobotDescription
    warnings: list[str] = Field(default_factory=list)
    unresolved_assets: list[str] = Field(default_factory=list)


def _numbers(value: str, count: int, label: str) -> tuple[float, ...]:
    """Parse a finite, fixed-width space-separated floating-point vector."""
    try:
        result = tuple(float(item) for item in value.split())
    except ValueError as exc:
        raise ValueError(f"Invalid {label}: {value!r}") from exc
    if len(result) != count or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{label} must contain {count} finite numbers: {value!r}")
    return result


def _attribute_number(element: ET.Element, key: str, label: str) -> float:
    """Return one required finite XML floating-point attribute."""
    raw = element.get(key)
    if raw is None:
        raise ValueError(f"{label} is missing required attribute {key!r}")
    return _numbers(raw, 1, label + "." + key)[0]


def _pose(element: ET.Element | None) -> RigidTransform:
    """Convert URDF xyz meters and extrinsic XYZ RPY into mm and XYZW quaternion."""
    if element is None:
        return RigidTransform.identity()
    xyz = _numbers(element.get("xyz", "0 0 0"), 3, "origin.xyz")
    roll, pitch, yaw = _numbers(element.get("rpy", "0 0 0"), 3, "origin.rpy")
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    return RigidTransform(
        translation_mm=tuple(1000.0 * x for x in xyz),
        rotation_quaternion_xyzw=q,
    )


def _safe_mesh_path(
    uri: str,
    root: Path,
    package_roots: Mapping[str, Path],
) -> Path:
    """Map local/ROS package URIs to allowed roots, rejecting unsafe paths."""
    if "\\" in uri or not uri or uri.startswith(("/", "~")):
        raise ValueError(f"Unsafe mesh URI: {uri!r}")
    if uri.startswith("package://"):
        remainder = uri[len("package://"):]
        package, sep, relative = remainder.partition("/")
        if not sep or not package or not relative:
            raise ValueError(f"Invalid package URI: {uri!r}")
        if package in (".", "..") or ":" in package:
            raise ValueError(f"Unsafe package name: {package!r}")
        base = Path(package_roots.get(package, root / package)).resolve()
        name = relative
    else:
        if "://" in uri or ":" in uri:
            raise ValueError(f"Unsupported mesh URI: {uri!r}")
        base = root
        name = uri
    parts = PurePosixPath(name).parts
    if not parts or ".." in parts or any(part in ("", ".") for part in name.split("/")):
        raise ValueError(f"Unsafe mesh path traversal: {uri!r}")
    candidate = base.joinpath(*parts).resolve()
    if not candidate.is_relative_to(base):
        raise ValueError(f"Mesh URI resolves outside its permitted root: {uri!r}")
    return candidate


def _parse_geometry(
    element: ET.Element,
    root: Path,
    package_roots: Mapping[str, Path],
    warnings: list[str],
    missing: list[str],
    require_assets: bool,
) -> RobotGeometry:
    """Parse one supported URDF primitive or resolve one referenced local mesh."""
    geometries = list(element)
    if len(geometries) != 1 or geometries[0].tag not in ("mesh", "box", "cylinder", "sphere"):
        raise ValueError("URDF <geometry> must contain exactly one supported shape.")
    shape = geometries[0]
    if shape.tag == "mesh":
        uri = shape.get("filename")
        if not uri:
            raise ValueError("URDF mesh is missing filename.")
        scale = _numbers(shape.get("scale", "1 1 1"), 3, "mesh.scale")
        if any(value <= 0 for value in scale):
            raise ValueError("URDF mesh scale values must be positive.")
        path = _safe_mesh_path(uri, root, package_roots)
        if not path.is_file():
            missing.append(uri)
            message = f"Missing URDF mesh {uri!r} (resolved to {path})"
            if require_assets:
                raise FileNotFoundError(message)
            warnings.append(message)
        return RobotGeometry(
            kind="mesh", source_uri=uri, resolved_path=str(path) if path.is_file() else None,
            mesh_scale=scale,
        )
    if shape.tag == "box":
        size = _numbers(shape.get("size", ""), 3, "box.size")
        if any(value <= 0 for value in size):
            raise ValueError("URDF box dimensions must be positive.")
        return RobotGeometry(kind="box", size_mm=tuple(1000.0 * value for value in size))
    if shape.tag == "cylinder":
        radius = _attribute_number(shape, "radius", "cylinder")
        length = _attribute_number(shape, "length", "cylinder")
        if min(radius, length) <= 0:
            raise ValueError("URDF cylinder dimensions must be positive.")
        return RobotGeometry(kind="cylinder", radius_mm=radius * 1000, length_mm=length * 1000)
    radius = _attribute_number(shape, "radius", "sphere")
    if radius <= 0:
        raise ValueError("URDF sphere radius must be positive.")
    return RobotGeometry(kind="sphere", radius_mm=radius * 1000)


def _parse_appearance(
    element: ET.Element,
    root: Path,
    package_roots: Mapping[str, Path],
    materials: Mapping[str, dict[str, Any]],
    warnings: list[str],
    missing: list[str],
    require_assets: bool,
) -> RobotVisual:
    """Read a visual/collision, keeping its independent origin and material."""
    geometry = element.find("geometry")
    if geometry is None:
        raise ValueError(f"URDF <{element.tag}> has no <geometry>.")
    material_elem = element.find("material")
    material: dict[str, Any] = {}
    if material_elem is not None:
        material_name = material_elem.get("name")
        if material_name:
            material["name"] = material_name
            material.update(materials.get(material_name, {}))
        color = material_elem.find("color")
        if color is not None:
            material["rgba"] = _numbers(color.get("rgba", ""), 4, "material.rgba")
        texture = material_elem.find("texture")
        if texture is not None:
            material["texture_uri"] = texture.get("filename")
    return RobotVisual(
        name=element.get("name"),
        origin=_pose(element.find("origin")),
        geometry=_parse_geometry(geometry, root, package_roots, warnings, missing, require_assets),
        material=material,
    )


def _parse_inertial(element: ET.Element) -> RobotInertial:
    """Preserve mass in kilograms and symmetric inertia entries in kg m²."""
    mass = element.find("mass")
    tensor = element.find("inertia")
    if mass is None or tensor is None:
        raise ValueError("URDF inertial must contain mass and inertia.")
    mass_kg = _attribute_number(mass, "value", "mass")
    if mass_kg < 0:
        raise ValueError("URDF link mass cannot be negative.")
    entries = {
        key: _attribute_number(tensor, key, "inertia")
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }
    return RobotInertial(origin=_pose(element.find("origin")), mass_kg=mass_kg, inertia_kg_m2=entries)


def _parse_joint(element: ET.Element) -> RobotJoint:
    """Parse a joint while retaining unsupported movement types without coercion."""
    name = element.get("name")
    joint_type = element.get("type")
    if not name or joint_type not in ("fixed", "revolute", "continuous", "prismatic", "planar", "floating"):
        raise ValueError(f"Unknown or unnamed URDF joint: {name!r} {joint_type!r}")
    parent = element.find("parent")
    child = element.find("child")
    if parent is None or child is None or not parent.get("link") or not child.get("link"):
        raise ValueError(f"URDF joint {name!r} must identify parent and child links.")
    axis = _numbers(
        (element.find("axis").get("xyz", "1 0 0") if element.find("axis") is not None else "1 0 0"),
        3, f"joint {name!r} axis",
    )
    if joint_type not in ("fixed", "floating") and sum(v * v for v in axis) < 1e-24:
        raise ValueError(f"URDF joint {name!r} has a zero-length motion axis.")
    limits_elem = element.find("limit")
    limits: dict[str, float] = {}
    if limits_elem is not None:
        for key in ("lower", "upper", "effort", "velocity"):
            if limits_elem.get(key) is not None:
                limits[key] = _attribute_number(limits_elem, key, f"joint {name!r}.limit")
    if joint_type == "continuous":
        # Only warn at the caller; preserve even nonstandard limits for fidelity.
        pass
    if "lower" in limits and "upper" in limits and limits["lower"] > limits["upper"]:
        raise ValueError(f"URDF joint {name!r} has reversed limits.")
    dynamics: dict[str, float] = {}
    dynamics_elem = element.find("dynamics")
    if dynamics_elem is not None:
        for key in ("damping", "friction"):
            if dynamics_elem.get(key) is not None:
                dynamics[key] = _attribute_number(dynamics_elem, key, f"joint {name!r}.dynamics")
    mimic_elem = element.find("mimic")
    mimic: dict[str, Any] | None = None
    if mimic_elem is not None:
        if not mimic_elem.get("joint"):
            raise ValueError(f"URDF joint {name!r} has mimic without a target joint.")
        mimic = {
            "joint": mimic_elem.get("joint"),
            "multiplier": _numbers(mimic_elem.get("multiplier", "1"), 1, "mimic.multiplier")[0],
            "offset": _numbers(mimic_elem.get("offset", "0"), 1, "mimic.offset")[0],
        }
    return RobotJoint(
        name=name, joint_type=joint_type, parent=parent.get("link"), child=child.get("link"),
        origin=_pose(element.find("origin")), axis=axis, limits=limits,
        dynamics=dynamics, mimic=mimic,
        metadata={"calibration_xml": ET.tostring(element.find("calibration"), encoding="unicode")}
        if element.find("calibration") is not None else {},
    )


def _validate_graph(
    links: Mapping[str, RobotLink],
    joints: Mapping[str, RobotJoint],
) -> str:
    """Validate rooted single-parent tree semantics before building an assembly."""
    if not links:
        raise ValueError("URDF contains no links.")
    parents: dict[str, str] = {}
    children: dict[str, list[str]] = {name: [] for name in links}
    for joint in joints.values():
        if joint.parent not in links or joint.child not in links:
            raise ValueError(f"Joint {joint.name!r} references a nonexistent link.")
        if joint.parent == joint.child or joint.child in parents:
            raise ValueError(f"Joint {joint.name!r} creates a self-edge or multi-parent link.")
        parents[joint.child] = joint.parent
        children[joint.parent].append(joint.child)
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError(f"URDF must have exactly one root link, got {sorted(roots)!r}.")
    root = next(iter(roots))
    visited: set[str] = set()
    def walk(name: str) -> None:
        if name in visited:
            raise ValueError("URDF joint graph contains a cycle.")
        visited.add(name)
        for child in children[name]:
            walk(child)
    walk(root)
    if len(visited) != len(links):
        raise ValueError("URDF joint graph is disconnected or cyclic.")
    return root


def import_urdf(
    path: str | Path,
    *,
    asset_root: str | Path | None = None,
    package_roots: Mapping[str, str | Path] | None = None,
    require_assets: bool = True,
) -> URDFImportResult:
    """Import a local URDF with safe mesh resolution, preserving robot semantics.

    Args:
        path: Local URDF file. No network loading is performed.
        asset_root: Directory containing relative mesh paths and package folders.
            Defaults to the URDF file's parent directory.
        package_roots: Explicit ROS package-name to local-directory mapping.
        require_assets: Fail for a missing mesh if true; otherwise retain its URI
            and produce a diagnostic rather than pretending geometry was loaded.

    Returns:
        Structured robot description and an OpenCAD AssemblyTree with stable
        link/mesh occurrence IDs, plus warnings and missing-asset diagnostics.

    Raises:
        ValueError: Malformed/unsafe URDF, unsupported geometry, or invalid graph.
        FileNotFoundError: Missing referenced mesh when require_assets is true.
    """
    source = Path(path).resolve()
    raw = source.read_bytes()
    if len(raw) > MAX_URDF_BYTES:
        raise ValueError("URDF XML exceeds the allowed 8 MiB limit.")
    if _UNSAFE_XML.search(raw):
        raise ValueError("URDF DOCTYPE/ENTITY declarations are not permitted.")
    try:
        xml = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ValueError("Invalid URDF XML.") from exc
    if xml.tag != "robot" or not xml.get("name"):
        raise ValueError("URDF root must be <robot name='...'>.")
    root = Path(asset_root).resolve() if asset_root is not None else source.parent
    if not root.is_dir():
        raise ValueError(f"URDF asset root does not exist: {root}")
    mapped_roots = {key: Path(value).resolve() for key, value in (package_roots or {}).items()}
    if any(not item.is_dir() for item in mapped_roots.values()):
        raise ValueError("All explicit URDF package roots must exist.")
    warnings: list[str] = []
    missing: list[str] = []
    material_defs: dict[str, dict[str, Any]] = {}
    for material in xml.findall("material"):
        name = material.get("name")
        if name:
            definition: dict[str, Any] = {}
            color = material.find("color")
            if color is not None:
                definition["rgba"] = _numbers(color.get("rgba", ""), 4, "material.rgba")
            texture = material.find("texture")
            if texture is not None:
                definition["texture_uri"] = texture.get("filename")
            material_defs[name] = definition
    links: dict[str, RobotLink] = {}
    for element in xml.findall("link"):
        name = element.get("name")
        if not name or name in links:
            raise ValueError(f"URDF has an unnamed or duplicate link: {name!r}")
        visuals = [
            _parse_appearance(item, root, mapped_roots, material_defs, warnings, missing, require_assets)
            for item in element.findall("visual")
        ]
        collisions = [
            _parse_appearance(item, root, mapped_roots, material_defs, warnings, missing, require_assets)
            for item in element.findall("collision")
        ]
        inertial_xml = element.find("inertial")
        legacy_origin = element.find("origin")
        if legacy_origin is not None:
            warnings.append(f"Link {name!r} has a nonstandard top-level <origin>; preserved in metadata.")
        links[name] = RobotLink(
            name=name, visuals=visuals, collisions=collisions,
            inertial=_parse_inertial(inertial_xml) if inertial_xml is not None else None,
            metadata={"nonstandard_link_origin_xml": ET.tostring(legacy_origin, encoding="unicode")}
            if legacy_origin is not None else {},
        )
    joints: dict[str, RobotJoint] = {}
    for element in xml.findall("joint"):
        joint = _parse_joint(element)
        if joint.name in joints:
            raise ValueError(f"URDF has duplicate joint {joint.name!r}.")
        joints[joint.name] = joint
        if joint.joint_type in ("planar", "floating"):
            warnings.append(f"Joint {joint.name!r}: {joint.joint_type} motion is retained but not evaluated.")
        elif joint.joint_type == "continuous":
            warnings.append(f"Joint {joint.name!r}: continuous rotation retained; OpenCAD FK does not support it yet.")
        if joint.mimic is not None:
            warnings.append(f"Joint {joint.name!r}: mimic metadata retained; coupling not solved.")
    root_link = _validate_graph(links, joints)
    for joint in joints.values():
        if joint.mimic and joint.mimic["joint"] not in joints:
            warnings.append(f"Joint {joint.name!r} mimics a missing joint {joint.mimic['joint']!r}.")
    parent_joint = {joint.child: joint for joint in joints.values()}
    children: dict[str, list[str]] = {name: [] for name in links}
    for joint in joints.values():
        children[joint.parent].append(joint.child)
    components = {
        f"urdf:{name}": AssemblyComponent(
            id=f"urdf:{name}", name=name,
            child_ids=[f"urdf:{child}" for child in children[name]],
            geometry_refs=[f"urdf:{name}:visual:{index}" for index in range(len(link.visuals))],
            transform=parent_joint[name].origin if name in parent_joint else RigidTransform.identity(),
            metadata={
                "urdf_link": name,
                "parent_joint": parent_joint[name].name if name in parent_joint else None,
                "visual_count": len(link.visuals),
                "collision_count": len(link.collisions),
            },
        )
        for name, link in links.items()
    }
    assembly = AssemblyTree(
        id=f"urdf:{xml.get('name')}", name=xml.get("name"),
        root_ids=[f"urdf:{root_link}"], components=components,
        metadata={"format": "urdf", "source_sha256": hashlib.sha256(raw).hexdigest()},
    )
    passthrough_tags = {
        tag: [ET.tostring(elem, encoding="unicode") for elem in xml.findall(tag)]
        for tag in ("transmission", "gazebo")
        if xml.findall(tag)
    }
    license_path = next((str(source.parent / name) for name in ("LICENSE", "LICENSE.md")
                         if (source.parent / name).is_file()), None)
    description = RobotDescription(
        name=xml.get("name"), root_link=root_link, links=links, joints=joints,
        assembly=assembly,
        metadata={
            "source_path": str(source),
            "source_sha256": hashlib.sha256(raw).hexdigest(),
            "license_path": license_path,
            "global_materials": material_defs,
            "passthrough_xml": passthrough_tags,
            "units": {"transforms": "mm", "mesh_vertices": "m (URDF convention)",
                      "joint_angles": "rad", "prismatic_limits": "m",
                      "mass": "kg", "inertia": "kg*m^2"},
            "verification": {"geometry_loaded": not bool(missing),
                             "kinematics_supported": all(j.joint_type in ("fixed", "revolute", "prismatic") and j.mimic is None for j in joints.values()),
                             "constraints_unresolved": any(j.mimic is not None for j in joints.values()),
                             "dynamics_unverified": True},
        },
    )
    if any(name.startswith("closing_") for name in links):
        warnings.append("URDF includes closing_* frames; passive/closed-loop linkage constraints are not solved.")
    return URDFImportResult(robot=description, warnings=warnings, unresolved_assets=missing)
