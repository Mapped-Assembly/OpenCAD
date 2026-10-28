"""Create portable ROS2 description packages and versioned OpenIndustries handoffs."""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from opencad.robotics.export import export_urdf
from opencad.robotics.oi_handoff import robot_to_oi_asset
from opencad.robotics.urdf import RobotDescription

_ROS_PACKAGE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


class ROSPackageResult(BaseModel):
    """Output paths for a ROS2 package and static OpenIndustries asset."""

    package_directory: str
    urdf_path: str
    manifest_path: str
    oi_asset_path: str
    mesh_paths: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


def _sha(path: Path) -> str:
    """Return a stable content hash rather than a host-specific path."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _handoff(
    robot: RobotDescription, urdf: Path, package: str, mesh_paths: list[Path],
    oi_bundle: dict[str, Any] | None,
) -> dict[str, Any]:
    """Build a path-independent sidecar retaining links, joints and evidence."""
    from opencad.robotics.constraints import diagnose_mechanism

    hashes = {p.name: _sha(p) for p in sorted(mesh_paths)}
    incoming = {joint.child: joint for joint in robot.joints.values()}
    links: dict[str, Any] = {}
    for name, link in sorted(robot.links.items()):
        parent = incoming.get(name)
        records: dict[str, dict[str, Any]] = {"visual": {}, "collision": {}}
        for tag, collection in (("visual", link.visuals), ("collision", link.collisions)):
            for index, occurrence in enumerate(collection):
                geo = occurrence.geometry
                geometry = geo.model_dump(mode="json", exclude={"resolved_path"})
                if geo.kind == "mesh":
                    if not geo.resolved_path:
                        raise ValueError(f"Unresolved {name} {tag} mesh.")
                    digest = _sha(Path(geo.resolved_path))
                    file = f"{digest}.stl"
                    if file not in hashes:
                        raise ValueError(f"Missing content-addressed ROS asset {file}")
                    geometry["source_uri"] = f"package://{package}/meshes/{file}"
                    geometry["sha256"] = digest
                records[tag][f"urdf:{name}:{tag}:{index}"] = {
                    "name": occurrence.name,
                    "origin": occurrence.origin.model_dump(mode="json"),
                    "geometry": geometry,
                    "material": occurrence.material,
                }
        links[name] = {
            "id": f"urdf:{name}",
            "parent_joint": parent.name if parent else None,
            "children": sorted(j.child for j in robot.joints.values() if j.parent == name),
            "visuals": records["visual"], "collisions": records["collision"],
            "inertial": link.inertial.model_dump(mode="json") if link.inertial else None,
        }
    diagnosis = diagnose_mechanism(robot)
    unresolved = (
        diagnosis.state == "constraints_unresolved"
        or bool(robot.metadata.get("verification", {}).get("constraints_unresolved"))
    )
    asset_id = oi_bundle["asset"]["id"] if oi_bundle is not None else None
    return {
        "schema": "opencad.robot_handoff.v1", "version": 1,
        "robot": robot.name, "root_link": robot.root_link,
        "source": {
            "format": "URDF", "ros_package": package, "urdf": f"urdf/{urdf.name}",
            "urdf_sha256": _sha(urdf),
            "source_urdf_sha256": robot.metadata.get("source_sha256"),
        },
        "units": {
            "opencad": {"length": "mm", "up_axis": "Z", "quaternion": "XYZW"},
            "urdf": {"length": "m", "up_axis": "Z", "angles": "rad"},
            "openindustries": {"length": "m", "up_axis": "Y",
                               "right_handed_conversion": "(x,z,-y)"},
        },
        "links": links,
        "joints": {name: joint.model_dump(mode="json") for name, joint in sorted(robot.joints.items())},
        "mesh_sha256": hashes,
        "openindustries": {
            "asset_bundle": "oi.asset.json" if oi_bundle else None,
            "asset_id": asset_id,
            "part_ids": sorted(p["id"] for p in oi_bundle["asset"]["parts"]) if oi_bundle else [],
            "asset_sha256": hashlib.sha256(
                json.dumps(oi_bundle, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest() if oi_bundle else None,
            "status": "static_neutral_geometry_only",
        },
        "verification": {
            "structural_handoff": "exported",
            "kinematic_agreement": "not_checked_by_handoff",
            "constraints_unresolved": unresolved,
            "dynamics_verified": False,
            "collision_clearance_verified": False,
            "hardware_validated": False,
            "physical_validation": "not_performed",
            "diagnostic": diagnosis.model_dump(mode="json"),
        },
    }


def export_ros_package(
    robot: RobotDescription,
    output_directory: str | Path,
    *,
    package_name: str | None = None,
    include_oi: bool = True,
) -> ROSPackageResult:
    """Export URDF, all local mesh assets and a typed OI static asset bundle.

    The destination must not already exist. Internal paths are content-addressed,
    and no absolute author workstation paths are included in output metadata.
    """
    package = package_name or re.sub(r"[^a-z0-9]+", "_", robot.name.lower()).strip("_")
    if not _ROS_PACKAGE_NAME.fullmatch(package):
        raise ValueError("ROS package_name must be lowercase snake_case, starting with a letter.")
    destination = Path(output_directory).resolve()
    if destination.exists():
        raise FileExistsError(f"ROS package destination already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".opencad-ros-", dir=destination.parent) as temp:
        root = Path(temp) / package
        urdf_dir = root / "urdf"
        urdf_dir.mkdir(parents=True)
        export = export_urdf(robot, urdf_dir / f"{robot.name}.urdf")
        generated = Path(export.urdf_path)
        generated_meshes = urdf_dir / "meshes"
        meshes = root / "meshes"
        meshes.mkdir()
        staged: list[Path] = []
        for source in sorted(generated_meshes.glob("*.stl")):
            target = meshes / source.name
            shutil.move(str(source), str(target))
            staged.append(target)
        generated_meshes.rmdir()
        tree = ET.parse(generated)
        for mesh in tree.findall(".//mesh"):
            uri = mesh.get("filename", "")
            if not uri.startswith("meshes/"):
                raise ValueError(f"Unexpected URDF mesh reference {uri!r}")
            digest_file = uri.removeprefix("meshes/")
            if not re.fullmatch(r"[0-9a-f]{64}\.stl", digest_file):
                raise ValueError("URDF geometry is not content-addressed.")
            if not (meshes / digest_file).is_file():
                raise ValueError(f"Missing ROS package mesh {digest_file}")
            mesh.set("filename", f"package://{package}/meshes/{digest_file}")
        ET.indent(tree, space="  ")
        tree.write(generated, encoding="utf-8", xml_declaration=True)
        # The temporary exporter manifest may include workstation-local paths.
        Path(export.manifest_path).unlink()
        oi_bundle = robot_to_oi_asset(robot, urdf_sha256=_sha(generated)) if include_oi else None
        if oi_bundle is not None:
            (root / "oi.asset.json").write_text(
                json.dumps(oi_bundle, sort_keys=True, indent=2) + "\n", encoding="utf-8"
            )
        manifest = _handoff(robot, generated, package, staged, oi_bundle)
        (root / "robot.handoff.json").write_text(
            json.dumps(manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8"
        )
        doc = ET.Element("package", {"format": "3"})
        for tag, value in (
            ("name", package), ("version", "0.1.0"),
            ("description", f"OpenCAD export of {robot.name}; no validated dynamics"),
            ("license", "NOASSERTION"), ("buildtool_depend", "ament_cmake"),
            ("exec_depend", "robot_state_publisher"),
        ):
            ET.SubElement(doc, tag).text = value
        ET.SubElement(doc, "maintainer", {"email": "replace-me@example.invalid"}).text = "Replace maintainer"
        ET.SubElement(ET.SubElement(doc, "export"), "build_type").text = "ament_cmake"
        ET.indent(doc, space="  ")
        ET.ElementTree(doc).write(root / "package.xml", encoding="utf-8", xml_declaration=True)
        (root / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.8)\n"
            f"project({package})\n"
            "find_package(ament_cmake REQUIRED)\n"
            "install(DIRECTORY urdf meshes DESTINATION share/" + package + ")\n"
            "ament_package()\n",
            encoding="utf-8",
        )
        shutil.move(str(root), str(destination))
    return ROSPackageResult(
        package_directory=str(destination),
        urdf_path=str(destination / "urdf" / f"{robot.name}.urdf"),
        manifest_path=str(destination / "robot.handoff.json"),
        oi_asset_path=str(destination / "oi.asset.json") if include_oi else "",
        mesh_paths=[str(destination / "meshes" / p.name) for p in staged],
        warnings=[
            "Replace template ROS maintainer and license declarations before distribution.",
            "OI preview is static neutral geometry; robot kinematics and physical safety are unverified.",
        ],
    )
