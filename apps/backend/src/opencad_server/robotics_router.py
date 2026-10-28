"""Stateless, bounded URDF bundle import/export and pose preview routes.

No uploaded robot files persist between requests. Browser export repeats the
original source bundle; the server never trusts temporary mesh paths returned
to the client. ZIP members are validated before extraction.
"""
from __future__ import annotations

import io
import math
import re
import struct
import tempfile
import zipfile
from pathlib import Path, PurePosixPath

from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from opencad.robotics import RobotDescription, export_urdf, import_urdf
from opencad.robotics.export import _validate_robot
from opencad.robotics.poses import RobotPoseResult, evaluate_robot_poses

router = APIRouter()
_MAX_BUNDLE_BYTES = 24 * 1024 * 1024
_MAX_UNCOMPRESSED = 80 * 1024 * 1024
_MAX_TRIANGLES = 250_000


class RobotImportResponse(BaseModel):
    """Browser-ready import data and explicit export/motion capability flags."""

    robot: RobotDescription
    meshes: list[dict]
    warnings: list[str] = Field(default_factory=list)
    export_enabled: bool = False
    export_blocker: str | None = None
    motion_enabled: bool = False


class RobotPoseRequest(BaseModel):
    """Typed forward-kinematic request, with joint values in URDF units."""

    robot: RobotDescription
    positions: dict[str, float] = Field(default_factory=dict)


async def _bounded_request(request: Request) -> bytes:
    """Read a file body incrementally, rejecting oversized uploads early."""
    total = 0
    chunks: list[bytes] = []
    async for chunk in request.stream():
        total += len(chunk)
        if total > _MAX_BUNDLE_BYTES:
            raise HTTPException(status_code=413, detail="Robot bundle exceeds 24 MiB.")
        chunks.append(chunk)
    if total == 0:
        raise HTTPException(status_code=400, detail="Empty robot upload.")
    return b"".join(chunks)


def _unpack(data: bytes, filename: str, directory: Path) -> Path:
    """Extract a single safe robot URDF or a local ZIP with bounded members."""
    if filename.lower().endswith(".urdf"):
        source = directory / "robot.urdf"
        source.write_bytes(data)
        return source
    if not filename.lower().endswith(".zip"):
        raise ValueError("Robot upload must be .urdf or .zip.")
    with zipfile.ZipFile(io.BytesIO(data)) as bundle:
        members = bundle.infolist()
        if len(members) > 1500:
            raise ValueError("Robot ZIP has too many entries.")
        if sum(item.file_size for item in members) > _MAX_UNCOMPRESSED:
            raise ValueError("Robot ZIP has too much uncompressed data.")
        urdfs: list[Path] = []
        seen: set[str] = set()
        for item in members:
            name = item.filename
            parts = PurePosixPath(name).parts
            if (
                not name or name.startswith(("/", "~")) or "\\" in name or
                ":" in name or ".." in parts or not parts or
                any(part in ("", ".") for part in name.rstrip("/").split("/")) or
                (item.external_attr >> 16) & 0o170000 == 0o120000
            ):
                raise ValueError(f"Unsafe ZIP entry: {name!r}.")
            if item.is_dir():
                continue
            if name in seen:
                raise ValueError(f"Duplicate ZIP path: {name!r}.")
            seen.add(name)
            if Path(name).suffix.lower() not in {".urdf", ".stl", ".xml"}:
                raise ValueError(f"Unsupported ZIP asset {name!r}; only URDF/STL/XML permitted.")
            path = directory.joinpath(*parts)
            if not path.resolve().is_relative_to(directory.resolve()):
                raise ValueError("ZIP member escapes extraction root.")
            path.parent.mkdir(parents=True, exist_ok=True)
            with bundle.open(item) as source, path.open("wb") as target:
                remaining = item.file_size
                while remaining > 0:
                    chunk = source.read(min(1024 * 1024, remaining))
                    if not chunk:
                        raise ValueError("Truncated ZIP member.")
                    target.write(chunk)
                    remaining -= len(chunk)
            if path.suffix.lower() == ".urdf":
                urdfs.append(path)
        if len(urdfs) != 1:
            raise ValueError("ZIP must contain exactly one .urdf file.")
        return urdfs[0]


def _triangles(path: Path, scale: tuple[float, float, float]) -> tuple[list[float], list[int]]:
    """Decode bounded ASCII or binary STL, retaining its native geometry units."""
    data = path.read_bytes()
    vertices: list[float] = []
    faces: list[int] = []
    if len(data) >= 84 and 84 + 50 * struct.unpack_from("<I", data, 80)[0] == len(data):
        count = struct.unpack_from("<I", data, 80)[0]
        if count > _MAX_TRIANGLES:
            raise ValueError(f"STL exceeds {_MAX_TRIANGLES} triangles: {path.name}")
        for n in range(count):
            offset = 84 + 50 * n + 12
            xyz = struct.unpack_from("<9f", data, offset)
            if not all(math.isfinite(v) for v in xyz):
                raise ValueError("Nonfinite STL vertex.")
            vertices.extend(xyz[i] * scale[i % 3] for i in range(9))
            base = 3 * n
            faces.extend((base, base + 1, base + 2))
    else:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("STL is not valid binary or UTF-8 ASCII.") from exc
        coords = re.findall(
            r"\bvertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)", text
        )
        if len(coords) > _MAX_TRIANGLES * 3 or len(coords) % 3:
            raise ValueError("Invalid or oversized ASCII STL triangle list.")
        for triple in coords:
            vertex = tuple(float(v) for v in triple)
            if not all(math.isfinite(v) for v in vertex):
                raise ValueError("Nonfinite STL vertex.")
            vertices.extend(vertex[i] * scale[i] for i in range(3))
        faces = list(range(len(coords)))
    return vertices, faces


def _view_meshes(robot: RobotDescription) -> list[dict]:
    """Provide triangle data for referenced STL visuals, once per occurrence."""
    data: list[dict] = []
    triangle_count = 0
    for link in robot.links.values():
        for n, visual in enumerate(link.visuals):
            geometry = visual.geometry
            if geometry.kind != "mesh" or not geometry.resolved_path:
                continue
            scale = tuple(
                v * geometry.mesh_vertex_unit_to_mm for v in geometry.mesh_scale
            )
            vertices, faces = _triangles(Path(geometry.resolved_path), scale)
            triangle_count += len(faces) // 3
            if triangle_count > _MAX_TRIANGLES:
                raise ValueError("Robot preview exceeds total triangle budget.")
            data.append({
                "shapeId": f"urdf:{link.name}:visual:{n}",
                "vertices": vertices, "faces": faces,
                "name": f"{link.name} visual {n}",
            })
    return data


def _import_bundle(data: bytes, filename: str, directory: Path) -> RobotDescription:
    """Load a safe upload, enforcing real referenced assets before GUI preview."""
    source = _unpack(data, filename, directory)
    return import_urdf(source, asset_root=source.parent).robot


@router.post("/import", response_model=RobotImportResponse)
async def import_robot(request: Request, filename: str = Query(min_length=1)) -> RobotImportResponse:
    """Import a local URDF bundle and return typed robot plus preview meshes."""
    data = await _bounded_request(request)
    try:
        with tempfile.TemporaryDirectory(prefix="opencad-robot-") as temp:
            robot = _import_bundle(data, filename, Path(temp))
            meshes = _view_meshes(robot)
            warnings: list[str] = [
                "Imported robot descriptions are not physics-verified.",
            ]
            export_blocker: str | None = None
            try:
                _validate_robot(robot)
            except ValueError as exc:
                export_blocker = str(exc)
            motion_enabled = True
            try:
                evaluate_robot_poses(robot)
            except ValueError as exc:
                motion_enabled = False
                warnings.append(f"Joint motion disabled: {exc}")
            if any(not link.collisions for link in robot.links.values()):
                warnings.append("Some links lack collision geometry.")
            if any(link.inertial is None for link in robot.links.values()):
                warnings.append("Some links lack inertial data.")
            return RobotImportResponse(
                robot=robot, meshes=meshes, warnings=warnings,
                export_enabled=export_blocker is None,
                export_blocker=export_blocker, motion_enabled=motion_enabled,
            )
    except (ValueError, FileNotFoundError, zipfile.BadZipFile) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@router.post("/poses", response_model=RobotPoseResult)
def robot_poses(request: RobotPoseRequest) -> RobotPoseResult:
    """Evaluate a supported robot at explicit joint positions."""
    try:
        return evaluate_robot_poses(request.robot, request.positions)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@router.post("/export")
async def export_robot(request: Request, filename: str = Query(min_length=1)) -> Response:
    """Reimport the original bundle and return portable URDF+STL ZIP.

    No user-supplied temporary filesystem paths are trusted on export.
    """
    data = await _bounded_request(request)
    try:
        with tempfile.TemporaryDirectory(prefix="opencad-robot-export-") as temp:
            directory = Path(temp)
            robot = _import_bundle(data, filename, directory / "source")
            output = directory / "output"
            output.mkdir()
            result = export_urdf(robot, output / "robot.urdf")
            target = io.BytesIO()
            with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as archive:
                archive.write(result.urdf_path, "robot.urdf")
                archive.write(result.manifest_path, "robot.opencad.json")
                for mesh in result.mesh_paths:
                    archive.write(mesh, f"meshes/{Path(mesh).name}")
            return Response(
                target.getvalue(), media_type="application/zip",
                headers={"Content-Disposition": 'attachment; filename="opencad-robot.zip"'},
            )
    except (ValueError, FileNotFoundError, zipfile.BadZipFile) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
