"""Portable, geometry-only OpenCAD robot -> OpenIndustries v1 asset handoff.

OpenIndustries is Y-up and meters, whereas OpenCAD is Z-up and millimeters.
Each link/visual is preserved as an independent, stable part occurrence.
This is a static *neutral configuration* asset, NOT motion or physics validation.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path
from typing import Any, Iterable

from opencad.kinematics import compose_transforms
from opencad.kernel.core.models import RigidTransform
from opencad.robotics.urdf import RobotDescription, RobotGeometry

_MAX_VERTICES = 2_000_000
_MAX_STL_BYTES = 25 * 1024 * 1024


def _sha256(data: bytes) -> str:
    """Return a stable SHA-256 hex digest."""
    return hashlib.sha256(data).hexdigest()


def _rotated(point: tuple[float, float, float], q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Rotate a vector with a normalized OpenCAD XYZW quaternion."""
    x, y, z, w = q
    magnitude = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(magnitude) or magnitude < 1e-12:
        raise ValueError("Invalid robot frame quaternion.")
    x, y, z, w = (value / magnitude for value in (x, y, z, w))
    px, py, pz = point
    tx, ty, tz = (2*(y*pz - z*py), 2*(z*px - x*pz), 2*(x*py - y*px))
    return (
        px + w*tx + (y*tz - z*ty),
        py + w*ty + (z*tx - x*tz),
        pz + w*tz + (x*ty - y*tx),
    )


def _z_up_mm_to_y_up_m(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Convert a right-handed Z-up OpenCAD vector to right-handed Y-up OI."""
    return point[0]/1000.0, point[2]/1000.0, -point[1]/1000.0


def _geometry_mesh(geometry: RobotGeometry) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Generate local millimeter vertices/indices for supported URDF geometry."""
    if geometry.kind == "box":
        if not geometry.size_mm or any(v <= 0 for v in geometry.size_mm):
            raise ValueError("Box geometry requires positive size_mm.")
        x, y, z = (size/2 for size in geometry.size_mm)
        points = [
            (-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),
            (-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z),
        ]
        indices = [
            0,2,1,0,3,2,4,5,6,4,6,7,
            0,1,5,0,5,4,1,2,6,1,6,5,
            2,3,7,2,7,6,3,0,4,3,4,7,
        ]
        return points, indices
    if geometry.kind == "cylinder":
        if not geometry.radius_mm or not geometry.length_mm or min(geometry.radius_mm, geometry.length_mm) <= 0:
            raise ValueError("Cylinder requires positive radius and length.")
        n = 24
        points: list[tuple[float,float,float]] = []
        for z in (-geometry.length_mm/2, geometry.length_mm/2):
            points.extend((geometry.radius_mm*math.cos(2*math.pi*i/n),
                           geometry.radius_mm*math.sin(2*math.pi*i/n),z) for i in range(n))
        points.extend([(0,0,-geometry.length_mm/2),(0,0,geometry.length_mm/2)])
        faces: list[int] = []
        for i in range(n):
            k = (i+1)%n
            faces.extend((i,k,n+k,i,n+k,n+i,2*n,k,i,2*n+1,n+i,n+k))
        return points, faces
    if geometry.kind == "sphere":
        if not geometry.radius_mm or geometry.radius_mm <= 0:
            raise ValueError("Sphere requires positive radius.")
        n = 24
        rings = 12
        points = []
        for j in range(rings+1):
            polar=math.pi*j/rings
            for i in range(n):
                az=2*math.pi*i/n
                points.append((geometry.radius_mm*math.sin(polar)*math.cos(az),
                               geometry.radius_mm*math.sin(polar)*math.sin(az),
                               geometry.radius_mm*math.cos(polar)))
        faces = []
        for j in range(rings):
            for i in range(n):
                a=j*n+i; b=j*n+(i+1)%n; c=(j+1)*n+i; d=(j+1)*n+(i+1)%n
                if j: faces.extend((a,c,b))
                if j<rings-1: faces.extend((b,c,d))
        return points, faces
    if geometry.kind != "mesh":
        raise ValueError(f"Unsupported geometry {geometry.kind!r}.")
    if not geometry.resolved_path:
        raise ValueError("Cannot hand off unresolved STL geometry.")
    path=Path(geometry.resolved_path)
    if not path.is_file() or path.suffix.lower()!=".stl":
        raise ValueError(f"Only existing local STL geometry is supported: {path}")
    if path.stat().st_size > _MAX_STL_BYTES:
        raise ValueError("STL exceeds 25 MiB handoff budget.")
    data=path.read_bytes()
    points=[]
    faces=[]
    if len(data)>=84 and 84+50*struct.unpack_from("<I",data,80)[0]==len(data):
        n=struct.unpack_from("<I",data,80)[0]
        if n*3>_MAX_VERTICES: raise ValueError("STL exceeds vertex budget.")
        for i in range(n):
            values=struct.unpack_from("<9f",data,84+50*i+12)
            for j in range(0,9,3):
                points.append(tuple(float(v) for v in values[j:j+3]))
            faces.extend((3*i,3*i+1,3*i+2))
    else:
        try: lines=data.decode("utf-8").splitlines()
        except UnicodeDecodeError as exc: raise ValueError("Unrecognized ASCII/binary STL.") from exc
        for line in lines:
            pieces=line.split()
            if pieces and pieces[0].lower()=="vertex":
                if len(pieces)!=4: raise ValueError("Bad ASCII STL vertex.")
                points.append(tuple(float(v) for v in pieces[1:]))
                if len(points)>_MAX_VERTICES: raise ValueError("STL exceeds vertex budget.")
        if not points or len(points)%3:
            raise ValueError("ASCII STL lacks a complete triangle list.")
        faces=list(range(len(points)))
    multiplier=geometry.mesh_vertex_unit_to_mm
    if multiplier<=0 or not math.isfinite(multiplier):
        raise ValueError("Invalid STL unit scale.")
    scale=geometry.mesh_scale
    if any(not math.isfinite(s) or s<=0 for s in scale):
        raise ValueError("Invalid STL mesh scale.")
    result=[tuple(p[i]*multiplier*scale[i] for i in range(3)) for p in points]
    return result,faces


def robot_to_oi_asset(robot: RobotDescription, *, urdf_sha256: str) -> dict[str, Any]:
    """Produce an actual OpenIndustries Asset v1 bundle accepted by its CLI.

    Parts contain geometries baked at zero joint configuration. The hierarchy
    retains stable link identity; full joint semantics are in the sidecar.
    """
    if not robot.links or robot.root_link not in robot.links:
        raise ValueError("Robot has no valid root link.")
    joints_by_parent: dict[str, list] = {name: [] for name in robot.links}
    for joint in robot.joints.values():
        if joint.parent not in robot.links or joint.child not in robot.links:
            raise ValueError(f"Broken URDF joint {joint.name}")
        joints_by_parent[joint.parent].append(joint)
    part_entries=[]
    bounds_min=[float("inf")]*3
    bounds_max=[-float("inf")]*3
    seen_links: set[str]=set()
    vertex_budget=0

    def traverse(link_name: str, pose: RigidTransform, depth: int) -> dict[str, Any]:
        """Build the native OI asset-tree node with nested URDF link IDs."""
        nonlocal vertex_budget
        if depth>64 or link_name in seen_links:
            raise ValueError("Invalid/cyclic or excessive URDF link hierarchy.")
        seen_links.add(link_name)
        link=robot.links[link_name]
        part_ids=[]
        for i, visual in enumerate(link.visuals):
            points,faces=_geometry_mesh(visual.geometry)
            if not points: raise ValueError(f"No geometry in {link_name} visual {i}.")
            vertex_budget+=len(points)
            if vertex_budget>_MAX_VERTICES:
                raise ValueError("OpenIndustries 2M vertex limit exceeded.")
            visual_pose=compose_transforms(pose,visual.origin)
            vertices=[]
            for point in points:
                rotated=_rotated(point,visual_pose.rotation_quaternion_xyzw)
                moved=tuple(rotated[j]+visual_pose.translation_mm[j] for j in range(3))
                converted=_z_up_mm_to_y_up_m(moved)
                if not all(math.isfinite(v) for v in converted):
                    raise ValueError("Nonfinite world geometry in robot handoff.")
                vertices.extend(converted)
                for axis,v in enumerate(converted):
                    bounds_min[axis]=min(bounds_min[axis],v)
                    bounds_max[axis]=max(bounds_max[axis],v)
            occurrence=f"urdf:{link_name}:visual:{i}"
            part_ids.append(occurrence)
            rgb=visual.material.get("rgba")
            metadata={
                "representation":"OpenCAD URDF visual; static neutral configuration",
                "link_id":f"urdf:{link_name}",
                "visual_id":occurrence,
                "frame":"OpenCAD Z-up/mm to OpenIndustries Y-up/m",
                "kinematic_status":"neutral_unvalidated",
                "source_urdf_sha256":urdf_sha256,
            }
            if visual.geometry.kind=="mesh":
                metadata["mesh_sha256"]=_sha256(Path(visual.geometry.resolved_path).read_bytes())
            part={"id":occurrence,"name":visual.name or f"{link_name}:visual:{i}",
                  "vertices":vertices,"indices":faces,"metadata":metadata}
            if isinstance(rgb,(list,tuple)) and len(rgb)>=3:
                if all(isinstance(v,(float,int)) and math.isfinite(v) and 0<=v<=1 for v in rgb[:3]):
                    part["color"]=list(rgb[:3])
            part_entries.append(part)
        children=[]
        for joint in sorted(joints_by_parent[link_name],key=lambda item:item.name):
            child_pose=compose_transforms(pose,joint.origin)  # neutral joint q=0
            children.append(traverse(joint.child,child_pose,depth+1))
        return {"id":f"urdf:{link_name}","name":link_name,"partIds":part_ids,"children":children}

    hierarchy=traverse(robot.root_link,RigidTransform.identity(),0)
    if set(robot.links)!=seen_links:
        raise ValueError("Robot hierarchy is disconnected.")
    if not part_entries: raise ValueError("OpenIndustries requires visible robot geometry.")
    center=[(bounds_min[0]+bounds_max[0])/2,bounds_min[1],(bounds_min[2]+bounds_max[2])/2]
    for part in part_entries:
        part["vertices"]=[round(v-center[i%3],12) for i,v in enumerate(part["vertices"])]
    dims=[round(bounds_max[i]-bounds_min[i],12) for i in range(3)]
    if max(dims)<=0: raise ValueError("Robot has no physical extent.")
    asset={
        "schemaVersion":1,
        "id":f"opencad-robot-{urdf_sha256[:24]}",
        "name":robot.name,
        "source":{"kind":"generated","generator":"opencad-robot",
                  "filename":f"{robot.name}.urdf","digest":urdf_sha256},
        "units":"m","upAxis":"Y",
        "parts":part_entries,"hierarchy":hierarchy,
        "dimensions":dims,"originOffset":center,
        "warnings":[
            "Static neutral-position robot geometry only; OpenIndustries animation is not verified robot kinematics.",
            "Passive constraints, collision clearance, dynamics and hardware safety are NOT validated.",
        ],
    }
    return {"schemaVersion":1,"asset":asset}
