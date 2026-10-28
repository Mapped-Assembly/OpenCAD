"""URDF-compatible tree forward kinematics in native OpenCAD millimeters.

This evaluator deliberately rejects unsupported topology, mimic coupling, and
closed-loop constraints rather than approximating them as physically solved.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

from pydantic import BaseModel, Field

from opencad.kernel.core.models import RigidTransform
from opencad.kinematics import compose_transforms
from opencad.robotics.urdf import RobotDescription, _validate_graph


class RobotPoseResult(BaseModel):
    """World-space link and visual transforms at one joint configuration."""

    link_transforms: dict[str, RigidTransform]
    shape_transforms: dict[str, RigidTransform]
    warnings: list[str] = Field(default_factory=list)


def _quat(axis: tuple[float, float, float], angle: float) -> tuple[float, float, float, float]:
    """Create a normalized XYZW rotation about a URDF joint-local axis."""
    length = math.sqrt(sum(component * component for component in axis))
    if length <= 1e-12:
        raise ValueError("Moving URDF joint axis must be non-zero.")
    s = math.sin(angle / 2.0) / length
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def evaluate_robot_poses(
    robot: RobotDescription, positions: Mapping[str, float] | None = None
) -> RobotPoseResult:
    """Evaluate a URDF tree at raw joint values (radians or meters).

    Args:
        robot: Canonical imported or programmatically authored URDF robot.
        positions: Joint names mapped to radians for revolute/continuous, and
            meters for prismatic. Fixed joints must remain zero.

    Returns:
        Link and per-visual occurrence world transforms, with stable IDs.

    Raises:
        ValueError: Bad positions, unsupported joint types, mimic couplings,
            inferred closed-loop mechanisms, or broken assembly topology.
    """
    _validate_graph(robot.links, robot.joints)
    positions = dict(positions or {})
    unknown = set(positions) - set(robot.joints)
    if unknown:
        raise ValueError(f"Unknown robot joint values: {sorted(unknown)}")
    if robot.metadata.get("verification", {}).get("constraints_unresolved"):
        raise ValueError("Robot has unresolved passive or closed-loop constraints; FK is not certified.")
    for joint in robot.joints.values():
        if joint.joint_type in ("planar", "floating") or joint.mimic is not None:
            raise ValueError(f"Cannot evaluate unsupported/mimic joint {joint.name!r}.")
    children: dict[str, list] = {name: [] for name in robot.links}
    for joint in robot.joints.values():
        children[joint.parent].append(joint)
    links: dict[str, RigidTransform] = {}
    shapes: dict[str, RigidTransform] = {}
    warnings: list[str] = []

    def visit(name: str, world: RigidTransform) -> None:
        """Recursively propagate joint-local transforms through a URDF tree."""
        links[name] = world
        for i, visual in enumerate(robot.links[name].visuals):
            shapes[f"urdf:{name}:visual:{i}"] = compose_transforms(world, visual.origin)
        for joint in children[name]:
            q = positions.get(joint.name, 0.0)
            if not math.isfinite(q):
                raise ValueError(f"Nonfinite position for joint {joint.name!r}.")
            if joint.joint_type == "fixed":
                if q != 0:
                    raise ValueError(f"Fixed joint {joint.name!r} cannot move.")
                motion = RigidTransform.identity()
            elif joint.joint_type == "prismatic":
                if sum(x*x for x in joint.axis) <= 1e-24:
                    raise ValueError(f"Joint {joint.name!r} has a zero axis.")
                length = math.sqrt(sum(x*x for x in joint.axis))
                motion = RigidTransform(
                    translation_mm=tuple(1000.0 * q * axis / length for axis in joint.axis)
                )
            else:
                motion = RigidTransform(rotation_quaternion_xyzw=_quat(joint.axis, q))
            if joint.joint_type in ("revolute", "prismatic"):
                lower, upper = joint.limits.get("lower"), joint.limits.get("upper")
                if (lower is not None and q < lower - 1e-9) or (upper is not None and q > upper + 1e-9):
                    raise ValueError(f"Joint {joint.name!r} exceeds its defined travel limits.")
            child_world = compose_transforms(compose_transforms(world, joint.origin), motion)
            visit(joint.child, child_world)

    visit(robot.root_link, RigidTransform.identity())
    if any(link.inertial is None for link in robot.links.values()):
        warnings.append("Some links lack mass/inertial data; dynamics are not validated.")
    if any(not link.collisions for link in robot.links.values()):
        warnings.append("Some links lack collision geometry; collision clearance is not validated.")
    warnings.append("Pose preview is tree forward kinematics, not physics or hardware validation.")
    return RobotPoseResult(link_transforms=links, shape_transforms=shapes, warnings=warnings)
