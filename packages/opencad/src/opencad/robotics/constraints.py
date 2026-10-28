"""Typed constraints and narrowly scoped planar four-bar verification.

URDF's link/joint hierarchy remains a tree. Extra constraints live in a
versioned OpenCAD metadata sidecar and must never be silently exported as
standard URDF tags or mistaken for a physics solver.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, model_validator

from opencad.robotics.urdf import RobotDescription

SIDECAR_KEY = "mechanism_constraints_v1"


class ConstraintModel(BaseModel):
    """Strict immutable-by-convention schema for versioned mechanism metadata."""

    model_config = ConfigDict(extra="forbid")


class FrameAnchor(ConstraintModel):
    """Point fixed to a named rigid link, in its local frame and millimeters."""

    link: str = Field(min_length=1)
    xyz_mm: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, 0.0)


class LoopClosure(ConstraintModel):
    """Coincident-point closure between two link-local anchors; NOT auto-solved."""

    id: str = Field(min_length=1)
    first: FrameAnchor
    second: FrameAnchor


class JointRole(ConstraintModel):
    """Explicit actuator/passive classification, never guessed from URDF type."""

    joint: str = Field(min_length=1)
    role: Literal["driven", "passive", "fixed"]
    evidence: str | None = None


class MimicConstraint(ConstraintModel):
    """A directed affine joint-angle relationship, not automatically evaluated."""

    joint: str = Field(min_length=1)
    driver: str = Field(min_length=1)
    multiplier: FiniteFloat = 1.0
    offset: FiniteFloat = 0.0


class SpringConstraint(ConstraintModel):
    """Optional spring metadata; values are NOT inferred from mesh appearance.

    No forces are computed by the four-bar position solver. SI units are
    explicit so upstream measured data cannot be confused with millimeters.
    """

    id: str = Field(min_length=1)
    first: FrameAnchor
    second: FrameAnchor
    rest_length_mm: FiniteFloat | None = Field(default=None, ge=0)
    stiffness_N_per_m: FiniteFloat | None = Field(default=None, ge=0)
    damping_N_s_per_m: FiniteFloat | None = Field(default=None, ge=0)
    provenance: str | None = None


class PlanarFourBar(ConstraintModel):
    """One planar ground–crank–coupler–rocker closure, lengths in millimeters.

    Ground pivots are A=(0,0), D=(ground_mm,0). The crank rotates about A,
    the rocker about D, and the coupler connects their distal endpoints.
    This model does not encode or solve a complete URDF joint tree.
    """

    id: str = Field(min_length=1)
    driver_joint: str = Field(min_length=1)
    ground_mm: FiniteFloat = Field(gt=0)
    crank_mm: FiniteFloat = Field(gt=0)
    coupler_mm: FiniteFloat = Field(gt=0)
    rocker_mm: FiniteFloat = Field(gt=0)
    assembly_branch: Literal["upper", "lower"] = "upper"
    provenance: str | None = None


class MechanismConstraintsV1(ConstraintModel):
    """Versioned additional relationship graph, separate from canonical URDF."""

    version: Literal[1] = 1
    joint_roles: list[JointRole] = Field(default_factory=list)
    mimic: list[MimicConstraint] = Field(default_factory=list)
    closures: list[LoopClosure] = Field(default_factory=list)
    springs: list[SpringConstraint] = Field(default_factory=list)
    planar_four_bars: list[PlanarFourBar] = Field(default_factory=list)

    @model_validator(mode="after")
    def reject_duplicate_ids(self) -> "MechanismConstraintsV1":
        """Reject ambiguous definition IDs and conflicting joint role assignments."""
        identifiers = [item.id for item in (*self.closures, *self.springs, *self.planar_four_bars)]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("Mechanism constraint IDs must be unique.")
        names = [item.joint for item in self.joint_roles]
        if len(names) != len(set(names)):
            raise ValueError("A joint may have only one declared role.")
        driven = [item.joint for item in self.mimic]
        if len(driven) != len(set(driven)):
            raise ValueError("A joint may have only one mimic driver.")
        return self


class MechanismDiagnostic(ConstraintModel):
    """Mechanism capability assessment, not evidence of dynamic correctness."""

    state: Literal["tree_only", "constraints_unresolved"]
    unresolved: list[str]
    possible_closure_markers: list[str]
    explicit_passive_joints: list[str]
    supported_solvers: list[str]
    physical_validation: Literal["not_performed"] = "not_performed"


class FourBarSolution(ConstraintModel):
    """A solved planar position with quantified closure residuals in mm."""

    model_id: str
    input_angle_rad: FiniteFloat
    crank_endpoint_mm: tuple[FiniteFloat, FiniteFloat]
    coupler_rocker_endpoint_mm: tuple[FiniteFloat, FiniteFloat]
    coupler_angle_rad: FiniteFloat
    rocker_angle_rad: FiniteFloat
    residual_mm: FiniteFloat = Field(ge=0)
    status: Literal["solved"] = "solved"
    physical_validation: Literal["not_performed"] = "not_performed"


def _validate_references(robot: RobotDescription, model: MechanismConstraintsV1) -> None:
    """Check constraints against real link/joint IDs without changing the URDF tree."""
    joint_names = set(robot.joints)
    link_names = set(robot.links)
    for role in model.joint_roles:
        if role.joint not in joint_names:
            raise ValueError(f"Unknown joint role reference: {role.joint}")
        if role.role == "fixed" and robot.joints[role.joint].joint_type != "fixed":
            raise ValueError(f"Joint {role.joint} declared fixed but URDF joint type is moving.")
    for mimic in model.mimic:
        if mimic.joint not in joint_names or mimic.driver not in joint_names:
            raise ValueError("Mimic relationship references an unknown joint.")
        if mimic.joint == mimic.driver:
            raise ValueError("A joint cannot mimic itself.")
    # Repeated mimic references form a directed graph; cycles are not solvable.
    by_joint = {item.joint: item.driver for item in model.mimic}
    for start in by_joint:
        visited: set[str] = set()
        cursor = start
        while cursor in by_joint:
            if cursor in visited:
                raise ValueError("Mimic relationship contains a directed cycle.")
            visited.add(cursor)
            cursor = by_joint[cursor]
    for relation in (*model.closures, *model.springs):
        if relation.first.link not in link_names or relation.second.link not in link_names:
            raise ValueError(f"Constraint {relation.id} refers to an unknown robot link.")
    for fourbar in model.planar_four_bars:
        if fourbar.driver_joint not in joint_names:
            raise ValueError(f"Four-bar driver {fourbar.driver_joint!r} does not exist.")
        if robot.joints[fourbar.driver_joint].joint_type not in ("revolute", "continuous"):
            raise ValueError("Four-bar drivers must use a rotational URDF joint.")
        roles = {item.joint: item.role for item in model.joint_roles}
        if roles.get(fourbar.driver_joint) != "driven":
            raise ValueError("A four-bar driver requires an explicit 'driven' joint role.")


def read_mechanism_constraints(robot: RobotDescription) -> MechanismConstraintsV1 | None:
    """Read and validate an optional constraints sidecar from robot metadata."""
    raw = robot.metadata.get(SIDECAR_KEY)
    if raw is None:
        return None
    model = MechanismConstraintsV1.model_validate(raw)
    _validate_references(robot, model)
    return model


def attach_mechanism_constraints(
    robot: RobotDescription,
    model: MechanismConstraintsV1,
) -> RobotDescription:
    """Return a copy with a validated sidecar and fail-closed motion flags."""
    _validate_references(robot, model)
    updated = robot.model_copy(deep=True)
    updated.metadata[SIDECAR_KEY] = model.model_dump(mode="json")
    check = diagnose_mechanism(updated)
    verification = dict(updated.metadata.get("verification", {}))
    verification["constraints_unresolved"] = check.state == "constraints_unresolved"
    verification["dynamics_unverified"] = True
    updated.metadata["verification"] = verification
    return updated


def diagnose_mechanism(robot: RobotDescription) -> MechanismDiagnostic:
    """Report evidence for unresolved mechanisms without guessing dimensions.

    A link named closing_* is a *possible* closure marker only. URDF alone
    cannot determine whether Petoi's springs/parallel linkage physically close.
    """
    model = read_mechanism_constraints(robot)
    notes: list[str] = []
    passive = [entry.joint for entry in model.joint_roles if entry.role == "passive"] if model else []
    closure_names = sorted(name for name in robot.links if name.startswith("closing_"))
    if closure_names:
        notes.append("Possible closure markers in URDF link names; geometric loop closure is not established.")
    for joint in robot.joints.values():
        if joint.mimic is not None:
            notes.append(f"Mimic relation for {joint.name} requires coupling evaluation.")
    if model:
        if model.closures:
            notes.append(f"{len(model.closures)} explicit point-closure constraints are not generically solved.")
        if model.mimic:
            notes.append(f"{len(model.mimic)} explicit mimic relationships are not generically solved.")
        if passive:
            notes.append(f"{len(passive)} joints are explicitly identified as passive.")
        if model.planar_four_bars:
            notes.append("Analytic planar four-bar position solver is available only for declared dimensions/branch.")
        if model.springs:
            notes.append(f"{len(model.springs)} springs have metadata only; no forces or dynamics are solved.")
    if robot.metadata.get("verification", {}).get("constraints_unresolved") and not notes:
        notes.append("Source marked constraints unresolved without a solvable mechanism definition.")
    return MechanismDiagnostic(
        state="constraints_unresolved" if notes else "tree_only",
        unresolved=notes,
        possible_closure_markers=closure_names,
        explicit_passive_joints=passive,
        supported_solvers=["planar_four_bar_position"] if model and model.planar_four_bars else [],
    )


def solve_planar_four_bar(
    model: PlanarFourBar, input_angle_rad: float, *, tolerance_mm: float = 1e-7
) -> FourBarSolution:
    """Solve one non-singular, attainable four-bar position by circle intersection.

    Raises ValueError for unreachable configurations, ambiguous collinear
    tangent positions, non-finite input, or residual above the stated tolerance.
    The upper/lower branches are signed relative to B→D, not fixed global Y.
    """
    angle = float(input_angle_rad)
    tolerance = float(tolerance_mm)
    if not math.isfinite(angle) or not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Four-bar input angle and positive tolerance must be finite.")
    g, a, b, c = model.ground_mm, model.crank_mm, model.coupler_mm, model.rocker_mm
    bx, by = a * math.cos(angle), a * math.sin(angle)
    dx, dy = g, 0.0
    ux, uy = dx - bx, dy - by
    separation = math.hypot(ux, uy)
    if separation <= tolerance:
        raise ValueError("Four-bar pivots coincide; the assembly branch is indeterminate.")
    if separation > b + c + tolerance or separation < abs(b - c) - tolerance:
        raise ValueError("Four-bar configuration is unreachable at this crank angle.")
    projection = (b*b - c*c + separation*separation) / (2 * separation)
    height_sq = b*b - projection*projection
    if height_sq <= tolerance*tolerance:
        raise ValueError("Four-bar assembly is singular or unreachable at this angle.")
    height = math.sqrt(height_sq)
    unit_x, unit_y = ux / separation, uy / separation
    sign = 1.0 if model.assembly_branch == "upper" else -1.0
    cx = bx + projection * unit_x - sign * height * unit_y
    cy = by + projection * unit_y + sign * height * unit_x
    errors = (
        abs(math.hypot(bx, by) - a),
        abs(math.hypot(cx - bx, cy - by) - b),
        abs(math.hypot(cx - dx, cy - dy) - c),
    )
    residual = max(errors)
    if residual > tolerance:
        raise ValueError(f"Four-bar loop-closure residual {residual} mm exceeds {tolerance} mm.")
    return FourBarSolution(
        model_id=model.id,
        input_angle_rad=angle,
        crank_endpoint_mm=(bx, by),
        coupler_rocker_endpoint_mm=(cx, cy),
        coupler_angle_rad=math.atan2(cy - by, cx - bx),
        rocker_angle_rad=math.atan2(cy, cx - g),
        residual_mm=residual,
    )


def solve_four_bar_sweep(
    model: PlanarFourBar, inputs_rad: list[float], *, tolerance_mm: float = 1e-7
) -> list[FourBarSolution]:
    """Solve every requested configuration, failing atomically on any invalid one."""
    if not inputs_rad:
        raise ValueError("A four-bar motion sweep requires at least one angle.")
    return [
        solve_planar_four_bar(model, angle, tolerance_mm=tolerance_mm)
        for angle in inputs_rad
    ]
