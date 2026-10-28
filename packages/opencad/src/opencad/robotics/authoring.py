"""Programmatic robot assembly authoring on top of OpenCAD's existing models."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping, Sequence

from opencad.assembly import AssemblyComponent, AssemblyTree
from opencad.kernel.core.models import RigidTransform
from opencad.robotics.export import _name
from opencad.robotics.urdf import (
    RobotDescription,
    RobotGeometry,
    RobotInertial,
    RobotJoint,
    RobotLink,
    RobotVisual,
    URDFJointType,
    _validate_graph,
)

if TYPE_CHECKING:
    from opencad.part import Part


class RobotBuilder:
    """Build a typed robot description without writing or importing URDF XML.

    OpenCAD Parts can be exported as distinct link-local STL occurrences.
    No inertial values or collider geometry are guessed: both are opt-in.
    """

    def __init__(self, name: str, *, metadata: Mapping[str, Any] | None = None) -> None:
        """Initialize a robot with an explicit, stable name and metadata."""
        if not name:
            raise ValueError("Robot name is required.")
        self.name = name
        self._links: dict[str, RobotLink] = {}
        self._joints: dict[str, RobotJoint] = {}
        self._metadata = dict(metadata or {})

    def add_link(
        self,
        name: str,
        *,
        visuals: Sequence[RobotVisual] = (),
        collisions: Sequence[RobotVisual] = (),
        inertial: RobotInertial | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> RobotBuilder:
        """Add one rigid body with optional visuals, collisions and SI inertia."""
        if not name or name in self._links:
            raise ValueError(f"Duplicate or empty robot link name {name!r}.")
        self._links[name] = RobotLink(
            name=name,
            visuals=list(visuals),
            collisions=list(collisions),
            inertial=inertial,
            metadata=dict(metadata or {}),
        )
        return self

    def add_part_link(
        self,
        name: str,
        part: Part,
        *,
        assets_dir: str | Path,
        visual_origin: RigidTransform | None = None,
        collision_from_mesh: bool = False,
        inertial: RobotInertial | None = None,
    ) -> RobotBuilder:
        """Export a native OpenCAD Part as a per-link STL and add it to the robot.

        Args:
            name: Stable link name.
            part: A fully built OpenCAD Part with a live OCCT geometry backend.
            assets_dir: Persistent output directory for the intermediate STL.
            visual_origin: Link-local placement of this mesh.
            collision_from_mesh: Explicitly reuse the visual mesh for collision;
                this does not certify any collision clearance or dynamics.
            inertial: Optional measured/computed SI inertial parameters.

        Raises:
            BackendUnavailableError: When the installed backend cannot emit STL.
        """
        _name(name, "link")  # Validate before forming a filesystem path.
        if name in self._links:
            raise ValueError(f"Duplicate robot link name {name!r}.")
        # The exporter expects native OpenCAD STL mesh coordinates in mm.
        root = Path(assets_dir).resolve()
        root.mkdir(parents=True, exist_ok=True)
        destination = root / f"{name}.stl"
        # Do not modify the builder if the CAD kernel fails to export.
        part.export_stl(str(destination))
        geometry = RobotGeometry(
            kind="mesh", resolved_path=str(destination),
            mesh_vertex_unit_to_mm=1.0,  # CAD STL coordinates are millimeters.
        )
        visual = RobotVisual(origin=visual_origin or RigidTransform.identity(), geometry=geometry)
        return self.add_link(
            name, visuals=[visual],
            collisions=[visual.model_copy(deep=True)] if collision_from_mesh else [],
            inertial=inertial,
            metadata={"source": "OpenCAD Part", "source_feature_id": part.feature_id},
        )

    def add_joint(
        self,
        name: str,
        parent: str,
        child: str,
        *,
        joint_type: URDFJointType = "fixed",
        origin: RigidTransform | None = None,
        axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
        limits: Mapping[str, float] | None = None,
        dynamics: Mapping[str, float] | None = None,
        mimic: Mapping[str, Any] | None = None,
    ) -> RobotBuilder:
        """Attach two named links with an explicitly defined joint frame."""
        if not name or name in self._joints:
            raise ValueError(f"Duplicate or empty joint name {name!r}.")
        if parent not in self._links or child not in self._links:
            raise ValueError(f"Joint {name!r} references an unknown parent or child link.")
        self._joints[name] = RobotJoint(
            name=name, joint_type=joint_type, parent=parent, child=child,
            origin=origin or RigidTransform.identity(),
            axis=axis, limits=dict(limits or {}), dynamics=dict(dynamics or {}),
            mimic=dict(mimic) if mimic is not None else None,
        )
        return self

    def build(self) -> RobotDescription:
        """Create a canonical OpenCAD assembly and separate robot-joint document."""
        root = _validate_graph(self._links, self._joints)
        children: dict[str, list[str]] = {name: [] for name in self._links}
        parent_joint: dict[str, RobotJoint] = {}
        for joint in self._joints.values():
            children[joint.parent].append(joint.child)
            parent_joint[joint.child] = joint
        components: dict[str, AssemblyComponent] = {}
        for name, link in self._links.items():
            joint = parent_joint.get(name)
            components[f"urdf:{name}"] = AssemblyComponent(
                id=f"urdf:{name}", name=name,
                child_ids=[f"urdf:{child}" for child in children[name]],
                geometry_refs=[f"urdf:{name}:visual:{index}" for index in range(len(link.visuals))],
                transform=joint.origin if joint is not None else RigidTransform.identity(),
                metadata={"urdf_link": name,
                          "parent_joint": joint.name if joint is not None else None},
            )
        return RobotDescription(
            name=self.name, root_link=root,
            links={name: link.model_copy(deep=True) for name, link in self._links.items()},
            joints={name: joint.model_copy(deep=True) for name, joint in self._joints.items()},
            assembly=AssemblyTree(
                id=f"urdf:{self.name}", name=self.name,
                root_ids=[f"urdf:{root}"], components=components,
                metadata={"format": "opencad-native-robot"},
            ),
            metadata={
                **self._metadata,
                "units": {
                    "transforms": "mm", "mesh_vertices": "m unless source mesh sets unit_to_mm",
                    "joint_angles": "rad", "prismatic_limits": "m",
                    "mass": "kg", "inertia": "kg*m^2",
                },
                "verification": {
                    "kinematics_supported": False,
                    "constraints_unresolved": False,
                    "dynamics_unverified": True,
                },
            },
        )
