"""Typed robot authoring and bidirectional URDF support for OpenCAD."""

from opencad.robotics.authoring import RobotBuilder
from opencad.robotics.constraints import (
    FrameAnchor, LoopClosure, JointRole, MimicConstraint, SpringConstraint,
    PlanarFourBar, MechanismConstraintsV1, MechanismDiagnostic, FourBarSolution,
    attach_mechanism_constraints, read_mechanism_constraints,
    diagnose_mechanism, solve_planar_four_bar, solve_four_bar_sweep,
)
from opencad.robotics.export import URDFExportResult, export_urdf
from opencad.robotics.oi_handoff import robot_to_oi_asset
from opencad.robotics.ros_package import ROSPackageResult, export_ros_package
from opencad.robotics.urdf import (
    RobotDescription,
    RobotGeometry,
    RobotInertial,
    RobotJoint,
    RobotLink,
    RobotVisual,
    URDFImportResult,
    import_urdf,
)

__all__ = [
    "FrameAnchor", "LoopClosure", "JointRole", "MimicConstraint",
    "SpringConstraint", "PlanarFourBar", "MechanismConstraintsV1",
    "MechanismDiagnostic", "FourBarSolution", "attach_mechanism_constraints",
    "read_mechanism_constraints", "diagnose_mechanism",
    "solve_planar_four_bar", "solve_four_bar_sweep",
    "RobotBuilder",
    "ROSPackageResult",
    "export_ros_package",
    "robot_to_oi_asset",
    "URDFExportResult",
    "export_urdf",
    "RobotDescription",
    "RobotGeometry",
    "RobotInertial",
    "RobotJoint",
    "RobotLink",
    "RobotVisual",
    "URDFImportResult",
    "import_urdf",
]
