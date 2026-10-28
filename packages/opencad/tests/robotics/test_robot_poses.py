"""Unit tests for URDF-compatible tree kinematics and unsupported motion."""
from __future__ import annotations

import math

import pytest

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import RobotBuilder, RobotGeometry, RobotVisual
from opencad.robotics.poses import evaluate_robot_poses


def _model(joint_type: str = "revolute"):
    """Build a two-link robot with a nontrivial joint origin and local mesh pose."""
    return (
        RobotBuilder("poses")
        .add_link("base")
        .add_link("tip", visuals=[
            RobotVisual(
                origin=RigidTransform(translation_mm=(10, 0, 0)),
                geometry=RobotGeometry(kind="sphere", radius_mm=5),
            ),
        ])
        .add_joint(
            "elbow", "base", "tip", joint_type=joint_type,
            origin=RigidTransform(
                translation_mm=(100, 0, 0),
                rotation_quaternion_xyzw=(0, 0, math.sqrt(0.5), math.sqrt(0.5)),
            ),
            axis=(1, 0, 0),
            limits={"lower": -0.8, "upper": 0.8, "effort": 2, "velocity": 1},
        )
        .build()
    )


def test_joint_origin_and_link_visual_frames() -> None:
    """Both parent-joint RPY and local visual placement must be composed."""
    robot = _model()
    rest = evaluate_robot_poses(robot, {"elbow": 0.0})
    moved = evaluate_robot_poses(robot, {"elbow": 0.5})
    at_rest = rest.shape_transforms["urdf:tip:visual:0"].translation_mm
    assert at_rest == pytest.approx((100, 10, 0))
    assert moved.shape_transforms["urdf:tip:visual:0"].rotation_quaternion_xyzw != (
        rest.shape_transforms["urdf:tip:visual:0"].rotation_quaternion_xyzw
    )
    assert moved.link_transforms["tip"].translation_mm == pytest.approx((100, 0, 0))


def test_prismatic_positions_are_meters() -> None:
    """Sliding 0.05 m along joint-local X after 90-degree yaw moves +Y 50 mm."""
    robot = _model("prismatic")
    moved = evaluate_robot_poses(robot, {"elbow": 0.05})
    assert moved.link_transforms["tip"].translation_mm == pytest.approx((100, 50, 0))


def test_limits_invalid_values_and_unbounded_continuous() -> None:
    """Explicit bounded joints reject violations, continuous joints remain unlimited."""
    robot = _model()
    with pytest.raises(ValueError, match="travel limits"):
        evaluate_robot_poses(robot, {"elbow": 0.9})
    with pytest.raises(ValueError, match="Nonfinite"):
        evaluate_robot_poses(robot, {"elbow": float("nan")})
    with pytest.raises(ValueError, match="Unknown"):
        evaluate_robot_poses(robot, {"missing": 0.1})
    unbounded = _model("continuous")
    assert "tip" in evaluate_robot_poses(unbounded, {"elbow": 1000}).link_transforms


def test_mimic_joints_are_never_faked() -> None:
    """Mimic requires an actual coupling solver rather than ordinary FK."""
    robot = _model()
    robot.joints["elbow"].mimic = {"joint": "elbow", "multiplier": 1}
    with pytest.raises(ValueError, match="unsupported/mimic"):
        evaluate_robot_poses(robot)


def test_unresolved_constraints_do_not_report_valid_pose() -> None:
    """A source warning about loop closures must block FK evaluation."""
    robot = _model()
    robot.metadata["verification"]["constraints_unresolved"] = True
    with pytest.raises(ValueError, match="closed-loop constraints"):
        evaluate_robot_poses(robot)
