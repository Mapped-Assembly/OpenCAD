"""Constraints are explicit, versioned, conservatively diagnosed and testable."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from opencad.robotics import (
    FrameAnchor,
    JointRole,
    LoopClosure,
    MechanismConstraintsV1,
    MimicConstraint,
    PlanarFourBar,
    RobotBuilder,
    SpringConstraint,
    attach_mechanism_constraints,
    diagnose_mechanism,
    read_mechanism_constraints,
    solve_four_bar_sweep,
    solve_planar_four_bar,
)
from opencad.robotics.poses import evaluate_robot_poses


def _robot():
    """Make a canonical tree with one driver and two explicitly passive axes."""
    return (
        RobotBuilder("test_mechanism")
        .add_link("base")
        .add_link("crank")
        .add_link("coupler")
        .add_link("rocker")
        .add_joint("driver", "base", "crank", joint_type="revolute",
                   limits={"lower": -math.pi, "upper": math.pi, "effort": 2, "velocity": 1})
        .add_joint("passive_1", "crank", "coupler", joint_type="continuous",
                   limits={"effort": 0, "velocity": 2})
        .add_joint("passive_2", "coupler", "rocker", joint_type="continuous",
                   limits={"effort": 0, "velocity": 2})
        .build()
    )


def _fourbar() -> PlanarFourBar:
    """Choose crank-rocker dimensions supporting a full crank revolution."""
    return PlanarFourBar(
        id="linkage", driver_joint="driver",
        ground_mm=100, crank_mm=40, coupler_mm=90, rocker_mm=70,
        assembly_branch="upper", provenance="analytical unit-test dimensions",
    )


def _analytic_endpoint(model: PlanarFourBar, angle: float) -> tuple[float, float]:
    """Independent cosine-law construction from the ground-rocker pivot.

    This reference does NOT call the circle-intersection implementation.
    """
    bx = model.crank_mm * math.cos(angle)
    by = model.crank_mm * math.sin(angle)
    ground_to_crank = math.hypot(bx - model.ground_mm, by)
    gamma = math.acos(
        (ground_to_crank**2 + model.rocker_mm**2 - model.coupler_mm**2)
        / (2 * ground_to_crank * model.rocker_mm)
    )
    direction = math.atan2(by, bx - model.ground_mm)
    target = direction + (-gamma if model.assembly_branch == "upper" else gamma)
    return (
        model.ground_mm + model.rocker_mm * math.cos(target),
        model.rocker_mm * math.sin(target),
    )


@pytest.mark.parametrize("branch", ["upper", "lower"])
def test_full_crank_sweep_matches_independent_law_of_cosines(branch: str) -> None:
    """Independently verify every sampled closure and branch across a turn."""
    model = _fourbar().model_copy(update={"assembly_branch": branch})
    inputs = [2 * math.pi * n / 72 for n in range(73)]
    results = solve_four_bar_sweep(model, inputs)
    assert len(results) == 73
    for sample in results:
        expected = _analytic_endpoint(model, sample.input_angle_rad)
        assert sample.coupler_rocker_endpoint_mm == pytest.approx(expected, abs=1e-9)
        assert sample.residual_mm < 1e-9
        assert sample.physical_validation == "not_performed"
    assert results[0].coupler_rocker_endpoint_mm == pytest.approx(
        results[-1].coupler_rocker_endpoint_mm, abs=1e-9
    )


def test_unreachable_and_singular_branches_fail_closed() -> None:
    """Unreachable and tangent circles must fail, not invent passive geometry."""
    impossible = PlanarFourBar(
        id="bad", driver_joint="driver",
        ground_mm=300, crank_mm=40, coupler_mm=60, rocker_mm=60,
    )
    with pytest.raises(ValueError, match="unreachable"):
        solve_planar_four_bar(impossible, 0)
    tangent = PlanarFourBar(
        id="singular", driver_joint="driver",
        ground_mm=200, crank_mm=20, coupler_mm=100, rocker_mm=80,
    )
    with pytest.raises(ValueError, match="singular"):
        solve_planar_four_bar(tangent, 0)
    with pytest.raises(ValueError, match="finite"):
        solve_planar_four_bar(_fourbar(), float("nan"))
    with pytest.raises(ValueError, match="at least one"):
        solve_four_bar_sweep(_fourbar(), [])


def test_versioned_constraints_round_trip_and_explicit_roles() -> None:
    """Attach without mutating URDF topology; persist every declared relation."""
    robot = _robot()
    constraints = MechanismConstraintsV1(
        joint_roles=[
            JointRole(joint="driver", role="driven", evidence="encoder attached"),
            JointRole(joint="passive_1", role="passive"),
            JointRole(joint="passive_2", role="passive"),
        ],
        closures=[LoopClosure(
            id="loop-A", first=FrameAnchor(link="rocker", xyz_mm=(0, 0, 0)),
            second=FrameAnchor(link="base", xyz_mm=(100, 0, 0)),
        )],
        springs=[SpringConstraint(
            id="extension-spring", first=FrameAnchor(link="base"),
            second=FrameAnchor(link="coupler"),
            rest_length_mm=24, stiffness_N_per_m=100,
            provenance="manufacturer-provided example, not measured",
        )],
        planar_four_bars=[_fourbar()],
    )
    changed = attach_mechanism_constraints(robot, constraints)
    assert robot.metadata.get("mechanism_constraints_v1") is None
    assert changed.assembly.model_dump(mode="json") == robot.assembly.model_dump(mode="json")
    assert changed.joints == robot.joints
    assert changed.metadata["verification"]["constraints_unresolved"] is True
    assert changed.metadata["verification"]["dynamics_unverified"] is True
    serialized = changed.model_dump_json()
    recovered = type(robot).model_validate_json(serialized)
    assert read_mechanism_constraints(recovered) == constraints
    diagnosis = diagnose_mechanism(recovered)
    assert diagnosis.state == "constraints_unresolved"
    assert diagnosis.explicit_passive_joints == ["passive_1", "passive_2"]
    assert diagnosis.supported_solvers == ["planar_four_bar_position"]
    assert diagnosis.physical_validation == "not_performed"
    with pytest.raises(ValueError, match="closed-loop constraints"):
        evaluate_robot_poses(recovered, {})
    # Even if somebody incorrectly clears the cached flag, the explicit
    # versioned sidecar must still prevent a false unrestricted FK result.
    recovered.metadata["verification"]["constraints_unresolved"] = False
    with pytest.raises(ValueError, match="closed-loop constraints"):
        evaluate_robot_poses(recovered, {})


def test_bad_refs_and_mimic_cycles_are_rejected() -> None:
    """Avoid silently ignoring missing links, unsupported driver or coupling cycles."""
    robot = _robot()
    with pytest.raises(ValueError, match="unknown robot link"):
        attach_mechanism_constraints(robot, MechanismConstraintsV1(
            closures=[LoopClosure(
                id="broken", first=FrameAnchor(link="base"),
                second=FrameAnchor(link="imaginary"),
            )]
        ))
    with pytest.raises(ValueError, match="requires an explicit"):
        attach_mechanism_constraints(robot, MechanismConstraintsV1(planar_four_bars=[_fourbar()]))
    with pytest.raises(ValueError, match="directed cycle"):
        attach_mechanism_constraints(robot, MechanismConstraintsV1(mimic=[
            MimicConstraint(joint="passive_1", driver="passive_2"),
            MimicConstraint(joint="passive_2", driver="passive_1"),
        ]))
    with pytest.raises(ValueError, match="must be unique"):
        MechanismConstraintsV1(closures=[
            LoopClosure(id="same", first=FrameAnchor(link="base"), second=FrameAnchor(link="crank")),
            LoopClosure(id="same", first=FrameAnchor(link="base"), second=FrameAnchor(link="rocker")),
        ])


def test_quaddle_style_closing_link_is_a_warning_not_geometry_claim() -> None:
    """Conservative source-level diagnosis before any actual loop is solved."""
    robot = (
        RobotBuilder("quaddle_style")
        .add_link("body")
        .add_link("closing_frame")
        .add_joint("frame", "body", "closing_frame", joint_type="fixed")
        .build()
    )
    diagnostic = diagnose_mechanism(robot)
    assert diagnostic.state == "constraints_unresolved"
    assert diagnostic.possible_closure_markers == ["closing_frame"]
    assert "possible" in diagnostic.unresolved[0].lower()
    assert diagnostic.supported_solvers == []
    with pytest.raises(ValueError, match="closed-loop constraints"):
        evaluate_robot_poses(robot, {})


def test_constraints_validate_material_properties_but_do_not_simulate_forces() -> None:
    """No mass/spring/actuator guesses; reject invalid physical-value types."""
    with pytest.raises(ValidationError):
        SpringConstraint(
            id="bad", first=FrameAnchor(link="base"),
            second=FrameAnchor(link="crank"), stiffness_N_per_m=-1,
        )
    with pytest.raises(ValidationError):
        SpringConstraint(
            id="bad", first=FrameAnchor(link="base"),
            second=FrameAnchor(link="crank"), damping_N_s_per_m=float("inf"),
        )
    with pytest.raises(ValidationError):
        PlanarFourBar(
            id="bad", driver_joint="driver",
            ground_mm=0, crank_mm=40, coupler_mm=90, rocker_mm=70,
        )


def test_mimic_diagnostics_preserve_native_urdf_metadata() -> None:
    """Original URDF mimic remains available but is not silently evaluated."""
    robot = _robot()
    robot.joints["passive_1"].mimic = {"joint": "driver", "multiplier": -1.0, "offset": 0.0}
    report = diagnose_mechanism(robot)
    assert report.state == "constraints_unresolved"
    assert any("Mimic" in message for message in report.unresolved)
