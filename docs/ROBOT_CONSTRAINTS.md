# Robot Mechanisms — OpenCAD Issue #8

URDF describes a serial tree, while physical robots can have closed kinematic
loops, passive joints, springs, and coupled actuators. OpenCAD does **not**
treat a visual tree as a verified dynamic model.

## Typed versioned sidecar

OpenCAD stores optional typed `MechanismConstraintsV1` metadata under
`RobotDescription.metadata['mechanism_constraints_v1']`, preserving the
original links, joints, assembly identity, and standard URDF export unchanged.

- `JointRole`: driven, passive, fixed; never guessed from URDF revolute type
- `MimicConstraint`: affine joint coupling with cycle/reference checks
- `LoopClosure`: coincidence requirement between link-local anchors in mm
- `SpringConstraint`: optional rest length (mm), stiffness (N/m), damping (N·s/m) and provenance; no force simulation
- `PlanarFourBar`: explicitly dimensioned planar ground–crank–coupler–rocker with two assembly branches

## Example

```python
from opencad.robotics import (
    RobotBuilder, JointRole, PlanarFourBar, MechanismConstraintsV1,
    attach_mechanism_constraints, diagnose_mechanism, solve_four_bar_sweep,
)

robot = (RobotBuilder('example')
    .add_link('base').add_link('crank')
    .add_joint('motor', 'base', 'crank', joint_type='continuous',
               limits={'effort': 2, 'velocity': 1})
    .build())
model = PlanarFourBar(id='test', driver_joint='motor', ground_mm=100,
    crank_mm=40, coupler_mm=90, rocker_mm=70, assembly_branch='upper',
    provenance='synthetic benchmark dimensions, not Quaddle')
robot = attach_mechanism_constraints(robot, MechanismConstraintsV1(
    joint_roles=[JointRole(joint='motor', role='driven')],
    planar_four_bars=[model],
))
print(diagnose_mechanism(robot).model_dump())
poses = solve_four_bar_sweep(model, [0.0, 0.4, 0.8, 1.2])
print(max(p.residual_mm for p in poses))
```

## Solver scope

Planar four-bar A=(0,0), D=(ground,0), B=(crank*cos(angle),crank*sin(angle))
is solved by circle intersection: C lies at distance coupler from B and
rocker from D. The branch sign selects the perpendicular intersection.
Closure error is the maximum link-length disagreement in mm.

Unreachable, coincident, singular/tangent, nonfinite inputs and excessive
residuals **raise errors**, not fabricated joint positions. This algorithm
returns dimensioned 2D position-only solutions. It is **not** a generic URDF
closed-loop simulator, actuator model, IK solver, or a physical safety check.

Independent benchmark: 181 samples for each assembly branch across 2π;
an independent law-of-cosines calculation compares the endpoint to the
circle-intersection solver. Required closure residual <= 1e-7 mm and
independent disagreement <= 1e-8 mm. Reports include actual measured maxima,
source geometry SHA-256, declared tolerances, and physical_validation=false.

## Real Petoi Quaddle

The official, commit-pinned Quaddle URDF (46 links and 45 joints) includes
`closing_*` frames, treated as **possible closure markers**, not evidence
of measured four-bar dimensions. The pinned independent fixture job now
requires `diagnose_mechanism(robot)` to flag those markers, assign **no**
guessed dimensioned solver, and keep generic unconstrained FK disabled.

Quaddle spring geometry, motor inputs, passive linkage dimensions, foot
contact, and hardware movement remain **unverified**. A synthetic benchmark
does not validate the real Quaddle. This requires measured geometry and
separate calibrated solver and hardware evidence.

## Fail-closed policy

`evaluate_robot_poses` validates the versioned sidecar at every invocation.
Clearing a legacy `verification.constraints_unresolved` flag does not bypass
real unresolved constraints. Standard URDF export retains the sidecar in its
provenance manifest without writing nonstandard tags into `robot.urdf`.

## Run

```bash
uv sync --package opencad --group test
uv run --no-sync --package opencad python -m pytest packages/opencad/tests/robotics/test_mechanism_constraints.py
uv run --no-sync --package opencad python scripts/verify_four_bar_benchmark.py
```

CI: `.github/workflows/robotics-constraints.yml` enforces the dimensioned
benchmark, and `.github/workflows/robotics-independent.yml` fetches and checks
the real pinned Quaddle model. Both must pass before #8 closes.
