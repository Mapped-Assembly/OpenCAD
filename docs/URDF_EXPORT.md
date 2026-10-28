# Generate URDF robots with OpenCAD

OpenCAD can now **author a robot from a typed Python API** and generate a
standard URDF XML file plus referenced STL files. The exporter reuses the
same robot and assembly types as [the importer](URDF_IMPORT.md).

**URDF describes links and joints.** Exporting it does not certify forward or
inverse kinematics, close four-bar loops, estimate actuators, verify collisions,
or imply that a robot can be manufactured or operated safely.

## Generate a robot entirely in Python (no hand-written XML)

\`\`\`python
from pathlib import Path

from opencad.kernel.core.models import RigidTransform
from opencad.robotics import (
    RobotBuilder, RobotGeometry, RobotVisual, export_urdf, import_urdf,
)

robot = (
    RobotBuilder("two_link_arm")
    .add_link(
        "base",
        visuals=[RobotVisual(geometry=RobotGeometry(
            kind="box", size_mm=(50.0, 50.0, 20.0),
        ))],
    )
    .add_link(
        "forearm",
        visuals=[RobotVisual(geometry=RobotGeometry(
            kind="cylinder", radius_mm=10.0, length_mm=120.0,
        ))],
    )
    .add_joint(
        "elbow", "base", "forearm", joint_type="revolute",
        origin=RigidTransform(translation_mm=(0.0, 0.0, 20.0)),
        axis=(0.0, 1.0, 0.0),
        limits={"lower": -1.57, "upper": 1.57, "effort": 2.0, "velocity": 1.0},
    )
    .build()
)

result = export_urdf(robot, "two_link_arm.urdf")
print(result.urdf_path, result.warnings)
restored = import_urdf(result.urdf_path).robot
assert set(restored.links) == {"base", "forearm"}
\`\`\`

## Build link geometry from native OpenCAD Parts

With the optional native OCCT geometry backend installed, each OpenCAD Part
can become an independent robot link with its own STL mesh.

\`\`\`python
from opencad import Part
from opencad.kernel.core.backend_factory import create_backend
from opencad.runtime import RuntimeContext
from opencad.robotics import RobotBuilder, export_urdf

context = RuntimeContext(backend=create_backend("occt"))
chassis = Part(context=context).box(50, 40, 10)
tool = Part(context=context).cylinder(5, 25)

robot = (
    RobotBuilder("bench_robot")
    .add_part_link("chassis", chassis, assets_dir="generated/cad")
    .add_part_link("tool", tool, assets_dir="generated/cad")
    .add_joint(
        "slide", "chassis", "tool", joint_type="prismatic",
        limits={"lower": 0, "upper": 0.05, "effort": 5, "velocity": 0.1},
    )
    .build()
)
export_urdf(robot, "generated/bench_robot.urdf")
\`\`\`

The generated \`generated/meshes/<sha256>.stl\` files are copied into a portable
bundle. They preserve OpenCAD native STL vertex coordinates in **millimeters**;
the URDF \`mesh scale="0.001 0.001 0.001"\` converts them to meters. The importer
preserves this scale when it reads the resulting URDF.

No implicit collision shapes are fabricated. Use
\`collision_from_mesh=True\` on \`add_part_link\` only when you intentionally want
the same mesh referenced as collision geometry; this is not a clearance or
collision accuracy guarantee.

## CLI: export previously imported URDF

The importer writes an \`URDFImportResult\` JSON envelope. This can be
exported directly, or you can save \`RobotDescription.model_dump_json()\`
from your own Python robot builder and export that JSON instead.

\`\`\`bash
opencad import urdf robot.urdf --asset-root . --output robot.opencad.json
opencad export urdf robot.opencad.json --output rebuilt/robot.urdf
\`\`\`

The export creates three types of artifacts:

- \`robot.urdf\`: standard ROS-compatible robot definition (no OpenCAD tags).
- \`meshes/<sha256>.stl\`: deduplicated content-addressed triangle meshes,
  if referenced by any link.
- \`robot.opencad.json\`: full typed robot metadata, source/provenance,
  diagnostics and mesh list for loss-aware interchange.

## Units, validation and limitations

| Field | OpenCAD document | Emitted URDF |
|---|---|---|
| Joint/link/visual translations | millimeters | meters |
| Fixed-axis RPY rotations | XYZW quaternion | XYZ roll, pitch, yaw in radians |
| Revolute/continuous joint limits | radians | radians |
| Prismatic limits | meters | meters |
| Mass | kilograms | kilograms |
| Inertial tensor | kg·m² | kg·m² |
| Native OpenCAD STL vertices | millimeters | preserved mesh plus explicit 0.001 scale |

The exporter refuses missing referenced meshes, invalid ROS names, malformed
assembly graphs, unsupported planar/floating joint export, missing mandatory
joint limits, non-finite numbers, and invalid inertial matrices. Missing
optional inertial data yields a warning, **not** guessed physics values.
Continuous joints export without lower and upper bounds.

If metadata cannot be expressed in standard URDF, the complete OpenCAD model
is retained in the companion JSON; non-standard tags are not injected into the
URDF. Deterministic XML and content-addressed STL names support reproducible
builds, though source asset byte content is not canonicalized.

The round-trip suite includes static primitives, multiple visual/collision
occurrences, nontrivial rotations, ROS unit conversions, native STL scaling,
CLI interchange and export refusal on invalid inputs. Native OCCT tests only
execute when that optional backend is available.

**Next steps:** independent ROS URDF parser smoke checks in CI, viewport export
controls, richer ROS package layouts, external forward-kinematics comparison,
and validated closed-loop constraints. See [#3](https://github.com/Mapped-Assembly/OpenCAD/issues/3)
and [#2](https://github.com/Mapped-Assembly/OpenCAD/issues/2).
