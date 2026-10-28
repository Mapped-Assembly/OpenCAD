# URDF import in OpenCAD — phase 1

OpenCAD can **read** local Unified Robot Description Format (URDF) files into a
typed robot description and the existing semantic `AssemblyTree`.

This is an implementation of [issue #2](https://github.com/Mapped-Assembly/OpenCAD/issues/2).
It is **not** yet a URDF exporter (see
[issue #3](https://github.com/Mapped-Assembly/OpenCAD/issues/3)), a trajectory
solver, a ROS controller, or a dynamics simulator.

## Python example

```python
from pathlib import Path
from opencad.robotics import import_urdf

result = import_urdf(
    Path("robots/quaddle/robot.urdf"),
    asset_root=Path("robots/quaddle"),  # contains assets/*.stl
)
robot = result.robot
print(robot.name, len(robot.links), len(robot.joints))
print(robot.assembly.root_ids)
print(robot.metadata["verification"])
for warning in result.warnings:
    print("WARNING:", warning)

# All data is JSON serializable:
Path("robot.opencad.json").write_text(result.model_dump_json(indent=2))
```

To inspect the XML without having every mesh locally, pass
`require_assets=False` and check `result.unresolved_assets`. The returned
description then does **not** claim that its geometry is loaded.

For another ROS package layout:

```python
result = import_urdf(
    "robot.urdf",
    package_roots={"my_robot_description": Path("/local/ros/src/my_robot_description")},
)
```

The importer does not download remote files. Obtain third-party meshes and their
licenses separately. `package://assets/example.stl`, as used by Petoi Quaddle,
resolves to `<asset_root>/assets/example.stl` by default. Explicit package
root mappings are opt-in.

## CLI

The importer is also available through the existing headless `opencad` CLI:

```bash
opencad import urdf robots/quaddle/robot.urdf \
  --asset-root robots/quaddle \
  --output quaddle.opencad.json

# Nonstandard ROS package installation:
opencad import urdf model.urdf --package-map my_robot_description=/local/ros/src/my_robot_description

# XML structure inspection without downloaded assets:
opencad import urdf model.urdf --allow-missing-assets
```

The generated JSON is an `URDFImportResult` envelope containing the
`RobotDescription`, `AssemblyTree`, and import diagnostics. It is not a
ROS simulation artifact or a kinematic-motion guarantee.

## Units and geometry

| Data | Representation |
|---|---|
| Joint, visual, collision, inertia origins | Millimeter translations, XYZW quaternions |
| Primitive box/cylinder/sphere dimensions | Millimeters |
| STL/mesh vertex data | Referenced file, not automatically tessellated into an OCCT B-rep |
| URDF mesh scale | Original dimensionless scale; nominal input vertices are in meters |
| Joint angles and angular limits | Radians |
| URDF prismatic limits | **Meters**, retained exactly, not mm |
| Mass and inertia | Kilograms; kilogram-square-meters |
| Joint effort and velocity | Original URDF units; preserved without reinterpretation |

`RobotDescription` and `URDFImportResult` preserve typed links, geometries,
joint names/types, origins/axes, limits, optional dynamics/mimic information,
and source provenance. The imported `AssemblyTree` keeps stable semantic
link IDs (`urdf:<link-name>`) and visual occurrence IDs
(`urdf:<link-name>:visual:<index>`), independently of OpenCAD's feature DAG.

Each component's parent-local transform is its zero-configuration joint origin.
This alone does **not** evaluate an articulated robot's full forward kinematics.

## Limitations and verification

- Existing OpenCAD forward kinematics handles fixed, revolute and prismatic
  joints in its current, narrower shape-occurrence model. Imported URDF
  joint descriptions are **not automatically executed** through that model.
  The importer reports `kinematics_supported=False` until that integration
  is implemented, even if all joint types are recognized.
- Continuous, planar, floating and mimic joints are preserved and/or diagnosed;
  they are not silently converted to fixed/revolute joints.
- URDF tree kinematics cannot, by itself, enforce Petoi Quaddle's four-bar
  closed-loop constraints, spring forces, actuation, or hardware behavior.
- Visual meshes are references, not editable parametric solids. Collision mesh
  conversion, mesh preview, joint controls, physics and generated URDF export
  remain follow-up work.
- The importer rejects DTD/entity declarations, path traversal, remote URIs,
  and symlink escapes. It has an 8 MiB XML size limit. Supplying an explicit
  ROS package root is an intentional local filesystem permission.

For an optional official Quaddle regression check, obtain the repository at
https://huggingface.co/petoi/quaddle and run:

```bash
OPENCAD_TEST_QUADDLE_URDF=/path/to/quaddle/robot.urdf \
  uv run --package opencad python -m pytest packages/opencad/tests/robotics/test_urdf_import.py
```

The optional fixture asserts 46 links, 45 joints (16 revolute, 8 continuous,
21 fixed) and the presence of passive/closing frames. The large mesh assets
are **not** vendored in OpenCAD.

## Next steps

1. Add explicit compatible joint-frame FK and viewport mesh loading.
2. Add UI file import, previews and joint controls.
3. Share the canonical robot document with URDF **export** in issue #3.
4. Compare supported FK link poses against an independent URDF implementation;
   flag simulation and physical validation as distinct stages.
