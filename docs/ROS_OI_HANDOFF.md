# ROS 2 package and OpenIndustries handoff (OpenCAD #9)

OpenCAD exports a **real ROS2 ament_cmake robot-description package** plus
an OpenIndustries v1 geometry asset, retaining stable robot link and joint IDs,
source geometry SHA-256, material and collision provenance and explicit physical
validation limits.

## Generate and verify a package

From OpenCAD's repository root:

```bash
uv sync --package opencad --group test
uv run --no-sync --package opencad python scripts/examples/ros_oi_handoff.py /tmp/bench_robot_description

# Also works from a typed robot JSON or a URDFImportResult document:
opencad export ros-package my-robot.opencad.json \
  --package-name my_robot_description \
  --output /tmp/my_robot_description

# Resolve the ROS package URI explicitly when importing:
opencad import urdf /tmp/my_robot_description/urdf/my_robot.urdf \
  --package-map my_robot_description=/tmp/my_robot_description
```

Output (all local; no ROS installation needed merely to export):

```text
my_robot_description/
  package.xml
  CMakeLists.txt
  urdf/my_robot.urdf
  meshes/<sha256>.stl
  robot.handoff.json
  oi.asset.json
```

The URDF uses **only** `package://my_robot_description/meshes/<sha256>.stl`
references; it cannot leak the author’s absolute paths. `package.xml` and
`CMakeLists.txt` form a minimal ROS2 `ament_cmake` description package. The
maintainer and license are explicitly marked as placeholders; replace these
before publishing. Never assume model authorship or redistribution rights from
the mere ability to export a file.

## OpenIndustries handoff

`oi.asset.json` is a genuine OpenIndustries `{"schemaVersion":1,"asset":...}`
bundle compatible with its local scene tool. It includes:

- `source.kind=generated`, `source.generator=opencad-robot` and SHA-256
  of the exported ROS URDF. It does **not** impersonate `form-industries`.
- `units=m`, `upAxis=Y`. Positions/meshes are converted from OpenCAD
  right-handed Z-up millimeters into OI right-handed Y-up meters using
  `(x,y,z) → (x,z,-y)`.
- A nested `AssetNode` tree retaining every URDF link's identity, with one
  distinct `Part` per visible mesh/primitive occurrence:
  `urdf:<link>:visual:<index>`. No links are silently flattened into one
  anonymous object.
- Per-part source link/visual IDs, mesh hashes, color/material provenance, and
  an explicit `neutral_unvalidated` status.

`robot.handoff.json` has `schema=opencad.robot_handoff.v1`. This sidecar
retains joint IDs, type/axis/limits/origin, visual **and collision** occurrence
metadata, mass/inertial values when provided, content-addressed asset paths,
unit conversions, source SHA-256 and `constraints_unresolved` diagnostics.
It contains no source filesystem paths and cannot be mistaken for a physics
simulation.

Bring the asset into the actual OI persistent scene system:

```bash
cd /path/to/OpenIndustries
npm ci
ASTRA_SCENE_BACKEND=local node --import tsx cli/local-scenes.ts \
  import-asset /tmp/my_robot_description/oi.asset.json
```

The CLI emits an immutable `{"kind":"local","version_id":"..."}` asset
reference. Use it within an `astra.create_scene` request as an instance asset:

```json
{
  "version": 1,
  "request_id": "b3c466be-c4e3-4070-a212-1df6590bcc8e",
  "agent": "opencad-handoff",
  "scene": {
    "name": "Robot layout (neutral geometry only)",
    "room": {"width": 5, "depth": 5, "height": 3},
    "instances": [
      {
        "id": "robot-1",
        "name": "Robot (unvalidated neutral geometry)",
        "asset": {"kind": "local", "version_id": "REPLACE_WITH_ACTUAL_ID"},
        "position": [2, 0, 2], "rotation": [0, 0, 0], "visible": true
      }
    ],
    "animation": {"duration": 3, "loop": false, "tracks": []}
  }
}
```

OI is a spatial workbench: this is an **asset-and-scene handoff**, not
automatic URDF joint playback. Part animations are separate authored tracks
and are **not** evidence of correct robot kinematics.

## Contract checks

```bash
uv run --no-sync --package opencad python -m pytest packages/opencad/tests/robotics/test_ros_oi_handoff.py
```

The companion consumer test in OpenIndustries is
`npm run test:opencad-robot`; set `OPENCAD_OI_ASSET_PATH` to the exported
`oi.asset.json` to enforce real producer → consumer validation instead of
its small standalone fixture. See the dedicated cross-repo GitHub workflow in
OpenCAD for this end-to-end check.

**Not validated:** robot motion, spring dynamics, loop closure, collision
clearance, actuator feasibility, safe hardware operation, IK or real-time
control. Those require additional physics, mechanism and hardware evidence.
