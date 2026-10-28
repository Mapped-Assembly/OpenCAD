# URDF Robot Workspace (issue #6)

OpenCAD's reference viewport now includes a separate **URDF Robot Workspace**
for inspecting a typed robot assembly and previewing supported tree joint poses.
The existing CAD feature-tree and independent scene-player documents remain
unchanged: importing URDF does not insert pseudo-operations into CAD history.

## Run

From the repository root:

```bash
uv sync --all-packages --group test
pnpm install --frozen-lockfile
pnpm --filter opencad-viewport build

OPENCAD_KERNEL_BACKEND=analytic \
  uv run --package opencad-backend python -m uvicorn opencad_server.app:app --port 8000
# In a second terminal:
pnpm --filter opencad-viewport-app dev
```

Open http://localhost:5173 and choose **URDF Robot Workspace**.
Upload a single `.urdf` file containing primitives, or a ZIP with exactly
one `.urdf` file and the referenced `.stl` mesh assets. Files may be nested
within the ZIP so long as the URDF's asset references resolve relative to the
URDF directory, including `package://assets/...` references.

The importer validates ZIP member paths and rejects traversal, symlinks, DTDs,
oversized files and unsupported formats. There is no implicit network fetch.
Uploads are processed in a temporary directory and discarded at request end.

## Inspect and export

- The link inspector follows actual `AssemblyTree` parent-child relationships,
  exposing stable per-link visual occurrence IDs.
- Visual primitives and STL meshes render using OpenCAD's standard
  `Viewport3D`. Local mesh data and URDF meter → OpenCAD millimeter scaling
  are applied once. Joint, link and visual origins compose in the correct
  parent-local frames; the camera frames the full robot.
- Supported fixed, revolute, continuous and prismatic tree joints use their
  raw URDF units (radians or meters). Bounded joints enforce supplied limits.
  Continuous joints use an **unbounded numeric input**, never a fabricated
  ±π slider. `Reset joints` evaluates zero positions.
- If a robot contains mimic, floating, planar or unresolved closed-loop
  constraints, motion controls are disabled. The initial geometry is labeled
  **UNVALIDATED neutral hierarchy preview** rather than an accurate mechanism
  simulation.
- Missing mass/inertia and collision descriptions cause warnings. No mass,
  collisions, springs or motors are inferred.
- **Export URDF bundle** reimports the original locally selected file on the
  backend, preventing stale temporary paths from being trusted. When the
  original assembly is export-valid, download a ZIP containing `robot.urdf`,
  `robot.opencad.json` (provenance), and referenced content-addressed STL
  meshes. Joint input is a viewer pose, **not an edit to the robot's home
  configuration**, so changing the controls does not silently rewrite URDF.

## API contract

- `POST /robotics/import?filename=robot.zip`: body is raw ZIP or URDF bytes;
  response has `robot`, `meshes`, `warnings`, `motion_enabled`,
  `export_enabled` and `export_blocker`.
- `POST /robotics/poses`: JSON `{robot, positions}`; emits per-link and
  per-visual world poses, with explicit errors for unsupported constraints.
- `POST /robotics/export?filename=robot.zip`: original raw file body;
  response is a portable ZIP.

The backend parses supported STL triangles with strict vertex and total-size
budgets. Large or unsupported assets fail explicitly; it does not present
missing geometry as a valid loaded robot.

## Verification

```bash
uv run --package opencad-backend python -m pytest apps/backend/tests/test_robotics_api.py
uv run --package opencad python -m pytest packages/opencad/tests/robotics/test_robot_poses.py
pnpm --filter opencad-viewport test
```

The repository also includes
`scripts/e2e_robot_viewport.py` and
`.github/workflows/robotics-browser.yml` to run actual Chromium against
the real Vite app and FastAPI service: **upload → articulate → reset →
export ZIP → re-import**. This check, not simply component/API tests, is
required before closing issue #6.

## Explicit exclusions

- No automatic inverse kinematics, collision-free planning or motion dynamics.
- No passive four-bar linkage or spring-constraint solution (see #8).
- Quaddle may render in an **unvalidated neutral pose**, but joint playback is
  blocked until its passive/closed-loop constraints are modeled correctly.
- Independent ROS parser/FK comparison and official pinned Quaddle benchmark
  remain in #7; ROS packaging and Open-Industries transport remain in #9.
