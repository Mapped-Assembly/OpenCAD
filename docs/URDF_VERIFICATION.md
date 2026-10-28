# Independent URDF / robot kinematics verification

OpenCAD issue [#7](https://github.com/Mapped-Assembly/OpenCAD/issues/7)
adds an **enforced, independent** interoperability and forward-kinematics
acceptance test. This is intentionally separate from OpenCAD's own unit tests.

## Reference implementations

- Independent URDF parsing, structural validation and forward kinematics:
  [yourdfpy 0.0.60](https://pypi.org/project/yourdfpy/0.0.60/).
  It is installed only in the dedicated verification run, not in the OpenCAD
  core or normal user environments.
- Real, **official** Petoi Quaddle URDF and STL reference from
  [petoi/quaddle](https://huggingface.co/petoi/quaddle) at pinned commit
  `d5ed8ccfd238f98c2c47eb47658ed23232b1a74d` (MIT license).
  All available `assets/*.stl` are downloaded for this check. GitHub
  must not silently skip the official integration test.
- The test also creates a **new** robot using OpenCAD `RobotBuilder` with
  fixed, revolute, continuous and prismatic joints, nontrivial XYZ roll/pitch/yaw
  orientations, arbitrary joint axes, visual local frames and STL assets.

## Verification contract

The mandatory standalone script performs:

1. Native OpenCAD robot → exported URDF/STL bundle → OpenCAD import.
2. Independent `yourdfpy.URDF.load` and strict `validate()` on the generated
   model, plus link/joint names and types.
3. Independent `yourdfpy.update_cfg` + `get_transform` at four distinct joint
   configurations (including negative rotations, 18 rad continuous rotation,
   non-axis-aligned joints and meter-based slides).
4. Comparisons of **every link and visual-origin pose** against OpenCAD's
   `evaluate_robot_poses` using a quaternion-to-rotation implementation in the
   verification harness. Position difference must be ≤ `1e-7 m`; SO(3)
   geodesic difference ≤ `1e-6 rad`.
5. OpenCAD imported description → URDF regeneration → reimport, with
   repeated independent frame comparisons and confirmed STL scaling.
6. A real pinned Quaddle model and its actual MIT license and STL assets are
   downloaded; 46 links and 45 joints (16 revolute, 8 continuous, 21 fixed)
   must match **both** parsers. Every referenced STL path must resolve. Its
   complete file is exported and reimported, then independently parsed again.
7. Quaddle's passive four-bar/closing frames must remain **unresolved**, and
   OpenCAD's unconstrained FK evaluation must be rejected for that model.
   Comparing tree descriptions is not validation of physical motion.

### Evidence

The run writes `reports/robot-verification.json` with model commit and SHA-256,
official mesh asset hashes, import/export SHA-256 hashes, external validation
diagnostics, compared pose counts, observed worst translation and angular errors,
explicit unsupported mechanisms and separate status labels:

- `structural_agreement`: independent parser/type/hierarchy agreement.
- `kinematic_agreement`: **only supported serial tree** agreement.
- `physical_validation`: always `not_performed` pending real measurements
  and constrained linkage/physics validation (see issue #8).

An independent parser's complaint about *upstream Quaddle source* is recorded
verbatim and not silently reclassified as valid. Generated OpenCAD reference
robot URDFs must pass strict external validation.

## Run locally

From the OpenCAD repository root:

```bash
uv sync --package opencad --group test
OPENCAD_ROBOT_VERIFICATION_REPORT=reports/robot-verification.json \
  uv run --package opencad --no-sync \
  --with 'yourdfpy==0.0.60' \
  --with 'huggingface-hub==1.12.0' \
  python scripts/verify_robot_urdf_independent.py
```

This intentionally downloads the official Quaddle asset snapshot (roughly
120 MB including meshes) on first run. A broken download, missing real asset,
failed independent parser, or failed FK tolerance **fails the command and CI**.
There is no environment flag that converts a missing official fixture to a
successful skipped test.

GitHub Actions workflow:
[.github/workflows/robotics-independent.yml](../.github/workflows/robotics-independent.yml).
The JSON evidence is published as a workflow artifact; mesh asset downloads
are kept in the CI workspace, not committed or reuploaded.

**Scientific boundary:** Two independent software engines agreeing on a URDF
tree does not prove the robot is dynamically stable, safe to control, or
kinematically consistent with a physically closed-loop mechanism. Quaddle's
spring geometry, passive coupling and foot contact require additional measured
and constrained-solver evidence. This acceptance suite does not claim it.
