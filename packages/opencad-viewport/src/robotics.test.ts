import { describe, expect, it } from "vitest";
import { jointControlRange, linkForShape, neutralRobotTransforms, robotPreviewMeshes } from "./robotics";
import type { RobotImportView, RobotJoint } from "./robotics";

const identity = {
  translation_mm: [0, 0, 0] as [number, number, number],
  rotation_quaternion_xyzw: [0, 0, 0, 1] as [number, number, number, number],
};

function fixture(): RobotImportView {
  return {
    robot: {
      name: "test_robot", root_link: "base",
      links: {
        base: { name: "base", visuals: [{
          name: null, origin: identity, material: {},
          geometry: { kind: "box", size_mm: [20, 10, 5], radius_mm: null, length_mm: null,
            source_uri: null, resolved_path: null, mesh_scale: [1, 1, 1],
            mesh_vertex_unit_to_mm: 1000 },
        }], collisions: [], inertial: null },
        tip: { name: "tip", visuals: [{
          name: null, origin: { ...identity, translation_mm: [10, 0, 0] }, material: {},
          geometry: { kind: "sphere", size_mm: null, radius_mm: 3, length_mm: null,
            source_uri: null, resolved_path: null, mesh_scale: [1, 1, 1],
            mesh_vertex_unit_to_mm: 1000 },
        }], collisions: [], inertial: null },
      },
      joints: {
        elbow: { name: "elbow", parent: "base", child: "tip", joint_type: "revolute",
          origin: { ...identity, translation_mm: [0, 20, 0] }, axis: [0, 0, 1],
          limits: { lower: -1, upper: 1, effort: 2, velocity: 3 },
          dynamics: {}, mimic: null },
      },
      assembly: { root_ids: ["urdf:base"], components: {
        "urdf:base": { name: "base", child_ids: ["urdf:tip"], geometry_refs: ["urdf:base:visual:0"] },
        "urdf:tip": { name: "tip", child_ids: [], geometry_refs: ["urdf:tip:visual:0"] },
      } },
      metadata: {},
    },
    meshes: [], warnings: [], export_enabled: true, export_blocker: null, motion_enabled: true,
  };
}

describe("robot viewport geometry and neutral assembly state", () => {
  it("creates individual stable visual shape IDs for native primitives", () => {
    const model = fixture();
    const meshes = robotPreviewMeshes(model);
    expect(meshes.map(mesh => mesh.shapeId)).toEqual(["urdf:base:visual:0", "urdf:tip:visual:0"]);
    expect(meshes.every(mesh => mesh.vertices.length > 0 && mesh.faces.length > 0)).toBe(true);
    expect(linkForShape(model.robot, meshes[1].shapeId)).toBe("tip");
    expect(linkForShape(model.robot, "missing")).toBeNull();
  });

  it("composes link and visual origins for an explicitly neutral preview", () => {
    const poses = neutralRobotTransforms(fixture().robot);
    expect(poses["urdf:tip:visual:0"].translation_mm).toEqual([10, 20, 0]);
  });

  it("does not invent a continuous-joint boundary or enable fixed/mimic motion", () => {
    const revolute = fixture().robot.joints.elbow;
    expect(jointControlRange(revolute)).toEqual({ min: -1, max: 1, step: 0.02, bounded: true });
    const continuous: RobotJoint = { ...revolute, joint_type: "continuous" };
    expect(jointControlRange(continuous)?.bounded).toBe(false);
    expect(jointControlRange(continuous)?.min).toBe(-Infinity);
    expect(jointControlRange({ ...revolute, joint_type: "fixed" })).toBeNull();
    expect(jointControlRange({ ...revolute, mimic: { joint: "another" } })).toBeNull();
  });
});
