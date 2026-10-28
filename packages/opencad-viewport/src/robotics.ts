/** Typed URDF robotics contracts, deliberately separate from scenes and CAD features. */
import { BoxGeometry, CylinderGeometry, Quaternion, SphereGeometry, Vector3 } from "three";
import type { BufferGeometry } from "three";
import type { MeshPayload, RigidTransform } from "./types";

export interface RobotVisual {
  name: string | null;
  origin: RigidTransform;
  geometry: {
    kind: "mesh" | "box" | "cylinder" | "sphere";
    size_mm: [number, number, number] | null;
    radius_mm: number | null;
    length_mm: number | null;
    source_uri: string | null;
    resolved_path: string | null;
    mesh_scale: [number, number, number];
    mesh_vertex_unit_to_mm: number;
  };
  material: Record<string, unknown>;
}

export interface RobotJoint {
  name: string;
  joint_type: "fixed" | "revolute" | "continuous" | "prismatic" | "planar" | "floating";
  parent: string;
  child: string;
  origin: RigidTransform;
  axis: [number, number, number];
  limits: Record<string, number>;
  dynamics: Record<string, number>;
  mimic: Record<string, unknown> | null;
}

export interface RobotLink {
  name: string;
  visuals: RobotVisual[];
  collisions: RobotVisual[];
  inertial: { mass_kg: number; inertia_kg_m2: Record<string, number> } | null;
}

export interface RobotDescriptionView {
  name: string;
  root_link: string;
  links: Record<string, RobotLink>;
  joints: Record<string, RobotJoint>;
  assembly: {
    root_ids: string[];
    components: Record<string, { child_ids: string[]; geometry_refs: string[]; name: string }>;
  };
  metadata: Record<string, unknown>;
}

export interface RobotImportView {
  robot: RobotDescriptionView;
  meshes: MeshPayload[];
  warnings: string[];
  export_enabled: boolean;
  export_blocker: string | null;
  motion_enabled: boolean;
}

export interface RobotPoseView {
  link_transforms: Record<string, RigidTransform>;
  shape_transforms: Record<string, RigidTransform>;
  warnings: string[];
}

/** Convert native Three primitives into the existing OpenCAD triangle-mesh contract. */
export function robotPreviewMeshes(imported: RobotImportView): MeshPayload[] {
  const meshes = [...imported.meshes];
  for (const [linkName, link] of Object.entries(imported.robot.links)) {
    link.visuals.forEach((visual, index) => {
      const geom = visual.geometry;
      if (geom.kind === "mesh") return;
      let primitive: BufferGeometry;
      if (geom.kind === "box" && geom.size_mm) {
        primitive = new BoxGeometry(...geom.size_mm);
      } else if (geom.kind === "cylinder" && geom.radius_mm != null && geom.length_mm != null) {
        primitive = new CylinderGeometry(geom.radius_mm, geom.radius_mm, geom.length_mm, 20);
        primitive.rotateX(Math.PI / 2);
      } else if (geom.kind === "sphere" && geom.radius_mm != null) {
        primitive = new SphereGeometry(geom.radius_mm, 20, 14);
      } else {
        throw new Error(`Missing dimensions for ${linkName} visual ${index}`);
      }
      const position = primitive.getAttribute("position");
      const vertices = Array.from({ length: position.count * 3 }, (_, i) => position.array[i]);
      const faces = primitive.index
        ? Array.from(primitive.index.array)
        : Array.from({ length: position.count }, (_, i) => i);
      meshes.push({
        shapeId: `urdf:${linkName}:visual:${index}`,
        vertices, faces, name: `${linkName} visual ${index}`,
      });
      primitive.dispose();
    });
  }
  return meshes;
}

/** Joint controls use raw URDF units (radians for rotation; meters for slides). */
export function jointControlRange(joint: RobotJoint): {
  min: number; max: number; step: number; bounded: boolean;
} | null {
  if (joint.joint_type === "fixed" || joint.joint_type === "planar" ||
      joint.joint_type === "floating" || joint.mimic !== null) return null;
  if (joint.joint_type === "continuous") {
    // Numerical input only; continuous joints must never acquire invented limits.
    return { min: -Infinity, max: Infinity, step: 0.05, bounded: false };
  }
  const min = joint.limits.lower;
  const max = joint.limits.upper;
  if (!Number.isFinite(min) || !Number.isFinite(max) || min > max) return null;
  return {
    min, max,
    step: joint.joint_type === "prismatic" ? 0.001 : 0.02,
    bounded: true,
  };
}

/** Select stable link identity from a preview shape occurrence ID. */
export function linkForShape(robot: RobotDescriptionView, shapeId: string): string | null {
  for (const [name, link] of Object.entries(robot.links)) {
    if (link.visuals.some((_, i) => `urdf:${name}:visual:${i}` === shapeId)) return name;
  }
  return null;
}

/** Derive an explicitly unvalidated neutral (zero-joint) hierarchy preview. */
export function neutralRobotTransforms(robot: RobotDescriptionView): Record<string, RigidTransform> {
  const transforms: Record<string, RigidTransform> = {};
  const byParent: Record<string, RobotJoint[]> = {};
  Object.values(robot.joints).forEach((joint) => {
    (byParent[joint.parent] ??= []).push(joint);
  });
  const worldByLink: Record<string, RigidTransform> = {};
  const compose = (a: RigidTransform, b: RigidTransform): RigidTransform => {
    const qa = new Quaternion(...a.rotation_quaternion_xyzw);
    const qb = new Quaternion(...b.rotation_quaternion_xyzw);
    const p = new Vector3(...b.translation_mm).applyQuaternion(qa)
      .add(new Vector3(...a.translation_mm));
    const q = qa.multiply(qb);
    return { translation_mm: p.toArray() as [number, number, number],
      rotation_quaternion_xyzw: q.toArray() as [number, number, number, number] };
  };
  const identity: RigidTransform = {
    translation_mm: [0, 0, 0], rotation_quaternion_xyzw: [0, 0, 0, 1],
  };
  const visit = (name: string, world: RigidTransform): void => {
    worldByLink[name] = world;
    robot.links[name]?.visuals.forEach((visual, index) => {
      transforms[`urdf:${name}:visual:${index}`] = compose(world, visual.origin);
    });
    (byParent[name] ?? []).forEach((joint) => visit(joint.child, compose(world, joint.origin)));
  };
  visit(robot.root_link, identity);
  return transforms;
}

