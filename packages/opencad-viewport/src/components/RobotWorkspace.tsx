import { useEffect, useMemo, useState } from "react";
import type { OpenCadApiClient } from "../api/client";
import { jointControlRange, linkForShape, neutralRobotTransforms, robotCameraFraming, robotPreviewMeshes } from "../robotics";
import type { RobotImportView, RobotPoseView } from "../robotics";
import { Viewport3D } from "./Viewport3D";

export interface RobotWorkspaceProps {
  apiClient: OpenCadApiClient;
  onBack?: () => void;
}

/** Dedicated articulated-robot workspace; does not mutate the CAD feature DAG. */
export function RobotWorkspace({ apiClient, onBack }: RobotWorkspaceProps): JSX.Element {
  const [file, setFile] = useState<File | null>(null);
  const [model, setModel] = useState<RobotImportView | null>(null);
  const [poses, setPoses] = useState<RobotPoseView | null>(null);
  const [positions, setPositions] = useState<Record<string, number>>({});
  const [selectedLink, setSelectedLink] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState<string | null>(null);
  const [poseError, setPoseError] = useState<string | null>(null);

  const meshes = useMemo(() => model ? robotPreviewMeshes(model) : [], [model]);
  const transforms = useMemo(
    () => model ? poses?.shape_transforms ?? neutralRobotTransforms(model.robot) : {},
    [model, poses],
  );
  const framing = useMemo(() => robotCameraFraming(meshes, transforms), [meshes, transforms]);
  const selectedShape = selectedLink && model
    ? model.robot.assembly.components[`urdf:${selectedLink}`]?.geometry_refs[0] ?? null
    : null;

  useEffect(() => {
    if (!model?.motion_enabled) return;
    let cancelled = false;
    void apiClient.evaluateRobot(model.robot, positions)
      .then((result) => {
        if (!cancelled) {
          setPoses(result);
          setPoseError(null);
        }
      })
      .catch((error: unknown) => {
        if (!cancelled) {
          setPoses(null);
          setPoseError(error instanceof Error ? error.message : String(error));
        }
      });
    return () => { cancelled = true; };
  }, [apiClient, model, positions]);

  async function openFile(nextFile: File): Promise<void> {
    /** Import one explicit local bundle with no implicit network mesh loading. */
    setFile(null);
    setModel(null);
    setPoses(null);
    setPositions({});
    setSelectedLink(null);
    setBusy(true);
    setMessage(null);
    setPoseError(null);
    try {
      const imported = await apiClient.importRobot(nextFile);
      setFile(nextFile);
      setModel(imported);
      setSelectedLink(imported.robot.root_link);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  async function exportFile(): Promise<void> {
    /** Export the original uploaded document, not stale temporary paths. */
    if (!file || !model?.export_enabled) return;
    setBusy(true);
    setMessage(null);
    try {
      const blob = await apiClient.exportRobot(file);
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `${model.robot.name}-urdf-bundle.zip`;
      document.body.appendChild(link);
      link.click();
      link.remove();
      URL.revokeObjectURL(url);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  }

  return (
    <main className="robot-workspace">
      <header className="robot-workspace-header">
        <div>
          <h2>OpenCAD Robotics</h2>
          <p>URDF assembly inspection and tree forward-kinematics preview</p>
        </div>
        {onBack && <button type="button" onClick={onBack}>Back to CAD</button>}
      </header>
      <div className="robot-workspace-actions">
        <label>
          Import robot (.urdf or .zip)
          <input aria-label="Import URDF robot" type="file" accept=".urdf,.zip"
            disabled={busy}
            onChange={(event) => {
              const nextFile = event.target.files?.[0];
              if (nextFile) void openFile(nextFile);
              event.target.value = "";
            }} />
        </label>
        <button type="button" onClick={() => void exportFile()}
          disabled={busy || !file || !model?.export_enabled}
          title={model?.export_blocker ?? "Download URDF, STL meshes and provenance"}>
          Export URDF bundle
        </button>
        {model && <button type="button" onClick={() => setPositions({})}
          disabled={busy || !model.motion_enabled}>Reset joints</button>}
        {busy && <span role="status">Processing robot bundle…</span>}
      </div>
      {message && <p role="alert" className="robot-error">{message}</p>}
      {model && (
        <>
          <div className="robot-notices" role="status">
            <strong>{model.robot.name}</strong> · {Object.keys(model.robot.links).length} links ·{" "}
            {Object.keys(model.robot.joints).length} joints
            <p>
              {model.motion_enabled
                ? "Tree forward kinematics only. Not validated for dynamics or hardware."
                : "Articulation unavailable: displaying an UNVALIDATED neutral hierarchy preview."}
            </p>
            {model.export_blocker && <p>Export disabled: {model.export_blocker}</p>}
            {model.warnings.map((warning, index) => <p key={index}>{warning}</p>)}
            {poses?.warnings.map((warning, index) => <p key={`pose-${index}`}>{warning}</p>)}
            {poseError && <p role="alert">Pose evaluation failed: {poseError}</p>}
          </div>
          <div className="robot-workspace-body">
            <aside className="robot-inspector">
              <h3>Robot links</h3>
              <div role="tree" aria-label="URDF link hierarchy">
                {Object.keys(model.robot.links).map((name) => (
                  <button type="button" role="treeitem" key={name}
                    aria-selected={selectedLink === name}
                    onClick={() => setSelectedLink(name)}>{name}</button>
                ))}
              </div>
              {selectedLink && model.robot.links[selectedLink] && (
                <section>
                  <h3>Selected link</h3>
                  <p>{selectedLink}</p>
                  <p>Visuals: {model.robot.links[selectedLink].visuals.length} ·
                    Colliders: {model.robot.links[selectedLink].collisions.length}</p>
                  <p>Mass: {model.robot.links[selectedLink].inertial?.mass_kg ?? "not specified"} kg</p>
                </section>
              )}
              <h3>Joint controls</h3>
              {Object.values(model.robot.joints).map((joint) => {
                const range = jointControlRange(joint);
                const editable = model.motion_enabled && range !== null;
                return (
                  <section className="robot-joint" key={joint.name}>
                    <strong>{joint.name}</strong>
                    <p>{joint.parent} → {joint.child} · {joint.joint_type}</p>
                    <p>Axis: {joint.axis.join(", ")} ·
                      Origin (mm): {joint.origin.translation_mm.join(", ")}</p>
                    <p>Limits: {range?.bounded
                      ? `${range.min} … ${range.max}`
                      : joint.joint_type === "continuous" ? "unbounded" : "not applicable"}</p>
                    {editable && <label>
                      {joint.joint_type === "prismatic" ? "Position (m)" : "Angle (rad)"}
                      <input aria-label={`Joint ${joint.name}`} type="number"
                        min={range.bounded ? range.min : undefined}
                        max={range.bounded ? range.max : undefined}
                        step={range.step} value={positions[joint.name] ?? 0}
                        onChange={(event) => {
                          const value = Number(event.target.value);
                          if (!Number.isFinite(value)) return;
                          if (range.bounded && (value < range.min || value > range.max)) return;
                          setPositions((current) => ({ ...current, [joint.name]: value }));
                        }} />
                    </label>}
                    {!editable && <p>Motion not supported in this preview.</p>}
                  </section>
                );
              })}
            </aside>
            <Viewport3D meshes={meshes} shapeTransforms={transforms}
              cameraPosition={framing.position} cameraTarget={framing.target}
              selectedShapeId={selectedShape}
              onSelectShape={(shapeId) => {
                const link = linkForShape(model.robot, shapeId);
                if (link) setSelectedLink(link);
              }} />
          </div>
        </>
      )}
    </main>
  );
}
