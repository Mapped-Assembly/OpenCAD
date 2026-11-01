"""Parametric Elfsys-inspired modular hydroponic rack for OpenCAD.

An independent, provisional concept model based on a previously discussed
friend's visualization, not a reverse-engineered reproduction of their CAD.
All dimensions are editable assumptions; no plumbing/electrical certification.

Usage from repo root:
  uv run --package opencad --extra occt python scripts/examples/elfsys_hydroponic_rack.py --levels 2 --out output/elfsys-2
  uv run --package opencad --extra occt python scripts/examples/elfsys_hydroponic_rack.py --levels 4 --out output/elfsys-4 --format both
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal

from opencad import Part, get_default_context

ExportFormat = Literal["step", "stl", "both", "none"]


@dataclass(frozen=True)
class HydroponicParameters:
    """Nominal dimensions in mm; Elfsys hardware dimensions are not known."""

    levels: int = 2
    sites_per_tray: int = 7
    trays_per_level: int = 2
    rack_width_mm: float = 1200.0
    rack_depth_mm: float = 570.0
    post_size_mm: float = 30.0
    tray_length_mm: float = 1080.0
    tray_width_mm: float = 170.0
    tray_height_mm: float = 90.0
    tray_wall_mm: float = 12.0
    tray_floor_mm: float = 14.0
    lid_thickness_mm: float = 8.0
    plant_hole_diameter_mm: float = 72.0
    plant_pitch_mm: float = 140.0
    tray_offset_y_mm: float = 116.0
    first_level_z_mm: float = 690.0
    level_pitch_mm: float = 440.0
    light_clearance_mm: float = 290.0
    reservoir_width_mm: float = 780.0
    reservoir_depth_mm: float = 370.0
    reservoir_height_mm: float = 220.0

    def validate(self) -> None:
        """Reject unsupported and geometrically inconsistent parameters."""
        if self.levels not in (2, 4):
            raise ValueError("Initial reference configurations support 2 or 4 levels")
        if self.trays_per_level != 2 or self.sites_per_tray != 7:
            raise ValueError("Reference concept has 2 trays × 7 sites per level")
        if any(value <= 0 for value in asdict(self).values()):
            raise ValueError("All counts and dimensions must be positive")
        if self.tray_length_mm > self.rack_width_mm - 2 * self.post_size_mm:
            raise ValueError("Trays must fit between the nominal rack envelope")
        if 2 * self.tray_offset_y_mm + self.tray_width_mm > self.rack_depth_mm - self.post_size_mm:
            raise ValueError("The two trays do not fit the rack depth")
        if ((self.sites_per_tray - 1) * self.plant_pitch_mm + self.plant_hole_diameter_mm
                > self.tray_length_mm - 2 * self.tray_wall_mm):
            raise ValueError("Seven plant holes do not fit the channel lid")
        if self.plant_hole_diameter_mm >= self.tray_width_mm - 2 * self.tray_wall_mm:
            raise ValueError("Plant opening is too wide for its channel")
        if self.light_clearance_mm >= self.level_pitch_mm:
            raise ValueError("Light would collide with the next growing level")

    @property
    def total_sites(self) -> int:
        """Return number of geometrical net-pot openings."""
        return self.levels * self.trays_per_level * self.sites_per_tray

    @property
    def top_z_mm(self) -> float:
        """Top of vertical frame relative to ground plane."""
        return self.first_level_z_mm + (self.levels - 1) * self.level_pitch_mm + 360.0


@dataclass
class Component:
    """Named assembly item retaining its independent parametric CAD solid."""

    name: str
    kind: str
    part: Part
    level: int | None
    notes: str = ""


def box(name: str, size: tuple[float, float, float], pos: tuple[float, float, float]) -> Part:
    """Create a dimensioned, positioned OpenCAD box."""
    return Part(name=name).box(*size, name=name).translate(pos)


def cylinder(name: str, radius: float, height: float, pos: tuple[float, float, float]) -> Part:
    """Create a vertical, positioned OpenCAD cylinder."""
    return Part(name=name).cylinder(radius, height, name=name).translate(pos)


def tray_body(params: HydroponicParameters, name: str, x: float, y: float, z: float) -> Part:
    """Create a real hollow channel with a through-floor drain opening."""
    length, width, height = params.tray_length_mm, params.tray_width_mm, params.tray_height_mm
    body = box(name, (length, width, height), (x, y, z))
    inner_height = height + 12.0
    inner_bottom = z - height / 2 + params.tray_floor_mm
    cavity = box(f"{name}_void", (length - 2 * params.tray_wall_mm,
                                 width - 2 * params.tray_wall_mm, inner_height),
                 (x, y, inner_bottom + inner_height / 2))
    body.cut(cavity, name=f"{name}_hollow")
    drain = cylinder(f"{name}_drain_cut", 12.0, height,
                     (x + length / 2 - 50, y, z))
    body.cut(drain, name=f"{name}_drain_port")
    return body


def perforated_lid(params: HydroponicParameters, name: str, x: float, y: float, z: float) -> Part:
    """Create removable channel lid with seven through-holes."""
    lid = box(name, (params.tray_length_mm, params.tray_width_mm,
                     params.lid_thickness_mm), (x, y, z))
    for i in range(params.sites_per_tray):
        hole_x = x + (i - (params.sites_per_tray - 1) / 2) * params.plant_pitch_mm
        tool = cylinder(f"{name}_site_{i+1:02d}_cut",
                        params.plant_hole_diameter_mm / 2,
                        params.lid_thickness_mm + 8, (hole_x, y, z))
        lid.cut(tool, name=f"{name}_site_{i+1:02d}")
    return lid


def build(params: HydroponicParameters) -> list[Component]:
    """Build rack supports, empty tank, grow tiers, lights and riser proxies."""
    params.validate()
    components: list[Component] = []

    def add(name: str, kind: str, part: Part, level: int | None = None,
            notes: str = "") -> None:
        """Append a named assembly member."""
        components.append(Component(name, kind, part, level, notes))

    post_x = params.rack_width_mm / 2 - params.post_size_mm / 2
    post_y = params.rack_depth_mm / 2 - params.post_size_mm / 2
    post_bottom = 95.0
    post_height = params.top_z_mm - post_bottom
    for xside in (-1, 1):
        for yside in (-1, 1):
            name = f"post_{'left' if xside < 0 else 'right'}_{'front' if yside < 0 else 'rear'}"
            add(name, "frame", box(name, (params.post_size_mm, params.post_size_mm,
                                         post_height),
                                   (xside * post_x, yside * post_y,
                                    post_bottom + post_height / 2)))
            caster = name.replace("post", "caster")
            add(caster, "caster_envelope",
                cylinder(caster, 34.0, 60.0,
                         (xside * post_x, yside * post_y, 55.0)),
                notes="Caster geometry placeholder; not a wheel engineering drawing")

    add("reservoir_shelf", "frame",
        box("reservoir_shelf",
            (params.rack_width_mm - 45, params.rack_depth_mm - 45, 20),
            (0, 0, 210)))
    tank = box("reservoir", (params.reservoir_width_mm, params.reservoir_depth_mm,
                             params.reservoir_height_mm), (0, 0, 345))
    void_h = params.reservoir_height_mm + 12
    tank_void = box("reservoir_void", (params.reservoir_width_mm - 24,
                                      params.reservoir_depth_mm - 24, void_h),
                    (0, 0, 345 - params.reservoir_height_mm / 2
                     + 16 + void_h / 2))
    tank.cut(tank_void, name="reservoir_open_top")
    add("reservoir", "reservoir", tank,
        notes="Open-top tank concept; watertightness and capacity unverified")
    add("pump_envelope", "equipment",
        box("pump_envelope", (125, 90, 90), (-250, 0, 330)),
        notes="Pump envelope only, not sized or powered")
    pipe_height = params.first_level_z_mm + (params.levels - 1) * params.level_pitch_mm - 320
    for name, x in (("feed_riser", -525.0), ("return_riser", 525.0)):
        add(name, "plumbing_proxy",
            cylinder(name, 16, pipe_height, (x, 0, 345 + pipe_height / 2)),
            notes="Visual riser only; unconnected to individual trays")
    for layer in range(params.levels):
        z = params.first_level_z_mm + layer * params.level_pitch_mm
        for lane in range(params.trays_per_level):
            y = (-1 if lane == 0 else 1) * params.tray_offset_y_mm
            prefix = f"level_{layer+1:02d}_lane_{lane+1:02d}"
            add(f"{prefix}_support", "frame",
                box(f"{prefix}_support", (params.tray_length_mm, 22, 30),
                    (0, y, z - 60)), layer + 1)
            add(f"{prefix}_trough", "grow_tray",
                tray_body(params, f"{prefix}_trough", 0, y, z), layer + 1)
            add(f"{prefix}_lid", "plant_site_lid",
                perforated_lid(params, f"{prefix}_lid", 0, y,
                               z + params.tray_height_mm / 2
                               + params.lid_thickness_mm / 2), layer + 1)
            add(f"{prefix}_led_bar", "lighting_proxy",
                box(f"{prefix}_led_bar",
                    (params.tray_length_mm - 80, 26, 18),
                    (0, y, z + params.light_clearance_mm)), layer + 1,
                notes="LED envelope only; intensity and wiring unvalidated")
    return components


def export_model(params: HydroponicParameters, output: Path,
                 fmt: ExportFormat) -> dict[str, Any]:
    """Export separate world-positioned STEP/STL components and native tree."""
    parts = build(params)
    output.mkdir(parents=True, exist_ok=True)
    if fmt != "none":
        (output / "parts").mkdir(exist_ok=True)
    records: list[dict[str, Any]] = []
    for component in parts:
        files: dict[str, str] = {}
        formats = ("step", "stl") if fmt == "both" else (() if fmt == "none" else (fmt,))
        for extension in formats:
            path = output / "parts" / f"{component.name}.{extension}"
            component.part.export(str(path))
            files[extension] = path.relative_to(output).as_posix()
        records.append({"name": component.name, "kind": component.kind,
                        "level": component.level, "files": files, "notes": component.notes})
    manifest: dict[str, Any] = {
        "schema_version": "elfsys-concept-0.1",
        "reference_url": "https://claude.ai/artifact/6DG68ZqUrsya1vr6Ap613W",
        "authoring_engine": "OpenCAD",
        "coordinate_system": "Z-up, millimetres, world-positioned components",
        "design_assumptions": "Dimensions are provisional; owner drawings unavailable",
        "not_validated": ["hydraulic connectivity", "pump sizing", "nutrient delivery",
                          "LED spectrum", "electrical safety", "structural stability",
                          "food contact", "manufacturability"],
        "parameters": asdict(params),
        "plant_sites": params.total_sites,
        "parts": records,
        "conceptual_water_path": ["reservoir", "pump_envelope", "feed_riser",
                                  "troughs", "drain_ports", "return_riser", "reservoir"],
    }
    (output / "assembly.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    get_default_context().save_tree_json(str(output / "features.opencad.json"))
    return manifest


def main(argv: list[str] | None = None) -> int:
    """Expose reference configurations as a reproducible command-line script."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--levels", type=int, choices=(2, 4), default=2)
    parser.add_argument("--out", type=Path, default=Path("output/elfsys-v0"))
    parser.add_argument("--format", choices=("step", "stl", "both", "none"),
                        default="step")
    args = parser.parse_args(argv)
    result = export_model(HydroponicParameters(levels=args.levels),
                          args.out, args.format)
    print(f"Elfsys v0: {result['plant_sites']} sites, {len(result['parts'])} parts -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
