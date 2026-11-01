# Elfsys modular hydroponic rack — OpenCAD concept v0

Reference: https://claude.ai/artifact/6DG68ZqUrsya1vr6Ap613W

This is an independently modeled **provisional concept**, not a CAD import or exact
reverse-engineered reproduction of the friend's Elfsys hardware. The original
Claude artifact could not be programmatically inspected, so component dimensions
are placeholders and should be checked with the hardware owner.

## Supported model

The previously discussed visualization presents a modular system extending
from **2 levels / 28 plant sites** to **4 levels / 56 plant sites**, with
growing channels, illumination, supports, drain system, wheels and a tank.

The example creates a distinct OpenCAD `Part` for each component:
- Two hollow, perforated-drain grow troughs per level
- Seven through-hole net-pot openings in each removable lid
- Four posts and caster *envelopes*, level supports, reservoir shelf
- Open-top reservoir, pump envelope, LED bar envelopes and vertical riser proxies

### Run locally

From the Mapped-Assembly/OpenCAD root:

```bash
uv sync --package opencad --extra occt
uv run --package opencad --extra occt python \
  scripts/examples/elfsys_hydroponic_rack.py \
  --levels 2 --out output/elfsys-2 --format step

uv run --package opencad --extra occt python \
  scripts/examples/elfsys_hydroponic_rack.py \
  --levels 4 --out output/elfsys-4 --format both
```

Outputs are `assembly.json`, `features.opencad.json`, and per-component
STEP and/or STL files under `parts/`. Every solid is at its absolute
Z-up position in millimetres. The JSON manifest provides unique names,
level indices and design assumptions. This example does **not** export
a single assembled STEP automatically, and does not animate assembly.

### Provisional measurements

| Nominal property | Value |
|---|---:|
| Width × depth | 1200 × 570 mm |
| Trough | 1080 × 170 × 90 mm |
| Plant opening diameter | 72 mm |
| Plant opening pitch | 140 mm |
| Row count | 2 per level |
| Plant sites per row | 7 |
| Level vertical pitch | 440 mm |
| First channel center | 690 mm above ground |
| Reservoir outside | 780 × 370 × 220 mm |
| Frame height (2 / 4 levels) | 1490 / 2370 mm |

**Not validated:** hydraulic connections, pump head, watertightness, nutrition
delivery, LED intensity/spectrum, wiring, loads, tipping, food-contact
materials, fabrication tolerances, or real-system safety.

## Next acceptance criteria

1. Confirm all dimensions with the original hardware designer.
2. Native OpenCAD CLI builds both layouts and exports/reimports valid solids.
3. Verify assembly clearances, joints, fasteners and manufacturability.
4. Connect every grow tray to an explicitly routed, sealed feed/return network;
   size pump, tank and plumbing with real hydraulic calculations.
5. Record energy/water/nutrient usage in an OpenIndustries scene.
6. Visualize 2→4 level assembly and nutrient circulation with an end-to-end video.

An initial independent CADQuery smoke test of the geometry produced 29 valid
part solids for the 2-level variant and 45 for the 4-level variant. This is
**not** a native OpenCAD CI run and must not be presented as one.
