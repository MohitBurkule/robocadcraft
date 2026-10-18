# Revron-style antweight vertical spinner

![Revron-style antweight vertical spinner](renders/iso.png)

Verifier (`aws2025`): passes. 0 blockers, 3 warnings, 18 checks passed.

Mass 130.4 g (range 110.1 to 150.7 g).

| Top | Exploded | Section |
|---|---|---|
| ![top](renders/top.png) | ![exploded](renders/exploded.png) | ![section](renders/section_z.png) |

## Files

- `spec.json`: the full editable design (shapes, parts, fasteners, wires, connections). Load it back with the robot tools to change and re-verify.
- `report.json`, `REPORT.md`: verifier output.
- `stl/`: one STL per part (mm, all in the robot start pose so they assemble when imported together, wires included).
- `renders/`: iso, top, back, exploded and section views.

Regenerate: `cargo run --release -p cadcraft-robot --example export -- revron_ant robots`

