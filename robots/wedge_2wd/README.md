# Reference 2WD wedge

![Reference 2WD wedge](renders/iso.png)

Verifier (`aws2025`): passes. 0 blockers, 1 warnings, 11 checks passed.

Mass 102.9 g (range 84.4 to 121.4 g).

| Top | Exploded | Section |
|---|---|---|
| ![top](renders/top.png) | ![exploded](renders/exploded.png) | ![section](renders/section_z.png) |

## Files

- `spec.json`: the full editable design (shapes, parts, fasteners, wires, connections). Load it back with the robot tools to change and re-verify.
- `report.json`, `REPORT.md`: verifier output.
- `stl/`: one STL per part (mm, all in the robot start pose so they assemble when imported together, wires included).
- `renders/`: iso, top, back, exploded and section views.

Regenerate: `cargo run --release -p cadcraft-robot --example export -- wedge_2wd robots`

