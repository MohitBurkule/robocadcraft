# Reference 2WD wedge

![Reference 2WD wedge](renders/iso.png)

Verifier (`aws2025`): passes. 0 blockers, 0 warnings, 13 checks passed.

Mass 107.9 g (range 88.5 to 127.4 g).

| Top | Exploded | Section |
|---|---|---|
| ![top](renders/top.png) | ![exploded](renders/exploded.png) | ![section](renders/section_z.png) |

## Files

- `spec.json`: the full editable design (shapes, parts, fasteners, wires, connections). Load it back with the robot tools to change and re-verify.
- `report.json`, `REPORT.md`: verifier output.
- `stl/`: one STL per part (mm, all in the robot start pose so they assemble when imported together, wires included).
- `renders/`: iso, top, back, exploded and section views.

Regenerate: `cargo run --release -p cadcraft-robot --example export -- wedge_2wd robots`

