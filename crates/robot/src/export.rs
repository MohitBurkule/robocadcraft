//! Writes a robot to disk: editable spec, verifier report, renders, STL meshes and a README.

use std::fmt::Write as _;
use std::path::Path;

use crate::checks::{Options, verify_resolved};
use crate::finding::Report;
use crate::math::{V3, v3};
use crate::render::{View, render_png};
use crate::spec::Spec;
use crate::world::{World, resolve};

/// Views written by default; the first is the one used in the README.
pub const VIEWS: [&str; 5] = ["iso", "top", "exploded", "section_z", "back"];

type Tri = [V3; 3];

/// Marching tetrahedra over the signed distance of body `i`, with sample spacing `h` mm.
pub fn mesh_body(world: &World, i: usize, h: f64) -> Vec<Tri> {
    let Some(b) = world.bodies.get(i) else { return Vec::new() };
    let bb = b.aabb.grow(h);
    let n = [
        ((bb.max.x - bb.min.x) / h).ceil() as usize + 1,
        ((bb.max.y - bb.min.y) / h).ceil() as usize + 1,
        ((bb.max.z - bb.min.z) / h).ceil() as usize + 1,
    ];
    if n[0].saturating_mul(n[1]).saturating_mul(n[2]) > 20_000_000 {
        return Vec::new();
    }
    let pos = |x: usize, y: usize, z: usize| v3(bb.min.x + x as f64 * h, bb.min.y + y as f64 * h, bb.min.z + z as f64 * h);
    let at = |x: usize, y: usize, z: usize| (z * n[1] + y) * n[0] + x;
    let mut f = vec![0.0_f32; n[0] * n[1] * n[2]];
    for z in 0..n[2] {
        for y in 0..n[1] {
            for x in 0..n[0] {
                if let Some(s) = f.get_mut(at(x, y, z)) {
                    *s = world.sdf(i, pos(x, y, z)) as f32;
                }
            }
        }
    }
    const CORNERS: [[usize; 3]; 8] = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
    const TETS: [[usize; 4]; 6] = [[0, 5, 1, 6], [0, 1, 2, 6], [0, 2, 3, 6], [0, 3, 7, 6], [0, 7, 4, 6], [0, 4, 5, 6]];
    let mut tris = Vec::new();
    for z in 0..n[2].saturating_sub(1) {
        for y in 0..n[1].saturating_sub(1) {
            for x in 0..n[0].saturating_sub(1) {
                let mut pv = [(V3::ZERO, 0.0_f64); 8];
                let mut any_in = false;
                let mut any_out = false;
                for (k, c) in CORNERS.iter().enumerate() {
                    let (cx, cy, cz) = (x + c[0], y + c[1], z + c[2]);
                    let v = f.get(at(cx, cy, cz)).copied().unwrap_or(1.0) as f64;
                    if let Some(slot) = pv.get_mut(k) {
                        *slot = (pos(cx, cy, cz), v);
                    }
                    any_in |= v < 0.0;
                    any_out |= v >= 0.0;
                }
                if !(any_in && any_out) {
                    continue;
                }
                for t in TETS {
                    let q: Vec<(V3, f64)> = t.iter().filter_map(|k| pv.get(*k).copied()).collect();
                    tet_tris(&q, &mut tris);
                }
            }
        }
    }
    tris
}

fn lerp_edge(a: (V3, f64), b: (V3, f64)) -> V3 {
    let d = a.1 - b.1;
    let t = if d.abs() < 1e-12 { 0.5 } else { (a.1 / d).clamp(0.0, 1.0) };
    a.0 + (b.0 - a.0) * t
}

fn push_oriented(out: &mut Vec<Tri>, tri: Tri, inside: V3, outside: V3) {
    let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    if n.dot(outside - inside) < 0.0 {
        out.push([tri[0], tri[2], tri[1]]);
    } else {
        out.push(tri);
    }
}

fn tet_tris(q: &[(V3, f64)], out: &mut Vec<Tri>) {
    let ins: Vec<usize> = (0..q.len()).filter(|k| q.get(*k).is_some_and(|p| p.1 < 0.0)).collect();
    let outs: Vec<usize> = (0..q.len()).filter(|k| q.get(*k).is_some_and(|p| p.1 >= 0.0)).collect();
    let g = |k: usize| q.get(k).copied().unwrap_or((V3::ZERO, 0.0));
    match (ins.len(), outs.len()) {
        (1, 3) | (3, 1) => {
            let (single, others) = if ins.len() == 1 { (ins[0], &outs) } else { (outs[0], &ins) };
            let tri = [lerp_edge(g(single), g(others[0])), lerp_edge(g(single), g(others[1])), lerp_edge(g(single), g(others[2]))];
            let (inside, outside) = if ins.len() == 1 { (g(single).0, g(others[0]).0) } else { (g(others[0]).0, g(single).0) };
            push_oriented(out, tri, inside, outside);
        }
        (2, 2) => {
            let (a, b, c, d) = (ins[0], ins[1], outs[0], outs[1]);
            let p = [lerp_edge(g(a), g(c)), lerp_edge(g(a), g(d)), lerp_edge(g(b), g(d)), lerp_edge(g(b), g(c))];
            let (inside, outside) = ((g(a).0 + g(b).0) * 0.5, (g(c).0 + g(d).0) * 0.5);
            push_oriented(out, [p[0], p[1], p[2]], inside, outside);
            push_oriented(out, [p[0], p[2], p[3]], inside, outside);
        }
        _ => {}
    }
}

pub fn stl_bytes(name: &str, tris: &[Tri]) -> Vec<u8> {
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    let mut head = [0u8; 80];
    for (d, s) in head.iter_mut().zip(name.bytes()) {
        *d = s;
    }
    out.extend_from_slice(&head);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in tris {
        let n = (t[1] - t[0]).cross(t[2] - t[0]).norm();
        for v in [n, t[0], t[1], t[2]] {
            for c in [v.x, v.y, v.z] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

pub fn report_markdown(spec: &Spec, rep: &Report) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "# Verifier report: {}\n", spec.name);
    let _ = writeln!(
        s,
        "Profile `{}`. {} blockers, {} warnings, {} checks passed.\n",
        rep.profile, rep.summary.blockers, rep.summary.warnings, rep.summary.passed
    );
    let m = |k: &str| rep.metrics.get(k).copied();
    if let (Some(t), Some(lo), Some(hi), Some(lim)) = (m("mass_total_g"), m("mass_lo_g"), m("mass_hi_g"), m("mass_limit_g")) {
        let _ = writeln!(s, "Mass {t:.1} g (range {lo:.1} to {hi:.1} g, limit {lim:.0} g).\n");
    }
    let _ = writeln!(s, "## Findings\n");
    for f in &rep.findings {
        let tag = if f.pass {
            "pass"
        } else if matches!(f.severity, crate::finding::Severity::Blocker) {
            "BLOCKER"
        } else {
            "warning"
        };
        let _ = writeln!(s, "- **{tag}** `{}`: {} {}", f.id, f.title, f.message);
        if let Some(fix) = &f.fix
            && !f.pass
        {
            let _ = writeln!(s, "  - Fix: {fix}");
        }
    }
    let _ = writeln!(s, "\n## Mass budget\n\n| Part | Grams | Plus/minus | Source |\n|---|---|---|---|");
    for l in &rep.mass {
        let _ = writeln!(s, "| {} | {:.2} | {:.2} | {} |", l.id, l.grams, l.tol_g, l.source);
    }
    if !rep.routes.is_empty() {
        let _ = writeln!(s, "\n## Wire routes\n\n| Wire | AWG | Diameter mm | Length mm |\n|---|---|---|---|");
        for r in &rep.routes {
            let _ = writeln!(s, "| {} | {} | {:.1} | {:.0} |", r.id, r.awg, r.od_mm, r.length_mm);
        }
    }
    if !rep.bench_checklist.is_empty() {
        let _ = writeln!(s, "\n## Bench checklist (CAD cannot prove these)\n");
        for c in &rep.bench_checklist {
            let _ = writeln!(s, "- [ ] {c}");
        }
    }
    s
}

pub fn readme(slug: &str, spec: &Spec, rep: &Report) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "# {}\n\n![{}](renders/iso.png)\n", spec.name, spec.name);
    let status = if rep.ok { "passes" } else { "has blockers" };
    let _ = writeln!(
        s,
        "Verifier (`{}`): {status}. {} blockers, {} warnings, {} checks passed.\n",
        rep.profile, rep.summary.blockers, rep.summary.warnings, rep.summary.passed
    );
    if let (Some(t), Some(lo), Some(hi)) = (rep.metrics.get("mass_total_g"), rep.metrics.get("mass_lo_g"), rep.metrics.get("mass_hi_g")) {
        let _ = writeln!(s, "Mass {t:.1} g (range {lo:.1} to {hi:.1} g).\n");
    }
    let _ = writeln!(
        s,
        "| Top | Exploded | Section |\n|---|---|---|\n| ![top](renders/top.png) | ![exploded](renders/exploded.png) | ![section](renders/section_z.png) |\n"
    );
    let _ = writeln!(
        s,
        "## Files\n\n- `spec.json`: the full editable design (shapes, parts, fasteners, wires, connections). Load it back with the robot tools to change and re-verify.\n- `report.json`, `REPORT.md`: verifier output.\n- `stl/`: one STL per part (mm, all in the robot start pose so they assemble when imported together, wires included).\n- `renders/`: iso, top, back, exploded and section views.\n\nRegenerate: `cargo run --release -p cadcraft-robot --example export -- {slug} robots`\n"
    );
    s
}

/// Verify `spec` and write everything under `dir/<slug>/`. Returns the files written.
pub fn write_all(slug: &str, spec: &Spec, dir: &Path, mesh_step_mm: f64) -> Result<(Report, Vec<String>), String> {
    let root = dir.join(slug);
    let io = |e: std::io::Error| e.to_string();
    std::fs::create_dir_all(root.join("renders")).map_err(io)?;
    std::fs::create_dir_all(root.join("stl")).map_err(io)?;
    let r = resolve(spec.clone());
    let rep = verify_resolved(&r, &Options::default());
    let world = World::start(&r).with_routes(&rep.routes).with_assembly(&rep.assembly);
    let mut files = Vec::new();
    let mut put = |rel: &str, bytes: Vec<u8>| -> Result<(), String> {
        std::fs::write(root.join(rel), bytes).map_err(|e| e.to_string())?;
        files.push(rel.to_string());
        Ok(())
    };
    put("spec.json", serde_json::to_vec_pretty(spec).map_err(|e| e.to_string())?)?;
    put("report.json", serde_json::to_vec_pretty(&rep).map_err(|e| e.to_string())?)?;
    put("REPORT.md", report_markdown(spec, &rep).into_bytes())?;
    put("README.md", readme(slug, spec, &rep).into_bytes())?;
    for v in VIEWS {
        if let Some(view) = View::named(v, 1100) {
            put(&format!("renders/{v}.png"), render_png(&world, &view, None, &[]))?;
        }
    }
    for (i, b) in world.bodies.iter().enumerate() {
        let tris = mesh_body(&world, i, mesh_step_mm);
        let name = b.id.replace([':', '/'], "_");
        put(&format!("stl/{name}.stl"), stl_bytes(&name, &tris))?;
    }
    let (bodies, bad, warn) = crate::step::bodies(&world);
    std::fs::create_dir_all(root.join("step")).map_err(io)?;
    for b in &bodies {
        put(&format!("step/{}.step", b.name), crate::step::step_text(&b.name, &[b]).into_bytes())?;
    }
    let all: Vec<&crate::step::StepBody> = bodies.iter().collect();
    put("step/assembly.step", crate::step::step_text(slug, &all).into_bytes())?;
    if !bad.is_empty() || !warn.is_empty() {
        let mut list: String = bad.iter().map(|(n, e)| format!("{n}: NOT EXPORTED: {e}\n")).collect();
        list.extend(warn.iter().map(|(n, e)| format!("{n}: {e}\n")));
        put("step/_skipped.txt", list.into_bytes())?;
    }
    Ok((rep, files))
}
