//! Making the parts: wall thickness for the process and material, and the printer bed.

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};
use crate::measure;

use super::Ctx;

const G: &str = "mfg";

fn normal(w: &crate::world::World, i: usize, p: V3) -> V3 {
    let e = 0.1;
    v3(
        w.sdf(i, p + v3(e, 0.0, 0.0)) - w.sdf(i, p - v3(e, 0.0, 0.0)),
        w.sdf(i, p + v3(0.0, e, 0.0)) - w.sdf(i, p - v3(0.0, e, 0.0)),
        w.sdf(i, p + v3(0.0, 0.0, e)) - w.sdf(i, p - v3(0.0, 0.0, e)),
    )
    .norm()
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let mut checked = 0;
    let mut overall_min = f64::INFINITY;
    for rp in &r.parts {
        let p = &rp.part;
        if p.catalog.is_some() || p.kind.is_electronic() {
            continue;
        }
        let Some(i) = w.index(&p.id) else { continue };
        let mat = p.material.as_deref().and_then(catalog::material);
        let printed = catalog::part_is_printed(p.process.as_deref(), p.material.as_deref());
        let process_min = match (p.process.as_deref(), mat.map(|m| m.class)) {
            (Some("resin"), _) => r.profile.min_wall_resin_mm,
            (_, Some(catalog::MatClass::Elastomer)) => r.profile.min_wall_tpu_mm,
            _ if printed => r.profile.min_wall_fdm_mm,
            _ => 0.0,
        };
        let min_wall = process_min.max(mat.map(|m| m.min_wall_mm).unwrap_or(0.0));
        // Bed size, any orientation.
        if printed {
            let mut s = w.bodies.get(i).map(|b| b.aabb.size()).map(|v| [v.x, v.y, v.z]).unwrap_or([0.0; 3]);
            s.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
            let mut bed = r.profile.bed_mm;
            bed.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
            if s.iter().zip(bed.iter()).any(|(a, b)| a > b) {
                out.push(
                    Finding::fail(
                        "mfg.bed",
                        G,
                        Severity::Blocker,
                        "Part is bigger than the print bed",
                        format!("`{}` is {:.0} x {:.0} x {:.0} mm.", p.id, s[0], s[1], s[2]),
                    )
                    .on(&[&p.id])
                    .fix("Split it into two parts joined with screws."),
                );
            }
        }
        if min_wall <= 0.0 {
            continue;
        }
        checked += 1;
        // Inward ray from sampled surface points; where it leaves the solid again is the local wall thickness.
        // Grid finer than a third of the wall limit, or a thin wall can slip between samples.
        let pts = measure::surface_points(w, i, (min_wall / 3.0).max(0.25));
        let stride = (pts.len() / 8000).max(1);
        let mut worst: Option<(f64, V3)> = None;
        let mut thin = 0;
        for (k, pt) in pts.iter().enumerate() {
            if k % stride != 0 {
                continue;
            }
            let n = normal(w, i, *pt);
            if n.len() < 0.5 {
                continue;
            }
            let mut t = 0.1;
            let mut through = None;
            while t <= min_wall * 2.5 + 0.2 {
                if w.sdf(i, *pt - n * t) >= 0.0 {
                    through = Some(t);
                    break;
                }
                t += 0.1;
            }
            if let Some(t) = through
                && t < min_wall - 0.05
            {
                thin += 1;
                if worst.is_none_or(|(wt, _)| t < wt) {
                    worst = Some((t, *pt));
                }
            }
        }
        if let Some((t, at)) = worst {
            overall_min = overall_min.min(t);
            // Slivers from the grid are not walls: need a cluster of samples.
            if thin >= 8 {
                let sev = if t < min_wall * 0.7 { Severity::Blocker } else { Severity::Warning };
                out.push(
                    Finding::fail(
                        "mfg.thin_wall",
                        G,
                        sev,
                        "Wall too thin to make",
                        format!(
                            "`{}` has walls down to {t:.2} mm near ({:.1}, {:.1}, {:.1}); {} {} needs at least {min_wall:.1} mm.",
                            p.id,
                            at.x,
                            at.y,
                            at.z,
                            p.process.as_deref().unwrap_or("this process"),
                            p.material.as_deref().unwrap_or("")
                        ),
                    )
                    .on(&[&p.id])
                    .here(at)
                    .expect(format!(">= {min_wall:.1} mm"), format!("{t:.2} mm"))
                    .fix("Thicken that wall to a multiple of the nozzle width (0.8, 1.2, 1.6 mm) or merge it into a thicker feature."),
                );
            }
        }
    }
    if overall_min.is_finite() {
        ctx.metrics.insert("min_wall_mm".into(), overall_min);
    }
    if checked > 0 && out.iter().all(|f| f.pass) {
        out.push(Finding::pass(
            "mfg.walls",
            G,
            "Walls are thick enough to make",
            format!("{checked} made parts checked for minimum wall and bed size."),
        ));
    }
    out
}
