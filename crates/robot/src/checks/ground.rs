//! Standing on the floor: wheels touch it, nothing else drags, the wedge sits at the right height and the centre of mass is supported.

use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};
use crate::measure;
use crate::spec::PartKind;

use super::Ctx;

const G: &str = "ground";

fn cross(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

fn hull(mut p: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    p.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let mut h: Vec<(f64, f64)> = Vec::new();
    for pass in 0..2 {
        let start = h.len();
        let iter: Vec<_> = if pass == 0 { p.clone() } else { p.iter().rev().cloned().collect() };
        for q in iter {
            while h.len() >= start + 2 && h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 1e-9 {
                h.pop();
            }
            h.push(q);
        }
        h.pop();
    }
    h
}

/// Smallest distance from `c` to the edges of a convex hull, negative when outside.
fn margin(h: &[(f64, f64)], c: (f64, f64)) -> f64 {
    if h.len() < 3 {
        return f64::NEG_INFINITY;
    }
    let mut m = f64::INFINITY;
    for (i, a) in h.iter().enumerate() {
        let b = h[(i + 1) % h.len()];
        let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt().max(1e-9);
        m = m.min(cross(*a, b, c) / len);
    }
    m
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let wheels: Vec<usize> = w.bodies.iter().enumerate().filter(|(_, b)| b.kind == PartKind::Wheel).map(|(i, _)| i).collect();
    if wheels.is_empty() {
        out.push(Finding::note("ground.no_wheels", G, "No wheels", "No wheel parts, so ground contact is not checked."));
        return out;
    }
    let lows: Vec<(usize, V3)> = wheels.iter().filter_map(|i| measure::lowest_point(w, *i, 0.4).map(|p| (*i, p))).collect();
    let floor = r.spec.ground_z.unwrap_or_else(|| lows.iter().map(|(_, p)| p.z).fold(f64::INFINITY, f64::min));
    ctx.metrics.insert("ground_z".into(), floor);
    let mut floating = Vec::new();
    for (i, p) in &lows {
        let gap = p.z - floor;
        if gap > 0.5 {
            floating.push((*i, gap));
        }
    }
    if floating.is_empty() {
        out.push(Finding::pass(
            "ground.wheels_touch",
            G,
            "Wheels touch the floor",
            format!("All {} wheels reach the floor plane (z = {floor:.2} mm).", lows.len()),
        ));
    } else {
        for (i, gap) in &floating {
            let id = w.bodies.get(*i).map(|b| b.id.as_str()).unwrap_or("?");
            out.push(
                Finding::fail(
                    "ground.wheel_floating",
                    G,
                    Severity::Blocker,
                    "Wheel does not touch the floor",
                    format!("`{id}` is {gap:.2} mm above the floor; with the other wheels down it would never drive."),
                )
                .on(&[id])
                .expect("0 mm (within 0.5)", format!("{gap:.2} mm"))
                .fix("Use the same wheel diameter and axle height on every wheel, or add a tyre; a lower wheel carries all the weight."),
            );
        }
    }

    // Clearance of everything that is not a wheel or the wedge.
    let mut worst: Option<(String, f64)> = None;
    let mut wedge_tips: Vec<(String, V3)> = Vec::new();
    for (i, b) in w.bodies.iter().enumerate() {
        if b.id.starts_with("wire:") || b.kind == PartKind::Wheel || b.kind == PartKind::Tyre {
            continue;
        }
        let Some(p) = measure::lowest_point(w, i, 0.8) else { continue };
        if matches!(b.kind, PartKind::Wedge | PartKind::Fork) {
            wedge_tips.push((b.id.clone(), p));
            continue;
        }
        let h = p.z - floor;
        if worst.as_ref().is_none_or(|(_, wh)| h < *wh) {
            worst = Some((b.id.clone(), h));
        }
    }
    let min_clear = r.profile.min_ground_clearance_mm;
    if let Some((id, h)) = worst {
        ctx.metrics.insert("min_ground_clearance_mm".into(), h);
        if h < min_clear {
            out.push(
                Finding::fail(
                    "ground.clearance",
                    G,
                    Severity::Warning,
                    "Body is close to the floor",
                    format!("`{id}` is {h:.2} mm above the floor; it will scrape on arena seams and ride on the ground when a tyre compresses."),
                )
                .on(&[&id])
                .expect(format!(">= {min_clear:.1} mm"), format!("{h:.2} mm"))
                .fix("Raise the chassis or use a bigger wheel; wedges and forks may go lower."),
            );
        } else {
            out.push(Finding::pass("ground.clearance", G, "Body clears the floor", format!("Lowest body part is {h:.1} mm above the floor.")));
        }
    }
    let [glo, ghi] = r.profile.wedge_gap_mm;
    for (id, p) in &wedge_tips {
        let gap = p.z - floor;
        if gap < glo || gap > ghi {
            out.push(
                Finding::fail("ground.wedge_gap", G, Severity::Warning, "Wedge tip at the wrong height", format!("`{id}` tip is {gap:.2} mm above the floor; it should be {glo:.1} to {ghi:.1} mm so it slides under opponents without digging in or dragging."))
                    .on(&[id])
                    .expect(format!("{glo:.1} to {ghi:.1} mm"), format!("{gap:.2} mm"))
                    .here(*p),
            );
        } else {
            out.push(Finding::pass("ground.wedge_gap", G, "Wedge tip height is right", format!("`{id}` tip is {gap:.2} mm above the floor.")));
        }
    }

    // Static stance: support polygon from wheel contacts and a wedge tip that can touch after a small tilt.
    let com = (ctx.metrics.get("com_x").copied().unwrap_or(0.0), ctx.metrics.get("com_y").copied().unwrap_or(0.0));
    let mut support: Vec<(f64, f64)> = lows.iter().map(|(i, p)| (w.bodies.get(*i).map(|b| b.xf.t.x).unwrap_or(p.x), p.y)).collect();
    let wheel_y = lows.iter().map(|(_, p)| p.y).sum::<f64>() / lows.len().max(1) as f64;
    let mut wedge_share = None;
    for (id, p) in &wedge_tips {
        if p.z - floor <= 2.0
            && let Some(b) = w.body(id)
        {
            support.push((b.aabb.min.x + 1.0, p.y));
            support.push((b.aabb.max.x - 1.0, p.y));
            let span = p.y - wheel_y;
            if span.abs() > 1.0 {
                wedge_share = Some(((com.1 - wheel_y) / span).clamp(0.0, 1.0));
            }
        }
    }
    let h = hull(support);
    let m = margin(&h, com);
    ctx.metrics.insert("stance_margin_mm".into(), m);
    if h.len() < 3 {
        out.push(
            Finding::fail(
                "ground.stance",
                G,
                Severity::Warning,
                "Robot has no support polygon",
                "Only two wheel contacts and nothing else can reach the floor; the robot will pitch onto whatever touches first.",
            )
            .fix("Add a skid, a third wheel, or lower the wedge to within 2 mm of the floor."),
        );
    } else if m < 2.0 {
        out.push(
            Finding::fail(
                "ground.stance",
                G,
                Severity::Blocker,
                "Centre of mass is outside the support",
                format!("Centre of mass ({:.1}, {:.1}) is {m:.1} mm from the support polygon edge; the robot tips over at rest.", com.0, com.1),
            )
            .expect(">= 2 mm inside", format!("{m:.1} mm")),
        );
    } else {
        out.push(Finding::pass(
            "ground.stance",
            G,
            "Stable on the floor",
            format!("Centre of mass is {m:.1} mm inside the support polygon of wheels and wedge."),
        ));
    }
    if let Some(share) = wedge_share {
        ctx.metrics.insert("wedge_load_frac".into(), share);
        if share > 0.25 {
            out.push(
                Finding::fail("ground.wedge_load", G, Severity::Warning, "Front rests on the wedge", format!("About {:.0}% of the weight sits on the wedge tip because the centre of mass is ahead of the axle; the tip drags and wears and the drive loses traction.", share * 100.0))
                    .fix("Move the battery and motors back, or add a front skid wheel, or put the axle forward."),
            );
        }
    }
    // Tip angle about the nearest support edge.
    let cz = ctx.metrics.get("com_z").copied().unwrap_or(0.0) - floor;
    if m > 0.0 && cz > 0.1 {
        ctx.metrics.insert("tip_angle_deg".into(), (m / cz).atan().to_degrees());
    }
    let _ = v3(0.0, 0.0, 0.0);
    out
}
