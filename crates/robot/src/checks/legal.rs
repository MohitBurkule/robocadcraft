//! Rules of the event: size, switch, failsafe, weapon controls.

use crate::finding::{Finding, Severity};
use crate::math::v3;

use super::Ctx;

const G: &str = "legal";

/// Smallest square, in any rotation about the vertical axis, that holds every solid body (bounds corners, so slightly conservative).
pub fn footprint_mm(ctx: &Ctx) -> f64 {
    let pts: Vec<_> = ctx.world.bodies.iter().filter(|b| !b.id.starts_with("wire:")).flat_map(|b| b.aabb.corners()).collect();
    let mut best = f64::MAX;
    for deg in 0..90 {
        let a = (deg as f64).to_radians();
        let (s, c) = a.sin_cos();
        let (mut lo, mut hi) = (v3(f64::MAX, f64::MAX, 0.0), v3(f64::MIN, f64::MIN, 0.0));
        for p in &pts {
            let (x, y) = (p.x * c + p.y * s, -p.x * s + p.y * c);
            lo = v3(lo.x.min(x), lo.y.min(y), 0.0);
            hi = v3(hi.x.max(x), hi.y.max(y), 0.0);
        }
        best = best.min((hi.x - lo.x).max(hi.y - lo.y));
    }
    best
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let mut out = Vec::new();
    let limit = ctx.r.profile.footprint_mm;
    let fp = footprint_mm(ctx);
    ctx.metrics.insert("footprint_mm".into(), fp);
    if fp > limit {
        out.push(
            Finding::fail(
                "legal.footprint",
                G,
                Severity::Blocker,
                "Robot does not fit the square",
                format!("Smallest square that holds the robot is {fp:.1} mm; the limit is {limit:.0} mm with no overhang at any height."),
            )
            .expect(format!("<= {limit:.0} mm"), format!("{fp:.1} mm"))
            .fix("Shorten the wedge, narrow the wheels or move parts inward."),
        );
    } else {
        out.push(Finding::pass("legal.footprint", G, "Fits the square", format!("Fits a {fp:.1} mm square (limit {limit:.0} mm).")));
    }
    out
}
