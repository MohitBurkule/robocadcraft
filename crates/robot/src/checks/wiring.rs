//! Wiring: connection graph and routed paths (filled in below).

use crate::finding::Finding;

use super::Ctx;

pub fn estimate_length(ctx: &Ctx) -> f64 {
    let mut total = 0.0;
    for w in &ctx.r.spec.wires {
        if let (Some(a), Some(b)) = (ctx.world.port(ctx.r, &w.from), ctx.world.port(ctx.r, &w.to)) {
            let d = b - a;
            total += (d.x.abs() + d.y.abs() + d.z.abs()) * 1.15 + w.slack_mm.unwrap_or(8.0);
        }
    }
    total
}

pub fn run(_ctx: &mut Ctx) -> Vec<Finding> {
    Vec::new()
}
