//! Screws: can the driver reach the head, does the thread bite enough, does the head sit on something.

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};

use super::Ctx;

const G: &str = "fasteners";

/// Radius of the clear cylinder a driver shaft needs, by tool.
fn tool_radius(tool: &str) -> f64 {
    let size: f64 = tool.trim_start_matches(|c: char| c.is_alphabetic()).parse().unwrap_or(1.5);
    match tool {
        t if t.starts_with("hex") => size * 0.5 * 1.16 + 0.4,
        t if t.starts_with("torx") => 1.5,
        t if t.starts_with("phillips") => 1.7,
        _ => 1.8,
    }
}

fn perp(d: V3) -> (V3, V3) {
    let a = if d.x.abs() < 0.9 { v3(1.0, 0.0, 0.0) } else { v3(0.0, 1.0, 0.0) };
    let u = d.cross(a).norm();
    (u, d.cross(u).norm())
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let mut ok = 0;
    for f in &r.spec.fasteners {
        let Some((d, len)) = catalog::parse_designation(&f.designation) else { continue };
        let (dk, k, _) = catalog::screw_dims(d);
        let dir = V3::from_arr(f.dir).norm();
        let at = V3::from_arr(f.at);
        let own = w.index(&f.id);
        let mut bad = false;
        // Driver access: a cylinder out of the head, along the opposite of the shank direction.
        if let Some(tool) = &f.tool {
            let rad = tool_radius(tool);
            let reach = r.profile.driver_access_len_mm;
            let top = at - dir * k;
            let (u, v) = perp(dir);
            let mut hit: Option<(String, f64)> = None;
            let steps = (reach / 0.5) as usize;
            'scan: for s in 0..=steps {
                let t = s as f64 * 0.5;
                let c = top - dir * (t + 0.05);
                for a in 0..8 {
                    let ang = a as f64 * std::f64::consts::FRAC_PI_4;
                    let p = if a == 0 { c } else { c + u * (rad * ang.cos()) + v * (rad * ang.sin()) };
                    for (i, b) in w.bodies.iter().enumerate() {
                        if Some(i) == own || b.id.starts_with("wire:") || !b.aabb.contains(p) {
                            continue;
                        }
                        if w.sdf(i, p) < 0.0 {
                            hit = Some((b.id.clone(), t));
                            break 'scan;
                        }
                    }
                }
            }
            if let Some((id, t)) = hit {
                bad = true;
                out.push(
                    Finding::fail("fastener.access", G, Severity::Blocker, "Screwdriver cannot reach the screw", format!("`{}` ({tool}) is blocked by `{id}` {t:.1} mm above the head; the driver needs a clear {:.1} mm wide, {reach:.0} mm long path straight out of the head.", f.id, rad * 2.0))
                        .on(&[&f.id, &id])
                        .here(top - dir * t)
                        .fix("Open a straight access hole or move the screw out from under the overhang; counterbores need to be wider than the head."),
                );
            }
        } else {
            out.push(
                Finding::fail(
                    "fastener.no_tool",
                    G,
                    Severity::Warning,
                    "Screw has no driver listed",
                    format!("`{}` has no `tool`, so access cannot be checked.", f.id),
                )
                .on(&[&f.id]),
            );
        }
        // Thread engagement in the part that holds the thread.
        if let Some(t) = &f.threads_into
            && let Some(tb) = w.body(t)
        {
            let tip = at + dir * len;
            let (lo, hi) = (at.dot(dir), tip.dot(dir));
            let cs: Vec<f64> = tb.aabb.corners().iter().map(|c| c.dot(dir)).collect();
            let (plo, phi) = (cs.iter().cloned().fold(f64::INFINITY, f64::min), cs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
            let engaged = (hi.min(phi) - lo.max(plo)).max(0.0);
            let need = r.profile.engagement_d * d;
            if engaged + 0.01 < need {
                bad = true;
                out.push(
                    Finding::fail(
                        "fastener.engagement",
                        G,
                        Severity::Blocker,
                        "Screw does not bite enough thread",
                        format!("`{}` engages {engaged:.1} mm in `{t}`; it needs at least {need:.1} mm (1.5 x diameter) or it will strip.", f.id),
                    )
                    .on(&[&f.id, t])
                    .expect(format!(">= {need:.1} mm"), format!("{engaged:.1} mm"))
                    .fix("Use a longer screw, a longer insert, or move the insert closer to the screw tip."),
                );
            }
            if hi > phi + 4.0 {
                out.push(
                    Finding::fail(
                        "fastener.too_long",
                        G,
                        Severity::Warning,
                        "Screw sticks out far past its nut",
                        format!("`{}` pokes {:.1} mm beyond `{t}`; it will catch on wires or the floor.", f.id, hi - phi),
                    )
                    .on(&[&f.id, t]),
                );
            }
        } else if f.threads_into.is_none() && f.through.len() < 2 {
            out.push(
                Finding::fail(
                    "fastener.no_thread",
                    G,
                    Severity::Warning,
                    "Screw holds nothing",
                    format!("`{}` has neither `threads_into` nor two `through` parts.", f.id),
                )
                .on(&[&f.id]),
            );
        }
        // Head seat: material must be just under the head.
        let (u, v) = perp(dir);
        let seat = (0..8).any(|a| {
            let ang = a as f64 * std::f64::consts::FRAC_PI_4;
            let p = at + dir * 0.15 + u * (dk * 0.36 * ang.cos()) + v * (dk * 0.36 * ang.sin());
            w.bodies.iter().enumerate().any(|(i, b)| Some(i) != own && !b.id.starts_with("wire:") && b.aabb.contains(p) && w.sdf(i, p) < 0.0)
        });
        if !seat {
            bad = true;
            out.push(
                Finding::fail(
                    "fastener.head_seat",
                    G,
                    Severity::Warning,
                    "Screw head sits on nothing",
                    format!("Nothing is under the head of `{}`; it cannot clamp the parts.", f.id),
                )
                .on(&[&f.id])
                .here(at)
                .fix("Move `at` to the surface the head presses on, or cut a counterbore whose floor is at `at`."),
            );
        }
        if !bad {
            ok += 1;
        }
    }
    if !r.spec.fasteners.is_empty() && out.iter().all(|f| f.pass) {
        out.push(Finding::pass(
            "fastener.ok",
            G,
            "Screws are reachable and bite",
            format!("{ok} screws: driver path clear, thread engagement enough, heads seated."),
        ));
    }
    out
}
