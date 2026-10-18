//! Touching structural parts must be joined by something declared: a screw, a shared print, or a stated connection.

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::measure;
use crate::spec::PartKind;

use super::Ctx;

const G: &str = "connections";
const METHODS: [&str; 10] = ["screw", "shaft", "pin", "press_fit", "glue", "tape", "snap", "interlock", "velcro", "none"];

fn structural(k: PartKind) -> bool {
    matches!(k, PartKind::Structure | PartKind::Armor | PartKind::Wedge | PartKind::Plate | PartKind::Fork | PartKind::LockingBar)
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let mut ok = 0;
    for c in &r.spec.connections {
        if !METHODS.contains(&c.method.as_str()) {
            out.push(
                Finding::fail(
                    "connection.method",
                    G,
                    Severity::Blocker,
                    "Unknown connection method",
                    format!("`{}`-`{}` uses `{}`; use one of {}.", c.a, c.b, c.method, METHODS.join(", ")),
                )
                .on(&[&c.a, &c.b]),
            );
        }
        if c.reason.trim().len() < 8 {
            out.push(
                Finding::fail(
                    "connection.reason",
                    G,
                    Severity::Blocker,
                    "Connection needs a reason",
                    format!("`{}`-`{}` ({}) has no reason; say why this join holds or why it is left loose.", c.a, c.b, c.method),
                )
                .on(&[&c.a, &c.b]),
            );
        }
        if r.spec.part(&c.a).is_none() || r.spec.part(&c.b).is_none() {
            out.push(
                Finding::fail("connection.ref", G, Severity::Blocker, "Connection names a missing part", format!("`{}`-`{}`.", c.a, c.b))
                    .on(&[&c.a, &c.b]),
            );
        }
    }
    let declared = |a: &str, b: &str| r.spec.connections.iter().find(|c| (c.a == a && c.b == b) || (c.a == b && c.b == a));
    let screwed = |a: &str, b: &str| {
        r.spec.fasteners.iter().any(|f| {
            let hits = |id: &str| f.through.iter().any(|t| t == id) || f.threads_into.as_deref() == Some(id);
            hits(a) && hits(b)
        })
    };
    let n = w.bodies.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let (Some(a), Some(b)) = (w.bodies.get(i), w.bodies.get(j)) else { continue };
            if a.fastener || b.fastener || !structural(a.kind) || !structural(b.kind) || !a.aabb.grow(0.4).overlaps(&b.aabb, 0.0) {
                continue;
            }
            let (Some(pa), Some(pb)) = (r.spec.part(&a.id), r.spec.part(&b.id)) else { continue };
            if measure::min_gap(w, i, j, 0.4) > 0.3 {
                continue;
            }
            let (ia, ib) = (&pa.id, &pb.id);
            if pa.parent.as_deref() == Some(ib) || pb.parent.as_deref() == Some(ia) || pa.joint.is_some() || pb.joint.is_some() {
                ok += 1;
                continue;
            }
            if pa.fused_with.as_deref() == Some(ib) || pb.fused_with.as_deref() == Some(ia) {
                let same = pa.material == pb.material && catalog::part_is_printed(pa.process.as_deref(), pa.material.as_deref());
                if same {
                    ok += 1;
                } else {
                    out.push(
                        Finding::fail(
                            "connection.fused_mismatch",
                            G,
                            Severity::Blocker,
                            "Fused parts are not one print",
                            format!("`{ia}` is fused to `{ib}` but they differ in material or are not both printed; they cannot be one piece."),
                        )
                        .on(&[ia, ib])
                        .fix("Print both in one material, or declare screws or glue instead of `fused_with`."),
                    );
                }
                continue;
            }
            if screwed(ia, ib) {
                ok += 1;
                continue;
            }
            match declared(ia, ib) {
                Some(c) if c.method == "none" => {
                    out.push(Finding::note("connection.free", G, "Touching parts deliberately unjoined", format!("`{ia}` and `{ib}`: {}", c.reason)).on(&[ia, ib]));
                }
                Some(c) if c.method == "screw" => out.push(
                    Finding::fail("connection.screw_missing", G, Severity::Blocker, "Declared screw join has no screw", format!("`{ia}`-`{ib}` says screw but no fastener passes through both."))
                        .on(&[ia, ib])
                        .fix("Add a fastener whose `through` or `threads_into` lists both parts."),
                ),
                Some(_) => ok += 1,
                None => out.push(
                    Finding::fail("connection.undeclared", G, Severity::Blocker, "Parts touch but nothing joins them", format!("`{ia}` and `{ib}` sit against each other with no screw, shared print or declared connection; they would just fall apart."))
                        .on(&[ia, ib])
                        .here(a.aabb.intersect(&b.aabb.grow(0.4)).center())
                        .fix("Add a fastener through both, or list the pair in `connections` with a method (screw, shaft, pin, press_fit, glue, tape, snap, interlock, velcro) and a reason, or `none` with a reason if loose is intended."),
                ),
            }
        }
    }
    if out.iter().all(|f| f.pass || f.id == "connection.free") {
        out.push(Finding::pass(
            "connection.joined",
            G,
            "Touching structural parts are joined",
            format!("{ok} touching pairs each have a screw, shared print, joint or declared connection."),
        ));
    }
    out
}
