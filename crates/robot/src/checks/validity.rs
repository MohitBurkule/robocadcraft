//! Model validity: ids, references, shapes, units, claims against measurements.

use std::collections::HashSet;

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::spec::PartKind;
use crate::world::Resolved;

use super::Ctx;

const G: &str = "validity";

pub fn run(r: &Resolved, out: &mut Vec<Finding>) -> bool {
    let mut ok = true;
    if r.spec.parts.is_empty() {
        out.push(
            Finding::fail("validity.no_parts", G, Severity::Blocker, "No parts", "The spec has no parts.")
                .fix("Add parts with a `shape` or a `catalog` id."),
        );
        return false;
    }
    if r.spec.parts.len() > 300 {
        out.push(Finding::fail(
            "validity.too_many_parts",
            G,
            Severity::Blocker,
            "Too many parts",
            format!("{} parts; the limit is 300.", r.spec.parts.len()),
        ));
        return false;
    }
    for (id, e) in &r.load_errors {
        ok = false;
        out.push(
            Finding::fail("validity.load", G, Severity::Blocker, "Part could not be loaded", e.clone())
                .on(&[id])
                .fix("Use a valid catalog id (see the catalog tool) or give a valid `shape`."),
        );
    }
    let mut seen = HashSet::new();
    for p in &r.spec.parts {
        if p.id.is_empty() || !seen.insert(p.id.clone()) {
            ok = false;
            out.push(
                Finding::fail("validity.part_id", G, Severity::Blocker, "Part id missing or duplicated", format!("id `{}`", p.id))
                    .on(&[&p.id])
                    .fix("Give every part a unique non-empty id."),
            );
        }
        if let Some(mat) = &p.material
            && catalog::density(mat).is_none()
        {
            out.push(
                Finding::fail(
                    "validity.material",
                    G,
                    Severity::Warning,
                    "Unknown material",
                    format!("`{mat}` has no density; mass cannot be computed."),
                )
                .on(&[&p.id])
                .fix("Use one of: pla, petg, abs, pc, nylon, tpu, resin, cf_plate, aluminium, steel, titanium, brass, hdpe."),
            );
        }
        if let Some(c) = &p.color
            && crate::world::parse_color(c).is_none()
        {
            out.push(Finding::fail("validity.color", G, Severity::Warning, "Invalid colour", format!("`{c}` is not `#rrggbb`.")).on(&[&p.id]));
        }
        if p.at.iter().chain(p.rot.iter()).any(|v| !v.is_finite()) {
            ok = false;
            out.push(Finding::fail("validity.pose", G, Severity::Blocker, "Non-finite pose", "at/rot contains NaN or infinity.").on(&[&p.id]));
        }
        if p.catalog.is_some() && p.shape.is_some() {
            out.push(
                Finding::fail(
                    "validity.catalog_shape",
                    G,
                    Severity::Warning,
                    "Catalog part with its own shape",
                    "The catalog shape is used; the given `shape` is ignored.",
                )
                .on(&[&p.id])
                .fix("Remove `shape`, or drop `catalog` for a custom part."),
            );
        }
        for refname in [p.parent.as_ref(), p.fused_with.as_ref(), p.cover.as_ref()].into_iter().flatten() {
            if r.spec.part(refname).is_none() {
                ok = false;
                out.push(
                    Finding::fail("validity.ref", G, Severity::Blocker, "Reference to a missing part", format!("`{}` refers to `{refname}`", p.id))
                        .on(&[&p.id]),
                );
            }
        }
    }
    for f in &r.spec.fasteners {
        if catalog::parse_designation(&f.designation).is_none() {
            ok = false;
            out.push(
                Finding::fail(
                    "validity.fastener",
                    G,
                    Severity::Blocker,
                    "Bad fastener designation",
                    format!("`{}` on `{}`; use e.g. `M2x16`.", f.designation, f.id),
                )
                .fix("Use the form M<d>x<length>."),
            );
        }
        let d = f.dir.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !(d.is_finite() && d > 1e-6) {
            ok = false;
            out.push(Finding::fail(
                "validity.fastener_dir",
                G,
                Severity::Blocker,
                "Fastener has no direction",
                format!("`{}` has a zero `dir`.", f.id),
            ));
        }
        for pid in f.through.iter().chain(f.threads_into.iter()) {
            if r.spec.part(pid).is_none() {
                ok = false;
                out.push(Finding::fail(
                    "validity.ref",
                    G,
                    Severity::Blocker,
                    "Reference to a missing part",
                    format!("fastener `{}` refers to `{pid}`", f.id),
                ));
            }
        }
    }
    if ok {
        out.push(Finding::pass(
            "validity.ok",
            G,
            "Spec is well formed",
            format!("{} parts, {} fasteners, {} wires.", r.spec.parts.len(), r.spec.fasteners.len(), r.spec.wires.len()),
        ));
    }
    for rp in &r.parts {
        if matches!(rp.part.kind, PartKind::Motor | PartKind::Battery | PartKind::Esc | PartKind::Receiver | PartKind::Servo | PartKind::WeaponMotor)
            && rp.cat.is_none()
            && rp.part.catalog.is_none()
        {
            out.push(
                Finding::fail(
                    "validity.no_catalog",
                    G,
                    Severity::Warning,
                    "Electronic part without a catalog entry",
                    format!("`{}` has custom dimensions; its mass and ratings are unverified.", rp.part.id),
                )
                .on(&[&rp.part.id])
                .fix("Use a catalog id so dimensions and ratings are locked to the real part."),
            );
        }
    }
    ok
}

/// Claims the author wrote about the design, checked against what the model measures.
pub fn claims(ctx: &mut Ctx) -> Vec<Finding> {
    let mut out = Vec::new();
    for c in &ctx.r.spec.claims {
        let Some(actual) = ctx.metrics.get(&c.metric).copied() else {
            out.push(Finding::fail(
                "claim.unknown_metric",
                "claims",
                Severity::Warning,
                "Claim cannot be checked",
                format!("claim `{}`: no measured metric `{}`", c.id, c.metric),
            ));
            continue;
        };
        let holds = match c.op.as_str() {
            "<=" => actual <= c.value + 1e-9,
            ">=" => actual >= c.value - 1e-9,
            "==" => (actual - c.value).abs() <= 0.05 * c.value.abs().max(1e-9),
            _ => false,
        };
        let msg = format!("\"{}\" ({} {} {})", c.text, c.metric, c.op, c.value);
        out.push(if holds {
            Finding::pass("claim.holds", "claims", "Claim matches the model", msg)
                .expect(format!("{} {} {}", c.metric, c.op, c.value), format!("{actual:.2}"))
        } else {
            Finding::fail("claim.false", "claims", Severity::Warning, "Claim contradicted by the model", msg)
                .expect(format!("{} {} {}", c.metric, c.op, c.value), format!("{actual:.2}"))
                .fix("Fix the model or correct the claim; the measured value is the truth.")
        });
    }
    out
}
