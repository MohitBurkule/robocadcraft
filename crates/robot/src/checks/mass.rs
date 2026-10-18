//! Mass and balance, from the model and the catalog, with an honest uncertainty range.

use crate::catalog;
use crate::finding::{Finding, MassLine, Severity};
use crate::math::V3;
use crate::measure;
use crate::spec::Locomotion;

use super::Ctx;

const G: &str = "mass";

fn kind_name(k: crate::spec::PartKind) -> String {
    serde_json::to_value(k).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_default()
}

pub fn run(ctx: &mut Ctx) -> (Vec<MassLine>, Vec<Finding>) {
    let r = ctx.r;
    let mut lines: Vec<MassLine> = Vec::new();
    let mut out = Vec::new();
    let mut com = V3::ZERO;
    let mut total = 0.0;
    let mut missing = Vec::new();
    for (i, b) in ctx.world.bodies.iter().enumerate() {
        let (grams, tol, source, kind) = if b.fastener {
            let m = measure::moments(ctx.world, i, 0.15);
            (m.volume * 7.85e-3, 0.05, "fastener", "fastener".to_string())
        } else {
            let Some(rp) = b.part_idx.and_then(|k| r.parts.get(k)) else { continue };
            let p = &rp.part;
            let kind = kind_name(p.kind);
            let cal = r.spec.calibration.get(&kind).copied().unwrap_or(1.0);
            let (g, t, src) = if let Some(m) = p.scale_mass_g {
                (m, 0.005, "scale")
            } else if let Some(m) = p.slicer_mass_g {
                (m, 0.06, "slicer")
            } else if let Some(c) = &rp.cat {
                let t = match c.confidence.as_str() {
                    "measured" => 0.015,
                    "datasheet" => 0.03,
                    _ => 0.15,
                };
                (c.mass_g, t, "catalog")
            } else if let Some(d) = p.material.as_deref().and_then(catalog::density) {
                let vol = measure::moments(ctx.world, i, ctx.opts.step_mm).volume;
                let printed = p.process.as_deref().map(catalog::is_printed).unwrap_or(false);
                let frac = p.solid_fraction.unwrap_or(if printed { 0.85 } else { 1.0 });
                (vol * 1e-3 * d * frac, if printed { 0.2 } else { 0.04 }, "computed")
            } else if let Some(m) = p.mass_g {
                (m, 0.15, "declared")
            } else {
                missing.push(p.id.clone());
                continue;
            };
            if let (Some(decl), Some(c)) = (p.mass_g, &rp.cat)
                && (decl - c.mass_g).abs() > 0.1 * c.mass_g.max(0.1)
            {
                out.push(
                    Finding::fail(
                        "mass.declared_vs_catalog",
                        G,
                        Severity::Warning,
                        "Declared mass disagrees with the catalog",
                        format!("`{}`: declared {decl:.1} g, catalog {:.1} g; the catalog value is used.", p.id, c.mass_g),
                    )
                    .on(&[&p.id]),
                );
            }
            (g * cal, t, src, kind)
        };
        let m = measure::moments(ctx.world, i, 0.4);
        com = com + m.centroid * grams;
        total += grams;
        ctx.metrics.insert(format!("part_mass_g.{}", b.id), grams);
        lines.push(MassLine { id: b.id.clone(), kind, grams, tol_g: grams * tol, source: source.into() });
    }
    for id in &missing {
        out.push(
            Finding::fail(
                "mass.unknown",
                G,
                Severity::Blocker,
                "Mass cannot be determined",
                format!("`{id}` has no catalog entry, material or `mass_g`."),
            )
            .on(&[id])
            .fix("Set `material` (and `process`), or `slicer_mass_g` / `scale_mass_g`."),
        );
    }
    // Wires and unmodelled bits.
    let wire_g = ctx.metrics.get("wire_mass_g").copied().unwrap_or_else(|| super::wiring::estimate_length(ctx) * 0.0022);
    if wire_g > 0.0 {
        lines.push(MassLine { id: "wires".into(), kind: "wire".into(), grams: wire_g, tol_g: wire_g * 0.3, source: "wire".into() });
        total += wire_g;
    }
    let allow = r.profile.allowance_g;
    lines.push(MassLine { id: "allowance".into(), kind: "allowance".into(), grams: allow, tol_g: allow * 0.5, source: "declared".into() });
    total += allow;
    let tol: f64 = lines.iter().map(|l| l.tol_g).sum();
    let limit = r.profile.mass_limit(r.spec.locomotion == Locomotion::Roller);
    let (lo, hi) = (total - tol, total + tol);
    ctx.metrics.insert("mass_total_g".into(), total);
    ctx.metrics.insert("mass_lo_g".into(), lo);
    ctx.metrics.insert("mass_hi_g".into(), hi);
    ctx.metrics.insert("mass_limit_g".into(), limit);
    let body_mass = total - allow - wire_g;
    if body_mass > 1e-9 {
        let c = com * (1.0 / (total - allow - wire_g).max(1e-9));
        ctx.metrics.insert("com_x".into(), c.x);
        ctx.metrics.insert("com_y".into(), c.y);
        ctx.metrics.insert("com_z".into(), c.z);
    }
    let est = lines.iter().filter(|l| l.source == "catalog" || l.source == "declared").map(|l| l.tol_g).sum::<f64>();
    let msg = format!(
        "{total:.1} g (range {lo:.1} to {hi:.1} g) against the {limit:.0} g limit; {est:.1} g of the uncertainty is catalog estimates, weigh the real parts and set `scale_mass_g`."
    );
    if lo > limit {
        out.push(
            Finding::fail("mass.over", G, Severity::Blocker, "Over the weight limit", msg)
                .expect(format!("<= {limit:.0} g"), format!("{total:.1} g"))
                .fix("Hollow structure, thin walls, smaller battery or motor; biggest masses are listed in the report."),
        );
    } else if hi > limit {
        let sev = if ctx.opts.strict_catalog { Severity::Blocker } else { Severity::Warning };
        out.push(
            Finding::fail("mass.may_be_over", G, sev, "May be over the weight limit", msg)
                .expect(format!("<= {limit:.0} g"), format!("{total:.1} g (up to {hi:.1})"))
                .fix("Weigh real parts and set `scale_mass_g`, or remove mass until the upper bound fits."),
        );
    } else {
        out.push(Finding::pass("mass.ok", G, "Within the weight limit", msg).expect(format!("<= {limit:.0} g"), format!("{total:.1} g")));
    }
    (lines, out)
}
