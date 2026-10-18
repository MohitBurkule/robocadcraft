//! Join graph: every part must be held to the chassis by a screw, a shared print, a pocket that captures it, a joint
//! or a stated connection, and each stated connection has to be believable for the load and the materials.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::measure;
use crate::spec::{Part, PartKind};

use super::Ctx;

const G: &str = "connections";
const METHODS: [&str; 10] = ["screw", "shaft", "pin", "press_fit", "glue", "tape", "snap", "interlock", "velcro", "none"];
const LOW_ENERGY: [&str; 8] = ["hdpe", "pp", "polypropylene", "pe", "uhmwpe", "nylon", "pa12", "taulman910"];

fn impact_part(p: &Part) -> bool {
    matches!(p.kind, PartKind::Wedge | PartKind::Weapon | PartKind::Armor | PartKind::Fork | PartKind::LockingBar)
}

fn is_low_energy(p: &Part) -> bool {
    p.material.as_deref().is_some_and(|m| LOW_ENERGY.contains(&m.to_ascii_lowercase().as_str()))
}

fn link(edges: &mut BTreeMap<String, BTreeSet<String>>, a: &str, b: &str) {
    edges.entry(a.to_string()).or_default().insert(b.to_string());
    edges.entry(b.to_string()).or_default().insert(a.to_string());
}

fn words(s: &str) -> usize {
    s.split_whitespace().count()
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let mut edges: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut floating_ok: BTreeSet<String> = BTreeSet::new();

    // Screws and inserts.
    for f in &r.spec.fasteners {
        let mut ids: Vec<&str> = f.through.iter().map(String::as_str).collect();
        if let Some(t) = &f.threads_into {
            ids.push(t);
        }
        for pair in ids.windows(2) {
            if let [a, b] = pair {
                link(&mut edges, a, b);
            }
        }
    }
    for p in &r.spec.parts {
        if let Some(fw) = &p.fused_with {
            let other = r.spec.part(fw);
            let same = other.is_some_and(|o| o.material == p.material && catalog::part_is_printed(p.process.as_deref(), p.material.as_deref()));
            if same {
                link(&mut edges, &p.id, fw);
            } else {
                out.push(
                    Finding::fail(
                        "connection.fused_mismatch",
                        G,
                        Severity::Blocker,
                        "Fused parts are not one print",
                        format!("`{}` is fused to `{fw}` but they differ in material or are not both printed; they cannot be one piece.", p.id),
                    )
                    .on(&[&p.id, fw])
                    .fix("Print both in one material, or declare screws instead of `fused_with`."),
                );
            }
        }
        if let Some(t) = &p.tyre_of {
            link(&mut edges, &p.id, t);
        }
        if let Some(b) = &p.belt {
            for pu in &b.pulleys {
                link(&mut edges, &p.id, pu);
            }
        }
        if let Some(wd) = &p.weapon {
            for x in [&wd.motor, &wd.locking_bar].into_iter().flatten() {
                link(&mut edges, &p.id, x);
            }
        }
        // A joint only holds if its pivot is really on both parts.
        if let (Some(j), Some(par)) = (&p.joint, p.parent.as_deref()) {
            let pivot = w.body(par).map(|b| b.xf.apply(crate::math::V3::from_arr(j.pivot)));
            let near = |id: &str| match (pivot, w.index(id)) {
                (Some(pv), Some(i)) => w.sdf(i, pv) <= 1.0,
                _ => false,
            };
            if near(&p.id) && near(par) {
                link(&mut edges, &p.id, par);
            } else {
                out.push(
                    Finding::fail(
                        "connection.joint_pivot",
                        G,
                        Severity::Blocker,
                        "Joint pivot is not on the parts",
                        format!(
                            "`{}` pivots about a point that is more than 1 mm from `{}` or from `{par}`; nothing holds the pivot pin.",
                            p.id, p.id
                        ),
                    )
                    .on(&[&p.id, par])
                    .fix("Put the pivot on the pin that passes through both parts and add that pin as a part."),
                );
            }
        }
    }

    // Declared joins.
    for c in &r.spec.connections {
        let (pa, pb) = (r.spec.part(&c.a), r.spec.part(&c.b));
        let (Some(pa), Some(pb)) = (pa, pb) else {
            out.push(
                Finding::fail("connection.ref", G, Severity::Blocker, "Connection names a missing part", format!("`{}`-`{}`.", c.a, c.b))
                    .on(&[&c.a, &c.b]),
            );
            continue;
        };
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
            continue;
        }
        if c.reason.trim().len() < 30 || words(&c.reason) < 6 {
            out.push(
                Finding::fail(
                    "connection.reason",
                    G,
                    Severity::Blocker,
                    "Connection needs real reasoning",
                    format!("`{}`-`{}` ({}): the reason is too short to be an argument.", c.a, c.b, c.method),
                )
                .on(&[&c.a, &c.b])
                .fix("Say what load the join carries, why this method suits the two materials, and how it is assembled."),
            );
            continue;
        }
        match c.method.as_str() {
            "none" => {
                floating_ok.insert(c.a.clone());
                floating_ok.insert(c.b.clone());
                out.push(
                    Finding::note("connection.free", G, "Parts deliberately unjoined", format!("`{}` and `{}`: {}", c.a, c.b, c.reason))
                        .on(&[&c.a, &c.b]),
                );
                continue;
            }
            "screw" => {
                let ok = edges.get(&c.a).is_some_and(|s| s.contains(&c.b));
                if !ok {
                    out.push(
                        Finding::fail(
                            "connection.screw_missing",
                            G,
                            Severity::Blocker,
                            "Declared screw join has no screw",
                            format!("`{}`-`{}` says screw but no fastener passes through both.", c.a, c.b),
                        )
                        .on(&[&c.a, &c.b])
                        .fix("Add a fastener whose `through` or `threads_into` lists both parts."),
                    );
                }
                continue;
            }
            "glue" | "tape" => {
                if impact_part(pa) || impact_part(pb) {
                    out.push(
                        Finding::fail(
                            "connection.glue_on_impact",
                            G,
                            Severity::Blocker,
                            "Adhesive on a part that takes hits",
                            format!(
                                "`{}`-`{}` is held by {} but one side is a wedge, weapon, armour or fork; hits peel adhesive on the first strike.",
                                c.a, c.b, c.method
                            ),
                        )
                        .on(&[&c.a, &c.b])
                        .fix("Bolt it: add M2 screws into inserts, or an interlocking tab plus screws. Use glue only as a secondary."),
                    );
                    continue;
                }
                if (is_low_energy(pa) || is_low_energy(pb)) && !c.mechanical_key {
                    out.push(
                        Finding::fail("connection.glue_low_energy", G, Severity::Blocker, "Glue will not stick to this plastic", format!("`{}`-`{}`: HDPE, polypropylene and nylon barely take adhesive or tape; set `mechanical_key` only if geometry holds it without the glue.", c.a, c.b))
                            .on(&[&c.a, &c.b])
                            .fix("Use screws, a press fit or an interlock, or add a keyed slot and set `mechanical_key`."),
                    );
                    continue;
                }
            }
            "press_fit" => {
                let ok = c.interference_mm.is_some_and(|d| (0.05..=0.4).contains(&d));
                if !ok {
                    out.push(
                        Finding::fail(
                            "connection.press_fit_interference",
                            G,
                            Severity::Warning,
                            "Press fit has no sensible interference",
                            format!("`{}`-`{}`: give `interference_mm` between 0.05 and 0.4 for a printed part.", c.a, c.b),
                        )
                        .on(&[&c.a, &c.b]),
                    );
                }
            }
            _ => {}
        }
        link(&mut edges, &c.a, &c.b);
    }

    // A pocket that holds a part in every direction joins it to what it touches; a declared hold joins it to what it touches.
    for p in &r.spec.parts {
        let Some(i) = w.index(&p.id) else { continue };
        if let Some(blockers) = ctx.captive.get(&p.id) {
            for b in blockers {
                link(&mut edges, &p.id, b);
            }
        }
        if p.retained.is_some() {
            for (j, o) in w.bodies.iter().enumerate() {
                if j != i
                    && !o.fastener
                    && o.part_idx.is_some()
                    && measure::min_gap(w, i, j, 0.4) <= 0.5
                    && matches!(r.spec.part(&o.id).map(|x| x.kind), Some(PartKind::Structure | PartKind::Plate))
                {
                    link(&mut edges, &p.id, &o.id);
                }
            }
        }
    }

    // Everything must be reachable from the chassis.
    let root = r.spec.parts.iter().find(|p| p.kind == PartKind::Structure).map(|p| p.id.clone());
    if let Some(root) = root {
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut q = VecDeque::from([root]);
        while let Some(id) = q.pop_front() {
            if !seen.insert(id.clone()) {
                continue;
            }
            if let Some(ns) = edges.get(&id) {
                q.extend(ns.iter().filter(|n| !seen.contains(*n)).cloned());
            }
        }
        let mut loose = 0;
        for p in &r.spec.parts {
            if seen.contains(&p.id) || floating_ok.contains(&p.id) {
                continue;
            }
            loose += 1;
            let near = w
                .index(&p.id)
                .map(|i| {
                    w.bodies
                        .iter()
                        .enumerate()
                        .filter(|(j, o)| *j != i && !o.fastener && o.part_idx.is_some())
                        .map(|(j, o)| (measure::min_gap(w, i, j, 0.5), o.id.clone()))
                        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
                })
                .unwrap_or(None);
            let hint = near.map(|(g, id)| format!(" Nearest part is `{id}`, {g:.1} mm away.")).unwrap_or_default();
            out.push(
                Finding::fail("connection.floating", G, Severity::Blocker, "Part is not attached to anything", format!("`{}` has no screw, shared print, captive pocket, joint or declared connection leading back to the chassis.{hint}", p.id))
                    .on(&[&p.id])
                    .here(w.body(&p.id).map(|b| b.aabb.center()).unwrap_or_default())
                    .fix("Add a fastener through it, or list it in `connections` with a method and a reason; use `none` with a reason only if it is meant to be loose."),
            );
        }
        if loose == 0 {
            out.push(Finding::pass(
                "connection.joined",
                G,
                "Every part is attached",
                format!(
                    "All {} parts lead back to the chassis through screws, shared prints, captive pockets, joints or declared connections.",
                    r.spec.parts.len()
                ),
            ));
        }
    }
    out
}
