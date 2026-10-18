//! Collisions: static overlaps, joint sweeps, spinning-weapon sweeps, and "what holds this part".

use std::collections::{BTreeMap, HashSet};

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};
use crate::measure;
use crate::spec::PartKind;
use crate::world::{Body, Resolved, World};

use super::Ctx;

const G: &str = "interference";

/// Maximum overlap depth allowed between two bodies (`Some(0.0)` = none), from the spec and from how
/// parts are meant to fit together.
pub fn allowed_depth(r: &Resolved, a: &Body, b: &Body) -> f64 {
    let tol = r.profile.contact_tol_mm;
    let mut allow = tol;
    let ids = (a.id.as_str(), b.id.as_str());
    for ao in &r.spec.allow_overlap {
        if (ao.a == ids.0 && ao.b == ids.1) || (ao.a == ids.1 && ao.b == ids.0) {
            allow = allow.max(ao.max_depth_mm.unwrap_or(0.5));
        }
    }
    let part = |x: &Body| x.part_idx.and_then(|i| r.parts.get(i)).map(|p| &p.part);
    let (pa, pb) = (part(a), part(b));
    if let (Some(pa), Some(pb)) = (pa, pb) {
        if pa.fused_with.as_deref() == Some(&pb.id) || pb.fused_with.as_deref() == Some(&pa.id) {
            return f64::INFINITY;
        }
        // Stretched tyres, belts on pulleys, shafts in hubs and bores.
        if pa.tyre_of.as_deref() == Some(&pb.id) || pb.tyre_of.as_deref() == Some(&pa.id) {
            allow = allow.max(1.0);
        }
        for (x, y) in [(pa, pb), (pb, pa)] {
            if let Some(bd) = &x.belt
                && bd.pulleys.iter().any(|p| *p == y.id)
            {
                allow = allow.max(0.8);
            }
            if let Some(w) = &x.weapon
                && (w.motor.as_deref() == Some(&y.id) || w.locking_bar.as_deref() == Some(&y.id))
            {
                allow = allow.max(1.0e3);
            }
        }
    }
    // A screw is meant to sit in the part that holds its thread.
    for (x, y) in [(a, b), (b, a)] {
        if x.fastener
            && let Some(f) = r.spec.fasteners.iter().find(|f| f.id == x.id)
            && f.threads_into.as_deref() == Some(&y.id)
        {
            return f64::INFINITY;
        }
    }
    allow
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let mut out = Vec::new();
    let r = ctx.r;
    let w = ctx.world;
    let n = w.bodies.len();
    let mut bad = 0;
    let mut checked = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            let (Some(a), Some(b)) = (w.bodies.get(i), w.bodies.get(j)) else { continue };
            if !a.aabb.overlaps(&b.aabb, 0.0) {
                continue;
            }
            let allow = allowed_depth(r, a, b);
            if allow.is_infinite() {
                continue;
            }
            checked += 1;
            if let Some(o) = measure::overlap(w, i, j, 0.5)
                && o.depth > allow
            {
                bad += 1;
                out.push(
                    Finding::fail(
                        "interference.overlap",
                        G,
                        Severity::Blocker,
                        "Parts collide",
                        format!("`{}` and `{}` overlap by {:.2} mm (about {:.1} mm3).", a.id, b.id, o.depth, o.volume),
                    )
                    .on(&[&a.id, &b.id])
                    .here(o.at)
                    .expect(format!("overlap <= {allow:.2} mm (touching is fine)"), format!("{:.2} mm", o.depth))
                    .fix("Move or resize one of them, cut a pocket in the housing, or list a deliberate press fit in `allow_overlap`."),
                );
            }
        }
    }
    if bad == 0 {
        out.push(Finding::pass(
            "interference.overlap",
            G,
            "No parts collide",
            format!(
                "{checked} touching or overlapping pairs checked; touching is allowed, overlap deeper than {:.2} mm is not.",
                r.profile.contact_tol_mm
            ),
        ));
    }
    if ctx.opts.deep {
        out.extend(joint_sweeps(ctx));
        out.extend(spin_sweeps(ctx));
    }
    out.extend(captive(ctx));
    out
}

fn descendants(r: &Resolved, root: &str) -> HashSet<String> {
    let mut set: HashSet<String> = HashSet::new();
    set.insert(root.to_string());
    loop {
        let before = set.len();
        for p in &r.spec.parts {
            if let Some(par) = &p.parent
                && set.contains(par)
            {
                set.insert(p.id.clone());
            }
        }
        if set.len() == before {
            break;
        }
    }
    set
}

fn joint_sweeps(ctx: &Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let mut out = Vec::new();
    let mut any = false;
    for rp in &r.parts {
        let Some(j) = &rp.part.joint else { continue };
        any = true;
        let moved = descendants(r, &rp.part.id);
        let (lo, hi) = (j.min.min(j.max), j.min.max(j.max));
        let step = if j.kind == "prismatic" { 0.8 } else { 3.0 };
        let n = (((hi - lo) / step).ceil() as usize).clamp(1, 80);
        let mut worst: BTreeMap<(String, String), (f64, f64, V3)> = BTreeMap::new();
        for k in 0..=n {
            let v = lo + (hi - lo) * k as f64 / n as f64;
            let jw = BTreeMap::from([(rp.part.id.clone(), v)]);
            let w = World::build(r, &jw);
            for (i, a) in w.bodies.iter().enumerate() {
                if !moved.contains(&a.id) {
                    continue;
                }
                for (jx, b) in w.bodies.iter().enumerate() {
                    if moved.contains(&b.id) || !a.aabb.overlaps(&b.aabb, 0.0) {
                        continue;
                    }
                    let allow = allowed_depth(r, a, b);
                    if allow.is_infinite() {
                        continue;
                    }
                    if let Some(o) = measure::overlap(&w, i, jx, 0.6)
                        && o.depth > allow
                    {
                        let e = worst.entry((a.id.clone(), b.id.clone())).or_insert((0.0, v, o.at));
                        if o.depth > e.0 {
                            *e = (o.depth, v, o.at);
                        }
                    }
                }
            }
        }
        let unit = if j.kind == "prismatic" { "mm" } else { "deg" };
        for ((a, b), (depth, v, at)) in &worst {
            out.push(
                Finding::fail(
                    "interference.sweep",
                    G,
                    Severity::Blocker,
                    "Moving part hits another part",
                    format!("`{a}` moving on joint `{}` runs into `{b}` (depth {depth:.2} mm) at {v:.1} {unit}.", rp.part.id),
                )
                .on(&[a, b])
                .here(*at)
                .expect(format!("no contact anywhere in {lo:.0}..{hi:.0} {unit}"), format!("contact at {v:.1} {unit}"))
                .fix("Limit the joint range (`min`/`max`), move the pivot, or cut clearance into the part it hits."),
            );
        }
        if worst.is_empty() {
            out.push(Finding::pass(
                "interference.sweep",
                G,
                "Joint sweep is clear",
                format!("`{}` swept {lo:.0}..{hi:.0} {unit} in {n} steps without hitting anything.", rp.part.id),
            ));
        }
    }
    if !any {
        out.push(Finding::note("interference.sweep", G, "No joints to sweep", "No part has a joint."));
    }
    out
}

/// Occupancy of a part revolved about an axis, on a grid of (radius, axial position).
struct Revolved {
    origin: V3,
    axis: V3,
    cell: f64,
    nr: usize,
    na: usize,
    amin: f64,
    occ: Vec<bool>,
}

impl Revolved {
    fn build(w: &World, i: usize, origin: V3, axis: V3, cell: f64) -> Option<Revolved> {
        let b = w.bodies.get(i)?;
        let (mut rmax, mut amin, mut amax) = (0.0_f64, f64::MAX, f64::MIN);
        for c in b.aabb.corners() {
            let q = c - origin;
            let a = q.dot(axis);
            rmax = rmax.max((q - axis * a).len());
            amin = amin.min(a);
            amax = amax.max(a);
        }
        let nr = (rmax / cell).ceil() as usize + 2;
        let na = ((amax - amin) / cell).ceil() as usize + 2;
        if nr * na > 4_000_000 {
            return None;
        }
        let mut rv = Revolved { origin, axis, cell, nr, na, amin, occ: vec![false; nr * na] };
        let h = cell;
        let bb = b.aabb.grow(0.2);
        let (nx, ny, nz) =
            (((bb.max.x - bb.min.x) / h).ceil() as i64, ((bb.max.y - bb.min.y) / h).ceil() as i64, ((bb.max.z - bb.min.z) / h).ceil() as i64);
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let p = v3(bb.min.x + (ix as f64 + 0.5) * h, bb.min.y + (iy as f64 + 0.5) * h, bb.min.z + (iz as f64 + 0.5) * h);
                    if w.sdf(i, p) < 0.0
                        && let Some(k) = rv.cell_of(p)
                        && let Some(o) = rv.occ.get_mut(k)
                    {
                        *o = true;
                    }
                }
            }
        }
        Some(rv)
    }
    fn cell_of(&self, p: V3) -> Option<usize> {
        let q = p - self.origin;
        let a = q.dot(self.axis);
        let r = (q - self.axis * a).len();
        let (ir, ia) = ((r / self.cell) as i64, ((a - self.amin) / self.cell).floor() as i64);
        if ir < 0 || ia < 0 || ir as usize >= self.nr || ia as usize >= self.na {
            return None;
        }
        Some(ia as usize * self.nr + ir as usize)
    }
    /// Occupied within `pad` cells.
    fn near(&self, p: V3, pad: i64) -> bool {
        let q = p - self.origin;
        let a = q.dot(self.axis);
        let r = (q - self.axis * a).len();
        let (ir, ia) = ((r / self.cell) as i64, ((a - self.amin) / self.cell).floor() as i64);
        for da in -pad..=pad {
            for dr in -pad..=pad {
                let (x, y) = (ir + dr, ia + da);
                if x < 0 || y < 0 || x as usize >= self.nr || y as usize >= self.na {
                    continue;
                }
                if self.occ.get(y as usize * self.nr + x as usize).copied().unwrap_or(false) {
                    return true;
                }
            }
        }
        false
    }
}

pub fn spin_axis(w: &World, r: &Resolved, i: usize) -> Option<(V3, V3)> {
    let b = w.bodies.get(i)?;
    let rp = r.parts.get(b.part_idx?)?;
    let wd = rp.part.weapon.as_ref()?;
    let local = wd.spin_axis.map(V3::from_arr).unwrap_or(v3(0.0, 0.0, 1.0));
    Some((b.xf.t, b.xf.apply_dir(local).norm()))
}

fn spin_sweeps(ctx: &Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    for (i, b) in w.bodies.iter().enumerate() {
        let Some(rp) = b.part_idx.and_then(|k| r.parts.get(k)) else { continue };
        let Some(wd) = &rp.part.weapon else { continue };
        if !matches!(wd.kind.as_str(), "spinner" | "drum" | "disc" | "bar") {
            continue;
        }
        let Some((origin, axis)) = spin_axis(w, r, i) else { continue };
        let Some(rev) = Revolved::build(w, i, origin, axis, 0.5) else {
            out.push(
                Finding::fail("interference.spin", G, Severity::Warning, "Weapon sweep not checked", "The weapon is too large to rasterise.")
                    .on(&[&b.id]),
            );
            continue;
        };
        let rmax = rev.nr as f64 * rev.cell;
        let mut hit_ids: BTreeMap<String, (bool, V3)> = BTreeMap::new();
        for (j, o) in w.bodies.iter().enumerate() {
            if j == i || o.fastener && false {
                continue;
            }
            if allowed_depth(r, b, o).is_infinite() || allowed_depth(r, b, o) >= 1.0e3 {
                continue;
            }
            // Body box against the swept cylinder.
            let reach = rmax + 1.0;
            let near_axis = o.aabb.corners().iter().any(|c| ((*c - origin) - axis * (*c - origin).dot(axis)).len() < reach + o.aabb.size().len());
            if !near_axis {
                continue;
            }
            let bb = o.aabb;
            let h = 0.5;
            let (nx, ny, nz) =
                (((bb.max.x - bb.min.x) / h).ceil() as i64, ((bb.max.y - bb.min.y) / h).ceil() as i64, ((bb.max.z - bb.min.z) / h).ceil() as i64);
            for iz in 0..nz.max(1) {
                for iy in 0..ny.max(1) {
                    for ix in 0..nx.max(1) {
                        let p = v3(bb.min.x + (ix as f64 + 0.5) * h, bb.min.y + (iy as f64 + 0.5) * h, bb.min.z + (iz as f64 + 0.5) * h);
                        let q = p - origin;
                        let rad = (q - axis * q.dot(axis)).len();
                        if rad > reach {
                            continue;
                        }
                        if w.sdf(j, p) >= 0.0 {
                            continue;
                        }
                        let hit = rev.near(p, 0);
                        let close = !hit && rev.near(p, ((ctx.r.profile.min_clearance_moving_mm / rev.cell).ceil() as i64).max(1));
                        if hit || close {
                            let e = hit_ids.entry(o.id.clone()).or_insert((false, p));
                            if hit && !e.0 {
                                *e = (true, p);
                            }
                        }
                    }
                }
            }
        }
        for (id, (hit, at)) in &hit_ids {
            if *hit {
                out.push(
                    Finding::fail(
                        "interference.spin",
                        G,
                        Severity::Blocker,
                        "Spinning weapon hits the robot",
                        format!("`{}` spinning sweeps through `{id}`.", b.id),
                    )
                    .on(&[&b.id, id])
                    .here(*at)
                    .fix("Move the part out of the swept disc, shrink the weapon, or add an explicit gap of at least 1 mm."),
                );
            } else {
                out.push(
                    Finding::fail(
                        "interference.spin_clearance",
                        G,
                        Severity::Warning,
                        "Spinning weapon has almost no clearance",
                        format!(
                            "`{id}` is within {:.1} mm of the sweep of `{}`; bearings, flex and hits will make them touch.",
                            ctx.r.profile.min_clearance_moving_mm, b.id
                        ),
                    )
                    .on(&[&b.id, id])
                    .here(*at)
                    .fix("Leave at least 1 mm between a spinning weapon and anything fixed."),
                );
            }
        }
        if hit_ids.is_empty() {
            out.push(Finding::pass(
                "interference.spin",
                G,
                "Weapon sweep is clear",
                format!("`{}` swept a full turn without touching another part.", b.id),
            ));
        }
    }
    out
}

fn captive(ctx: &Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let slop = r.profile.capture_slop_mm;
    let dirs = [
        ("+x", v3(1.0, 0.0, 0.0)),
        ("-x", v3(-1.0, 0.0, 0.0)),
        ("+y", v3(0.0, 1.0, 0.0)),
        ("-y", v3(0.0, -1.0, 0.0)),
        ("+z", v3(0.0, 0.0, 1.0)),
        ("-z", v3(0.0, 0.0, -1.0)),
    ];
    let mut checked = 0;
    let mut loose_ok = 0;
    for (i, b) in w.bodies.iter().enumerate() {
        let Some(rp) = b.part_idx.and_then(|k| r.parts.get(k)) else { continue };
        let p = &rp.part;
        let must = matches!(p.kind, PartKind::Battery | PartKind::Motor | PartKind::WeaponMotor | PartKind::Servo);
        let should = matches!(
            p.kind,
            PartKind::Esc
                | PartKind::Receiver
                | PartKind::Switch
                | PartKind::Insert
                | PartKind::Nut
                | PartKind::Bearing
                | PartKind::Shaft
                | PartKind::Pin
        );
        if !(must || should) || p.fused_with.is_some() {
            continue;
        }
        let held_by_screw = r.spec.fasteners.iter().any(|f| f.threads_into.as_deref() == Some(&p.id) || f.through.iter().any(|t| *t == p.id));
        if held_by_screw {
            continue;
        }
        checked += 1;
        if let Some(how) = &p.retained {
            loose_ok += 1;
            out.push(Finding::note("retention.declared", G, "Held by declared method", format!("`{}` is held by {how}.", p.id)).on(&[&p.id]));
            continue;
        }
        let mut free = Vec::new();
        for (name, d) in &dirs {
            let blocked = w.bodies.iter().enumerate().any(|(j, o)| {
                j != i && o.aabb.overlaps(&b.aabb.grow(slop), 0.0) && allowed_depth(r, b, o) < 1.0e3 && {
                    let allow = allowed_depth(r, b, o);
                    measure::overlap_shifted(w, i, j, *d * slop, 0.5).is_some_and(|ov| ov.depth > allow + 0.02)
                }
            });
            if !blocked {
                free.push(*name);
            }
        }
        if free.is_empty() {
            continue;
        }
        let sev = if must { Severity::Blocker } else { Severity::Warning };
        out.push(
            Finding::fail("retention.loose", G, sev, "Nothing holds this part in place", format!("`{}` can move more than {slop:.1} mm toward {} without touching anything; in a hit it will fall out or rattle.", p.id, free.join(", ")))
                .on(&[&p.id])
                .here(b.aabb.center())
                .fix("Make the pocket snug on that side (0.2 to 0.5 mm gap), add a retaining rib or strap, or declare how it is held in `retained` (tape, adhesive, velcro, press_fit)."),
        );
    }
    if checked > 0 && out.iter().all(|f| f.pass) {
        out.push(Finding::pass(
            "retention.captive",
            G,
            "Every loose part is held",
            format!("{checked} parts checked, {loose_ok} held by a declared method, the rest captive in their pockets."),
        ));
    }
    out
}

pub fn is_printed_part(r: &Resolved, id: &str) -> bool {
    r.spec.part(id).is_some_and(|p| catalog::part_is_printed(p.process.as_deref(), p.material.as_deref()))
}
