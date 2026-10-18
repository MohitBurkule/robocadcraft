//! Assembly by disassembly: take the robot apart one part at a time, each sliding straight out along a clear path.
//! If every part can leave, the reverse order assembles it without anything passing through anything else.

use serde::{Deserialize, Serialize};

use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};
use crate::measure;
use crate::spec::PartKind;

use super::Ctx;
use super::interference::allowed_depth;

const G: &str = "assembly";
const STEP: f64 = 0.7;
/// Surface samples per body, built on first use.
pub struct Surfaces {
    cells: std::cell::RefCell<Vec<Option<std::rc::Rc<Vec<V3>>>>>,
}

impl Surfaces {
    fn new(n: usize) -> Surfaces {
        Surfaces { cells: std::cell::RefCell::new(vec![None; n]) }
    }
    fn get(&self, w: &crate::world::World, i: usize) -> std::rc::Rc<Vec<V3>> {
        let mut c = self.cells.borrow_mut();
        if let Some(Some(v)) = c.get(i) {
            return v.clone();
        }
        let v = std::rc::Rc::new(measure::surface_points(w, i, 0.8));
        if let Some(slot) = c.get_mut(i) {
            *slot = Some(v.clone());
        }
        v
    }
}

const MAX_TRAVEL: f64 = 140.0;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AssemblyStep {
    /// Position in the disassembly order (1 = comes off first).
    pub order: usize,
    pub id: String,
    /// Unit direction the part slides out along.
    pub dir: [f64; 3],
    /// Travel in mm until it is clear of everything still in place.
    pub travel_mm: f64,
}

/// Members that move with part `id` (tyres on their wheel, children on a joint).
fn group(ctx: &Ctx, id: &str) -> Vec<usize> {
    let r = ctx.r;
    let mut ids = vec![id.to_string()];
    loop {
        let n = ids.len();
        for p in &r.spec.parts {
            if (p.tyre_of.as_deref().is_some_and(|t| ids.iter().any(|i| i == t)) || p.parent.as_deref().is_some_and(|t| ids.iter().any(|i| i == t)))
                && !ids.contains(&p.id)
            {
                ids.push(p.id.clone());
            }
        }
        if ids.len() == n {
            break;
        }
    }
    ids.iter().filter_map(|i| ctx.world.index(i)).collect()
}

/// Slide `members` along `dir`; `Ok(travel)` when they get clear of every body in `present`, `Err(j)` for the first body hit.
fn slide(ctx: &Ctx, pts: &Surfaces, members: &[usize], present: &[bool], dir: V3) -> Result<f64, (usize, f64, V3)> {
    let w = ctx.world;
    let r = ctx.r;
    let steps = (MAX_TRAVEL / STEP) as usize;
    let mut last_overlap_t = 0.0;
    // Once the moving group is wholly beyond every other body along `dir`, nothing more can be hit.
    let others_max = w
        .bodies
        .iter()
        .enumerate()
        .filter(|(j, o)| present.get(*j).copied().unwrap_or(false) && !members.contains(j) && !o.id.starts_with("wire:"))
        .flat_map(|(_, o)| o.aabb.corners())
        .map(|c| c.dot(dir))
        .fold(f64::NEG_INFINITY, f64::max);
    let mine_min = members.iter().filter_map(|m| w.bodies.get(*m)).flat_map(|b| b.aabb.corners()).map(|c| c.dot(dir)).fold(f64::INFINITY, f64::min);
    for k in 0..=steps {
        let t = k as f64 * STEP;
        if mine_min + t > others_max + 0.1 {
            break;
        }
        let shift = dir * t;
        for (j, o) in w.bodies.iter().enumerate() {
            if !present.get(j).copied().unwrap_or(false) || members.contains(&j) || o.id.starts_with("wire:") {
                continue;
            }
            for &m in members {
                let Some(b) = w.bodies.get(m) else { continue };
                let moved = crate::math::Aabb { min: b.aabb.min + shift, max: b.aabb.max + shift };
                if !moved.overlaps(&o.aabb, 0.0) {
                    continue;
                }
                last_overlap_t = t;
                let allow = allowed_depth(r, b, o);
                if allow.is_infinite() || allow >= 1.0e3 {
                    continue;
                }
                if measure::surfaces_penetrate(w, m, &pts.get(w, m), j, &pts.get(w, j), shift, allow.max(0.05)) {
                    return Err((j, t, b.aabb.center() + shift));
                }
            }
        }
    }
    Ok(last_overlap_t + 3.0)
}

fn candidate_dirs(ctx: &Ctx, i: usize) -> Vec<V3> {
    let (Some(b), r) = (ctx.world.bodies.get(i), ctx.r) else { return Vec::new() };
    let axes = [v3(0.0, 0.0, 1.0), v3(0.0, 0.0, -1.0), v3(1.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, -1.0, 0.0)];
    if b.fastener {
        return r.spec.fasteners.iter().find(|f| f.id == b.id).map(|f| vec![V3::from_arr(f.dir).norm() * -1.0]).unwrap_or_default();
    }
    let mut d = axes.to_vec();
    for col in [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)] {
        let a = b.xf.r.mul_v(col);
        if axes.iter().all(|x| x.dot(a).abs() < 0.99) {
            d.push(a);
            d.push(a * -1.0);
        }
    }
    d
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let n = w.bodies.len();
    let mut present: Vec<bool> = w.bodies.iter().map(|b| !b.id.starts_with("wire:")).collect();
    let root = r.spec.parts.iter().find(|p| p.kind == PartKind::Structure).and_then(|p| w.index(&p.id));
    let surf = Surfaces::new(n);
    let mut steps: Vec<AssemblyStep> = Vec::new();
    loop {
        let mut progress = false;
        // Screws first, then everything else, so a bolt never blocks the part it holds.
        let mut order: Vec<usize> = (0..n).filter(|i| present.get(*i).copied().unwrap_or(false) && Some(*i) != root).collect();
        order.sort_by_key(|i| if w.bodies.get(*i).is_some_and(|b| b.fastener) { 0 } else { 1 });
        for i in order {
            let Some(b) = w.bodies.get(i) else { continue };
            if !present.get(i).copied().unwrap_or(false) {
                continue;
            }
            let members = if b.fastener { vec![i] } else { group(ctx, &b.id) };
            if members.iter().any(|m| Some(*m) == root) {
                continue;
            }
            let dirs = candidate_dirs(ctx, i);
            let mut best: Option<(V3, f64)> = None;
            for d in dirs {
                if let Ok(t) = slide(ctx, &surf, &members, &present, d) {
                    best = Some((d, t));
                    break;
                }
            }
            if let Some((d, t)) = best {
                for m in &members {
                    if let Some(p) = present.get_mut(*m) {
                        *p = false;
                    }
                    if let Some(mb) = w.bodies.get(*m) {
                        steps.push(AssemblyStep { order: steps.len() + 1, id: mb.id.clone(), dir: [d.x, d.y, d.z], travel_mm: t });
                    }
                }
                progress = true;
            }
        }
        let left = (0..n).filter(|i| present.get(*i).copied().unwrap_or(false) && Some(*i) != root).count();
        if left == 0 || !progress {
            break;
        }
    }
    let stuck: Vec<usize> = (0..n).filter(|i| present.get(*i).copied().unwrap_or(false) && Some(*i) != root).collect();
    for &i in &stuck {
        let Some(b) = w.bodies.get(i) else { continue };
        // Which body stops it, along the direction it got furthest in.
        let mut worst: Option<(f64, usize, V3, V3)> = None;
        for d in candidate_dirs(ctx, i) {
            let members = if b.fastener { vec![i] } else { group(ctx, &b.id) };
            if let Err((j, t, at)) = slide(ctx, &surf, &members, &present, d)
                && worst.is_none_or(|(wt, ..)| t > wt)
            {
                worst = Some((t, j, d, at));
            }
        }
        let msg = match worst {
            Some((t, j, d, _)) => format!(
                "`{}` is trapped: its best way out is along ({:.0}, {:.0}, {:.0}), which gets {t:.1} mm before it hits `{}`.",
                b.id,
                d.x,
                d.y,
                d.z,
                w.bodies.get(j).map(|o| o.id.as_str()).unwrap_or("?")
            ),
            None => format!("`{}` has no way out.", b.id),
        };
        out.push(
            Finding::fail("assembly.blocked", G, Severity::Blocker, "Part cannot be assembled", msg)
                .on(&[&b.id])
                .here(worst.map(|x| x.3).unwrap_or(b.aabb.center()))
                .fix("Open the pocket on the side it enters from (a U-slot instead of a hole for a shaft, a bore open to the mating face for an insert, an open wall for a wheel), or split the housing so the part can drop in."),
        );
    }
    if stuck.is_empty() {
        let text: Vec<String> = steps.iter().rev().map(|s| s.id.clone()).collect();
        out.push(Finding::pass(
            "assembly.order",
            G,
            "Everything can be assembled",
            format!("Every part slides in along a clear straight path. Assembly order: {}.", text.join(", ")),
        ));
    }
    ctx.assembly = steps;
    out
}
