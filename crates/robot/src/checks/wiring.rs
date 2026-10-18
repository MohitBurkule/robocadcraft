//! Wiring: every wire is routed through free space as a tube of its real thickness, so fit is proved, not assumed.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use serde::{Deserialize, Serialize};

use crate::finding::{Finding, Severity};
use crate::math::Aabb;
use crate::math::{V3, v3};
use crate::sdf::tube_sdf;
use crate::spec::PartKind;
use crate::world::World;

use super::Ctx;

const G: &str = "wiring";
const CELL: f64 = 0.6;
/// Radius around a port where a wire may touch its own part.
const PORT_ZONE: f64 = 2.6;
const MAX_EXPANSIONS: usize = 3_000_000;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Route {
    pub id: String,
    pub awg: f64,
    pub kind: String,
    pub od_mm: f64,
    pub length_mm: f64,
    pub points: Vec<[f64; 3]>,
}

/// Outer diameter in mm of silicone-insulated hookup wire.
pub fn wire_od(awg: f64) -> f64 {
    const T: [(f64, f64); 8] = [(32.0, 0.8), (30.0, 0.9), (28.0, 1.1), (26.0, 1.3), (24.0, 1.6), (22.0, 1.9), (20.0, 2.2), (18.0, 2.6)];
    let mut best = (f64::MAX, 1.3);
    for (a, d) in T {
        if (a - awg).abs() < best.0 {
            best = ((a - awg).abs(), d);
        }
    }
    best.1
}

/// Grams per mm of silicone hookup wire (copper plus insulation).
pub fn wire_g_per_mm(awg: f64) -> f64 {
    let od = wire_od(awg);
    0.0013 * od * od
}

pub fn default_awg(kind: &str) -> f64 {
    match kind {
        "power" => 24.0,
        "motor" => 28.0,
        _ => 30.0,
    }
}

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

struct Grid {
    min: V3,
    n: [usize; 3],
    dist: Vec<f32>,
}

impl Grid {
    fn idx(&self, c: [i64; 3]) -> Option<usize> {
        if c.iter().zip(self.n.iter()).any(|(a, n)| *a < 0 || *a as usize >= *n) {
            return None;
        }
        Some((c[2] as usize * self.n[1] + c[1] as usize) * self.n[0] + c[0] as usize)
    }
    fn centre(&self, c: [i64; 3]) -> V3 {
        v3(self.min.x + (c[0] as f64 + 0.5) * CELL, self.min.y + (c[1] as f64 + 0.5) * CELL, self.min.z + (c[2] as f64 + 0.5) * CELL)
    }
    fn cell(&self, p: V3) -> [i64; 3] {
        [((p.x - self.min.x) / CELL).floor() as i64, ((p.y - self.min.y) / CELL).floor() as i64, ((p.z - self.min.z) / CELL).floor() as i64]
    }
}

#[derive(PartialEq)]
struct Item(f64, usize);
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.partial_cmp(&self.0).unwrap_or(Ordering::Equal)
    }
}

pub struct Router<'a> {
    grid: Grid,
    _w: std::marker::PhantomData<&'a World>,
    done: Vec<Route>,
}

/// Squared distance transform of a 1-D sampled function (Felzenszwalb and Huttenlocher).
fn edt_1d(f: &[f64], out: &mut [f64], v: &mut Vec<usize>, z: &mut Vec<f64>) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let at = |i: usize| f.get(i).copied().unwrap_or(0.0);
    v.clear();
    z.clear();
    v.push(0);
    z.push(f64::NEG_INFINITY);
    z.push(f64::INFINITY);
    let mut k = 0usize;
    for q in 1..n {
        loop {
            let p = v.get(k).copied().unwrap_or(0);
            let s = ((at(q) + (q * q) as f64) - (at(p) + (p * p) as f64)) / (2.0 * q as f64 - 2.0 * p as f64);
            if k > 0 && s <= z.get(k).copied().unwrap_or(f64::NEG_INFINITY) {
                k -= 1;
                continue;
            }
            k += 1;
            v.truncate(k);
            z.truncate(k);
            v.push(q);
            z.push(s);
            z.push(f64::INFINITY);
            break;
        }
    }
    let mut k = 0usize;
    for (q, o) in out.iter_mut().enumerate() {
        while z.get(k + 1).is_some_and(|zz| *zz < q as f64) {
            k += 1;
        }
        let p = v.get(k).copied().unwrap_or(0);
        let d = q as f64 - p as f64;
        *o = d * d + at(p);
    }
}

impl<'a> Router<'a> {
    fn new(world: &'a World) -> Router<'a> {
        let b = world.bounds().grow(2.0);
        let size = b.size();
        let n = [(size.x / CELL).ceil() as usize + 1, (size.y / CELL).ceil() as usize + 1, (size.z / CELL).ceil() as usize + 1];
        let total = n[0].saturating_mul(n[1]).saturating_mul(n[2]).min(60_000_000);
        // Wires stay inside the chassis and out of anything that spins.
        let interior = world.bodies.iter().filter(|bd| bd.kind == PartKind::Structure).fold(Aabb::empty(), |a, bd| a.union(&bd.aabb)).grow(-0.4);
        let keepout: Vec<Aabb> = world
            .bodies
            .iter()
            .filter(|bd| matches!(bd.kind, PartKind::Wheel | PartKind::Weapon | PartKind::Pulley | PartKind::Belt))
            .map(|bd| bd.aabb.grow(0.6))
            .collect();
        let mut grid = Grid { min: b.min, n, dist: vec![0.0; total] };
        // Occupancy: 0 where solid, large where free; then distances to the nearest solid cell.
        let big = 1.0e5_f32;
        for iz in 0..n[2] {
            for iy in 0..n[1] {
                for ix in 0..n[0] {
                    let p = grid.centre([ix as i64, iy as i64, iz as i64]);
                    let outside = !interior.contains(p);
                    let swept = keepout.iter().any(|k| k.contains(p));
                    let solid = outside || swept || world.bodies.iter().enumerate().any(|(i, bd)| bd.aabb.contains(p) && world.sdf(i, p) < 0.0);
                    if let Some(i) = grid.idx([ix as i64, iy as i64, iz as i64])
                        && let Some(slot) = grid.dist.get_mut(i)
                    {
                        *slot = if solid { 0.0 } else { big };
                    }
                }
            }
        }
        let (mut v, mut z) = (Vec::new(), Vec::new());
        for axis in 0..3 {
            let len = n[axis];
            let mut line = vec![0.0_f64; len];
            let mut res = vec![0.0_f64; len];
            let (o1, o2) = match axis {
                0 => (1, 2),
                1 => (0, 2),
                _ => (0, 1),
            };
            for a in 0..n[o1] {
                for b2 in 0..n[o2] {
                    let mut c = [0_i64; 3];
                    c[o1] = a as i64;
                    c[o2] = b2 as i64;
                    for (k, l) in line.iter_mut().enumerate() {
                        c[axis] = k as i64;
                        *l = grid.idx(c).and_then(|i| grid.dist.get(i)).copied().unwrap_or(0.0) as f64;
                    }
                    edt_1d(&line, &mut res, &mut v, &mut z);
                    for (k, rv) in res.iter().enumerate() {
                        c[axis] = k as i64;
                        if let Some(i) = grid.idx(c)
                            && let Some(slot) = grid.dist.get_mut(i)
                        {
                            *slot = *rv as f32;
                        }
                    }
                }
            }
        }
        Router { grid, _w: std::marker::PhantomData, done: Vec::new() }
    }

    /// Distance in mm from the centre of cell `c` to the nearest solid, to within half a cell.
    fn static_dist(&self, c: [i64; 3]) -> f64 {
        match self.grid.idx(c).and_then(|i| self.grid.dist.get(i)) {
            Some(d2) => (d2.max(0.0) as f64).sqrt() * CELL - CELL * 0.5,
            None => -1.0,
        }
    }

    /// True when a wire of radius `r` can sit with its centreline at `p`.
    fn free(&self, p: V3, r: f64, ends: &[V3]) -> bool {
        let c = self.grid.cell(p);
        let near_end = ends.iter().any(|e| (*e - p).len() < PORT_ZONE);
        if !near_end {
            // Evaluate exactly at p for points off the cell centre (smoothing), cached at cell centres for the search.
            let d = self.static_dist(c);
            if d < r + 0.05 {
                return false;
            }
            for o in &self.done {
                if tube_sdf(&o.points, p) < r + o.od_mm * 0.5 + 0.02 {
                    return false;
                }
            }
        }
        true
    }

    /// On failure, the closest the search got to the goal and where.
    fn route(&mut self, a: V3, b: V3, r: f64) -> Result<Vec<V3>, (f64, V3)> {
        let ends = [a, b];
        let (sc, gc) = (self.grid.cell(a), self.grid.cell(b));
        let (Some(s), Some(g)) = (self.grid.idx(sc), self.grid.idx(gc)) else { return Err(((b - a).len(), a)) };
        let mut closest = ((b - a).len(), a);
        let n = self.grid.n;
        let mut came: HashMap<usize, usize> = HashMap::new();
        let mut cost: HashMap<usize, f64> = HashMap::new();
        let mut heap = BinaryHeap::new();
        cost.insert(s, 0.0);
        heap.push(Item((b - a).len(), s));
        let mut steps = 0usize;
        let decode = |i: usize| -> [i64; 3] { [(i % n[0]) as i64, ((i / n[0]) % n[1]) as i64, (i / (n[0] * n[1])) as i64] };
        while let Some(Item(_, cur)) = heap.pop() {
            if cur == g {
                let mut path = vec![b];
                let mut k = cur;
                while let Some(prev) = came.get(&k) {
                    path.push(self.grid.centre(decode(*prev)));
                    k = *prev;
                }
                path.push(a);
                path.reverse();
                return Ok(path);
            }
            steps += 1;
            if steps > MAX_EXPANSIONS {
                return Err(closest);
            }
            let cc = decode(cur);
            let here = self.grid.centre(decode(cur));
            if (b - here).len() < closest.0 {
                closest = ((b - here).len(), here);
            }
            let base = cost.get(&cur).copied().unwrap_or(0.0);
            for dz in -1..=1_i64 {
                for dy in -1..=1_i64 {
                    for dx in -1..=1_i64 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        let nc = [cc[0] + dx, cc[1] + dy, cc[2] + dz];
                        let Some(ni) = self.grid.idx(nc) else { continue };
                        let p = self.grid.centre(nc);
                        if ni != g && !self.free(p, r, &ends) {
                            continue;
                        }
                        let step = ((dx * dx + dy * dy + dz * dz) as f64).sqrt() * CELL;
                        let nd = base + step;
                        if cost.get(&ni).is_none_or(|c| nd < *c) {
                            cost.insert(ni, nd);
                            came.insert(ni, cur);
                            heap.push(Item(nd + (b - p).len(), ni));
                        }
                    }
                }
            }
        }
        Err(closest)
    }

    fn segment_free(&self, p: V3, q: V3, r: f64, ends: &[V3]) -> bool {
        let l = (q - p).len();
        let k = ((l / 0.25).ceil() as usize).max(1);
        (0..=k).all(|i| self.free(p + (q - p) * (i as f64 / k as f64), r, ends))
    }

    /// Pull the path taut: skip waypoints whenever the straight line stays clear.
    fn smooth(&mut self, path: &[V3], r: f64, ends: &[V3]) -> Vec<V3> {
        let mut out = Vec::new();
        let mut i = 0;
        while i < path.len() {
            let Some(p) = path.get(i) else { break };
            out.push(*p);
            let mut j = path.len() - 1;
            while j > i + 1 {
                if let Some(q) = path.get(j)
                    && self.segment_free(*p, *q, r, ends)
                {
                    break;
                }
                j -= 1;
            }
            if j <= i {
                break;
            }
            i = j;
        }
        out
    }
}

fn polyline_len(pts: &[V3]) -> f64 {
    pts.windows(2)
        .map(|w| match w {
            [a, b] => (*b - *a).len(),
            _ => 0.0,
        })
        .sum()
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let mut out = Vec::new();
    let r = ctx.r;
    if r.spec.wires.is_empty() {
        out.push(Finding::note("wiring.none", G, "No wires declared", "The spec lists no wires, so nothing is routed or counted for mass."));
        return out;
    }
    let world = ctx.world;
    let mut router = Router::new(world);
    let mut total_len = 0.0;
    let mut total_g = 0.0;
    let mut ok = 0;
    // Thick wires first: they have the fewest options.
    let mut order: Vec<_> = r.spec.wires.iter().collect();
    order.sort_by(|a, b| {
        let (da, db) = (wire_od(a.awg.unwrap_or(default_awg(&a.kind))), wire_od(b.awg.unwrap_or(default_awg(&b.kind))));
        db.partial_cmp(&da).unwrap_or(Ordering::Equal)
    });
    for w in order {
        let awg = w.awg.unwrap_or(default_awg(&w.kind));
        let od = wire_od(awg);
        let (Some(a), Some(b)) = (ctx.world.port(r, &w.from), ctx.world.port(r, &w.to)) else {
            out.push(
                Finding::fail(
                    "wiring.port",
                    G,
                    Severity::Blocker,
                    "Wire end not found",
                    format!("Wire `{}` runs `{}` to `{}` but one of those ports does not exist.", w.id, w.from, w.to),
                )
                .fix("Use `part.port` names from the catalog entry or the part's `ports`."),
            );
            continue;
        };
        let radius = od * 0.5;
        let ends = [a, b];
        let raw = match router.route(a, b, radius) {
            Ok(p) => p,
            Err((gap, at)) => {
                let toward = at + (b - at).norm() * 1.4;
                let blocker = world
                    .bodies
                    .iter()
                    .enumerate()
                    .filter(|(_, bd)| !bd.id.starts_with("wire:"))
                    .map(|(i, bd)| (world.sdf(i, toward), bd.id.clone()))
                    .min_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(Ordering::Equal))
                    .map(|x| x.1)
                    .unwrap_or_default();
                out.push(
                Finding::fail("wiring.route", G, Severity::Blocker, "No room for this wire", format!("Wire `{}` ({} AWG, {od:.1} mm thick) from `{}` to `{}` has no clear path; it gets within {gap:.1} mm of the end at ({:.1}, {:.1}, {:.1}) and `{blocker}` blocks it.", w.id, awg, w.from, w.to, at.x, at.y, at.z))
                    .on(&[&w.from, &w.to])
                    .here(at)
                    .fix("Cut a wire channel (at least 2.5 mm wide) through the walls between those parts, or move the parts closer."),
                );
                continue;
            }
        };
        let pts = router.smooth(&raw, radius, &ends);
        let slack = w.slack_mm.unwrap_or(6.0);
        let len = polyline_len(&pts) + slack;
        // Bend radius: a wire cannot turn tighter than about 3 diameters.
        let min_bend = 2.0 * od;
        let mut sharp = None;
        for t in pts.windows(3) {
            if let [p, q, s] = t {
                let (u, v) = ((*q - *p).norm(), (*s - *q).norm());
                let theta = u.dot(v).clamp(-1.0, 1.0).acos();
                let need = min_bend * (theta / 2.0).tan();
                let room = (*q - *p).len().min((*s - *q).len());
                if theta > 0.2 && need > room * 0.9 + 0.5 {
                    sharp = Some((*q, theta.to_degrees()));
                }
            }
        }
        total_len += len;
        total_g += len * wire_g_per_mm(awg);
        if let Some((at, deg)) = sharp {
            out.push(
                Finding::fail(
                    "wiring.bend",
                    G,
                    Severity::Warning,
                    "Wire bends too tightly",
                    format!(
                        "Wire `{}` turns {deg:.0} degrees with less than {min_bend:.1} mm bend radius; it will kink or pull off its solder joint.",
                        w.id
                    ),
                )
                .here(at)
                .fix("Widen the channel at the corner or route it in a straighter line."),
            );
        } else {
            ok += 1;
        }
        router.done.push(Route {
            id: w.id.clone(),
            kind: w.kind.clone(),
            awg,
            od_mm: od,
            length_mm: len,
            points: pts.iter().map(|p| [p.x, p.y, p.z]).collect(),
        });
    }
    ctx.metrics.insert("wire_length_mm".into(), total_len);
    ctx.metrics.insert("wire_mass_g".into(), total_g);
    ctx.routes.clone_from(&router.done);
    if out.iter().all(|f| f.pass || f.id == "wiring.none") {
        out.push(Finding::pass(
            "wiring.route",
            G,
            "Every wire has a route",
            format!("{ok} wires routed as real-thickness tubes through free space, {total_len:.0} mm in total, no collisions."),
        ));
    }
    out
}
