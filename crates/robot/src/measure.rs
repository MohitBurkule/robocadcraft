//! Measurements by sampling the signed distance fields.

use crate::math::{Aabb, V3, v3};
use crate::sdf::coverage;
use crate::world::World;

pub const MAX_SAMPLES: f64 = 6.0e6;

#[derive(Clone, Copy, Debug, Default)]
pub struct Moments {
    pub volume: f64,
    pub centroid: V3,
}

/// Step size that keeps a box of `b` under the sample cap, but never coarser than `want`.
pub fn step_for(b: &Aabb, want: f64) -> f64 {
    let s = b.size();
    let n = (s.x / want + 1.0) * (s.y / want + 1.0) * (s.z / want + 1.0);
    if n <= MAX_SAMPLES { want } else { want * (n / MAX_SAMPLES).cbrt() }
}

/// Volume (mm3) and centroid of body `i`, sampled with sub-voxel coverage.
pub fn moments(world: &World, i: usize, want_step: f64) -> Moments {
    let Some(b) = world.bodies.get(i) else { return Moments::default() };
    let bb = b.aabb.grow(0.2);
    let h = step_for(&bb, want_step);
    let (mut vol, mut c) = (0.0, V3::ZERO);
    let nx = ((bb.max.x - bb.min.x) / h).ceil() as i64;
    let ny = ((bb.max.y - bb.min.y) / h).ceil() as i64;
    let nz = ((bb.max.z - bb.min.z) / h).ceil() as i64;
    let vh = h * h * h;
    for iz in 0..nz {
        let z = bb.min.z + (iz as f64 + 0.5) * h;
        for iy in 0..ny {
            let y = bb.min.y + (iy as f64 + 0.5) * h;
            for ix in 0..nx {
                let p = v3(bb.min.x + (ix as f64 + 0.5) * h, y, z);
                let d = world.sdf(i, p);
                if d < h {
                    let w = coverage(d, h) * vh;
                    vol += w;
                    c = c + p * w;
                }
            }
        }
    }
    Moments { volume: vol, centroid: if vol > 1e-9 { c * (1.0 / vol) } else { b.aabb.center() } }
}

/// Moment of inertia (mm5 per unit density, i.e. multiply by density in g/mm3 for g mm2) about a
/// world axis through `origin` along unit `axis`.
pub fn axial_inertia(world: &World, i: usize, origin: V3, axis: V3, want_step: f64) -> f64 {
    let Some(b) = world.bodies.get(i) else { return 0.0 };
    let bb = b.aabb.grow(0.2);
    let h = step_for(&bb, want_step);
    let a = axis.norm();
    let mut sum = 0.0;
    let nx = ((bb.max.x - bb.min.x) / h).ceil() as i64;
    let ny = ((bb.max.y - bb.min.y) / h).ceil() as i64;
    let nz = ((bb.max.z - bb.min.z) / h).ceil() as i64;
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = v3(bb.min.x + (ix as f64 + 0.5) * h, bb.min.y + (iy as f64 + 0.5) * h, bb.min.z + (iz as f64 + 0.5) * h);
                let d = world.sdf(i, p);
                if d < h {
                    let q = p - origin;
                    let perp = q - a * q.dot(a);
                    sum += coverage(d, h) * h * h * h * perp.dot(perp);
                }
            }
        }
    }
    sum
}

/// Result of an overlap test between two bodies.
#[derive(Clone, Copy, Debug)]
pub struct Overlap {
    /// Deepest penetration in mm (0 when only touching or apart).
    pub depth: f64,
    pub at: V3,
    /// Approximate overlapping volume in mm3.
    pub volume: f64,
}

/// Deepest penetration between bodies `a` and `b`, with `a` moved by `shift` first. A coarse grid in
/// the box intersection finds candidate cells near contact, which are refined four times finer.
pub fn overlap_shifted(world: &World, a: usize, b: usize, shift: V3, coarse: f64) -> Option<Overlap> {
    let (ba, bb) = (world.bodies.get(a)?, world.bodies.get(b)?);
    let abox = Aabb { min: ba.aabb.min + shift, max: ba.aabb.max + shift };
    let ib = abox.intersect(&bb.aabb);
    if ib.is_empty() {
        return None;
    }
    let ib = ib.grow(coarse * 0.5);
    let c = step_for(&ib, coarse);
    let (nx, ny, nz) =
        (((ib.max.x - ib.min.x) / c).ceil() as i64, ((ib.max.y - ib.min.y) / c).ceil() as i64, ((ib.max.z - ib.min.z) / c).ceil() as i64);
    let f = |p: V3| world.sdf(a, p - shift).max(world.sdf(b, p));
    let mut cands: Vec<(f64, V3)> = Vec::new();
    let mut vol = 0.0;
    for iz in 0..nz.max(1) {
        for iy in 0..ny.max(1) {
            for ix in 0..nx.max(1) {
                let p = v3(ib.min.x + (ix as f64 + 0.5) * c, ib.min.y + (iy as f64 + 0.5) * c, ib.min.z + (iz as f64 + 0.5) * c);
                let da = world.sdf(a, p - shift);
                if da >= c {
                    continue;
                }
                let m = da.max(world.sdf(b, p));
                if m < c {
                    cands.push((m, p));
                    if m < 0.0 {
                        vol += coverage(m, c).min(coverage(da, c)) * c * c * c;
                    }
                }
            }
        }
    }
    cands.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    cands.truncate(3000);
    let (mut depth, mut at) = (0.0_f64, ib.center());
    let q = c / 4.0;
    for (m, p) in &cands {
        if *m < 0.0 && -m > depth {
            depth = -m;
            at = *p;
        }
        for dz in -2..=2 {
            for dy in -2..=2 {
                for dx in -2..=2 {
                    let pp = *p + v3(dx as f64 * q, dy as f64 * q, dz as f64 * q);
                    let mm = f(pp);
                    if -mm > depth {
                        depth = -mm;
                        at = pp;
                    }
                }
            }
        }
    }
    Some(Overlap { depth, at, volume: vol })
}

/// True when `a` shifted by `shift` penetrates `b` deeper than `thresh` mm. Cheaper than `overlap_shifted`: it stops at the
/// first proof and refines only the few most promising cells.
pub fn overlap_exceeds(world: &World, a: usize, b: usize, shift: V3, coarse: f64, thresh: f64) -> bool {
    let (Some(ba), Some(bb)) = (world.bodies.get(a), world.bodies.get(b)) else { return false };
    let abox = Aabb { min: ba.aabb.min + shift, max: ba.aabb.max + shift };
    let ib = abox.intersect(&bb.aabb);
    if ib.is_empty() {
        return false;
    }
    let ib = ib.grow(coarse * 0.5);
    let c = step_for(&ib, coarse);
    let (nx, ny, nz) =
        (((ib.max.x - ib.min.x) / c).ceil() as i64, ((ib.max.y - ib.min.y) / c).ceil() as i64, ((ib.max.z - ib.min.z) / c).ceil() as i64);
    let mut cands: Vec<(f64, V3)> = Vec::new();
    for iz in 0..nz.max(1) {
        for iy in 0..ny.max(1) {
            for ix in 0..nx.max(1) {
                let p = v3(ib.min.x + (ix as f64 + 0.5) * c, ib.min.y + (iy as f64 + 0.5) * c, ib.min.z + (iz as f64 + 0.5) * c);
                let da = world.sdf(a, p - shift);
                if da >= c {
                    continue;
                }
                let m = da.max(world.sdf(b, p));
                if -m > thresh {
                    return true;
                }
                if m < c {
                    cands.push((m, p));
                }
            }
        }
    }
    cands.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
    cands.truncate(500);
    let q = c / 4.0;
    for (_, p) in &cands {
        for dz in -2..=2 {
            for dy in -2..=2 {
                for dx in -2..=2 {
                    let pp = *p + v3(dx as f64 * q, dy as f64 * q, dz as f64 * q);
                    if -(world.sdf(a, pp - shift).max(world.sdf(b, pp))) > thresh {
                        return true;
                    }
                }
            }
        }
    }
    false
}

pub fn overlap(world: &World, a: usize, b: usize, coarse: f64) -> Option<Overlap> {
    overlap_shifted(world, a, b, V3::ZERO, coarse)
}

/// Lowest point of body `i` (world), by sphere tracing up each column of an xy grid.
pub fn lowest_point(world: &World, i: usize, step: f64) -> Option<V3> {
    let b = world.bodies.get(i)?;
    let bb = b.aabb;
    let (nx, ny) = (((bb.max.x - bb.min.x) / step).ceil() as i64 + 1, ((bb.max.y - bb.min.y) / step).ceil() as i64 + 1);
    let mut best: Option<V3> = None;
    for iy in 0..ny {
        for ix in 0..nx {
            let (x, y) = (bb.min.x + ix as f64 * step, bb.min.y + iy as f64 * step);
            let mut z = bb.min.z;
            while z <= bb.max.z + 1e-9 {
                if best.is_some_and(|p| z >= p.z) {
                    break;
                }
                let d = world.sdf(i, v3(x, y, z));
                if d < 0.01 {
                    best = Some(v3(x, y, z));
                    break;
                }
                z += d.max(0.04);
            }
        }
    }
    best
}

/// Highest point of body `i`.
pub fn highest_point(world: &World, i: usize, step: f64) -> Option<V3> {
    let b = world.bodies.get(i)?;
    let bb = b.aabb;
    let (nx, ny) = (((bb.max.x - bb.min.x) / step).ceil() as i64 + 1, ((bb.max.y - bb.min.y) / step).ceil() as i64 + 1);
    let mut best: Option<V3> = None;
    for iy in 0..ny {
        for ix in 0..nx {
            let (x, y) = (bb.min.x + ix as f64 * step, bb.min.y + iy as f64 * step);
            let mut z = bb.max.z;
            while z >= bb.min.z - 1e-9 {
                if best.is_some_and(|p| z <= p.z) {
                    break;
                }
                let d = world.sdf(i, v3(x, y, z));
                if d < 0.01 {
                    best = Some(v3(x, y, z));
                    break;
                }
                z -= d.max(0.04);
            }
        }
    }
    best
}

/// Smallest distance (mm) from the surface of `a` to `b` (negative when they interpenetrate),
/// from points within a voxel of `a`'s surface sampled at step `h`.
pub fn min_gap(world: &World, a: usize, b: usize, h: f64) -> f64 {
    let Some(ba) = world.bodies.get(a) else { return f64::INFINITY };
    let bb = ba.aabb.grow(0.1);
    let h = step_for(&bb, h);
    let mut best = f64::INFINITY;
    let nx = ((bb.max.x - bb.min.x) / h).ceil() as i64;
    let ny = ((bb.max.y - bb.min.y) / h).ceil() as i64;
    let nz = ((bb.max.z - bb.min.z) / h).ceil() as i64;
    for iz in 0..nz.max(1) {
        for iy in 0..ny.max(1) {
            for ix in 0..nx.max(1) {
                let p = v3(bb.min.x + (ix as f64 + 0.5) * h, bb.min.y + (iy as f64 + 0.5) * h, bb.min.z + (iz as f64 + 0.5) * h);
                if world.sdf(a, p).abs() <= h * 0.75 {
                    best = best.min(world.sdf(b, p));
                }
            }
        }
    }
    best
}

/// World-space points on body `i`'s surface: wherever the sign of the distance changes between neighbouring samples of a grid with
/// spacing `h`, the crossing is bisected. Only the sign of the distance is trusted, so subtractions do not confuse it.
pub fn surface_points(world: &World, i: usize, h: f64) -> Vec<V3> {
    let Some(b) = world.bodies.get(i) else { return Vec::new() };
    let bb = b.aabb.grow(h);
    let n = [
        ((bb.max.x - bb.min.x) / h).ceil() as usize + 1,
        ((bb.max.y - bb.min.y) / h).ceil() as usize + 1,
        ((bb.max.z - bb.min.z) / h).ceil() as usize + 1,
    ];
    if n[0].saturating_mul(n[1]).saturating_mul(n[2]) > 30_000_000 {
        return Vec::new();
    }
    let at = |x: usize, y: usize, z: usize| (z * n[1] + y) * n[0] + x;
    let pos = |x: usize, y: usize, z: usize| v3(bb.min.x + x as f64 * h, bb.min.y + y as f64 * h, bb.min.z + z as f64 * h);
    let mut inside = vec![false; n[0] * n[1] * n[2]];
    for z in 0..n[2] {
        for y in 0..n[1] {
            for x in 0..n[0] {
                if let Some(s) = inside.get_mut(at(x, y, z)) {
                    *s = world.sdf(i, pos(x, y, z)) < 0.0;
                }
            }
        }
    }
    let mut pts = Vec::new();
    for z in 0..n[2] {
        for y in 0..n[1] {
            for x in 0..n[0] {
                let here = inside.get(at(x, y, z)).copied().unwrap_or(false);
                for (dx, dy, dz) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
                    let (nx, ny, nz) = (x + dx, y + dy, z + dz);
                    if nx >= n[0] || ny >= n[1] || nz >= n[2] || inside.get(at(nx, ny, nz)).copied().unwrap_or(here) == here {
                        continue;
                    }
                    let (mut lo, mut hi) = (pos(x, y, z), pos(nx, ny, nz));
                    for _ in 0..6 {
                        let mid = (lo + hi) * 0.5;
                        if (world.sdf(i, mid) < 0.0) == here {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    // Put the sample on the solid side so that touching bodies are not counted as penetrating.
                    pts.push(if here { lo } else { hi });
                }
            }
        }
    }
    pts
}

/// Fast collision test from surface samples: true when a surface point of `a` (moved by `shift`) lies deeper than `thresh` inside
/// `b`, or a surface point of `b` lies deeper than `thresh` inside moved `a`.
pub fn surfaces_penetrate(world: &World, a: usize, a_pts: &[V3], b: usize, b_pts: &[V3], shift: V3, thresh: f64) -> bool {
    let (Some(ba), Some(bb)) = (world.bodies.get(a), world.bodies.get(b)) else { return false };
    let abox = Aabb { min: ba.aabb.min + shift, max: ba.aabb.max + shift };
    let zone = abox.intersect(&bb.aabb).grow(0.01);
    if zone.is_empty() {
        return false;
    }
    a_pts.iter().any(|p| zone.contains(*p + shift) && world.sdf(b, *p + shift) < -thresh)
        || b_pts.iter().any(|p| zone.contains(*p) && world.sdf(a, *p - shift) < -thresh)
}
