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

/// Deepest penetration (mm) between bodies `a` and `b` and where, sampled on a grid of step `h`
/// inside the intersection of their boxes; `None` when the boxes do not overlap.
pub fn overlap_depth(world: &World, a: usize, b: usize, h: f64) -> Option<(f64, V3, f64)> {
    let (ba, bb) = (world.bodies.get(a)?, world.bodies.get(b)?);
    let ib = ba.aabb.intersect(&bb.aabb);
    if ib.is_empty() {
        return None;
    }
    let ib = ib.grow(h * 0.5);
    let h = step_for(&ib, h);
    let (mut depth, mut at, mut vol) = (0.0_f64, ib.center(), 0.0);
    let nx = ((ib.max.x - ib.min.x) / h).ceil() as i64;
    let ny = ((ib.max.y - ib.min.y) / h).ceil() as i64;
    let nz = ((ib.max.z - ib.min.z) / h).ceil() as i64;
    for iz in 0..nz.max(1) {
        for iy in 0..ny.max(1) {
            for ix in 0..nx.max(1) {
                let p = v3(ib.min.x + (ix as f64 + 0.5) * h, ib.min.y + (iy as f64 + 0.5) * h, ib.min.z + (iz as f64 + 0.5) * h);
                let da = world.sdf(a, p);
                if da >= h {
                    continue;
                }
                let m = da.max(world.sdf(b, p));
                if m < 0.0 {
                    vol += coverage(m, h).min(coverage(da, h)) * h * h * h;
                    if -m > depth {
                        depth = -m;
                        at = p;
                    }
                }
            }
        }
    }
    Some((depth, at, vol))
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
