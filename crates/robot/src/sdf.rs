//! Solids as signed distance functions (negative inside), with CSG.
//!
//! Distances are exact for primitives and for unions, and a conservative bound for
//! subtractions and intersections; the sign is always right. That is enough for overlap
//! depth, clearance, ray casts and volume by sampling.

use serde::{Deserialize, Serialize};

use crate::math::{Aabb, Mat3, V3, Xform, v3};

pub const MAX_DEPTH: usize = 24;
pub const MAX_NODES: usize = 4000;

fn arr3(a: [f64; 3]) -> V3 {
    V3::from_arr(a)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Shape {
    /// Box centred on the origin.
    Box {
        size: [f64; 3],
    },
    /// Cylinder along Z, centred on the origin.
    Cylinder {
        radius: f64,
        height: f64,
    },
    Sphere {
        radius: f64,
    },
    /// Polygon in XY extruded along Z (centred), counter-clockwise or clockwise.
    Prism {
        points: Vec<[f64; 2]>,
        height: f64,
    },
    Union {
        items: Vec<Shape>,
    },
    Intersect {
        items: Vec<Shape>,
    },
    Subtract {
        base: Box<Shape>,
        cut: Vec<Shape>,
    },
    /// Move and rotate a child (rotation is Euler degrees, `Rz*Ry*Rx`).
    Place {
        shape: Box<Shape>,
        #[serde(default)]
        at: [f64; 3],
        #[serde(default)]
        rot: [f64; 3],
    },
    /// Grow (positive) or shrink (negative) a child by `d` mm.
    Offset {
        shape: Box<Shape>,
        d: f64,
    },
}

impl Shape {
    pub fn boxed(x: f64, y: f64, z: f64) -> Shape {
        Shape::Box { size: [x, y, z] }
    }
    pub fn cyl(radius: f64, height: f64) -> Shape {
        Shape::Cylinder { radius, height }
    }
    /// Cylinder with its axis along +X.
    pub fn cyl_x(radius: f64, length: f64) -> Shape {
        Shape::cyl(radius, length).rot([0.0, 90.0, 0.0])
    }
    /// Cylinder with its axis along +Y.
    pub fn cyl_y(radius: f64, length: f64) -> Shape {
        Shape::cyl(radius, length).rot([-90.0, 0.0, 0.0])
    }
    pub fn at(self, x: f64, y: f64, z: f64) -> Shape {
        Shape::Place { shape: Box::new(self), at: [x, y, z], rot: [0.0; 3] }
    }
    pub fn rot(self, rot: [f64; 3]) -> Shape {
        Shape::Place { shape: Box::new(self), at: [0.0; 3], rot }
    }
    pub fn place(self, at: [f64; 3], rot: [f64; 3]) -> Shape {
        Shape::Place { shape: Box::new(self), at, rot }
    }
    pub fn minus(self, cut: Vec<Shape>) -> Shape {
        if cut.is_empty() { self } else { Shape::Subtract { base: Box::new(self), cut } }
    }
    pub fn union(items: Vec<Shape>) -> Shape {
        Shape::Union { items }
    }
    pub fn grow(self, d: f64) -> Shape {
        Shape::Offset { shape: Box::new(self), d }
    }
    /// Mirror across the plane `x = 0` (`axis` 0), `y = 0` (1) or `z = 0` (2).
    pub fn mirrored(&self, axis: usize) -> Shape {
        mirror_shape(self, axis)
    }

    /// Count nodes and depth; `Err` for hostile or degenerate input.
    pub fn validate(&self) -> Result<(), String> {
        let mut n = 0usize;
        self.check(0, &mut n)
    }

    fn check(&self, depth: usize, n: &mut usize) -> Result<(), String> {
        *n += 1;
        if depth > MAX_DEPTH {
            return Err(format!("shape nesting deeper than {MAX_DEPTH}"));
        }
        if *n > MAX_NODES {
            return Err(format!("shape has more than {MAX_NODES} nodes"));
        }
        let pos =
            |name: &str, v: f64| if v.is_finite() && v > 0.0 { Ok(()) } else { Err(format!("{name} must be a positive finite number, got {v}")) };
        match self {
            Shape::Box { size } => {
                for s in size {
                    pos("box size", *s)?;
                }
            }
            Shape::Cylinder { radius, height } => {
                pos("cylinder radius", *radius)?;
                pos("cylinder height", *height)?;
            }
            Shape::Sphere { radius } => pos("sphere radius", *radius)?,
            Shape::Prism { points, height } => {
                pos("prism height", *height)?;
                if points.len() < 3 {
                    return Err("prism needs at least 3 points".into());
                }
                if points.len() > 2000 {
                    return Err("prism has more than 2000 points".into());
                }
                if points.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
                    return Err("prism point is not finite".into());
                }
            }
            Shape::Union { items } | Shape::Intersect { items } => {
                if items.is_empty() {
                    return Err("union/intersect needs at least one item".into());
                }
                for i in items {
                    i.check(depth + 1, n)?;
                }
            }
            Shape::Subtract { base, cut } => {
                base.check(depth + 1, n)?;
                for c in cut {
                    c.check(depth + 1, n)?;
                }
            }
            Shape::Place { shape, at, rot } => {
                if at.iter().chain(rot.iter()).any(|v| !v.is_finite()) {
                    return Err("place has a non-finite value".into());
                }
                shape.check(depth + 1, n)?;
            }
            Shape::Offset { shape, d } => {
                if !d.is_finite() || d.abs() > 50.0 {
                    return Err("offset must be finite and within +-50 mm".into());
                }
                shape.check(depth + 1, n)?;
            }
        }
        Ok(())
    }

    /// Signed distance from `p` (local coordinates) to the surface; negative inside.
    pub fn sdf(&self, p: V3) -> f64 {
        match self {
            Shape::Box { size } => {
                let q = v3(p.x.abs() - size[0] * 0.5, p.y.abs() - size[1] * 0.5, p.z.abs() - size[2] * 0.5);
                let out = v3(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).len();
                out + q.x.max(q.y).max(q.z).min(0.0)
            }
            Shape::Cylinder { radius, height } => {
                let d = (p.x * p.x + p.y * p.y).sqrt() - radius;
                let h = p.z.abs() - height * 0.5;
                combine2(d, h)
            }
            Shape::Sphere { radius } => p.len() - radius,
            Shape::Prism { points, height } => {
                let d = poly_sdf(points, p.x, p.y);
                let h = p.z.abs() - height * 0.5;
                combine2(d, h)
            }
            Shape::Union { items } => items.iter().map(|s| s.sdf(p)).fold(f64::INFINITY, f64::min),
            Shape::Intersect { items } => items.iter().map(|s| s.sdf(p)).fold(f64::NEG_INFINITY, f64::max),
            Shape::Subtract { base, cut } => {
                let mut d = base.sdf(p);
                for c in cut {
                    if d < -1e6 {
                        break;
                    }
                    d = d.max(-c.sdf(p));
                }
                d
            }
            Shape::Place { shape, at, rot } => {
                let x = Xform::new(arr3(*at), arr3(*rot));
                shape.sdf(x.inv_apply(p))
            }
            Shape::Offset { shape, d } => shape.sdf(p) - d,
        }
    }

    /// Conservative axis-aligned bounds in local coordinates.
    pub fn bounds(&self) -> Aabb {
        match self {
            Shape::Box { size } => Aabb::from_center_half(V3::ZERO, arr3(*size) * 0.5),
            Shape::Cylinder { radius, height } => Aabb::from_center_half(V3::ZERO, v3(*radius, *radius, height * 0.5)),
            Shape::Sphere { radius } => Aabb::from_center_half(V3::ZERO, v3(*radius, *radius, *radius)),
            Shape::Prism { points, height } => {
                let mut b = Aabb::empty();
                for p in points {
                    b = b.include(v3(p[0], p[1], -height * 0.5)).include(v3(p[0], p[1], height * 0.5));
                }
                b
            }
            Shape::Union { items } => items.iter().fold(Aabb::empty(), |b, s| b.union(&s.bounds())),
            Shape::Intersect { items } => {
                let mut it = items.iter();
                let first = it.next().map(Shape::bounds).unwrap_or_else(Aabb::empty);
                it.fold(first, |b, s| b.intersect(&s.bounds()))
            }
            Shape::Subtract { base, .. } => base.bounds(),
            Shape::Place { shape, at, rot } => shape.bounds().transformed(&Xform::new(arr3(*at), arr3(*rot))),
            Shape::Offset { shape, d } => {
                let b = shape.bounds();
                if b.is_empty() { b } else { b.grow(d.max(0.0)) }
            }
        }
    }
}

fn combine2(d: f64, h: f64) -> f64 {
    let out = (d.max(0.0).powi(2) + h.max(0.0).powi(2)).sqrt();
    out + d.max(h).min(0.0)
}

/// Exact signed distance from (x, y) to a simple polygon (negative inside).
fn poly_sdf(pts: &[[f64; 2]], x: f64, y: f64) -> f64 {
    let n = pts.len();
    let mut d2 = f64::INFINITY;
    let mut inside = false;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
        let (wx, wy) = (x - a[0], y - a[1]);
        let l2 = ex * ex + ey * ey;
        let t = if l2 > 0.0 { ((wx * ex + wy * ey) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let (dx, dy) = (wx - ex * t, wy - ey * t);
        d2 = d2.min(dx * dx + dy * dy);
        if (a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    let d = d2.sqrt();
    if inside { -d } else { d }
}

fn mirror_shape(s: &Shape, axis: usize) -> Shape {
    let flip = |mut a: [f64; 3]| {
        if let Some(v) = a.get_mut(axis) {
            *v = -*v;
        }
        a
    };
    match s {
        Shape::Box { .. } | Shape::Cylinder { .. } | Shape::Sphere { .. } => s.clone(),
        Shape::Prism { points, height } => {
            let pts = match axis {
                0 => points.iter().map(|p| [-p[0], p[1]]).collect(),
                1 => points.iter().map(|p| [p[0], -p[1]]).collect(),
                _ => points.clone(),
            };
            Shape::Prism { points: pts, height: *height }
        }
        Shape::Union { items } => Shape::Union { items: items.iter().map(|i| mirror_shape(i, axis)).collect() },
        Shape::Intersect { items } => Shape::Intersect { items: items.iter().map(|i| mirror_shape(i, axis)).collect() },
        Shape::Subtract { base, cut } => {
            Shape::Subtract { base: Box::new(mirror_shape(base, axis)), cut: cut.iter().map(|c| mirror_shape(c, axis)).collect() }
        }
        Shape::Offset { shape, d } => Shape::Offset { shape: Box::new(mirror_shape(shape, axis)), d: *d },
        Shape::Place { shape, at, rot } => {
            // Reflect the placement: M * (R * p + t) = (M R M) * (M p) + M t, and the child is mirrored too.
            let r = crate::math::Mat3::from_euler_deg(arr3(*rot));
            let mut m = crate::math::Mat3::IDENTITY;
            m.0[axis][axis] = -1.0;
            let rm = m.mul(&r).mul(&m);
            let (ex, ey, ez) = euler_from_mat(&rm);
            Shape::Place { shape: Box::new(mirror_shape(shape, axis)), at: flip(*at), rot: [ex, ey, ez] }
        }
    }
}

/// Euler angles in degrees such that `Mat3::from_euler_deg` reproduces `m` (`Rz*Ry*Rx`).
pub fn euler_from_mat(m: &crate::math::Mat3) -> (f64, f64, f64) {
    let a = &m.0;
    let sy = -a[2][0];
    let y = sy.clamp(-1.0, 1.0).asin();
    let (x, z);
    if y.cos().abs() > 1e-9 {
        x = a[2][1].atan2(a[2][2]);
        z = a[1][0].atan2(a[0][0]);
    } else {
        x = (-a[1][2]).atan2(a[1][1]);
        z = 0.0;
    }
    (x.to_degrees(), y.to_degrees(), z.to_degrees())
}

/// Fraction of a cubic voxel of edge `h` covered by a solid with signed distance `d` at its centre.
pub fn coverage(d: f64, h: f64) -> f64 {
    (0.5 - d / h).clamp(0.0, 1.0)
}

/// A shape prepared for fast evaluation: rotations precomputed, child bounds cached so unions and
/// subtractions skip children that cannot matter at the query point.
#[derive(Clone, Debug)]
pub struct Compiled {
    node: Node,
    pub bounds: Aabb,
}

#[derive(Clone, Debug)]
enum Node {
    Box(V3),
    Cyl { r: f64, h2: f64 },
    Sphere(f64),
    Prism { pts: Vec<[f64; 2]>, h2: f64 },
    Union(Vec<Compiled>),
    Intersect(Vec<Compiled>),
    Subtract { base: Box<Compiled>, cuts: Vec<Compiled> },
    Place { child: Box<Compiled>, rt: Mat3, t: V3 },
    Offset { child: Box<Compiled>, d: f64 },
}

impl Shape {
    pub fn compile(&self) -> Compiled {
        let bounds = self.bounds();
        let node = match self {
            Shape::Box { size } => Node::Box(arr3(*size) * 0.5),
            Shape::Cylinder { radius, height } => Node::Cyl { r: *radius, h2: height * 0.5 },
            Shape::Sphere { radius } => Node::Sphere(*radius),
            Shape::Prism { points, height } => Node::Prism { pts: points.clone(), h2: height * 0.5 },
            Shape::Union { items } => Node::Union(items.iter().map(Shape::compile).collect()),
            Shape::Intersect { items } => Node::Intersect(items.iter().map(Shape::compile).collect()),
            Shape::Subtract { base, cut } => Node::Subtract { base: Box::new(base.compile()), cuts: cut.iter().map(Shape::compile).collect() },
            Shape::Place { shape, at, rot } => {
                Node::Place { child: Box::new(shape.compile()), rt: Mat3::from_euler_deg(arr3(*rot)).transpose(), t: arr3(*at) }
            }
            Shape::Offset { shape, d } => Node::Offset { child: Box::new(shape.compile()), d: *d },
        };
        Compiled { node, bounds }
    }
}

impl Compiled {
    pub fn sdf(&self, p: V3) -> f64 {
        match &self.node {
            Node::Box(h) => {
                let q = v3(p.x.abs() - h.x, p.y.abs() - h.y, p.z.abs() - h.z);
                v3(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).len() + q.x.max(q.y).max(q.z).min(0.0)
            }
            Node::Cyl { r, h2 } => combine2((p.x * p.x + p.y * p.y).sqrt() - r, p.z.abs() - h2),
            Node::Sphere(r) => p.len() - r,
            Node::Prism { pts, h2 } => combine2(poly_sdf(pts, p.x, p.y), p.z.abs() - h2),
            Node::Union(items) => {
                let mut best = f64::INFINITY;
                for it in items {
                    if it.bounds.dist(p) >= best {
                        continue;
                    }
                    best = best.min(it.sdf(p));
                }
                best
            }
            Node::Intersect(items) => items.iter().map(|i| i.sdf(p)).fold(f64::NEG_INFINITY, f64::max),
            Node::Subtract { base, cuts } => {
                let mut d = base.sdf(p);
                for c in cuts {
                    if c.bounds.dist(p) >= -d {
                        continue;
                    }
                    d = d.max(-c.sdf(p));
                }
                d
            }
            Node::Place { child, rt, t } => child.sdf(rt.mul_v(p - *t)),
            Node::Offset { child, d } => child.sdf(p) - d,
        }
    }
}
