//! Boundary-representation solids built from a [`Shape`] tree, for STEP export.
//!
//! The SDF tree stays the source of truth for checks; this module rebuilds the same tree as real
//! B-rep solids (planes, cylinders, spheres, NURBS) with exact booleans, so a part can be opened in
//! a CAD program. Wires are not manufactured, so a tube that cannot be unioned falls back to a set
//! of overlapping solids.

use std::f64::consts::{PI, TAU};
use std::panic::{AssertUnwindSafe, catch_unwind};

use truck_modeling::{Matrix4, Point3, Rad, Solid, Vector3, builder};
use truck_shapeops::{and, or};

use crate::math::Xform;
use crate::sdf::Shape;

/// Boolean tolerances tried in order (mm).
const TOLS: [f64; 4] = [0.05, 0.01, 0.1, 0.002];

fn pt(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Axis-aligned box centred on the origin.
pub fn box_solid(x: f64, y: f64, z: f64) -> Solid {
    let v = builder::vertex(pt(-x / 2.0, -y / 2.0, -z / 2.0));
    let e = builder::tsweep(&v, Vector3::new(x, 0.0, 0.0));
    let f = builder::tsweep(&e, Vector3::new(0.0, y, 0.0));
    builder::tsweep(&f, Vector3::new(0.0, 0.0, z))
}

/// Cylinder along Z centred on the origin.
pub fn cylinder_solid(r: f64, h: f64) -> Result<Solid, String> {
    let v = builder::vertex(pt(r, 0.0, -h / 2.0));
    let w = builder::rsweep(&v, pt(0.0, 0.0, -h / 2.0), Vector3::unit_z(), Rad(TAU));
    let f = builder::try_attach_plane(&[w]).map_err(|e| format!("cylinder cap: {e}"))?;
    Ok(builder::tsweep(&f, Vector3::new(0.0, 0.0, h)))
}

/// Sphere centred on the origin.
pub fn sphere_solid(r: f64) -> Result<Solid, String> {
    let top = builder::vertex(pt(0.0, 0.0, r));
    let arc = builder::rsweep(&top, pt(0.0, 0.0, 0.0), Vector3::unit_y(), Rad(PI));
    let shell = builder::rsweep(&arc, pt(0.0, 0.0, 0.0), Vector3::unit_z(), Rad(TAU));
    Solid::try_new(vec![shell]).map_err(|e| format!("sphere: {e}"))
}

/// Polygon in XY extruded along Z, centred on the origin.
pub fn prism_solid(points: &[[f64; 2]], h: f64) -> Result<Solid, String> {
    if points.len() < 3 {
        return Err("prism needs 3 or more points".into());
    }
    let mut p: Vec<[f64; 2]> = points.to_vec();
    let area: f64 = (0..p.len())
        .map(|i| {
            let a = p[i];
            let b = p[(i + 1) % p.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    if area < 0.0 {
        p.reverse();
    }
    let vs: Vec<_> = p.iter().map(|q| builder::vertex(pt(q[0], q[1], -h / 2.0))).collect();
    let edges: Vec<_> = (0..vs.len()).map(|i| builder::line(&vs[i], &vs[(i + 1) % vs.len()])).collect();
    let wire: truck_modeling::Wire = edges.into_iter().collect();
    let f = builder::try_attach_plane(&[wire]).map_err(|e| format!("prism face: {e}"))?;
    Ok(builder::tsweep(&f, Vector3::new(0.0, 0.0, h)))
}

fn matrix(xf: &Xform) -> Matrix4 {
    let m = &xf.r.0;
    let g = |r: usize, c: usize| m.get(r).and_then(|row| row.get(c)).copied().unwrap_or(0.0);
    // cgmath is column-major
    Matrix4::new(
        g(0, 0), g(1, 0), g(2, 0), 0.0,
        g(0, 1), g(1, 1), g(2, 1), 0.0,
        g(0, 2), g(1, 2), g(2, 2), 0.0,
        xf.t.x, xf.t.y, xf.t.z, 1.0,
    )
}

pub fn transform(s: &Solid, xf: &Xform) -> Solid {
    builder::transformed(s, matrix(xf))
}

/// Longest a single boolean may run before it is abandoned (the kernel has no cancellation, so
/// the worker thread is left to finish on its own).
const OP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(10);

fn guarded<T: Send + 'static>(f: impl FnOnce() -> Option<T> + Send + 'static) -> Option<T> {
    let (tx, rx) = std::sync::mpsc::channel();
    let spawned = std::thread::Builder::new().stack_size(64 << 20).spawn(move || {
        let r = catch_unwind(AssertUnwindSafe(f)).ok().flatten();
        let _ = tx.send(r);
    });
    spawned.ok()?;
    rx.recv_timeout(OP_TIMEOUT).ok().flatten()
}

/// Centre of the vertex bounding box.
fn centre(s: &Solid) -> Option<[f64; 3]> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for sh in s.boundaries() {
        for v in sh.vertex_iter() {
            let p = v.point();
            for (k, c) in [p.x, p.y, p.z].into_iter().enumerate() {
                if let (Some(l), Some(h)) = (lo.get_mut(k), hi.get_mut(k)) {
                    *l = l.min(c);
                    *h = h.max(c);
                }
            }
        }
    }
    if lo.iter().chain(hi.iter()).any(|v| !v.is_finite()) {
        return None;
    }
    Some([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0])
}

/// Scale about the bounding-box centre. Used to break coincident faces, which the kernel's
/// booleans cannot handle: the change is a few microns.
fn scaled(s: &Solid, k: f64) -> Solid {
    let c = centre(s).unwrap_or([0.0; 3]);
    let t = Matrix4::from_translation(Vector3::new(c[0], c[1], c[2]))
        * Matrix4::from_scale(k)
        * Matrix4::from_translation(Vector3::new(-c[0], -c[1], -c[2]));
    builder::transformed(s, t)
}

/// Operand variants tried when the exact booleans fail, from least to most change.
const NUDGES: [f64; 8] = [1.0 + 2e-4, 1.0 - 2e-4, 1.0 + 1e-3, 1.0 - 1e-3, 1.0 + 5e-3, 1.0 - 5e-3, 1.0 + 2e-2, 1.0];

fn boolean(a: &Solid, b: &Solid, op: fn(&Solid, &Solid, f64) -> Option<Solid>, what: &str) -> Result<Solid, String> {
    let dbg = std::env::var_os("ROBOT_BREP_DEBUG").is_some();
    for k in NUDGES {
        let bb = if k == 1.0 { b.clone() } else { scaled(b, k) };
        // Other tolerances only help the exact attempt; nudged operands use the first.
        for tol in TOLS.iter().take(1) {
            let t = std::time::Instant::now();
            let (a2, b2, tol2) = (a.clone(), bb.clone(), *tol);
            let r = guarded(move || op(&a2, &b2, tol2));
            if dbg {
                eprintln!("{what} k={k} tol={tol} {:?} {}", t.elapsed(), r.is_some());
            }
            if let Some(r) = r {
                return Ok(r);
            }
        }
    }
    Err(format!("{what} failed"))
}

fn union2(a: &Solid, b: &Solid) -> Result<Solid, String> {
    boolean(a, b, or, "union")
}

fn subtract2(a: &Solid, b: &Solid) -> Result<Solid, String> {
    let mut inv = b.clone();
    inv.not();
    boolean(a, &inv, and, "subtraction")
}

fn intersect2(a: &Solid, b: &Solid) -> Result<Solid, String> {
    boolean(a, b, and, "intersection")
}

/// Unit-Z cylinder placed between two points.
fn segment_solid(a: [f64; 3], b: [f64; 3], r: f64) -> Result<Option<Solid>, String> {
    let d = Vector3::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    let len = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
    if len < 1e-6 {
        return Ok(None);
    }
    let c = cylinder_solid(r, len)?;
    let z = crate::math::v3(d.x / len, d.y / len, d.z / len);
    let xf = Xform { r: crate::math::Mat3::align_z_to(z), t: crate::math::v3((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0) };
    Ok(Some(transform(&c, &xf)))
}

/// Wires are not manufactured, so a tube is a set of overlapping cylinders and joint spheres
/// rather than a boolean union (which is slow and fragile in the kernel).
fn tube(points: &[[f64; 3]], r: f64) -> Result<Vec<Solid>, String> {
    let mut parts = Vec::new();
    for w in points.windows(2) {
        if let (Some(a), Some(b)) = (w.first(), w.get(1)) {
            if let Some(s) = segment_solid(*a, *b, r)? {
                parts.push(s);
            }
        }
    }
    for p in points.iter().skip(1).take(points.len().saturating_sub(2)) {
        let s = sphere_solid(r)?;
        parts.push(transform(&s, &Xform { r: crate::math::Mat3::IDENTITY, t: crate::math::V3::from_arr(*p) }));
    }
    if parts.is_empty() {
        return Err("empty tube".into());
    }
    Ok(parts)
}

/// Rebuild `shape` as B-rep. Usually one solid; a failed tube union gives several.
pub fn solids(shape: &Shape) -> Result<Vec<Solid>, String> {
    shape.validate()?;
    build(shape)
}

/// Balanced pairwise union: smaller operands make each boolean cheaper than a left fold.
fn union_all(mut parts: Vec<Solid>) -> Result<Solid, String> {
    if parts.is_empty() {
        return Err("empty union".into());
    }
    while parts.len() > 1 {
        let mut next = Vec::with_capacity(parts.len() / 2 + 1);
        let mut it = parts.into_iter();
        while let Some(a) = it.next() {
            match it.next() {
                Some(b) => next.push(union2(&a, &b)?),
                None => next.push(a),
            }
        }
        parts = next;
    }
    parts.pop().ok_or_else(|| "empty union".to_string())
}

thread_local! {
    static NOTES: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn note(s: String) {
    NOTES.with(|n| n.borrow_mut().push(s));
}

/// Like [`solids`], plus warnings about operations that had to be skipped (geometry is then
/// slightly off: a hole or pocket is missing).
pub fn solids_with_notes(shape: &Shape) -> (Result<Vec<Solid>, String>, Vec<String>) {
    NOTES.with(|n| n.borrow_mut().clear());
    let r = solids(shape);
    (r, NOTES.with(|n| std::mem::take(&mut *n.borrow_mut())))
}

fn one(shape: &Shape) -> Result<Solid, String> {
    let mut v = build(shape)?;
    if v.len() == 1 { v.pop().ok_or_else(|| "empty".to_string()) } else { Err("a boolean operand split into several solids".into()) }
}

fn build(shape: &Shape) -> Result<Vec<Solid>, String> {
    Ok(match shape {
        Shape::Box { size } => vec![box_solid(size[0], size[1], size[2])],
        Shape::Cylinder { radius, height } => vec![cylinder_solid(*radius, *height)?],
        Shape::Sphere { radius } => vec![sphere_solid(*radius)?],
        Shape::Prism { points, height } => vec![prism_solid(points, *height)?],
        Shape::Tube { points, radius } => tube(points, *radius)?,
        Shape::Union { items } => {
            let parts = items.iter().map(one).collect::<Result<Vec<_>, _>>()?;
            vec![union_all(parts)?]
        }
        Shape::Intersect { items } => {
            let mut it = items.iter();
            let Some(first) = it.next() else { return Err("empty intersection".into()) };
            let mut acc = one(first)?;
            for s in it {
                acc = intersect2(&acc, &one(s)?)?;
            }
            vec![acc]
        }
        Shape::Subtract { base, cut } => {
            let mut acc = one(base)?;
            for c in cut {
                for s in build(c)? {
                    match subtract2(&acc, &s) {
                        Ok(r) => acc = r,
                        Err(e) => note(format!("a cut was skipped: {e}")),
                    }
                }
            }
            vec![acc]
        }
        Shape::Place { shape, at, rot } => {
            let xf = Xform::new(crate::math::V3::from_arr(*at), crate::math::V3::from_arr(*rot));
            build(shape)?.iter().map(|s| transform(s, &xf)).collect()
        }
        Shape::Offset { shape, d } => vec![offset(shape, *d)?],
    })
}

/// Offset is exact only for primitives (the offset of a box would round its edges, so it is
/// approximated by a larger box).
fn offset(shape: &Shape, d: f64) -> Result<Solid, String> {
    match shape {
        Shape::Box { size } => Ok(box_solid(size[0] + 2.0 * d, size[1] + 2.0 * d, size[2] + 2.0 * d)),
        Shape::Cylinder { radius, height } => cylinder_solid(radius + d, height + 2.0 * d),
        Shape::Sphere { radius } => sphere_solid(radius + d),
        Shape::Place { shape, at, rot } => {
            let xf = Xform::new(crate::math::V3::from_arr(*at), crate::math::V3::from_arr(*rot));
            Ok(transform(&offset(shape, d)?, &xf))
        }
        _ => Err("offset of a composite shape is not supported in B-rep".into()),
    }
}
