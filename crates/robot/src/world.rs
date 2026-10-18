//! Resolved geometry: catalog parts expanded, joints applied, fasteners turned into solids.

use std::collections::{BTreeMap, HashMap};

use crate::catalog::{self, CatalogEntry};
use crate::math::{Aabb, Mat3, V3, Xform, v3};
use crate::profile::Profile;
use std::sync::Arc;

use crate::sdf::{Compiled, Shape};
use crate::spec::{Part, PartKind, Spec};

#[derive(Clone, Debug)]
pub struct RPart {
    pub part: Part,
    pub shape: Shape,
    pub cs: Arc<Compiled>,
    pub cat: Option<CatalogEntry>,
}

#[derive(Clone, Debug)]
pub struct Resolved {
    pub spec: Spec,
    pub profile: Profile,
    pub parts: Vec<RPart>,
    /// Problems found while resolving (unknown catalog id, missing shape, bad shape).
    pub load_errors: Vec<(String, String)>,
}

pub fn resolve(spec: Spec) -> Resolved {
    let profile = Profile::named(&spec.profile);
    let mut parts = Vec::new();
    let mut load_errors = Vec::new();
    for p in &spec.parts {
        let cat = match &p.catalog {
            Some(id) => {
                let c = catalog::find(id);
                if c.is_none() {
                    load_errors.push((p.id.clone(), format!("unknown catalog id `{id}`")));
                }
                c
            }
            None => None,
        };
        let mut shape = match (&cat, &p.shape) {
            (Some(c), _) => c.shape.clone(),
            (None, Some(s)) => s.clone(),
            (None, None) => {
                load_errors.push((p.id.clone(), "part has neither `shape` nor `catalog`".into()));
                Shape::boxed(0.1, 0.1, 0.1)
            }
        };
        if let Err(e) = shape.validate() {
            load_errors.push((p.id.clone(), format!("bad shape: {e}")));
            shape = Shape::boxed(0.1, 0.1, 0.1);
        }
        let cs = Arc::new(shape.compile());
        parts.push(RPart { part: p.clone(), shape, cs, cat });
    }
    Resolved { spec, profile, parts, load_errors }
}

#[derive(Clone, Debug)]
pub struct Body {
    pub id: String,
    pub kind: PartKind,
    pub shape: Shape,
    pub cs: Arc<Compiled>,
    pub xf: Xform,
    pub aabb: Aabb,
    pub color: [u8; 3],
    /// Index into `Resolved::parts`, or `None` for generated fasteners.
    pub part_idx: Option<usize>,
    pub fastener: bool,
}

#[derive(Clone, Debug)]
pub struct World {
    pub bodies: Vec<Body>,
    idx: HashMap<String, usize>,
    pub joints: BTreeMap<String, f64>,
}

pub fn parse_color(s: &str) -> Option<[u8; 3]> {
    let h = s.trim().trim_start_matches('#');
    if h.len() != 6 {
        return None;
    }
    let v = u32::from_str_radix(h, 16).ok()?;
    Some([(v >> 16) as u8, (v >> 8) as u8, v as u8])
}

pub fn default_color(kind: PartKind) -> [u8; 3] {
    match kind {
        PartKind::Structure => [0x2b, 0x2e, 0x34],
        PartKind::Armor => [0x44, 0x48, 0x50],
        PartKind::Wedge => [0xff, 0x6a, 0x1a],
        PartKind::Motor => [0x9a, 0xa0, 0xa6],
        PartKind::WeaponMotor => [0x5a, 0x5e, 0x66],
        PartKind::Wheel => [0x25, 0x28, 0x2c],
        PartKind::Battery => [0x2a, 0x9d, 0x8f],
        PartKind::Esc => [0xe7, 0x6f, 0x51],
        PartKind::Receiver => [0xff, 0x98, 0x00],
        PartKind::Servo => [0x4a, 0x4e, 0x57],
        PartKind::Switch => [0xc0, 0x20, 0x20],
        PartKind::Weapon => [0xc8, 0xcd, 0xd2],
        PartKind::LockingBar => [0xff, 0xd1, 0x00],
        PartKind::Plate => [0x1c, 0x1c, 0x1e],
        _ => [0xb0, 0xb4, 0xba],
    }
}

impl World {
    /// Build the world with each joint at the given value (start pose when missing).
    pub fn build(r: &Resolved, joint_values: &BTreeMap<String, f64>) -> World {
        let by_id: HashMap<&str, usize> = r.parts.iter().enumerate().map(|(i, p)| (p.part.id.as_str(), i)).collect();
        let mut cache: Vec<Option<Xform>> = vec![None; r.parts.len()];
        let mut bodies = Vec::new();
        let mut joints = BTreeMap::new();
        for (i, rp) in r.parts.iter().enumerate() {
            let xf = world_xf(r, &by_id, i, joint_values, &mut cache, 0);
            if let Some(j) = &rp.part.joint {
                let v = joint_values.get(&rp.part.id).copied().or_else(|| r.spec.start_pose.get(&rp.part.id).copied()).unwrap_or(j.default);
                joints.insert(rp.part.id.clone(), v);
            }
            let aabb = rp.shape.bounds().transformed(&xf);
            let color = rp
                .part
                .color
                .as_deref()
                .and_then(parse_color)
                .or_else(|| rp.cat.as_ref().and_then(|c| parse_color(&c.color)))
                .unwrap_or_else(|| default_color(rp.part.kind));
            bodies.push(Body {
                id: rp.part.id.clone(),
                kind: rp.part.kind,
                shape: rp.shape.clone(),
                cs: rp.cs.clone(),
                xf,
                aabb,
                color,
                part_idx: Some(i),
                fastener: false,
            });
        }
        for f in &r.spec.fasteners {
            if let Some((d, l)) = catalog::parse_designation(&f.designation) {
                let (dk, k, _) = catalog::screw_dims(d);
                let head = Shape::cyl(dk / 2.0, k).at(0.0, 0.0, -k / 2.0);
                let shank = Shape::cyl(d / 2.0, l).at(0.0, 0.0, l / 2.0);
                let shape = Shape::union(vec![head, shank]);
                let dir = V3::from_arr(f.dir);
                let xf = Xform { r: Mat3::align_z_to(dir), t: V3::from_arr(f.at) };
                let aabb = shape.bounds().transformed(&xf);
                let cs = Arc::new(shape.compile());
                bodies.push(Body {
                    id: f.id.clone(),
                    kind: PartKind::Other,
                    shape,
                    cs,
                    xf,
                    aabb,
                    color: [0xcc, 0xcc, 0xd0],
                    part_idx: None,
                    fastener: true,
                });
            }
        }
        let idx = bodies.iter().enumerate().map(|(i, b)| (b.id.clone(), i)).collect();
        World { bodies, idx, joints }
    }

    /// A copy with parts moved apart for an exploded view (`amount` 1.0 = full).
    pub fn exploded(&self, amount: f64) -> World {
        let mut w = self.clone();
        for b in &mut w.bodies {
            let id = b.id.as_str();
            let d = if b.fastener {
                v3(0.0, 0.0, 42.0)
            } else if id.contains("top") {
                v3(0.0, 0.0, 30.0)
            } else {
                match b.kind {
                    PartKind::Wheel => v3(if b.xf.t.x >= 0.0 { 16.0 } else { -16.0 }, 0.0, 0.0),
                    PartKind::Insert | PartKind::Nut => v3(0.0, 0.0, -12.0),
                    PartKind::Battery | PartKind::Esc | PartKind::Receiver | PartKind::Servo => v3(0.0, 0.0, 12.0),
                    PartKind::Switch => v3(0.0, 0.0, 22.0),
                    PartKind::Wedge => v3(0.0, 9.0, -4.0),
                    PartKind::Weapon | PartKind::LockingBar => v3(0.0, 0.0, 26.0),
                    PartKind::Motor | PartKind::WeaponMotor => v3(if b.xf.t.x >= 0.0 { 6.0 } else { -6.0 }, 0.0, 4.0),
                    _ => V3::ZERO,
                }
            } * amount;
            b.xf.t = b.xf.t + d;
            b.aabb = b.shape.bounds().transformed(&b.xf);
        }
        w
    }

    pub fn start(r: &Resolved) -> World {
        World::build(r, &BTreeMap::new())
    }

    pub fn index(&self, id: &str) -> Option<usize> {
        self.idx.get(id).copied()
    }

    pub fn body(&self, id: &str) -> Option<&Body> {
        self.index(id).and_then(|i| self.bodies.get(i))
    }

    /// A copy of this world with each routed wire added as a tube body, for rendering and exports.
    pub fn with_routes(&self, routes: &[crate::checks::wiring::Route]) -> World {
        let mut w = self.clone();
        for rt in routes {
            let shape = Shape::Tube { points: rt.points.clone(), radius: rt.od_mm * 0.5 };
            let color = match rt.kind.as_str() {
                "power" => [0xd0, 0x20, 0x20],
                "motor" => [0xe8, 0xb0, 0x20],
                _ => [0x30, 0x80, 0xe0],
            };
            let id = format!("wire:{}", rt.id);
            w.idx.insert(id.clone(), w.bodies.len());
            w.bodies.push(Body {
                id,
                kind: PartKind::Other,
                aabb: shape.bounds(),
                cs: Arc::new(shape.compile()),
                shape,
                xf: Xform::IDENTITY,
                color,
                part_idx: None,
                fastener: false,
            });
        }
        w
    }

    /// Signed distance of body `i` at world point `p`.
    pub fn sdf(&self, i: usize, p: V3) -> f64 {
        match self.bodies.get(i) {
            Some(b) => b.cs.sdf(b.xf.inv_apply(p)),
            None => f64::INFINITY,
        }
    }

    /// Signed distance to the union of all bodies for which `use_body` is true.
    pub fn scene_sdf(&self, p: V3, use_body: &dyn Fn(usize) -> bool) -> f64 {
        let mut best = f64::INFINITY;
        for (i, b) in self.bodies.iter().enumerate() {
            if !use_body(i) {
                continue;
            }
            // Distance to the box is a lower bound of the distance to the solid inside it.
            if b.aabb.dist(p) >= best.max(0.0) && best >= 0.0 {
                continue;
            }
            best = best.min(self.sdf(i, p));
        }
        best
    }

    pub fn bounds(&self) -> Aabb {
        self.bodies.iter().fold(Aabb::empty(), |a, b| a.union(&b.aabb))
    }

    /// World-space point of a named port (`part.port`).
    pub fn port(&self, r: &Resolved, name: &str) -> Option<V3> {
        let (pid, port) = name.split_once('.')?;
        let i = self.index(pid)?;
        let rp = r.parts.get(self.bodies.get(i)?.part_idx?)?;
        let local = rp.part.ports.get(port).or_else(|| rp.cat.as_ref().and_then(|c| c.ports.get(port)))?;
        Some(self.bodies.get(i)?.xf.apply(V3::from_arr(*local)))
    }
}

fn world_xf(r: &Resolved, by_id: &HashMap<&str, usize>, i: usize, jv: &BTreeMap<String, f64>, cache: &mut Vec<Option<Xform>>, depth: usize) -> Xform {
    if let Some(Some(x)) = cache.get(i) {
        return *x;
    }
    let Some(rp) = r.parts.get(i) else { return Xform::IDENTITY };
    let own = Xform::new(V3::from_arr(rp.part.at), V3::from_arr(rp.part.rot));
    let mut local = own;
    if let Some(j) = &rp.part.joint {
        let v = jv.get(&rp.part.id).copied().or_else(|| r.spec.start_pose.get(&rp.part.id).copied()).unwrap_or(j.default);
        let axis = if V3::from_arr(j.axis).len() > 1e-9 { V3::from_arr(j.axis) } else { v3(1.0, 0.0, 0.0) };
        let jx = if j.kind == "prismatic" { Xform::translation(axis.norm() * v) } else { Xform::rotation_about(V3::from_arr(j.pivot), axis, v) };
        local = jx.then_after(&own);
    }
    let parent = rp.part.parent.as_deref().and_then(|p| by_id.get(p).copied());
    let x = match parent {
        Some(pi) if depth < 16 && pi != i => world_xf(r, by_id, pi, jv, cache, depth + 1).then_after(&local),
        _ => local,
    };
    if let Some(c) = cache.get_mut(i) {
        *c = Some(x);
    }
    x
}
