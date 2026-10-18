//! Small 3D math: vectors, rotation matrices, rigid transforms and boxes.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub fn from_arr(a: [f64; 3]) -> V3 {
        v3(a[0], a[1], a[2])
    }
    pub fn arr(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: V3) -> V3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn norm(self) -> V3 {
        let l = self.len();
        if l > 1e-12 { self * (1.0 / l) } else { V3::ZERO }
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    pub fn min(self, o: V3) -> V3 {
        v3(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }
    pub fn max(self, o: V3) -> V3 {
        v3(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }
    pub fn get(self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

impl std::ops::Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl std::ops::Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl std::ops::Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, k: f64) -> V3 {
        v3(self.x * k, self.y * k, self.z * k)
    }
}
impl std::ops::Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

/// Row-major 3x3 rotation matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3(pub [[f64; 3]; 3]);

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    pub fn mul_v(&self, v: V3) -> V3 {
        let m = &self.0;
        v3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        )
    }
    pub fn mul(&self, o: &Mat3) -> Mat3 {
        let mut r = [[0.0; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, c) in row.iter_mut().enumerate() {
                *c = (0..3).map(|k| self.0[i][k] * o.0[k][j]).sum();
            }
        }
        Mat3(r)
    }
    pub fn transpose(&self) -> Mat3 {
        let m = &self.0;
        Mat3([[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]])
    }
    /// Rotation `R = Rz * Ry * Rx` from Euler angles in degrees.
    pub fn from_euler_deg(rot: V3) -> Mat3 {
        let (sx, cx) = rot.x.to_radians().sin_cos();
        let (sy, cy) = rot.y.to_radians().sin_cos();
        let (sz, cz) = rot.z.to_radians().sin_cos();
        let rx = Mat3([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]);
        let ry = Mat3([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]);
        let rz = Mat3([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]);
        rz.mul(&ry).mul(&rx)
    }
    /// Rotation taking +Z onto the unit direction `d`.
    pub fn align_z_to(d: V3) -> Mat3 {
        let d = d.norm();
        if d.len() < 0.5 {
            return Mat3::IDENTITY;
        }
        let z = v3(0.0, 0.0, 1.0);
        let c = z.dot(d);
        if c > 1.0 - 1e-12 {
            return Mat3::IDENTITY;
        }
        if c < -1.0 + 1e-12 {
            return Mat3::axis_angle_deg(v3(1.0, 0.0, 0.0), 180.0);
        }
        let axis = z.cross(d);
        Mat3::axis_angle_deg(axis, c.clamp(-1.0, 1.0).acos().to_degrees())
    }
    /// Rotation by `deg` about the unit `axis` (Rodrigues).
    pub fn axis_angle_deg(axis: V3, deg: f64) -> Mat3 {
        let a = axis.norm();
        let (s, c) = deg.to_radians().sin_cos();
        let t = 1.0 - c;
        Mat3([
            [t * a.x * a.x + c, t * a.x * a.y - s * a.z, t * a.x * a.z + s * a.y],
            [t * a.x * a.y + s * a.z, t * a.y * a.y + c, t * a.y * a.z - s * a.x],
            [t * a.x * a.z - s * a.y, t * a.y * a.z + s * a.x, t * a.z * a.z + c],
        ])
    }
}

/// Rigid transform `p -> r*p + t`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Xform {
    pub r: Mat3,
    pub t: V3,
}

impl Xform {
    pub const IDENTITY: Xform = Xform { r: Mat3::IDENTITY, t: V3::ZERO };
    pub fn new(pos: V3, rot_deg: V3) -> Xform {
        Xform { r: Mat3::from_euler_deg(rot_deg), t: pos }
    }
    pub fn apply(&self, p: V3) -> V3 {
        self.r.mul_v(p) + self.t
    }
    pub fn apply_dir(&self, d: V3) -> V3 {
        self.r.mul_v(d)
    }
    pub fn inv_apply(&self, p: V3) -> V3 {
        self.r.transpose().mul_v(p - self.t)
    }
    pub fn inv_dir(&self, d: V3) -> V3 {
        self.r.transpose().mul_v(d)
    }
    /// `self * o`: apply `o` first, then `self`.
    pub fn then_after(&self, o: &Xform) -> Xform {
        Xform { r: self.r.mul(&o.r), t: self.r.mul_v(o.t) + self.t }
    }
    /// Rotation by `deg` about the line through `pivot` along `axis`.
    pub fn rotation_about(pivot: V3, axis: V3, deg: f64) -> Xform {
        let r = Mat3::axis_angle_deg(axis, deg);
        Xform { r, t: pivot - r.mul_v(pivot) }
    }
    pub fn translation(t: V3) -> Xform {
        Xform { r: Mat3::IDENTITY, t }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    pub min: V3,
    pub max: V3,
}

impl Aabb {
    pub fn empty() -> Aabb {
        Aabb { min: v3(f64::INFINITY, f64::INFINITY, f64::INFINITY), max: v3(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY) }
    }
    pub fn from_center_half(c: V3, h: V3) -> Aabb {
        Aabb { min: c - h, max: c + h }
    }
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }
    pub fn union(&self, o: &Aabb) -> Aabb {
        Aabb { min: self.min.min(o.min), max: self.max.max(o.max) }
    }
    pub fn include(&self, p: V3) -> Aabb {
        Aabb { min: self.min.min(p), max: self.max.max(p) }
    }
    pub fn intersect(&self, o: &Aabb) -> Aabb {
        Aabb { min: self.min.max(o.min), max: self.max.min(o.max) }
    }
    pub fn overlaps(&self, o: &Aabb, margin: f64) -> bool {
        self.min.x <= o.max.x + margin
            && self.max.x + margin >= o.min.x
            && self.min.y <= o.max.y + margin
            && self.max.y + margin >= o.min.y
            && self.min.z <= o.max.z + margin
            && self.max.z + margin >= o.min.z
    }
    pub fn grow(&self, m: f64) -> Aabb {
        Aabb { min: self.min - v3(m, m, m), max: self.max + v3(m, m, m) }
    }
    pub fn size(&self) -> V3 {
        if self.is_empty() { V3::ZERO } else { self.max - self.min }
    }
    pub fn center(&self) -> V3 {
        (self.min + self.max) * 0.5
    }
    pub fn corners(&self) -> [V3; 8] {
        let (a, b) = (self.min, self.max);
        [
            v3(a.x, a.y, a.z),
            v3(b.x, a.y, a.z),
            v3(a.x, b.y, a.z),
            v3(b.x, b.y, a.z),
            v3(a.x, a.y, b.z),
            v3(b.x, a.y, b.z),
            v3(a.x, b.y, b.z),
            v3(b.x, b.y, b.z),
        ]
    }
    pub fn transformed(&self, x: &Xform) -> Aabb {
        self.corners().iter().fold(Aabb::empty(), |b, c| b.include(x.apply(*c)))
    }
    pub fn contains(&self, p: V3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y && p.z >= self.min.z && p.z <= self.max.z
    }
    /// Distance from `p` to the box (0 inside).
    pub fn dist(&self, p: V3) -> f64 {
        let dx = (self.min.x - p.x).max(p.x - self.max.x).max(0.0);
        let dy = (self.min.y - p.y).max(p.y - self.max.y).max(0.0);
        let dz = (self.min.z - p.z).max(p.z - self.max.z).max(0.0);
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}
