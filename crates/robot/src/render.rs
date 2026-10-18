//! Orthographic renders of the solids by sphere tracing: fixed views, sections, failure markers.

use crate::math::{Aabb, V3, v3};
use crate::world::World;

#[derive(Clone, Debug)]
pub struct View {
    pub name: String,
    /// Direction the camera looks along (from the camera into the scene).
    pub dir: V3,
    pub up: V3,
    /// Keep only the half-space `n . p <= d` (a section cut).
    pub section: Option<(V3, f64)>,
    pub width: u32,
    /// Move parts apart (1.0 = full explosion).
    pub explode: f64,
}

impl View {
    pub fn named(name: &str, width: u32) -> Option<View> {
        let z = v3(0.0, 0.0, 1.0);
        let (dir, up, section) = match name {
            "top" => (v3(0.0, 0.0, -1.0), v3(0.0, 1.0, 0.0), None),
            "bottom" => (v3(0.0, 0.0, 1.0), v3(0.0, -1.0, 0.0), None),
            "front" => (v3(0.0, -1.0, 0.0), z, None),
            "back" => (v3(0.0, 1.0, 0.0), z, None),
            "side" => (v3(-1.0, 0.0, 0.0), z, None),
            "iso" => (v3(-0.55, 0.75, -0.62).norm(), z, None),
            "exploded" => (v3(-0.5, 0.78, -0.55).norm(), z, None),
            "iso_back" => (v3(0.6, -0.7, -0.55).norm(), z, None),
            "section_z" => (v3(0.0, 0.0, -1.0), v3(0.0, 1.0, 0.0), Some((z, 0.0))),
            "section_x" => (v3(-1.0, 0.0, 0.0), z, Some((v3(1.0, 0.0, 0.0), 0.0))),
            _ => return None,
        };
        Some(View { name: name.into(), dir, up, section, width, explode: if name == "exploded" { 1.0 } else { 0.0 } })
    }
}

pub const VIEW_NAMES: &[&str] = &["iso", "exploded", "iso_back", "top", "front", "side", "section_z", "section_x"];

pub struct Marker {
    pub at: V3,
    pub number: usize,
}

struct Cam {
    right: V3,
    up: V3,
    dir: V3,
    center: V3,
    scale: f64, // mm per pixel
    w: u32,
    h: u32,
    depth: f64,
}

fn camera(b: &Aabb, v: &View) -> Cam {
    let dir = v.dir.norm();
    let right = dir.cross(v.up).norm();
    let up = right.cross(dir).norm();
    let c = b.center();
    let (mut minx, mut maxx, mut miny, mut maxy, mut minz, mut maxz) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in b.corners() {
        let q = p - c;
        minx = minx.min(q.dot(right));
        maxx = maxx.max(q.dot(right));
        miny = miny.min(q.dot(up));
        maxy = maxy.max(q.dot(up));
        minz = minz.min(q.dot(dir));
        maxz = maxz.max(q.dot(dir));
    }
    let pad = 6.0;
    let (wmm, hmm) = (maxx - minx + 2.0 * pad, maxy - miny + 2.0 * pad);
    let w = v.width.max(64);
    let scale = wmm / w as f64;
    let h = ((hmm / scale).ceil() as u32).clamp(32, 4000);
    let ctr = c + right * ((minx + maxx) * 0.5) + up * ((miny + maxy) * 0.5);
    Cam { right, up, dir, center: ctr, scale, w, h, depth: maxz - minz + 40.0 }
}

fn project(c: &Cam, p: V3) -> (f64, f64) {
    let q = p - c.center;
    (c.w as f64 / 2.0 + q.dot(c.right) / c.scale, c.h as f64 / 2.0 - q.dot(c.up) / c.scale)
}

struct Hit {
    body: usize,
    n: V3,
}

fn trace(world: &World, near: &[usize], o: V3, d: V3, tmax: f64, section: Option<(V3, f64)>) -> Option<Hit> {
    let f = |p: V3| -> f64 {
        let mut best = f64::INFINITY;
        for &i in near {
            if let Some(b) = world.bodies.get(i) {
                if b.aabb.dist(p) > best.max(0.5) && best >= 0.0 {
                    continue;
                }
                best = best.min(world.sdf(i, p));
            }
        }
        match section {
            Some((n, dd)) => best.max(n.dot(p) - dd),
            None => best,
        }
    };
    let mut t = 0.0;
    for _ in 0..160 {
        let p = o + d * t;
        let s = f(p);
        if s < 0.012 {
            let e = 0.08;
            let n = v3(
                f(p + v3(e, 0.0, 0.0)) - f(p - v3(e, 0.0, 0.0)),
                f(p + v3(0.0, e, 0.0)) - f(p - v3(0.0, e, 0.0)),
                f(p + v3(0.0, 0.0, e)) - f(p - v3(0.0, 0.0, e)),
            )
            .norm();
            let mut body = near.first().copied().unwrap_or(0);
            let mut bd = f64::INFINITY;
            for &i in near {
                let sd = world.sdf(i, p);
                if sd < bd {
                    bd = sd;
                    body = i;
                }
            }
            return Some(Hit { body, n });
        }
        t += s.max(0.02);
        if t > tmax {
            return None;
        }
    }
    None
}

fn ray_hits_aabb(b: &Aabb, o: V3, d: V3) -> bool {
    let mut t0 = f64::NEG_INFINITY;
    let mut t1 = f64::INFINITY;
    for a in 0..3 {
        let (oa, da) = (o.get(a), d.get(a));
        let (lo, hi) = (b.min.get(a), b.max.get(a));
        if da.abs() < 1e-12 {
            if oa < lo || oa > hi {
                return false;
            }
        } else {
            let (mut ta, mut tb) = ((lo - oa) / da, (hi - oa) / da);
            if ta > tb {
                std::mem::swap(&mut ta, &mut tb);
            }
            t0 = t0.max(ta);
            t1 = t1.min(tb);
        }
    }
    t0 <= t1
}

/// Render `view` of the world to PNG bytes. `ground_z` draws a floor grid; `markers` are numbered
/// red rings at failure locations.
pub fn render_png(world: &World, view: &View, ground_z: Option<f64>, markers: &[Marker]) -> Vec<u8> {
    let exploded;
    let world = if view.explode > 0.0 {
        exploded = world.exploded(view.explode);
        &exploded
    } else {
        world
    };
    let b = world.bounds().grow(1.0);
    if b.is_empty() {
        return Vec::new();
    }
    let cam = camera(&b, view);
    let (w, h) = (cam.w as usize, cam.h as usize);
    let light = v3(-0.4, 0.5, 0.8).norm();
    let mut px = vec![[0u8; 3]; w * h];
    let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4).min(16);
    let rows_per = h.div_ceil(threads).max(1);
    std::thread::scope(|s| {
        for (ci, chunk) in px.chunks_mut(rows_per * w).enumerate() {
            let cam = &cam;
            s.spawn(move || {
                for (ri, row) in chunk.chunks_mut(w).enumerate() {
                    let y = ci * rows_per + ri;
                    for (x, out) in row.iter_mut().enumerate() {
                        let u = (x as f64 + 0.5 - cam.w as f64 / 2.0) * cam.scale;
                        let vv = (cam.h as f64 / 2.0 - (y as f64 + 0.5)) * cam.scale;
                        let o = cam.center + cam.right * u + cam.up * vv - cam.dir * (cam.depth * 0.5);
                        let near: Vec<usize> =
                            world.bodies.iter().enumerate().filter(|(_, bd)| ray_hits_aabb(&bd.aabb.grow(0.5), o, cam.dir)).map(|(i, _)| i).collect();
                        let hit = if near.is_empty() { None } else { trace(world, &near, o, cam.dir, cam.depth, view.section) };
                        *out = match hit {
                            Some(hit) => {
                                let base = world.bodies.get(hit.body).map(|b| b.color).unwrap_or([200, 200, 200]);
                                let lam = hit.n.dot(light).max(0.0);
                                let rim = (1.0 - (-hit.n.dot(cam.dir)).clamp(0.0, 1.0)).powi(2) * 0.25;
                                let k = 0.32 + 0.68 * lam - rim * 0.3;
                                [shade(base[0], k), shade(base[1], k), shade(base[2], k)]
                            }
                            None => background(cam, o, ground_z, view),
                        };
                    }
                }
            });
        }
    });
    // Dark outlines where the visible surface or its depth changes sharply.
    let snapshot = px.clone();
    for y in 1..h.saturating_sub(1) {
        for x in 1..w.saturating_sub(1) {
            let c = lum(snapshot.get(y * w + x));
            let e = (c - lum(snapshot.get(y * w + x + 1))).abs().max((c - lum(snapshot.get((y + 1) * w + x))).abs());
            if e > 38.0 {
                if let Some(p) = px.get_mut(y * w + x) {
                    *p = [p[0] / 2 + 8, p[1] / 2 + 8, p[2] / 2 + 8];
                }
            }
        }
    }
    for m in markers {
        let (cx, cy) = project(&cam, m.at);
        ring(&mut px, w, h, cx, cy, 10.0, [235, 40, 40]);
        digits(&mut px, w, h, cx + 12.0, cy - 12.0, m.number);
    }
    let mut raw = Vec::with_capacity(w * h * 3);
    for p in &px {
        raw.extend_from_slice(p);
    }
    let mut out = Vec::new();
    if let Some(img) = image::RgbImage::from_raw(w as u32, h as u32, raw) {
        let _ = img.write_to(&mut std::io::Cursor::new(&mut out), image::ImageFormat::Png);
    }
    out
}

fn shade(c: u8, k: f64) -> u8 {
    (c as f64 * k).clamp(0.0, 255.0) as u8
}

fn lum(p: Option<&[u8; 3]>) -> f64 {
    p.map(|p| 0.3 * p[0] as f64 + 0.59 * p[1] as f64 + 0.11 * p[2] as f64).unwrap_or(0.0)
}

fn background(cam: &Cam, o: V3, ground_z: Option<f64>, view: &View) -> [u8; 3] {
    if let Some(gz) = ground_z
        && cam.dir.z.abs() > 0.2
        && view.section.is_none()
    {
        let t = (gz - o.z) / cam.dir.z;
        if t > 0.0 {
            let p = o + cam.dir * t;
            let gx = (p.x / 10.0).rem_euclid(1.0);
            let gy = (p.y / 10.0).rem_euclid(1.0);
            let line = gx < 0.04 || gy < 0.04;
            let fade = (1.0 - ((p.x * p.x + p.y * p.y).sqrt() / 220.0)).clamp(0.0, 1.0);
            let base = if line { 150.0 } else { 188.0 };
            let v = (base * (0.7 + 0.3 * fade)) as u8;
            return [v, v, (v as f64 * 1.03).min(255.0) as u8];
        }
    }
    let g = 222u8;
    [g, g, 226]
}

fn ring(px: &mut [[u8; 3]], w: usize, h: usize, cx: f64, cy: f64, r: f64, c: [u8; 3]) {
    let n = (r * 14.0) as usize;
    for i in 0..n {
        let a = i as f64 / n as f64 * std::f64::consts::TAU;
        for dr in [-1.0, 0.0, 1.0] {
            let (x, y) = ((cx + (r + dr) * a.cos()).round() as i64, (cy + (r + dr) * a.sin()).round() as i64);
            if x >= 0
                && y >= 0
                && (x as usize) < w
                && (y as usize) < h
                && let Some(p) = px.get_mut(y as usize * w + x as usize)
            {
                *p = c;
            }
        }
    }
}

const FONT: [[u8; 5]; 10] = [
    [0b111, 0b101, 0b101, 0b101, 0b111],
    [0b010, 0b110, 0b010, 0b010, 0b111],
    [0b111, 0b001, 0b111, 0b100, 0b111],
    [0b111, 0b001, 0b111, 0b001, 0b111],
    [0b101, 0b101, 0b111, 0b001, 0b001],
    [0b111, 0b100, 0b111, 0b001, 0b111],
    [0b111, 0b100, 0b111, 0b101, 0b111],
    [0b111, 0b001, 0b010, 0b010, 0b010],
    [0b111, 0b101, 0b111, 0b101, 0b111],
    [0b111, 0b101, 0b111, 0b001, 0b111],
];

fn digits(px: &mut [[u8; 3]], w: usize, h: usize, x0: f64, y0: f64, n: usize) {
    let s = n.to_string();
    let scale = 3i64;
    for (ci, ch) in s.chars().enumerate() {
        let Some(d) = ch.to_digit(10) else { continue };
        let Some(glyph) = FONT.get(d as usize) else { continue };
        for (ry, row) in glyph.iter().enumerate() {
            for rx in 0..3 {
                if row >> (2 - rx) & 1 == 1 {
                    for sy in 0..scale {
                        for sx in 0..scale {
                            let x = x0 as i64 + (ci as i64 * 4 + rx as i64) * scale + sx;
                            let y = y0 as i64 + ry as i64 * scale + sy;
                            if x >= 0
                                && y >= 0
                                && (x as usize) < w
                                && (y as usize) < h
                                && let Some(p) = px.get_mut(y as usize * w + x as usize)
                            {
                                *p = [235, 40, 40];
                            }
                        }
                    }
                }
            }
        }
    }
}
