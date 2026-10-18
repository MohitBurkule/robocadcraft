//! Checks every spinning weapon must pass: bearing support, reach, exposure, invertibility, mass share.
//! Anything the verifier cannot judge is listed in `not_checked` so a clean report is never read as "safe".

use crate::finding::{Finding, Severity};
use crate::math::{V3, v3};
use crate::measure;
use crate::spec::{PartKind, RobotType};
use crate::world::Resolved;

use super::interference::spin_axis;
use super::{Ctx, Options};

const G: &str = "spinner";

fn is_support(k: PartKind) -> bool {
    matches!(k, PartKind::Structure | PartKind::Plate | PartKind::Fork | PartKind::Armor)
}

/// Millimetres of shaft that sit inside a support body on one side of the weapon (0.25 mm steps).
fn hold_mm(ctx: &Ctx, shaft: usize, weapon: usize, origin: V3, axis: V3, half: f64, sign: f64) -> f64 {
    let w = ctx.world;
    let mut len = 0.0;
    let mut d = half;
    while d < half + 40.0 {
        let p = origin + axis * (sign * d);
        if w.sdf(shaft, p) < 0.0 {
            let held = w.bodies.iter().enumerate().any(|(j, b)| j != weapon && j != shaft && !b.fastener && (is_support(b.kind) || b.kind == PartKind::Bearing) && b.aabb.contains(p) && w.sdf(j, p) < 0.0);
            if held {
                len += 0.25;
            }
        } else if len > 0.0 {
            break;
        }
        d += 0.25;
    }
    len
}

const MIN_HOLD_MM: f64 = 3.0;

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let vertical = matches!(r.spec.robot_type, RobotType::VerticalSpinner | RobotType::Drum);
    let total = ctx.metrics.get("mass_total_g").copied().unwrap_or(150.0).max(1.0);
    for rp in r.parts.iter().filter(|rp| rp.part.kind == PartKind::Weapon) {
        let Some(wd) = &rp.part.weapon else { continue };
        if !matches!(wd.kind.as_str(), "spinner" | "drum" | "disc" | "bar") {
            continue;
        }
        let id = rp.part.id.as_str();
        let Some(i) = w.index(id) else { continue };
        let Some((origin, axis)) = spin_axis(w, r, i) else { continue };
        let pts = measure::surface_points(w, i, 1.0);
        let half = pts.iter().map(|p| (*p - origin).dot(axis).abs()).fold(0.0_f64, f64::max);
        let energy = ctx.metrics.get("weapon_energy_j").copied();

        // Bearing support: one shaft goes through the weapon centre and is gripped by structure on both sides.
        let mut best = (0.0_f64, 0.0_f64, String::new());
        for (j, b) in w.bodies.iter().enumerate() {
            if b.fastener || !matches!(b.kind, PartKind::Shaft) || w.sdf(j, origin) >= 0.0 {
                continue;
            }
            let (a, c) = (hold_mm(ctx, j, i, origin, axis, half, 1.0), hold_mm(ctx, j, i, origin, axis, half, -1.0));
            if a.min(c) >= best.0.min(best.1) {
                best = (a, c, b.id.clone());
            }
        }
        if best.0 >= MIN_HOLD_MM && best.1 >= MIN_HOLD_MM {
            ctx.metrics.insert("weapon_shaft_hold_mm".into(), best.0.min(best.1));
            out.push(Finding::pass(
                "spinner.bearing",
                G,
                "Weapon on a shaft gripped on both sides",
                format!("`{id}` turns on `{}`, which sits {:.1} mm and {:.1} mm in structure or bearings either side (need {MIN_HOLD_MM:.0}).", best.2, best.0, best.1),
            ));
        } else {
            let escalate = vertical || wd.supported_both_ends || energy.is_some_and(|e| e > 20.0);
            let why = if best.2.is_empty() {
                "no shaft passes through the weapon centre".to_string()
            } else {
                format!("shaft `{}` is held only {:.1} mm on one side and {:.1} mm on the other (need {MIN_HOLD_MM:.0} mm each)", best.2, best.0, best.1)
            };
            out.push(
                Finding::fail(
                    "spinner.bearing",
                    G,
                    if escalate { Severity::Blocker } else { Severity::Warning },
                    "Weapon is cantilevered or loosely held",
                    format!("`{id}`: {why}. A hit's side load goes into a hobby motor bearing or a loose shaft."),
                )
                .on(&[id])
                .fix("Run one `shaft` part through the weapon bore and into a hole or bearing seat in structure on both sides (3 mm or more each), forks thick enough to stay stiff (5-6 mm)."),
            );
        }

        // Reach within 30 degrees of straight ahead: the weapon must meet the opponent before the wedge does.
        let others: Vec<usize> = w
            .bodies
            .iter()
            .enumerate()
            .filter(|(j, b)| *j != i && !b.fastener && matches!(b.kind, PartKind::Wedge | PartKind::Structure | PartKind::Plate | PartKind::Armor | PartKind::Fork))
            .map(|(j, _)| j)
            .collect();
        let mut worst = f64::MAX;
        let mut worst_deg = 0.0;
        for deg in (-30..=30).step_by(10) {
            let a = (deg as f64).to_radians();
            let dir = v3(a.sin(), a.cos(), 0.0);
            let wr = pts.iter().map(|p| p.dot(dir)).fold(f64::MIN, f64::max);
            let or = others.iter().filter_map(|&j| w.bodies.get(j)).flat_map(|b| b.aabb.corners()).map(|p| p.dot(dir)).fold(f64::MIN, f64::max);
            if wr - or < worst {
                worst = wr - or;
                worst_deg = deg as f64;
            }
        }
        ctx.metrics.insert("weapon_reach_lead_mm".into(), worst);
        if vertical && worst < 3.0 {
            out.push(
                Finding::fail(
                    "spinner.reach",
                    G,
                    Severity::Warning,
                    "Weapon does not lead the front",
                    format!("Looking {worst_deg:.0} degrees off straight ahead the chassis or wedge is {:.1} mm ahead of `{id}`; the opponent meets it first.", -worst),
                )
                .on(&[id])
                .fix("Cut the wedge back or move the weapon forward until it leads by 5-10 mm across plus or minus 30 degrees."),
            );
        }

        // Mass share and stored energy, scaled to the robot's mass.
        let wm = ctx.metrics.get(&format!("part_mass_g.{id}")).copied().unwrap_or(0.0);
        ctx.metrics.insert("weapon_mass_g".into(), wm);
        ctx.metrics.insert("weapon_mass_fraction".into(), wm / total);
        if wm / total < 0.20 {
            out.push(
                Finding::fail(
                    "spinner.mass_share",
                    G,
                    Severity::Warning,
                    "Weapon is a small share of the robot",
                    format!("`{id}` is {wm:.1} g, {:.0}% of {total:.0} g; spinners that win put 25-40% of the mass into the weapon (warns below 20%).", 100.0 * wm / total),
                )
                .on(&[id]),
            );
        }
        if let Some(e) = energy {
            let per_g = e / total;
            ctx.metrics.insert("weapon_energy_per_robot_g".into(), per_g);
            // 150 g robot: warn below 15 J, note below 30 J.
            if per_g < 0.10 {
                out.push(
                    Finding::fail("spinner.energy_low", G, Severity::Warning, "Weapon stores little energy", format!("`{id}` stores {e:.1} J at 85% speed ({per_g:.2} J per gram of robot; warns below 0.10, i.e. 15 J at 150 g)."))
                        .on(&[id])
                        .fix("Raise the moment of inertia (larger radius, mass at the rim) before raising rpm."),
                );
            } else if per_g < 0.20 {
                out.push(Finding::note("spinner.energy_modest", G, "Weapon energy is modest", format!("`{id}` stores {e:.1} J; top antweight spinners carry 30 J or more.")).on(&[id]));
            }
        }

        // Invertibility: the weapon must reach the floor plane whichever way up the robot lands.
        let all: Vec<&crate::world::Body> = w.bodies.iter().enumerate().filter(|(j, b)| *j != i && !b.fastener && !b.id.starts_with("wire:") && b.kind != PartKind::Wheel).map(|(_, b)| b).collect();
        let bot = w.bodies.iter().filter(|b| !b.fastener).map(|b| b.aabb.min.z).fold(f64::MAX, f64::min);
        let top = all.iter().map(|b| b.aabb.max.z).fold(f64::MIN, f64::max).max(w.bodies.iter().filter(|b| b.kind == PartKind::Wheel).map(|b| b.aabb.max.z).fold(f64::MIN, f64::max));
        let (wlo, whi) = pts.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.z), a.1.max(p.z)));
        if vertical && !pts.is_empty() {
            let upright = wlo - bot;
            let flipped = top - whi;
            ctx.metrics.insert("weapon_floor_gap_upright_mm".into(), upright);
            ctx.metrics.insert("weapon_floor_gap_inverted_mm".into(), flipped);
            if upright > 2.0 || (r.spec.can_run_inverted && flipped > 2.0) {
                out.push(
                    Finding::fail(
                        "spinner.invert",
                        G,
                        Severity::Warning,
                        "Weapon does not reach the floor both ways up",
                        format!("`{id}` is {upright:.1} mm above the floor upright and {flipped:.1} mm below the floor plane when inverted; it must be within 2 mm either way."),
                    )
                    .on(&[id])
                    .fix("Centre the weapon on the body mid-plane and size it so its rim just touches the floor plane either way up."),
                );
            }
            if !r.spec.can_run_inverted {
                out.push(Finding::fail(
                    "spinner.invert",
                    G,
                    Severity::Warning,
                    "Robot cannot run upside down",
                    "A vertical spinner gets flipped often; if it cannot drive inverted it is out of the fight the first time that happens.",
                ));
            }
        }
        out.extend(sweep_near(ctx, i, id, origin, axis, half, &pts));
    }
    out.extend(exposure(ctx));
    out
}

fn rim(pts: &[V3], origin: V3, axis: V3) -> f64 {
    pts.iter().map(|p| {
        let q = *p - origin;
        (q - axis * q.dot(axis)).len()
    })
    .fold(0.0_f64, f64::max)
}

/// Electronics, motors and wires within 3 mm of the swept volume of the weapon.
fn sweep_near(ctx: &Ctx, weapon: usize, id: &str, origin: V3, axis: V3, half: f64, wpts: &[V3]) -> Vec<Finding> {
    let w = ctx.world;
    let rmax = rim(wpts, origin, axis);
    let inside = |p: V3| {
        let q = p - origin;
        q.dot(axis).abs() <= half + 3.0 && (q - axis * q.dot(axis)).len() <= rmax + 3.0
    };
    let mut out = Vec::new();
    for (j, b) in w.bodies.iter().enumerate() {
        if j == weapon || b.fastener || !matches!(b.kind, PartKind::Esc | PartKind::Battery | PartKind::Receiver | PartKind::Servo | PartKind::Switch | PartKind::Motor) {
            continue;
        }
        let ps = measure::surface_points(w, j, 2.0);
        if let Some(p) = ps.iter().find(|p| inside(**p)) {
            out.push(Finding::fail("spinner.sweep_near", G, Severity::Warning, "Part inside the weapon's reach", format!("`{}` comes within 3 mm of the swept volume of `{id}`.", b.id)).on(&[&b.id, id]).here(*p).fix("Move it away from the disc or put an armour wall between."));
        }
    }
    for rt in &ctx.routes {
        if let Some(p) = rt.points.iter().map(|p| V3::from_arr(*p)).find(|p| inside(*p)) {
            out.push(Finding::fail("spinner.sweep_near", G, Severity::Warning, "Wire inside the weapon's reach", format!("Wire `{}` passes within 3 mm of the swept volume of `{id}`; one hit cuts it.", rt.id)).on(&[&rt.id, id]).here(p).fix("Route it inside the chassis, away from the disc."));
        }
    }
    out
}

/// 26 directions: faces, edges and corners of the unit cube.
fn dirs26() -> Vec<V3> {
    let mut v = Vec::new();
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                if x != 0 || y != 0 || z != 0 {
                    v.push(v3(x as f64, y as f64, z as f64).norm());
                }
            }
        }
    }
    v
}

fn open_air(w: &crate::world::World, skip: &dyn Fn(usize) -> bool, p: V3, d: V3) -> bool {
    let bounds = w.bounds().grow(1.0);
    let mut t = 0.0;
    while t < 400.0 {
        let q = p + d * t;
        if !bounds.contains(q) {
            return true;
        }
        if w.scene_sdf(q, &|j| !skip(j)) < 0.3 {
            return false;
        }
        t += 1.0;
    }
    true
}

/// Parts and wires that see open air from any surface point in any of 26 directions are not armoured.
fn exposure(ctx: &Ctx) -> Vec<Finding> {
    let w = ctx.world;
    let mut out = Vec::new();
    let dirs = dirs26();
    for (i, b) in w.bodies.iter().enumerate() {
        if b.fastener || !matches!(b.kind, PartKind::WeaponMotor | PartKind::Esc | PartKind::Battery | PartKind::Receiver) {
            continue;
        }
        let mut pts = measure::surface_points(w, i, 2.5);
        let stride = (pts.len() / 24).max(1);
        pts = pts.into_iter().step_by(stride).collect();
        let (mut open, mut total) = (0usize, 0usize);
        let mut sample = None;
        for p in &pts {
            for d in &dirs {
                let start = *p + *d * 0.6;
                // Only rays that leave the part itself.
                if w.sdf(i, start) < 0.0 {
                    continue;
                }
                total += 1;
                if open_air(w, &|j| j == i, start, *d) {
                    open += 1;
                    sample.get_or_insert(*p);
                }
            }
        }
        let frac = if total > 0 { open as f64 / total as f64 } else { 0.0 };
        if frac > 0.02 {
            let mut f = Finding::fail(
                "spinner.exposed",
                G,
                Severity::Warning,
                "Part sits outside the armour",
                format!("`{}` has a clear path to open air from {:.0}% of its outward rays; a hit or the floor reaches it directly.", b.id, frac * 100.0),
            )
            .on(&[&b.id])
            .fix("Move it inside the shell or add an armour wall (1.2 mm nylon, or 0.8 mm steel or titanium) on the open side.");
            if let Some(p) = sample {
                f = f.here(p);
            }
            out.push(f);
        }
    }
    // Wires: each route sample must be walled in on the sides.
    for rt in &ctx.routes {
        let samples: Vec<V3> = rt.points.iter().step_by(2).map(|p| V3::from_arr(*p)).collect();
        let mut bad = None;
        for p in &samples {
            if let Some(d) = dirs.iter().filter(|d| d.z.abs() < 0.9).find(|d| open_air(w, &|_| false, *p + **d * 0.8, **d)) {
                if std::env::var("ROBOT_TIMING").is_ok() {
                    eprintln!("wire {} open at {:?} dir {:?}", rt.id, p, d);
                }
                bad = Some(*p);
                break;
            }
        }
        if let Some(p) = bad {
            out.push(
                Finding::fail("spinner.wire_exposed", G, Severity::Warning, "Wire runs outside the armour", format!("Wire `{}` has a clear line to open air at this point; a hit or the weapon can cut it.", rt.id))
                    .on(&[&rt.id])
                    .here(p)
                    .fix("Route it through the chassis interior or add a wall on the open side."),
            );
        }
    }
    out
}

/// What this verifier does not judge for this robot, so a pass is not read as proof.
pub fn not_checked(r: &Resolved, opts: &Options) -> Vec<String> {
    let mut v: Vec<String> = vec![
        "Strength: arm, fork and shell bending under impact (no stress or stiffness model; load path is not traced)".into(),
        "Battery access and field service: swap time, number of fasteners to reach the battery or change a wheel".into(),
        "Wire tether in assembly order: soldered wires between parts are not part of the disassembly sweep".into(),
        "Heat-set insert iron clearance and soldering-iron access".into(),
        "Gyro, spin-up time and ESC current limits of the weapon".into(),
        "Thermal limits and glue or tape strength (declared only)".into(),
    ];
    if !opts.deep {
        v.push("Wire routing, wire mass and the assembly/disassembly sweep (fast mode skipped them)".into());
    }
    if r.spec.robot_type.is_spinner() {
        v.push("Spinner balance and vibration at speed (mass centre of the weapon is not compared with its axis)".into());
    }
    v
}
