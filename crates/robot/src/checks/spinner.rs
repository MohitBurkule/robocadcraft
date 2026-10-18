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

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let vertical = matches!(r.spec.robot_type, RobotType::VerticalSpinner | RobotType::Drum);
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

        // Bearing support: a shaft or bearing must carry the weapon and end in structure on both sides.
        let side_support = |sign: f64| -> bool {
            let mut shaft = false;
            let mut hold = false;
            let mut d = half + 0.5;
            while d < half + 40.0 {
                let p = origin + axis * (sign * d);
                for (j, b) in w.bodies.iter().enumerate() {
                    if j == i || b.fastener || !b.aabb.contains(p) {
                        continue;
                    }
                    if w.sdf(j, p) < 0.0 {
                        if matches!(b.kind, PartKind::Shaft | PartKind::Bearing) {
                            shaft = true;
                        } else if is_support(b.kind) {
                            hold = true;
                        }
                    }
                }
                d += 0.5;
            }
            shaft && hold
        };
        let (a, b) = (side_support(1.0), side_support(-1.0));
        if a && b {
            out.push(Finding::pass("spinner.bearing", G, "Weapon supported on both sides", format!("`{id}` sits on a shaft or bearing held in structure on both sides.")));
        } else {
            let sev = if vertical || wd.supported_both_ends { Severity::Blocker } else { Severity::Warning };
            let why = if wd.supported_both_ends { "`supported_both_ends` is declared but no shaft or bearing reaches structure on both sides" } else { "it hangs off one motor bearing" };
            out.push(
                Finding::fail(
                    "spinner.bearing",
                    G,
                    sev,
                    "Weapon is cantilevered",
                    format!("`{id}`: {why}. The side load of a hit goes straight into a hobby motor bearing and bends the shaft."),
                )
                .on(&[id])
                .fix("Run a steel shaft (a `shaft` part) through the weapon and into both forks with bearings or bushings, and drive the weapon by belt or a hub motor clamped on that shaft."),
            );
        }

        // Reach: the weapon must hit before the wedge or chassis does.
        let reach = pts.iter().map(|p| p.y).fold(f64::MIN, f64::max);
        let other = w
            .bodies
            .iter()
            .enumerate()
            .filter(|(j, b)| *j != i && !b.fastener && matches!(b.kind, PartKind::Wedge | PartKind::Structure | PartKind::Plate | PartKind::Armor | PartKind::Fork))
            .map(|(_, b)| b.aabb.max.y)
            .fold(f64::MIN, f64::max);
        ctx.metrics.insert("weapon_reach_lead_mm".into(), reach - other);
        if vertical && reach < other + 3.0 {
            out.push(
                Finding::fail(
                    "spinner.reach",
                    G,
                    Severity::Warning,
                    "Weapon does not lead the front",
                    format!("`{id}` reaches y={reach:.1} mm but the front of the chassis or wedge reaches y={other:.1} mm; the opponent meets the wedge before the weapon."),
                )
                .on(&[id])
                .fix("Cut the wedge back or move the weapon forward until the weapon leads by 5-10 mm."),
            );
        }

        // Share of the robot's mass in the weapon.
        let wm = ctx.metrics.get(&format!("part_mass_g.{id}")).copied().unwrap_or(0.0);
        let total = ctx.metrics.get("mass_total_g").copied().unwrap_or(150.0).max(1.0);
        ctx.metrics.insert("weapon_mass_g".into(), wm);
        ctx.metrics.insert("weapon_mass_fraction".into(), wm / total);
        if wm / total < 0.15 {
            out.push(
                Finding::fail(
                    "spinner.mass_share",
                    G,
                    Severity::Warning,
                    "Weapon is a small share of the robot",
                    format!("`{id}` is {wm:.1} g, {:.0}% of {total:.0} g; competitive spinners put 25-40% of the mass into the weapon.", 100.0 * wm / total),
                )
                .on(&[id])
                .fix("Make the weapon larger or thicker and take the mass from armour that does not carry load."),
            );
        }
        if let Some(e) = ctx.metrics.get("weapon_energy_j").copied() {
            if e < 15.0 {
                out.push(
                    Finding::fail(
                        "spinner.energy_low",
                        G,
                        Severity::Warning,
                        "Weapon stores little energy",
                        format!("`{id}` stores {e:.1} J at 85% speed; a hit that transfers a fraction of that will not move or damage an antweight."),
                    )
                    .on(&[id])
                    .fix("Raise the weapon's moment of inertia (larger radius, more mass at the rim) before raising rpm."),
                );
            }
        }

        // Invertibility: a vertical weapon should sit at the body mid-plane so it works either way up.
        if vertical {
            let lo = w.bodies.iter().enumerate().filter(|(j, bd)| *j != i && matches!(bd.kind, PartKind::Wheel)).map(|(_, bd)| bd.aabb.min.z).fold(f64::MAX, f64::min);
            let hi = w.bodies.iter().enumerate().filter(|(j, bd)| *j != i && !bd.fastener && !bd.id.starts_with("wire:")).map(|(_, bd)| bd.aabb.max.z).fold(f64::MIN, f64::max);
            if lo < f64::MAX && hi > f64::MIN {
                let mid = 0.5 * (lo + hi);
                if (origin.z - mid).abs() > 3.0 && r.spec.can_run_inverted {
                    out.push(
                        Finding::fail(
                            "spinner.invert",
                            G,
                            Severity::Warning,
                            "Weapon axis is off the mid-plane",
                            format!("`{id}` axis is at z={:.1} mm, the mid-plane is z={mid:.1} mm; flipped over it works at a different height.", origin.z),
                        )
                        .on(&[id]),
                    );
                }
            }
            if !r.spec.can_run_inverted {
                out.push(
                    Finding::fail(
                        "spinner.invert",
                        G,
                        Severity::Warning,
                        "Robot cannot run upside down",
                        "A vertical spinner gets flipped often; if it cannot drive inverted it is out of the fight the first time that happens.",
                    )
                    .fix("Make the body symmetric about the wheel axle plane, or add a self-righting arm."),
                );
            }
        }
    }
    out.extend(exposure(ctx));
    out
}

/// Electronics and motors whose side faces see open air: nothing armours them from a hit.
fn exposure(ctx: &Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let dirs = [v3(1.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, -1.0, 0.0), v3(0.0, 0.0, 1.0)];
    for (i, b) in w.bodies.iter().enumerate() {
        if b.fastener || !matches!(b.kind, PartKind::WeaponMotor | PartKind::Esc | PartKind::Battery | PartKind::Receiver) {
            continue;
        }
        let c = b.aabb.center();
        let mut open: Vec<&str> = Vec::new();
        for (k, d) in dirs.iter().enumerate() {
            let size = b.aabb.size();
            let reach = 0.5 * [size.x, size.x, size.y, size.y, size.z][k];
            let mut hit = false;
            let mut t = reach + 0.2;
            while t < reach + 60.0 {
                let p: V3 = c + *d * t + if k < 4 { v3(0.0, 0.0, 1.0) } else { v3(0.0, 0.0, 0.0) };
                if w.scene_sdf(p, &|j| j != i && !w.bodies[j].fastener) < 0.0 {
                    hit = true;
                    break;
                }
                t += 1.0;
            }
            if !hit {
                open.push(["+x", "-x", "+y", "-y", "+z"][k]);
            }
        }
        // Open to the sides with nothing in the way: exposed. Open only to the top is the normal lid case.
        let side_open = open.iter().filter(|s| !s.ends_with('z')).count();
        if side_open > 0 && r.parts.iter().any(|rp| rp.part.id == b.id) {
            out.push(
                Finding::fail(
                    "spinner.exposed",
                    G,
                    Severity::Warning,
                    "Part sits outside the armour",
                    format!("`{}` has a clear line of sight to open air on {}; a weapon hit or the floor reaches it directly.", b.id, open.join(", ")),
                )
                .on(&[&b.id])
                .here(c)
                .fix("Move it inside the shell or add an armour wall (min 1.2 mm nylon, or 0.8 mm steel/titanium) on that side."),
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
