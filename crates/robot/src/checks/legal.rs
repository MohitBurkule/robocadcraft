//! Rules of the event: size, switch, failsafe, weapon controls.

use crate::catalog;
use crate::finding::{Finding, Severity};
use crate::math::v3;
use crate::measure;
use crate::spec::PartKind;

use super::Ctx;

const G: &str = "legal";

/// Smallest square, in any rotation about the vertical axis, that holds every solid body (bounds corners, so slightly conservative).
pub fn footprint_mm(ctx: &Ctx) -> f64 {
    let pts: Vec<_> = ctx.world.bodies.iter().filter(|b| !b.id.starts_with("wire:")).flat_map(|b| b.aabb.corners()).collect();
    let mut best = f64::MAX;
    for deg in 0..90 {
        let a = (deg as f64).to_radians();
        let (s, c) = a.sin_cos();
        let (mut lo, mut hi) = (v3(f64::MAX, f64::MAX, 0.0), v3(f64::MIN, f64::MIN, 0.0));
        for p in &pts {
            let (x, y) = (p.x * c + p.y * s, -p.x * s + p.y * c);
            lo = v3(lo.x.min(x), lo.y.min(y), 0.0);
            hi = v3(hi.x.max(x), hi.y.max(y), 0.0);
        }
        best = best.min((hi.x - lo.x).max(hi.y - lo.y));
    }
    best
}

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let mut out = Vec::new();
    out.extend(controls(ctx));
    out.extend(weapons(ctx));
    let limit = ctx.r.profile.footprint_mm;
    let fp = footprint_mm(ctx);
    ctx.metrics.insert("footprint_mm".into(), fp);
    if fp > limit {
        out.push(
            Finding::fail(
                "legal.footprint",
                G,
                Severity::Blocker,
                "Robot does not fit the square",
                format!("Smallest square that holds the robot is {fp:.1} mm; the limit is {limit:.0} mm with no overhang at any height."),
            )
            .expect(format!("<= {limit:.0} mm"), format!("{fp:.1} mm"))
            .fix("Shorten the wedge, narrow the wheels or move parts inward."),
        );
    } else {
        out.push(Finding::pass("legal.footprint", G, "Fits the square", format!("Fits a {fp:.1} mm square (limit {limit:.0} mm).")));
    }
    out
}

fn colour_dist(a: [u8; 3], b: [u8; 3]) -> f64 {
    let d = |x: u8, y: u8| (x as f64 - y as f64).powi(2);
    (d(a[0], b[0]) + d(a[1], b[1]) + d(a[2], b[2])).sqrt()
}

/// Power switch, failsafe, weapon disable and sharp-edge covers.
fn controls(ctx: &Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let mut out = Vec::new();
    let switches: Vec<_> = r.spec.parts.iter().filter(|p| p.kind == PartKind::Switch).collect();
    if r.profile.require_tool_free_switch {
        if let Some(s) = switches.iter().find(|s| s.tool_free && s.marked) {
            out.push(Finding::pass("legal.switch", G, "Marked tool-free power switch", format!("`{}` is a marked, tool-free power switch.", s.id)));
        } else {
            out.push(
                Finding::fail(
                    "legal.switch",
                    G,
                    Severity::Blocker,
                    "No tool-free marked power switch",
                    "Rule 4a needs an externally accessible way to shut down that works without tools and is clearly marked on/off.",
                )
                .fix("Add a `switch` part (or removable link) with `tool_free: true` and `marked: true` and a visible ON/OFF `label`."),
            );
        }
    }
    if r.spec.electronics.failsafe {
        out.push(Finding::pass("legal.failsafe", G, "Failsafe declared", "The robot stops when the transmitter is switched off."));
    } else {
        out.push(
            Finding::fail(
                "legal.failsafe",
                G,
                Severity::Blocker,
                "Failsafe not declared",
                "Rule 4d: the robot must stop all motion when the transmitter is off.",
            )
            .fix("Set `electronics.failsafe` to true and configure the receiver and ESC failsafe."),
        );
    }
    let armed =
        r.spec.robot_type.is_spinner() || r.spec.parts.iter().any(|p| p.weapon.as_ref().is_some_and(|w| w.kind != "none" && w.kind != "wedge"));
    if armed {
        if r.spec.electronics.weapon_remote_disable {
            out.push(Finding::pass("legal.weapon_disable", G, "Weapon can be disabled remotely", "The weapon is switched off from the transmitter."));
        } else {
            out.push(
                Finding::fail(
                    "legal.weapon_disable",
                    G,
                    Severity::Blocker,
                    "Weapon cannot be disabled remotely",
                    "Rule 4c: all weaponry must be deactivatable by remote control.",
                )
                .fix("Set `electronics.weapon_remote_disable` and put the weapon on its own receiver channel."),
            );
        }
    }
    for p in r.spec.parts.iter().filter(|p| p.sharp) {
        if p.cover.is_none() {
            out.push(
                Finding::fail(
                    "legal.sharp_cover",
                    G,
                    Severity::Blocker,
                    "Sharp edge has no cover",
                    format!("`{}` is marked sharp but has no protective cover.", p.id),
                )
                .on(&[&p.id])
                .fix("Name the cover in `cover` (a clearly visible guard fitted outside the arena)."),
            );
        }
    }
    out
}

/// Weapon rules: material, locking bar, tip speed and stored energy.
fn weapons(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    for rp in r.parts.iter().filter(|rp| rp.part.kind == PartKind::Weapon) {
        let p = &rp.part;
        let Some(wd) = &p.weapon else {
            out.push(
                Finding::fail(
                    "legal.weapon_def",
                    G,
                    Severity::Blocker,
                    "Weapon has no definition",
                    format!("`{}` is a weapon but has no `weapon` block (kind, motor, reduction).", p.id),
                )
                .on(&[&p.id]),
            );
            continue;
        };
        if let Some(m) = p.material.as_deref().and_then(catalog::material) {
            if m.brittle {
                out.push(
                    Finding::fail(
                        "legal.weapon_brittle",
                        G,
                        Severity::Blocker,
                        "Weapon material shatters",
                        format!(
                            "`{}` is {} which is brittle; a weapon that shatters sends fragments into the arena and is not allowed.",
                            p.id,
                            p.material.as_deref().unwrap_or("?")
                        ),
                    )
                    .on(&[&p.id])
                    .fix("Use steel, titanium, hardened tool steel or AR400 for a spinner."),
                );
            }
            if matches!(m.class, catalog::MatClass::Printed | catalog::MatClass::Elastomer | catalog::MatClass::Tape)
                && matches!(wd.kind.as_str(), "spinner" | "drum" | "disc" | "bar")
            {
                out.push(
                    Finding::fail(
                        "legal.weapon_soft",
                        G,
                        Severity::Warning,
                        "Weapon is a soft plastic",
                        format!("`{}` is a printed or elastomer spinner; it will deform in the first hit and carry almost no energy.", p.id),
                    )
                    .on(&[&p.id]),
                );
            }
        }
        let spins = matches!(wd.kind.as_str(), "spinner" | "drum" | "disc" | "bar");
        if r.profile.require_locking_bar && (spins || wd.kind == "flipper" || wd.kind == "lifter") && spins {
            let lb = wd.locking_bar.as_deref().and_then(|id| r.spec.part(id));
            match lb {
                None => out.push(
                    Finding::fail("legal.locking_bar", G, Severity::Blocker, "No locking bar", "Rule 5f: every spinner needs a locking bar that physically stops the weapon from turning.")
                        .on(&[&p.id])
                        .fix("Add a `locking_bar` part (a pin through the weapon into the frame), name it in `weapon.locking_bar`, give it a contrasting colour and `tethered: true`."),
                ),
                Some(bar) => {
                    let chassis = w.bodies.iter().find(|b| b.part_idx.and_then(|i| r.parts.get(i)).is_some_and(|x| x.part.kind == PartKind::Structure)).map(|b| b.color).unwrap_or([60, 60, 60]);
                    let barc = w.body(&bar.id).map(|b| b.color).unwrap_or([0, 0, 0]);
                    if colour_dist(chassis, barc) < 140.0 {
                        out.push(Finding::fail("legal.locking_bar_colour", G, Severity::Warning, "Locking bar is hard to see", format!("`{}` is too close in colour to the chassis; the rule wants a contrasting colour.", bar.id)).on(&[&bar.id]).fix("Colour it bright red or orange against a dark chassis."));
                    }
                    if !bar.tethered {
                        out.push(Finding::fail("legal.locking_bar_tether", G, Severity::Warning, "Locking bar can be lost", format!("`{}` is not tethered; it must be fixed so it cannot be accidentally removed or lost.", bar.id)).on(&[&bar.id]).fix("Set `tethered: true` and describe the lanyard in a connection."));
                    }
                    let i = w.index(&p.id);
                    let j = w.index(&bar.id);
                    let blocks = match (i, j) {
                        (Some(i), Some(j)) => measure::overlap(w, i, j, 0.5).is_some_and(|o| o.depth > 0.05) || measure::min_gap(w, i, j, 0.4) < 0.3,
                        _ => false,
                    };
                    if !blocks {
                        out.push(Finding::fail("legal.locking_bar_blocks", G, Severity::Blocker, "Locking bar does not touch the weapon", format!("`{}` does not reach `{}`, so it cannot hold it still.", bar.id, p.id)).on(&[&bar.id, &p.id]).fix("Move the pin so it passes through a hole in the weapon or sits against a tooth."));
                    }
                }
            }
        }
    }
    out
}
