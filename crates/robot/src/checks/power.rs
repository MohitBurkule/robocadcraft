//! Electrical and weapon numbers: tip speed, stored energy, ESC and battery limits.

use crate::finding::{Finding, Severity};
use crate::math::v3;
use crate::measure;
use crate::spec::PartKind;

use super::Ctx;
use super::interference::spin_axis;

const G: &str = "power";

pub fn run(ctx: &mut Ctx) -> Vec<Finding> {
    let r = ctx.r;
    let w = ctx.world;
    let mut out = Vec::new();
    let cells = r
        .spec
        .electronics
        .cells
        .or_else(|| r.parts.iter().find_map(|p| p.cat.as_ref().filter(|c| c.kind == PartKind::Battery).and_then(|c| c.props.get("cells").copied())))
        .unwrap_or(2.0);
    let mut weapon_current = 0.0;
    for rp in r.parts.iter().filter(|rp| rp.part.kind == PartKind::Weapon) {
        let Some(wd) = &rp.part.weapon else { continue };
        let Some(i) = w.index(&rp.part.id) else { continue };
        let motor = wd.motor.as_deref().and_then(|id| r.parts.iter().find(|m| m.part.id == id));
        let Some(kv) = motor.and_then(|m| m.cat.as_ref()).and_then(|c| c.props.get("kv").copied()) else {
            out.push(
                Finding::fail(
                    "power.weapon_motor",
                    G,
                    Severity::Blocker,
                    "Weapon has no motor with a known kV",
                    format!("`{}` names motor {:?}; it must be a catalog brushless motor so speed can be computed.", rp.part.id, wd.motor),
                )
                .on(&[&rp.part.id])
                .fix("Set `weapon.motor` to a `weapon_motor` part that uses a catalog entry with kV."),
            );
            continue;
        };
        weapon_current += motor.and_then(|m| m.cat.as_ref()).and_then(|c| c.props.get("max_current_a").copied()).unwrap_or(0.0);
        let ratio = wd.reduction.unwrap_or(1.0).max(0.01);
        let wcells = wd.cells.unwrap_or(cells);
        // AWS 5e: max cell voltage x cells x kV / reduction; no-load speed.
        let rpm = r.profile.max_cell_v * wcells * kv / ratio;
        let Some((origin, axis)) = spin_axis(w, r, i) else { continue };
        let pts = measure::surface_points(w, i, 1.0);
        let rmax = pts
            .iter()
            .map(|p| {
                let q = *p - origin;
                (q - axis * q.dot(axis)).len()
            })
            .fold(0.0_f64, f64::max);
        let tip_mph = rpm * (2.0 * rmax) * std::f64::consts::PI * 60.0 / 1_609_344.0;
        ctx.metrics.insert("tip_speed_mph".into(), tip_mph);
        ctx.metrics.insert("weapon_rpm".into(), rpm);
        let limit = r.profile.max_tip_speed_mph;
        if tip_mph > limit {
            out.push(
                Finding::fail(
                    "power.tip_speed",
                    G,
                    Severity::Blocker,
                    "Weapon tip is too fast",
                    format!("{rpm:.0} rpm at {:.0} mm diameter gives {tip_mph:.0} mph at the tip; the limit is {limit:.0} mph.", 2.0 * rmax),
                )
                .on(&[&rp.part.id])
                .expect(format!("<= {limit:.0} mph"), format!("{tip_mph:.0} mph"))
                .fix("Add a reduction (`weapon.reduction` above 1), fit a lower kV motor, use fewer cells or a smaller disc."),
            );
        } else {
            out.push(Finding::pass(
                "power.tip_speed",
                G,
                "Tip speed is legal",
                format!("{rpm:.0} rpm at {:.0} mm diameter is {tip_mph:.0} mph (limit {limit:.0}).", 2.0 * rmax),
            ));
        }
        // Stored energy at 85% of no-load speed (it sags under load).
        if let Some(dens) = rp.part.material.as_deref().and_then(crate::catalog::density) {
            let iner = measure::axial_inertia(w, i, origin, axis, 0.5) * dens * 1.0e-3 * 1.0e-9; // kg m2
            let omega = rpm * 0.85 * 2.0 * std::f64::consts::PI / 60.0;
            let energy = 0.5 * iner * omega * omega;
            ctx.metrics.insert("weapon_energy_j".into(), energy);
            if energy < 3.0 {
                out.push(
                    Finding::fail(
                        "power.weapon_energy",
                        G,
                        Severity::Warning,
                        "Weapon stores very little energy",
                        format!("About {energy:.1} J at 85% of no-load speed; competitive ant spinners store 10 to 50 J. It will bounce off armour."),
                    )
                    .on(&[&rp.part.id])
                    .fix("Make the weapon heavier toward the rim, larger, or spin it faster (within the tip speed limit)."),
                );
            } else {
                out.push(Finding::pass(
                    "power.weapon_energy",
                    G,
                    "Weapon stores useful energy",
                    format!("About {energy:.1} J at 85% of no-load speed."),
                ));
            }
        }
    }
    // Battery against peak current.
    let bat = r.parts.iter().find(|p| p.part.kind == PartKind::Battery).and_then(|p| p.cat.as_ref());
    if let Some(b) = bat {
        let max_a = b.props.get("capacity_mah").copied().unwrap_or(0.0) / 1000.0 * b.props.get("c_rating").copied().unwrap_or(0.0);
        let drive: f64 = r
            .parts
            .iter()
            .filter(|p| p.part.kind == PartKind::Motor)
            .filter_map(|p| p.cat.as_ref().and_then(|c| c.props.get("stall_current_a").copied()))
            .sum();
        let peak = drive * 0.5 + weapon_current;
        ctx.metrics.insert("peak_current_a".into(), peak);
        if max_a > 0.0 && peak > max_a {
            out.push(
                Finding::fail(
                    "power.battery",
                    G,
                    Severity::Warning,
                    "Battery may sag under peak load",
                    format!(
                        "Estimated peak {peak:.1} A against a {max_a:.1} A rating ({} mAh x {} C); the voltage will collapse in a hit.",
                        b.props.get("capacity_mah").copied().unwrap_or(0.0),
                        b.props.get("c_rating").copied().unwrap_or(0.0)
                    ),
                )
                .fix("Use a bigger or higher-C pack, or a motor that draws less."),
            );
        } else if max_a > 0.0 {
            out.push(Finding::pass("power.battery", G, "Battery covers peak current", format!("Peak {peak:.1} A within the {max_a:.1} A rating.")));
        }
    }
    // ESC against what it drives.
    for esc in r.parts.iter().filter(|p| p.part.kind == PartKind::Esc) {
        let Some(c) = &esc.cat else { continue };
        let cont = c.props.get("cont_current_a").copied().unwrap_or(0.0);
        if cont > 0.0 && weapon_current > cont * 1.6 && c.id.contains("brushless") {
            out.push(
                Finding::fail(
                    "power.esc",
                    G,
                    Severity::Warning,
                    "ESC is small for this motor",
                    format!("`{}` is rated {cont:.0} A continuous but the weapon motor can draw {weapon_current:.0} A.", esc.part.id),
                )
                .on(&[&esc.part.id])
                .fix("Use a bigger ESC or a weaker motor."),
            );
        }
    }
    let _ = v3(0.0, 0.0, 0.0);
    out
}
