//! Reference designs that pass the verifier: starting points an author edits.

use crate::sdf::Shape;
use crate::spec::*;

fn outline(xh: f64, y0: f64, y1: f64, ch: f64) -> Vec<[f64; 2]> {
    vec![[-xh, y1], [xh, y1], [xh, y0 + ch], [xh - ch, y0], [-xh + ch, y0], [-xh, y0 + ch]]
}

/// A box cut that starts `depth` below the parting plane (z = 0) and reaches 0.5 mm above it.
const WEDGE_BOLT_X: f64 = 17.0;

fn cut(cx: f64, cy: f64, sx: f64, sy: f64, depth: f64) -> Shape {
    Shape::boxed(sx, sy, depth + 0.5).at(cx, cy, -depth / 2.0 + 0.25)
}

/// Full-height wall block around a pocket.
fn block(cx: f64, cy: f64, sx: f64, sy: f64, hh: f64) -> Shape {
    Shape::boxed(sx, sy, hh).at(cx, cy, -hh / 2.0)
}

fn part(id: &str, kind: PartKind) -> Part {
    Part { id: id.into(), kind, ..Part::default() }
}

fn cat(id: &str, kind: PartKind, catalog: &str, at: [f64; 3]) -> Part {
    Part { id: id.into(), kind, catalog: Some(catalog.into()), at, ..Part::default() }
}

/// Reference 2WD wedge: printed two-shell chassis with a ribbed wedge, two N10 gearmotors driving
/// 28 mm wheels at the rear, a 2S 180 mAh pack, brushed ESC and receiver. Meant to pass with margin.
pub fn wedge_2wd() -> Spec {
    let hh = 10.0;
    let (xh, y0, y1) = (42.0, -40.0, 36.0);
    let wall = 1.4;
    let outer = Shape::Prism { points: outline(xh, y0, y1, 4.0), height: hh }.at(0.0, 0.0, -hh / 2.0);
    let inner = Shape::Prism { points: outline(xh - 1.6, y0 + 1.6, y1 - 1.6, 2.4), height: 9.0 }.at(0.0, 0.0, -4.0);
    let shell = outer.minus(vec![inner]);

    let wy = -16.0; // wheel and motor axis (y)
    let xm = 16.4; // motor centre x
    let bay = (16.6, 22.4, 12.0);
    let xw = 34.4; // wheel centre x
    let mut blocks = vec![
        block(0.0, 8.0, 28.8 + 2.0 * wall, 18.8 + 2.0 * wall, hh),
        block(0.0, 26.0, 20.8 + 2.0 * wall, 14.8 + 2.0 * wall, hh),
        block(0.0, -31.5, 18.0 + 2.0 * wall, 13.0 + 2.0 * wall, hh),
        block(-18.0, -34.0, 12.4 + 2.0 * wall, 4.4 + 2.0 * wall, hh),
    ];
    let mut pockets = vec![
        cut(0.0, 8.0, 28.8, 18.8, 5.4),
        cut(0.0, 18.5, 6.0, 3.4, 5.0),
        cut(0.0, 26.0, 20.8, 14.8, 4.5),
        cut(0.0, -31.5, 18.0, 13.0, 4.5),
        cut(-18.0, -34.0, 12.4, 4.4, 3.5),
        cut(-10.0, -34.0, 4.0, 3.0, 5.0),
        // wire channel between the motors, bypass trench beside the battery
        cut(0.0, -17.0, 6.0, 32.0, 5.0),
        cut(8.5, -2.5, 17.0, 3.0, 5.0),
        cut(16.5, 9.0, 3.6, 26.0, 5.0),
        cut(14.15, 21.5, 8.3, 3.0, 5.0),
        cut(8.5, 19.0, 17.0, 3.0, 5.0),
    ];
    for s in [1.0, -1.0] {
        blocks.push(block(s * bay.0, wy, bay.1 + 2.0 * wall, bay.2 + 2.0 * wall, hh));
        blocks.push(block(s * xw, wy, 10.8 + 2.0 * wall, 29.0 + 2.0 * wall, hh));
        pockets.push(cut(s * bay.0, wy, bay.1, bay.2, 6.8));
        pockets.push(cut(s * 4.15, wy, 3.3, 5.0, 3.0));
        pockets.push(Shape::cyl_x(2.0, 2.4).at(s * 28.4, wy, 0.0));
        pockets.push(cut(s * xw, wy, 10.8, 29.0, 10.0));
    }
    let bolts: [(&str, f64, f64); 4] = [("fl", 26.0, 29.0), ("fr", -26.0, 29.0), ("rl", 30.0, -35.0), ("rr", -30.0, -35.0)];
    for (_, x, y) in bolts {
        blocks.push(Shape::cyl(4.0, hh).at(x, y, -hh / 2.0));
    }
    // Wedge mount bosses on the front wall.
    for s in [1.0, -1.0] {
        blocks.push(Shape::cyl_y(3.0, 10.0).at(s * WEDGE_BOLT_X, 31.0, -6.0));
    }
    let mut base_blocks = vec![shell];
    base_blocks.extend(blocks);
    let body = Shape::union(base_blocks).minus(pockets);

    // Bottom shell: heat-set insert seats under each bolt.
    let mut bottom_cuts = Vec::new();
    for s in [1.0, -1.0] {
        bottom_cuts.push(Shape::cyl_y(1.6, 3.6).at(s * WEDGE_BOLT_X, 34.2, -6.0));
    }
    for (_, x, y) in bolts {
        bottom_cuts.push(Shape::cyl(1.1, 5.9).at(x, y, -2.45));
        bottom_cuts.push(Shape::cyl(1.6, 3.0).at(x, y, -6.7));
    }
    // Wedge: thin sloped plate on triangular ribs, printed with the bottom shell.
    let ang = 14.0_f64.to_radians();
    let (t, tip_y, tip_z) = (2.0, y1 + 46.0, -13.4);
    let (sn, cs) = (ang.sin(), ang.cos());
    let b1 = [tip_y, tip_z];
    let t1 = [tip_y + t * sn, tip_z + t * cs];
    let t0z = t1[1] + (t1[0] - y1) * ang.tan();
    let t0 = [y1, t0z];
    let b0 = [y1 - t * sn, t0z - t * cs];
    let to_x = |s: Shape, x: f64| s.rot([90.0, 0.0, 90.0]).at(x, 0.0, 0.0);
    let plate = to_x(Shape::Prism { points: vec![b0, t0, t1, b1], height: 2.0 * (xh - 1.0) }, 0.0);
    let rib_y_end = b0[0] + (b0[1] + 10.0) / ang.tan();
    let mut wedge_items = vec![plate];
    // Back plate against the chassis front face, bolted to the front bosses.
    wedge_items.push(Shape::boxed(2.0 * (xh - 1.0), 2.4, 9.65).at(0.0, y1 + 1.2, -5.175));
    for x in [0.0, 14.0, -14.0, 28.0, -28.0, 41.0, -41.0] {
        let rib = Shape::Prism { points: vec![[b0[0], b0[1] + 0.2], [rib_y_end, -10.0 + 0.0], [y1, -10.0], [y1, b0[1] + 0.2]], height: 1.2 };
        wedge_items.push(to_x(rib, x));
    }

    let mut chassis_bottom = part("chassis_bottom", PartKind::Structure);
    chassis_bottom.shape = Some(body.clone().minus(bottom_cuts));
    chassis_bottom.material = Some("nylon".into());
    chassis_bottom.process = Some("fdm".into());
    chassis_bottom.color = Some("#23262b".into());

    let mut top_cuts = Vec::new();
    for (_, x, y) in bolts {
        top_cuts.push(Shape::cyl(1.2, 11.0).at(x, y, 5.0));
        top_cuts.push(Shape::cyl(2.3, 2.7).at(x, y, 9.15));
    }
    top_cuts.push(Shape::boxed(12.4, 4.4, 8.5).at(-18.0, -34.0, 6.25));
    let mut chassis_top = part("chassis_top", PartKind::Structure);
    chassis_top.shape = Some(body.mirrored(2).minus(top_cuts));
    chassis_top.material = Some("nylon".into());
    chassis_top.process = Some("fdm".into());
    chassis_top.color = Some("#3a3e46".into());

    let mut wedge = part("wedge", PartKind::Wedge);
    let wedge_holes: Vec<Shape> = [1.0, -1.0].iter().map(|s| Shape::cyl_y(1.1, 4.0).at(s * WEDGE_BOLT_X, y1 + 1.2, -6.0)).collect();
    let mut wedge_cuts = wedge_holes;
    // Nothing of the wedge goes behind the chassis front face.
    wedge_cuts.push(Shape::boxed(100.0, 10.0, 40.0).at(0.0, y1 - 5.0, 0.0));
    wedge.shape = Some(Shape::union(wedge_items).minus(wedge_cuts));
    wedge.material = Some("petg_cf".into());
    wedge.process = Some("fdm".into());
    wedge.color = Some("#ff6a1a".into());

    let mut parts = vec![chassis_bottom, chassis_top, wedge];
    let mut motor_l = cat("motor_l", PartKind::Motor, "n10_300rpm_6v", [xm, wy, 0.0]);
    motor_l.label = Some("left drive".into());
    let mut motor_r = cat("motor_r", PartKind::Motor, "n10_300rpm_6v", [-xm, wy, 0.0]);
    motor_r.rot = [0.0, 0.0, 180.0];
    motor_r.label = Some("right drive".into());
    parts.push(motor_l);
    parts.push(motor_r);
    parts.push(cat("wheel_l", PartKind::Wheel, "wheel_28x10", [xw, wy, 0.0]));
    parts.push(cat("wheel_r", PartKind::Wheel, "wheel_28x10", [-xw, wy, 0.0]));
    parts.push(cat("battery", PartKind::Battery, "lipo_2s_180", [0.0, 8.0, 0.0]));
    let mut esc = cat("esc", PartKind::Esc, "esc_dual_brushed", [0.0, -31.5, 0.0]);
    esc.retained = Some("tape".into());
    parts.push(esc);
    let mut rx = cat("rx", PartKind::Receiver, "rx_fs_ia6_decased", [0.0, 26.0, 0.0]);
    rx.retained = Some("tape".into());
    parts.push(rx);
    let mut sw = cat("power_switch", PartKind::Switch, "switch_slide_12x4", [-18.0, -34.0, 7.8]);
    sw.tool_free = true;
    sw.marked = true;
    sw.label = Some("ON/OFF".into());
    sw.retained = Some("adhesive".into());
    parts.push(sw);

    let mut fasteners = Vec::new();
    for (tag, s) in [("wl", 1.0), ("wr", -1.0)] {
        let mut ins = cat(&format!("insert_{tag}"), PartKind::Insert, "heatset_m2", [s * WEDGE_BOLT_X, 34.3, -6.0]);
        ins.rot = [90.0, 0.0, 0.0];
        ins.retained = Some("press_fit".into());
        parts.push(ins);
        fasteners.push(Fastener {
            id: format!("bolt_{tag}"),
            designation: "M2x6".into(),
            at: [s * WEDGE_BOLT_X, y1 + 2.4, -6.0],
            dir: [0.0, -1.0, 0.0],
            through: vec!["wedge".into()],
            threads_into: Some(format!("insert_{tag}")),
            tool: Some("hex1.5".into()),
            removable: true,
            head: Some("socket".into()),
        });
    }
    for (tag, x, y) in bolts {
        let mut ins = cat(&format!("insert_{tag}"), PartKind::Insert, "heatset_m2", [x, y, -6.7]);
        ins.retained = Some("press_fit".into());
        parts.push(ins);
        fasteners.push(Fastener {
            id: format!("bolt_{tag}"),
            designation: "M2x16".into(),
            at: [x, y, 7.8],
            dir: [0.0, 0.0, -1.0],
            through: vec!["chassis_top".into(), "chassis_bottom".into()],
            threads_into: Some(format!("insert_{tag}")),
            tool: Some("hex1.5".into()),
            removable: true,
            head: Some("socket".into()),
        });
    }
    let wire = |id: &str, from: &str, to: &str, kind: &str| Wire {
        id: id.into(),
        from: from.into(),
        to: to.into(),
        awg: Some(26.0),
        kind: kind.into(),
        slack_mm: Some(6.0),
    };
    let wires = vec![
        wire("w_bat_sw", "battery.xt30", "power_switch.in", "power"),
        wire("w_sw_esc", "power_switch.out", "esc.bat", "power"),
        wire("w_esc_ml", "esc.m2", "motor_l.terminals", "motor"),
        wire("w_esc_mr", "esc.m1", "motor_r.terminals", "motor"),
        wire("w_rx_esc", "rx.esc", "esc.rx", "signal"),
    ];
    Spec {
        name: "Reference 2WD wedge".into(),
        profile: "aws2025".into(),
        robot_type: RobotType::Wedge,
        locomotion: Locomotion::Roller,
        parts,
        fasteners,
        wires,
        electronics: Electronics { failsafe: true, weapon_remote_disable: false, radio: Some("2.45 GHz".into()), cells: Some(2.0) },
        connections: vec![
            Connection {
                a: "motor_l".into(),
                b: "wheel_l".into(),
                method: "press_fit".into(),
                reason: "Printed wheel hub pressed onto the 3 mm D-shaft of the N10 motor; drive torque is low so a 0.1 mm interference plus a drop of retainer holds it".into(),
                interference_mm: Some(0.1),
                ..Connection::default()
            },
            Connection {
                a: "motor_r".into(),
                b: "wheel_r".into(),
                method: "press_fit".into(),
                reason: "Printed wheel hub pressed onto the 3 mm D-shaft of the N10 motor; drive torque is low so a 0.1 mm interference plus a drop of retainer holds it".into(),
                interference_mm: Some(0.1),
                ..Connection::default()
            },
        ],
        claims: vec![
            Claim { id: "c_mass".into(), text: "weighs 150 g or less".into(), metric: "mass_total_g".into(), op: "<=".into(), value: 150.0 },
            Claim { id: "c_fit".into(), text: "fits the 150 mm square".into(), metric: "footprint_mm".into(), op: "<=".into(), value: 150.0 },
        ],
        ..Spec::default()
    }
}

pub fn by_name(name: &str) -> Option<Spec> {
    match name {
        "wedge" | "wedge_2wd" => Some(wedge_2wd()),
        _ => None,
    }
}

pub const NAMES: &[&str] = &["wedge_2wd"];
