//! Parts catalog: real components with locked dimensions, mass, ports and electrical data.
//!
//! An author (human or LLM) picks catalog ids; it cannot change their size. Every entry says how
//! sure we are of its numbers: `datasheet`, `measured` or `estimate`. Estimates make the mass
//! report show a wider range until the real part is weighed (`scale_mass_g` on the part).
//!
//! Frame conventions: motors and wheels have their shaft/axle along local +X, origin at the
//! body centre; batteries have their long side along X; flat electronics lie in the XY plane.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::sdf::Shape;
use crate::spec::PartKind;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CatalogEntry {
    pub id: String,
    pub kind: PartKind,
    pub name: String,
    pub mass_g: f64,
    /// `datasheet`, `measured` or `estimate`.
    pub confidence: String,
    pub shape: Shape,
    /// Named connection points in the part frame.
    pub ports: BTreeMap<String, [f64; 3]>,
    /// Output shaft: start, direction, length, diameter.
    pub shaft: Option<ShaftDef>,
    /// Electrical and other numbers: `nominal_v`, `rpm_nominal`, `stall_current_a`, `kv`, ...
    pub props: BTreeMap<String, f64>,
    pub color: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShaftDef {
    pub at: [f64; 3],
    pub dir: [f64; 3],
    pub len: f64,
    pub dia: f64,
}

fn props(items: &[(&str, f64)]) -> BTreeMap<String, f64> {
    items.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

fn ports(items: &[(&str, [f64; 3])]) -> BTreeMap<String, [f64; 3]> {
    items.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

/// Gearmotor with a round can with two flats and a brass gearbox, shaft along +X.
fn gearmotor(
    id: &str,
    name: &str,
    can_len: f64,
    can_dia: f64,
    flats: f64,
    gb_len: f64,
    gb_h: f64,
    shaft_len: f64,
    mass: f64,
    p: &[(&str, f64)],
) -> CatalogEntry {
    let total = can_len + gb_len;
    let gb_c = total / 2.0 - gb_len / 2.0;
    let can_c = -total / 2.0 + can_len / 2.0;
    let can =
        Shape::Intersect { items: vec![Shape::cyl_x(can_dia / 2.0, can_len), Shape::boxed(can_len + 2.0, flats, can_dia + 2.0)] }.at(can_c, 0.0, 0.0);
    let gb = Shape::boxed(gb_len, flats, gb_h).at(gb_c, 0.0, 0.0);
    let shaft_at = total / 2.0;
    let shaft = Shape::cyl_x(1.5, shaft_len).at(shaft_at + shaft_len / 2.0, 0.0, 0.0);
    CatalogEntry {
        id: id.into(),
        kind: PartKind::Motor,
        name: name.into(),
        mass_g: mass,
        confidence: "estimate".into(),
        shape: Shape::union(vec![can, gb, shaft]),
        ports: ports(&[("terminals", [-total / 2.0 - 0.3, 0.0, 0.0])]),
        shaft: Some(ShaftDef { at: [shaft_at, 0.0, 0.0], dir: [1.0, 0.0, 0.0], len: shaft_len, dia: 3.0 }),
        props: props(p),
        color: "#9aa0a6".into(),
    }
}

fn outrunner(id: &str, name: &str, dia: f64, len: f64, shaft_len: f64, mass: f64, kv: f64, p: &[(&str, f64)]) -> CatalogEntry {
    let body = Shape::cyl_x(dia / 2.0, len).at(0.0, 0.0, 0.0);
    let shaft = Shape::cyl_x(2.5, shaft_len).at(len / 2.0 + shaft_len / 2.0 - 1.0, 0.0, 0.0);
    let mut pr = props(p);
    pr.insert("kv".into(), kv);
    CatalogEntry {
        id: id.into(),
        kind: PartKind::WeaponMotor,
        name: name.into(),
        mass_g: mass,
        confidence: "estimate".into(),
        shape: Shape::union(vec![body, shaft]),
        ports: ports(&[("phases", [-len / 2.0, 0.0, 0.0])]),
        shaft: Some(ShaftDef { at: [len / 2.0 - 1.0, 0.0, 0.0], dir: [1.0, 0.0, 0.0], len: shaft_len, dia: 5.0 }),
        props: pr,
        color: "#3a3d42".into(),
    }
}

fn lipo(id: &str, name: &str, l: f64, w: f64, h: f64, mass: f64, cells: f64, mah: f64, c: f64) -> CatalogEntry {
    CatalogEntry {
        id: id.into(),
        kind: PartKind::Battery,
        name: name.into(),
        mass_g: mass,
        confidence: "estimate".into(),
        shape: Shape::boxed(l, w, h),
        ports: ports(&[("xt30", [0.0, w / 2.0 + 2.0, 0.0])]),
        shaft: None,
        props: props(&[
            ("cells", cells),
            ("capacity_mah", mah),
            ("c_rating", c),
            ("nominal_v", 3.7 * cells),
            ("internal_resistance_mohm", 90.0 / (mah / 100.0).max(0.5) * cells),
        ]),
        color: "#2a9d8f".into(),
    }
}

fn flat(
    id: &str,
    name: &str,
    kind: PartKind,
    x: f64,
    y: f64,
    z: f64,
    mass: f64,
    pts: &[(&str, [f64; 3])],
    p: &[(&str, f64)],
    color: &str,
) -> CatalogEntry {
    CatalogEntry {
        id: id.into(),
        kind,
        name: name.into(),
        mass_g: mass,
        confidence: "estimate".into(),
        shape: Shape::boxed(x, y, z),
        ports: ports(pts),
        shaft: None,
        props: props(p),
        color: color.into(),
    }
}

fn wheel(id: &str, dia: f64, width: f64, hub: f64, mass: f64) -> CatalogEntry {
    let tire = Shape::cyl_x(dia / 2.0, width).minus(vec![Shape::cyl_x(hub / 2.0 - 0.5, width + 2.0)]);
    let hubs = Shape::cyl_x(hub / 2.0, width).minus(vec![Shape::cyl_x(1.5, width + 2.0)]);
    CatalogEntry {
        id: id.into(),
        kind: PartKind::Wheel,
        name: format!("{dia} x {width} mm wheel (printed hub, silicone tyre)"),
        mass_g: mass,
        confidence: "estimate".into(),
        shape: Shape::union(vec![tire, hubs]),
        ports: BTreeMap::new(),
        shaft: Some(ShaftDef { at: [-width / 2.0, 0.0, 0.0], dir: [1.0, 0.0, 0.0], len: width, dia: 3.0 }),
        props: props(&[("diameter_mm", dia), ("width_mm", width), ("friction", 0.9)]),
        color: "#25282c".into(),
    }
}

pub fn catalog() -> Vec<CatalogEntry> {
    let mut v = vec![
        gearmotor(
            "n10_300rpm_6v",
            "N10 micro gearmotor 6 V 300 rpm",
            12.0,
            12.0,
            10.0,
            9.1,
            12.0,
            9.0,
            7.5,
            &[("nominal_v", 6.0), ("rpm_nominal", 300.0), ("stall_current_a", 0.65), ("no_load_current_a", 0.06), ("stall_torque_gcm", 900.0)],
        ),
        gearmotor(
            "n10_2200rpm_6v",
            "N10 micro gearmotor 6 V 2200 rpm",
            12.0,
            12.0,
            10.0,
            9.1,
            12.0,
            9.0,
            7.5,
            &[("nominal_v", 6.0), ("rpm_nominal", 2200.0), ("stall_current_a", 0.8), ("no_load_current_a", 0.08), ("stall_torque_gcm", 150.0)],
        ),
        gearmotor(
            "n20_1000rpm_6v",
            "N20 gearmotor 6 V 1000 rpm",
            15.0,
            12.0,
            10.0,
            9.0,
            12.0,
            10.0,
            10.5,
            &[("nominal_v", 6.0), ("rpm_nominal", 1000.0), ("stall_current_a", 0.8), ("no_load_current_a", 0.07), ("stall_torque_gcm", 600.0)],
        ),
        outrunner("bl_1806_2300kv", "1806 2300 kV outrunner", 21.0, 14.0, 8.0, 17.5, 2300.0, &[("max_current_a", 15.0), ("typ_current_a", 8.0)]),
        outrunner("bl_2306_1400kv", "2306 1400 kV outrunner", 27.5, 14.0, 10.0, 29.0, 1400.0, &[("max_current_a", 20.0), ("typ_current_a", 10.0)]),
        lipo("lipo_2s_180", "2S 180 mAh LiPo (nano-tech class)", 28.0, 18.0, 10.0, 12.5, 2.0, 180.0, 40.0),
        lipo("lipo_2s_350", "2S 350 mAh LiPo", 36.0, 17.0, 11.0, 21.5, 2.0, 350.0, 30.0),
        lipo("lipo_1s_300", "1S 300 mAh LiPo", 30.0, 13.0, 8.0, 8.5, 1.0, 300.0, 30.0),
        flat(
            "esc_dual_brushed",
            "Dual brushed ESC (hobby, heat-shrunk)",
            PartKind::Esc,
            15.0,
            10.0,
            5.0,
            3.0,
            &[("bat", [0.0, -6.0, 3.0]), ("m1", [-8.0, 0.0, 3.0]), ("m2", [8.0, 0.0, 3.0]), ("rx", [0.0, 6.0, 3.0])],
            &[("cont_current_a", 2.5), ("burst_current_a", 5.0), ("channels", 2.0)],
            "#e76f51",
        ),
        flat(
            "esc_brushless_12a",
            "Brushless ESC 12 A with BEC",
            PartKind::Esc,
            20.0,
            12.0,
            4.0,
            6.0,
            &[("bat", [0.0, -6.5, 0.0]), ("motor", [-10.5, 0.0, 0.0]), ("rx", [10.5, 0.0, 0.0])],
            &[("cont_current_a", 12.0), ("burst_current_a", 15.0), ("channels", 1.0)],
            "#d4af37",
        ),
        flat(
            "rx_fs_ia6_decased",
            "FlySky FS-iA6 receiver, decased",
            PartKind::Receiver,
            20.0,
            14.0,
            4.0,
            4.5,
            &[("esc", [0.0, 6.8, 0.0]), ("servo", [-5.0, 6.8, 0.0])],
            &[("antenna_len_mm", 80.0)],
            "#ff9800",
        ),
        flat(
            "servo_cs07",
            "KingMax CS07 micro servo",
            PartKind::Servo,
            22.0,
            11.5,
            20.0,
            6.5,
            &[("signal", [-11.5, 0.0, 0.0])],
            &[("stall_torque_kgcm", 1.8), ("nominal_v", 4.8), ("stall_current_a", 0.7)],
            "#4a4e57",
        ),
        flat(
            "switch_slide_12x4",
            "Miniature slide switch 11.8 x 3.8",
            PartKind::Switch,
            11.8,
            3.8,
            4.0,
            0.5,
            &[("in", [-4.0, 0.0, -2.5]), ("out", [4.0, 0.0, -2.5])],
            &[],
            "#111111",
        ),
        wheel("wheel_28x10", 28.0, 10.0, 19.0, 3.4),
        wheel("wheel_30x8", 30.0, 8.0, 19.0, 3.0),
        wheel("wheel_20x8", 20.0, 8.0, 12.0, 1.8),
    ];
    v.push(CatalogEntry {
        id: "heatset_m2".into(),
        kind: PartKind::Insert,
        name: "M2 brass heat-set insert 3.0 mm".into(),
        mass_g: 0.12,
        confidence: "estimate".into(),
        shape: Shape::cyl(1.6, 3.0).minus(vec![Shape::cyl(1.0, 4.0)]),
        ports: BTreeMap::new(),
        shaft: None,
        props: props(&[("thread_d_mm", 2.0)]),
        color: "#d4af37".into(),
    });
    v.push(CatalogEntry {
        id: "nut_m2".into(),
        kind: PartKind::Nut,
        name: "M2 hex nut".into(),
        mass_g: 0.1,
        confidence: "datasheet".into(),
        shape: Shape::Prism { points: hexagon(2.0), height: 1.6 }.minus(vec![Shape::cyl(1.0, 3.0)]),
        ports: BTreeMap::new(),
        shaft: None,
        props: props(&[("thread_d_mm", 2.0)]),
        color: "#aaaaaa".into(),
    });
    v
}

fn hexagon(r: f64) -> Vec<[f64; 2]> {
    (0..6).map(|i| (i as f64 * 60.0_f64).to_radians()).map(|a| [r * a.cos() * 1.1547, r * a.sin() * 1.1547]).collect()
}

pub fn find(id: &str) -> Option<CatalogEntry> {
    catalog().into_iter().find(|c| c.id == id)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MatClass {
    /// Extruded or resin printed plastic.
    Printed,
    /// Rigid sheet or machined plastic (HDPE, polypropylene, polycarbonate, UHMW).
    Sheet,
    /// Flexible elastomer (TPU, silicone, rubber).
    Elastomer,
    /// Carbon-fibre or glass laminate plate.
    Laminate,
    Metal,
    /// Adhesive tapes, heat shrink, hook-and-loop: thin, glue the part, carry no load.
    Tape,
}

#[derive(Clone, Copy, Debug)]
pub struct Material {
    pub density: f64,
    pub class: MatClass,
    /// Shatters or cracks under impact (a bad choice for weapons and armour).
    pub brittle: bool,
    /// Smallest wall that prints or machines reliably (mm); 0 when not applicable.
    pub min_wall_mm: f64,
}

const fn mat(density: f64, class: MatClass, brittle: bool, min_wall_mm: f64) -> Material {
    Material { density, class, brittle, min_wall_mm }
}

/// Look up a material by id. Ids: `pla`, `petg`, `abs`, `asa`, `nylon`, `pla_cf`, `petg_cf`, `pa_cf`,
/// `tpu`, `tpu_85a`, `silicone`, `rubber`, `hdpe`, `pp`, `uhmw`, `pc`, `cf_plate`, `fr4`, `aluminium`,
/// `steel`, `titanium`, `brass`, `copper`, `tape`, `double_sided_tape`, `heat_shrink`, `velcro`, `resin`.
pub fn material(id: &str) -> Option<Material> {
    use MatClass::*;
    Some(match id {
        "pla" | "pla_st" => mat(1.24, Printed, true, 0.9),
        "petg" => mat(1.27, Printed, false, 0.9),
        "abs" | "asa" => mat(1.05, Printed, false, 0.9),
        "nylon" | "taulman910" | "pa12" => mat(1.12, Printed, false, 0.9),
        "pla_cf" | "petg_cf" | "pa_cf" | "cf_filament" => mat(1.3, Printed, true, 1.0),
        "pc" | "polycarbonate" => mat(1.2, Sheet, false, 0.5),
        "resin" => mat(1.15, Printed, true, 0.8),
        "tpu" | "tpu_95a" | "tpu_85a" => mat(1.2, Elastomer, false, 1.2),
        "silicone" => mat(1.1, Elastomer, false, 1.0),
        "rubber" | "lego_tyre" => mat(1.15, Elastomer, false, 0.8),
        "hdpe" => mat(0.95, Sheet, false, 1.0),
        "pp" | "polypropylene" => mat(0.91, Sheet, false, 1.0),
        "uhmw" => mat(0.94, Sheet, false, 1.0),
        "cf_plate" | "carbon_fibre" => mat(1.55, Laminate, true, 0.5),
        "fr4" | "g10" => mat(1.85, Laminate, true, 0.5),
        "aluminium" | "aluminum" => mat(2.7, Metal, false, 0.5),
        "steel" | "hardox" | "stainless" | "304" => mat(7.9, Metal, false, 0.3),
        "titanium" => mat(4.43, Metal, false, 0.4),
        "brass" => mat(8.5, Metal, false, 0.3),
        "copper" => mat(8.96, Metal, false, 0.2),
        "tape" | "kapton" | "duct_tape" => mat(0.9, Tape, false, 0.0),
        "double_sided_tape" | "foam_tape" => mat(0.5, Tape, false, 0.0),
        "heat_shrink" => mat(1.2, Tape, false, 0.0),
        "velcro" => mat(0.4, Tape, false, 0.0),
        _ => return None,
    })
}

/// Material densities in g/cm3.
pub fn density(id: &str) -> Option<f64> {
    material(id).map(|m| m.density)
}

pub fn is_printed(process: &str) -> bool {
    matches!(process, "fdm" | "resin")
}

/// Printed by process, or by material class when no process is given.
pub fn part_is_printed(process: Option<&str>, material: Option<&str>) -> bool {
    match process {
        Some(p) => is_printed(p),
        None => material.and_then(self::material).is_some_and(|m| m.class == MatClass::Printed),
    }
}

/// ISO socket-head cap screw: (head diameter, head height, socket across flats, key size) by thread diameter.
pub fn screw_dims(d: f64) -> (f64, f64, f64) {
    match d {
        d if d <= 1.65 => (3.0, 1.6, 1.5),
        d if d <= 2.05 => (3.8, 2.0, 1.5),
        d if d <= 2.55 => (4.5, 2.5, 2.0),
        d if d <= 3.05 => (5.5, 3.0, 2.5),
        d if d <= 4.05 => (7.0, 4.0, 3.0),
        _ => (8.5, 5.0, 4.0),
    }
}

/// Parse `M2x16` into (thread diameter, length).
pub fn parse_designation(s: &str) -> Option<(f64, f64)> {
    let s = s.trim().trim_start_matches(['M', 'm']);
    let (d, l) = s.split_once(['x', 'X'])?;
    let (d, l) = (d.parse::<f64>().ok()?, l.parse::<f64>().ok()?);
    if d > 0.0 && l > 0.0 && d < 12.0 && l < 100.0 { Some((d, l)) } else { None }
}
