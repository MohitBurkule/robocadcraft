//! Mutation tests: each check must catch the mistake it exists for.

use cadcraft_robot::checks::{Options, verify};
use cadcraft_robot::finding::Report;
use cadcraft_robot::sdf::Shape;
use cadcraft_robot::spec::{Fastener, Part, PartKind, Spec};

fn part(id: &str, kind: PartKind, shape: Shape, at: [f64; 3], material: &str, process: &str) -> Part {
    Part { id: id.into(), kind, shape: Some(shape), at, material: Some(material.into()), process: Some(process.into()), ..Part::default() }
}

fn failed(rep: &Report, id: &str) -> bool {
    rep.findings.iter().any(|f| !f.pass && f.id == id)
}

#[test]
fn thin_printed_wall_is_flagged() {
    let thin = Shape::boxed(30.0, 30.0, 10.0).minus(vec![Shape::boxed(29.2, 29.2, 12.0)]);
    let spec = Spec { name: "thin".into(), parts: vec![part("shell", PartKind::Structure, thin, [0.0; 3], "petg", "fdm")], ..Spec::default() };
    assert!(failed(&verify(spec, &Options::default()), "mfg.thin_wall"));
}

#[test]
fn thick_printed_wall_is_fine() {
    let ok = Shape::boxed(30.0, 30.0, 10.0).minus(vec![Shape::boxed(27.0, 27.0, 12.0)]);
    let spec = Spec { name: "ok".into(), parts: vec![part("shell", PartKind::Structure, ok, [0.0; 3], "petg", "fdm")], ..Spec::default() };
    assert!(!failed(&verify(spec, &Options::default()), "mfg.thin_wall"));
}

fn screw_spec(with_overhang: bool) -> Spec {
    let mut parts = vec![part("base", PartKind::Structure, Shape::boxed(30.0, 30.0, 6.0), [0.0; 3], "nylon", "fdm")];
    if with_overhang {
        parts.push(part("cap", PartKind::Structure, Shape::boxed(30.0, 30.0, 3.0), [0.0, 0.0, 9.0], "nylon", "fdm"));
    }
    let f = Fastener {
        id: "s1".into(),
        designation: "M2x6".into(),
        at: [0.0, 0.0, 3.0],
        dir: [0.0, 0.0, -1.0],
        through: vec!["base".into()],
        tool: Some("hex1.5".into()),
        head: Some("socket".into()),
        ..Fastener::default()
    };
    Spec { name: "screw".into(), parts, fasteners: vec![f], ..Spec::default() }
}

#[test]
fn screw_under_overhang_cannot_be_reached() {
    assert!(failed(&verify(screw_spec(true), &Options::default()), "fastener.access"));
}

#[test]
fn open_screw_can_be_reached() {
    assert!(!failed(&verify(screw_spec(false), &Options::default()), "fastener.access"));
}

#[test]
fn floating_part_is_flagged() {
    let spec = Spec {
        name: "float".into(),
        parts: vec![
            part("base", PartKind::Structure, Shape::boxed(30.0, 30.0, 6.0), [0.0; 3], "nylon", "fdm"),
            part("lid", PartKind::Armor, Shape::boxed(30.0, 30.0, 2.0), [0.0, 0.0, 12.0], "nylon", "fdm"),
        ],
        ..Spec::default()
    };
    assert!(failed(&verify(spec, &Options::default()), "connection.floating"));
}
