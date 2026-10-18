//! Assembly-path check: a part sealed in a closed cavity is flagged, the same part in an open pocket is not.

use cadcraft_robot::checks::{Options, verify};
use cadcraft_robot::sdf::Shape;
use cadcraft_robot::spec::{Part, PartKind, Spec};

fn part(id: &str, kind: PartKind, shape: Shape, at: [f64; 3]) -> Part {
    Part { id: id.into(), kind, shape: Some(shape), at, material: Some("nylon".into()), process: Some("fdm".into()), ..Part::default() }
}

fn robot(open_top: bool) -> Spec {
    let shell = Shape::boxed(30.0, 30.0, 30.0).minus(vec![Shape::boxed(10.0, 10.0, 10.0)]);
    let shell = if open_top { shell.minus(vec![Shape::boxed(10.0, 10.0, 30.0).at(0.0, 0.0, 10.0)]) } else { shell };
    Spec {
        name: "assembly test".into(),
        profile: "aws2025".into(),
        parts: vec![
            part("housing", PartKind::Structure, shell, [0.0, 0.0, 0.0]),
            part("block", PartKind::Other, Shape::boxed(9.0, 9.0, 9.0), [0.0, 0.0, 0.0]),
        ],
        ..Spec::default()
    }
}

#[test]
fn sealed_part_cannot_be_assembled() {
    let rep = verify(robot(false), &Options::default());
    assert!(rep.findings.iter().any(|f| !f.pass && f.id == "assembly.blocked" && f.parts.iter().any(|p| p == "block")));
}

#[test]
fn part_in_open_pocket_can_be_assembled() {
    let rep = verify(robot(true), &Options::default());
    assert!(
        !rep.findings.iter().any(|f| !f.pass && f.id == "assembly.blocked"),
        "{:?}",
        rep.findings.iter().filter(|f| !f.pass).map(|f| &f.message).collect::<Vec<_>>()
    );
    assert!(rep.findings.iter().any(|f| f.pass && f.id == "assembly.order"));
}
