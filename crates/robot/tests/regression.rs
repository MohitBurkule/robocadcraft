//! Every flaw a reviewer or builder found in a real design stays a test: the verifier must keep catching it.

use cadcraft_robot::checks::{CHECKS_REVISION, Options, verify};
use cadcraft_robot::finding::Report;
use cadcraft_robot::spec::Spec;

const REVRON_V1: &str = include_str!("fixtures/revron_v1.json");

fn revron() -> Spec {
    serde_json::from_str(REVRON_V1).expect("fixture parses")
}

fn failed(rep: &Report, id: &str) -> bool {
    rep.findings.iter().any(|f| !f.pass && f.id == id)
}

#[test]
fn revron_v1_cantilevered_disc_is_blocked() {
    let rep = verify(revron(), &Options::default());
    assert!(failed(&rep, "spinner.bearing"), "disc on one motor bearing must be flagged");
    assert!(failed(&rep, "spinner.mass_share"));
    assert!(failed(&rep, "spinner.exposed"), "weapon motor sits outside the armour");
    assert!(!rep.ok);
}

#[test]
fn revron_v1_fastener_flaws_are_caught() {
    let rep = verify(revron(), &Options::default());
    assert!(failed(&rep, "fastener.engagement"));
    assert!(failed(&rep, "mfg.thin_wall"));
}

#[test]
fn removing_locking_bar_is_blocked() {
    let mut s = revron();
    for p in &mut s.parts {
        if let Some(w) = p.weapon.as_mut() {
            w.locking_bar = None;
        }
    }
    s.parts.retain(|p| p.id != "lock_pin");
    assert!(failed(&verify(s, &Options::default()), "legal.locking_bar"));
}

#[test]
fn overspeed_is_blocked() {
    let mut s = revron();
    for p in &mut s.parts {
        if let Some(w) = p.weapon.as_mut() {
            w.reduction = Some(0.1);
        }
    }
    assert!(failed(&verify(s, &Options::default()), "power.tip_speed"));
}

#[test]
fn missing_remote_disable_is_blocked() {
    let mut s = revron();
    s.electronics.weapon_remote_disable = false;
    assert!(failed(&verify(s, &Options::default()), "legal.weapon_disable"));
}

#[test]
fn declared_shaft_support_is_verified_not_trusted() {
    let mut s = revron();
    for p in &mut s.parts {
        if let Some(w) = p.weapon.as_mut() {
            w.supported_both_ends = true;
        }
    }
    assert!(failed(&verify(s, &Options::default()), "spinner.bearing"));
}

#[test]
fn report_lists_what_it_did_not_check() {
    let rep = verify(revron(), &Options::default());
    assert_eq!(rep.checks_revision, CHECKS_REVISION);
    assert!(!rep.not_checked.is_empty(), "a pass must never read as 'everything was checked'");
}
