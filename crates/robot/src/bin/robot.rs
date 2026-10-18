//! Command line for building and checking antweight robot designs: `robot guide|catalog|materials|templates|template|verify|render|export`.

use std::io::Write;
use std::path::Path;

use cadcraft_robot::catalog;
use cadcraft_robot::checks::{Options, verify_resolved};
use cadcraft_robot::export;
use cadcraft_robot::finding::{Report, Severity};
use cadcraft_robot::render::{Marker, View, render_png};
use cadcraft_robot::spec::Spec;
use cadcraft_robot::templates;
use cadcraft_robot::world::{World, resolve};

/// Print without panicking when the reader closes the pipe (`robot catalog | head`).
macro_rules! out {
    ($($a:tt)*) => {{
        let _ = writeln!(std::io::stdout(), $($a)*);
    }};
}

const GUIDE: &str = include_str!("../../GUIDE.md");

fn load(path: &str) -> Result<Spec, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    serde_json::from_str::<Spec>(&text).map_err(|e| format!("{path} is not a valid spec: {e}"))
}

fn print_report(rep: &Report, full: bool) {
    out!("{}: {} blockers, {} warnings, {} checks passed. ok={}", rep.name, rep.summary.blockers, rep.summary.warnings, rep.summary.passed, rep.ok);
    out!("check set revision {}", rep.checks_revision);
    let m = |k: &str| rep.metrics.get(k).copied();
    if let (Some(t), Some(lo), Some(hi)) = (m("mass_total_g"), m("mass_lo_g"), m("mass_hi_g")) {
        out!("mass {t:.1} g (range {lo:.1} to {hi:.1} g, limit {:.0})", m("mass_limit_g").unwrap_or(150.0));
    }
    if let Some(fp) = m("footprint_mm") {
        out!("footprint {fp:.1} mm square");
    }
    let mut n = 0;
    for f in &rep.findings {
        if f.pass && !full {
            continue;
        }
        n += 1;
        let tag = if f.pass {
            "pass".to_string()
        } else if f.severity == Severity::Blocker {
            "BLOCKER".to_string()
        } else {
            "warning".to_string()
        };
        let at = f.at.map(|p| format!(" at ({:.1}, {:.1}, {:.1})", p[0], p[1], p[2])).unwrap_or_default();
        out!("[{tag}] {} - {}{}{}", f.id, f.message, if f.parts.is_empty() { String::new() } else { format!(" parts: {}", f.parts.join(", ")) }, at);
        if !f.pass
            && let Some(fix) = &f.fix
        {
            out!("    fix: {fix}");
        }
    }
    if n == 0 {
        out!("nothing to fix.");
    }
    if !rep.not_checked.is_empty() {
        out!("NOT CHECKED (a pass says nothing about these):");
        for x in &rep.not_checked {
            out!("  - {x}");
        }
    }
}

fn run(args: &[String]) -> Result<(), String> {
    let cmd = args.first().map(String::as_str).unwrap_or("help");
    let rest = args.get(1..).unwrap_or(&[]);
    let full = rest.iter().any(|a| a == "--full");
    let pos: Vec<&String> = rest.iter().filter(|a| !a.starts_with("--")).collect();
    match cmd {
        "guide" | "help" => out!("{GUIDE}"),
        "catalog" => {
            for e in catalog::catalog() {
                let b = e.shape.bounds().size();
                let ports: Vec<&String> = e.ports.keys().collect();
                out!(
                    "{} | {:?} | {} | {:.1} g ({}) | box {:.1} x {:.1} x {:.1} mm | ports: {}",
                    e.id,
                    e.kind,
                    e.name,
                    e.mass_g,
                    e.confidence,
                    b.x,
                    b.y,
                    b.z,
                    ports.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ")
                );
            }
        }
        "materials" => {
            for id in [
                "pla",
                "petg",
                "abs",
                "asa",
                "nylon",
                "pla_cf",
                "petg_cf",
                "pa_cf",
                "tpu",
                "silicone",
                "rubber",
                "hdpe",
                "pp",
                "uhmw",
                "pc",
                "cf_plate",
                "fr4",
                "aluminium",
                "steel",
                "titanium",
                "brass",
                "copper",
                "tape",
                "velcro",
                "resin",
            ] {
                if let Some(m) = catalog::material(id) {
                    out!("{id}: {:.2} g/cm3, {:?}, min wall {:.1} mm{}", m.density, m.class, m.min_wall_mm, if m.brittle { ", brittle" } else { "" });
                }
            }
        }
        "templates" => {
            for n in templates::NAMES {
                out!("{n}");
            }
        }
        "template" => {
            let name = pos.first().ok_or("usage: robot template <name>")?;
            let spec = templates::by_name(name).ok_or(format!("unknown template {name}; try `robot templates`"))?;
            out!("{}", serde_json::to_string_pretty(&spec).map_err(|e| e.to_string())?);
        }
        "verify" => {
            let spec = load(pos.first().ok_or("usage: robot verify <spec.json> [--full]")?)?;
            let r = resolve(spec);
            let rep = verify_resolved(&r, &Options::default());
            print_report(&rep, full);
            if !rep.ok {
                std::process::exit(2);
            }
        }
        "render" => {
            let (Some(path), Some(view), Some(out)) = (pos.first(), pos.get(1), pos.get(2)) else {
                return Err("usage: robot render <spec.json> <view> <out.png> [--markers]".into());
            };
            let spec = load(path)?;
            let r = resolve(spec);
            let rep = verify_resolved(&r, &Options::default());
            let world = World::start(&r).with_routes(&rep.routes).with_assembly(&rep.assembly);
            let v = View::named(view, 1100).ok_or(format!("unknown view {view}"))?;
            let mut markers = Vec::new();
            if rest.iter().any(|a| a == "--markers") {
                for f in rep.findings.iter().filter(|f| !f.pass) {
                    if let Some(p) = f.at {
                        markers.push(Marker { at: cadcraft_robot::math::V3::from_arr(p), number: markers.len() + 1 });
                        out!("{}: {} {}", markers.len(), f.id, f.title);
                    }
                }
            }
            std::fs::write(out, render_png(&world, &v, None, &markers)).map_err(|e| e.to_string())?;
            out!("wrote {out}");
        }
        "export" => {
            let (Some(path), Some(slug), Some(dir)) = (pos.first(), pos.get(1), pos.get(2)) else {
                return Err("usage: robot export <spec.json> <slug> <dir>".into());
            };
            let spec = load(path)?;
            let (rep, files) = export::write_all(slug, &spec, Path::new(dir.as_str()), 0.8)?;
            out!("wrote {} files to {dir}/{slug}; ok={} ({} blockers, {} warnings)", files.len(), rep.ok, rep.summary.blockers, rep.summary.warnings);
        }
        other => return Err(format!("unknown command {other}; run `robot guide`")),
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Err(e) = run(&args) {
        let _ = writeln!(std::io::stderr(), "error: {e}");
        std::process::exit(1);
    }
}
