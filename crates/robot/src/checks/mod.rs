//! The verification pipeline: one function per group of checks, all feeding one `Report`.

use std::collections::BTreeMap;

use crate::finding::{Finding, Report};
use crate::spec::Spec;
use crate::world::{Resolved, World, resolve};

pub mod assembly;
pub mod connections;
pub mod fasteners;
pub mod ground;
pub mod interference;
pub mod legal;
pub mod mass;
pub mod mfg;
pub mod power;
pub mod validity;
pub mod wiring;

#[derive(Clone, Debug)]
pub struct Options {
    /// Sampling step in mm for volumes and overlaps.
    pub step_mm: f64,
    /// Run the slow sweeps (joint ranges, wiring routes, disassembly).
    pub deep: bool,
    /// Treat estimated catalog masses as unverified in the pass/fail decision.
    pub strict_catalog: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options { step_mm: 0.4, deep: true, strict_catalog: false }
    }
}

/// Everything a check needs: the resolved spec, the start-pose world and a place to put metrics.
pub struct Ctx<'a> {
    pub r: &'a Resolved,
    pub world: &'a World,
    pub opts: &'a Options,
    pub metrics: &'a mut BTreeMap<String, f64>,
    pub routes: Vec<wiring::Route>,
    /// Parts that cannot move in any direction, with the parts that block them (filled by `interference`).
    pub captive: BTreeMap<String, Vec<String>>,
    pub assembly: Vec<assembly::AssemblyStep>,
}

pub fn verify(spec: Spec, opts: &Options) -> Report {
    let r = resolve(spec);
    verify_resolved(&r, opts)
}

pub fn verify_resolved(r: &Resolved, opts: &Options) -> Report {
    let mut rep = Report { name: r.spec.name.clone(), profile: r.profile.name.clone(), ..Report::default() };
    let mut findings: Vec<Finding> = Vec::new();
    let valid = validity::run(r, &mut findings);
    let mut metrics = BTreeMap::new();
    if valid {
        let world = World::start(r);
        {
            let mut ctx = Ctx { r, world: &world, opts, metrics: &mut metrics, routes: Vec::new(), captive: BTreeMap::new(), assembly: Vec::new() };
            if opts.deep {
                let t0 = std::time::Instant::now();
                findings.extend(wiring::run(&mut ctx));
                if std::env::var("ROBOT_TIMING").is_ok() {
                    eprintln!("wiring {:?}", t0.elapsed());
                }
            }
            let (mass_lines, mut f) = mass::run(&mut ctx);
            rep.mass = mass_lines;
            findings.append(&mut f);
            let t0 = std::time::Instant::now();
            findings.extend(interference::run(&mut ctx));
            if std::env::var("ROBOT_TIMING").is_ok() {
                eprintln!("interference {:?}", t0.elapsed());
            }
            let t0 = std::time::Instant::now();
            findings.extend(connections::run(&mut ctx));
            if std::env::var("ROBOT_TIMING").is_ok() {
                eprintln!("connections {:?}", t0.elapsed());
            }
            findings.extend(ground::run(&mut ctx));
            findings.extend(legal::run(&mut ctx));
            findings.extend(fasteners::run(&mut ctx));
            findings.extend(power::run(&mut ctx));
            findings.extend(mfg::run(&mut ctx));
            if opts.deep {
                let t0 = std::time::Instant::now();
                findings.extend(assembly::run(&mut ctx));
                if std::env::var("ROBOT_TIMING").is_ok() {
                    eprintln!("assembly {:?}", t0.elapsed());
                }
            }
            findings.extend(validity::claims(&mut ctx));
            rep.routes = std::mem::take(&mut ctx.routes);
            rep.assembly = std::mem::take(&mut ctx.assembly);
        }
    }
    rep.findings = findings;
    rep.metrics = metrics;
    rep.bench_checklist = crate::review::bench_checklist(r);
    rep.finish();
    rep
}
