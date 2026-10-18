//! STEP (ISO 10303, AP214 automotive design, advanced B-rep) export of robot bodies.

use truck_modeling::Solid;
use truck_stepio::out::{CompleteStepDisplay, StepHeaderDescriptor, StepModels};

use crate::brep;
use crate::world::World;

/// A named group of solids in world coordinates (mm).
pub struct StepBody {
    pub name: String,
    pub solids: Vec<Solid>,
}

/// B-rep for every body of `world` in its current pose. Identical shapes (bolts, inserts) are
/// built once and built in parallel. Bodies whose geometry cannot be rebuilt are returned as
/// `(name, reason)`; skipped cuts come back as warnings.
pub fn bodies(world: &World) -> (Vec<StepBody>, Vec<(String, String)>, Vec<(String, String)>) {
    let mut keys: Vec<String> = Vec::new();
    let mut uniq: Vec<(&crate::sdf::Shape, &crate::math::Xform)> = Vec::new();
    let key_of: Vec<usize> = world
        .bodies
        .iter()
        .map(|b| {
            let k = format!("{}{:?}", serde_json::to_string(&b.shape).unwrap_or_default(), b.xf);
            keys.iter().position(|x| *x == k).unwrap_or_else(|| {
                keys.push(k);
                uniq.push((&b.shape, &b.xf));
                keys.len() - 1
            })
        })
        .collect();
    let threads = std::env::var("ROBOT_BREP_THREADS").ok().and_then(|v| v.parse().ok()).unwrap_or_else(|| std::thread::available_parallelism().map(|n| n.get()).unwrap_or(2)).min(uniq.len().max(1));
    let next = std::sync::atomic::AtomicUsize::new(0);
    let results: std::sync::Mutex<Vec<Option<(Result<Vec<Solid>, String>, Vec<String>)>>> = std::sync::Mutex::new(vec![None; uniq.len()].into_iter().map(|_: Option<()>| None).collect());
    // Slowest first (largest trees) so one big part does not start last.
    let mut order: Vec<usize> = (0..uniq.len()).collect();
    order.sort_by_key(|&i| std::cmp::Reverse(uniq.get(i).map(|s| serde_json::to_string(s.0).map(|t| t.len()).unwrap_or(0)).unwrap_or(0)));
    std::thread::scope(|sc| {
        for _ in 0..threads {
            sc.spawn(|| {
                loop {
                    let n = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let Some(&i) = order.get(n) else { break };
                    let Some(&(shape, xf)) = uniq.get(i) else { break };
                    let t = std::time::Instant::now();
                    let r = brep::solids_with_notes(shape, xf);
                    eprintln!("[brep] shape {i}: {} in {:.1?}", if r.0.is_ok() { "ok" } else { "FAILED" }, t.elapsed());
                    if let Ok(mut g) = results.lock() {
                        if let Some(slot) = g.get_mut(i) {
                            *slot = Some(r);
                        }
                    }
                }
            });
        }
    });
    let results = results.into_inner().unwrap_or_default();
    let mut ok = Vec::new();
    let mut bad = Vec::new();
    let mut warn = Vec::new();
    for (b, &k) in world.bodies.iter().zip(&key_of) {
        let name = b.id.replace([':', '/'], "_");
        match results.get(k).and_then(|r| r.as_ref()) {
            Some((Ok(s), notes)) => {
                warn.extend(notes.iter().map(|n| (name.clone(), n.clone())));
                ok.push(StepBody { name, solids: s.clone() });
            }
            Some((Err(e), _)) => bad.push((name, e.clone())),
            None => bad.push((name, "not built".into())),
        }
    }
    (ok, bad, warn)
}

/// One STEP file holding `bodies`; each solid is named after its body.
pub fn step_text(title: &str, bodies: &[&StepBody]) -> String {
    let mut names = Vec::new();
    let compressed: Vec<_> = bodies
        .iter()
        .flat_map(|b| {
            names.extend(std::iter::repeat_n(b.name.clone(), b.solids.len()));
            b.solids.iter().map(|s| s.compress())
        })
        .collect();
    let models: StepModels<_, _, _> = compressed.iter().collect();
    let header = StepHeaderDescriptor {
        file_name: format!("{title}.step"),
        time_stamp: String::new(),
        authors: vec!["cadcraft-robot".to_string()],
        organization: vec!["CADCraft".to_string()],
        organization_system: "cadcraft-robot".to_string(),
        authorization: String::new(),
    };
    let text = CompleteStepDisplay::new(models, header).to_string();
    name_solids(&text, &names)
}

/// Fill the empty names of `MANIFOLD_SOLID_BREP('', ..)` entities in order.
fn name_solids(text: &str, names: &[String]) -> String {
    const KEY: &str = "MANIFOLD_SOLID_BREP('',";
    let mut out = String::with_capacity(text.len() + names.len() * 16);
    let mut rest = text;
    let mut it = names.iter();
    while let Some(i) = rest.find(KEY) {
        out.push_str(rest.get(..i).unwrap_or(""));
        let n = it.next().map(|s| s.replace('\'', "")).unwrap_or_default();
        out.push_str(&format!("MANIFOLD_SOLID_BREP('{n}',"));
        rest = rest.get(i + KEY.len()..).unwrap_or("");
    }
    out.push_str(rest);
    out
}
