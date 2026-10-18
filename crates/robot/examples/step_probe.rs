use cadcraft_robot::{step, templates, world};
use std::time::Instant;

fn main() {
    let name = std::env::args().nth(1).unwrap_or_else(|| "wedge_2wd".into());
    let Some(spec) = templates::by_name(&name) else { return };
    let r = world::resolve(spec);
    let w = world::World::start(&r);
    let t = Instant::now();
    let (ok, bad, warn) = step::bodies(&w);
    eprintln!("bodies {:?}: {} ok, {} bad, {} warn", t.elapsed(), ok.len(), bad.len(), warn.len());
    for (n, e) in bad.iter().chain(warn.iter()) {
        eprintln!("  {n}: {e}");
    }
    for b in &ok {
        let t = Instant::now();
        let s = step::step_text(&b.name, &[b]);
        eprintln!("{:20} text {:>8.2?} {} KB", b.name, t.elapsed(), s.len() / 1024);
    }
}
