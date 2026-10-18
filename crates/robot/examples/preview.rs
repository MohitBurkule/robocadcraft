use cadcraft_robot::checks::{Options, verify};
use cadcraft_robot::render::{View, render_png};
use cadcraft_robot::templates;
use cadcraft_robot::world::{World, resolve};

fn main() {
    let name = std::env::args().nth(1).unwrap_or_else(|| "wedge".into());
    let out = std::env::args().nth(2).unwrap_or_else(|| "/tmp/robot".into());
    let _ = std::fs::create_dir_all(&out);
    let Some(spec) = templates::by_name(&name) else { return };
    let t = std::time::Instant::now();
    let rep = verify(spec.clone(), &Options::default());
    println!("verify {:?}", t.elapsed());
    println!("{}", serde_json::to_string_pretty(&rep.metrics).unwrap_or_default());
    for l in &rep.mass {
        println!("{:>16} {:>7.2} g +-{:.2} {}", l.id, l.grams, l.tol_g, l.source);
    }
    for f in rep.findings.iter().filter(|f| !f.pass) {
        println!("[{:?}] {} {} :: {} at {:?}", f.severity, f.id, f.title, f.message, f.at);
    }
    println!("summary {:?} ok={}", rep.summary, rep.ok);
    let r = resolve(spec);
    let w = World::start(&r).with_routes(&rep.routes);
    for v in std::env::args().skip(3) {
        let Some(view) = View::named(&v, 900) else { continue };
        let t = std::time::Instant::now();
        let png = render_png(&w, &view, None, &[]);
        let _ = std::fs::write(format!("{out}/{v}.png"), png);
        println!("render {v} {:?}", t.elapsed());
    }
}
