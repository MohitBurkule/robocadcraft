use cadcraft_robot::{export, templates};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(name), Some(dir)) = (args.first(), args.get(1)) else {
        eprintln!("usage: export <template> <out_dir> [mesh_step_mm]");
        return;
    };
    let step = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.8);
    let Some(spec) = templates::by_name(name) else {
        eprintln!("unknown template {name}");
        return;
    };
    match export::write_all(name, &spec, std::path::Path::new(dir), step) {
        Ok((rep, files)) => println!("ok={} {} files, {:?}", rep.ok, files.len(), rep.summary),
        Err(e) => eprintln!("export failed: {e}"),
    }
}
