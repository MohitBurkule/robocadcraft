use cadcraft_robot::math::V3;
use cadcraft_robot::measure;
use cadcraft_robot::templates;
use cadcraft_robot::world::{World, resolve};
fn main() {
    let Some(spec) = templates::by_name("wedge") else { return };
    let r = resolve(spec);
    let w = World::start(&r);
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (Some(a), Some(b)) = (args.first().and_then(|x| w.index(x)), args.get(1).and_then(|x| w.index(x))) else { return };
    let pa = measure::surface_points(&w, a, 0.8);
    let pb = measure::surface_points(&w, b, 0.8);
    println!("pts {} {}", pa.len(), pb.len());
    let mut worst = (0.0, V3::ZERO, "a");
    for p in &pa {
        let d = -w.sdf(b, *p);
        if d > worst.0 {
            worst = (d, *p, "a-in-b");
        }
    }
    for p in &pb {
        let d = -w.sdf(a, *p);
        if d > worst.0 {
            worst = (d, *p, "b-in-a");
        }
    }
    println!("worst {:?}", worst);
    let q = worst.1;
    println!("sdf a {:.3} b {:.3}", w.sdf(a, q), w.sdf(b, q));
    for dz in [-0.5, -0.2, 0.0, 0.2, 0.5] {
        let p = q + V3 { x: 0.0, y: 0.0, z: dz };
        println!("dz {dz}: a {:.3} b {:.3}", w.sdf(a, p), w.sdf(b, p));
    }
}
#[allow(dead_code)]
fn unused() {}
