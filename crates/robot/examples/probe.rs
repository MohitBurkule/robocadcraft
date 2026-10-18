use cadcraft_robot::math::v3;
use cadcraft_robot::templates;
use cadcraft_robot::world::{World, resolve};
fn main() {
    let Some(spec) = templates::by_name("wedge") else { return };
    let r = resolve(spec);
    let w = World::start(&r);
    let args: Vec<f64> = std::env::args().skip(1).filter_map(|a| a.parse().ok()).collect();
    if let [x0, y0, z0, x1, y1, z1] = args[..] {
        let (a, b) = (v3(x0, y0, z0), v3(x1, y1, z1));
        for k in 0..=30 {
            let p = a + (b - a) * (k as f64 / 30.0);
            let mut best = (f64::MAX, String::new());
            for (i, bd) in w.bodies.iter().enumerate() {
                let d = w.sdf(i, p);
                if d < best.0 {
                    best = (d, bd.id.clone());
                }
            }
            println!("{:6.1} {:6.1} {:5.1}  sdf {:6.2} {}", p.x, p.y, p.z, best.0, best.1);
        }
    }
}
