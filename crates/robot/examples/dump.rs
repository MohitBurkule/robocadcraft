use cadcraft_robot::{templates, world};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let Some(spec) = templates::by_name("wedge_2wd") else { return };
    let r = world::resolve(spec);
    let w = world::World::start(&r);
    for b in &w.bodies {
        if Some(&b.id) == a.get(1) {
            println!("{}", serde_json::to_string(&b.shape).unwrap_or_default());
        }
    }
}
