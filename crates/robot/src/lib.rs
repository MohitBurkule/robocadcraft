//! CADCraft robot verifier.
//!
//! Describes a 150 g antweight combat robot as solids (signed distance fields) plus a parts
//! catalog, fasteners and wires, then verifies it with layered checks: rules, numbers measured
//! from the model, kinematic sweeps, wiring routes, assembly access, simple physics and renders.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

pub mod catalog;
pub mod checks;
pub mod export;
pub mod finding;
pub mod math;
pub mod measure;
pub mod profile;
pub mod render;
pub mod review;
pub mod sdf;
pub mod spec;
pub mod templates;
pub mod world;
