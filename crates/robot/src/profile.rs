//! Rule sets. `aws2025` is the Antweight World Series ruleset (version 05.06.25).

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Profile {
    pub name: String,
    pub roller_mass_g: f64,
    pub non_roller_mass_g: f64,
    /// Side of the square floor space, no overhang at any height.
    pub footprint_mm: f64,
    pub max_tip_speed_mph: f64,
    pub max_cell_v: f64,
    pub require_locking_bar: bool,
    pub require_tool_free_switch: bool,
    /// Overlap deeper than this is a collision; shallower counts as touching.
    pub contact_tol_mm: f64,
    pub min_clearance_moving_mm: f64,
    /// Allowed wedge-tip height above the floor.
    pub wedge_gap_mm: [f64; 2],
    pub wedge_angle_deg: [f64; 2],
    pub min_wall_fdm_mm: f64,
    pub min_wall_tpu_mm: f64,
    pub min_wall_resin_mm: f64,
    /// Warn when the mass range reaches this fraction of the limit.
    pub mass_warn_frac: f64,
    pub engagement_d: f64,
    pub driver_access_radius_mm: f64,
    pub driver_access_len_mm: f64,
    pub wire_bend_x_dia: f64,
    /// Glue, paint, tape and solder not modelled.
    pub allowance_g: f64,
    /// A part is "captive" when it cannot move more than this in any axis direction.
    pub capture_slop_mm: f64,
    pub min_ground_clearance_mm: f64,
    pub bed_mm: [f64; 3],
    pub fight_s: f64,
    pub cell_full_v: f64,
}

impl Default for Profile {
    fn default() -> Self {
        Profile {
            name: "aws2025".into(),
            roller_mass_g: 150.0,
            non_roller_mass_g: 225.0,
            footprint_mm: 150.0,
            max_tip_speed_mph: 600.0,
            max_cell_v: 4.2,
            require_locking_bar: true,
            require_tool_free_switch: true,
            contact_tol_mm: 0.08,
            min_clearance_moving_mm: 0.4,
            wedge_gap_mm: [0.2, 1.5],
            wedge_angle_deg: [6.0, 30.0],
            min_wall_fdm_mm: 0.9,
            min_wall_tpu_mm: 1.2,
            min_wall_resin_mm: 0.8,
            mass_warn_frac: 0.98,
            engagement_d: 1.5,
            driver_access_radius_mm: 2.6,
            driver_access_len_mm: 14.0,
            wire_bend_x_dia: 3.0,
            allowance_g: 1.2,
            capture_slop_mm: 1.2,
            min_ground_clearance_mm: 1.0,
            bed_mm: [180.0, 180.0, 180.0],
            fight_s: 120.0,
            cell_full_v: 4.2,
        }
    }
}

impl Profile {
    pub fn named(name: &str) -> Profile {
        let mut p = Profile::default();
        if !name.is_empty() {
            p.name = name.to_string();
        }
        p
    }
    pub fn mass_limit(&self, roller: bool) -> f64 {
        if roller { self.roller_mass_g } else { self.non_roller_mass_g }
    }
}
