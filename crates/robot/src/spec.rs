//! The robot description an LLM (or a person) writes: parts, fasteners, wires and claims.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::sdf::Shape;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RobotType {
    #[default]
    Wedge,
    HorizontalSpinner,
    VerticalSpinner,
    Drum,
    Lifter,
    Flipper,
    Other,
}

impl RobotType {
    pub fn is_spinner(self) -> bool {
        matches!(self, RobotType::HorizontalSpinner | RobotType::VerticalSpinner | RobotType::Drum)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Locomotion {
    #[default]
    Roller,
    NonRoller,
    Ntl,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PartKind {
    #[default]
    Structure,
    Armor,
    Wedge,
    Motor,
    WeaponMotor,
    Wheel,
    Battery,
    Esc,
    Receiver,
    Servo,
    Switch,
    Connector,
    Weapon,
    LockingBar,
    Insert,
    Nut,
    Bearing,
    Plate,
    Other,
}

impl PartKind {
    pub fn is_electronic(self) -> bool {
        matches!(
            self,
            PartKind::Motor | PartKind::WeaponMotor | PartKind::Battery | PartKind::Esc | PartKind::Receiver | PartKind::Servo | PartKind::Switch
        )
    }
    pub fn is_structural(self) -> bool {
        matches!(self, PartKind::Structure | PartKind::Armor | PartKind::Wedge | PartKind::Plate)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Joint {
    /// `revolute` (degrees) or `prismatic` (mm).
    pub kind: String,
    pub axis: [f64; 3],
    /// Pivot point in the parent frame.
    pub pivot: [f64; 3],
    pub min: f64,
    pub max: f64,
    pub default: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WeaponDef {
    /// `spinner`, `drum`, `lifter`, `flipper`, `wedge`, `none`.
    pub kind: String,
    /// Part id of the weapon motor.
    pub motor: Option<String>,
    pub cells: Option<f64>,
    /// Pulley / gear ratio, output over motor (1 = direct drive).
    pub reduction: Option<f64>,
    /// Spin axis in the weapon part's frame (default local Z).
    pub spin_axis: Option<[f64; 3]>,
    pub supported_both_ends: bool,
    /// Part id of the locking bar.
    pub locking_bar: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Part {
    pub id: String,
    pub kind: PartKind,
    /// Catalog id: dimensions, mass, ports and mount holes then come from the catalog.
    pub catalog: Option<String>,
    pub material: Option<String>,
    /// `fdm`, `resin`, `cnc`, `laser`, `waterjet`, `cast`, `bought`.
    pub process: Option<String>,
    /// Required unless `catalog` supplies the shape.
    pub shape: Option<Shape>,
    pub at: [f64; 3],
    pub rot: [f64; 3],
    /// Kinematic parent: this part's frame moves with it.
    pub parent: Option<String>,
    pub joint: Option<Joint>,
    /// Mass written by the author; counted as unverified.
    pub mass_g: Option<f64>,
    /// Mass from the slicer.
    pub slicer_mass_g: Option<f64>,
    /// Mass from a scale (real part).
    pub scale_mass_g: Option<f64>,
    /// Effective material fraction for printed parts when not hollowed explicitly (default 0.45).
    pub solid_fraction: Option<f64>,
    /// `#rrggbb`.
    pub color: Option<String>,
    pub label: Option<String>,
    /// Switch/link clearly marked on/off.
    pub marked: bool,
    /// Operated without tools.
    pub tool_free: bool,
    /// Has a sharp edge that needs a cover.
    pub sharp: bool,
    pub cover: Option<String>,
    /// How a part with no fastener is kept in place: `tape`, `adhesive`, `press_fit`, `velcro`.
    /// Without it the part must be captive (blocked in every direction by neighbours).
    pub retained: Option<String>,
    /// Printed or bonded as one piece with this part: no overlap check, no retention needed.
    pub fused_with: Option<String>,
    pub weapon: Option<WeaponDef>,
    /// Extra named ports in the part frame.
    pub ports: BTreeMap<String, [f64; 3]>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Fastener {
    pub id: String,
    /// `M2x16`, `M3x10`, `M1.6x6`.
    pub designation: String,
    /// Where the underside of the head sits.
    pub at: [f64; 3],
    /// Direction the shank points.
    pub dir: [f64; 3],
    /// Parts the shank passes through (clearance holes).
    pub through: Vec<String>,
    /// Part that holds the thread: a nut or insert part, or a structure part (self tapping).
    pub threads_into: Option<String>,
    /// `hex1.5`, `hex2`, `phillips`, `torx`.
    pub tool: Option<String>,
    /// Taken out in normal service (battery access, panel removal).
    pub removable: bool,
    /// `socket`, `pan`, `countersunk`.
    pub head: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Wire {
    pub id: String,
    /// `part.port`
    pub from: String,
    pub to: String,
    pub awg: Option<f64>,
    /// `power`, `motor`, `signal`.
    pub kind: String,
    pub slack_mm: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Electronics {
    pub failsafe: bool,
    pub weapon_remote_disable: bool,
    pub radio: Option<String>,
    pub cells: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AllowOverlap {
    pub a: String,
    pub b: String,
    pub max_depth_mm: Option<f64>,
    pub reason: String,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Claim {
    pub id: String,
    pub text: String,
    /// A measured metric name, e.g. `mass_total_g`, `min_wall_mm`, `footprint_mm`.
    pub metric: String,
    /// `<=`, `>=`, `==` (with 5 % tolerance).
    pub op: String,
    pub value: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Spec {
    pub name: String,
    pub profile: String,
    pub robot_type: RobotType,
    pub locomotion: Locomotion,
    pub parts: Vec<Part>,
    pub fasteners: Vec<Fastener>,
    pub wires: Vec<Wire>,
    pub electronics: Electronics,
    pub allow_overlap: Vec<AllowOverlap>,
    pub claims: Vec<Claim>,
    /// Joint id (part id) -> value overriding the default for the start configuration.
    pub start_pose: BTreeMap<String, f64>,
    /// Ground plane height; by default the lowest wheel point.
    pub ground_z: Option<f64>,
    /// Real measured mass per part kind versus predicted, from past builds (e.g. `{"structure": 1.08}`).
    pub calibration: BTreeMap<String, f64>,
}

impl Spec {
    pub fn part(&self, id: &str) -> Option<&Part> {
        self.parts.iter().find(|p| p.id == id)
    }
}
