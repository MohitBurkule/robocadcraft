//! Check results.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    /// Illegal, unbuildable or broken: export is refused.
    Blocker,
    /// Probably wrong; read it.
    Warning,
    Info,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Finding {
    /// Stable machine id, e.g. `fastener.no_retention`.
    pub id: String,
    pub group: String,
    pub severity: Severity,
    pub pass: bool,
    pub title: String,
    pub message: String,
    pub expected: Option<String>,
    pub actual: Option<String>,
    pub parts: Vec<String>,
    /// World position in mm for render markers.
    pub at: Option<[f64; 3]>,
    /// What the author should change.
    pub fix: Option<String>,
}

impl Finding {
    pub fn pass(id: &str, group: &str, title: &str, message: impl Into<String>) -> Finding {
        Finding {
            id: id.into(),
            group: group.into(),
            severity: Severity::Info,
            pass: true,
            title: title.into(),
            message: message.into(),
            expected: None,
            actual: None,
            parts: vec![],
            at: None,
            fix: None,
        }
    }
    pub fn fail(id: &str, group: &str, severity: Severity, title: &str, message: impl Into<String>) -> Finding {
        Finding {
            id: id.into(),
            group: group.into(),
            severity,
            pass: false,
            title: title.into(),
            message: message.into(),
            expected: None,
            actual: None,
            parts: vec![],
            at: None,
            fix: None,
        }
    }
    pub fn note(id: &str, group: &str, title: &str, message: impl Into<String>) -> Finding {
        Finding { pass: true, severity: Severity::Info, ..Finding::pass(id, group, title, message) }
    }
    pub fn expect(mut self, expected: impl Into<String>, actual: impl Into<String>) -> Finding {
        self.expected = Some(expected.into());
        self.actual = Some(actual.into());
        self
    }
    pub fn on(mut self, parts: &[&str]) -> Finding {
        self.parts = parts.iter().map(|s| s.to_string()).collect();
        self
    }
    pub fn here(mut self, p: crate::math::V3) -> Finding {
        self.at = Some(p.arr());
        self
    }
    pub fn fix(mut self, hint: impl Into<String>) -> Finding {
        self.fix = Some(hint.into());
        self
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Summary {
    pub blockers: usize,
    pub warnings: usize,
    pub passed: usize,
    pub total: usize,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MassLine {
    pub id: String,
    pub kind: String,
    pub grams: f64,
    /// Plus/minus uncertainty in grams.
    pub tol_g: f64,
    /// `scale`, `slicer`, `catalog`, `computed`, `declared`, `wire`, `fastener`.
    pub source: String,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Report {
    pub name: String,
    pub profile: String,
    /// True when no blocker failed.
    pub ok: bool,
    pub summary: Summary,
    pub findings: Vec<Finding>,
    /// Measured numbers: `mass_total_g`, `footprint_mm`, `min_wall_mm`, ...
    pub metrics: BTreeMap<String, f64>,
    pub mass: Vec<MassLine>,
    /// Things to try on the bench that CAD cannot prove.
    pub bench_checklist: Vec<String>,
    /// Routed wires, thickness included.
    pub routes: Vec<crate::checks::wiring::Route>,
}

impl Report {
    pub fn finish(&mut self) {
        let blockers = self.findings.iter().filter(|f| !f.pass && f.severity == Severity::Blocker).count();
        let warnings = self.findings.iter().filter(|f| !f.pass && f.severity == Severity::Warning).count();
        let passed = self.findings.iter().filter(|f| f.pass).count();
        self.summary = Summary { blockers, warnings, passed, total: self.findings.len() };
        self.ok = blockers == 0;
    }
    pub fn failed(&self, id: &str) -> bool {
        self.findings.iter().any(|f| !f.pass && f.id == id)
    }
    pub fn get(&self, id: &str) -> Option<&Finding> {
        self.findings.iter().find(|f| f.id == id)
    }
}
