#!/bin/bash
# Pre-fetch and pre-build Rust deps so cold cloud sessions start warm.
set -euo pipefail
[ "${CLAUDE_CODE_REMOTE:-}" = "true" ] || exit 0
cd "$CLAUDE_PROJECT_DIR"
cargo fetch --locked || cargo fetch
# Robot verifier CLI: release (what threads run) and fast (probe iteration).
cargo build --release -p cadcraft-robot --bin robot
cargo build --profile fast -p cadcraft-robot --bin robot
