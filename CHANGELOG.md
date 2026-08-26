## [unreleased]

### 🚀 Features

- Add auto-tune catch-window lock module (issue 7)
- Add auto-tune tuner core module (issue 6)
- Capture ring, tune guards, and tune skeleton (issue 8)
- DSP adapter and headless end-to-end tune (issue 9)
- Tune display states (issue 10)
- Audition gate for the tune sweep (issue 11)
- Catch-window unlock integration and exit semantics (issues 12 + 13)
- *(contract)* Emit additive sn key in status envelope
- *(serial)* OCOTP serial-identity module + sketch wiring

### 💼 Other

- Merge pull request #4 from qrnix-labs/release/0.3.13

Prepare release v0.3.13
- Merge pull request #15 from qrnix-labs/feat/autotune

feat: auto-tune — tuner core, capture guards, end-to-end tune, catch-window unlock (issues 6-13)
- Merge pull request #21 from qrnix-labs/sn-serial

Serial: report unit serial in the status envelope

### 📚 Documentation

- Document auto-tune flow, serial vocabulary, and gates

### 🧪 Testing

- Unify native test entry via shared Unity runner
## [0.3.13] - 2026-08-25

### 🚀 Features

- Emit ADR-0003 JSON wire contract envelopes (cv=1)

### 🐛 Bug Fixes

- Make release --dry-run a true rehearsal (no commit)
- Revert version string to last shipped v0.3.12

### 💼 Other

- Merge pull request #2 from qrnix-labs/feat/json-wire-contract

feat: emit ADR-0003 JSON wire contract envelopes (cv=1)
- Merge pull request #3 from qrnix-labs/build/release-pr-flow

build: ship releases through a PR (main is branch-protected)

### 🧪 Testing

- Native unit tests for the wire-contract emitter

### 🛠️ Build System

- Add commit-msg hook and document convention
- Add PlatformIO build and native test workflow
- Ship releases through a PR (main is branch-protected)
## [0.3.12] - 2026-08-20

### 🛠️ Build System

- Add cliff.toml
## [0.3.11] - 2026-08-20

### 🛠️ Build System

- One-command release driver
## [0.3.10] - 2026-08-20

### 🐛 Bug Fixes

- Render proper version header in release notes
## [0.3.9] - 2026-08-20

### 🐛 Bug Fixes

- Render proper version header in release notes
## [0.3.8] - 2026-08-20

### 🐛 Bug Fixes

- Correct release digest check and repo extraction
## [0.3.7] - 2026-08-20

### 🛠️ Build System

- Add release script and process docs
## [0.3.5] - 2026-08-19

### 💼 Other

- Add README and MIT license
- Import firmware sources under LGPL 2.1-or-later

- src/ (55 files, sketch renamed to qrnix.cpp, LGPL header added)
- include/ (2 headers), platformio.ini
- LICENSE: LGPL 2.1-or-later (full text)
- .gitignore: PlatformIO build artifacts
- README removed; to be rewritten from scratch
- Brand display strings as QRNix (boot splash and mode labels)
- Add README (developer guide, from scratch) and CONTRIBUTING
