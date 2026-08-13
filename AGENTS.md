# Agent Development Guide

## Commands

- **Format:** `cargo fmt --all`
- **Lint:** `cargo clippy --workspace --all-targets --all-features -- -D warnings`
- **Test:**: `cargo test --workspace --all-features`

## Directory Structure

- `cavalier_contours/` — Core geometry library, integration tests, and Criterion benchmarks.
- `cavalier_contours_ffi/` — C foreign-function interface, header generation config, and FFI tests.
- `cavalier_contours_ui/` — Native and WebAssembly `egui` app for testing and visualizing the library.
- `examples/` — Standalone workspace crate containing runnable API examples.
- `.github/workflows/` — CI and GitHub Pages workflows.

## Visualization and Test APIs

Some `cavalier_contours` functions and types are public only so the UI crate, examples, benchmarks, or integration tests can inspect and display internal results. Treat these as workspace-internal APIs rather than compatibility constraints. Do not preserve an awkward API, add a compatibility wrapper, or avoid a breaking change solely for these callers. Make the simpler core change and update all workspace callers in the same change.

## Benchmarks

Benchmarks use the optimized bench profile and therefore compile separately from debug tests and Clippy. Filter benchmark runs to the group being measured when possible, for example:

```sh
cargo bench -p cavalier_contours --bench benches -- raw_offset_creation
```

For faster iteration default to using a 1-second warm-up and a 3-second measurement time:

```sh
cargo bench -p cavalier_contours --bench benches -- raw_offset_creation --warm-up-time 1 --measurement-time 3
```
