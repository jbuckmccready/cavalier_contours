# Shape Boolean Fuzz Targets

These harnesses are opt-in and are excluded from the normal workspace. Run them
with `cargo fuzz` from the repository root.

Useful commands:

```sh
cargo install cargo-fuzz
cargo fuzz run shape_boolean_rects
cargo fuzz run shape_boolean_polygons
cargo fuzz run shape_boolean_donuts
cargo fuzz run shape_boolean_arcs
cargo fuzz run shape_boolean_deep_nesting
cargo fuzz run shape_boolean_adversarial_corpus
cargo fuzz run shape_boolean_singularity_corpus
cargo fuzz run shape_transform_then_boolean
cargo fuzz run shape_boolean_vertex_drag_ui
cargo fuzz run shape_boolean_vertex_drag_ui_xor
cargo fuzz run shape_boolean_vertex_drag_rebuilt
cargo fuzz run pline_inversion_view_boolean
```

Replay a minimized case with:

```sh
cargo fuzz run shape_boolean_rects fuzz/artifacts/shape_boolean_rects/<case>
```

Minimized failures should be converted into deterministic regression tests in
`cavalier_contours/tests/test_shape_boolean.rs`. The harnesses assert output
shape validity and sampled set-membership semantics; they treat panics,
invalid loop bins, non-finite coordinates, stale bounds, and sampled semantic
mismatches as failures.
