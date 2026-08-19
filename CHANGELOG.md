# cavalier_contours changelog

All notable changes to the cavalier_contours crate will be documented in this file.

## Unreleased

Focus of this release is on robustness and performance of polyline offset generation.

The main changes are:

- Invalid slices are detected and tracked in raw offset creation to filter out invalid slices.
- Slices are stitched together using intersect topology rather than global geometric queries.

These both improve robustness in dealing with heavily overlapping/degenerate scenarios. Additionally
this removes the need for `slice_join_eps` and made it possible to add options for behavior in
dealing with coincident segments (keep or discard) and "touching loops" (longest continuation
or split) in the offset results.

Notably the intersect topology approach I think has application in the boolean operations to
improve robustness there as well. Also the `Shape` offset algorithm has not yet been updated.

Other improvements are mostly in optimizing some of lower level segment processing functions to
avoid square root and trigonometric functions.

There was significant improvements in performance for inputs that involve lots of arcs or generate
raw offsets with many self intersects points.

Some benchmarks collected before (release 0.8.0) and after (this release):

Negative differences mean this release is faster.

| Case                      | Before (`f5fe179`) | After (`277790c`) |  Difference |
| ------------------------- | -----------------: | ----------------: | ----------: |
| `profile1`                |          319.45 µs |         221.14 µs | **-30.77%** |
| `profile2`                |          673.86 µs |         461.33 µs | **-31.54%** |
| `profile1_no_arcs`        |           4.387 ms |          4.192 ms |  **-4.44%** |
| `profile2_no_arcs`        |           9.270 ms |          8.768 ms |  **-5.41%** |
| `floor_plan`              |          157.83 µs |         159.96 µs |  **+1.35%** |
| `mechanical_bracket`      |          105.23 µs |          92.64 µs | **-11.96%** |
| `road_centerline`         |          823.36 µs |         818.09 µs |  **-0.64%** |
| `bezier_enclosure`        |           3.481 ms |          3.402 ms |  **-2.28%** |
| `involute_gear`           |           4.395 ms |          4.523 ms |  **+2.91%** |
| `involute_gear_with_arcs` |           3.453 ms |          3.363 ms |  **-2.61%** |
| `pathological1`           |          62.825 ms |         18.002 ms | **-71.35%** |
| `pathological1_no_arcs`   |         245.604 ms |         99.504 ms | **-59.49%** |
| `invalid_line_zigzag`     |           9.851 ms |          4.367 ms | **-55.67%** |
| `invalid_line_arc_zigzag` |          19.296 ms |          6.087 ms | **-68.45%** |
| `closed_invalid_runs`     |          12.569 ms |          6.809 ms | **-45.83%** |
| `tapered_link_strip/128`  |          890.35 µs |         728.91 µs | **-18.13%** |
| `tapered_link_strip/512`  |           4.710 ms |          3.503 ms | **-25.63%** |
| `tapered_link_strip/2048` |          20.045 ms |         15.667 ms | **-21.84%** |
| `tapered_link_strip/4096` |          42.864 ms |         34.156 ms | **-20.32%** |

### Added ⭐

- Added `TouchingLoopBehavior` and `CoincidentSegmentBehavior` to polyline offset options. The
  defaults preserve tangent-touching loops and coincident spans. Callers can instead separate
  tangent-touching loops or discard every coincident raw span. The C FFI exposes matching constants
  and option fields.
- Two new dependencies: `ahash` and `smallvec`. These provide meaningful speedups and are both small
  and popular crates.

### Changed 🔧

- ⚠️ BREAKING: Removed `PlineOffsetOptions::slice_join_eps` and the matching
  `cavc_pline_parallel_offset_o` field. Polyline offset slices now connect through explicit
  intersection topology and use `pos_equal_eps` only to clean repeated positions. The separate
  `ShapeOffsetOptions::slice_join_eps` field remains unchanged.
- ⚠️ BREAKING: Replaced `PlineIntersectVisitor::visit_basic_intr` and
  `visit_overlapping_intr` with a single `visit(PlineIntersect)` method. Custom visitors must now
  match the `Basic` and `Overlapping` variants. The trait also has an optional `filter_map` hook.
- ⚠️ BREAKING: Removed the unused public `PlineVertexVisitor` and `PlineSegVisitor` traits.
- ⚠️ BREAKING: Changed `cavc_pline_parallel_offset_o::to_internal` to return `Option` when behavior
  values are invalid. The C `cavc_pline_parallel_offset` function can now return error code `2` for
  an unrecognized behavior value.

### Fixed 🐛

- Fixed repeated polyline offsets dropping a valid small-loop span when two loops touch at a
  tangent point. The default result keeps the full self-touching path, while separate-touch mode
  returns the two closed loops.
- Fixed parallel offsets containing locally inverted source spans, which could pass the
  global distance checks to produce invalid outputs (this is also a significant optimization).
  This fixes the "wifi leaking" pattern with repeated offsets reported
  ([#79](https://github.com/jbuckmccready/cavalier_contours/issues/79)).
- Fixed visible arc distortion in the UI at high zoom by using a more accurate shared arc
  approximation for polyline and raw offset rendering.

### Optimizations ⚡

- Improved polyline offset stitching by following recorded intersections directly instead of
  searching for nearby endpoints or sorting temporary candidate lists.
- Reduced time and memory use for offsets with many intersections or coincident segments.
- Sped up common polyline arc operations, including bounds, closest points, splits, lengths, and
  distance checks.
- Sped up round joins in raw offsets.
- Reused temporary storage when finding open-offset end-circle intersections.
- Sped up arc winding checks by avoiding unneeded center and radius calculations.
- Reduced allocations when finding all self-intersections in a polyline.
- Sped up two-polyline intersection duplicate cleanup by sorting candidate segment indexes instead
  of storing them in hash sets.

### Internal

- Added Criterion benchmarks for polyline area, polyline segment geometry, raw round joins, raw and
  final parallel offset creation, offset topology scaling, and intersection duplicate cleanup.
- Added initial AGENTS.md.
- Refactored raw offset slice validation to share common logic between single and dual raw offsets.
- Marked workspace-only algorithm APIs as hidden from generated documentation.

## 0.8.0 - 2026-08-09

### Changed 🔧

- ⚠️ BREAKING: Normalized C FFI container counts, lengths, capacities, and indexes to `size_t`
  from `uint32_t` or `uintptr_t`, and updated the generated header accordingly.
- ⚠️ BREAKING: Changed `Shape::parallel_offset` to borrow `ShapeOffsetOptions` instead of taking it
  by value.
- ⚠️ BREAKING: Changed `Shape::stitch_slices_together` to borrow a slice of `DissectedSlice` values
  instead of taking a `Vec` by value.
- Changed the public boolean slice pruning helpers under `polyline::internal` to use explicit pruning
  modes, grouped slice boundaries, and a separate intersection lookup helper.
- Added `#[must_use]` to public functions and types, which may affect users of `-D warnings`.
- Updated `static_aabb2d_index` from 2.0 to 2.1.

### Fixed 🐛

- Fixed how position epsilon is applied to line-circle intersections so distinct near-tangent
  intersections are not merged.
- Fixed how position epsilon is applied to arc sweep checks so they behave consistently at different
  scales and small negative offsets are no longer dropped
  ([#82](https://github.com/jbuckmccready/cavalier_contours/issues/82)).
- Improved `parallel_offset` robustness for repeat-position input by sanitizing repeat vertices
  before offsetting.
- Fixed collapsed near-vertex offset slices that could cause a debug panic
  ([#83](https://github.com/jbuckmccready/cavalier_contours/pull/83)).
- Fixed C FFI shape API documentation to reference `shape` parameters and `cavc_shape_create`
  instead of their polyline counterparts.
- Fixed the UI polyline editor failing to detect pending changes when polylines were added or
  removed.

### Internal

- Updated the UI crate to Rust 1.95.0.
- Updated the UI crate to `egui` 0.36.1 and matching support crates.
- Enabled workspace-wide Clippy pedantic lints and fixed the reported warnings.
- Added a checked-in `cbindgen` configuration for C header generation.
- Updated the GitHub Actions checkout and Rust toolchain actions.

## 0.7.0 - 2026-01-02

### Added ⭐

- ⚠️ BREAKING: Added collapsed area parameter to pline boolean options to allow for pruning
  collapsed polylines from results. This is only breaking due to struct initialization, if you use
  default initialization this defaults to no change in behavior ([#71](https://github.com/jbuckmccready/cavalier_contours/pull/71)).
- Added `examples` crate to workspace to demonstrate cavalier_contours functionality ([#74](https://github.com/jbuckmccready/cavalier_contours/pull/74)).

### Fixed 🐛

- Fixed bug in pline segment intersection when two arcs only touch at endpoints at one point, have
  the same arc radius and center, and are in opposite directions. This also fixes some cases for
  algorithms that depend on finding interescts (boolean, offset, etc.) ([#71](https://github.com/jbuckmccready/cavalier_contours/pull/71)).
- Fixed offset slice stitching to use consistent epsilon (`join_eps`) when removing repeat vertices,
  preventing tiny segments at slice boundaries when offsetting polylines with close vertices
  ([#77](https://github.com/jbuckmccready/cavalier_contours/issues/77)).

## 0.6.0 - 2025-07-08

### Added ⭐

- Added `egui` interactive demo UI crate and auto deployment to GitHub pages for the demo
  [page is here](https://www.cavaliercontours.dev/).
- Added `visit_intersects`, `contains`, and `scan_for_self_intersect` to pline traits ([#68](https://github.com/jbuckmccready/cavalier_contours/pull/68)).
- Added `user_data` to traits for tracking data through operations ([#63](https://github.com/jbuckmccready/cavalier_contours/pull/63)).
- Added multi polyline offset algorithm to c ffi ([#63](https://github.com/jbuckmccready/cavalier_contours/pull/63)).
- Added more doc comments/tests for `PlineSource` and `PlineSourceMut`.
- Added `README.md` file to `cavalier_contours_ffi` crate.

### Changed 🔧

- ⚠️ BREAKING: Updated MSRV to 1.88 and Rust edition 2024.
  Only breaking if unable to compile with Rust 1.88 or later.
- Refactored multipolyline offset algorithm to be step-by-step.
- Simplified `prune_slices` function in pline_boolean.
- Refactored two-polyline intersection visitation to use visitor pattern and
  eliminated allocation inside loop ([#68](https://github.com/jbuckmccready/cavalier_contours/pull/68)).

### Fixed 🐛

- ⚠️ BREAKING: Fix memory leak in cavc_plinelist by implementing Drop trait ([#64](https://github.com/jbuckmccready/cavalier_contours/pull/64)).
  This is not likely breaking for most users but if you are calling `cavc_pline_f` on each pline
  in a `cavc_plinelist` without removing them from the list then you will get a double free on
  the plines not removed when the `cavc_plinelist` is freed/dropped.
- Improved offset slice validation by checking multiple segment midpoints ([#69](https://github.com/jbuckmccready/cavalier_contours/pull/69)). Fixes bug
  reported in issue [#66](https://github.com/jbuckmccready/cavalier_contours/issues/66).

## 0.5.0 - 2025-07-08

- `cargo release` got so excited it jumped a version! Nothing to see here...

## 0.4.0 - 2024-02-21

### Added ⭐

- Added `Shape` type and parallel offset method on shape type to perform simultaneous multi/island
  polyline parallel offsetting ([#7](https://github.com/jbuckmccready/cavalier_contours/issues/7)).

### Changed 🔧

- ⚠️ BREAKING: Removed `slice_join_eps` from `PlineBooleanOptions`. This is breaking for any code
  that interacts with this option struct directly (does not use defaults). `slice_join_eps` was
  also removed from the equivalent `cavc_pline_boolean_o` struct in the C FFI.
  ([#31](https://github.com/jbuckmccready/cavalier_contours/pull/31))
- ⚠️ BREAKING: `PlineSource::create_aabb_index` and `PlineSource::create_approx_aabb_index` now
  just return a `StaticAABB2DIndex` rather than an `Option<StaticAABB2DIndex>`. The equivalent C api
  calls `cavc_pline_create_aabbindex` and `cavc_pline_create_approx_aabbindex` also no longer
  return an error code of 2 when polyline has less than 2 vertexes (empty aabb index is returned).
  ([#29](https://github.com/jbuckmccready/cavalier_contours/pull/29))
- Updated `StaticAABB2DIndex` dependency to version 2.0.
- Bumped rust edition to 2021.
- Use `with_capacity` instead of `reserve` in some places to avoid over allocation behavior of Vec
  (minor performance improvement).

### Fixed 🐛

- Fixed `PlineView::from_slice_points` to handle wrapping on same segment
  ([#28](https://github.com/jbuckmccready/cavalier_contours/pull/28)).
- Fixed `line_circle_intr` accuracy in cases with nearly vertical line
  ([#30](https://github.com/jbuckmccready/cavalier_contours/pull/30)).

## 0.3.0 - 2023-02-18

### Added ⭐

- Added `BooleanResultInfo` enum used for new `result_info` field on `BooleanResult`. The enum
  is used to return information about what happened during the boolean operation.
- Added doc tests and improved documentation to intersect functions `circle_circle_intr`,
  `line_circle_intr`, and `line_line_intr`
  ([#18](https://github.com/jbuckmccready/cavalier_contours/pull/18)).
- Added `#![forbid(unsafe_code)]` to main `cavalier_contours` crate to ensure no use of unsafe
  (obviously does not apply to the `cavalier_contours_ffi` crate).

### Changed 🔧

- Added epsilon parameter to `PlineSource::closest_point` method to allow for consistency with other
  calculations using epsilon values for fuzzy comparing.
- `line_line_intr` function now scales parametric t values by segment lengths before fuzzy comparing
  with epsilon value to avoid introduced error when line segments are very long or very short.
- `pline_seg_intr` function now scales parametric t values by segment lengths and angle values by
  arc radii before fuzzy comparing with epsilon value to avoid introduced error when line segments
  are very long or very short, or arc radius is very small or very large.
- Reimplemented `line_circle_intr` function for improved numerical stability.
- Added epsilon parameter to `parametric_from_point` function to allow consistency with other
  calculations using epsilon values for fuzzy comparing.
- Reduce error in calculation done in `parametric_from_point` by using larger component difference
  rather than explicitly checking for vertical line case.
- Added epsilon parameter to `point_within_arc_sweep` function to allow for consistency with other
  calculations using epsilon values for fuzzy comparing.
- Added `#[inline]` attribute to all of the small Vector2 and base math functions.
- Updated `static_aabb2d_index` crate to 0.7.0 (for use of `IndexableNum::total_cmp`).
- Replaced all uses for `PartialOrd::partial_cmp` with `IndexableNum::total_cmp` to avoid panics
  when `partial_cmp` returns `None` (e.g., if float is a NaN) and to eliminate branch in compare.

### Fixed 🐛

- Fixed `PlineSource::remove_redundant` to use epsilon values in all fuzzy compares. Previously
  used default epsilon value from Num trait in some comparisons. If epsilon value passed in is less
  than default Num trait value (`1e-8` for `f32` and `f64` values) then it is more aggressive in
  removing vertexes.
- Fixed pline `find_intersects` function to use epsilon parameter in all cases.
- Fixed boolean operation bugs caused by `line_circle_intr` numerical stability problem,
  inconsistencies between epsilon values used across functions, and lack of scaling parametric t
  values and angles for fuzzy comparing with epsilon values
  ([#23](https://github.com/jbuckmccready/cavalier_contours/issues/23)).
- Fixed some of the doc comments around pline boolean operation types.
- Fixed polyline find_intersects to use pos_equal_eps passed in options for querying bounding boxes
  ([#22](https://github.com/jbuckmccready/cavalier_contours/pull/22)).
- Fixed `PlineViewData::from_new_start` to not discard bulge value if start point lies on top of the
  first vertex of a closed polyline, and added tests to cover case.
- Fixed pline `remove_redundant` panic when there was more than 2 equal points at start
  ([#26](https://github.com/jbuckmccready/cavalier_contours/pull/26)).

## 0.2.0 - 2022-05-12

### Added ⭐

- Added CHANGELOG.md file for tracking changes and releases.
- New traits `PlineSource`, `PlineSourceMut`, and `PlineCreation` for sharing methods across
  different polyline data views (for example sub views/selections over polylines or direction
  inversion).
- `PlineViewData` and `PlineView` types. `PlineView` implements `PlineSource` trait allowing for
  flexible views over polyline data to be processed with all the shared behavior/methods.
  `PlineViewData` holds the data used to index into an underlying source polyline to form a
  `PlineView`.

### Changed 🔧

- All Polyline methods have moved to the appropriate trait (`PlineSource`, `PlineSourceMut`, or
  `PlineCreation`).
- Slices used in offset and boolean operations now all use new `PlineViewData` and `PlineView`
  types.
- Changed polyline methods that returned `Cow` types to return `Option` types to indicate if changes
  were done on the input polyline or not.
- Changed intersect functions (`circle_circle_intr`, `line_circle_intr`, and `line_line_intr`) to
  accept epsilon value parameter for fuzzy float comparisons.
- Changed polyline offset and boolean operations to use `pos_equal_eps` epsilon value in intersect
  functions.
- Changed `pline_seg_intr` function to avoid inconsistencies between intersect functions in some
  cases involving line-arc intersects due to fuzzy comparing - this fixes problems that can arise in
  operations that depend on finding intersects.
- Added `include_overlapping` parameter to `all_self_intersects_as_basic` function.

### Fixed 🐛

- Fixed arc overlap intersects not being found in some cases and segment intersects disagreeing due
  to fuzzy comparing causing incorrect output for polyline boolean operations and added test cases
  that found the issue.
- Fixed bug in parallel offset caused when raw offset polyline produced an overlapping self
  intersect. Parallel offset algorithm no longer includes overlapping intersects when finding self
  intersects of raw offset polyline for creating slices.

### Removed 🔥

- `Polyline::visit_segments` (use `PlineSource::iter_segments` instead).
- `Polyline::len` method removed (use `PlineSource::vertex_count` instead).
- `PolylineSlice` trait and `OpenPlineSlice` type (covered by new view types and polyline traits).
- `PolylineSlice::stitch_onto` and `PolylineSlice::to_polyline` removed, to get same functionality
  construct the view with `PlineData::view` and pass it to `PlineSourceMut::extend_remove_repeat` or
  `PlineCreation::create_from_remove_repeat`.
