# cavalier_contours changelog

All notable changes to the cavalier_contours crate will be documented in this file.

## Unreleased

### Added ⭐

- ⚠️ BREAKING: Added collapsed area parameter to pline boolean options to allow for pruning
  collapsed polylines from results. This is only breaking due to struct initialization, if you use
  default initialization this defaults to no change in behavior ([#71](https://github.com/jbuckmccready/cavalier_contours/pull/71)).
- Added `examples` crate to workspace to demonstrate cavalier_contours functionality ([#74](https://github.com/jbuckmccready/cavalier_contours/pull/74)).

### Fixed 🐛

- Fixed bug in pline segment intersection when two arcs only touch at endpoints at one point, have
  the same arc radius and center, and are in opposite directions. This also fixes some cases for
  algorithms that depend on finding interescts (boolean, offset, etc.) ([#71](https://github.com/jbuckmccready/cavalier_contours/pull/71)).

## 0.6.0 2025-07-08

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
