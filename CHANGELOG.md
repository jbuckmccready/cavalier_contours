# cavalier_contours changelog

All notable changes to the cavalier_contours crate will be documented in this file.

## Unreleased

### Added ⭐
* Added CHANGELOG.md file for tracking changes and releases.
* New traits `PlineSource`, `PlineSourceMut`, and `PlineCreation` for sharing methods across
different polyline data views (for example sub views/selections over polylines or direction
inversion).
* `PlineViewData` and `PlineView` types. `PlineView` implements `PlineSource` trait allowing for
flexible views over polyline data to be processed with all the shared behavior/methods.
`PlineViewData` holds the data used to index into an underlying source polyline to form a
`PlineView`.

### Changed 🔧
* All Polyline methods have moved to the appropriate trait (`PlineSource`, `PlineSourceMut`, or
`PlineCreation`).
* Slices used in offset and boolean operations now all use new `PlineViewData` and `PlineView`
types.
* Changed polyline methods that returned `Cow` types to return `Option` types to indicate if changes
were done on the input polyline or not.
* Changed intersect functions (`circle_circle_intr`, `line_circle_intr`, and `line_line_intr`) to
accept epsilon value parameter for fuzzy float comparisons.
* Changed polyline offset and boolean operations to use `pos_equal_eps` epsilon value in intersect
functions.
* Changed `pline_seg_intr` function to avoid inconsistencies between intersect functions in some
cases involving line-arc intersects due to fuzzy comparing - this fixes problems that can arise in
operations that depend on finding intersects.
* Added `include_overlapping` parameter to `all_self_intersects_as_basic` function.

### Fixed 🐛
* Fixed arc overlap intersects not being found in some cases and segment intersects disagreeing due
to fuzzy comparing causing incorrect output for polyline boolean operations and added test cases
that found the issue.
* Fixed bug in parallel offset caused when raw offset polyline produced an overlapping self
intersect. Parallel offset algorithm no longer includes overlapping intersects when finding self
intersects of raw offset polyline for creating slices.

### Removed 🔥
* `Polyline::visit_segments` (use `PlineSource::iter_segments` instead).
* `Polyline::len` method removed (use `PlineSource::vertex_count` instead).
* `PolylineSlice` trait and `OpenPlineSlice` type (covered by new view types and polyline traits).
* `PolylineSlice::stitch_onto` and `PolylineSlice::to_polyline` removed, to get same functionality
construct the view with `PlineData::view` and pass it to `PlineSourceMut::extend_remove_repeat` or
`PlineCreation::create_from_remove_repeat`.