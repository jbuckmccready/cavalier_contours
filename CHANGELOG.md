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

### Removed 🔥
* `Polyline::visit_segments` (use `PlineSource::iter_segments` instead).
* `Polyline::len` method removed (use `PlineSource::vertex_count` instead).
* `PolylineSlice` trait and `OpenPlineSlice` type (covered by new view types and polyline traits).
* `PolylineSlice::stitch_onto` and `PolylineSlice::to_polyline` removed, to get same functionality
construct the view with `PlineData::view` and pass it to `PlineSourceMut::extend_remove_repeat` or
`PlineCreation::create_from_remove_repeat`.