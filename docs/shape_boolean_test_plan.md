# Shape Boolean Test Suite Plan

This plan targets the experimental shape boolean implementation on the
`shape-boolean-transformation` branch. The implementation composes existing
polyline booleans across `Shape::{ccw_plines,cw_plines}`, uses
`PlineInversionView` for holes, tracks pairwise loop participation, and then
reassembles final loops. The tests should therefore be hostile to orientation,
holes, containment, duplicate loop emission, pairwise aggregation, and numeric
edge cases.

Implementation anchors:

- Shape-level work delegates contour clipping to the crate's
  [`PlineSource::boolean`](../cavalier_contours/src/polyline/traits.rs) path.
- Hole handling depends on
  [`PlineInversionView`](../cavalier_contours/src/polyline/pline_inversion_view.rs)
  because clockwise shape loops are inverted before lower-level boolean calls.
- Shape membership checks should mirror the crate's non-zero winding behavior
  from [`PlineSource::winding_number`](../cavalier_contours/src/polyline/traits.rs).

## 1. Build Shape-Level Test Utilities First

- Add a reusable `shape_test_utils` layer for shape boolean tests.
- Validate every output shape:
  - every loop is closed;
  - every CCW loop has positive area;
  - every CW loop has negative area;
  - no loop contains adjacent repeat-position vertices;
  - every loop has finite coordinates and finite bulges;
  - each loop spatial index matches that loop's extents;
  - the top-level shape index matches the union of child loop bounds;
  - empty shapes have an empty top-level index.
- Add `shape_signed_area(shape)` as the main semantic metric:
  - sum CCW loop areas;
  - add CW loop areas, which should already be negative.
- Add a stable `shape_signature(shape)`:
  - CCW loop count;
  - CW loop count;
  - signed area;
  - absolute perimeter sum;
  - extents;
  - sorted per-loop polyline property set.
- Add assertion helpers:
  - `assert_shape_valid`;
  - `assert_shape_area_eq`;
  - `assert_shape_extents_eq`;
  - `assert_shape_equiv`;
  - `assert_no_duplicate_loops`;
  - `assert_boolean_samples`.

## 2. Use Membership Sampling as the Semantic Oracle

- Do not rely only on vertex counts or exact resulting geometry.
- Evaluate expected set membership from the original inputs using winding
  numbers.
- For each sample point:
  - union expects `in_a || in_b`;
  - intersection expects `in_a && in_b`;
  - difference expects `in_a && !in_b`;
  - xor expects `in_a != in_b`.
- Sample:
  - a deterministic grid over the combined extents;
  - input vertices nudged in four diagonal directions;
  - segment midpoints nudged along small offsets;
  - result vertices and result extents;
  - hand-picked points for each golden case;
  - fuzz-generated random points.
- Print the failing sample point, operation, expected membership, actual
  membership, shape summaries, and copy-pasteable polyline literals on failure.

## 3. Port Polyline Boolean Discipline to Shape Boolean

- Existing polyline boolean tests already exercise orientation inversion and
  cyclic start-vertex changes. Shape tests should do the same.
- For each deterministic case, run variants:
  - original loop order;
  - reversed loop order;
  - cyclically shifted start vertex for each loop;
  - inverted source polyline direction before `Shape::from_plines`;
  - swapped operands for symmetric operations;
  - reordered islands and holes.
- Assert algebraic identities:
  - `A union B == B union A`;
  - `A intersection B == B intersection A`;
  - `A xor B == B xor A`;
  - `A difference empty == A`;
  - `empty difference A == empty`;
  - `A difference A == empty`;
  - `A xor A == empty`;
  - `(A difference B) intersection B == empty`;
  - `(A difference B) union (A intersection B) == A`, checked by sampling.

## 4. Deterministic Golden Cases

- Empty vs empty.
- Empty vs non-empty.
- Disjoint rectangles.
- Identical rectangles.
- One rectangle fully inside another.
- Rectangles touching at one point.
- Rectangles touching along a whole edge.
- Rectangles partially sharing an edge.
- Thin sliver overlap.
- Epsilon-separated and epsilon-overlapping rectangles.
- T-junction edge cases.
- Plus-sign and cross-shaped overlaps.
- L-shape against rectangle.
- Multiple disjoint islands in both operands.
- Circle vs rectangle.
- Circle vs circle:
  - disjoint;
  - tangent;
  - overlapping;
  - identical;
  - contained.
- Capsule/pill overlaps.
- Arc endpoint exactly on another segment.
- Arc endpoint almost on another segment.
- Arc-arc tangent.
- Arc-arc overlap.
- Very small arcs.
- Bulges near `0`, near `1`, greater than `1`, and negative.

## 5. Hole-Specific Cases

- Donut union empty.
- Donut intersection empty.
- Donut difference empty.
- Donut xor empty.
- Donut union solid square outside the donut.
- Donut union solid square wholly inside the hole.
- Donut union solid square overlapping the hole boundary.
- Donut intersection square wholly inside filled ring.
- Donut intersection square wholly inside the hole.
- Donut difference square cutting through the outer ring.
- Donut difference square wholly inside the hole.
- Donut xor square wholly inside the hole.
- Two disjoint donuts.
- Two overlapping donuts.
- Donut hole overlapping another shape's filled region.
- Nested rings: outer CCW, hole CW, island CCW inside the hole.
- Adjacent holes touching at one point.
- Adjacent holes sharing an edge.
- Multiple holes where only one participates in the operation.

These cases directly target the pairwise `ccw`/`cw` boolean loops and unused-loop
retention logic.

## 6. Multi-Loop Aggregation Cases

- One shape has two islands; the other has one large loop overlapping both.
- One shape has one island; the other has two loops both overlapping it.
- Chain overlap: `A1` overlaps `B1`, `B1` overlaps `A2`, but `A1` and `A2` do
  not touch.
- Three or more loops whose pairwise results must merge into one final
  component.
- One operand contains two disjoint loops of the other operand.
- A shape with a hole and a separate island booleaned against a shape touching
  both.
- Assert no duplicate coincident loops and no area inflation.

## 7. Property-Based Tests with `proptest`

- Add `proptest` as a dev dependency.
- Use the `proptest!` macro and shrinking model documented by the
  [`proptest` crate](https://docs.rs/proptest/latest/proptest/).
- Start with polygon-only generators, then add arc generators after the straight
  segment cases are stable.
- Generators:
  - axis-aligned rectangles;
  - relationship-biased rectangle pairs;
  - convex polygons from sorted polar angles;
  - star-like simple polygons with bounded radial variation;
  - donuts from outer and inner rectangles/circles;
  - multi-island shapes from non-overlapping components;
  - capsules with line and arc segments.
- Bias generated cases toward:
  - shared coordinates;
  - nearly equal coordinates;
  - tangent boundaries;
  - endpoint-on-segment;
  - very small gaps;
  - thin slivers;
  - coincident edges;
  - duplicated reversed loops;
  - holes close to outer boundaries.
- Properties:
  - identities with empty and self;
  - commutativity for union, intersection, xor;
  - difference decomposition;
  - area conservation identities;
  - translation invariance;
  - uniform scaling invariance;
  - rotation invariance by sampled membership;
  - mirror invariance of absolute area.

## 8. Fuzzing

- Add `cargo-fuzz` targets after deterministic and `proptest` infrastructure is
  in place.
- Use the Rust Fuzz Book's
  [`cargo-fuzz` workflow](https://rust-fuzz.github.io/book/cargo-fuzz.html) for
  target setup, corpus replay, and sanitizer-backed runs.
- Fuzz targets:
  - `shape_boolean_rects`;
  - `shape_boolean_polygons`;
  - `shape_boolean_donuts`;
  - `shape_boolean_arcs`;
  - `shape_transform_then_boolean`;
  - `pline_inversion_view_boolean`.
- Treat these as failures:
  - panic;
  - debug assertion;
  - NaN or infinite output;
  - invalid orientation bin;
  - stale or incorrect spatial index;
  - duplicate coincident loops;
  - sampled semantic mismatch.
- Every minimized fuzz case should emit a copy-pasteable Rust regression test.

## 9. Differential Testing

- For straight-line polygon-only shapes, compare against an independent oracle.
- Preferred options:
  - `geo`/`geo-booleanop` in ignored or nightly tests, with `geo`'s
    [`BooleanOps`](https://docs.rs/geo/latest/geo/algorithm/bool_ops/trait.BooleanOps.html)
    as the current first-choice Rust oracle;
  - dense raster or point-sampling oracle for normal tests;
  - exact known golden expectations for small hand-authored cases.
- Keep oracle tests separate from arc tests unless the oracle can represent arcs
  accurately.

## 10. Debugging and Trace Support

- Add failure dumping behind `CAVC_DUMP_FAILURE=/path`.
- On failure, emit:
  - `input_a.svg`;
  - `input_b.svg`;
  - `result.svg`;
  - `overlay.svg`;
  - sample mismatch visualization;
  - `case.rs` reproduction.
- Add optional trace logging behind `CAVC_TRACE_SHAPE_BOOLEAN=1`:
  - input loop bins;
  - per-loop orientation and area;
  - pairwise bbox-overlap matrix;
  - actual intersection counts;
  - lower-level polyline boolean `result_info`;
  - used-loop bookkeeping;
  - final loop assembly.

## 11. Regression Policy

- Every found bug becomes a deterministic named test.
- Naming convention:
  - `reported_shape_boolean_<category>_<short_reason>_<n>`.
- Each regression stores:
  - original seed, when available;
  - reduced coordinates;
  - operation;
  - validity assertions;
  - sampled semantic assertions;
  - area and loop-count assertions where stable.

## 12. CI Tiers

- Normal CI:
  - deterministic tests;
  - `proptest` with moderate case count;
  - debug assertions enabled.
- Nightly/manual:
  - high case-count `proptest`;
  - ignored differential tests;
  - fuzz corpus replay.
- Pre-release:
  - fixed-duration fuzzing per target;
  - test with and without `unsafe_optimizations`;
  - debug and release test runs.

## 13. First Implementation Priorities

1. Shape validation helpers.
2. Membership sampling oracle.
3. Empty, identity, disjoint, containment, and touching rectangle tests.
4. Donut and hole tests.
5. Multi-loop pairwise aggregation tests.
6. Regression ports from polyline boolean reported cases.
7. `proptest` rectangle and donut generators.
8. Arc-heavy deterministic cases.
9. Fuzz harnesses.
10. SVG/trace failure dumps.

## 14. Implementation Backlog From Review

These tasks extend the test plan into concrete implementation work that should
make the patch faster, easier to debug, and easier to submit upstream.

1. Implemented: use shape-level spatial indexes for boolean pair selection:
   - replace nested loop-pair scans with `Shape::plines_index` queries;
   - preserve the existing CCW/CW pairing semantics and used-loop bookkeeping;
   - add or keep multi-loop tests that prove unused distant loops still survive.
2. Implemented: canonicalize same-shape detection:
   - treat equal shapes as equal even when loop order differs;
   - treat equal closed loops as equal when their start vertex is cyclically
     shifted;
   - keep exact signed-loop bin semantics so holes are not mistaken for islands.
3. Implemented: reduce repeated boolean composition and allocation:
   - audit `Xor`'s `(A - B) union (B - A)` composition;
   - audit `And`'s repeated shape-level `Or` accumulation;
   - extract private normalized-loop assembly helpers so final loop orientation,
     area filtering, and index construction are shared by all shape boolean
     paths;
   - short-circuit `Xor` composition when either difference side is empty;
   - batch `And` pair outputs through one merge/assembly path instead of
     repeatedly rebuilding intermediate shapes.
4. Implemented: centralize shape-boolean tolerances:
   - remove one-off literal epsilons from fast paths and tests;
   - align tolerances with lower-level polyline boolean options where possible;
   - route shape boolean area filtering through one private helper so remaining
     tolerance differences are explicit.
5. Implemented: upgrade debug dumps and traces:
   - emit SVG path commands for arc segments instead of polygon-only previews;
   - include lower-level `BooleanResultInfo` for each loop pair;
   - include the pairwise bbox-overlap/candidate matrix and used-loop flags;
   - emit a sample-mismatch SVG and a text trace alongside the JSON and Rust
     reproduction snippets.
6. Implemented: add real fuzz harnesses:
   - create `cargo-fuzz` targets for rectangles, donuts, arcs, transforms, and
     `PlineInversionView` booleans;
   - define corpus replay and minimization instructions;
   - make minimized cases easy to paste back into deterministic regression tests.
7. Implemented: add differential testing:
   - compare straight-line polygon-only shapes against an independent oracle in
     ignored/manual tests;
   - keep arc cases on sampled membership until an arc-capable oracle is
     available;
   - use a rectangle-set point-membership oracle that is independent from
     `Shape` winding numbers for manual differential checks.
