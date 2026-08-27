# Intersection Refactor Implementation Plan

## Status

Proposed implementation plan. No code in this document has been implemented yet.

## Goal

Make intersection finding use one clear sequence:

1. Find intersections between whole lines and circles.
2. Check those points against both finite segments.
3. Resolve endpoint snaps and geometric error.
4. Return one consistent finite-segment result.
5. Add polyline segment indexes at the polyline collection layer.

The refactor should remove the current mix of whole-circle, finite-line, finite-arc, ordering, and snapping concerns from `pline_seg_intr`.

## Fixed design decisions

- An arc has a signed sweep:
  - positive is counterclockwise;
  - negative is clockwise.
- An arc does not have a separate direction Boolean.
- A final finite-segment hit stores both the resolved point and the parameter on each segment.
- The resolved point is authoritative. Callers do not regenerate it from a parameter.
- Arc parameters are normalized to `0..=1` along the signed sweep.
- Two point intersections are ordered only by the second segment's parameter.
- Equal ordering values retain processing order. Do not add coordinate, segment-index, or direction-independent tie-breakers.
- Swapping arguments or reversing a segment may change fuzzy endpoint selection. Do not impose direction-invariant behavior.
- Position comparisons and domain boundaries retain strict epsilon checks.
- Position-equal results retain the result with the smallest final geometric error.
- The geometric behavior covered by the issue 43 regressions must remain intact. Exact numeric expectations may change only when the new result is within the intended tolerance and does not weaken the regression.
- Tests may be updated for intentional API changes and small floating-point differences caused by equivalent calculations. Before changing an expected value, classify the failure as an implementation bug, an outdated test assumption, or an acceptable numeric adjustment.
- Do not change expected intersection count, tangent classification, strict epsilon behavior, segment-2 ordering, endpoint ownership, or minimum-error selection without identifying an intentional semantic change.
- A result snapped to a stored source endpoint must continue to use that exact endpoint coordinate. Small numeric adjustments apply only to calculated points, parameters, and error values.
- Do not introduce the terms `root` or `support` in the new API. Use whole-curve intersection, candidate intersection, and finite-segment intersection.

## Architecture

The implementation will have three layers.

### 1. Whole-curve math

This layer intersects:

- infinite line with infinite line;
- infinite line with full circle;
- full circle with full circle.

It knows nothing about finite line bounds, arc sweeps, polyline vertices, segment ordering, or polyline indexes.

### 2. Finite segments

This layer owns:

- line parameter checks;
- signed arc parameter checks;
- endpoint snapping;
- point reconciliation when both segments snap;
- final error calculation;
- duplicate removal;
- ordering by segment 2;
- finite line overlap;
- finite arc overlap.

### 3. Polyline collection

This layer owns:

- conversion from polyline vertex pairs to finite segments;
- segment indexes;
- spatial-index traversal;
- deduplication between adjacent segment pairs;
- polyline-level intersection and overlap records.

It must not repeat line, circle, or arc-domain intersection logic.

## Target geometry types

### `Line`

An infinite line used only by whole-curve math.

```rust
#[derive(Debug, Copy, Clone)]
pub struct Line<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub direction: Vector2<T>,
}
```

A zero direction does not define an infinite line. `LineSegment::as_line` should therefore return `None` for a point segment. Finite pair functions will handle point segments directly and preserve current degenerate-segment behavior.

### `Circle`

A full circle used by whole-curve math.

```rust
#[derive(Debug, Copy, Clone)]
pub struct Circle<T>
where
    T: Real,
{
    pub center: Vector2<T>,
    pub radius: T,
}
```

### `LineSegment`

A finite directed line segment.

```rust
#[derive(Debug, Copy, Clone)]
pub struct LineSegment<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
}
```

Required methods:

```rust
impl<T: Real> LineSegment<T> {
    pub fn as_line(&self) -> Option<Line<T>>;
    pub fn parameter_at(&self, point: Vector2<T>) -> T;
    pub fn point_at(&self, t: T) -> Vector2<T>;
}
```

`parameter_at` should reuse the project's robust parametric math rather than introduce a new formula with different rounding behavior.

### `ArcSegment`

A finite directed circular arc.

```rust
#[derive(Debug, Copy, Clone)]
pub struct ArcSegment<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
    pub center: Vector2<T>,
    pub radius: T,
    pub sweep: T,
}
```

Required properties:

- `sweep > 0` means counterclockwise;
- `sweep < 0` means clockwise;
- supported polyline arcs retain the current maximum half-circle sweep;
- exact source endpoints remain stored for snapping;
- radius and sweep are calculated once when the segment is built.

Required methods:

```rust
impl<T: Real> ArcSegment<T> {
    pub fn circle(&self) -> Circle<T>;
    pub fn parameter_at(&self, point: Vector2<T>) -> T;
    pub fn point_at(&self, t: T) -> Vector2<T>;
}
```

`parameter_at` calculates the signed angle from `start` to `point` and divides it by `sweep`. A point in the finite domain has a parameter in `0..=1` for either sweep direction.

`point_at` is not part of normal intersection result consumption. Callers use the stored result point. If `point_at` handles `t == 0` or `t == 1`, it should return the stored exact endpoint rather than calculate it with trigonometry.

### `Segment`

A finite polyline segment in a form that can dispatch without re-reading bulge state.

```rust
#[derive(Debug, Copy, Clone)]
pub enum Segment<T>
where
    T: Real,
{
    Line(LineSegment<T>),
    Arc(ArcSegment<T>),
}
```

Required construction:

```rust
impl<T: Real> Segment<T> {
    pub fn from_vertices(start: PlineVertex<T>, end: PlineVertex<T>) -> Self;
}
```

`from_vertices` is the only place in the segment intersection path that interprets a polyline bulge and calculates the arc center, radius, and signed sweep.

## Target whole-curve intersection types

### `IntersectionPoint`

One point from whole-line or full-circle intersection math.

```rust
#[derive(Debug, Copy, Clone)]
pub struct IntersectionPoint<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub error_squared: T,
}
```

`error_squared` remains the sum of squared geometric adjustments used to produce the point. Zero means the point was not adjusted.

### `CurveIntersections`

One result shape for all whole-curve functions.

```rust
#[derive(Debug, Copy, Clone)]
pub enum CurveIntersections<T>
where
    T: Real,
{
    None,
    Tangent(IntersectionPoint<T>),
    One(IntersectionPoint<T>),
    Two([IntersectionPoint<T>; 2]),
    Coincident,
}
```

Meaning:

- `None`: no intersection points;
- `Tangent`: one point with tangent contact;
- `One`: one non-tangent point, used by line-line;
- `Two`: two distinct points;
- `Coincident`: the two lines or the two circles coincide within the current tolerance rules.

`Coincident` does not claim that finite segments overlap. The finite layer determines that.

## Target finite-segment intersection types

### `SegmentIntersection`

One final finite-segment hit.

```rust
#[derive(Debug, Copy, Clone)]
pub struct SegmentIntersection<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub error_squared: T,
    pub seg1_t: T,
    pub seg2_t: T,
}
```

Field meaning:

- `point` is the authoritative resolved intersection returned to callers;
- `error_squared` is the final whole-curve error plus finite-domain adjustment error;
- `seg1_t` locates the candidate on segment 1;
- `seg2_t` locates the candidate on segment 2.

For an accepted result, both parameters must be in `0..=1`.

Fuzzy endpoint handling means these three points may differ:

```rust
seg1.point_at(hit.seg1_t)
seg2.point_at(hit.seg2_t)
hit.point
```

For example, two position-equal endpoints can have `seg1_t == 1`, `seg2_t == 0`, and distinct source coordinates. The resolver selects one authoritative output point and records the adjustment in `error_squared`.

### `SegmentIntersections`

One result shape for every finite segment pair.

```rust
#[derive(Debug, Copy, Clone)]
pub enum SegmentIntersections<T>
where
    T: Real,
{
    None,
    Tangent(SegmentIntersection<T>),
    One(SegmentIntersection<T>),
    Two([SegmentIntersection<T>; 2]),
    Overlap {
        start: SegmentIntersection<T>,
        end: SegmentIntersection<T>,
    },
}
```

Rules:

- `Two` is ordered by `seg2_t`.
- `Overlap.start` and `Overlap.end` are ordered by segment 2.
- Line and arc overlaps use the same variant.
- A positive overlap whose resolved boundaries are position-equal coalesces to `One`, preserving current behavior.
- One surviving point from a clipped `Two` result is `One`, not `Tangent`.
- Two distinct curve points that coalesce after snapping produce `One`, not `Tangent`.
- A curve tangent that survives finite-domain resolution remains `Tangent`.

## Private finite-domain type

Only one short-lived type is needed while checking a candidate point against one finite segment.

```rust
#[derive(Debug, Copy, Clone)]
struct SegmentLocation<T>
where
    T: Real,
{
    t: T,
    point: Vector2<T>,
    adjustment_error_squared: T,
}
```

It means that a whole-curve intersection has been accepted on one finite segment.

- For an interior point, `point` is unchanged and adjustment error is zero.
- For an endpoint snap, `point` is the exact stored endpoint, `t` is zero or one, and adjustment error is the squared move to that endpoint.
- A rejected point is represented by `None`, not by an `Outside` variant.

Required methods:

```rust
impl<T: Real> LineSegment<T> {
    fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>>;
}

impl<T: Real> ArcSegment<T> {
    fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>>;
}

impl<T: Real> Segment<T> {
    fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>>;
}
```

This replaces `ArcDomain`, `IntrDomainPosition`, `FiniteDomainMatch`, and `PointAdjustment`.

## Target whole-curve functions

Use consistent verb-first names and typed geometry inputs.

```rust
pub fn intersect_lines<T: Real>(
    line1: Line<T>,
    line2: Line<T>,
    epsilon: T,
) -> CurveIntersections<T>;

pub fn intersect_line_circle<T: Real>(
    line: Line<T>,
    circle: Circle<T>,
    epsilon: T,
) -> CurveIntersections<T>;

pub fn intersect_circles<T: Real>(
    circle1: Circle<T>,
    circle2: Circle<T>,
    epsilon: T,
) -> CurveIntersections<T>;
```

These functions do not:

- clamp line parameters;
- check arc sweeps;
- snap to finite endpoints;
- order results according to a finite segment;
- return finite overlap boundaries.

The existing numeric calculations should be extracted rather than rewritten where possible. Separating finite bounds from curve math must not weaken current large-coordinate, near-tangent, or rounding recovery behavior.

## Target finite-segment functions

Provide a function for each ordered geometry pair because result parameters and fuzzy endpoint choices refer to argument order.

```rust
pub fn intersect_line_segments<T: Real>(
    seg1: LineSegment<T>,
    seg2: LineSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;

pub fn intersect_line_arc<T: Real>(
    line: LineSegment<T>,
    arc: ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;

pub fn intersect_arc_line<T: Real>(
    arc: ArcSegment<T>,
    line: LineSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;

pub fn intersect_arcs<T: Real>(
    arc1: ArcSegment<T>,
    arc2: ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;

pub fn intersect_segments<T: Real>(
    seg1: Segment<T>,
    seg2: Segment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;
```

`intersect_arc_line` is a thin ordered wrapper around the same line-circle calculation used by `intersect_line_arc`. It must resolve candidates with the arc as segment 1 and the line as segment 2. Do not resolve in line-arc order and swap afterward: an equal-error endpoint choice may depend on segment argument order.

## Shared point resolution

### Resolve one candidate

```rust
fn resolve_intersection<T: Real>(
    seg1: &Segment<T>,
    seg2: &Segment<T>,
    candidate: IntersectionPoint<T>,
    epsilon_squared: T,
) -> Option<SegmentIntersection<T>>;
```

Steps:

1. Call `seg1.locate(candidate, epsilon_squared)`.
2. Call `seg2.locate(candidate, epsilon_squared)`.
3. Reject immediately if either returns `None`.
4. Reconcile the two located points.
5. Return the authoritative point, final error, and both parameters.

Reconciliation rules:

1. If neither segment moved the candidate, keep the candidate point.
2. If one segment snapped, use that exact endpoint only if the move is strictly inside epsilon.
3. If both segments snapped to the same coordinate, use it.
4. If both snapped to distinct coordinates, require:
   - each candidate-to-endpoint move to be strictly inside epsilon;
   - endpoint separation to be strictly inside epsilon.
5. Select the endpoint with the smallest final error.
6. On equal error, retain segment argument order.
7. Add whole-curve and finite-domain errors once. Do not count the same move twice.

### Resolve a point result

```rust
fn resolve_curve_intersections<T: Real>(
    seg1: &Segment<T>,
    seg2: &Segment<T>,
    intersections: CurveIntersections<T>,
    epsilon: T,
) -> SegmentIntersections<T>;
```

Responsibilities:

- preserve tangent status for a surviving tangent;
- resolve each `One` or `Two` point through both finite domains;
- discard rejected points;
- merge position-equal accepted points;
- keep the merged point with the smallest final `error_squared`;
- sort two accepted points only by `seg2_t`;
- preserve processing order when `seg2_t` values compare equal.

`Coincident` is handled by the geometry-pair function because line overlap and arc overlap require different interval calculations.

## Geometry-pair flow

### Line-line

```text
LineSegment + LineSegment
    -> handle point-segment degeneracy if needed
    -> convert nondegenerate segments to Line
    -> intersect_lines
    -> resolve distinct point through both segments
       or resolve finite collinear interval
    -> SegmentIntersections
```

Coincident line handling must return overlap boundaries with:

- an authoritative point;
- a parameter on each segment;
- a real adjustment error;
- ordering according to segment 2.

Do not set overlap boundary error to zero when the boundary was moved under tolerance.

### Line-arc

```text
LineSegment + ArcSegment
    -> handle point-segment degeneracy if needed
    -> intersect_line_circle(line.as_line(), arc.circle())
    -> resolve each point through the finite line and finite arc
    -> SegmentIntersections
```

`intersect_arc_line` uses the same curve calculation but passes the finite segments to resolution in arc-line order.

This removes the current `line_is_seg1` flag and the synthetic accepted line-domain state.

### Arc-arc

```text
ArcSegment + ArcSegment
    -> intersect_circles(arc1.circle(), arc2.circle())
    -> resolve distinct points through both arcs
       or resolve coincident finite arc intervals
    -> SegmentIntersections
```

Move coincident arc interval work to:

```rust
fn intersect_coincident_arcs<T: Real>(
    arc1: &ArcSegment<T>,
    arc2: &ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>;
```

The function should express arc 1 in arc 2's signed parameter direction, intersect finite intervals, preserve wraparound behavior, and return boundaries ordered by arc 2.

## Polyline adapter

`pline_seg_intr` becomes a small adapter:

```rust
pub fn pline_seg_intr<T: Real>(
    seg1_start: PlineVertex<T>,
    seg1_end: PlineVertex<T>,
    seg2_start: PlineVertex<T>,
    seg2_end: PlineVertex<T>,
    epsilon: T,
) -> SegmentIntersections<T> {
    intersect_segments(
        Segment::from_vertices(seg1_start, seg1_end),
        Segment::from_vertices(seg2_start, seg2_end),
        epsilon,
    )
}
```

It should contain no geometry-pair algorithm, candidate mapping, endpoint adjustment, overlap calculation, or ordering code.

## Type migration

| Current type | Replacement |
| --- | --- |
| `IntrPoint` | `IntersectionPoint` |
| `LineCircleIntrPoint` | Removed; whole-curve result uses `IntersectionPoint` |
| `LineLineIntrPoint` | Removed; whole-curve result uses `IntersectionPoint` |
| `LineCircleIntr` | `CurveIntersections` |
| `CircleCircleIntr` | `CurveIntersections` |
| `LineLineIntr` | `CurveIntersections` |
| `PlineSegIntrPoint` | `SegmentIntersection` |
| `PlineSegIntr` | `SegmentIntersections` |
| `ArcDomain` | `ArcSegment` methods |
| `IntrDomainPosition` | `Option<SegmentLocation>` |
| `FiniteDomainMatch` | `SegmentLocation` |
| `PointAdjustment` | `SegmentLocation` fields |
| `IntersectionCandidate` | Removed; resolve `IntersectionPoint` directly |
| `AcceptedIntersection` | Removed; resolution returns `SegmentIntersection` |
| `OverlappingLines` | `SegmentIntersections::Overlap` |
| `OverlappingArcs` | `SegmentIntersections::Overlap` |

Polyline-level records that add segment indexes remain separate because they represent different information.

## Target file layout

```text
cavalier_contours/src/core/math/
    intersections/
        mod.rs
        line_line.rs
        line_circle.rs
        circle_circle.rs
    mod.rs

cavalier_contours/src/polyline/
    segment.rs
    segment_intersections/
        mod.rs
        resolve.rs
        line_line.rs
        line_arc.rs
        arc_arc.rs
    pline_seg_intersect.rs
```

Responsibilities:

- `core/math/intersections/mod.rs`
  - `Line`
  - `Circle`
  - `IntersectionPoint`
  - `CurveIntersections`
  - exports for whole-curve functions
- `line_line.rs`, `line_circle.rs`, `circle_circle.rs`
  - numeric whole-curve calculations and focused tests
- `polyline/segment.rs`
  - `LineSegment`
  - `ArcSegment`
  - `Segment`
  - signed parameters and finite-domain location
- `segment_intersections/resolve.rs`
  - point reconciliation
  - error accumulation
  - minimum-error duplicate removal
  - segment-2 ordering
- pair files
  - pair dispatch and coincident interval handling
- `pline_seg_intersect.rs`
  - polyline vertex adapter and public compatibility entry point while callers migrate

Delete `core/math/intersect_common.rs` after its concrete responsibilities have moved. Do not replace it with another general `common`, `util`, or `helpers` file.

## Workspace caller migration

Before changing APIs, inventory all uses of:

```text
line_line_intr
line_circle_intr
circle_circle_intr
LineLineIntr
LineCircleIntr
CircleCircleIntr
IntrPoint
ArcDomain
IntrDomainPosition
PlineSegIntr
PlineSegIntrPoint
```

Known callers include:

- `cavalier_contours/src/polyline/pline_seg_intersect.rs`
- `cavalier_contours/src/polyline/internal/raw_pline_offset.rs`
- `cavalier_contours/src/polyline/internal/pline_offset.rs`
- `cavalier_contours/src/polyline/internal/pline_intersects.rs`
- `cavalier_contours/src/polyline/pline_types.rs`

Update offset callers to consume `SegmentIntersection.point`, `seg1_t`, and `seg2_t` directly. Remove their repeated circle-point filtering and arc parameter calculations once the finite pair functions provide those results.

Update all workspace examples, benchmarks, FFI code, UI code, and tests in the same change when the old public result types are removed.

## Implementation phases

### Phase 0: Record the baseline

1. Confirm the working tree state.
2. Run all current validation:

   ```sh
   cargo fmt --all -- --check
   cargo clippy --workspace --all-targets --all-features -- -D warnings
   cargo test --workspace --all-features
   ```

3. Record the current passing test count and any unrelated warnings.

### Phase 1: Add behavior tests before moving code

Add or strengthen tests for:

- issue 43 regressions;
- positive and negative arc sweeps;
- exact arc start and end parameters;
- line-arc and arc-line argument order;
- two intersections ordered by segment 2;
- reversed segment 2 order;
- equal ordering values retaining processing order;
- strict epsilon boundary acceptance and rejection;
- one endpoint snap;
- both segments snapping to one coordinate;
- both segments snapping to distinct position-equal endpoints;
- minimum-final-error duplicate selection;
- coincident line intervals;
- coincident arc intervals across angle wrap;
- positive overlaps shorter than epsilon;
- position-equal overlap boundaries coalescing to one event;
- nonzero line segments shorter than epsilon;
- zero-length line segments;
- large coordinate scales;
- recovered external and internal tangencies.

Do not change expected behavior in this phase unless a test clearly conflicts with the stated decisions.

### Phase 2: Add geometry types

1. Add `Line`, `Circle`, `LineSegment`, `ArcSegment`, and `Segment`.
2. Build `ArcSegment` from polyline vertices with a signed sweep.
3. Add parameter methods and exact endpoint handling.
4. Add focused unit tests for:
   - positive sweep parameters;
   - negative sweep parameters;
   - angle wrap;
   - exact endpoints;
   - line parameter direction;
   - degenerate line conversion.
5. Keep old intersection paths unchanged.

### Phase 3: Extract whole-curve calculations

1. Add `IntersectionPoint` and `CurveIntersections`.
2. Extract infinite line-line calculation from finite clipping.
3. Extract infinite line-circle calculation from finite line clipping.
4. Adapt circle-circle calculation to the shared types.
5. Keep current numerical recovery formulas where possible.
6. Add direct whole-curve tests.
7. Temporarily keep old functions as local wrappers only while migrating callers; do not retain them as permanent compatibility APIs.

### Phase 4: Implement finite-domain location

1. Add `SegmentLocation`.
2. Implement `LineSegment::locate`.
3. Move `ArcDomain` classification into `ArcSegment::locate` and convert it to signed sweep parameters.
4. Preserve strict epsilon comparisons.
5. Ensure exact interior points remain unchanged.
6. Ensure snapped endpoints return exact stored coordinates and `t` equal to zero or one.

### Phase 5: Implement shared point resolution

1. Add `SegmentIntersection` and `SegmentIntersections`.
2. Implement `resolve_intersection`.
3. Implement `resolve_curve_intersections`.
4. Move final error accumulation into the resolver.
5. Move minimum-error duplicate removal into the resolver.
6. Move segment-2 ordering into the resolver.
7. Remove coordinate and total-order tie-breakers.

### Phase 6: Implement finite pair functions

1. Implement `intersect_line_segments`.
2. Implement `intersect_line_arc`.
3. Implement ordered `intersect_arc_line` without post-resolution swapping.
4. Implement distinct-point `intersect_arcs`.
5. Move coincident line overlap into the line-line module.
6. Move coincident arc overlap into the arc-arc module.
7. Implement `intersect_segments` as the four-way dispatcher.

Run focused segment tests after each pair function moves.

### Phase 7: Reduce `pline_seg_intr`

1. Convert both vertex pairs to `Segment`.
2. Call `intersect_segments`.
3. Remove pair-specific closures and branches.
4. Remove candidate mapping from `pline_seg_intersect.rs`.
5. Keep the existing entry point name only if workspace callers still require it; its body must remain an adapter.

### Phase 8: Migrate all callers

1. Update polyline intersection collection.
2. Update raw offset joins.
3. Update offset intersection visitors.
4. Update polyline-level constructors and records.
5. Update FFI, UI, examples, and benchmarks.
6. Remove repeated arc validation and parameter calculations from callers.
7. Use the stored point directly; do not call `point_at` merely to recover an intersection point.

### Phase 9: Remove old types and files

Remove:

- old per-pair point types;
- old per-pair result enums;
- `PlineSegIntrPoint` and `PlineSegIntr` after migration;
- `ArcDomain` and `IntrDomainPosition`;
- `PointAdjustment` and `FiniteDomainMatch`;
- `IntersectionCandidate` and `AcceptedIntersection`;
- obsolete imports and adapters;
- `intersect_common.rs`.

Review exports in `core/math/mod.rs` and `polyline/mod.rs` so the new API has one clear path.

### Phase 10: Final review and validation

1. Review every changed line for scope and simpler alternatives.
2. Search for old type and function names; no stale uses should remain.
3. Search comments and docs for obsolete behavior descriptions.
4. Run:

   ```sh
   cargo fmt --all
   cargo clippy --workspace --all-targets --all-features -- -D warnings
   cargo test --workspace --all-features
   ```

5. Run relevant optimized benchmarks if the changed parameter and arc code affects measured paths:

   ```sh
   cargo bench -p cavalier_contours --bench benches -- raw_offset_creation --warm-up-time 1 --measurement-time 3
   ```

6. Compare benchmark results with the baseline. Arc candidate handling should not add repeated trigonometric work in callers.

## Test invariants

Every finite point result must satisfy:

- `0 <= seg1_t <= 1`;
- `0 <= seg2_t <= 1`;
- `error_squared >= 0`;
- `point` is the authoritative result;
- an unchanged analytic point has zero finite adjustment error;
- endpoint snaps use exact source endpoint coordinates;
- strict epsilon boundaries remain strict.

Every `Two` result must satisfy:

- the two points are not position-equal under the supplied epsilon;
- `hit1.seg2_t <= hit2.seg2_t`;
- equal parameters do not invoke another tie-breaker.

Every `Overlap` result must satisfy:

- both boundaries have valid parameters on both segments;
- boundaries follow segment 2's direction;
- boundaries are not position-equal under epsilon;
- any boundary adjustment is included in its error.

## Acceptance criteria

The refactor is complete when:

- `pline_seg_intr` only builds segments and dispatches;
- line-arc and arc-arc mapping no longer lives in `pline_seg_intersect.rs`;
- all whole-curve functions use `IntersectionPoint` and `CurveIntersections`;
- all finite pair functions use `SegmentIntersection` and `SegmentIntersections`;
- arcs use signed sweep with no direction Boolean;
- final hits store a point and both normalized parameters;
- callers do not regenerate intersection points from parameters;
- finite-domain rejection uses `Option`, not an `Outside` result state;
- line and arc overlaps use one final overlap shape;
- duplicate points retain the smallest final error;
- equal-error and equal-order cases add no total-order tie-breaker;
- `intersect_common.rs` and the old near-duplicate types are gone;
- repeated finite arc checks have been removed from offset callers;
- formatting, Clippy, all workspace tests, and issue 43 regressions pass;
- relevant benchmarks show no material regression from repeated angle or point reconstruction work.
