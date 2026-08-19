//! Raw geometry construction for the polyline offset algorithm.
//!
//! The public items in this module are exposed only so workspace visualization, benchmarks, and
//! tests can inspect raw offset geometry.

use crate::{
    core::{
        math::{
            CircleCircleIntr, LineCircleIntr, LineLineIntr, Vector2, angle, angle_from_bulge,
            arc_sweep_extents, bulge_from_angle, circle_circle_intr, dist_squared,
            line_circle_intr, line_line_intr, normalize_radians, parametric_from_point,
            point_from_parametric,
        },
        traits::Real,
    },
    polyline::{
        PlineCreation, PlineSource, PlineSourceMut, PlineVertex, line_seg_bounding_box,
        seg_arc_radius_and_center,
    },
};
use static_aabb2d_index::AABB;

/// An untrimmed offset line.
#[derive(Debug, Copy, Clone)]
pub struct RawOffsetLine<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
}

/// An untrimmed offset arc with cached support geometry.
#[derive(Debug, Copy, Clone)]
pub struct RawOffsetArc<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
    pub center: Vector2<T>,
    pub radius: T,
    pub start_angle: T,
    pub sweep: T,
}

impl<T> RawOffsetArc<T>
where
    T: Real,
{
    #[inline]
    fn tangent_at(&self, point: Vector2<T>) -> Vector2<T> {
        let tangent = (point - self.center).perp();
        if self.sweep < T::zero() {
            -tangent
        } else {
            tangent
        }
    }
}

/// A source arc offset past zero radius, retained as a straight clipping scaffold.
#[derive(Debug, Copy, Clone)]
pub struct CollapsedRawOffsetArc<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
    /// Source arc tangent at `start`, cached to avoid recomputing it during join classification.
    pub start_tangent: Vector2<T>,
    /// Source arc tangent at `end`, cached to avoid recomputing it during join classification.
    pub end_tangent: Vector2<T>,
}

impl<T> RawOffsetLine<T>
where
    T: Real,
{
    #[inline]
    fn param_at(&self, point: Vector2<T>, pos_equal_eps: T) -> T {
        parametric_from_point(self.start, self.end, point, pos_equal_eps)
    }
}

/// An untrimmed source-derived offset segment.
///
/// Arc support data is cached when the segment is created so joins and direction checks do not
/// reconstruct it from polyline vertices.
#[derive(Debug, Copy, Clone)]
pub enum RawOffsetSeg<T>
where
    T: Real,
{
    Line(RawOffsetLine<T>),
    Arc(RawOffsetArc<T>),
    /// An arc whose radius crossed zero while offsetting. It remains as line-shaped clipping
    /// geometry, but every emitted source span derived from it is locally invalid.
    Collapsed(CollapsedRawOffsetArc<T>),
}

impl<T> RawOffsetSeg<T>
where
    T: Real,
{
    /// Returns the segment start.
    #[inline]
    pub fn start(&self) -> Vector2<T> {
        match *self {
            Self::Line(line) => line.start,
            Self::Arc(arc) => arc.start,
            Self::Collapsed(arc) => arc.start,
        }
    }

    /// Returns the segment end.
    #[inline]
    pub fn end(&self) -> Vector2<T> {
        match *self {
            Self::Line(line) => line.end,
            Self::Arc(arc) => arc.end,
            Self::Collapsed(arc) => arc.end,
        }
    }

    /// Returns whether the source arc was offset past zero radius.
    #[inline]
    pub fn is_collapsed(&self) -> bool {
        matches!(self, Self::Collapsed(_))
    }

    /// Returns the segment's exact axis-aligned bounding box.
    pub fn bounding_box(&self) -> AABB<T> {
        match *self {
            Self::Line(line) => line_seg_bounding_box(line.start, line.end),
            Self::Collapsed(arc) => line_seg_bounding_box(arc.start, arc.end),
            Self::Arc(arc) => {
                let (min_point, max_point) = arc_sweep_extents(
                    arc.start,
                    arc.end,
                    arc.center,
                    arc.radius,
                    arc.start_angle,
                    arc.sweep,
                );
                AABB::new(min_point.x, min_point.y, max_point.x, max_point.y)
            }
        }
    }

    #[inline]
    fn span_bulge(&self, start_param: T, end_param: T) -> T {
        match *self {
            Self::Arc(arc) => bulge_from_angle(arc.sweep * (end_param - start_param)),
            Self::Line(_) | Self::Collapsed(_) => T::zero(),
        }
    }

    #[inline]
    fn span_is_invalid(&self, start_param: T, end_param: T) -> bool {
        self.is_collapsed() || end_param <= start_param
    }

    #[inline]
    fn start_tangent(&self) -> Vector2<T> {
        match *self {
            Self::Line(line) => line.end - line.start,
            Self::Arc(arc) => arc.tangent_at(arc.start),
            Self::Collapsed(arc) => arc.start_tangent,
        }
    }

    #[inline]
    fn end_tangent(&self) -> Vector2<T> {
        match *self {
            Self::Line(line) => line.end - line.start,
            Self::Arc(arc) => arc.tangent_at(arc.end),
            Self::Collapsed(arc) => arc.end_tangent,
        }
    }
}

/// Joined raw offset geometry and segment-aligned local validity metadata.
#[derive(Debug, Clone)]
pub struct RawOffsetResult<O> {
    /// Joined raw offset geometry.
    pub polyline: O,
    /// One flag per output segment; `true` marks locally invalid geometry.
    pub invalid_segments: Vec<bool>,
    /// Sorted indexes of invalid output segments.
    pub invalid_segment_indexes: Vec<usize>,
}

impl<O> RawOffsetResult<O>
where
    O: PlineCreation,
{
    fn empty() -> Self {
        Self {
            polyline: O::empty(),
            invalid_segments: Vec::new(),
            invalid_segment_indexes: Vec::new(),
        }
    }
}

/// Local classification of a boundary between adjacent raw offset segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JoinClass {
    /// Raw endpoints coincide, so no connector is needed.
    Coincident,
    /// An inner join trimmed both source spans to a shared intersection.
    Trimmed,
    /// An outer join bridged by an arc centered at the shared source vertex.
    OuterRound,
    /// Opposite collinear tangents bridged by an arc centered at the shared source vertex.
    ReversalRound,
    /// An inner join has no usable intersection, so its straight connector is invalid.
    InvalidInnerGap,
    /// A zero-length or same-direction collinear tangent configuration leaves an invalid gap.
    InvalidDegenerateGap,
}

/// Geometry and source-span limits chosen for one join.
#[derive(Debug, Clone, Copy)]
struct JoinBoundary<T> {
    /// Geometric join classification.
    class: JoinClass,
    /// End of the retained current source span.
    current_end: Vector2<T>,
    /// Parametric position of `current_end` on the current segment, where 0 is the start and 1 is
    /// the end.
    current_param: T,
    /// Start of the retained next source span.
    next_start: Vector2<T>,
    /// Parametric position of `next_start` on the next segment, where 0 is the start and 1 is the
    /// end.
    next_param: T,
    /// Bulge of the connector from `current_end` to `next_start`.
    connector_bulge: T,
}

impl<T> JoinBoundary<T>
where
    T: Real,
{
    #[inline]
    fn connector_is_invalid(&self) -> bool {
        matches!(
            self.class,
            JoinClass::InvalidInnerGap | JoinClass::InvalidDegenerateGap
        )
    }
}

struct RawOffsetBuilder<'a, P, T, O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineSourceMut<Num = T>,
{
    source: &'a P,
    raw_segments: Vec<RawOffsetSeg<T>>,
    polyline: O,
    invalid_segment_indexes: Vec<usize>,
    offset: T,
    connector_is_clockwise: bool,
    pos_equal_eps: T,
}

impl<'a, P, T, O> RawOffsetBuilder<'a, P, T, O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    fn new(source: &'a P, raw_segments: Vec<RawOffsetSeg<T>>, offset: T, pos_equal_eps: T) -> Self {
        Self {
            source,
            polyline: O::with_capacity(source.vertex_count() + 1, false),
            invalid_segment_indexes: Vec::new(),
            raw_segments,
            offset,
            connector_is_clockwise: offset >= T::zero(),
            pos_equal_eps,
        }
    }

    #[inline]
    fn arc_connection_boundary(
        &self,
        class: JoinClass,
        current_end: Vector2<T>,
        next_start: Vector2<T>,
        shared_source_vertex: Vector2<T>,
    ) -> JoinBoundary<T> {
        let current_direction = (current_end - shared_source_vertex).normalize();
        let next_direction = (next_start - shared_source_vertex).normalize();
        let mut cos_sweep = current_direction.dot(next_direction);
        if cos_sweep < -T::one() {
            cos_sweep = -T::one();
        } else if cos_sweep > T::one() {
            cos_sweep = T::one();
        }
        let mut sin_sweep = current_direction.perp_dot(next_direction).abs();
        if sin_sweep > T::one() {
            sin_sweep = T::one();
        }

        // Both branches equal tan(sweep / 4). For sweeps up to a quarter circle, the first
        // avoids subtracting cos_sweep from one when they are nearly equal. For larger sweeps,
        // the second avoids the first form's numerator and denominator both approaching zero as
        // the sweep approaches a half circle.
        let connector_bulge_magnitude = if cos_sweep >= T::zero() {
            sin_sweep / (T::one() + cos_sweep + (T::two() * (T::one() + cos_sweep)).sqrt())
        } else {
            (T::one() - cos_sweep) / (sin_sweep + (T::two() * (T::one() - cos_sweep)).sqrt())
        };
        let connector_bulge = if self.connector_is_clockwise {
            -connector_bulge_magnitude
        } else {
            connector_bulge_magnitude
        };
        JoinBoundary {
            class,
            current_end,
            current_param: T::one(),
            next_start,
            next_param: T::zero(),
            connector_bulge,
        }
    }

    #[inline]
    fn line_connection_boundary(
        class: JoinClass,
        current_end: Vector2<T>,
        next_start: Vector2<T>,
    ) -> JoinBoundary<T> {
        JoinBoundary {
            class,
            current_end,
            current_param: T::one(),
            next_start,
            next_param: T::zero(),
            connector_bulge: T::zero(),
        }
    }

    /// Classifies joins that need no intersection solve. Returns `None` for inner joins.
    fn join_without_intersection(
        &self,
        current: &RawOffsetSeg<T>,
        next: &RawOffsetSeg<T>,
        shared_source_vertex: Vector2<T>,
    ) -> Option<JoinBoundary<T>> {
        let incoming = current.end_tangent();
        let outgoing = next.start_tangent();
        let incoming_len_sq = incoming.length_squared();
        let outgoing_len_sq = outgoing.length_squared();
        let position_eps_sq = self.pos_equal_eps * self.pos_equal_eps;

        // Invalid degenerate gap: a zero-length tangent has no usable direction.
        if incoming_len_sq <= position_eps_sq || outgoing_len_sq <= position_eps_sq {
            return Some(Self::line_connection_boundary(
                JoinClass::InvalidDegenerateGap,
                current.end(),
                next.start(),
            ));
        }

        let turn = incoming.perp_dot(outgoing);
        let alignment = incoming.dot(outgoing);
        let angle_eps = T::fuzzy_epsilon();

        if turn * turn <= angle_eps * angle_eps * incoming_len_sq * outgoing_len_sq {
            if alignment < T::zero() {
                // Reversal round: bridge opposite collinear tangents with an arc.
                Some(self.arc_connection_boundary(
                    JoinClass::ReversalRound,
                    current.end(),
                    next.start(),
                    shared_source_vertex,
                ))
            } else {
                // Invalid degenerate gap: same-direction collinear tangents leave a gap.
                Some(Self::line_connection_boundary(
                    JoinClass::InvalidDegenerateGap,
                    current.end(),
                    next.start(),
                ))
            }
        } else if self.offset * turn > T::zero() {
            // Inner join: defer classification until the offset primitives are intersected.
            None
        } else {
            // Outer round: bridge the offset endpoints with an arc.
            Some(self.arc_connection_boundary(
                JoinClass::OuterRound,
                current.end(),
                next.start(),
                shared_source_vertex,
            ))
        }
    }

    fn analyze_join(&self, current_index: usize, next_index: usize) -> JoinBoundary<T> {
        let current = &self.raw_segments[current_index];
        let next = &self.raw_segments[next_index];
        let shared_source_vertex = self.source.at(next_index).pos();

        // Coincident: no connector is needed.
        if current.end().fuzzy_eq_eps(next.start(), self.pos_equal_eps) {
            return Self::line_connection_boundary(
                JoinClass::Coincident,
                current.end(),
                current.end(),
            );
        }

        if let Some(boundary) = self.join_without_intersection(current, next, shared_source_vertex)
        {
            return boundary;
        }

        // Collapsed arcs have no support circle from which to obtain a trim point.
        if current.is_collapsed() || next.is_collapsed() {
            return Self::line_connection_boundary(
                JoinClass::InvalidInnerGap,
                current.end(),
                next.start(),
            );
        }

        // Intersect the finite offset primitives to trim the inner join.
        match (current, next) {
            (RawOffsetSeg::Line(current_line), RawOffsetSeg::Line(next_line)) => {
                self.line_line_join(current_line, next_line)
            }
            (RawOffsetSeg::Line(current_line), RawOffsetSeg::Arc(next_arc)) => {
                self.line_arc_join(current_line, next_arc, shared_source_vertex)
            }
            (RawOffsetSeg::Arc(current_arc), RawOffsetSeg::Line(next_line)) => {
                self.arc_line_join(current_arc, next_line, shared_source_vertex)
            }
            (RawOffsetSeg::Arc(current_arc), RawOffsetSeg::Arc(next_arc)) => {
                self.arc_arc_join(current_arc, next_arc, shared_source_vertex)
            }
            (RawOffsetSeg::Collapsed(_), _) | (_, RawOffsetSeg::Collapsed(_)) => {
                unreachable!("collapsed joins return before intersection solving")
            }
        }
    }

    fn line_line_join(
        &self,
        current: &RawOffsetLine<T>,
        next: &RawOffsetLine<T>,
    ) -> JoinBoundary<T> {
        match line_line_intr(
            current.start,
            current.end,
            next.start,
            next.end,
            self.pos_equal_eps,
        ) {
            LineLineIntr::NoIntersect | LineLineIntr::FalseIntersect { .. } => {
                Self::line_connection_boundary(JoinClass::InvalidInnerGap, current.end, next.start)
            }
            LineLineIntr::TrueIntersect { seg1_t, seg2_t } => {
                // Clamp intersections accepted just past an endpoint.
                let clamp_param = |param| {
                    if param < T::zero() {
                        T::zero()
                    } else if param > T::one() {
                        T::one()
                    } else {
                        param
                    }
                };
                let current_param = clamp_param(seg1_t);
                let next_param = clamp_param(seg2_t);

                // Use the exact endpoint position when a parameter was clamped.
                let point = if seg1_t < T::zero() {
                    current.start
                } else if seg1_t > T::one() {
                    current.end
                } else if seg2_t < T::zero() {
                    next.start
                } else if seg2_t > T::one() {
                    next.end
                } else {
                    point_from_parametric(current.start, current.end, seg1_t)
                };
                Self::trimmed_boundary(point, current_param, next_param)
            }
            LineLineIntr::Overlapping { .. } => {
                // Prefer the current endpoint when it lies on the overlapping next span.
                let next_param = next.param_at(current.end, self.pos_equal_eps);
                if let Some(next_param) =
                    self.usable_line_param(next_param, (next.end - next.start).length_squared())
                {
                    Self::trimmed_boundary(current.end, T::one(), next_param)
                } else {
                    Self::line_connection_boundary(
                        JoinClass::InvalidInnerGap,
                        current.end,
                        next.start,
                    )
                }
            }
        }
    }

    #[inline]
    fn trimmed_boundary(point: Vector2<T>, current_param: T, next_param: T) -> JoinBoundary<T> {
        JoinBoundary {
            class: JoinClass::Trimmed,
            current_end: point,
            current_param,
            next_start: point,
            next_param,
            connector_bulge: T::zero(),
        }
    }

    /// Clamps a line parameter when its endpoint overflow is within position tolerance.
    #[inline]
    fn usable_line_param(&self, param: T, length_squared: T) -> Option<T> {
        let eps_squared = self.pos_equal_eps * self.pos_equal_eps;
        let clamped = if param < T::zero() {
            T::zero()
        } else if param > T::one() {
            T::one()
        } else {
            return Some(param);
        };

        let overflow = param - clamped;
        if overflow * overflow * length_squared > eps_squared {
            None
        } else {
            Some(clamped)
        }
    }

    /// Returns a finite-sweep arc parameter, allowing endpoint overflow within position tolerance.
    #[inline]
    fn usable_arc_param(&self, arc: &RawOffsetArc<T>, point: Vector2<T>) -> Option<T> {
        // Match endpoints before converting their polar angles.
        if point.fuzzy_eq_eps(arc.start, self.pos_equal_eps) {
            return Some(T::zero());
        }
        if point.fuzzy_eq_eps(arc.end, self.pos_equal_eps) {
            return Some(T::one());
        }

        let point_angle = angle(arc.center, point);
        let traveled = if arc.sweep < T::zero() {
            normalize_radians(arc.start_angle - point_angle)
        } else {
            normalize_radians(point_angle - arc.start_angle)
        };
        let sweep = arc.sweep.abs();

        // Accept sweep overflow within position tolerance.
        if traveled > sweep {
            if arc.radius * (traveled - sweep) > self.pos_equal_eps {
                return None;
            }
            return Some(T::one());
        }

        Some(traveled / sweep)
    }

    /// Tests squared distances within position tolerance without taking square roots.
    #[inline]
    fn squared_distances_equal(&self, dist1: T, dist2: T) -> bool {
        let eps_squared = self.pos_equal_eps * self.pos_equal_eps;
        let sum = dist1 + dist2;
        if sum <= eps_squared {
            return true;
        }
        let difference_from_eps = sum - eps_squared;
        difference_from_eps * difference_from_eps < T::four() * dist1 * dist2
    }

    fn line_arc_intersection(
        &self,
        line: &RawOffsetLine<T>,
        arc: &RawOffsetArc<T>,
        shared_source_vertex: Vector2<T>,
    ) -> Option<(Vector2<T>, T, T)> {
        let line_length_squared = (line.end - line.start).length_squared();

        // Keep candidates on both finite spans and snap tolerant endpoint hits.
        let process_intersect = |t: T, intersect: Vector2<T>| {
            let line_param = self.usable_line_param(t, line_length_squared)?;
            let point = if t < T::zero() {
                line.start
            } else if t > T::one() {
                line.end
            } else if intersect.fuzzy_eq_eps(arc.start, self.pos_equal_eps) {
                arc.start
            } else if intersect.fuzzy_eq_eps(arc.end, self.pos_equal_eps) {
                arc.end
            } else {
                intersect
            };
            let arc_param = self.usable_arc_param(arc, point)?;
            Some((point, line_param, arc_param))
        };

        // Prefer the intersection nearest the shared source vertex.
        match line_circle_intr(
            line.start,
            line.end,
            arc.radius,
            arc.center,
            self.pos_equal_eps,
        ) {
            LineCircleIntr::NoIntersect => None,
            LineCircleIntr::TangentIntersect { t0 } => {
                process_intersect(t0, point_from_parametric(line.start, line.end, t0))
            }
            LineCircleIntr::TwoIntersects { t0, t1 } => {
                let intr1 = point_from_parametric(line.start, line.end, t0);
                let intr2 = point_from_parametric(line.start, line.end, t1);
                let dist1 = dist_squared(intr1, shared_source_vertex);
                let dist2 = dist_squared(intr2, shared_source_vertex);
                if self.squared_distances_equal(dist1, dist2) {
                    process_intersect(t0, intr1).or_else(|| process_intersect(t1, intr2))
                } else if dist1 < dist2 {
                    process_intersect(t0, intr1)
                } else {
                    process_intersect(t1, intr2)
                }
            }
        }
    }

    fn line_arc_join(
        &self,
        current: &RawOffsetLine<T>,
        next: &RawOffsetArc<T>,
        shared_source_vertex: Vector2<T>,
    ) -> JoinBoundary<T> {
        match self.line_arc_intersection(current, next, shared_source_vertex) {
            Some((point, line_param, arc_param)) => {
                Self::trimmed_boundary(point, line_param, arc_param)
            }
            None => {
                Self::line_connection_boundary(JoinClass::InvalidInnerGap, current.end, next.start)
            }
        }
    }

    fn arc_line_join(
        &self,
        current: &RawOffsetArc<T>,
        next: &RawOffsetLine<T>,
        shared_source_vertex: Vector2<T>,
    ) -> JoinBoundary<T> {
        match self.line_arc_intersection(next, current, shared_source_vertex) {
            Some((point, line_param, arc_param)) => {
                Self::trimmed_boundary(point, arc_param, line_param)
            }
            None => {
                Self::line_connection_boundary(JoinClass::InvalidInnerGap, current.end, next.start)
            }
        }
    }

    fn arc_arc_join(
        &self,
        current: &RawOffsetArc<T>,
        next: &RawOffsetArc<T>,
        shared_source_vertex: Vector2<T>,
    ) -> JoinBoundary<T> {
        // Keep candidates on both finite spans and snap tolerant endpoint hits.
        let process_intersect = |intersect: Vector2<T>| {
            let point = if intersect.fuzzy_eq_eps(current.start, self.pos_equal_eps) {
                current.start
            } else if intersect.fuzzy_eq_eps(current.end, self.pos_equal_eps) {
                current.end
            } else if intersect.fuzzy_eq_eps(next.start, self.pos_equal_eps) {
                next.start
            } else if intersect.fuzzy_eq_eps(next.end, self.pos_equal_eps) {
                next.end
            } else {
                intersect
            };
            let current_param = self.usable_arc_param(current, point)?;
            let next_param = self.usable_arc_param(next, point)?;
            Some(Self::trimmed_boundary(point, current_param, next_param))
        };

        // Prefer the intersection nearest the shared source vertex.
        let result = match circle_circle_intr(
            current.radius,
            current.center,
            next.radius,
            next.center,
            self.pos_equal_eps,
        ) {
            CircleCircleIntr::NoIntersect => None,
            CircleCircleIntr::TangentIntersect { point } => process_intersect(point),
            CircleCircleIntr::TwoIntersects { point1, point2 } => {
                let dist1 = dist_squared(point1, shared_source_vertex);
                let dist2 = dist_squared(point2, shared_source_vertex);
                if self.squared_distances_equal(dist1, dist2) {
                    process_intersect(point1).or_else(|| process_intersect(point2))
                } else if dist1 < dist2 {
                    process_intersect(point1)
                } else {
                    process_intersect(point2)
                }
            }
            CircleCircleIntr::Overlapping => {
                process_intersect(next.start).or_else(|| process_intersect(current.end))
            }
        };

        result.unwrap_or_else(|| {
            Self::line_connection_boundary(JoinClass::InvalidInnerGap, current.end, next.start)
        })
    }

    fn add_first(&mut self, point: Vector2<T>) {
        debug_assert!(self.polyline.is_empty());
        self.polyline
            .add_vertex(PlineVertex::from_vector2(point, T::zero()));
    }

    /// Emits a segment unless its endpoints are fuzzy equal.
    fn emit_segment(&mut self, end: Vector2<T>, bulge: T, locally_invalid: bool) {
        let last = self.polyline.last().unwrap();

        if last.pos().fuzzy_eq_eps(end, self.pos_equal_eps) {
            return;
        }

        self.polyline.set_last(last.with_bulge(bulge));
        self.polyline
            .add_vertex(PlineVertex::from_vector2(end, T::zero()));

        if locally_invalid {
            self.invalid_segment_indexes
                .push(self.polyline.segment_count() - 1);
        }
    }

    fn emit_source_span(
        &mut self,
        source_index: usize,
        start: Vector2<T>,
        start_param: T,
        end: Vector2<T>,
        end_param: T,
    ) {
        debug_assert!(
            self.polyline
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(start, self.pos_equal_eps)
        );

        let raw_segment = self.raw_segments[source_index];
        let bulge = raw_segment.span_bulge(start_param, end_param);
        self.emit_segment(
            end,
            bulge,
            raw_segment.span_is_invalid(start_param, end_param),
        );
    }

    fn close_segment(&mut self, bulge: T, locally_invalid: bool) {
        let first_pos = self.polyline.at(0).pos();
        let last = self.polyline.last().unwrap();

        if last.pos().fuzzy_eq_eps(first_pos, self.pos_equal_eps) {
            return;
        }

        self.polyline.set_last(last.with_bulge(bulge));
        if locally_invalid {
            self.invalid_segment_indexes
                .push(self.polyline.segment_count());
        }
    }

    fn emit_join(&mut self, boundary: JoinBoundary<T>) {
        debug_assert!(
            self.polyline
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(boundary.current_end, self.pos_equal_eps)
        );
        self.emit_segment(
            boundary.next_start,
            boundary.connector_bulge,
            boundary.connector_is_invalid(),
        );
    }

    fn build_open(mut self) -> RawOffsetResult<O> {
        let first_start = self.raw_segments[0].start();
        self.add_first(first_start);
        let mut source_start = first_start;
        let mut source_start_param = T::zero();

        for current_index in 0..self.raw_segments.len() - 1 {
            let boundary = self.analyze_join(current_index, current_index + 1);
            self.emit_source_span(
                current_index,
                source_start,
                source_start_param,
                boundary.current_end,
                boundary.current_param,
            );
            self.emit_join(boundary);
            source_start = boundary.next_start;
            source_start_param = boundary.next_param;
        }

        let last_index = self.raw_segments.len() - 1;
        self.emit_source_span(
            last_index,
            source_start,
            source_start_param,
            self.raw_segments[last_index].end(),
            T::one(),
        );
        self.finish(false)
    }

    fn build_closed(mut self) -> RawOffsetResult<O> {
        let last_index = self.raw_segments.len() - 1;
        let closing_boundary = self.analyze_join(last_index, 0);

        // Start after the closing join.
        self.add_first(closing_boundary.next_start);
        let mut source_start = closing_boundary.next_start;
        let mut source_start_param = closing_boundary.next_param;

        for current_index in 0..last_index {
            let boundary = self.analyze_join(current_index, current_index + 1);
            self.emit_source_span(
                current_index,
                source_start,
                source_start_param,
                boundary.current_end,
                boundary.current_param,
            );
            self.emit_join(boundary);
            source_start = boundary.next_start;
            source_start_param = boundary.next_param;
        }

        // Fold a connector-free closing join into the final source span.
        let raw_segment = self.raw_segments[last_index];
        let (closing_bulge, is_invalid_source) = if closing_boundary
            .current_end
            .fuzzy_eq_eps(closing_boundary.next_start, self.pos_equal_eps)
        {
            (
                raw_segment.span_bulge(source_start_param, closing_boundary.current_param),
                raw_segment.span_is_invalid(source_start_param, closing_boundary.current_param),
            )
        } else {
            self.emit_source_span(
                last_index,
                source_start,
                source_start_param,
                closing_boundary.current_end,
                closing_boundary.current_param,
            );
            (
                closing_boundary.connector_bulge,
                closing_boundary.connector_is_invalid(),
            )
        };
        self.close_segment(closing_bulge, is_invalid_source);
        self.finish(true)
    }

    fn finish(mut self, is_closed: bool) -> RawOffsetResult<O> {
        // Remove a duplicate closing vertex before marking the output closed.
        if is_closed && self.polyline.vertex_count() > 1 {
            let first_pos = self.polyline.at(0).pos();
            if self
                .polyline
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(first_pos, self.pos_equal_eps)
            {
                self.polyline.remove_last();
            }
            self.polyline.set_is_closed(true);
        }

        // A lone vertex has no offset geometry.
        if self.polyline.vertex_count() == 1 {
            self.polyline.clear();
            self.invalid_segment_indexes.clear();
        }

        debug_assert!(
            self.invalid_segment_indexes
                .iter()
                .all(|&index| index < self.polyline.segment_count())
        );

        // Expand sparse invalid indexes into segment-aligned flags.
        let invalid_segment_indexes = self.invalid_segment_indexes;
        let mut invalid_segments = vec![false; self.polyline.segment_count()];
        for &index in &invalid_segment_indexes {
            invalid_segments[index] = true;
        }

        RawOffsetResult {
            polyline: self.polyline,
            invalid_segments,
            invalid_segment_indexes,
        }
    }
}

/// Creates one untrimmed offset segment per source segment.
pub fn create_untrimmed_raw_offset_segs<P, T>(polyline: &P, offset: T) -> Vec<RawOffsetSeg<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut result = Vec::with_capacity(polyline.segment_count());
    // Direct Vec pushes avoid poor codegen from extend(map(...)) with this mixed enum.
    for (v1, v2) in polyline.iter_segments() {
        let segment = if v1.bulge_is_zero() {
            // Offset the line along its left normal.
            let line = v2.pos() - v1.pos();
            let offset_vector = line.unit_perp().scale(offset);
            RawOffsetSeg::Line(RawOffsetLine {
                start: v1.pos() + offset_vector,
                end: v2.pos() + offset_vector,
            })
        } else {
            // Positive bulges sweep counterclockwise, so a positive offset reduces their radius.
            let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
            let is_clockwise = v1.bulge_is_neg();
            let offset_direction = if is_clockwise { offset } else { -offset };
            let radius_after_offset = arc_radius + offset_direction;
            let radial_scale = radius_after_offset / arc_radius;
            let start_radial = v1.pos() - arc_center;
            let end_radial = v2.pos() - arc_center;
            let start = arc_center + start_radial.scale(radial_scale);
            let end = arc_center + end_radial.scale(radial_scale);

            if radius_after_offset.fuzzy_lt(T::zero()) {
                // Past zero radius, retain a straight clipping scaffold and source tangents.
                let (start_tangent, end_tangent) = if is_clockwise {
                    (-start_radial.perp(), -end_radial.perp())
                } else {
                    (start_radial.perp(), end_radial.perp())
                };
                RawOffsetSeg::Collapsed(CollapsedRawOffsetArc {
                    start,
                    end,
                    start_tangent,
                    end_tangent,
                })
            } else {
                RawOffsetSeg::Arc(RawOffsetArc {
                    start,
                    end,
                    center: arc_center,
                    radius: radius_after_offset,
                    start_angle: angle(arc_center, start),
                    sweep: angle_from_bulge(v1.bulge),
                })
            }
        };
        result.push(segment);
    }
    result
}

/// Constructs raw offset geometry and marks locally invalid output segments.
pub fn create_raw_offset<P, T, O>(polyline: &P, offset: T, pos_equal_eps: T) -> RawOffsetResult<O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    if polyline.vertex_count() < 2 {
        return RawOffsetResult::empty();
    }

    // Preserve exact zero-offset geometry.
    if offset == T::zero() {
        let polyline = O::create_from(polyline);
        return RawOffsetResult {
            invalid_segments: vec![false; polyline.segment_count()],
            invalid_segment_indexes: Vec::new(),
            polyline,
        };
    }

    let raw_segments = create_untrimmed_raw_offset_segs(polyline, offset);

    // A lone collapsed arc cannot contribute valid output.
    if raw_segments.len() == 1 && raw_segments[0].is_collapsed() {
        return RawOffsetResult::empty();
    }

    let builder = RawOffsetBuilder::<P, T, O>::new(polyline, raw_segments, offset, pos_equal_eps);
    if polyline.is_closed() {
        builder.build_closed()
    } else {
        builder.build_open()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::traits::FuzzyEq;
    use crate::polyline::Polyline;

    fn open(vertexes: &[(f64, f64, f64)]) -> Polyline<f64> {
        let mut result = Polyline::new();
        for &(x, y, bulge) in vertexes {
            result.add(x, y, bulge);
        }
        result
    }

    fn closed(vertexes: &[(f64, f64, f64)]) -> Polyline<f64> {
        let mut result = open(vertexes);
        result.set_is_closed(true);
        result
    }

    fn join_at(input: &Polyline, offset: f64, current_index: usize) -> JoinBoundary<f64> {
        let raw_segments = create_untrimmed_raw_offset_segs(input, offset);
        let builder = RawOffsetBuilder::<_, _, Polyline>::new(input, raw_segments, offset, 1e-5);
        builder.analyze_join(current_index, current_index + 1)
    }

    #[test]
    fn raw_offset_segments_cache_line_arc_and_collapsed_geometry() {
        let input = open(&[(0.0, 0.0, 0.0), (2.0, 0.0, 1.0), (4.0, 0.0, 0.0)]);
        let segments = create_untrimmed_raw_offset_segs(&input, 0.5);
        assert!(matches!(segments[0], RawOffsetSeg::Line(_)));
        assert!(matches!(segments[1], RawOffsetSeg::Arc(_)));

        let collapsed = create_untrimmed_raw_offset_segs(&input, 2.0);
        assert!(matches!(collapsed[1], RawOffsetSeg::Collapsed(_)));
    }
    #[test]
    fn raw_arc_span_bulge_uses_cached_sweep_and_trim_parameters() {
        let input = open(&[(0.0, 0.0, 1.0), (2.0, 0.0, 0.0)]);
        let segment = create_untrimmed_raw_offset_segs(&input, -0.5)[0];
        let RawOffsetSeg::Arc(arc) = segment else {
            panic!("expected an arc");
        };

        assert!((segment.span_bulge(0.0, 1.0) - bulge_from_angle(arc.sweep)).abs() < f64::EPSILON);
        assert!(
            (segment.span_bulge(0.25, 0.75) - bulge_from_angle(arc.sweep * 0.5)).abs()
                < f64::EPSILON
        );
        assert!(
            (segment.span_bulge(0.75, 0.25) - bulge_from_angle(arc.sweep * -0.5)).abs()
                < f64::EPSILON
        );
    }

    #[test]
    fn round_join_bulge_matches_sweep_and_offset_direction() {
        let source = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]);
        for sweep in [
            1e-12,
            1e-8,
            1e-4,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI - 1e-8,
            std::f64::consts::PI,
        ] {
            let next = Vector2::new(sweep.cos(), sweep.sin());
            for offset in [-1.0, 1.0] {
                let builder =
                    RawOffsetBuilder::<_, _, Polyline>::new(&source, Vec::new(), offset, 1e-5);
                let boundary = builder.arc_connection_boundary(
                    JoinClass::OuterRound,
                    Vector2::new(1.0, 0.0),
                    next,
                    Vector2::zero(),
                );
                let expected = (sweep / 4.0).tan().copysign(-offset);
                assert!(
                    boundary.connector_bulge.fuzzy_eq_eps(expected, 5e-15),
                    "{sweep}, {offset}: {} != {expected}",
                    boundary.connector_bulge
                );
            }
        }
    }
    #[test]
    fn normal_raw_offsets_have_no_invalid_segments() {
        let input = open(&[(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]);
        let result: RawOffsetResult<Polyline> = create_raw_offset(&input, -1.0, 1e-5);
        assert!(!result.invalid_segments.contains(&true));
        assert_eq!(result.polyline.segment_count(), 3);
    }
    #[test]
    fn reported_raw_offset_has_an_invalid_source_span() {
        let input = open(&[
            (-450.5191502893827, -43.535303351368704, 0.0),
            (-451.1680760707164, -42.2734516183154, 0.0),
            (-451.52354482078823, -41.663725063291785, 0.0),
            (-451.9166112272101, -41.09344423983177, 0.0),
            (-452.0787206022149, -40.854568155450046, 0.0),
        ]);
        let result: RawOffsetResult<Polyline> = create_raw_offset(&input, 11.0, 1e-5);
        assert_eq!(result.invalid_segments, [false, true, false, false, false]);

        let source_segments = create_untrimmed_raw_offset_segs(&input, 11.0);
        let RawOffsetSeg::Line(source_segment) = source_segments[1] else {
            panic!("expected a line");
        };
        let start = result.polyline.at(1).pos();
        let end = result.polyline.at(2).pos();
        let start_param = source_segment.param_at(start, 1e-5);
        let end_param = source_segment.param_at(end, 1e-5);
        assert!((start_param - 0.4117844684788834).abs() < 1e-9);
        assert!((end_param - 0.4101801862932303).abs() < 1e-9);
        assert!(end_param < start_param);
    }
    #[test]
    fn collapsed_arc_spans_are_invalid() {
        let input = open(&[(0.0, 0.0, 1.0), (2.0, 0.0, 0.0), (3.0, 1.0, 0.0)]);
        let result: RawOffsetResult<Polyline> = create_raw_offset(&input, 2.0, 1e-5);
        assert!(result.invalid_segments.contains(&true));
    }
    #[test]
    fn closed_raw_offsets_have_segment_aligned_validity() {
        let fixtures = [
            closed(&[(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]),
            closed(&[(0.0, 0.0, 0.5), (10.0, 0.0, 0.0), (10.0, 10.0, -0.5)]),
        ];

        for input in fixtures {
            for offset in [-1.0, 1.0] {
                let result: RawOffsetResult<Polyline> = create_raw_offset(&input, offset, 1e-5);
                assert!(result.polyline.is_closed());
                assert_eq!(
                    result.invalid_segments.len(),
                    result.polyline.segment_count()
                );
            }
        }
    }
    #[test]
    fn line_line_joins_have_explicit_local_classes() {
        let corner = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]);
        assert_eq!(join_at(&corner, 0.2, 0).class, JoinClass::Trimmed);
        assert_eq!(join_at(&corner, -0.2, 0).class, JoinClass::OuterRound);
        assert_eq!(join_at(&corner, 2.0, 0).class, JoinClass::InvalidInnerGap);

        let straight = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]);
        assert_eq!(join_at(&straight, 0.2, 0).class, JoinClass::Coincident);

        let reversal = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]);
        assert_eq!(join_at(&reversal, 0.2, 0).class, JoinClass::ReversalRound);
    }
    #[test]
    fn near_collinear_corner_uses_normalized_angular_tolerance() {
        let source = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 1.0e-10, 0.0)]);
        let raw_segments = create_untrimmed_raw_offset_segs(&source, 1.0e6);
        let builder = RawOffsetBuilder::<_, _, Polyline>::new(&source, raw_segments, 1.0e6, 1e-5);
        assert_eq!(
            builder.analyze_join(0, 1).class,
            JoinClass::InvalidDegenerateGap
        );
    }
    #[test]
    fn mixed_and_arc_joins_have_explicit_local_classes() {
        let line_arc = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)]);
        assert_eq!(join_at(&line_arc, -0.2, 0).class, JoinClass::Trimmed);
        assert_eq!(join_at(&line_arc, 0.2, 0).class, JoinClass::OuterRound);
        let short_line_arc = open(&[(0.9, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)]);
        assert_eq!(
            join_at(&short_line_arc, -2.0, 0).class,
            JoinClass::InvalidInnerGap
        );

        let arc_line = open(&[(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]);
        assert_eq!(join_at(&arc_line, -0.2, 0).class, JoinClass::Trimmed);
        assert_eq!(join_at(&arc_line, 0.2, 0).class, JoinClass::OuterRound);
        let arc_short_line = open(&[(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.1, 0.0, 0.0)]);
        assert_eq!(
            join_at(&arc_short_line, -2.0, 0).class,
            JoinClass::InvalidInnerGap
        );

        let arc_arc = open(&[(0.0, 0.0, 1.0), (1.0, 0.0, -1.0), (1.0, -1.0, 0.0)]);
        assert_eq!(join_at(&arc_arc, -0.2, 0).class, JoinClass::Trimmed);
        assert_eq!(join_at(&arc_arc, 0.2, 0).class, JoinClass::OuterRound);
        assert_eq!(join_at(&arc_arc, -2.0, 0).class, JoinClass::InvalidInnerGap);
    }
    #[test]
    fn inner_join_endpoint_tolerance_clamps_the_local_candidate() {
        let source = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]);
        let raw_segments = create_untrimmed_raw_offset_segs(&source, 0.2);
        let builder = RawOffsetBuilder::<_, _, Polyline>::new(&source, raw_segments, 0.2, 1e-5);
        let current = RawOffsetLine {
            start: Vector2::new(0.0, 0.0),
            end: Vector2::new(1.0, 0.0),
        };
        let next = RawOffsetLine {
            start: Vector2::new(1.0 + 5e-6, -1.0),
            end: Vector2::new(1.0 + 5e-6, 1.0),
        };

        let boundary = builder.line_line_join(&current, &next);
        assert_eq!(boundary.class, JoinClass::Trimmed);
        assert!((boundary.current_param - 1.0).abs() < f64::EPSILON);
        assert_eq!(boundary.current_end, current.end);
    }
    #[test]
    fn unusable_nearest_line_arc_candidate_does_not_select_farther_crossing() {
        let source = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)]);
        let raw_segments = create_untrimmed_raw_offset_segs(&source, -0.2);
        let builder = RawOffsetBuilder::<_, _, Polyline>::new(&source, raw_segments, -0.2, 1e-5);
        let line = RawOffsetLine {
            start: Vector2::new(0.5, 0.0),
            end: Vector2::new(2.0, 0.0),
        };
        let arc = RawOffsetArc {
            start: Vector2::new(1.0, 0.0),
            end: Vector2::new(0.0, 1.0),
            center: Vector2::zero(),
            radius: 1.0,
            start_angle: 0.0,
            sweep: std::f64::consts::FRAC_PI_2,
        };

        assert_eq!(
            builder
                .line_arc_join(&line, &arc, Vector2::new(-1.0, 0.0))
                .class,
            JoinClass::InvalidInnerGap
        );
        assert_eq!(
            builder.line_arc_join(&line, &arc, Vector2::zero()).class,
            JoinClass::Trimmed
        );
    }
    #[test]
    fn failed_inner_join_connector_is_recorded_as_invalid() {
        let input = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]);
        let result: RawOffsetResult<Polyline> = create_raw_offset(&input, 2.0, 1e-5);
        assert_eq!(result.polyline.segment_count(), 3);
        assert_eq!(result.invalid_segments, [false, true, false]);
    }
    #[test]
    fn zero_raw_offset_is_an_exact_copy() {
        let mut input = closed(&[
            (0.0, 0.0, 0.5),
            (2.0, 0.0, 0.0),
            (2.0, 0.0, -0.5),
            (0.0, 2.0, 0.0),
        ]);
        input.set_userdata_values([3, 5]);

        let result: RawOffsetResult<Polyline> = create_raw_offset(&input, -0.0, 1e-5);
        assert_eq!(
            result.polyline.iter_vertexes().collect::<Vec<_>>(),
            input.iter_vertexes().collect::<Vec<_>>()
        );
        assert_eq!(result.polyline.is_closed(), input.is_closed());
        assert_eq!(
            result.polyline.get_userdata_values().collect::<Vec<_>>(),
            [3, 5]
        );
        assert!(!result.invalid_segments.contains(&true));
    }
}
