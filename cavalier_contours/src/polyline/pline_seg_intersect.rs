use super::PlineVertex;
use super::pline_seg::seg_arc_radius_and_center;
use crate::core::{
    math::Vector2,
    math::{
        CircleCircleIntr, LineCircleIntr, LineLineIntr, circle_circle_intr, dist_squared,
        line_circle_intr, line_line_intr, point_from_parametric,
    },
    traits::Real,
};

#[derive(Copy, Clone)]
struct ArcSweep<T>
where
    T: Real,
{
    center: Vector2<T>,
    start_vector: Vector2<T>,
    end_vector: Vector2<T>,
    is_clockwise: bool,
    epsilon_squared: T,
    start_fuzzy_limit: T,
    end_fuzzy_limit: T,
}

impl<T> ArcSweep<T>
where
    T: Real,
{
    #[inline]
    fn new(
        center: Vector2<T>,
        start: Vector2<T>,
        end: Vector2<T>,
        is_clockwise: bool,
        epsilon: T,
    ) -> Self {
        let start_vector = start - center;
        let end_vector = end - center;
        let epsilon_squared = epsilon * epsilon;
        Self {
            center,
            start_vector,
            end_vector,
            is_clockwise,
            epsilon_squared,
            start_fuzzy_limit: epsilon_squared * start_vector.length_squared(),
            end_fuzzy_limit: epsilon_squared * end_vector.length_squared(),
        }
    }

    #[inline]
    fn contains(&self, point: Vector2<T>) -> bool {
        let point_vector = point - self.center;
        if point_vector.length_squared() < self.epsilon_squared {
            return true;
        }

        let start_cross = self.start_vector.perp_dot(point_vector);
        let end_cross = self.end_vector.perp_dot(point_vector);
        let exactly_within_sweep = if self.is_clockwise {
            start_cross <= T::zero() && end_cross >= T::zero()
        } else {
            start_cross >= T::zero() && end_cross <= T::zero()
        };
        if exactly_within_sweep {
            return true;
        }

        (self.start_vector.dot(point_vector) >= T::zero()
            && start_cross * start_cross < self.start_fuzzy_limit)
            || (self.end_vector.dot(point_vector) >= T::zero()
                && end_cross * end_cross < self.end_fuzzy_limit)
    }
}

#[derive(Copy, Clone)]
struct ArcGeometry<T>
where
    T: Real,
{
    sweep: ArcSweep<T>,
    radius_lower_bound: T,
    radius_lower_bound_squared: T,
    radius_upper_bound_squared: T,
}

impl<T> ArcGeometry<T>
where
    T: Real,
{
    #[inline]
    fn new(
        center: Vector2<T>,
        start: Vector2<T>,
        end: Vector2<T>,
        is_clockwise: bool,
        radius: T,
        epsilon: T,
    ) -> Self {
        let radius_lower_bound = radius - epsilon;
        let radius_upper_bound = radius + epsilon;
        Self {
            sweep: ArcSweep::new(center, start, end, is_clockwise, epsilon),
            radius_lower_bound,
            radius_lower_bound_squared: radius_lower_bound * radius_lower_bound,
            radius_upper_bound_squared: radius_upper_bound * radius_upper_bound,
        }
    }

    #[inline]
    fn sweeps(&self, point: Vector2<T>) -> bool {
        self.sweep.contains(point)
    }

    #[inline]
    fn point_lies_on_arc(&self, point: Vector2<T>) -> bool {
        let distance_squared = dist_squared(point, self.sweep.center);
        distance_squared < self.radius_upper_bound_squared
            && (self.radius_lower_bound < T::zero()
                || distance_squared > self.radius_lower_bound_squared)
            && self.sweeps(point)
    }
}

/// Holds the result of finding the intersect between two polyline segments.
#[derive(Debug, Copy, Clone)]
pub enum PlineSegIntr<T>
where
    T: Real,
{
    /// No intersects found.
    NoIntersect,
    /// One tangent intersect point found.
    TangentIntersect {
        /// Holds the tangent intersect point.
        point: Vector2<T>,
    },
    /// One non-tangent intersect point found.
    OneIntersect {
        /// Holds the intersect point.
        point: Vector2<T>,
    },
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// Holds the first intersect point (according to the second segment direction).
        point1: Vector2<T>,
        /// Holds the second intersect point (according to the second segment direction).
        point2: Vector2<T>,
    },
    /// Polyline segments are both lines and they overlap.
    OverlappingLines {
        /// Holds the start (according to the second segment direction) point of the line overlap.
        point1: Vector2<T>,
        /// Holds the end (according to the second segment direction) point of the line overlap.
        point2: Vector2<T>,
    },
    /// Polyline segments are both arcs and they overlap.
    OverlappingArcs {
        /// Holds the start (according to the second segment direction) point of the arc overlap.
        point1: Vector2<T>,
        /// Holds the end (according to the second segment direction) point of the arc overlap.
        point2: Vector2<T>,
    },
}

/// Finds the intersects between two polyline segments.
///
/// Segments are defined by `v1->v2` and `u1->u2`. `pos_equal_eps` is used for fuzzy float
/// comparisons.
#[must_use]
pub fn pline_seg_intr<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    u1: PlineVertex<T>,
    u2: PlineVertex<T>,
    pos_equal_eps: T,
) -> PlineSegIntr<T>
where
    T: Real,
{
    use PlineSegIntr::{
        NoIntersect, OneIntersect, OverlappingArcs, TangentIntersect, TwoIntersects,
    };
    let v_is_line = v1.bulge_is_zero();
    let u_is_line = u1.bulge_is_zero();

    if v_is_line && u_is_line {
        let intr_result = line_line_intr(v1.pos(), v2.pos(), u1.pos(), u2.pos(), pos_equal_eps);
        match intr_result {
            LineLineIntr::NoIntersect | LineLineIntr::FalseIntersect { .. } => {
                return NoIntersect;
            }
            LineLineIntr::TrueIntersect { seg1_t, .. } => {
                return OneIntersect {
                    point: point_from_parametric(v1.pos(), v2.pos(), seg1_t),
                };
            }
            LineLineIntr::Overlapping { seg2_t0, seg2_t1 } => {
                return PlineSegIntr::OverlappingLines {
                    point1: point_from_parametric(u1.pos(), u2.pos(), seg2_t0),
                    point2: point_from_parametric(u1.pos(), u2.pos(), seg2_t1),
                };
            }
        }
    }

    let process_line_arc_intr = |p0: Vector2<T>,
                                 p1: Vector2<T>,
                                 a1: PlineVertex<T>,
                                 a2: PlineVertex<T>|
     -> PlineSegIntr<T> {
        let (arc_radius, arc_center) = seg_arc_radius_and_center(a1, a2);

        // Note if intersect is detected we check if the line segment starts or ends on the arc
        // segment and if so then use that end point as the intersect point.
        // Why: this avoids inconsistencies between segment intersects where a line may "overlap" an
        // arc according to the fuzzy epsilon values (e.g., imagine the arc has a large radius and
        // the line has two intersects but is almost tangent to the arc), in such a case the
        // line-circle intersect function will return two solutions, one on either side of the end
        // point, but the end point is an equally valid solution according to the fuzzy epsilon and
        // ensures consistency with other intersects. E.g., if the end of the line segment is the
        // start of an arc that overlaps with another arc then we want the overlap intersect end
        // points to agree with the intersect returned from this function, to ensure this
        // consistency we use the end point when valid to do so (end points are "sticky").
        let intr_result = line_circle_intr(p0, p1, arc_radius, arc_center, pos_equal_eps);
        if matches!(intr_result, LineCircleIntr::NoIntersect) {
            return NoIntersect;
        }

        let arc_geometry = ArcGeometry::new(
            arc_center,
            a1.pos(),
            a2.pos(),
            a1.bulge_is_neg(),
            arc_radius,
            pos_equal_eps,
        );

        // line segment length used for scaling parametric t value for fuzzy comparing
        let line_length = (p1 - p0).length();

        let point_in_sweep = |t: T| -> Option<Vector2<T>> {
            if !(t * line_length).fuzzy_in_range_eps(T::zero(), line_length, pos_equal_eps) {
                return None;
            }

            let p = point_from_parametric(p0, p1, t);
            let within_sweep = arc_geometry.sweeps(p);
            if within_sweep { Some(p) } else { None }
        };

        match intr_result {
            LineCircleIntr::NoIntersect => unreachable!(),
            LineCircleIntr::TangentIntersect { t0 } => {
                // check if either end point lies on the arc and substitute intersect point with end
                // point if so
                if arc_geometry.point_lies_on_arc(p0) {
                    TangentIntersect { point: p0 }
                } else if arc_geometry.point_lies_on_arc(p1) {
                    TangentIntersect { point: p1 }
                } else if let Some(point) = point_in_sweep(t0) {
                    TangentIntersect { point }
                } else {
                    NoIntersect
                }
            }
            LineCircleIntr::TwoIntersects { t0, t1 } => {
                let t0_point = point_in_sweep(t0);
                let t1_point = point_in_sweep(t1);
                match (t0_point, t1_point) {
                    (None, None) => NoIntersect,
                    (None, Some(point)) | (Some(point), None) => {
                        // check if either end point lies on arc and substitute intersect point with
                        // end point if so
                        if arc_geometry.point_lies_on_arc(p0) {
                            OneIntersect { point: p0 }
                        } else if arc_geometry.point_lies_on_arc(p1) {
                            OneIntersect { point: p1 }
                        } else {
                            OneIntersect { point }
                        }
                    }
                    (Some(point1), Some(point2)) => {
                        // check if either end point lies on arc and substitute intersect point with
                        // end point if so (using distance check to determine which to substitute)
                        let (point1, point2) = match (
                            arc_geometry.point_lies_on_arc(p0),
                            arc_geometry.point_lies_on_arc(p1),
                        ) {
                            (true, true) => {
                                if dist_squared(p0, point1) < dist_squared(p0, point2) {
                                    // substitute point1 with p0, point2 with p1
                                    (p0, p1)
                                } else {
                                    // substitute point1 with p1, point2 with p0
                                    (p1, p0)
                                }
                            }
                            (true, false) => {
                                if dist_squared(p0, point1) < dist_squared(p0, point2) {
                                    // substitute point1 with p0
                                    (p0, point2)
                                } else {
                                    // substitute point2 with p0
                                    (point1, p0)
                                }
                            }
                            (false, true) => {
                                if dist_squared(p1, point1) < dist_squared(p1, point2) {
                                    // substitute point1 with p1
                                    (p1, point2)
                                } else {
                                    // substitute point2 with p1
                                    (point1, p1)
                                }
                            }
                            (false, false) => {
                                // no substitutions
                                (point1, point2)
                            }
                        };

                        // return points ordered according to second segment direction
                        if u_is_line
                            || (dist_squared(point1, a1.pos()) < dist_squared(point2, a1.pos()))
                        {
                            TwoIntersects { point1, point2 }
                        } else {
                            TwoIntersects {
                                point1: point2,
                                point2: point1,
                            }
                        }
                    }
                }
            }
        }
    };

    if v_is_line {
        // v is line, u is arc
        return process_line_arc_intr(v1.pos(), v2.pos(), u1, u2);
    }

    if u_is_line {
        // u is line, v is arc
        return process_line_arc_intr(u1.pos(), u2.pos(), v1, v2);
    }

    // both v and u are arcs
    let (arc1_radius, arc1_center) = seg_arc_radius_and_center(v1, v2);
    let (arc2_radius, arc2_center) = seg_arc_radius_and_center(u1, u2);

    let intr_result = circle_circle_intr(
        arc1_radius,
        arc1_center,
        arc2_radius,
        arc2_center,
        pos_equal_eps,
    );

    match intr_result {
        CircleCircleIntr::NoIntersect => NoIntersect,
        CircleCircleIntr::TangentIntersect { point } => {
            let arc1 = ArcGeometry::new(
                arc1_center,
                v1.pos(),
                v2.pos(),
                v1.bulge_is_neg(),
                arc1_radius,
                pos_equal_eps,
            );
            let arc2 = ArcGeometry::new(
                arc2_center,
                u1.pos(),
                u2.pos(),
                u1.bulge_is_neg(),
                arc2_radius,
                pos_equal_eps,
            );
            // first check if end points lie on arcs and substitute with end point if so to be
            // consistent with stickiness to end points done in other cases (e.g., line-arc
            // intersect)
            if arc1.point_lies_on_arc(u1.pos()) {
                TangentIntersect { point: u1.pos() }
            } else if arc1.point_lies_on_arc(u2.pos()) {
                TangentIntersect { point: u2.pos() }
            } else if arc2.point_lies_on_arc(v1.pos()) {
                TangentIntersect { point: v1.pos() }
            } else if arc2.point_lies_on_arc(v2.pos()) {
                TangentIntersect { point: v2.pos() }
            } else if arc1.sweeps(point) && arc2.sweeps(point) {
                TangentIntersect { point }
            } else {
                NoIntersect
            }
        }
        CircleCircleIntr::TwoIntersects { point1, point2 } => {
            let arc1 = ArcGeometry::new(
                arc1_center,
                v1.pos(),
                v2.pos(),
                v1.bulge_is_neg(),
                arc1_radius,
                pos_equal_eps,
            );
            let arc2 = ArcGeometry::new(
                arc2_center,
                u1.pos(),
                u2.pos(),
                u1.bulge_is_neg(),
                arc2_radius,
                pos_equal_eps,
            );
            // determine if end points lie on arcs and substitute with end points if so to be
            // consistent with stickiness to end points done in other cases (e.g., line-arc
            // intersect)
            let mut end_point_intrs: [Option<Vector2<T>>; 2] = [None; 2];
            // helper function to collect end point intersects
            let mut try_add_end_point_intr = |intr: Vector2<T>| {
                for slot in &mut end_point_intrs {
                    if let Some(pt) = slot {
                        if pt.fuzzy_eq_eps(intr, pos_equal_eps) {
                            // duplicate point, skip it (end point from both arcs touch)
                            break;
                        }
                    } else {
                        // insert the end point as intersect
                        *slot = Some(intr);
                        break;
                    }
                }
            };

            if arc1.point_lies_on_arc(u1.pos()) {
                try_add_end_point_intr(u1.pos());
            }

            if arc1.point_lies_on_arc(u2.pos()) {
                try_add_end_point_intr(u2.pos());
            }

            if arc2.point_lies_on_arc(v1.pos()) {
                try_add_end_point_intr(v1.pos());
            }

            if arc2.point_lies_on_arc(v2.pos()) {
                try_add_end_point_intr(v2.pos());
            }

            let pt1_in_sweep = arc1.sweeps(point1) && arc2.sweeps(point1);
            let pt2_in_sweep = arc1.sweeps(point2) && arc2.sweeps(point2);
            if pt1_in_sweep && pt2_in_sweep {
                match (end_point_intrs[0], end_point_intrs[1]) {
                    (None, None) => TwoIntersects { point1, point2 },
                    (None, Some(end_pt)) | (Some(end_pt), None) => {
                        if dist_squared(end_pt, point1) < dist_squared(end_pt, point2) {
                            TwoIntersects {
                                point1: end_pt,
                                point2,
                            }
                        } else {
                            TwoIntersects {
                                point1,
                                point2: end_pt,
                            }
                        }
                    }
                    (Some(end_pt1), Some(end_pt2)) => {
                        if dist_squared(end_pt1, point1) < dist_squared(end_pt2, point1) {
                            TwoIntersects {
                                point1: end_pt1,
                                point2: end_pt2,
                            }
                        } else {
                            TwoIntersects {
                                point1: end_pt2,
                                point2: end_pt1,
                            }
                        }
                    }
                }
            } else if pt1_in_sweep {
                match (end_point_intrs[0], end_point_intrs[1]) {
                    (None, None) => OneIntersect { point: point1 },
                    (None, Some(end_pt)) | (Some(end_pt), None) => OneIntersect { point: end_pt },
                    (Some(end_pt1), Some(end_pt2)) => TwoIntersects {
                        point1: end_pt1,
                        point2: end_pt2,
                    },
                }
            } else if pt2_in_sweep {
                match (end_point_intrs[0], end_point_intrs[1]) {
                    (None, None) => OneIntersect { point: point2 },
                    (None, Some(end_pt)) | (Some(end_pt), None) => OneIntersect { point: end_pt },
                    (Some(end_pt1), Some(end_pt2)) => TwoIntersects {
                        point1: end_pt1,
                        point2: end_pt2,
                    },
                }
            } else {
                match (end_point_intrs[0], end_point_intrs[1]) {
                    (None, None) => NoIntersect,
                    (None, Some(end_pt)) | (Some(end_pt), None) => OneIntersect { point: end_pt },
                    (Some(end_pt1), Some(end_pt2)) => TwoIntersects {
                        point1: end_pt1,
                        point2: end_pt2,
                    },
                }
            }
        }
        CircleCircleIntr::Overlapping => {
            if (v1.pos() == u1.pos() && v2.pos() == u2.pos() && v1.bulge == u1.bulge)
                || (v1.pos() == u2.pos() && v2.pos() == u1.pos() && v1.bulge == -u1.bulge)
            {
                return OverlappingArcs {
                    point1: u1.pos(),
                    point2: u2.pos(),
                };
            }

            let same_direction_arcs = v1.bulge_is_neg() == u1.bulge_is_neg();
            // Make both arc sweeps go the same direction to simplify the overlap checks.
            let (arc2_start, arc2_end) = if same_direction_arcs {
                (u1.pos(), u2.pos())
            } else {
                (u2.pos(), u1.pos())
            };
            let avg_radius = (arc1_radius + arc2_radius) / T::two();

            // Check if only endpoints touch. Chord distance is a close lower bound for arc distance
            // at the position tolerance scale and avoids computing angles.
            let position_epsilon_squared = pos_equal_eps * pos_equal_eps;
            match (
                dist_squared(v1.pos(), arc2_end) < position_epsilon_squared,
                dist_squared(arc2_start, v2.pos()) < position_epsilon_squared,
            ) {
                (true, true) => {
                    // two half circle arcs with end points touching
                    // note: point1 and point2 are returned in order according to second segment
                    // (u1->u2) direction
                    TwoIntersects {
                        point1: u1.pos(),
                        point2: u2.pos(),
                    }
                }
                (true, false) => {
                    // only touch at start of arc1
                    OneIntersect { point: v1.pos() }
                }
                (false, true) => {
                    // only touch at start of arc2
                    // NOTE: have to check and adjust for the direction flip done above to have
                    // matching direction
                    let point = if same_direction_arcs {
                        u1.pos()
                    } else {
                        u2.pos()
                    };
                    OneIntersect { point }
                }
                (false, false) => {
                    // Preserve the fixed angular tolerance used for overlap containment by scaling
                    // it to a position tolerance at the average fuzzy-equal radius.
                    let sweep_epsilon = avg_radius * T::fuzzy_epsilon();
                    let arc1_sweep = ArcSweep::new(
                        arc1_center,
                        v1.pos(),
                        v2.pos(),
                        v1.bulge_is_neg(),
                        sweep_epsilon,
                    );
                    let arc2_sweep = ArcSweep::new(
                        arc2_center,
                        arc2_start,
                        arc2_end,
                        v1.bulge_is_neg(),
                        sweep_epsilon,
                    );
                    let arc2_starts_in_arc1 = arc1_sweep.contains(arc2_start);
                    let arc2_ends_in_arc1 = arc1_sweep.contains(arc2_end);
                    if arc2_starts_in_arc1 && arc2_ends_in_arc1 {
                        // arc2 is fully overlapped by arc1
                        OverlappingArcs {
                            point1: u1.pos(),
                            point2: u2.pos(),
                        }
                    } else if arc2_starts_in_arc1 {
                        // check if direction reversed to ensure the correct points are used
                        // note: point1 and point2 are returned in order according to second segment
                        // (u1->u2) direction
                        if same_direction_arcs {
                            OverlappingArcs {
                                point1: u1.pos(),
                                point2: v2.pos(),
                            }
                        } else {
                            OverlappingArcs {
                                point1: v2.pos(),
                                point2: u2.pos(),
                            }
                        }
                    } else if arc2_ends_in_arc1 {
                        // check if direction reversed to ensure the correct points are used
                        // note: point1 and point2 are returned in order according to second segment
                        // (u1->u2) direction
                        if same_direction_arcs {
                            OverlappingArcs {
                                point1: v1.pos(),
                                point2: u2.pos(),
                            }
                        } else {
                            OverlappingArcs {
                                point1: u1.pos(),
                                point2: v1.pos(),
                            }
                        }
                    } else {
                        if arc2_sweep.contains(v1.pos()) {
                            // arc1 is fully overlapped by arc2
                            // note: point1 and point2 are returned in order according to second
                            // segment (u1->u2) direction
                            if same_direction_arcs {
                                OverlappingArcs {
                                    point1: v1.pos(),
                                    point2: v2.pos(),
                                }
                            } else {
                                OverlappingArcs {
                                    point1: v2.pos(),
                                    point2: v1.pos(),
                                }
                            }
                        } else {
                            NoIntersect
                        }
                    }
                }
            }
        }
    }
}
