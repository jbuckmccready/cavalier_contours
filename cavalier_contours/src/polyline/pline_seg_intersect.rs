use super::pline_seg::seg_arc_radius_and_center;
use super::PlineVertex;
use crate::core::{
    math::Vector2,
    math::{
        angle, angle_from_bulge, angle_is_within_sweep, circle_circle_intr, delta_angle,
        dist_squared, line_circle_intr, line_line_intr, normalize_radians, point_from_parametric,
        point_within_arc_sweep, CircleCircleIntr, LineCircleIntr, LineLineIntr,
    },
    traits::Real,
};

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
    use PlineSegIntr::*;
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

        let point_lies_on_arc = |pt: Vector2<T>| -> bool {
            point_within_arc_sweep(
                arc_center,
                a1.pos(),
                a2.pos(),
                a1.bulge_is_neg(),
                pt,
                pos_equal_eps,
            ) && dist_squared(pt, arc_center)
                .sqrt()
                .fuzzy_eq_eps(arc_radius, pos_equal_eps)
        };

        // line segment length used for scaling parametric t value for fuzzy comparing
        let line_length = (p1 - p0).length();

        let point_in_sweep = |t: T| -> Option<Vector2<T>> {
            if !(t * line_length).fuzzy_in_range_eps(T::zero(), line_length, pos_equal_eps) {
                return None;
            }

            let p = point_from_parametric(p0, p1, t);
            let within_sweep = point_within_arc_sweep(
                arc_center,
                a1.pos(),
                a2.pos(),
                a1.bulge_is_neg(),
                p,
                pos_equal_eps,
            );
            if within_sweep {
                Some(p)
            } else {
                None
            }
        };

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
        match intr_result {
            LineCircleIntr::NoIntersect => NoIntersect,
            LineCircleIntr::TangentIntersect { t0 } => {
                // check if either end point lies on the arc and substitute intersect point with end
                // point if so
                if point_lies_on_arc(p0) {
                    TangentIntersect { point: p0 }
                } else if point_lies_on_arc(p1) {
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
                        if point_lies_on_arc(p0) {
                            OneIntersect { point: p0 }
                        } else if point_lies_on_arc(p1) {
                            OneIntersect { point: p1 }
                        } else {
                            OneIntersect { point }
                        }
                    }
                    (Some(point1), Some(point2)) => {
                        // check if either end point lies on arc and substitute intersect point with
                        // end point if so (using distance check to determine which to substitute)
                        let (point1, point2) = match (point_lies_on_arc(p0), point_lies_on_arc(p1))
                        {
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

    let start_and_sweep_angle = |sp: Vector2<T>, center: Vector2<T>, bulge: T| -> (T, T) {
        let start_angle = normalize_radians(angle(center, sp));
        let sweep_angle = angle_from_bulge(bulge);
        (start_angle, sweep_angle)
    };

    // helper function to test if both arcs sweep a point
    let both_arcs_sweep_point = |pt: Vector2<T>| -> bool {
        point_within_arc_sweep(
            arc1_center,
            v1.pos(),
            v2.pos(),
            v1.bulge_is_neg(),
            pt,
            pos_equal_eps,
        ) && point_within_arc_sweep(
            arc2_center,
            u1.pos(),
            u2.pos(),
            u1.bulge_is_neg(),
            pt,
            pos_equal_eps,
        )
    };

    // helper function to test if a point lies on arc1 segment
    let point_lies_on_arc1 = |pt: Vector2<T>| -> bool {
        point_within_arc_sweep(
            arc1_center,
            v1.pos(),
            v2.pos(),
            v1.bulge_is_neg(),
            pt,
            pos_equal_eps,
        ) && dist_squared(pt, arc1_center)
            .sqrt()
            .fuzzy_eq_eps(arc1_radius, pos_equal_eps)
    };

    // helper function to test if a point lies on arc2 segment
    let point_lies_on_arc2 = |pt: Vector2<T>| -> bool {
        point_within_arc_sweep(
            arc2_center,
            u1.pos(),
            u2.pos(),
            u1.bulge_is_neg(),
            pt,
            pos_equal_eps,
        ) && dist_squared(pt, arc2_center)
            .sqrt()
            .fuzzy_eq_eps(arc2_radius, pos_equal_eps)
    };

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
            // first check if end points lie on arcs and substitute with end point if so to be
            // consistent with stickiness to end points done in other cases (e.g., line-arc
            // intersect)
            if point_lies_on_arc1(u1.pos()) {
                TangentIntersect { point: u1.pos() }
            } else if point_lies_on_arc1(u2.pos()) {
                TangentIntersect { point: u2.pos() }
            } else if point_lies_on_arc2(v1.pos()) {
                TangentIntersect { point: v1.pos() }
            } else if point_lies_on_arc2(v2.pos()) {
                TangentIntersect { point: v2.pos() }
            } else if both_arcs_sweep_point(point) {
                TangentIntersect { point }
            } else {
                NoIntersect
            }
        }
        CircleCircleIntr::TwoIntersects { point1, point2 } => {
            // determine if end points lie on arcs and substitute with end points if so to be
            // consistent with stickiness to end points done in other cases (e.g., line-arc
            // intersect)
            let mut end_point_intrs: [Option<Vector2<T>>; 2] = [None; 2];
            // helper function to collect end point intersects
            let mut try_add_end_point_intr = |intr: Vector2<T>| {
                for slot in end_point_intrs.iter_mut() {
                    match slot {
                        Some(pt) => {
                            if pt.fuzzy_eq_eps(intr, pos_equal_eps) {
                                // duplicate point, skip it (end point from both arcs touch)
                                break;
                            }
                        }
                        None => {
                            // insert the end point as intersect
                            *slot = Some(intr);
                            break;
                        }
                    }
                }
            };

            if point_lies_on_arc1(u1.pos()) {
                try_add_end_point_intr(u1.pos());
            }

            if point_lies_on_arc1(u2.pos()) {
                try_add_end_point_intr(u2.pos());
            }

            if point_lies_on_arc2(v1.pos()) {
                try_add_end_point_intr(v1.pos());
            }

            if point_lies_on_arc2(v2.pos()) {
                try_add_end_point_intr(v2.pos());
            }

            let pt1_in_sweep = both_arcs_sweep_point(point1);
            let pt2_in_sweep = both_arcs_sweep_point(point2);
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
            // determine if arcs overlap along their sweep
            let same_direction_arcs = v1.bulge_is_neg() == u1.bulge_is_neg();
            let (arc1_start, arc1_sweep) = start_and_sweep_angle(v1.pos(), arc1_center, v1.bulge);
            let (arc2_start, arc2_sweep) =
                // we have the arc sweeps go the same direction to simplify checks
                if same_direction_arcs {
                    start_and_sweep_angle(u1.pos(), arc2_center, u1.bulge)
                } else {
                    start_and_sweep_angle(u2.pos(), arc2_center, -u1.bulge)
                };

            let arc1_end = arc1_start + arc1_sweep;
            let arc2_end = arc2_start + arc2_sweep;
            // using average radius for fuzzy compare (arc radii are fuzzy equal, this is to produce
            // best fuzzy overlap approximation)
            let avg_radius = (arc1_radius + arc2_radius) / T::two();

            // check if only end points touch (because we made arc sweeps go same direction we
            // only have to test the delta angle between the start and end)

            // note: for fuzzy compare using arc length (radius * angle) rather than just the sweep
            // angle so that the epsilon value is used in the context of the arc size/scale
            match (
                (avg_radius * delta_angle(arc1_start, arc2_end)).fuzzy_eq_zero_eps(pos_equal_eps),
                (avg_radius * delta_angle(arc2_start, arc1_end)).fuzzy_eq_zero_eps(pos_equal_eps),
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
                    OneIntersect { point: u1.pos() }
                }
                (false, false) => {
                    // not just the end points touch, determine how the arcs overlap
                    let arc2_starts_in_arc1 =
                        angle_is_within_sweep(arc2_start, arc1_start, arc1_sweep);
                    let arc2_ends_in_arc1 = angle_is_within_sweep(arc2_end, arc1_start, arc1_sweep);
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
                        let arc1_starts_in_arc2 =
                            angle_is_within_sweep(arc1_start, arc2_start, arc2_sweep);
                        if arc1_starts_in_arc2 {
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
