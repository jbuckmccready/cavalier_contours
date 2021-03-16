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
/// Segments are defined by `v1->v2` and `u1->u2`.
pub fn pline_seg_intr<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    u1: PlineVertex<T>,
    u2: PlineVertex<T>,
) -> PlineSegIntr<T>
where
    T: Real,
{
    use PlineSegIntr::*;
    let v_is_line = v1.bulge_is_zero();
    let u_is_line = u1.bulge_is_zero();

    if v_is_line && u_is_line {
        let intr_result = line_line_intr(v1.pos(), v2.pos(), u1.pos(), u2.pos());
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

        let point_in_sweep = |t: T| -> Option<Vector2<T>> {
            if !t.fuzzy_in_range(T::zero(), T::one()) {
                return None;
            }

            let p = point_from_parametric(p0, p1, t);
            let within_sweep =
                point_within_arc_sweep(arc_center, a1.pos(), a2.pos(), a1.bulge_is_neg(), p);
            if within_sweep {
                Some(p)
            } else {
                None
            }
        };

        let intr_result = line_circle_intr(p0, p1, arc_radius, arc_center);
        match intr_result {
            LineCircleIntr::NoIntersect => NoIntersect,
            LineCircleIntr::TangentIntersect { t0 } => {
                if let Some(point) = point_in_sweep(t0) {
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
                    (None, Some(point)) | (Some(point), None) => OneIntersect { point },
                    (Some(point1), Some(point2)) => {
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

    let both_arcs_sweep_point = |pt: Vector2<T>| -> bool {
        point_within_arc_sweep(arc1_center, v1.pos(), v2.pos(), v1.bulge_is_neg(), pt)
            && point_within_arc_sweep(arc2_center, u1.pos(), u2.pos(), u1.bulge_is_neg(), pt)
    };

    let intr_result = circle_circle_intr(arc1_radius, arc1_center, arc2_radius, arc2_center);

    match intr_result {
        CircleCircleIntr::NoIntersect => NoIntersect,
        CircleCircleIntr::TangentIntersect { point } => {
            if both_arcs_sweep_point(point) {
                TangentIntersect { point }
            } else {
                NoIntersect
            }
        }
        CircleCircleIntr::TwoIntersects { point1, point2 } => {
            let pt1_in_sweep = both_arcs_sweep_point(point1);
            let pt2_in_sweep = both_arcs_sweep_point(point2);
            if pt1_in_sweep && pt2_in_sweep {
                TwoIntersects { point1, point2 }
            } else if pt1_in_sweep {
                OneIntersect { point: point1 }
            } else if pt2_in_sweep {
                OneIntersect { point: point2 }
            } else {
                NoIntersect
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

            // check if only end points touch (because we made arc sweeps go same direction we
            // only have to test the delta angle between the start and end)
            match (
                delta_angle(arc1_start, arc2_end).fuzzy_eq_zero(),
                delta_angle(arc2_start, arc1_end).fuzzy_eq_zero(),
            ) {
                (true, true) => {
                    // two half circle arcs with end points touching
                    // note: point1 and point2 are returned in order according to second segment (u1->u2) direction
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
                        // note: point1 and point2 are returned in order according to second segment (u1->u2) direction
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
                        // note: point1 and point2 are returned in order according to second segment (u1->u2) direction
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
                            // note: point1 and point2 are returned in order according to second segment (u1->u2) direction
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
