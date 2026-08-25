use super::PlineVertex;
use super::pline_seg::seg_arc_radius_and_center;
use crate::core::{
    math::Vector2,
    math::{
        ArcDomain, CircleCircleIntr, IntrDomainPosition, LineCircleIntr, LineLineIntr,
        angle_from_bulge, circle_circle_intr, dist_squared, line_circle_intr, line_line_intr,
        point_from_parametric,
    },
    traits::Real,
};

type DomainPosition<T> = IntrDomainPosition<T>;

#[derive(Copy, Clone)]
struct CandidateDomains<T>
where
    T: Real,
{
    analytic_point: Vector2<T>,
    error_squared: T,
    first: DomainPosition<T>,
    second: DomainPosition<T>,
}

#[derive(Copy, Clone)]
struct AcceptedCandidate<T>
where
    T: Real,
{
    hit: PlineSegIntrPoint<T>,
    second_path_distance: T,
}

/// One finite polyline segment intersection point.
#[derive(Debug, Copy, Clone)]
pub struct PlineSegIntrPoint<T>
where
    T: Real,
{
    /// Returned finite-domain point.
    pub point: Vector2<T>,
    /// Sum of squared geometric adjustments used to produce `point`.
    ///
    /// Zero means `point` is an unchanged analytic solution.
    pub error_squared: T,
}

impl<T> PlineSegIntrPoint<T>
where
    T: Real,
{
    #[inline]
    fn adjusted(point: Vector2<T>, error_sq: T) -> Self {
        Self {
            point,
            error_squared: error_sq,
        }
    }
}

#[inline]
fn suppress_duplicate_endpoint_claims<T>(candidates: &mut [CandidateDomains<T>; 2])
where
    T: Real,
{
    let suppress_for_side = |candidates: &mut [CandidateDomains<T>; 2], first_side: bool| {
        let positions = if first_side {
            [candidates[0].first, candidates[1].first]
        } else {
            [candidates[0].second, candidates[1].second]
        };
        let [Some(point0), Some(point1)] =
            [positions[0].snapped_point(), positions[1].snapped_point()]
        else {
            return;
        };
        if point0 != point1 {
            return;
        }

        let reject_index = usize::from(
            dist_squared(candidates[0].analytic_point, point0)
                <= dist_squared(candidates[1].analytic_point, point1),
        );
        if first_side {
            candidates[reject_index].first = DomainPosition::Outside;
        } else {
            candidates[reject_index].second = DomainPosition::Outside;
        }
    };

    suppress_for_side(candidates, true);
    suppress_for_side(candidates, false);
}

#[inline]
fn accept_candidate<T>(
    candidate: CandidateDomains<T>,
    epsilon_sq: T,
) -> Option<AcceptedCandidate<T>>
where
    T: Real,
{
    let second_path_distance = candidate.second.path_distance()?;
    candidate.first.path_distance()?;
    let first_snap = candidate.first.snapped_point();
    let second_snap = candidate.second.snapped_point();
    let hit = match (first_snap, second_snap) {
        (None, None) => {
            PlineSegIntrPoint::adjusted(candidate.analytic_point, candidate.error_squared)
        }
        (Some(point), None) | (None, Some(point)) => {
            let snap_error_sq = dist_squared(point, candidate.analytic_point);
            if snap_error_sq >= epsilon_sq {
                return None;
            }
            PlineSegIntrPoint::adjusted(point, candidate.error_squared + snap_error_sq)
        }
        (Some(point1), Some(point2)) => {
            let distance1_sq = dist_squared(point1, candidate.analytic_point);
            let distance2_sq = dist_squared(point2, candidate.analytic_point);
            if distance1_sq >= epsilon_sq
                || distance2_sq >= epsilon_sq
                || dist_squared(point1, point2) >= epsilon_sq
            {
                return None;
            }
            let endpoint_error_sq = dist_squared(point1, point2);
            let (point, snap_error_sq) = if distance1_sq < distance2_sq {
                (point1, distance1_sq + endpoint_error_sq)
            } else if distance2_sq < distance1_sq {
                (point2, distance2_sq + endpoint_error_sq)
            } else if point1.x > point2.x || (point1.x == point2.x && point1.y > point2.y) {
                (point1, distance1_sq + endpoint_error_sq)
            } else {
                (point2, distance2_sq + endpoint_error_sq)
            };
            PlineSegIntrPoint::adjusted(point, candidate.error_squared + snap_error_sq)
        }
    };

    Some(AcceptedCandidate {
        hit,
        second_path_distance,
    })
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
        hit: PlineSegIntrPoint<T>,
    },
    /// One non-tangent intersect point found.
    OneIntersect {
        /// Holds the intersect point.
        hit: PlineSegIntrPoint<T>,
    },
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// Holds the first intersect point (according to the second segment direction).
        hit1: PlineSegIntrPoint<T>,
        /// Holds the second intersect point (according to the second segment direction).
        hit2: PlineSegIntrPoint<T>,
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
/// Segments are defined by `v1->v2` and `u1->u2`. Supporting-curve solutions inside both finite
/// domains are returned unchanged. A solution just outside a line is clamped by line-path
/// overshoot, and a solution just outside an arc is clamped by arc-length overshoot, only when the
/// overshoot and solution-to-endpoint distance are both strictly less than `pos_equal_eps`.
/// Endpoint proximity to the other supporting curve does not create an intersection.
/// Position-equal boundaries of a line or arc overlap coalesce to one endpoint event.
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
            LineLineIntr::NoIntersect => return NoIntersect,
            LineLineIntr::OneIntersect { hit } => {
                return OneIntersect {
                    hit: PlineSegIntrPoint::adjusted(hit.point, hit.error_squared),
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
                                 a2: PlineVertex<T>,
                                 line_is_first: bool|
     -> PlineSegIntr<T> {
        let (arc_radius, arc_center) = seg_arc_radius_and_center(a1, a2);
        let intr_result = line_circle_intr(p0, p1, arc_radius, arc_center, pos_equal_eps);
        let epsilon_sq = pos_equal_eps * pos_equal_eps;
        let arc_domain = ArcDomain::new(
            arc_center,
            arc_radius,
            a1.pos(),
            a2.pos(),
            angle_from_bulge(a1.bulge).abs(),
            a1.bulge_is_neg(),
        );
        let line_length = (p1 - p0).length();
        let candidate = |hit: crate::core::math::LineCircleIntrPoint<T>| {
            let analytic_point = hit.point;
            let line_domain = DomainPosition::Inside {
                path_distance: hit.line_t * line_length,
            };
            let arc_domain = arc_domain.classify(analytic_point, epsilon_sq);
            if line_is_first {
                CandidateDomains {
                    analytic_point,
                    error_squared: hit.error_squared,
                    first: line_domain,
                    second: arc_domain,
                }
            } else {
                CandidateDomains {
                    analytic_point,
                    error_squared: hit.error_squared,
                    first: arc_domain,
                    second: line_domain,
                }
            }
        };

        match intr_result {
            LineCircleIntr::NoIntersect => NoIntersect,
            LineCircleIntr::TangentIntersect { hit } => {
                accept_candidate(candidate(hit), epsilon_sq).map_or(NoIntersect, |accepted| {
                    TangentIntersect { hit: accepted.hit }
                })
            }
            LineCircleIntr::OneIntersect { hit } => accept_candidate(candidate(hit), epsilon_sq)
                .map_or(NoIntersect, |accepted| OneIntersect { hit: accepted.hit }),
            LineCircleIntr::TwoIntersects { hit1, hit2 } => {
                let mut candidates = [candidate(hit1), candidate(hit2)];
                suppress_duplicate_endpoint_claims(&mut candidates);
                let accepted = [
                    accept_candidate(candidates[0], epsilon_sq),
                    accept_candidate(candidates[1], epsilon_sq),
                ];
                match accepted {
                    [None, None] => NoIntersect,
                    [Some(accepted), None] | [None, Some(accepted)] => {
                        OneIntersect { hit: accepted.hit }
                    }
                    [Some(mut accepted1), Some(mut accepted2)] => {
                        if accepted2.second_path_distance < accepted1.second_path_distance {
                            std::mem::swap(&mut accepted1, &mut accepted2);
                        }
                        TwoIntersects {
                            hit1: accepted1.hit,
                            hit2: accepted2.hit,
                        }
                    }
                }
            }
        }
    };

    if v_is_line {
        // v is line, u is arc
        return process_line_arc_intr(v1.pos(), v2.pos(), u1, u2, true);
    }

    if u_is_line {
        // u is line, v is arc
        return process_line_arc_intr(u1.pos(), u2.pos(), v1, v2, false);
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

    let epsilon_sq = pos_equal_eps * pos_equal_eps;
    let arc1_domain = ArcDomain::new(
        arc1_center,
        arc1_radius,
        v1.pos(),
        v2.pos(),
        angle_from_bulge(v1.bulge).abs(),
        v1.bulge_is_neg(),
    );
    let arc2_domain = ArcDomain::new(
        arc2_center,
        arc2_radius,
        u1.pos(),
        u2.pos(),
        angle_from_bulge(u1.bulge).abs(),
        u1.bulge_is_neg(),
    );
    let candidate = |hit: crate::core::math::IntrPoint<T>| CandidateDomains {
        analytic_point: hit.point,
        error_squared: hit.error_squared,
        first: arc1_domain.classify(hit.point, epsilon_sq),
        second: arc2_domain.classify(hit.point, epsilon_sq),
    };

    match intr_result {
        CircleCircleIntr::NoIntersect => NoIntersect,
        CircleCircleIntr::TangentIntersect { hit } => accept_candidate(candidate(hit), epsilon_sq)
            .map_or(NoIntersect, |accepted| TangentIntersect {
                hit: accepted.hit,
            }),
        CircleCircleIntr::TwoIntersects { hit1, hit2 } => {
            let mut candidates = [candidate(hit1), candidate(hit2)];
            suppress_duplicate_endpoint_claims(&mut candidates);
            let accepted = [
                accept_candidate(candidates[0], epsilon_sq),
                accept_candidate(candidates[1], epsilon_sq),
            ];
            match accepted {
                [None, None] => NoIntersect,
                [Some(accepted), None] | [None, Some(accepted)] => {
                    OneIntersect { hit: accepted.hit }
                }
                [Some(mut accepted1), Some(mut accepted2)] => {
                    if accepted2.second_path_distance < accepted1.second_path_distance {
                        std::mem::swap(&mut accepted1, &mut accepted2);
                    }
                    TwoIntersects {
                        hit1: accepted1.hit,
                        hit2: accepted2.hit,
                    }
                }
            }
        }
        CircleCircleIntr::Overlapping => {
            // Express both arc sets in the second arc's direction and intersect exact angular
            // intervals before considering tolerant endpoint-only contact. This preserves positive
            // overlaps even when their length is below the position tolerance.
            let same_direction_arcs = v1.bulge_is_neg() == u1.bulge_is_neg();
            let arc1_oriented_start = if same_direction_arcs {
                v1.pos()
            } else {
                v2.pos()
            };
            let arc1_oriented_end = if same_direction_arcs {
                v2.pos()
            } else {
                v1.pos()
            };
            let arc1_interval_start = arc2_domain.forward_angle(arc1_oriented_start);
            let mut overlap = None;
            for shift in [-T::tau(), T::zero(), T::tau()] {
                let shifted_start = arc1_interval_start + shift;
                let shifted_end = shifted_start + arc1_domain.sweep();
                let low = if shifted_start > T::zero() {
                    shifted_start
                } else {
                    T::zero()
                };
                let high = if shifted_end < arc2_domain.sweep() {
                    shifted_end
                } else {
                    arc2_domain.sweep()
                };
                if low < high {
                    let point1 = if low == T::zero() {
                        u1.pos()
                    } else {
                        arc1_oriented_start
                    };
                    let point2 = if high == arc2_domain.sweep() {
                        u2.pos()
                    } else {
                        arc1_oriented_end
                    };
                    overlap = Some((point1, point2));
                    break;
                }
            }

            if let Some((point1, point2)) = overlap {
                let point1 = arc2_domain.project_to_circle(point1);
                let point2 = arc2_domain.project_to_circle(point2);
                let candidate1 = candidate(crate::core::math::IntrPoint {
                    point: point1,
                    error_squared: T::zero(),
                });
                let candidate2 = candidate(crate::core::math::IntrPoint {
                    point: point2,
                    error_squared: T::zero(),
                });
                if candidate1.first.path_distance().is_some()
                    && candidate1.second.path_distance().is_some()
                    && candidate2.first.path_distance().is_some()
                    && candidate2.second.path_distance().is_some()
                {
                    // Position-equal overlap boundaries represent one endpoint event.
                    let boundary_distance_sq = dist_squared(point1, point2);
                    if boundary_distance_sq < epsilon_sq {
                        return OneIntersect {
                            hit: PlineSegIntrPoint::adjusted(point2, boundary_distance_sq),
                        };
                    }
                    return OverlappingArcs { point1, point2 };
                }
            }

            // With no positive interval overlap, only position-equal endpoints can create contact.
            let first_endpoints = [v1.pos(), v2.pos()];
            let endpoint_hit = |point: Vector2<T>| {
                let distance_sq = first_endpoints
                    .iter()
                    .map(|endpoint| dist_squared(*endpoint, point))
                    .min_by(T::total_cmp)
                    .unwrap();
                PlineSegIntrPoint::adjusted(point, distance_sq)
            };
            let start_distance_sq = first_endpoints
                .iter()
                .map(|point| dist_squared(*point, u1.pos()))
                .min_by(T::total_cmp)
                .unwrap();
            let end_distance_sq = first_endpoints
                .iter()
                .map(|point| dist_squared(*point, u2.pos()))
                .min_by(T::total_cmp)
                .unwrap();
            let start_touches = start_distance_sq < epsilon_sq;
            let end_touches = end_distance_sq < epsilon_sq;
            match (start_touches, end_touches) {
                (true, true) if u1.pos().fuzzy_eq_eps(u2.pos(), pos_equal_eps) => OneIntersect {
                    hit: endpoint_hit(u2.pos()),
                },
                (true, true) => TwoIntersects {
                    hit1: endpoint_hit(u1.pos()),
                    hit2: endpoint_hit(u2.pos()),
                },
                (true, false) => OneIntersect {
                    hit: endpoint_hit(u1.pos()),
                },
                (false, true) => OneIntersect {
                    hit: endpoint_hit(u2.pos()),
                },
                (false, false) => NoIntersect,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::bulge_from_angle;
    use std::f64::consts::FRAC_PI_2;

    #[derive(Debug, Copy, Clone)]
    enum ExpectedPlineSegIntr<T>
    where
        T: Real,
    {
        NoIntersect,
        TangentIntersect {
            point: Vector2<T>,
        },
        OneIntersect {
            point: Vector2<T>,
        },
        TwoIntersects {
            point1: Vector2<T>,
            point2: Vector2<T>,
        },
        OverlappingLines {
            point1: Vector2<T>,
            point2: Vector2<T>,
        },
        OverlappingArcs {
            point1: Vector2<T>,
            point2: Vector2<T>,
        },
    }
    use ExpectedPlineSegIntr::*;

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (PlineSegIntr::NoIntersect, NoIntersect) => {}
                (PlineSegIntr::TangentIntersect { hit }, TangentIntersect { point })
                | (PlineSegIntr::OneIntersect { hit }, OneIntersect { point })
                    if hit.point.fuzzy_eq(point) => {}
                (
                    PlineSegIntr::TwoIntersects { hit1: a1, hit2: b1 },
                    TwoIntersects {
                        point1: a2,
                        point2: b2,
                    },
                ) if a1.point.fuzzy_eq(a2) && b1.point.fuzzy_eq(b2) => {}
                (
                    PlineSegIntr::OverlappingLines {
                        point1: a1,
                        point2: b1,
                    },
                    OverlappingLines {
                        point1: a2,
                        point2: b2,
                    },
                )
                | (
                    PlineSegIntr::OverlappingArcs {
                        point1: a1,
                        point2: b1,
                    },
                    OverlappingArcs {
                        point1: a2,
                        point2: b2,
                    },
                ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            };
        };
    }

    fn arc_segment(
        center: Vector2<f64>,
        radius: f64,
        start_angle: f64,
        sweep: f64,
    ) -> (PlineVertex<f64>, PlineVertex<f64>) {
        let point_at_angle = |angle: f64| {
            let (sin, cos) = angle.sin_cos();
            center + Vector2::new(radius * cos, radius * sin)
        };
        (
            PlineVertex::from_vector2(point_at_angle(start_angle), (sweep / 4.0).tan()),
            PlineVertex::from_vector2(point_at_angle(start_angle + sweep), 0.0),
        )
    }

    #[test]
    fn line_arc_candidate_just_outside_line_clamps_to_endpoint() {
        let epsilon = 1e-5;
        let line_start = PlineVertex::new(0.0, -1.0 - 0.5 * epsilon, 0.0);
        let line_end = PlineVertex::new(0.0, -2.0, 0.0);
        let (arc_start, arc_end) = arc_segment(
            Vector2::new(0.0, 0.0),
            1.0,
            std::f64::consts::PI,
            std::f64::consts::PI,
        );

        assert_case_eq!(
            pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
            OneIntersect {
                point: line_start.pos()
            }
        );

        let result = pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon);
        let PlineSegIntr::OneIntersect { hit } = result else {
            panic!("expected one intersection: {result:?}");
        };
        let expected_distance_sq = (0.5 * epsilon) * (0.5 * epsilon);
        assert!((hit.error_squared - expected_distance_sq).abs() < expected_distance_sq * 1e-9);
    }

    #[test]
    fn line_line_domain_boundaries_are_strict_and_interior_points_are_unchanged() {
        let epsilon = 1.0f64 / 1024.0;
        let line_start = PlineVertex::new(0.0, 0.0, 0.0);
        let line_end = PlineVertex::new(2.0, 0.0, 0.0);

        for at_end in [false, true] {
            let endpoint_x = if at_end { 2.0 } else { 0.0 };
            for factor in [0.25, 0.99, 1.0, 1.01] {
                let distance = factor * epsilon;
                let outside_x = if at_end {
                    endpoint_x + distance
                } else {
                    endpoint_x - distance
                };
                let crossing_start = PlineVertex::new(outside_x, -1.0, 0.0);
                let crossing_end = PlineVertex::new(outside_x, 1.0, 0.0);
                let expected = if factor < 1.0 {
                    OneIntersect {
                        point: Vector2::new(endpoint_x, 0.0),
                    }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(line_start, line_end, crossing_start, crossing_end, epsilon,),
                    expected
                );
                assert_case_eq!(
                    pline_seg_intr(crossing_start, crossing_end, line_start, line_end, epsilon,),
                    expected
                );
            }

            let inside_x = if at_end {
                endpoint_x - 0.25 * epsilon
            } else {
                endpoint_x + 0.25 * epsilon
            };
            let crossing_start = PlineVertex::new(inside_x, -1.0, 0.0);
            let crossing_end = PlineVertex::new(inside_x, 1.0, 0.0);
            let expected = OneIntersect {
                point: Vector2::new(inside_x, 0.0),
            };
            assert_case_eq!(
                pline_seg_intr(line_start, line_end, crossing_start, crossing_end, epsilon,),
                expected
            );
        }

        assert_case_eq!(
            pline_seg_intr(
                line_start,
                line_end,
                PlineVertex::new(0.0, -1.0, 0.0),
                PlineVertex::new(0.0, 1.0, 0.0),
                epsilon,
            ),
            OneIntersect {
                point: line_start.pos()
            }
        );
    }

    #[test]
    fn collinear_line_contacts_use_strict_gaps_and_coalesce_position_equal_overlaps() {
        let epsilon = 1.0f64 / 1024.0;
        let line1_start = PlineVertex::new(0.0, 0.0, 0.0);
        let line1_end = PlineVertex::new(1.0, 0.0, 0.0);

        for (factor, intersects) in [(0.25, true), (0.99, true), (1.0, false), (1.01, false)] {
            let gap = factor * epsilon;
            let line2_start = PlineVertex::new(1.0 + gap, 0.0, 0.0);
            let line2_end = PlineVertex::new(2.0 + gap, 0.0, 0.0);
            let expected = if intersects {
                OneIntersect {
                    point: line1_end.pos(),
                }
            } else {
                NoIntersect
            };
            assert_case_eq!(
                pline_seg_intr(line1_start, line1_end, line2_start, line2_end, epsilon,),
                expected
            );
            assert_case_eq!(
                pline_seg_intr(line2_start, line2_end, line1_start, line1_end, epsilon,),
                if intersects {
                    OneIntersect {
                        point: line2_start.pos(),
                    }
                } else {
                    NoIntersect
                }
            );
        }

        let overlap_start = Vector2::new(1.0 - 0.25 * epsilon, 0.0);
        assert_case_eq!(
            pline_seg_intr(
                line1_start,
                line1_end,
                PlineVertex::from_vector2(overlap_start, 0.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                epsilon,
            ),
            OneIntersect {
                point: overlap_start
            }
        );
    }

    #[test]
    fn line_arc_line_domain_boundaries_are_strict_and_symmetric() {
        let epsilon = 1.0f64 / 1024.0;
        let (arc_start, arc_end) = arc_segment(
            Vector2::zero(),
            1.0,
            std::f64::consts::PI,
            std::f64::consts::PI,
        );

        for at_end in [false, true] {
            for factor in [0.25, 0.99, 1.0, 1.01] {
                let distance = factor * epsilon;
                let (line_start, line_end, endpoint) = if at_end {
                    (
                        PlineVertex::new(0.0, -2.0, 0.0),
                        PlineVertex::new(0.0, -1.0 - distance, 0.0),
                        Vector2::new(0.0, -1.0 - distance),
                    )
                } else {
                    (
                        PlineVertex::new(0.0, -1.0 - distance, 0.0),
                        PlineVertex::new(0.0, -2.0, 0.0),
                        Vector2::new(0.0, -1.0 - distance),
                    )
                };
                let expected = if factor < 1.0 {
                    OneIntersect { point: endpoint }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
                    expected
                );
                assert_case_eq!(
                    pline_seg_intr(arc_start, arc_end, line_start, line_end, epsilon),
                    expected
                );
            }
        }

        for (line_start, line_end) in [
            (
                PlineVertex::new(0.0, -1.0 - 0.25 * epsilon, 0.0),
                PlineVertex::new(0.0, 0.0, 0.0),
            ),
            (
                PlineVertex::new(0.0, 0.0, 0.0),
                PlineVertex::new(0.0, -1.0 - 0.25 * epsilon, 0.0),
            ),
        ] {
            let expected = OneIntersect {
                point: Vector2::new(0.0, -1.0),
            };
            assert_case_eq!(
                pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
                expected
            );
        }
    }

    #[test]
    fn line_arc_exact_endpoints_are_inside_without_snapping() {
        let (lower_arc_start, lower_arc_end) = arc_segment(
            Vector2::zero(),
            1.0,
            std::f64::consts::PI,
            std::f64::consts::PI,
        );
        let line_start = PlineVertex::new(0.0, -1.0, 0.0);
        let line_end = PlineVertex::new(0.0, -2.0, 0.0);
        assert_case_eq!(
            pline_seg_intr(line_start, line_end, lower_arc_start, lower_arc_end, 1e-5,),
            OneIntersect {
                point: line_start.pos()
            }
        );

        let radial_line_start = PlineVertex::new(0.5, 0.0, 0.0);
        let radial_line_end = PlineVertex::new(1.5, 0.0, 0.0);
        let (quarter_arc_start, quarter_arc_end) =
            arc_segment(Vector2::zero(), 1.0, 0.0, FRAC_PI_2);
        assert_case_eq!(
            pline_seg_intr(
                radial_line_start,
                radial_line_end,
                quarter_arc_start,
                quarter_arc_end,
                1e-5,
            ),
            OneIntersect {
                point: quarter_arc_start.pos()
            }
        );
    }

    #[test]
    fn line_arc_recovered_tangent_uses_strict_boundary_across_scales() {
        for radius in [1.0 / 64.0, 1.0, 64.0] {
            let epsilon = radius / 1024.0;
            for (factor, intersects) in [(0.99, true), (1.0, false), (1.01, false)] {
                let gap = factor * epsilon;
                let center = Vector2::new(0.0, radius + gap);
                let line_start = PlineVertex::new(-2.0 * radius, 0.0, 0.0);
                let line_end = PlineVertex::new(2.0 * radius, 0.0, 0.0);
                let (arc_start, arc_end) =
                    arc_segment(center, radius, std::f64::consts::PI, std::f64::consts::PI);
                let expected = if intersects {
                    TangentIntersect {
                        point: Vector2::zero(),
                    }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
                    expected
                );
                assert_case_eq!(
                    pline_seg_intr(arc_start, arc_end, line_start, line_end, epsilon),
                    expected
                );
            }
        }
    }

    #[test]
    fn line_arc_arc_domain_boundaries_are_strict_and_preserve_interior_candidates() {
        let epsilon = 1.0f64 / 1024.0;
        let line_start = PlineVertex::new(0.5, 0.0, 0.0);
        let line_end = PlineVertex::new(1.5, 0.0, 0.0);
        let analytic_point = Vector2::new(1.0, 0.0);

        for factor in [0.25, 0.99, 1.0, 1.01] {
            let delta = factor * epsilon;
            for (start_angle, expected_endpoint) in [
                (delta, Vector2::new(delta.cos(), delta.sin())),
                (
                    -std::f64::consts::FRAC_PI_2 - delta,
                    Vector2::new((-delta).cos(), (-delta).sin()),
                ),
            ] {
                let (arc_start, arc_end) =
                    arc_segment(Vector2::zero(), 1.0, start_angle, FRAC_PI_2);
                let expected = if factor < 1.0 {
                    OneIntersect {
                        point: expected_endpoint,
                    }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
                    expected
                );
                assert_case_eq!(
                    pline_seg_intr(arc_start, arc_end, line_start, line_end, epsilon),
                    expected
                );
            }
        }

        for start_angle in [
            -0.25 * epsilon,
            0.25 * epsilon - std::f64::consts::FRAC_PI_2,
        ] {
            let (arc_start, arc_end) = arc_segment(Vector2::zero(), 1.0, start_angle, FRAC_PI_2);
            assert_case_eq!(
                pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
                OneIntersect {
                    point: analytic_point
                }
            );
        }
    }

    #[test]
    fn nonzero_sub_epsilon_line_keeps_an_interior_circle_root() {
        let epsilon = 1e-5;
        let line_start = PlineVertex::new(1.0 - 0.25 * epsilon, 0.0, 0.0);
        let line_end = PlineVertex::new(1.0 + 0.25 * epsilon, 0.0, 0.0);
        let (arc_start, arc_end) = arc_segment(Vector2::zero(), 1.0, 0.0, std::f64::consts::PI);

        assert_case_eq!(
            pline_seg_intr(line_start, line_end, arc_start, arc_end, epsilon),
            OneIntersect {
                point: Vector2::new(1.0, 0.0)
            }
        );
    }

    #[test]
    fn line_endpoint_near_arc_does_not_replace_distant_tangent_solution() {
        let line_start = PlineVertex::new(0.01, 1.0, 0.0);
        let line_end = PlineVertex::new(1.0, 1.0, 0.0);
        let (arc_start, arc_end) = arc_segment(Vector2::zero(), 1.0, 0.0, std::f64::consts::PI);

        assert_case_eq!(
            pline_seg_intr(line_start, line_end, arc_start, arc_end, 1e-4),
            NoIntersect::<f64>
        );
        assert_case_eq!(
            pline_seg_intr(arc_start, arc_end, line_start, line_end, 1e-4),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn line_arc_two_intersections_follow_second_segment_direction() {
        let y = -0.5;
        let x = (1.0f64 - y * y).sqrt();
        let left = Vector2::new(-x, y);
        let right = Vector2::new(x, y);
        let line_start = PlineVertex::new(-2.0, y, 0.0);
        let line_end = PlineVertex::new(2.0, y, 0.0);
        let (arc_start, arc_end) = arc_segment(
            Vector2::zero(),
            1.0,
            std::f64::consts::PI,
            std::f64::consts::PI,
        );

        assert_case_eq!(
            pline_seg_intr(line_start, line_end, arc_start, arc_end, 1e-5),
            TwoIntersects {
                point1: left,
                point2: right
            }
        );
        assert_case_eq!(
            pline_seg_intr(arc_start, arc_end, line_start, line_end, 1e-5),
            TwoIntersects {
                point1: left,
                point2: right
            }
        );

        let (reverse_arc_start, reverse_arc_end) =
            arc_segment(Vector2::zero(), 1.0, 0.0, -std::f64::consts::PI);
        assert_case_eq!(
            pline_seg_intr(
                line_start,
                line_end,
                reverse_arc_start,
                reverse_arc_end,
                1e-5,
            ),
            TwoIntersects {
                point1: right,
                point2: left
            }
        );
    }

    #[test]
    fn overlapping_arcs_handle_angle_wrap_at_different_scales() {
        for scale in [0.001, 1.0, 1e9] {
            let center = Vector2::new(2.5 * scale, -3.25 * scale);
            let arc1 = arc_segment(center, scale, 6.0, 0.8);
            let arc2 = arc_segment(center, scale, 0.1, 0.8);
            let (arc2_radius, arc2_center) = seg_arc_radius_and_center(arc2.0, arc2.1);
            let end_radial = arc1.1.pos() - arc2_center;
            let expected_end = arc2_center + end_radial.scale(arc2_radius / end_radial.length());
            assert_case_eq!(
                pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, 1e-5 * scale,),
                OverlappingArcs {
                    point1: arc2.0.pos(),
                    point2: expected_end
                }
            );
        }
    }

    #[test]
    fn overlapping_arc_endpoint_touch_uses_position_tolerance() {
        for scale in [1.0 / 64.0, 1.0, 64.0] {
            let epsilon = scale / 1024.0;
            let center = Vector2::new(0.0, 0.0);
            let arc1 = arc_segment(center, scale, 0.0, 0.5);

            for (factor, intersects) in [(0.25, true), (0.99, true), (1.01, false)] {
                let chord_distance = factor * epsilon;
                let angle_gap = 2.0 * (chord_distance / (2.0 * scale)).asin();
                let arc2 = arc_segment(center, scale, 0.5 + angle_gap, 0.5);
                if intersects {
                    assert_case_eq!(
                        pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
                        OneIntersect {
                            point: arc2.0.pos()
                        }
                    );
                    assert_case_eq!(
                        pline_seg_intr(arc2.0, arc2.1, arc1.0, arc1.1, epsilon),
                        OneIntersect {
                            point: arc1.1.pos()
                        }
                    );
                } else {
                    assert_case_eq!(
                        pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
                        NoIntersect::<f64>
                    );
                    assert_case_eq!(
                        pline_seg_intr(arc2.0, arc2.1, arc1.0, arc1.1, epsilon),
                        NoIntersect::<f64>
                    );
                }
            }
        }
    }

    #[test]
    fn coincident_arc_intervals_reject_large_gaps_and_coalesce_position_equal_overlaps() {
        let center = Vector2::zero();
        let separated1 = arc_segment(center, 1.0, 0.0, 0.5);
        let separated2 = arc_segment(center, 1.0, 0.500000005, 0.5);
        assert_case_eq!(
            pline_seg_intr(
                separated1.0,
                separated1.1,
                separated2.0,
                separated2.1,
                1e-12,
            ),
            NoIntersect::<f64>
        );

        let overlap1 = arc_segment(center, 1.0, 0.0, 4e-6);
        let overlap2 = arc_segment(center, 1.0, 1e-6, 4e-6);
        assert_case_eq!(
            pline_seg_intr(overlap1.0, overlap1.1, overlap2.0, overlap2.1, 1e-5,),
            OneIntersect {
                point: overlap1.1.pos()
            }
        );
    }

    #[test]
    fn coincident_arc_overlap_boundary_uses_the_shared_finite_domain() {
        let center = Vector2::zero();
        let arc1 = arc_segment(center, 1.0, 0.0, 1.0);
        let arc2 = arc_segment(center, 1.0, -5e-9, 0.500000005);
        assert_case_eq!(
            pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, 1e-5),
            OverlappingArcs {
                point1: arc1.0.pos(),
                point2: arc2.1.pos()
            }
        );
    }

    #[test]
    fn position_equal_coincident_arc_contacts_coalesce_under_argument_swap() {
        let center = Vector2::zero();
        let arc1 = arc_segment(center, 1.0, 0.0, 0.5);
        let arc2 = arc_segment(center, 1.0, 0.500001, 0.000002);
        assert_case_eq!(
            pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, 1e-5),
            OneIntersect {
                point: arc2.1.pos()
            }
        );
        assert_case_eq!(
            pline_seg_intr(arc2.0, arc2.1, arc1.0, arc1.1, 1e-5),
            OneIntersect {
                point: arc1.1.pos()
            }
        );
    }

    #[test]
    fn arc_arc_domain_boundaries_are_strict_and_preserve_interior_candidates() {
        let epsilon = 1.0f64 / 1024.0;
        let center1 = Vector2::zero();
        let center2 = Vector2::new(1.0, 0.0);
        let analytic_angle = 2.0 * std::f64::consts::PI / 3.0;
        let analytic_point = Vector2::new(0.5, 3.0f64.sqrt() / 2.0);
        let arc1 = arc_segment(center1, 1.0, 0.0, FRAC_PI_2);

        for factor in [0.25, 0.99, 1.0, 1.01] {
            let delta = factor * epsilon;
            for (start_angle, endpoint_angle) in [
                (analytic_angle + delta, analytic_angle + delta),
                (analytic_angle - FRAC_PI_2 - delta, analytic_angle - delta),
            ] {
                let arc2 = arc_segment(center2, 1.0, start_angle, FRAC_PI_2);
                let expected_endpoint =
                    center2 + Vector2::new(endpoint_angle.cos(), endpoint_angle.sin());
                let expected = if factor < 1.0 {
                    OneIntersect {
                        point: expected_endpoint,
                    }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
                    expected
                );
                assert_case_eq!(
                    pline_seg_intr(arc2.0, arc2.1, arc1.0, arc1.1, epsilon),
                    expected
                );
            }
        }

        for start_angle in [
            analytic_angle - 0.25 * epsilon,
            analytic_angle + 0.25 * epsilon - FRAC_PI_2,
        ] {
            let arc2 = arc_segment(center2, 1.0, start_angle, FRAC_PI_2);
            assert_case_eq!(
                pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
                OneIntersect {
                    point: analytic_point
                }
            );
        }
    }

    #[test]
    fn arc_arc_two_intersections_follow_second_segment_direction() {
        let center1 = Vector2::zero();
        let center2 = Vector2::new(1.0, 0.0);
        let lower = Vector2::new(0.5, -3.0f64.sqrt() / 2.0);
        let upper = Vector2::new(0.5, 3.0f64.sqrt() / 2.0);
        let arc1 = arc_segment(
            center1,
            1.0,
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
        );
        let arc2 = arc_segment(
            center2,
            1.0,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
        );

        assert_case_eq!(
            pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, 1e-5),
            TwoIntersects {
                point1: upper,
                point2: lower
            }
        );
        assert_case_eq!(
            pline_seg_intr(arc2.0, arc2.1, arc1.0, arc1.1, 1e-5),
            TwoIntersects {
                point1: lower,
                point2: upper
            }
        );

        let reverse_arc2 = arc_segment(
            center2,
            1.0,
            -std::f64::consts::FRAC_PI_2,
            -std::f64::consts::PI,
        );
        assert_case_eq!(
            pline_seg_intr(arc1.0, arc1.1, reverse_arc2.0, reverse_arc2.1, 1e-5),
            TwoIntersects {
                point1: lower,
                point2: upper
            }
        );
    }

    #[test]
    fn arc_arc_recovered_tangent_uses_strict_support_and_finite_domain_boundaries() {
        for radius in [1.0 / 64.0, 1.0, 64.0] {
            let epsilon = radius / 1024.0;
            for (factor, intersects) in [(0.25, true), (0.99, true), (1.0, false), (1.01, false)] {
                let gap = factor * epsilon;
                let center1 = Vector2::zero();
                let center2 = Vector2::new(2.0 * radius + gap, 0.0);
                let arc1 = arc_segment(
                    center1,
                    radius,
                    -std::f64::consts::FRAC_PI_2,
                    std::f64::consts::PI,
                );
                let arc2 = arc_segment(
                    center2,
                    radius,
                    std::f64::consts::FRAC_PI_2,
                    std::f64::consts::PI,
                );
                let expected = if intersects {
                    TangentIntersect {
                        point: Vector2::new(radius + gap / 2.0, 0.0),
                    }
                } else {
                    NoIntersect
                };
                assert_case_eq!(
                    pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
                    expected
                );
            }
        }
    }

    #[test]
    fn recovered_internal_arc_tangent_stays_within_both_supports() {
        let epsilon = 1e-5;
        let center1 = Vector2::zero();
        let center2 = Vector2::new(0.0000101, 0.0);
        let arc1 = arc_segment(
            center1,
            1.0,
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
        );
        let arc2 = arc_segment(
            center2,
            0.99998,
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
        );

        assert_case_eq!(
            pline_seg_intr(arc1.0, arc1.1, arc2.0, arc2.1, epsilon),
            TangentIntersect {
                point: Vector2::new(0.99999505, 0.0)
            }
        );
    }

    #[test]
    fn arc_line_no_intersect() {
        let v1 = PlineVertex::new(0.0, 0.0, 1.0);
        let v2 = PlineVertex::new(2.0, 0.0, 0.0);
        let u1 = PlineVertex::new(0.0, 1.0, 0.0);
        let u2 = PlineVertex::new(2.0, 3.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn line_arc_no_intersect() {
        let v1 = PlineVertex::new(0.0, 1.0, 0.0);
        let v2 = PlineVertex::new(2.0, 3.0, 0.0);
        let u1 = PlineVertex::new(0.0, 0.0, 1.0);
        let u2 = PlineVertex::new(2.0, 0.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn overlapping_lines() {
        let v1 = PlineVertex::new(3.0, 3.0, 0.0);
        let v2 = PlineVertex::new(1.0, 1.0, 0.0);
        let u1 = PlineVertex::new(1.0, 1.0, 0.0);
        let u2 = PlineVertex::new(2.0, 2.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingLines {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(2.0, 2.0)
            }
        );
    }

    #[test]
    fn overlapping_lines_reverse_dir() {
        let v1 = PlineVertex::new(1.0, 1.0, 0.0);
        let v2 = PlineVertex::new(3.0, 3.0, 0.0);
        let u1 = PlineVertex::new(2.0, 2.0, 0.0);
        let u2 = PlineVertex::new(1.0, 1.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingLines {
                point1: Vector2::new(2.0, 2.0),
                point2: Vector2::new(1.0, 1.0)
            }
        );
    }

    #[test]
    fn overlapping_same_arcs() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 3.0, 0.0);
        let u1 = PlineVertex::new(1.0, 1.0, 1.0);
        let u2 = PlineVertex::new(3.0, 3.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(3.0, 3.0)
            }
        );
    }

    #[test]
    fn overlapping_same_arcs_reverse_dir() {
        let v1 = PlineVertex::new(3.0, 3.0, -1.0);
        let v2 = PlineVertex::new(1.0, 1.0, 0.0);
        let u1 = PlineVertex::new(1.0, 1.0, 1.0);
        let u2 = PlineVertex::new(3.0, 3.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(3.0, 3.0)
            }
        );
    }

    #[test]
    fn arc_arc_end_points_touch() {
        let v1 = PlineVertex::new(3.0, 3.0, 1.0);
        let v2 = PlineVertex::new(1.0, 1.0, 0.0);
        let u1 = PlineVertex::new(1.0, 1.0, 1.0);
        let u2 = PlineVertex::new(3.0, 3.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            TwoIntersects {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(3.0, 3.0)
            }
        );
    }

    #[test]
    fn arc_arc_end_points_touch_reverse_dir() {
        let v1 = PlineVertex::new(1.0, 1.0, -1.0);
        let v2 = PlineVertex::new(3.0, 3.0, 0.0);
        let u1 = PlineVertex::new(1.0, 1.0, 1.0);
        let u2 = PlineVertex::new(3.0, 3.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            TwoIntersects {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(3.0, 3.0)
            }
        );

        // reverse parameter order should yield the same result
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            TwoIntersects {
                point1: Vector2::new(1.0, 1.0),
                point2: Vector2::new(3.0, 3.0)
            }
        );

        // changing direction of arc2 should yield the same result BUT point1/point2 ordered according to
        // second segment direction
        let u1 = PlineVertex::new(3.0, 3.0, -1.0);
        let u2 = PlineVertex::new(1.0, 1.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            TwoIntersects {
                point1: Vector2::new(3.0, 3.0),
                point2: Vector2::new(1.0, 1.0)
            }
        );
    }

    #[test]
    fn arc2_within_arc1_overlapping() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let bulge = bulge_from_angle(FRAC_PI_2);
        let u1 = PlineVertex::new(2.0, 0.0, bulge);
        let u2 = PlineVertex::new(3.0, 1.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc1_within_arc2_overlapping() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let bulge = bulge_from_angle(FRAC_PI_2);
        let u1 = PlineVertex::new(2.0, 0.0, bulge);
        let u2 = PlineVertex::new(3.0, 1.0, 0.0);
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc2_within_arc1_overlapping_reverse_dir() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let bulge = bulge_from_angle(FRAC_PI_2);
        let u1 = PlineVertex::new(3.0, 1.0, -bulge);
        let u2 = PlineVertex::new(2.0, 0.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(3.0, 1.0),
                point2: Vector2::new(2.0, 0.0)
            }
        );
    }

    #[test]
    fn arc1_within_arc2_overlapping_reverse_dir() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let bulge = bulge_from_angle(FRAC_PI_2);
        let u1 = PlineVertex::new(3.0, 1.0, -bulge);
        let u2 = PlineVertex::new(2.0, 0.0, 0.0);
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 0.0, 1.0);
        let u2 = PlineVertex::new(2.0, 2.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap_flipped() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 0.0, 1.0);
        let u2 = PlineVertex::new(2.0, 2.0, 0.0);
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap_arc2_reverse_dir() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 2.0, -1.0);
        let u2 = PlineVertex::new(2.0, 0.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(3.0, 1.0),
                point2: Vector2::new(2.0, 0.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap_arc2_reverse_dir_flipped() {
        let v1 = PlineVertex::new(1.0, 1.0, 1.0);
        let v2 = PlineVertex::new(3.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 2.0, -1.0);
        let u2 = PlineVertex::new(2.0, 0.0, 0.0);
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap_arc1_reverse_dir() {
        let v1 = PlineVertex::new(3.0, 1.0, -1.0);
        let v2 = PlineVertex::new(1.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 0.0, 1.0);
        let u2 = PlineVertex::new(2.0, 2.0, 0.0);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(2.0, 0.0),
                point2: Vector2::new(3.0, 1.0)
            }
        );
    }

    #[test]
    fn arc_arc_partial_overlap_arc1_reverse_dir_flipped() {
        let v1 = PlineVertex::new(3.0, 1.0, -1.0);
        let v2 = PlineVertex::new(1.0, 1.0, 0.0);

        let u1 = PlineVertex::new(2.0, 0.0, 1.0);
        let u2 = PlineVertex::new(2.0, 2.0, 0.0);
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OverlappingArcs {
                point1: Vector2::new(3.0, 1.0),
                point2: Vector2::new(2.0, 0.0)
            }
        );
    }

    #[test]
    fn arc_arc_opposite_direction_touch_at_ends_bug() {
        // This test case reproduces the bug where arcs have the same radius and center but opposite
        // directions and only touch at the end points.
        // The bug was that when same_direction_arcs = false, the code would return u1.pos()
        // as the intersection point, but after direction adjustment, u1.pos() is actually
        // the END of arc2, not the start. The actual intersection should be at u2.pos().
        //
        // Original issue that found it: https://github.com/jbuckmccready/cavalier_contours/issues/42

        // Arc1
        let v1 = PlineVertex::new(-189.0, -196.91384910249, 0.553407781718062);
        let v2 = PlineVertex::new(-170.999999999999, -225.631646989572, -0.553407781718061);

        // Arc2
        let u1 = PlineVertex::new(-153.0, -196.91384910249, -0.553407781718061);
        let u2 = PlineVertex::new(-171.0, -225.631646989571, -0.553407781718061);

        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);

        // The arcs should intersect at u2.pos() (where arc1 and arc2 ends),
        // NOT at u1.pos() (which is ~34 units away from the actual intersection)
        assert_case_eq!(
            result,
            OneIntersect {
                point: Vector2::new(-171.0, -225.631646989571) // u2.pos()
            }
        );

        // reverse parameter order should yield the same result
        let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            OneIntersect {
                point: Vector2::new(-171.0, -225.631646989571) // u2.pos()
            }
        );

        // changing direction of arc2 should yield the same result
        let u1 = PlineVertex::new(-171.0, -225.631646989571, 0.553407781718062);
        let u2 = PlineVertex::new(-153.0, -196.91384910249, -0.553407781718061);
        let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
        assert_case_eq!(
            result,
            OneIntersect {
                point: Vector2::new(-171.0, -225.631646989571)
            }
        );
    }

    #[test]
    fn arc_arc_endpoint_candidates_map_to_one_circle_solution() {
        let v1 = PlineVertex::new(7.52, 22.24658, 1.0);
        let v2 = PlineVertex::new(7.92, 22.24658, -0.41421356237309503);
        let u1 = PlineVertex::new(7.92, 22.24469, 0.26911088880453216);
        let u2 = PlineVertex::new(7.768872523567473, 22.50514531618044, -0.308562760357285);
        let expected = OneIntersect {
            point: Vector2::new(7.919998195716457, 22.245730464737726),
        };

        assert_case_eq!(pline_seg_intr(v1, v2, u1, u2, 1e-5), expected);
        assert_case_eq!(pline_seg_intr(u1, u2, v1, v2, 1e-5), expected);
    }

    #[test]
    fn arc_arc_endpoint_candidates_do_not_create_false_tangent() {
        let v1 = PlineVertex::new(7.92, 22.24658, -0.41421356237309503);
        let v2 = PlineVertex::new(8.62842, 22.955, 1.0);
        let u1 = PlineVertex::new(7.92, 22.24469, 0.26911088880453216);
        let u2 = PlineVertex::new(7.768872523567473, 22.50514531618044, -0.308562760357285);

        assert_case_eq!(pline_seg_intr(v1, v2, u1, u2, 1e-5), NoIntersect::<f64>);
        assert_case_eq!(pline_seg_intr(u1, u2, v1, v2, 1e-5), NoIntersect::<f64>);
    }

    #[test]
    fn arc_arc_distant_endpoint_candidates_keep_circle_solution() {
        let v1 = PlineVertex::new(8.93748920434549, 13.180270831399689, 0.19857667697861422);
        let v2 = PlineVertex::new(8.452046699604008, 14.343605930822443, 1.0);
        let u1 = PlineVertex::new(8.45225145592399, 14.343401587209927, -0.19891241846297356);
        let u2 = PlineVertex::new(7.350000000000007, 17.004130739679155, 0.0);
        let expected = OneIntersect {
            point: Vector2::new(8.452152567343411, 14.343500470478041),
        };

        assert_case_eq!(pline_seg_intr(v1, v2, u1, u2, 1e-5), expected);
        assert_case_eq!(pline_seg_intr(u1, u2, v1, v2, 1e-5), expected);
    }
}
