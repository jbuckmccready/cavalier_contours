use super::{
    Vector2,
    base_math::{dist_squared, point_from_parametric},
};
use crate::core::traits::Real;
use num_traits::real::Real as NumReal;

/// One intersection between two finite line segments.
#[derive(Debug, Copy, Clone)]
pub struct LineLineIntrPoint<T>
where
    T: Real,
{
    /// Resolved intersection point.
    pub point: Vector2<T>,
    /// Sum of successive squared geometric adjustments used to produce `point`.
    pub error_squared: T,
    /// Resolved parameter on the first segment.
    pub seg1_t: T,
    /// Resolved parameter on the second segment.
    pub seg2_t: T,
}

/// Holds the result of finding the intersect between two line segments.
#[derive(Debug, Copy, Clone)]
pub enum LineLineIntr<T>
where
    T: Real,
{
    /// No finite-segment intersection.
    NoIntersect,
    /// One finite-segment intersection.
    OneIntersect {
        /// Resolved intersection.
        hit: LineLineIntrPoint<T>,
    },
    // TODO: Carry an error score for each overlap boundary. Supports accepted as collinear within
    // epsilon can produce overlap points that required a nonzero geometric adjustment.
    /// Segments overlap each other (are collinear) by some amount.
    Overlapping {
        /// Parametric value for start of coincidence along second segment.
        seg2_t0: T,
        /// Parametric value for end of coincidence along second segment.
        seg2_t1: T,
    },
}

/// Finds the intersects between two lines segments.
///
/// This function returns the parametric solution(s) using the general line segment equation
/// `P(t) = p0 + t * (p1 - p0)`. Where `t` then goes from 0 to 1. For `t < 0` or `t > 1` the result
/// is on the same line but not within the line segment.
///
/// Solutions just outside either finite segment are clamped to the nearest endpoint when their
/// path overshoot is strictly less than `epsilon`. More distant supporting-line solutions are
/// omitted. Returned parameters are therefore always in `0..=1`.
///
/// Tolerance-resolved points are argument ordered. When distinct fuzzy-equal endpoint candidates
/// are otherwise tied, the endpoint from the first segment is preferred. The returned point is
/// therefore not guaranteed to remain the same when the segment arguments are swapped.
///
/// `epsilon` must be greater than zero. It is a position tolerance for finite-domain,
/// collinearity, and endpoint checks. All position comparisons are strict, so a distance equal to
/// `epsilon` is not accepted. Parallelism uses the dimensionless default numeric epsilon on the
/// normalized direction cross product.
///
/// # Explanation on result cases `LineLineIntr`
/// ## `NoIntersect`
/// Either of the following cases:
/// * Lines are parallel within the default dimensionless angular tolerance
/// * The supporting lines intersect outside the finite domains by at least `epsilon`
/// * Both line segments are points and distinct from each other
/// * One line segment is a point and distinct from the other line segment
///
/// ## `OneIntersect`
/// Either of the following cases:
/// * Nonparallel supporting lines intersect with both parameters in `0..=1` or within the strict
///   path-distance tolerance of that range
/// * Both line segments are points and lie over each other (using `epsilon` for position compare)
/// * One line segment is a point and lies in other line segment (again using `epsilon`)
///
/// ## `Overlapping`
/// Either of the following cases:
/// * The lines are collinear and have a positive-length finite overlap
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::core::math::LineLineIntr::OneIntersect;
/// // A line segment tangent-intersecting a circle with one of the line segments end points.
/// let v1 = Vector2::new(0.0, 0.0);
/// let v2 = Vector2::new(1.0, 0.0);
/// let u1 = Vector2::new(0.5, -1.0);
/// let u2 = Vector2::new(0.5, 1.0);
/// if let LineLineIntr::OneIntersect { hit } = line_line_intr(v1, v2, u1, u2, 1e-5)
/// {
///     assert_eq!(hit.seg1_t, 0.5);
///     assert_eq!(hit.seg2_t, 0.5);
/// } else {
///     unreachable!("expected an intersection between line segments");
/// }
///```
/// Line segments are defined by `v1->v2` and `u1->u2`.
/// Handles the cases where the lines may be parallel, collinear, or single points.
#[must_use]
pub fn line_line_intr<T>(
    v1: Vector2<T>,
    v2: Vector2<T>,
    u1: Vector2<T>,
    u2: Vector2<T>,
    epsilon: T,
) -> LineLineIntr<T>
where
    T: Real,
{
    // This implementation works by processing the segments in parametric equation form and using
    // perpendicular products
    // http://geomalgorithms.com/a05-_intersect-1.html
    // http://mathworld.wolfram.com/PerpDotProduct.html

    use LineLineIntr::{NoIntersect, OneIntersect, Overlapping};

    let v = v2 - v1;
    let u = u2 - u1;
    let v_pdot_u = v.perp_dot(u);
    let w = v1 - u1;

    let eps = epsilon;
    let eps_sq = eps * eps;
    let v_length_sq = v.length_squared();
    let u_length_sq = u.length_squared();
    // Only a zero-length segment is a point. A nonzero segment shorter than epsilon still has a
    // supporting line and must retain its direction.
    let v_is_point = v_length_sq == T::zero();
    let u_is_point = u_length_sq == T::zero();
    let resolve_hit = |support_point: Vector2<T>, seg1_t: T, seg2_t: T, support_error_sq: T| {
        let resolve_side = |t: T, length_sq: T, start: Vector2<T>, end: Vector2<T>| {
            if t < T::zero() {
                if t * t * length_sq >= eps_sq {
                    return None;
                }
                Some((T::zero(), Some(start)))
            } else if t > T::one() {
                let overshoot = t - T::one();
                if overshoot * overshoot * length_sq >= eps_sq {
                    return None;
                }
                Some((T::one(), Some(end)))
            } else {
                Some((t, None))
            }
        };
        let (seg1_t, snap1) = resolve_side(seg1_t, v_length_sq, v1, v2)?;
        let (seg2_t, snap2) = resolve_side(seg2_t, u_length_sq, u1, u2)?;
        let (point, snap_error_sq) = match (snap1, snap2) {
            (None, None) => (support_point, T::zero()),
            (Some(point), None) | (None, Some(point)) => {
                (point, dist_squared(support_point, point))
            }
            (Some(point1), Some(point2)) => {
                let endpoint_error_sq = dist_squared(point1, point2);
                if endpoint_error_sq >= eps_sq {
                    return None;
                }
                let distance1_sq = dist_squared(support_point, point1);
                let distance2_sq = dist_squared(support_point, point2);
                if distance1_sq <= distance2_sq {
                    (point1, distance1_sq + endpoint_error_sq)
                } else {
                    (point2, distance2_sq + endpoint_error_sq)
                }
            }
        };
        Some(LineLineIntrPoint {
            point,
            error_squared: support_error_sq + snap_error_sq,
            seg1_t,
            seg2_t,
        })
    };

    if v_is_point && u_is_point {
        let error_squared = dist_squared(v1, u1);
        if error_squared < eps_sq {
            return OneIntersect {
                hit: LineLineIntrPoint {
                    point: v1,
                    error_squared,
                    seg1_t: T::zero(),
                    seg2_t: T::zero(),
                },
            };
        }
        return NoIntersect;
    }

    let v_pdot_w = v.perp_dot(w);
    let u_pdot_w = u.perp_dot(w);

    if v_is_point {
        if u_pdot_w * u_pdot_w >= eps_sq * u_length_sq {
            return NoIntersect;
        }
        let seg2_t = (v1 - u1).dot(u) / u_length_sq;
        let support_point = point_from_parametric(u1, u2, seg2_t);
        let support_error_sq = dist_squared(v1, support_point);
        return resolve_hit(support_point, T::zero(), seg2_t, support_error_sq)
            .map_or(NoIntersect, |hit| OneIntersect { hit });
    }

    if u_is_point {
        if v_pdot_w * v_pdot_w >= eps_sq * v_length_sq {
            return NoIntersect;
        }
        let seg1_t = (u1 - v1).dot(v) / v_length_sq;
        let support_point = point_from_parametric(v1, v2, seg1_t);
        let support_error_sq = dist_squared(u1, support_point);
        return resolve_hit(support_point, seg1_t, T::zero(), support_error_sq)
            .map_or(NoIntersect, |hit| OneIntersect { hit });
    }

    // Use a dimensionless angular threshold for parallelism. The caller's position epsilon is
    // applied separately to finite-domain and collinearity checks.
    let angular_epsilon = T::fuzzy_epsilon();
    if v_pdot_u * v_pdot_u >= angular_epsilon * angular_epsilon * v_length_sq * u_length_sq {
        // segments not parallel or collinear
        let seg1_t = u_pdot_w / v_pdot_u;
        let seg2_t = v_pdot_w / v_pdot_u;
        let point = point_from_parametric(v1, v2, seg1_t);
        return resolve_hit(point, seg1_t, seg2_t, T::zero())
            .map_or(NoIntersect, |hit| OneIntersect { hit });
    }

    // segments are parallel and possibly collinear
    let v_pdot_u2 = v.perp_dot(u2 - v1);
    let u_pdot_v2 = u.perp_dot(v2 - u1);

    // Parallel supports are collinear only when each endpoint-to-support distance is within the
    // strict position tolerance.
    if v_pdot_w * v_pdot_w >= eps_sq * v_length_sq
        || u_pdot_w * u_pdot_w >= eps_sq * u_length_sq
        || v_pdot_u2 * v_pdot_u2 >= eps_sq * v_length_sq
        || u_pdot_v2 * u_pdot_v2 >= eps_sq * u_length_sq
    {
        for (v_point, u_point, seg1_t, seg2_t) in [
            (v1, u1, T::zero(), T::zero()),
            (v1, u2, T::zero(), T::one()),
            (v2, u1, T::one(), T::zero()),
            (v2, u2, T::one(), T::one()),
        ] {
            let error_squared = dist_squared(v_point, u_point);
            if error_squared < eps_sq {
                return OneIntersect {
                    hit: LineLineIntrPoint {
                        point: v_point,
                        error_squared,
                        seg1_t,
                        seg2_t,
                    },
                };
            }
        }
        // parallel and not collinear so no intersect
        return NoIntersect;
    }

    // Nondegenerate collinear segments: check if they overlap.
    let w2 = v2 - u1;
    let (mut seg2_t0, mut seg2_t1) = if u.x.abs() < u.y.abs() {
        (w.y / u.y, w2.y / u.y)
    } else {
        (w.x / u.x, w2.x / u.x)
    };

    if seg2_t0 > seg2_t1 {
        std::mem::swap(&mut seg2_t0, &mut seg2_t1);
    }

    let seg2_length = u_length_sq.sqrt();
    // using threshold check here to make intersect "sticky" to prefer considering it an intersect
    if !(seg2_t0 * seg2_length).fuzzy_lt_eps(seg2_length, eps)
        || !(seg2_t1 * seg2_length).fuzzy_gt_eps(T::zero(), eps)
    {
        return NoIntersect;
    }

    seg2_t0 = NumReal::max(T::zero(), NumReal::min(seg2_t0, T::one()));
    seg2_t1 = NumReal::max(T::zero(), NumReal::min(seg2_t1, T::one()));

    if ((seg2_t1 - seg2_t0) * seg2_length).fuzzy_eq_zero_eps(eps) {
        // intersect is a single point (segments line up end to end)
        let point = point_from_parametric(u1, u2, seg2_t0);
        let seg1_t = (point - v1).dot(v) / v_length_sq;
        let support_point = point_from_parametric(v1, v2, seg1_t);

        let support_error_sq = dist_squared(point, support_point);
        return resolve_hit(support_point, seg1_t, seg2_t0, support_error_sq)
            .map_or(NoIntersect, |hit| OneIntersect { hit });
    }

    Overlapping { seg2_t0, seg2_t1 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::traits::FuzzyEq;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};

    #[derive(Debug, Copy, Clone)]
    enum ExpectedLineLineIntr<T> {
        NoIntersect,
        TrueIntersect { seg1_t: T, seg2_t: T },
        Rejected,
        Overlapping { seg2_t0: T, seg2_t1: T },
    }
    use ExpectedLineLineIntr::*;

    fn clamp01<T: Real>(value: T) -> T {
        if value < T::zero() {
            T::zero()
        } else if value > T::one() {
            T::one()
        } else {
            value
        }
    }

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (LineLineIntr::NoIntersect, NoIntersect | Rejected) => {}
                (
                    LineLineIntr::OneIntersect { hit },
                    TrueIntersect {
                        seg1_t: a2,
                        seg2_t: b2,
                    },
                ) if hit.seg1_t.fuzzy_eq(clamp01(a2)) && hit.seg2_t.fuzzy_eq(clamp01(b2)) => {}
                (
                    LineLineIntr::Overlapping {
                        seg2_t0: a1,
                        seg2_t1: b1,
                    },
                    Overlapping {
                        seg2_t0: a2,
                        seg2_t1: b2,
                    },
                ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            }
        };
    }

    const TEST_ROTATION_ANGLES: &[f64] = &[FRAC_PI_8, FRAC_PI_6, FRAC_PI_4, FRAC_PI_3, FRAC_PI_2];

    #[test]
    fn true_intersect() {
        assert_case_eq!(
            line_line_intr(
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::new(-1.0, 1.0),
                Vector2::new(1.0, -1.0),
                1e-5,
            ),
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn small_scale_perpendicular_lines_keep_the_analytic_intersection() {
        assert_case_eq!(
            line_line_intr(
                Vector2::new(0.0, 0.0),
                Vector2::new(0.001, 0.0),
                Vector2::new(0.0005, -0.0005),
                Vector2::new(0.0005, 0.0005),
                1e-5,
            ),
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn nonzero_segment_shorter_than_epsilon_keeps_its_direction() {
        let epsilon = 1.0 / 1024.0;
        assert_case_eq!(
            line_line_intr(
                Vector2::zero(),
                Vector2::new(2.0, 0.0),
                Vector2::new(1.0, -epsilon / 4.0),
                Vector2::new(1.0, epsilon / 4.0),
                epsilon,
            ),
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn point_segments_use_strict_position_equality() {
        let epsilon = 1.0 / 1024.0;
        let point = Vector2::zero();
        assert_case_eq!(
            line_line_intr(
                point,
                point,
                Vector2::new(epsilon / 2.0, 0.0),
                Vector2::new(epsilon / 2.0, 0.0),
                epsilon,
            ),
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );
        assert_case_eq!(
            line_line_intr(
                point,
                point,
                Vector2::new(epsilon, 0.0),
                Vector2::new(epsilon, 0.0),
                epsilon,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn point_segment_checks_normal_distance_and_parameter_range() {
        let epsilon = 1.0 / 1024.0;
        let line_start = Vector2::zero();
        let line_end = Vector2::new(2.0, 0.0);
        for (point, expected) in [
            (Vector2::new(1.0, epsilon / 2.0), Some(0.5)),
            (Vector2::new(1.0, epsilon), None),
            (Vector2::new(-epsilon / 2.0, 0.0), Some(-epsilon / 4.0)),
            (Vector2::new(-epsilon, 0.0), None),
        ] {
            if let Some(line_t) = expected {
                assert_case_eq!(
                    line_line_intr(line_start, line_end, point, point, epsilon),
                    TrueIntersect {
                        seg1_t: line_t,
                        seg2_t: 0.0
                    }
                );
                assert_case_eq!(
                    line_line_intr(point, point, line_start, line_end, epsilon),
                    TrueIntersect {
                        seg1_t: 0.0,
                        seg2_t: line_t
                    }
                );
            } else {
                assert_case_eq!(
                    line_line_intr(line_start, line_end, point, point, epsilon),
                    NoIntersect::<f64>
                );
                assert_case_eq!(
                    line_line_intr(point, point, line_start, line_end, epsilon),
                    NoIntersect::<f64>
                );
            }
        }
    }

    #[test]
    fn point_segment_uses_the_projected_support_point() {
        let epsilon = 1e-3;
        let offset = 1e-4;
        let line_start = Vector2::zero();
        let line_end = Vector2::new(2.0, 2.0);
        let point = Vector2::new(1.0, 1.0 + offset);
        let expected_t = 0.5 + offset / 4.0;
        let expected_point = Vector2::new(1.0 + offset / 2.0, 1.0 + offset / 2.0);
        let expected_error_sq = offset * offset / 2.0;

        for result in [
            line_line_intr(line_start, line_end, point, point, epsilon),
            line_line_intr(point, point, line_start, line_end, epsilon),
        ] {
            let LineLineIntr::OneIntersect { hit } = result else {
                panic!("expected one intersection");
            };
            assert!(hit.point.fuzzy_eq_eps(expected_point, 1e-14));
            assert!((hit.error_squared - expected_error_sq).abs() < expected_error_sq * 1e-6);
            assert!(
                hit.seg1_t.fuzzy_eq(expected_t) || hit.seg2_t.fuzzy_eq(expected_t),
                "expected projected segment parameter"
            );
        }
    }

    #[test]
    fn point_segment_support_and_snap_errors_do_not_overlap() {
        let epsilon = 1e-3;
        let overshoot = epsilon / 4.0;
        let support_distance = epsilon / 3.0;
        let line_start = Vector2::zero();
        let line_end = Vector2::new(1.0, 0.0);
        let point = Vector2::new(1.0 + overshoot, support_distance);
        let expected_error_sq = overshoot * overshoot + support_distance * support_distance;

        for result in [
            line_line_intr(line_start, line_end, point, point, epsilon),
            line_line_intr(point, point, line_start, line_end, epsilon),
        ] {
            let LineLineIntr::OneIntersect { hit } = result else {
                panic!("expected one intersection");
            };
            assert_eq!(hit.point, line_end);
            assert!((hit.error_squared - expected_error_sq).abs() < expected_error_sq * 1e-12);
        }
    }

    #[test]
    fn near_parallel_lines_keep_a_shared_endpoint_without_false_overlap() {
        assert_case_eq!(
            line_line_intr(
                Vector2::new(0.0, 0.0),
                Vector2::new(1e9, 0.0),
                Vector2::new(0.0, 0.0),
                Vector2::new(1e9, 1.0),
                1e-5,
            ),
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );
        assert_case_eq!(
            line_line_intr(
                Vector2::new(0.0, 0.0),
                Vector2::new(1e9, 0.0),
                Vector2::new(0.0, 2e-5),
                Vector2::new(1e9, 1.0),
                1e-5,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn position_equal_collinear_overlap_uses_a_point_on_both_segments() {
        for (line1_end, line2_start, line2_end) in [
            (
                Vector2::new(0.0, 10.0),
                Vector2::new(0.0, 5.0),
                Vector2::new(0.0, 5.000005),
            ),
            (
                Vector2::new(10.0, 0.0),
                Vector2::new(5.0, 0.0),
                Vector2::new(5.000005, 0.0),
            ),
        ] {
            assert_case_eq!(
                line_line_intr(Vector2::zero(), line1_end, line2_start, line2_end, 1e-5),
                TrueIntersect {
                    seg1_t: 0.5,
                    seg2_t: 0.0
                }
            );
        }
    }

    #[test]
    fn collinear_gap_uses_strict_path_distance_boundary() {
        let epsilon = 1.0 / 1024.0;
        let second_start = Vector2::zero();
        let second_end = Vector2::new(2.0, 0.0);
        let gap = epsilon / 2.0;
        assert_case_eq!(
            line_line_intr(
                Vector2::new(2.0 + gap, 0.0),
                Vector2::new(4.0 + gap, 0.0),
                second_start,
                second_end,
                epsilon,
            ),
            TrueIntersect {
                seg1_t: -gap / 2.0,
                seg2_t: 1.0
            }
        );
        assert_case_eq!(
            line_line_intr(
                Vector2::new(2.0 + epsilon, 0.0),
                Vector2::new(4.0 + epsilon, 0.0),
                second_start,
                second_end,
                epsilon,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn overlap_parameters_use_the_second_segment_domain() {
        let first_start = Vector2::new(0.5, 0.0);
        let first_end = Vector2::new(1.5, 0.0);
        let second_start = Vector2::zero();
        let second_end = Vector2::new(1.0, 0.0);
        assert_case_eq!(
            line_line_intr(first_start, first_end, second_start, second_end, 1e-5),
            Overlapping {
                seg2_t0: 0.5,
                seg2_t1: 1.0
            }
        );
        assert_case_eq!(
            line_line_intr(second_start, second_end, first_start, first_end, 1e-5),
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 0.5
            }
        );
    }

    #[test]
    fn collinearity_uses_strict_scale_aware_distance() {
        let epsilon = 1.0 / 1024.0;
        let first_start = Vector2::zero();
        let first_end = Vector2::new(1024.0, 0.0);
        assert_case_eq!(
            line_line_intr(
                first_start,
                first_end,
                Vector2::new(0.0, epsilon / 2.0),
                Vector2::new(1024.0, epsilon / 2.0),
                epsilon,
            ),
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );
        assert_case_eq!(
            line_line_intr(
                first_start,
                first_end,
                Vector2::new(0.0, epsilon),
                Vector2::new(1024.0, epsilon),
                epsilon,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn endpoint_touches_are_stable_under_argument_swap_and_rotation() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(1.0, 1.0);
        let v2 = Vector2::new(2.0, 2.0);

        assert_case_eq!(
            line_line_intr(u1, u2, v1, v2, 1e-5),
            TrueIntersect {
                seg1_t: 1.0,
                seg2_t: 0.0
            }
        );
        assert_case_eq!(
            line_line_intr(v1, v2, u1, u2, 1e-5),
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 1.0
            }
        );
        for &angle in TEST_ROTATION_ANGLES {
            assert_case_eq!(
                line_line_intr(u1, u2, v1, v2.rotate_about(v1, angle), 1e-5),
                TrueIntersect {
                    seg1_t: 1.0,
                    seg2_t: 0.0
                }
            );
        }

        let u1 = Vector2::zero();
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::zero();
        let v2 = Vector2::new(-1.0, -1.0);
        assert_case_eq!(
            line_line_intr(u1, u2, v1, v2, 1e-5),
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );
        assert_case_eq!(
            line_line_intr(v1, v2, u1, u2, 1e-5),
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );
        for &angle in TEST_ROTATION_ANGLES {
            assert_case_eq!(
                line_line_intr(u1, u2, v1, v2.rotate_about(v1, angle), 1e-5),
                TrueIntersect {
                    seg1_t: 0.0,
                    seg2_t: 0.0
                }
            );
        }
    }

    #[test]
    fn distant_supporting_intersection_is_rejected() {
        assert_case_eq!(
            line_line_intr(
                Vector2::new(-1.0, -1.0),
                Vector2::new(-0.5, -0.5),
                Vector2::new(-1.0, 1.0),
                Vector2::new(1.0, -1.0),
                1e-5,
            ),
            Rejected::<f64>
        );
    }

    #[test]
    fn exterior_parameters_use_bounded_segment_classification() {
        let epsilon = 1.0 / 1024.0;
        let line1_start = Vector2::zero();
        let line1_end = Vector2::new(2.0, 0.0);

        for distance in [0.25 * epsilon, 0.99 * epsilon] {
            for x in [-distance, 2.0 + distance] {
                assert_case_eq!(
                    line_line_intr(
                        line1_start,
                        line1_end,
                        Vector2::new(x, -1.0),
                        Vector2::new(x, 1.0),
                        epsilon,
                    ),
                    TrueIntersect {
                        seg1_t: x / 2.0,
                        seg2_t: 0.5
                    }
                );
            }
        }

        for distance in [epsilon, 1.01 * epsilon] {
            for x in [-distance, 2.0 + distance] {
                assert_case_eq!(
                    line_line_intr(
                        line1_start,
                        line1_end,
                        Vector2::new(x, -1.0),
                        Vector2::new(x, 1.0),
                        epsilon,
                    ),
                    Rejected::<f64>
                );
            }
        }
    }

    #[test]
    fn endpoint_snap_reports_squared_error() {
        let epsilon = 1e-5;
        let gap = epsilon / 2.0;
        let LineLineIntr::OneIntersect { hit } = line_line_intr(
            Vector2::zero(),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0 + gap, -1.0),
            Vector2::new(1.0 + gap, 1.0),
            epsilon,
        ) else {
            panic!("expected one intersection");
        };
        assert_eq!(hit.point, Vector2::new(1.0, 0.0));
        assert!((hit.error_squared - gap * gap).abs() < f64::EPSILON);
    }

    #[test]
    fn parallel_noncollinear_lines_do_not_intersect() {
        for (v1, v2, u1, u2) in [
            (
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::new(0.0, 1.0),
                Vector2::new(1.0, 2.0),
            ),
            (
                Vector2::new(2.0, 0.0),
                Vector2::new(2.0, 1.0),
                Vector2::new(-1.0, -1.0),
                Vector2::new(-1.0, -2.0),
            ),
            (
                Vector2::new(-2.0, -1.0),
                Vector2::new(2.0, -1.0),
                Vector2::new(-1.0, 5.0),
                Vector2::new(1.0, 5.0),
            ),
        ] {
            assert_case_eq!(line_line_intr(v1, v2, u1, u2, 1e-5), NoIntersect::<f64>);
        }
    }

    #[test]
    fn partially_overlapping_lines_return_the_second_segment_domain() {
        assert_case_eq!(
            line_line_intr(
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::new(0.0, 0.0),
                Vector2::new(0.5, 0.5),
                1e-5,
            ),
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );
    }

    #[test]
    fn point_segments_intersect_lines_in_any_orientation_and_argument_order() {
        for (line_start, line_end, point, line_t) in [
            (
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::zero(),
                0.5,
            ),
            (
                Vector2::new(0.0, -1.0),
                Vector2::new(0.0, 1.0),
                Vector2::zero(),
                0.5,
            ),
            (
                Vector2::new(-1.0, 0.0),
                Vector2::new(1.0, 0.0),
                Vector2::zero(),
                0.5,
            ),
            (
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::new(-1.0, -1.0),
                0.0,
            ),
            (
                Vector2::new(-1.0, -1.0),
                Vector2::new(1.0, 1.0),
                Vector2::new(1.0, 1.0),
                1.0,
            ),
        ] {
            assert_case_eq!(
                line_line_intr(line_start, line_end, point, point, 1e-5),
                TrueIntersect {
                    seg1_t: line_t,
                    seg2_t: 0.0
                }
            );
            assert_case_eq!(
                line_line_intr(point, point, line_start, line_end, 1e-5),
                TrueIntersect {
                    seg1_t: 0.0,
                    seg2_t: line_t
                }
            );
        }
    }

    #[test]
    fn entirely_overlapping_lines_are_stable_under_direction_and_rotation() {
        let start = Vector2::new(-1.0, -1.0);
        let end = Vector2::new(1.0, 1.0);
        let expected = Overlapping {
            seg2_t0: 0.0,
            seg2_t1: 1.0,
        };

        assert_case_eq!(line_line_intr(start, end, start, end, 1e-5), expected);
        assert_case_eq!(line_line_intr(start, end, end, start, 1e-5), expected);
        assert_case_eq!(line_line_intr(end, start, start, end, 1e-5), expected);

        for &angle in TEST_ROTATION_ANGLES {
            let rotated_end = end.rotate_about(start, angle);
            assert_case_eq!(
                line_line_intr(start, rotated_end, start, rotated_end, 1e-5),
                expected
            );
            assert_case_eq!(
                line_line_intr(start, rotated_end, rotated_end, start, 1e-5),
                expected
            );
        }
    }
}
