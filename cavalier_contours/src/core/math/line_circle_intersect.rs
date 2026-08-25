use super::{Vector2, dist_squared};
use crate::core::traits::Real;

/// One intersection between a line segment and a circle.
#[derive(Debug, Copy, Clone)]
pub struct LineCircleIntrPoint<T>
where
    T: Real,
{
    /// Resolved intersection point.
    pub point: Vector2<T>,
    /// Sum of squared geometric adjustments used to produce `point`.
    pub error_squared: T,
    /// Resolved line parameter. Values accepted just outside the segment are clamped to 0 or 1.
    pub line_t: T,
}

/// Holds the result of finding the intersection between a line segment and a circle.
#[derive(Debug, Copy, Clone)]
pub enum LineCircleIntr<T>
where
    T: Real,
{
    /// No intersects found.
    NoIntersect,
    /// One tangent intersect point found.
    TangentIntersect {
        /// Tangent intersection.
        hit: LineCircleIntrPoint<T>,
    },
    /// One non-tangent intersection point found.
    OneIntersect {
        /// Intersection point.
        hit: LineCircleIntrPoint<T>,
    },
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// First intersection, ordered by line parameter.
        hit1: LineCircleIntrPoint<T>,
        /// Second intersection, ordered by line parameter.
        hit2: LineCircleIntrPoint<T>,
    },
}

/// Finds the intersects between a line segment and a circle.
///
/// Solutions just outside the finite line domain are clamped to the nearest endpoint when their
/// line-path overshoot is strictly less than `epsilon`. More distant solutions are omitted.
///
/// `epsilon` must be greater than zero. All epsilon comparisons are strict:
///
/// - A point segment is on the circle when its distance from the circle edge is less than
///   `epsilon`.
/// - A supporting line less than `epsilon` outside the circle returns one intersection at its
///   nearest point.
/// - Two intersection points less than `epsilon` apart return one intersection at their midpoint.
///
/// A distance equal to `epsilon` does not satisfy any of these cases.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::core::math::LineCircleIntr::TangentIntersect;
/// // A line segment tangent-intersecting a circle with one of the line segments end points.
/// let p0 = Vector2::new(0.0, 0.0);
/// let p1 = Vector2::new(1.0, 0.0);
/// let r = 1.0;
/// let c = Vector2::new(0.0, 1.0);
///
/// if let LineCircleIntr::TangentIntersect { hit } = line_circle_intr(p0, p1, r, c, 1e-5) {
///     assert_eq!(hit.line_t, 0.0);
/// } else {
///     unreachable!("expected tangent intersect");
/// }
///```
#[must_use]
pub fn line_circle_intr<T>(
    p0: Vector2<T>,
    p1: Vector2<T>,
    radius: T,
    circle_center: Vector2<T>,
    epsilon: T,
) -> LineCircleIntr<T>
where
    T: Real,
{
    use LineCircleIntr::{NoIntersect, OneIntersect, TangentIntersect, TwoIntersects};

    let eps = epsilon;
    let eps_sq = eps * eps;
    let line_dir = p1 - p0;
    let line_len_sq = line_dir.length_squared();

    // Only a zero-length segment is a point. A nonzero segment shorter than epsilon still has a
    // supporting line and must retain its direction.
    if line_len_sq == T::zero() {
        let center_offset = p0 - circle_center;
        let center_dist_sq = center_offset.length_squared();
        let outer_limit = radius + eps;
        let inner_limit = radius - eps;
        if center_dist_sq < outer_limit * outer_limit
            && (inner_limit < T::zero() || center_dist_sq > inner_limit * inner_limit)
        {
            let radial_error = center_dist_sq.sqrt() - radius;
            return TangentIntersect {
                hit: LineCircleIntrPoint {
                    point: p0,
                    error_squared: radial_error * radial_error,
                    line_t: T::zero(),
                },
            };
        }

        return NoIntersect;
    }

    let center_offset = circle_center - p0;
    let cross = line_dir.perp_dot(center_offset);
    let cross_sq = cross * cross;
    let radius_sq = radius * radius;
    let scaled_radius_sq = radius_sq * line_len_sq;
    let resolve_hit = |t: T, support_error_sq: T| {
        if t < T::zero() {
            let snap_error_sq = t * t * line_len_sq;
            if snap_error_sq >= eps_sq {
                return None;
            }
            Some(LineCircleIntrPoint {
                point: p0,
                error_squared: support_error_sq + snap_error_sq,
                line_t: T::zero(),
            })
        } else if t > T::one() {
            let overshoot = t - T::one();
            let snap_error_sq = overshoot * overshoot * line_len_sq;
            if snap_error_sq >= eps_sq {
                return None;
            }
            Some(LineCircleIntrPoint {
                point: p1,
                error_squared: support_error_sq + snap_error_sq,
                line_t: T::one(),
            })
        } else {
            Some(LineCircleIntrPoint {
                point: p0 + line_dir.scale(t),
                error_squared: support_error_sq,
                line_t: t,
            })
        }
    };

    if cross_sq >= scaled_radius_sq {
        let recovery_limit = radius + eps;
        if cross_sq >= recovery_limit * recovery_limit * line_len_sq {
            return NoIntersect;
        }
        let inv_line_len_sq = T::one() / line_len_sq;
        let t = center_offset.dot(line_dir) * inv_line_len_sq;
        let line_distance = (cross_sq * inv_line_len_sq).sqrt();
        let radial_error = line_distance - radius;
        return resolve_hit(t, radial_error * radial_error)
            .map_or(NoIntersect, |hit| TangentIntersect { hit });
    }

    let inv_line_len_sq = T::one() / line_len_sq;
    let midpoint_t = center_offset.dot(line_dir) * inv_line_len_sq;
    let half_chord_sq = radius_sq - cross_sq * inv_line_len_sq;
    if T::four() * half_chord_sq < eps_sq {
        return resolve_hit(midpoint_t, half_chord_sq)
            .map_or(NoIntersect, |hit| TangentIntersect { hit });
    }

    let t_offset = (half_chord_sq * inv_line_len_sq).sqrt();
    let first = resolve_hit(midpoint_t - t_offset, T::zero());
    let second = resolve_hit(midpoint_t + t_offset, T::zero());
    match (first, second) {
        (None, None) => NoIntersect,
        (Some(hit), None) | (None, Some(hit)) => OneIntersect { hit },
        (Some(hit1), Some(hit2)) => {
            let separation_sq = dist_squared(hit1.point, hit2.point);
            if separation_sq < eps_sq {
                let mut hit = if hit1.error_squared <= hit2.error_squared {
                    hit1
                } else {
                    hit2
                };
                hit.error_squared = hit.error_squared + separation_sq;
                OneIntersect { hit }
            } else {
                TwoIntersects { hit1, hit2 }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::traits::FuzzyEq;

    #[derive(Debug, Copy, Clone)]
    enum ExpectedLineCircleIntr<T> {
        NoIntersect,
        TangentIntersect { t0: T },
        OneIntersect { t0: T },
        TwoIntersects { t0: T, t1: T },
    }
    use ExpectedLineCircleIntr as LineCircleIntr;

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (super::LineCircleIntr::NoIntersect, LineCircleIntr::NoIntersect) => {}
                (
                    super::LineCircleIntr::TangentIntersect { hit },
                    LineCircleIntr::TangentIntersect { t0: a2 },
                ) if hit.line_t.fuzzy_eq(a2) => {}
                (
                    super::LineCircleIntr::OneIntersect { hit },
                    LineCircleIntr::OneIntersect { t0: a2 },
                ) if hit.line_t.fuzzy_eq(a2) => {}
                (
                    super::LineCircleIntr::TwoIntersects { hit1, hit2 },
                    LineCircleIntr::TwoIntersects { t0: a2, t1: b2 },
                ) if hit1.line_t.fuzzy_eq(a2) && hit2.line_t.fuzzy_eq(b2) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            };
        };
    }

    #[test]
    fn no_intersect() {
        let result = line_circle_intr(
            Vector2::new(-1.0, -1.0),
            Vector2::new(1.0, 1.0),
            0.5,
            Vector2::new(0.0, 5.0),
            1e-5,
        );
        assert_case_eq!(result, LineCircleIntr::NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_vertical() {
        let result = line_circle_intr(
            Vector2::new(0.0, -1.0),
            Vector2::new(0.0, 1.0),
            0.5,
            Vector2::new(2.0, 0.0),
            1e-5,
        );
        assert_case_eq!(result, LineCircleIntr::NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_horizontal() {
        let result = line_circle_intr(
            Vector2::new(1.0, 1.0),
            Vector2::new(3.0, 1.0),
            0.5,
            Vector2::new(2.0, -2.0),
            1e-5,
        );
        assert_case_eq!(result, LineCircleIntr::NoIntersect::<f64>);
    }

    #[test]
    fn two_intersects_true() {
        let p0 = Vector2::new(-1.0, -1.0);
        let p1 = Vector2::new(1.0, 1.0);
        // Place one edge of the circle at the origin.
        let radius = 0.5f64;
        let offset = (radius * radius / 2.0).sqrt();
        let expected_t1 = (2.0 * offset - p0.x) / (p1.x - p0.x);
        assert_case_eq!(
            line_circle_intr(p0, p1, radius, Vector2::new(offset, offset), 1e-5),
            LineCircleIntr::TwoIntersects {
                t0: 0.5,
                t1: expected_t1
            }
        );
    }

    #[test]
    fn two_intersects_seg_inside_vertical() {
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(0.0, -1.0),
                Vector2::new(0.0, 1.0),
                1.0,
                Vector2::zero(),
                1e-5,
            ),
            LineCircleIntr::TwoIntersects { t0: 0.0, t1: 1.0 }
        );
    }

    #[test]
    fn two_intersects_seg_inside_horizontal() {
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(-1.0, 0.0),
                Vector2::new(1.0, 0.0),
                1.0,
                Vector2::zero(),
                1e-5,
            ),
            LineCircleIntr::TwoIntersects { t0: 0.0, t1: 1.0 }
        );
    }

    #[test]
    fn two_intersects_seg_touching() {
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(0.0, -1.0),
                Vector2::new(0.0, 1.0),
                1.0,
                Vector2::zero(),
                1e-5,
            ),
            LineCircleIntr::TwoIntersects { t0: 0.0, t1: 1.0 }
        );
    }

    #[test]
    fn tangent_intersect_vertical() {
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(0.0, -1.0),
                Vector2::new(0.0, 1.0),
                1.0,
                Vector2::new(1.0, 0.0),
                1e-5,
            ),
            LineCircleIntr::TangentIntersect { t0: 0.5 }
        );
    }

    #[test]
    fn tangent_intersect_horizontal() {
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(-1.0, 0.0),
                Vector2::new(1.0, 0.0),
                1.0,
                Vector2::new(0.0, -1.0),
                1e-5,
            ),
            LineCircleIntr::TangentIntersect { t0: 0.5 }
        );
    }

    #[test]
    fn tangent_at_start_point() {
        // This case previously failed due to rounding at exact tangency.
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(161.29, 113.665),
                Vector2::new(167.64, 113.665),
                0.634_999_999_999_998,
                Vector2::new(161.29, 114.30000000000001),
                1e-5,
            ),
            LineCircleIntr::TangentIntersect { t0: 0.0 }
        );
    }

    #[test]
    fn near_tangent_intersections_use_position_epsilon_at_different_radii() {
        let p0 = Vector2::new(-1.0, 0.0);
        let p1 = Vector2::new(1.0, 0.0);
        let half_intersect_spacing = 5e-4f64;

        for radius in [0.01, 1.0, 100.0] {
            let center_y =
                (radius * radius - half_intersect_spacing * half_intersect_spacing).sqrt();
            assert_case_eq!(
                line_circle_intr(p0, p1, radius, Vector2::new(0.0, center_y), 1e-5),
                LineCircleIntr::TwoIntersects {
                    t0: 0.5 - half_intersect_spacing / 2.0,
                    t1: 0.5 + half_intersect_spacing / 2.0,
                }
            );
        }
    }

    #[test]
    fn near_tangent_intersections_snap_when_positions_are_equal() {
        let p0 = Vector2::new(-1.0, 0.0);
        let p1 = Vector2::new(1.0, 0.0);
        let half_intersect_spacing = 2e-6f64;

        for radius in [0.01, 1.0, 100.0] {
            let center_y =
                (radius * radius - half_intersect_spacing * half_intersect_spacing).sqrt();
            assert_case_eq!(
                line_circle_intr(p0, p1, radius, Vector2::new(0.0, center_y), 1e-5),
                LineCircleIntr::TangentIntersect { t0: 0.5 }
            );
        }
    }

    #[test]
    fn coalesced_roots_report_squared_error() {
        let half_spacing = 2e-6f64;
        let center_y = (1.0 - half_spacing * half_spacing).sqrt();
        let super::LineCircleIntr::TangentIntersect { hit } = line_circle_intr(
            Vector2::new(-1.0, 0.0),
            Vector2::new(1.0, 0.0),
            1.0,
            Vector2::new(0.0, center_y),
            1e-5,
        ) else {
            panic!("expected coalesced intersections");
        };
        let expected_error_sq = half_spacing * half_spacing;
        assert!((hit.error_squared - expected_error_sq).abs() < expected_error_sq * 1e-4);
    }

    #[test]
    fn point_segment_uses_position_epsilon() {
        let point = Vector2::new(0.0, 100.001);
        assert_case_eq!(
            line_circle_intr(point, point, 100.0, Vector2::zero(), 0.01),
            LineCircleIntr::TangentIntersect { t0: 0.0 }
        );
    }

    #[test]
    fn point_segments_use_strict_radial_distance_boundaries() {
        let epsilon = 1.0 / 1024.0;
        for point in [
            Vector2::new(1.0 + epsilon / 2.0, 0.0),
            Vector2::new(1.0 - epsilon / 2.0, 0.0),
        ] {
            assert_case_eq!(
                line_circle_intr(point, point, 1.0, Vector2::zero(), epsilon),
                LineCircleIntr::TangentIntersect { t0: 0.0 }
            );
        }
        for point in [
            Vector2::new(1.0 + epsilon, 0.0),
            Vector2::new(1.0 - epsilon, 0.0),
        ] {
            assert_case_eq!(
                line_circle_intr(point, point, 1.0, Vector2::zero(), epsilon),
                LineCircleIntr::NoIntersect::<f64>
            );
        }
        assert_case_eq!(
            line_circle_intr(
                Vector2::zero(),
                Vector2::zero(),
                epsilon / 2.0,
                Vector2::zero(),
                epsilon,
            ),
            LineCircleIntr::TangentIntersect { t0: 0.0 }
        );
        assert_case_eq!(
            line_circle_intr(
                Vector2::zero(),
                Vector2::zero(),
                epsilon,
                Vector2::zero(),
                epsilon,
            ),
            LineCircleIntr::NoIntersect::<f64>
        );
    }

    #[test]
    fn recovered_tangent_uses_strict_position_boundary_at_different_radii() {
        for radius in [1.0 / 64.0, 1.0, 64.0] {
            let epsilon = radius / 1024.0;
            let p0 = Vector2::new(-2.0 * radius, 0.0);
            let p1 = Vector2::new(2.0 * radius, 0.0);
            for (factor, intersects) in [(0.25, true), (0.99, true), (1.0, false), (1.01, false)] {
                let center = Vector2::new(0.0, radius + factor * epsilon);
                let result = line_circle_intr(p0, p1, radius, center, epsilon);
                if intersects {
                    assert_case_eq!(result, LineCircleIntr::TangentIntersect { t0: 0.5 });
                } else {
                    assert_case_eq!(result, LineCircleIntr::NoIntersect::<f64>);
                }
            }
        }
    }

    #[test]
    fn recovered_tangent_reports_squared_error() {
        let epsilon = 1e-5f64;
        let gap = epsilon / 2.0;
        let super::LineCircleIntr::TangentIntersect { hit } = line_circle_intr(
            Vector2::new(-2.0, 0.0),
            Vector2::new(2.0, 0.0),
            1.0,
            Vector2::new(0.0, 1.0 + gap),
            epsilon,
        ) else {
            panic!("expected a recovered tangent");
        };
        let expected_error_sq = gap * gap;
        assert!((hit.error_squared - expected_error_sq).abs() < expected_error_sq * 1e-9);
    }

    #[test]
    fn endpoint_snap_reports_squared_error() {
        let epsilon = 1e-5f64;
        let gap = epsilon / 2.0;
        let super::LineCircleIntr::OneIntersect { hit } = line_circle_intr(
            Vector2::new(1.0 + gap, 0.0),
            Vector2::new(2.0, 0.0),
            1.0,
            Vector2::zero(),
            epsilon,
        ) else {
            panic!("expected an endpoint intersection");
        };
        assert_eq!(hit.point, Vector2::new(1.0 + gap, 0.0));
        assert!(hit.line_t.fuzzy_eq(0.0));
        assert!((hit.error_squared - gap * gap).abs() < f64::EPSILON);
    }

    #[test]
    fn endpoint_snapped_root_coalesces_with_nearby_interior_root() {
        let epsilon = 1e-5f64;
        let half_spacing = 0.75 * epsilon;
        let center = Vector2::new(-0.15 * epsilon, (1.0 - half_spacing * half_spacing).sqrt());
        let super::LineCircleIntr::OneIntersect { hit } = line_circle_intr(
            Vector2::zero(),
            Vector2::new(1.0, 0.0),
            1.0,
            center,
            epsilon,
        ) else {
            panic!("expected coalesced endpoint and interior intersections");
        };
        assert!(hit.point.x > 0.0 && hit.point.x < epsilon);
        assert!(hit.error_squared > 0.0);
        assert!((hit.error_squared - hit.point.length_squared()).abs() < hit.error_squared * 1e-9);
    }

    #[test]
    fn very_short_diagonal_line_keeps_its_finite_root() {
        let result = line_circle_intr(
            Vector2::new(0.9999999995, -0.0000000005),
            Vector2::new(1.0000000005, 0.0000000005),
            1.0,
            Vector2::zero(),
            1e-12,
        );
        assert_case_eq!(result, LineCircleIntr::OneIntersect { t0: 0.5 });
    }

    #[test]
    fn nonzero_segment_shorter_than_epsilon_rejects_distant_roots() {
        let epsilon = 1.0 / 1024.0;
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(-epsilon / 4.0, 0.0),
                Vector2::new(epsilon / 4.0, 0.0),
                1.0,
                Vector2::zero(),
                epsilon,
            ),
            LineCircleIntr::NoIntersect::<f64>
        );
    }

    #[test]
    fn coalesced_intersections_use_strict_point_separation_boundary() {
        let p0 = Vector2::new(-4.0, 4.0);
        let p1 = Vector2::new(4.0, 4.0);
        assert_case_eq!(
            line_circle_intr(p0, p1, 5.0, Vector2::zero(), 6.25),
            LineCircleIntr::TangentIntersect { t0: 0.5 }
        );
        for epsilon in [6.0, 5.75] {
            assert_case_eq!(
                line_circle_intr(p0, p1, 5.0, Vector2::zero(), epsilon),
                LineCircleIntr::TwoIntersects {
                    t0: 0.125,
                    t1: 0.875
                }
            );
        }
    }
}
