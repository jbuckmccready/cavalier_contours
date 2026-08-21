use super::Vector2;
use crate::core::traits::Real;

/// Holds the result of finding the intersect between a line segment and a circle.
#[derive(Debug, Copy, Clone)]
pub enum LineCircleIntr<T>
where
    T: Real,
{
    /// No intersects found.
    NoIntersect,
    /// One tangent intersect point found.
    TangentIntersect {
        /// Holds the line segment parametric value for where the intersect point is.
        t0: T,
    },
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// Holds the line segment parametric value for where the first intersect point is.
        t0: T,
        /// Holds the line segment parametric value for where the second intersect point is.
        t1: T,
    },
}

/// Finds the intersects between a line segment and a circle.
///
/// This function returns the parametric solution(s) for the line segment equation
/// `P(t) = p0 + t * (p1 - p0)` for `t = 0` to `t = 1`. If `t < 0` or `t > 1` then intersect occurs
/// only when extending the segment out past the points `p0` and `p1` given.
/// If `t < 0` then the intersect is nearest to `p0`, if `t > 1.0` then the intersect is
/// nearest to `p1`. Finite segment domain acceptance is left to the caller.
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
/// if let LineCircleIntr::TangentIntersect{ t0: t } = line_circle_intr(p0, p1, r, c, 1e-5) {
///     assert_eq!(t, 0.0);
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
    use LineCircleIntr::{NoIntersect, TangentIntersect, TwoIntersects};

    let eps = epsilon;
    let line_dir = p1 - p0;
    let line_len2 = line_dir.length_squared();

    // Only a zero-length segment is a point. A nonzero segment shorter than epsilon still has a
    // supporting line and must retain its direction.
    if line_len2 == T::zero() {
        let center_offset = p0 - circle_center;
        let center_dist2 = center_offset.length_squared();
        let outer_limit = radius + eps;
        let inner_limit = radius - eps;
        if center_dist2 < outer_limit * outer_limit
            && (inner_limit < T::zero() || center_dist2 > inner_limit * inner_limit)
        {
            return TangentIntersect { t0: T::zero() };
        }

        return NoIntersect;
    }

    let center_offset = circle_center - p0;
    let cross = line_dir.perp_dot(center_offset);
    let cross2 = cross * cross;
    let radius2 = radius * radius;
    let scaled_radius2 = radius2 * line_len2;

    if cross2 >= scaled_radius2 {
        let recovery_limit = radius + eps;
        if cross2 >= recovery_limit * recovery_limit * line_len2 {
            return NoIntersect;
        }
        let inv_line_len2 = T::one() / line_len2;
        return TangentIntersect {
            t0: center_offset.dot(line_dir) * inv_line_len2,
        };
    }

    let inv_line_len2 = T::one() / line_len2;
    let midpoint_t = center_offset.dot(line_dir) * inv_line_len2;
    let half_chord2 = radius2 - cross2 * inv_line_len2;
    if T::four() * half_chord2 < eps * eps {
        return TangentIntersect { t0: midpoint_t };
    }

    let t_offset = (half_chord2 * inv_line_len2).sqrt();
    TwoIntersects {
        t0: midpoint_t - t_offset,
        t1: midpoint_t + t_offset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::traits::FuzzyEq;

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (LineCircleIntr::NoIntersect, LineCircleIntr::NoIntersect) => {}
                (
                    LineCircleIntr::TangentIntersect { t0: a1 },
                    LineCircleIntr::TangentIntersect { t0: a2 },
                ) if a1.fuzzy_eq(a2) => {}
                (
                    LineCircleIntr::TwoIntersects { t0: a1, t1: b1 },
                    LineCircleIntr::TwoIntersects { t0: a2, t1: b2 },
                ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
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
    fn very_short_diagonal_line_keeps_both_supporting_roots() {
        let result = line_circle_intr(
            Vector2::new(0.9999999995, -0.0000000005),
            Vector2::new(1.0000000005, 0.0000000005),
            1.0,
            Vector2::zero(),
            1e-12,
        );
        assert!(
            matches!(result, LineCircleIntr::TwoIntersects { .. }),
            "{result:?}"
        );
    }

    #[test]
    fn nonzero_segment_shorter_than_epsilon_keeps_both_supporting_roots() {
        let epsilon = 1.0 / 1024.0;
        assert_case_eq!(
            line_circle_intr(
                Vector2::new(-epsilon / 4.0, 0.0),
                Vector2::new(epsilon / 4.0, 0.0),
                1.0,
                Vector2::zero(),
                epsilon,
            ),
            LineCircleIntr::TwoIntersects {
                t0: 0.5 - 2.0 / epsilon,
                t1: 0.5 + 2.0 / epsilon
            }
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
