use super::Vector2;
use crate::core::traits::Real;

/// Holds the result of finding the intersect between two circles.
#[derive(Debug, Copy, Clone)]
pub enum CircleCircleIntr<T>
where
    T: Real,
{
    /// No intersects found.
    NoIntersect,
    /// One touching point found, or two nearby points combined into one.
    TangentIntersect {
        /// Holds the tangent intersect point.
        point: Vector2<T>,
    },
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// Holds the first intersect point.
        point1: Vector2<T>,
        /// Holds the second intersect point.
        point2: Vector2<T>,
    },
    /// Circles are treated as the same circle.
    Overlapping,
}

/// Finds the intersects between two circles defined by the radius and center.
///
/// `epsilon` must be greater than zero. All epsilon comparisons are strict:
///
/// - Circles are [`CircleCircleIntr::Overlapping`] when center distance plus radius difference is
///   less than `epsilon`.
/// - Circles whose nearest edges are less than `epsilon` apart return
///   [`CircleCircleIntr::TangentIntersect`] at the midpoint of those edges.
/// - Two intersection points less than `epsilon` apart return
///   [`CircleCircleIntr::TangentIntersect`] at their midpoint.
///
/// A distance equal to `epsilon` does not satisfy any of these cases.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::core::math::CircleCircleIntr::TangentIntersect;
/// // Two tangent-intersecting circles of radius `1.0` with euclidean distance of `2.0`
/// let r1 = 1.0;
/// let c1 = Vector2::new(0.0, 0.0);
/// let r2 = 1.0;
/// let c2 = Vector2::new(0.0, 2.0);
/// if let CircleCircleIntr::TangentIntersect { point: p } =
///     circle_circle_intr(r1, c1, r2, c2, 1e-5)
/// {
///     assert_eq!(p, Vector2::new(0.0, 1.0));
/// } else {
///     unreachable!("expected a tangent intersection");
/// }
///```
#[must_use]
pub fn circle_circle_intr<T>(
    radius1: T,
    center1: Vector2<T>,
    radius2: T,
    center2: Vector2<T>,
    epsilon: T,
) -> CircleCircleIntr<T>
where
    T: Real,
{
    // Reference algorithm: http://paulbourke.net/geometry/circlesphere/
    use CircleCircleIntr::{NoIntersect, Overlapping, TangentIntersect, TwoIntersects};

    let cv = center2 - center1;
    let d2 = cv.dot(cv);
    let eps = epsilon;
    let radius_difference = (radius1 - radius2).abs();

    // Fuzzy-equal circles must stay within the position tolerance over their whole curves. Center
    // and radius errors can add, so compare the center distance against the remaining tolerance.
    if radius_difference < eps {
        let remaining_epsilon = eps - radius_difference;
        if d2 < remaining_epsilon * remaining_epsilon {
            return Overlapping;
        }
    }

    // Equal centers are either overlapping above or concentric with no intersections. Return
    // before dividing by the center distance.
    if d2 == T::zero() {
        return NoIntersect;
    }

    let external_limit = radius1 + radius2;
    let external_limit_squared = external_limit * external_limit;
    if d2 >= external_limit_squared {
        let external_reject_limit = external_limit + eps;
        if d2 >= external_reject_limit * external_reject_limit {
            return NoIntersect;
        }
        let d = d2.sqrt();
        let unit_center_vector = cv.scale(T::one() / d);
        return TangentIntersect {
            point: center1 + unit_center_vector.scale((radius1 + d - radius2) / T::two()),
        };
    }

    let radius_difference_squared = radius_difference * radius_difference;
    if d2 <= radius_difference_squared {
        if radius_difference >= eps {
            let internal_reject_limit = radius_difference - eps;
            if d2 <= internal_reject_limit * internal_reject_limit {
                return NoIntersect;
            }
        }
        let d = d2.sqrt();
        let unit_center_vector = cv.scale(T::one() / d);
        let distance_from_center1 = if radius1 > radius2 {
            (radius1 + d + radius2) / T::two()
        } else {
            (d - radius2 - radius1) / T::two()
        };
        return TangentIntersect {
            point: center1 + unit_center_vector.scale(distance_from_center1),
        };
    }

    let d = d2.sqrt();
    let rad1_sq = radius1 * radius1;
    let inv_d = T::one() / d;
    let a = (rad1_sq - radius2 * radius2 + d2) * inv_d / T::two();
    let midpoint = center1 + cv.scale(a * inv_d);
    let diff = rad1_sq - a * a;

    if diff <= T::zero() {
        return TangentIntersect { point: midpoint };
    }

    if T::four() * diff < eps * eps {
        return TangentIntersect { point: midpoint };
    }

    let h_over_d = diff.sqrt() * inv_d;
    let x_term = h_over_d * cv.y;
    let y_term = h_over_d * cv.x;

    let pt1 = Vector2::new(midpoint.x + x_term, midpoint.y - y_term);
    let pt2 = Vector2::new(midpoint.x - x_term, midpoint.y + y_term);

    TwoIntersects {
        point1: pt1,
        point2: pt2,
    }
}

#[cfg(test)]
mod tests {
    use super::{CircleCircleIntr::*, *};

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (NoIntersect, NoIntersect) => {}
                (TangentIntersect { point: a1 }, TangentIntersect { point: a2 })
                    if a1.fuzzy_eq(a2) => {}
                (
                    TwoIntersects {
                        point1: a1,
                        point2: b1,
                    },
                    TwoIntersects {
                        point1: a2,
                        point2: b2,
                    },
                ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
                (Overlapping, Overlapping) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            }
        };
    }

    #[test]
    fn no_intersect_outside() {
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::new(-1.0, -1.0),
                0.5,
                Vector2::new(0.0, 5.0),
                1e-5,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn no_intersect_inside() {
        assert_case_eq!(
            circle_circle_intr(
                5.0,
                Vector2::new(-1.0, -1.0),
                0.5,
                Vector2::new(1.0, 1.0),
                1e-5,
            ),
            NoIntersect::<f64>
        );
    }

    #[test]
    fn tangent_intersect_outside() {
        let center1 = Vector2::new(-1.0, 1.0);
        let center2 = Vector2::new(0.5, 1.0);
        let expected = TangentIntersect {
            point: Vector2::new(0.0, 1.0),
        };
        assert_case_eq!(
            circle_circle_intr(1.0, center1, 0.5, center2, 1e-5),
            expected
        );
        assert_case_eq!(
            circle_circle_intr(0.5, center2, 1.0, center1, 1e-5),
            expected
        );
    }

    #[test]
    fn tangent_intersect_inside() {
        let center1 = Vector2::new(0.0, 1.0);
        let center2 = Vector2::zero();
        let expected = TangentIntersect {
            point: Vector2::new(0.0, 4.0),
        };
        assert_case_eq!(
            circle_circle_intr(3.0, center1, 4.0, center2, 1e-5),
            expected
        );
        assert_case_eq!(
            circle_circle_intr(4.0, center2, 3.0, center1, 1e-5),
            expected
        );
    }

    #[test]
    fn two_intersects() {
        assert_case_eq!(
            circle_circle_intr(
                3.0,
                Vector2::new(0.0, 1.0),
                4.0,
                Vector2::new(5.0, 5.0),
                1e-5,
            ),
            TwoIntersects {
                point1: Vector2::new(2.945782625365772, 1.567771718292785),
                point2: Vector2::new(1.2005588380488623, 3.749301452438922)
            }
        );
    }

    #[test]
    fn overlapping() {
        let center = Vector2::new(-1.0, 1.0);
        assert_case_eq!(
            circle_circle_intr(1.0, center, 1.0, center, 1e-5),
            Overlapping::<f64>
        );
    }

    #[test]
    fn recovered_external_tangency_uses_strict_position_boundary_across_scales() {
        for radius in [0.01, 1.0, 100.0] {
            let epsilon = radius / 1024.0;
            for (factor, intersects) in [(0.25, true), (0.99, true), (1.0, false), (1.01, false)] {
                let center_gap = factor * epsilon;
                let result = circle_circle_intr(
                    radius,
                    Vector2::zero(),
                    radius,
                    Vector2::new(2.0 * radius + center_gap, 0.0),
                    epsilon,
                );
                if intersects {
                    assert_case_eq!(
                        result,
                        TangentIntersect {
                            point: Vector2::new(radius + center_gap / 2.0, 0.0)
                        }
                    );
                } else {
                    assert_case_eq!(result, NoIntersect::<f64>);
                }
            }
        }
    }

    #[test]
    fn recovered_external_tangency_handles_unequal_radii_and_argument_swap() {
        let epsilon = 1.0 / 1024.0;
        let gap = epsilon / 2.0;
        let center1 = Vector2::zero();
        let center2 = Vector2::new(3.0 + gap, 0.0);
        let expected = TangentIntersect {
            point: Vector2::new(1.0 + gap / 2.0, 0.0),
        };

        assert_case_eq!(
            circle_circle_intr(1.0, center1, 2.0, center2, epsilon),
            expected
        );
        assert_case_eq!(
            circle_circle_intr(2.0, center2, 1.0, center1, epsilon),
            expected
        );
    }

    #[test]
    fn coalesced_intersection_point_is_invariant_under_argument_swap() {
        let radius = 1.0f64;
        let half_spacing = 2e-6f64;
        let center_distance = 2.0 * (radius * radius - half_spacing * half_spacing).sqrt();
        let center1 = Vector2::zero();
        let center2 = Vector2::new(center_distance, 0.0);
        let expected = TangentIntersect {
            point: Vector2::new(center_distance / 2.0, 0.0),
        };

        assert_case_eq!(
            circle_circle_intr(radius, center1, radius, center2, 1e-5),
            expected
        );
        assert_case_eq!(
            circle_circle_intr(radius, center2, radius, center1, 1e-5),
            expected
        );
    }

    #[test]
    fn coalesced_intersections_use_strict_point_separation_boundary() {
        let center1 = Vector2::zero();
        let center2 = Vector2::new(8.0, 0.0);
        assert_case_eq!(
            circle_circle_intr(5.0, center1, 5.0, center2, 6.25),
            TangentIntersect {
                point: Vector2::new(4.0, 0.0)
            }
        );
        for epsilon in [6.0, 5.75] {
            assert_case_eq!(
                circle_circle_intr(5.0, center1, 5.0, center2, epsilon),
                TwoIntersects {
                    point1: Vector2::new(4.0, -3.0),
                    point2: Vector2::new(4.0, 3.0)
                }
            );
        }
    }

    #[test]
    fn recovered_internal_tangency_uses_strict_position_boundary() {
        let outer_radius = 2.0;
        let inner_radius = 1.0;
        let epsilon = 1.0 / 1024.0;
        for (factor, intersects) in [(0.25, true), (0.99, true), (1.0, false), (1.01, false)] {
            let center_distance = outer_radius - inner_radius - factor * epsilon;
            let result = circle_circle_intr(
                outer_radius,
                Vector2::zero(),
                inner_radius,
                Vector2::new(center_distance, 0.0),
                epsilon,
            );
            if intersects {
                assert_case_eq!(
                    result,
                    TangentIntersect {
                        point: Vector2::new(
                            (outer_radius + center_distance + inner_radius) / 2.0,
                            0.0
                        )
                    }
                );
            } else {
                assert_case_eq!(result, NoIntersect::<f64>);
            }
        }
    }

    #[test]
    fn overlapping_circle_equality_has_strict_boundaries() {
        let epsilon = 1.0 / 1024.0;
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0 + 0.99 * epsilon,
                Vector2::zero(),
                epsilon,
            ),
            Overlapping::<f64>
        );
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0 + epsilon,
                Vector2::zero(),
                epsilon,
            ),
            NoIntersect::<f64>
        );
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0,
                Vector2::new(0.99 * epsilon, 0.0),
                epsilon,
            ),
            Overlapping::<f64>
        );
        assert!(matches!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0,
                Vector2::new(epsilon, 0.0),
                epsilon,
            ),
            TwoIntersects { .. }
        ));
    }

    #[test]
    fn recovered_internal_tangent_stays_bounded_for_nearly_equal_radii() {
        let epsilon = 1e-5f64;
        let center2 = Vector2::new(0.0000101, 0.0);
        let result = circle_circle_intr(1.0, Vector2::zero(), 0.99998, center2, epsilon);
        let expected = Vector2::new(0.99999505, 0.0);
        assert_case_eq!(result, TangentIntersect { point: expected });
        assert!((expected.length() - 1.0).abs() < epsilon);
        assert!(((expected - center2).length() - 0.99998).abs() < epsilon);
    }

    #[test]
    fn overlapping_circle_equality_bounds_combined_center_and_radius_error() {
        let epsilon = 1.0 / 1024.0;
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0 + epsilon / 4.0,
                Vector2::new(epsilon / 4.0, 0.0),
                epsilon,
            ),
            Overlapping::<f64>
        );
        let expected = TangentIntersect {
            point: Vector2::new(-1.0, 0.0),
        };
        assert_case_eq!(
            circle_circle_intr(
                1.0,
                Vector2::zero(),
                1.0 + epsilon / 2.0,
                Vector2::new(epsilon / 2.0, 0.0),
                epsilon,
            ),
            expected
        );
        assert_case_eq!(
            circle_circle_intr(
                1.0 + epsilon / 2.0,
                Vector2::new(epsilon / 2.0, 0.0),
                1.0,
                Vector2::zero(),
                epsilon,
            ),
            expected
        );
    }
}
