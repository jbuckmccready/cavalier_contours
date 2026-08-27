use super::{CurveIntersections, IntersectionPoint, Line};
use crate::core::traits::Real;

/// Intersects two infinite lines.
#[must_use]
pub fn intersect_lines<T>(line1: Line<T>, line2: Line<T>, epsilon: T) -> CurveIntersections<T>
where
    T: Real,
{
    let direction_cross = line1.direction.perp_dot(line2.direction);
    let direction1_length_squared = line1.direction.length_squared();
    let direction2_length_squared = line2.direction.length_squared();
    if direction1_length_squared == T::zero() || direction2_length_squared == T::zero() {
        return CurveIntersections::None;
    }

    let delta = line1.point - line2.point;
    let angular_epsilon = T::fuzzy_epsilon();
    if direction_cross * direction_cross
        >= angular_epsilon * angular_epsilon * direction1_length_squared * direction2_length_squared
    {
        let line1_t = line2.direction.perp_dot(delta) / direction_cross;
        return CurveIntersections::One(IntersectionPoint::exact(
            line1.point + line1.direction.scale(line1_t),
        ));
    }

    let epsilon_squared = epsilon * epsilon;
    let line1_distance_numerator = line1.direction.perp_dot(delta);
    let line2_distance_numerator = line2.direction.perp_dot(delta);
    if line1_distance_numerator * line1_distance_numerator
        < epsilon_squared * direction1_length_squared
        && line2_distance_numerator * line2_distance_numerator
            < epsilon_squared * direction2_length_squared
    {
        CurveIntersections::Coincident
    } else {
        CurveIntersections::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::Vector2;

    #[test]
    fn intersection_is_not_clipped_to_finite_bounds() {
        let result = intersect_lines(
            Line {
                point: Vector2::zero(),
                direction: Vector2::new(1.0, 0.0),
            },
            Line {
                point: Vector2::new(2.0, -1.0),
                direction: Vector2::new(0.0, 1.0),
            },
            1e-5,
        );
        let CurveIntersections::One(hit) = result else {
            panic!("expected one intersection");
        };
        assert_eq!(hit.point, Vector2::new(2.0, 0.0));
    }

    #[test]
    fn coincident_check_uses_strict_position_epsilon() {
        let first = Line {
            point: Vector2::zero(),
            direction: Vector2::new(2.0, 0.0),
        };
        for (offset, coincident) in [(0.5e-5, true), (1e-5, false)] {
            let second = Line {
                point: Vector2::new(0.0, offset),
                direction: Vector2::new(1.0, 0.0),
            };
            assert_eq!(
                matches!(
                    intersect_lines(first, second, 1e-5),
                    CurveIntersections::Coincident
                ),
                coincident
            );
        }
    }
}
