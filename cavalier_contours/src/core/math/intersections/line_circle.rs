use super::{Circle, CurveIntersections, IntersectionPoint, Line};
use crate::core::traits::Real;

/// Intersects an infinite line and a full circle.
#[must_use]
#[inline(always)]
pub fn intersect_line_circle<T>(
    line: Line<T>,
    circle: Circle<T>,
    epsilon: T,
) -> CurveIntersections<T>
where
    T: Real,
{
    let direction_length_squared = line.direction.length_squared();
    if direction_length_squared == T::zero() {
        return CurveIntersections::None;
    }

    let center_offset = circle.center - line.point;
    let cross = line.direction.perp_dot(center_offset);
    let cross_squared = cross * cross;
    let radius_squared = circle.radius * circle.radius;
    let scaled_radius_squared = radius_squared * direction_length_squared;
    let inverse_direction_length_squared = T::one() / direction_length_squared;
    let midpoint_t = center_offset.dot(line.direction) * inverse_direction_length_squared;
    let midpoint = line.point + line.direction.scale(midpoint_t);

    if cross_squared >= scaled_radius_squared {
        let recovery_limit = circle.radius + epsilon;
        if cross_squared >= recovery_limit * recovery_limit * direction_length_squared {
            return CurveIntersections::None;
        }
        let line_distance = (cross_squared * inverse_direction_length_squared).sqrt();
        let radial_error = line_distance - circle.radius;
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            midpoint,
            radial_error * radial_error,
        ));
    }

    let half_chord_squared = radius_squared - cross_squared * inverse_direction_length_squared;
    if T::four() * half_chord_squared < epsilon * epsilon {
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            midpoint,
            half_chord_squared,
        ));
    }

    let t_offset = (half_chord_squared * inverse_direction_length_squared).sqrt();
    CurveIntersections::Two([
        IntersectionPoint::exact(line.point + line.direction.scale(midpoint_t - t_offset)),
        IntersectionPoint::exact(line.point + line.direction.scale(midpoint_t + t_offset)),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::Vector2;

    #[test]
    fn intersections_are_not_clipped_to_finite_bounds() {
        let result = intersect_line_circle(
            Line {
                point: Vector2::new(2.0, 0.0),
                direction: Vector2::new(1.0, 0.0),
            },
            Circle {
                center: Vector2::zero(),
                radius: 1.0,
            },
            1e-5,
        );
        let CurveIntersections::Two([first, second]) = result else {
            panic!("expected two intersections");
        };
        assert_eq!(first.point, Vector2::new(-1.0, 0.0));
        assert_eq!(second.point, Vector2::new(1.0, 0.0));
    }

    #[test]
    fn recovered_tangent_keeps_geometric_error() {
        let epsilon = 1e-5f64;
        let gap = epsilon / 2.0;
        let result = intersect_line_circle(
            Line {
                point: Vector2::new(-2.0, 0.0),
                direction: Vector2::new(4.0, 0.0),
            },
            Circle {
                center: Vector2::new(0.0, 1.0 + gap),
                radius: 1.0,
            },
            epsilon,
        );
        let CurveIntersections::Tangent(hit) = result else {
            panic!("expected a recovered tangent");
        };
        assert!((hit.error_squared - gap * gap).abs() < gap * gap * 1e-9);
    }
}
