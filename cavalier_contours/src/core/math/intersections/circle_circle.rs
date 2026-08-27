use super::{Circle, CurveIntersections, IntersectionPoint};
use crate::core::{math::Vector2, traits::Real};

/// Intersects two full circles.
#[must_use]
pub fn intersect_circles<T>(
    circle1: Circle<T>,
    circle2: Circle<T>,
    epsilon: T,
) -> CurveIntersections<T>
where
    T: Real,
{
    let center_delta = circle2.center - circle1.center;
    let center_distance_squared = center_delta.dot(center_delta);
    let half = T::one() / T::two();
    let radius_difference = (circle1.radius - circle2.radius).abs();

    if radius_difference < epsilon {
        let remaining_epsilon = epsilon - radius_difference;
        if center_distance_squared < remaining_epsilon * remaining_epsilon {
            return CurveIntersections::Coincident;
        }
    }

    if center_distance_squared == T::zero() {
        return CurveIntersections::None;
    }

    let external_limit = circle1.radius + circle2.radius;
    if center_distance_squared >= external_limit * external_limit {
        let reject_limit = external_limit + epsilon;
        if center_distance_squared >= reject_limit * reject_limit {
            return CurveIntersections::None;
        }
        let center_distance = center_distance_squared.sqrt();
        let gap = center_distance - external_limit;
        let point = circle1.center
            + center_delta.scale(
                (circle1.radius + center_distance - circle2.radius) / (T::two() * center_distance),
            );
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            point,
            gap * gap / T::two(),
        ));
    }

    if center_distance_squared <= radius_difference * radius_difference {
        if radius_difference >= epsilon {
            let reject_limit = radius_difference - epsilon;
            if center_distance_squared <= reject_limit * reject_limit {
                return CurveIntersections::None;
            }
        }
        if center_distance_squared == radius_difference * radius_difference {
            let distance = if circle1.radius > circle2.radius {
                circle1.radius
            } else {
                -circle1.radius
            };
            return CurveIntersections::Tangent(IntersectionPoint::exact(
                circle1.center + center_delta.scale(distance / radius_difference),
            ));
        }
        let center_distance = center_distance_squared.sqrt();
        let gap = radius_difference - center_distance;
        let half_gap = gap * half;
        let distance = if circle1.radius > circle2.radius {
            circle1.radius - half_gap
        } else {
            -circle1.radius - half_gap
        };
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            circle1.center + center_delta.scale(distance / center_distance),
            gap * half_gap,
        ));
    }

    let radius1_squared = circle1.radius * circle1.radius;
    let a_numerator = radius1_squared - circle2.radius * circle2.radius + center_distance_squared;
    let a_over_distance = a_numerator * half / center_distance_squared;
    let midpoint = circle1.center + center_delta.scale(a_over_distance);
    let a_squared = a_numerator * a_over_distance * half;
    let half_chord_squared = radius1_squared - a_squared;

    if half_chord_squared <= T::zero() {
        let center_distance = center_distance_squared.sqrt();
        let a = a_over_distance * center_distance;
        let radius1_error = a.abs() - circle1.radius;
        let radius2_error = (center_distance - a).abs() - circle2.radius;
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            midpoint,
            radius1_error * radius1_error + radius2_error * radius2_error,
        ));
    }

    if T::four() * half_chord_squared < epsilon * epsilon {
        return CurveIntersections::Tangent(IntersectionPoint::adjusted(
            midpoint,
            half_chord_squared,
        ));
    }

    let h_over_distance = (half_chord_squared / center_distance_squared).sqrt();
    let x_term = h_over_distance * center_delta.y;
    let y_term = h_over_distance * center_delta.x;
    CurveIntersections::Two([
        IntersectionPoint::exact(Vector2::new(midpoint.x + x_term, midpoint.y - y_term)),
        IntersectionPoint::exact(Vector2::new(midpoint.x - x_term, midpoint.y + y_term)),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returns_two_whole_circle_intersections() {
        let result = intersect_circles(
            Circle {
                center: Vector2::zero(),
                radius: 1.0,
            },
            Circle {
                center: Vector2::new(1.0, 0.0),
                radius: 1.0,
            },
            1e-5,
        );
        let CurveIntersections::Two([first, second]) = result else {
            panic!("expected two intersections");
        };
        assert!(first.point.y < 0.0);
        assert!(second.point.y > 0.0);
    }

    #[test]
    fn coincident_check_includes_center_and_radius_error() {
        let epsilon = 1e-5;
        let result = intersect_circles(
            Circle {
                center: Vector2::zero(),
                radius: 1.0,
            },
            Circle {
                center: Vector2::new(epsilon / 4.0, 0.0),
                radius: 1.0 + epsilon / 4.0,
            },
            epsilon,
        );
        assert!(matches!(result, CurveIntersections::Coincident));
    }
}
