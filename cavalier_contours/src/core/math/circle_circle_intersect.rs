use super::Vector2;
use crate::core::traits::Real;

/// Holds the result of finding the intersect between a line segment and a circle.
#[derive(Debug, Copy, Clone)]
pub enum CircleCircleIntr<T>
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
    /// Simple case of two intersect points found.
    TwoIntersects {
        /// Holds the first intersect point.
        point1: Vector2<T>,
        /// Holds the second intersect point.
        point2: Vector2<T>,
    },
    /// Circles overlap each other (same circle).
    Overlapping,
}

/// Finds the intersects between two circles.
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
    use CircleCircleIntr::*;

    let cv = center2 - center1;
    let d2 = cv.dot(cv);
    let d = d2.sqrt();

    let eps = epsilon;

    if d.fuzzy_eq_zero_eps(eps) {
        // same center position
        if radius1.fuzzy_eq_eps(radius2, eps) {
            return Overlapping;
        }
        return NoIntersect;
    }

    // different center position
    if !d.fuzzy_lt_eps(radius1 + radius2, eps) || !d.fuzzy_gt_eps((radius1 - radius2).abs(), eps) {
        // distance relative to radii is too large or too small for intersects to occur
        return NoIntersect;
    }

    let rad1_sq = radius1 * radius1;
    let a = (rad1_sq - radius2 * radius2 + d2) / (T::two() * d);
    let midpoint = center1 + cv.scale(a / d);
    let diff = rad1_sq - a * a;

    if diff < T::zero() {
        return TangentIntersect { point: midpoint };
    }

    let h = diff.sqrt();
    let h_over_d = h / d;
    let x_term = h_over_d * cv.y;
    let y_term = h_over_d * cv.x;

    let pt1 = Vector2::new(midpoint.x + x_term, midpoint.y - y_term);
    let pt2 = Vector2::new(midpoint.x - x_term, midpoint.y + y_term);

    if pt1.fuzzy_eq_eps(pt2, eps) {
        return TangentIntersect { point: pt1 };
    }

    TwoIntersects {
        point1: pt1,
        point2: pt2,
    }
}
