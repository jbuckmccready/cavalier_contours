use super::base_math::{min_max, quadratic_solutions};
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
/// This function returns the parametric solution(s) for the line segment equation `P(t) = p0 + t * (p1 - p0)`
/// for `t = 0` to `t = 1`. If `t < 0` or `t > 1` then intersect occurs only when extending the segment out past the
/// points `p0` and `p1` given. If `t < 0` then the intersect is nearest to `p0`, if `t > 1.0` then the intersect is nearest to `p1`).
/// Intersects are "sticky" and "snap" to tangent points using fuzzy comparisons, e.g. a segment very close to being tangent line
/// will return a single intersect point.
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
    // This function solves for t by substituting the parametric equations for the segment x = v1.X +
    // t * (v2.X - v1.X) and y = v1.Y + t * (v2.Y - v1.Y) for t = 0 to t = 1 into the circle equation
    // (x-h)^2 + (y-k)^2 = r^2 and then solving the resulting equation in the form a*t^2 + b*t + c = 0
    // using the quadratic formula

    use LineCircleIntr::*;

    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let h = circle_center.x;
    let k = circle_center.y;

    let a_quad = dx * dx + dy * dy;

    let eps = epsilon;

    if a_quad.fuzzy_eq_zero_eps(eps) {
        // p0 == p1, test if point is on the circle
        let xh = p0.x - h;
        let yk = p0.y - k;
        if (xh * xh + yk * yk).fuzzy_eq_eps(radius * radius, eps) {
            return TangentIntersect { t0: T::zero() };
        }

        return NoIntersect;
    }

    let b_quad = T::two() * (dx * (p0.x - h) + dy * (p0.y - k));

    let c_quad = (p0.x * p0.x - T::two() * h * p0.x + h * h)
        + (p0.y * p0.y - T::two() * k * p0.y + k * k)
        - radius * radius;

    let discriminant = b_quad * b_quad - T::four() * a_quad * c_quad;

    if discriminant.fuzzy_eq_zero_eps(eps) {
        // 1 solution (tangent line)
        return TangentIntersect {
            t0: -b_quad / (T::two() * a_quad),
        };
    }

    if discriminant < T::zero() {
        return NoIntersect;
    }

    let (sol1, sol2) = quadratic_solutions(a_quad, b_quad, c_quad, discriminant);

    let (t0, t1) = min_max(sol1, sol2);
    TwoIntersects { t0, t1 }
}
