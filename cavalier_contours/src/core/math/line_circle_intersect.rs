use super::Vector2;
use super::base_math::min_max;
use crate::core::math::parametric_from_point;
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
/// nearest to `p1`. Intersects are "sticky" and "snap" to tangent points using fuzzy comparisons.
/// Two intersect points that are fuzzy equal are returned as one tangent intersect. A line outside
/// the circle but within `epsilon` of its radius is also returned as a tangent intersect.
///
/// `epsilon` is a positional tolerance used for fuzzy comparisons.
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
    // This function solves for t by solving for cartesian intersect points via geometric
    // equations with the circle centered at (0, 0). Using the line equation of the form
    // Ax + By + C = 0 (taken from p1 and p0 shifted to the origin) and comparing with the circle
    // radius. The x, y cartesian points are then converted to parametric t representation using
    // p0 and p1.

    // This approach was found to be more numerically stable than solving for t using the quadratic
    // equations.

    use LineCircleIntr::{NoIntersect, TangentIntersect, TwoIntersects};

    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let h = circle_center.x;
    let k = circle_center.y;

    let eps = epsilon;

    if p0.fuzzy_eq_eps(p1, eps) {
        // p0 == p1, test if point is on the circle, using average of the points x and y values for
        // fuzziness
        let xh = (p0.x + p1.x) / T::two() - h;
        let yk = (p0.y + p1.y) / T::two() - k;
        if (xh * xh + yk * yk).sqrt().fuzzy_eq_eps(radius, eps) {
            return TangentIntersect { t0: T::zero() };
        }

        return NoIntersect;
    }

    let p0_shifted = p0 - circle_center;
    let p1_shifted = p1 - circle_center;

    // note: using Real number's defined epsilon for this check since it's just avoiding division by
    // too small a number, using the epsilon passed into this function causes unneeded loss of
    // precision (this branch is not directly determining the intersect result case returned)
    let (a, b, c) = if dx.fuzzy_eq_zero() {
        // vertical line, using average of point x values for fuzziness
        let x_pos = (p1_shifted.x + p0_shifted.x) / T::two();
        // x = x_pos
        // x - x_pos = 0

        // A = 1
        // B = 0
        // C = -x_pos
        (T::one(), T::zero(), -x_pos)
    } else {
        // (y - y1) = m(x - x1)
        // y - y1 = mx - mx1
        // mx - y + y1 - mx1 = 0

        // A = -m
        // B = 1.0
        // C = -y1 + m*x1

        // m = (y1 - y0) / (x1 - x0)

        let m = dy / dx;
        (m, -T::one(), p1_shifted.y - m * p1_shifted.x)
    };

    let a2 = a * a;
    let b2 = b * b;
    let c2 = c * c;
    let r2 = radius * radius;
    let a2_b2 = a2 + b2;

    // shortest distance from point on line to origin
    let shortest_dist = c.abs() / (a2_b2).sqrt();

    if shortest_dist > radius + eps {
        return NoIntersect;
    }

    let inv_a2_b2 = T::one() / a2_b2;
    // adding h and k back to solution terms (shifting from origin back to real coordinates)
    let x0 = -a * c * inv_a2_b2 + h;
    let y0 = -b * c * inv_a2_b2 + k;

    if shortest_dist >= radius {
        let t = parametric_from_point(p0, p1, Vector2::new(x0, y0), eps);
        return TangentIntersect { t0: t };
    }

    let d = r2 - c2 * inv_a2_b2;
    // Taking abs avoids NaN if round-off makes d slightly negative after shortest_dist compared
    // less than radius above.
    let mult = (d * inv_a2_b2).abs().sqrt();

    let point1 = Vector2::new(x0 + b * mult, y0 - a * mult);
    let point2 = Vector2::new(x0 - b * mult, y0 + a * mult);
    // Apply the positional epsilon to the intersection positions. Avoiding comparing shortest_dist
    // with the radius because that can merge points much farther apart than epsilon when the circle
    // radius is large.
    if point1.fuzzy_eq_eps(point2, eps) {
        let t = parametric_from_point(p0, p1, Vector2::new(x0, y0), eps);
        return TangentIntersect { t0: t };
    }

    let sol1 = parametric_from_point(p0, p1, point1, eps);
    let sol2 = parametric_from_point(p0, p1, point2, eps);
    let (t0, t1) = min_max(sol1, sol2);
    TwoIntersects { t0, t1 }
}
