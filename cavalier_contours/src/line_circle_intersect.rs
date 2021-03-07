use crate::{
    base_math::{min_max, quadratic_solutions},
    Real, Vector2,
};

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

    let a = dx * dx + dy * dy;
    if a.fuzzy_eq_zero() {
        // p0 == p1, test if point is on the circle
        let xh = p0.x - h;
        let yk = p0.y - k;
        if (xh * xh + yk * yk).fuzzy_eq(radius * radius) {
            return TangentIntersect { t0: T::zero() };
        }

        return NoIntersect;
    }

    let b = T::two() * (dx * (p0.x - h) + dy * (p0.y - k));
    let c = (p0.x * p0.x - T::two() * h * p0.x + h * h)
        + (p0.y * p0.y - T::two() * k * p0.y + k * k)
        - radius * radius;
    let discriminant = b * b - T::four() * a * c;

    if discriminant.fuzzy_eq_zero() {
        // 1 solution (tangent line)
        return TangentIntersect {
            t0: -b / (T::two() * a),
        };
    }

    if discriminant < T::zero() {
        return NoIntersect;
    }

    let (sol1, sol2) = quadratic_solutions(a, b, c, discriminant);

    let (t0, t1) = min_max(sol1, sol2);
    TwoIntersects { t0, t1 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base_traits::FuzzyEq;
    use LineCircleIntr::*;

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (NoIntersect, NoIntersect) => {}
                (TangentIntersect { t0: a1 }, TangentIntersect { t0: a2 }) if a1.fuzzy_eq(a2) => {}
                (TwoIntersects { t0: a1, t1: b1 }, TwoIntersects { t0: a2, t1: b2 })
                    if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            };
        };
    }

    #[test]
    fn no_intersect() {
        let p0 = Vector2::new(-1.0, -1.0);
        let p1 = Vector2::new(1.0, 1.0);
        let circle_center = Vector2::new(0.0, 5.0);
        let radius = 0.5;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_vertical() {
        let p0 = Vector2::new(0.0, -1.0);
        let p1 = Vector2::new(0.0, 1.0);
        let circle_center = Vector2::new(2.0, 0.0);
        let radius = 0.5;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_horizontal() {
        let p0 = Vector2::new(1.0, 1.0);
        let p1 = Vector2::new(3.0, 1.0);
        let circle_center = Vector2::new(2.0, -2.0);
        let radius = 0.5;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn two_intersects_true() {
        let p0 = Vector2::new(-1.0, -1.0);
        let p1 = Vector2::new(1.0, 1.0);
        // placing edge of circle at (0, 0)
        let radius = 0.5f64;
        let offset = (radius * radius / 2.0).sqrt();
        let circle_center = Vector2::new(offset, offset);
        let expected_t1_intr_point_x = 2.0 * offset;
        let expected_t1 = (expected_t1_intr_point_x - p0.x) / (p1.x - p0.x);
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(
            result,
            TwoIntersects {
                t0: 0.5,
                t1: expected_t1
            }
        );
    }

    #[test]
    fn two_intersects_seg_inside_vertical() {
        let p0 = Vector2::new(0.0, -1.0);
        let p1 = Vector2::new(0.0, 1.0);
        let circle_center = Vector2::new(0.0, 0.0);
        let radius = 1.0;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
    }

    #[test]
    fn two_intersects_seg_inside_horizontal() {
        let p0 = Vector2::new(-1.0, 0.0);
        let p1 = Vector2::new(1.0, 0.0);
        let circle_center = Vector2::new(0.0, 0.0);
        let radius = 1.0;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
    }

    #[test]
    fn two_intersects_seg_touching() {
        let p0 = Vector2::new(0.0, -1.0);
        let p1 = Vector2::new(0.0, 1.0);
        let circle_center = Vector2::new(0.0, 0.0);
        let radius = 1.0;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
    }

    #[test]
    fn tangent_intersect_vertical() {
        let p0 = Vector2::new(0.0, -1.0);
        let p1 = Vector2::new(0.0, 1.0);
        let circle_center = Vector2::new(1.0, 0.0);
        let radius = 1.0;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, TangentIntersect { t0: 0.5 });
    }

    #[test]
    fn tangent_intersect_horizontal() {
        let p0 = Vector2::new(-1.0, 0.0);
        let p1 = Vector2::new(1.0, 0.0);
        let circle_center = Vector2::new(0.0, -1.0);
        let radius = 1.0;
        let result = line_circle_intr(p0, p1, radius, circle_center);
        assert_case_eq!(result, TangentIntersect { t0: 0.5 });
    }
}
