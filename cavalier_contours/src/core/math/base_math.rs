use super::Vector2;
use crate::core::traits::Real;

/// Returns the (min, max) values from `v1` and `v2`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// let (min_val, max_val) = min_max(8, 4);
/// assert_eq!(min_val, 4);
/// assert_eq!(max_val, 8);
/// ```
#[inline]
pub fn min_max<T>(v1: T, v2: T) -> (T, T)
where
    T: PartialOrd,
{
    if v1 < v2 {
        (v1, v2)
    } else {
        (v2, v1)
    }
}

/// Normalize radians to be between `0` and `2PI`, e.g. `-PI/4` becomes `7PI/4` and `5PI` becomes
/// `PI`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::core::traits::*;
/// use std::f64::consts::PI;
/// assert!(normalize_radians(5.0 * PI).fuzzy_eq(PI));
/// assert!(normalize_radians(-PI / 4.0).fuzzy_eq(7.0 * PI / 4.0));
/// // anything between 0 and 2PI inclusive is left unchanged
/// assert!(normalize_radians(0.0).fuzzy_eq(0.0));
/// assert!(normalize_radians(PI).fuzzy_eq(PI));
/// assert!(normalize_radians(2.0 * PI).fuzzy_eq(2.0 * PI));
/// ```
#[inline]
pub fn normalize_radians<T>(angle: T) -> T
where
    T: Real,
{
    if angle >= T::zero() && angle <= T::tau() {
        return angle;
    }

    angle - (angle / T::tau()).floor() * T::tau()
}

/// Returns the smaller difference between two angles.
///
/// Result is negative if `normalize_radians(angle2 - angle1) > PI`. See [normalize_radians] for
/// more information.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::core::traits::*;
/// use std::f64::consts::PI;
/// assert!(delta_angle(5.0 * PI, 5.0 * PI).fuzzy_eq(0.0));
/// // note here the return is positive in both cases (since there is PI difference)
/// assert!(delta_angle(4.0 * PI, 5.0 * PI).fuzzy_eq(PI));
/// assert!(delta_angle(5.0 * PI, 4.0 * PI).fuzzy_eq(PI));
/// // these cases show when the order can change the sign
/// assert!(delta_angle(0.5 * PI, 0.25 * PI).fuzzy_eq(-0.25 * PI));
/// assert!(delta_angle(0.25 * PI, 0.5 * PI).fuzzy_eq(0.25 * PI));
/// ```
#[inline]
pub fn delta_angle<T>(angle1: T, angle2: T) -> T
where
    T: Real,
{
    let mut diff = normalize_radians(angle2 - angle1);
    if diff > T::pi() {
        diff = diff - T::tau();
    }

    diff
}

/// Returns the smaller difference between two angles and applies the sign given.
///
/// This function is similar to [delta_angle] but always returns a negative result if `negative` is
/// true or a positive result if `negative` is false. This is useful for ensuring a particular
/// polarity for edge cases, e.g. if `angle1` is 0 and `angle2` is PI then the delta angle could be
/// be considered positive or negative ([delta_angle] always returns positive).
///
#[inline]
pub fn delta_angle_signed<T>(angle1: T, angle2: T, negative: bool) -> T
where
    T: Real,
{
    let diff = delta_angle(angle1, angle2);
    if negative {
        -diff.abs()
    } else {
        diff.abs()
    }
}

/// Tests if `test_angle` is between a `start_angle` and `end_angle`.
///
/// Test assumes counter clockwise `start_angle` to `end_angle`, and is inclusive using `epsilon`.
/// See [angle_is_between] function to use default fuzzy epsilon.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// use std::f64::consts::PI;
/// assert!(angle_is_between_eps(PI / 2.0, 0.0, PI, 1e-5));
/// assert!(angle_is_between_eps(0.0, 0.0, PI, 1e-5));
/// assert!(angle_is_between_eps(PI, 0.0, PI, 1e-5));
/// // note: always calculated as going counter clockwise
/// // going from PI to PI / 2 counter clockwise sweeps 0.0
/// assert!(angle_is_between_eps(0.0, PI, PI / 2.0, 1e-5));
/// ```
#[inline]
pub fn angle_is_between_eps<T>(test_angle: T, start_angle: T, end_angle: T, epsilon: T) -> bool
where
    T: Real,
{
    let end_sweep = normalize_radians(end_angle - start_angle);
    let mid_sweep = normalize_radians(test_angle - start_angle);

    mid_sweep < end_sweep + epsilon
}

/// Same as [angle_is_between_eps] using default epsilon.
///
/// Default epsilon is [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon)
/// from [FuzzyEq](crate::core::traits::FuzzyEq) trait.
#[inline]
pub fn angle_is_between<T>(test_angle: T, start_angle: T, end_angle: T) -> bool
where
    T: Real,
{
    angle_is_between_eps(test_angle, start_angle, end_angle, T::fuzzy_epsilon())
}

/// Tests if `test_angle` is within the `sweep_angle` starting at `start_angle`.
///
/// If `sweep_angle` is positive then sweep is counter clockwise, otherwise it is clockwise.
/// `epsilon` controls the fuzzy inclusion.
#[inline]
pub fn angle_is_within_sweep_eps<T>(
    test_angle: T,
    start_angle: T,
    sweep_angle: T,
    epsilon: T,
) -> bool
where
    T: Real,
{
    let end_angle = start_angle + sweep_angle;
    if sweep_angle < T::zero() {
        return angle_is_between_eps(test_angle, end_angle, start_angle, epsilon);
    }

    angle_is_between_eps(test_angle, start_angle, end_angle, epsilon)
}

/// Same as [angle_is_within_sweep_eps] using default epsilon.
///
/// Default epsilon is [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon)
/// from [FuzzyEq](crate::core::traits::FuzzyEq) trait.
#[inline]
pub fn angle_is_within_sweep<T>(test_angle: T, start_angle: T, sweep_angle: T) -> bool
where
    T: Real,
{
    angle_is_within_sweep_eps(test_angle, start_angle, sweep_angle, T::fuzzy_epsilon())
}

/// Returns the solutions to the quadratic equation.
///
/// Quadratic equation is `-b +/- sqrt(b * b - 4 * a * c) / (2 * a)`.
/// With the `sqrt_discriminant` defined as `sqrt(b * b - 4 * a * c)`.
///
/// The purpose of this function is to minimize error in the process of finding solutions
/// to the quadratic equation.
#[inline]
pub fn quadratic_solutions<T>(a: T, b: T, c: T, sqrt_discriminant: T) -> (T, T)
where
    T: Real,
{
    debug_assert!(
        (b * b - T::four() * a * c)
            .sqrt()
            .fuzzy_eq(sqrt_discriminant),
        "discriminant is not valid"
    );
    // Avoids loss in precision due to taking the difference of two floating point values that are
    // very near each other in value.
    // https://math.stackexchange.com/questions/311382/solving-a-quadratic-equation-with-precision-when-using-floating-point-variables
    let denom = T::two() * a;
    let sol1 = if b < T::zero() {
        (-b + sqrt_discriminant) / denom
    } else {
        (-b - sqrt_discriminant) / denom
    };

    let sol2 = (c / a) / sol1;

    (sol1, sol2)
}

/// Distance squared between the points `p0` and `p1`.
#[inline]
pub fn dist_squared<T>(p0: Vector2<T>, p1: Vector2<T>) -> T
where
    T: Real,
{
    let d = p0 - p1;
    d.dot(d)
}

/// Angle of the direction vector described by `p0` to `p1`.
#[inline]
pub fn angle<T>(p0: Vector2<T>, p1: Vector2<T>) -> T
where
    T: Real,
{
    T::atan2(p1.y - p0.y, p1.x - p0.x)
}

/// Midpoint of a line segment defined by `p0` to `p1`.
#[inline]
pub fn midpoint<T>(p0: Vector2<T>, p1: Vector2<T>) -> Vector2<T>
where
    T: Real,
{
    Vector2::new((p0.x + p1.x) / T::two(), (p0.y + p1.y) / T::two())
}

/// Returns the point on the circle with `radius`, `center`, and polar `angle` in radians given.
#[inline]
pub fn point_on_circle<T>(radius: T, center: Vector2<T>, angle: T) -> Vector2<T>
where
    T: Real,
{
    let (s, c) = angle.sin_cos();
    Vector2::new(center.x + radius * c, center.y + radius * s)
}

/// Returns the point on the line segment going from `p0` to `p1` at parametric value `t`.
#[inline]
pub fn point_from_parametric<T>(p0: Vector2<T>, p1: Vector2<T>, t: T) -> Vector2<T>
where
    T: Real,
{
    p0 + (p1 - p0).scale(t)
}

/// Returns the parametric value on the line segment going from `p0` to `p1` at the `point` given.
///
/// Note this function assumes the `point` is on the line and properly handles the cases of vertical
/// and horizontal lines by using the `epsilon` parameter to fuzzy compare for when `p0.x == p1.x`.
#[inline]
pub fn parametric_from_point<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>, epsilon: T) -> T
where
    T: Real,
{
    if p0.x.fuzzy_eq_eps(p1.x, epsilon) {
        // vertical segment, use y coordinate
        debug_assert!(
            point.x.fuzzy_eq_eps(p0.x, epsilon),
            "point does not lie on the line defined by p0 to p1"
        );
        (point.y - p0.y) / (p1.y - p0.y)
    } else {
        // use x coordinate
        debug_assert!(
            point.fuzzy_eq_eps(p0, epsilon)
                || ((point.y - p0.y) / (point.x - p0.x))
                    .fuzzy_eq_eps((p1.y - p0.y) / (p1.x - p0.x), epsilon),
            "point does not lie on the line defined by p0 to p1"
        );
        (point.x - p0.x) / (p1.x - p0.x)
    }
}

/// Returns the closest point on the line segment from `p0` to `p1` to the `point` given.
#[inline]
pub fn line_seg_closest_point<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> Vector2<T>
where
    T: Real,
{
    // Dot product used to find angles
    // See: http://geomalgorithms.com/a02-_lines.html
    let v = p1 - p0;
    let w = point - p0;
    let c1 = w.dot(v);
    if c1 < T::fuzzy_epsilon() {
        return p0;
    }

    let c2 = v.length_squared();
    if c2 < c1 + T::fuzzy_epsilon() {
        return p1;
    }

    let b = c1 / c2;
    p0 + v.scale(b)
}

/// Helper function to avoid repeating code for is_left and is_right checks.
#[inline]
fn perp_dot_test_value<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> T
where
    T: Real,
{
    (p1.x - p0.x) * (point.y - p0.y) - (p1.y - p0.y) * (point.x - p0.x)
}

/// Returns true if `point` is left of a direction vector.
///
/// Direction vector is defined as `p1 - p0`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// let p0 = Vector2::new(1.0, 1.0);
/// let p1 = Vector2::new(2.0, 2.0);
/// assert!(is_left(p0, p1, Vector2::new(0.0, 1.0)));
/// assert!(!is_left(p0, p1, Vector2::new(1.0, 0.0)));
/// ```
#[inline]
pub fn is_left<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> bool
where
    T: Real,
{
    perp_dot_test_value(p0, p1, point) > T::zero()
}

/// Same as [is_left] but uses <= operator rather than < for boundary inclusion.
#[inline]
pub fn is_left_or_equal<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> bool
where
    T: Real,
{
    perp_dot_test_value(p0, p1, point) >= T::zero()
}

/// Returns true if `point` is left of a direction vector with fuzzy inclusion.
///
/// Returns true if point is left or fuzzy coincident with the
/// direction vector defined by `p1 - p0`.
///
/// `epsilon` controls the fuzzy compare.
#[inline]
pub fn is_left_or_coincident_eps<T>(
    p0: Vector2<T>,
    p1: Vector2<T>,
    point: Vector2<T>,
    epsilon: T,
) -> bool
where
    T: Real,
{
    debug_assert!(epsilon > T::zero());
    perp_dot_test_value(p0, p1, point) > -epsilon
}

/// Same as [is_left_or_coincident_eps] using default epsilon.
///
/// Default epsilon is [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon)
/// from [FuzzyEq](crate::core::traits::FuzzyEq) trait.
#[inline]
pub fn is_left_or_coincident<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> bool
where
    T: Real,
{
    is_left_or_coincident_eps(p0, p1, point, T::fuzzy_epsilon())
}

/// Returns true if `point` is right of a direction vector with fuzzy inclusion.
///
/// Returns true if point is right or fuzzy coincident with the
/// direction vector defined by `p1 - p0`.
///
/// `epsilon` controls the fuzzy compare.
#[inline]
pub fn is_right_or_coincident_eps<T>(
    p0: Vector2<T>,
    p1: Vector2<T>,
    point: Vector2<T>,
    epsilon: T,
) -> bool
where
    T: Real,
{
    debug_assert!(epsilon > T::zero());
    perp_dot_test_value(p0, p1, point) < epsilon
}

/// Same as [is_right_or_coincident_eps] using default epsilon.
///
/// Default epsilon is [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon)
/// from [FuzzyEq](crate::core::traits::FuzzyEq) trait.
#[inline]
pub fn is_right_or_coincident<T>(p0: Vector2<T>, p1: Vector2<T>, point: Vector2<T>) -> bool
where
    T: Real,
{
    is_right_or_coincident_eps(p0, p1, point, T::fuzzy_epsilon())
}

/// Test if a `point` is within a arc sweep angle region.
///
/// Arc is defined by `center`, `arc_start`, `arc_end`, and arc direction parameter `is_clockwise`.
/// The angle region is defined as if the arc had infinite radius projected outward in a cone.
///
/// This function uses the default epsilon of
/// [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon) from
/// [FuzzyEq](crate::core::traits::FuzzyEq) trait.
///
/// # Examples
/// ```
/// # use cavalier_contours::core::math::*;
/// // defining an arc that projects an angle region covering all of
/// // quadrant I (x positive, y positive space)
/// let arc_center = Vector2::new(0.0, 0.0);
/// let arc_start = Vector2::new(1.0, 0.0);
/// let arc_end = Vector2::new(0.0, 1.0);
/// assert!(point_within_arc_sweep(arc_center, arc_start, arc_end, false, Vector2::new(1.0, 1.0)));
/// // check is fuzzy inclusive
/// assert!(point_within_arc_sweep(arc_center, arc_start, arc_end, false, Vector2::new(1.0, 0.0)));
/// assert!(point_within_arc_sweep(arc_center, arc_start, arc_end, false, Vector2::new(0.0, 1.0)));
/// ```
#[inline]
pub fn point_within_arc_sweep<T>(
    center: Vector2<T>,
    arc_start: Vector2<T>,
    arc_end: Vector2<T>,
    is_clockwise: bool,
    point: Vector2<T>,
) -> bool
where
    T: Real,
{
    if is_clockwise {
        is_right_or_coincident(center, arc_start, point)
            && is_left_or_coincident(center, arc_end, point)
    } else {
        is_left_or_coincident(center, arc_start, point)
            && is_right_or_coincident(center, arc_end, point)
    }
}

/// Returns the bulge for the given arc `sweep_angle`.
///
/// By definition `bulge = tan(arc_sweep_angle / 4)`.
/// Note if `angle` is negative then bulge returned will be negative (clockwise arc).
#[inline]
pub fn bulge_from_angle<T>(angle: T) -> T
where
    T: Real,
{
    (angle / T::four()).tan()
}

/// Returns the arc sweep angle for the given `bulge`.
///
/// By definition `arc_sweep_angle = 4 * atan(bulge)`.
/// Note if `bulge` is negative then angle returned will be negative (clockwise arc).
#[inline]
pub fn angle_from_bulge<T>(bulge: T) -> T
where
    T: Real,
{
    T::four() * bulge.atan()
}
