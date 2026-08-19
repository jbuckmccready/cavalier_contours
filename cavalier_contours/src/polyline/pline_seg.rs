use super::PlineVertex;
use crate::core::{
    math::{
        Vector2, angle, angle_from_bulge, arc_sweep_extents, delta_angle_signed, dist_squared,
        line_seg_closest_point, midpoint, min_max, point_within_arc_sweep,
    },
    traits::Real,
};
use static_aabb2d_index::AABB;

/// Get the arc radius and center of an arc polyline segment defined by `v1` to `v2`.
/// Behavior undefined (may panic or return without error) if v1.bulge is zero.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::polyline::*;
/// # use cavalier_contours::core::traits::*;
/// # use cavalier_contours::core::math::*;
/// // arc half circle arc segment going from (0, 0) to (1, 0) counter clockwise
/// let v1 = PlineVertex::new(0.0, 0.0, 1.0);
/// let v2 = PlineVertex::new(1.0, 0.0, 0.0);
/// let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
/// assert!(arc_radius.fuzzy_eq(0.5));
/// assert!(arc_center.fuzzy_eq(Vector2::new(0.5, 0.0)));
///```
pub fn seg_arc_radius_and_center<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> (T, Vector2<T>)
where
    T: Real,
{
    debug_assert!(!v1.bulge_is_zero(), "v1 to v2 must be an arc");
    debug_assert!(!v1.pos().fuzzy_eq(v2.pos()), "v1 must not be on top of v2");

    let bulge = v1.bulge;
    let bulge_squared = bulge * bulge;
    let chord = v2.pos() - v1.pos();
    let inv_four_bulge = T::one() / (T::four() * bulge);
    let radius = chord.length() * (T::one() + bulge_squared) * inv_four_bulge.abs();
    let center = v1.pos()
        + chord.scale(T::one() / T::two())
        + chord
            .perp()
            .scale((T::one() - bulge) * (T::one() + bulge) * inv_four_bulge);

    (radius, center)
}

/// Result from splitting a segment using [`seg_split_at_point`].
#[derive(Debug, Copy, Clone)]
pub struct SplitResult<T = f64>
where
    T: Real,
{
    /// Updated start vertex (has same position as start of segment but with updated bulge value).
    pub updated_start: PlineVertex<T>,
    /// Vertex at split point (position is equal to split point, bulge set to maintain same curve to
    /// the next vertex).
    pub split_vertex: PlineVertex<T>,
}

/// Splits a polyline segment defined by `v1` to `v2` at the `point_on_seg` given. Assumes the `point_on_seg` lies on the segment.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::polyline::*;
/// // arc half circle arc segment going from (0, 0) to (1, 0) counter clockwise
/// let v1 = PlineVertex::new(0.0, 0.0, 1.0);
/// let v2 = PlineVertex::new(1.0, 0.0, 0.0);
/// let point = Vector2::new(0.5, -0.5);
/// let SplitResult { updated_start, split_vertex } = seg_split_at_point(v1, v2, point, 1e-5);
/// let quarter_circle_bulge = (std::f64::consts::PI / 8.0).tan();
/// assert!(updated_start.fuzzy_eq(PlineVertex::new(v1.x, v1.y, quarter_circle_bulge)));
/// assert!(split_vertex.fuzzy_eq(PlineVertex::new(point.x, point.y, quarter_circle_bulge)));
/// ```
pub fn seg_split_at_point<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    point_on_seg: Vector2<T>,
    pos_equal_eps: T,
) -> SplitResult<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        // v1->v2 is a line segment, just use point as end point
        let updated_start = v1;
        let split_vertex = PlineVertex::new(point_on_seg.x, point_on_seg.y, T::zero());
        return SplitResult {
            updated_start,
            split_vertex,
        };
    }

    if v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps)
        || v1.pos().fuzzy_eq_eps(point_on_seg, pos_equal_eps)
    {
        // v1 == v2 or v1 == point, updated_start is put on top of split_vertex
        let updated_start = PlineVertex::new(point_on_seg.x, point_on_seg.y, T::zero());
        let split_vertex = PlineVertex::new(point_on_seg.x, point_on_seg.y, v1.bulge);
        return SplitResult {
            updated_start,
            split_vertex,
        };
    }

    if v2.pos().fuzzy_eq_eps(point_on_seg, pos_equal_eps) {
        // point is at end point of segment
        let updated_start = v1;
        let split_vertex = PlineVertex::new(v2.x, v2.y, T::zero());
        return SplitResult {
            updated_start,
            split_vertex,
        };
    }

    let abs_bulge = v1.bulge.abs();
    let chord_length_squared = (v2.pos() - v1.pos()).length_squared();
    let diameter_ratio = T::two() * abs_bulge / (abs_bulge * abs_bulge + T::one());
    let diameter_ratio_squared = diameter_ratio * diameter_ratio;

    // chord / diameter = sin(sweep / 2), and tan(sweep / 4) can then be found
    // from the half-angle identity. Compute the smaller subarc this way because
    // chord length becomes ill-conditioned as the sweep approaches a half circle.
    // Derive the larger subarc with the tangent subtraction identity instead.
    let bulge_from_chord_length_squared = |sub_chord_length_squared: T| {
        let sin_half_sweep_squared =
            sub_chord_length_squared / chord_length_squared * diameter_ratio_squared;
        let sin_half_sweep = if sin_half_sweep_squared > T::one() {
            T::one()
        } else {
            sin_half_sweep_squared.sqrt()
        };
        let cos_half_sweep = ((T::one() - sin_half_sweep) * (T::one() + sin_half_sweep)).sqrt();
        let magnitude = sin_half_sweep / (T::one() + cos_half_sweep);
        if v1.bulge_is_neg() {
            -magnitude
        } else {
            magnitude
        }
    };

    let chord1_length_squared = (point_on_seg - v1.pos()).length_squared();
    let chord2_length_squared = (v2.pos() - point_on_seg).length_squared();
    let (bulge1, bulge2) = if chord1_length_squared <= chord2_length_squared {
        let bulge1 = bulge_from_chord_length_squared(chord1_length_squared);
        let bulge2 = (v1.bulge - bulge1) / (T::one() + v1.bulge * bulge1);
        (bulge1, bulge2)
    } else {
        let bulge2 = bulge_from_chord_length_squared(chord2_length_squared);
        let bulge1 = (v1.bulge - bulge2) / (T::one() + v1.bulge * bulge2);
        (bulge1, bulge2)
    };

    let updated_start = PlineVertex::new(v1.x, v1.y, bulge1);
    let split_vertex = PlineVertex::new(point_on_seg.x, point_on_seg.y, bulge2);

    SplitResult {
        updated_start,
        split_vertex,
    }
}

/// Find the tangent direction vector (*NOT* normalized) on a polyline segment defined by `v1` to
/// `v2` at `point_on_seg`.
///
/// Note: The vector returned is just the direction vector, add the `point_on_seg` position if
/// you need to offset from that position.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::polyline::*;
/// # use cavalier_contours::core::math::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// let midpoint = Vector2::new(3.0, 1.0);
/// assert!(seg_tangent_vector(v1, v2, midpoint).normalize().fuzzy_eq(Vector2::new(1.0, 0.0)));
/// assert!(seg_tangent_vector(v1, v2, v1.pos()).normalize().fuzzy_eq(Vector2::new(0.0, -1.0)));
/// assert!(seg_tangent_vector(v1, v2, v2.pos()).normalize().fuzzy_eq(Vector2::new(0.0, 1.0)));
/// ```
pub fn seg_tangent_vector<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    point_on_seg: Vector2<T>,
) -> Vector2<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        return v2.pos() - v1.pos();
    }

    let bulge = v1.bulge;
    let chord = v2.pos() - v1.pos();
    let point_offset = (point_on_seg - v1.pos()).scale(T::two()) - chord;
    let numerator = chord.scale((T::one() - bulge) * (T::one() + bulge))
        + point_offset.perp().scale(T::two() * bulge);
    numerator.scale(T::one() / (T::four() * bulge.abs()))
}

/// Find the closest point on a polyline segment defined by `v1` to `v2` to `point` given.
/// If there are multiple closest points then one is chosen (which is chosen is not defined).
///
/// `epsilon` is used for fuzzy float comparisons.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::polyline::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// assert!(
///     seg_closest_point(v1, v2, Vector2::new(3.0, 0.0), 1e-5).fuzzy_eq(Vector2::new(3.0, 1.0))
/// );
/// assert!(
///     seg_closest_point(v1, v2, Vector2::new(3.0, 1.2), 1e-5).fuzzy_eq(Vector2::new(3.0, 1.0))
/// );
/// assert!(seg_closest_point(v1, v2, v1.pos(), 1e-5).fuzzy_eq(v1.pos()));
/// assert!(seg_closest_point(v1, v2, v2.pos(), 1e-5).fuzzy_eq(v2.pos()));
/// ```
pub fn seg_closest_point<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    point: Vector2<T>,
    epsilon: T,
) -> Vector2<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        return line_seg_closest_point(v1.pos(), v2.pos(), point);
    }

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    if point.fuzzy_eq_eps(arc_center, epsilon) {
        // avoid normalizing zero length vector (point is at center, just return start point)
        return v1.pos();
    }

    if point_within_arc_sweep(
        arc_center,
        v1.pos(),
        v2.pos(),
        v1.bulge_is_neg(),
        point,
        epsilon,
    ) {
        // closest point is on the arc
        let v_to_point = (point - arc_center).normalize();
        return v_to_point.scale(arc_radius) + arc_center;
    }

    // closest point is one of the ends
    let dist1 = dist_squared(v1.pos(), point);
    let dist2 = dist_squared(v2.pos(), point);
    if dist1 < dist2 {
        return v1.pos();
    }

    v2.pos()
}

/// Returns the axis-aligned bounding box of a line segment.
#[inline]
pub(crate) fn line_seg_bounding_box<T>(start: Vector2<T>, end: Vector2<T>) -> AABB<T>
where
    T: Real,
{
    let (min_x, max_x) = min_max(start.x, end.x);
    let (min_y, max_y) = min_max(start.y, end.y);
    AABB::new(min_x, min_y, max_x, max_y)
}

/// Computes a fast approximate axis aligned bounding box of a polyline segment defined by `v1` to `v2`.
///
/// The bounding box may be larger than the true bounding box for the segment (but is never smaller).
/// For the true axis aligned bounding box use [`seg_bounding_box`] but this function is faster for arc
/// segments.
pub fn seg_fast_approx_bounding_box<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> AABB<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        return line_seg_bounding_box(v1.pos(), v2.pos());
    }

    // For arcs we don't compute the actual extents which is slower, instead we create an approximate
    // bounding box from the rectangle formed by extending the chord by the sagitta, note this
    // approximate bounding box is always equal to or bigger than the true bounding box
    let b = v1.bulge;
    let offs_x = b * (v2.y - v1.y) / T::two();
    let offs_y = -b * (v2.x - v1.x) / T::two();

    let (pt_x_min, pt_x_max) = min_max(v1.x + offs_x, v2.x + offs_x);
    let (pt_y_min, pt_y_max) = min_max(v1.y + offs_y, v2.y + offs_y);

    let (end_point_x_min, end_point_x_max) = min_max(v1.x, v2.x);
    let (end_point_y_min, end_point_y_max) = min_max(v1.y, v2.y);

    let min_x = num_traits::real::Real::min(end_point_x_min, pt_x_min);
    let min_y = num_traits::real::Real::min(end_point_y_min, pt_y_min);
    let max_x = num_traits::real::Real::max(end_point_x_max, pt_x_max);
    let max_y = num_traits::real::Real::max(end_point_y_max, pt_y_max);

    AABB::new(min_x, min_y, max_x, max_y)
}

/// Returns the arc segment bounding box. Assumes `v1` to `v2` is an arc.
pub(crate) fn arc_seg_bounding_box<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> AABB<T>
where
    T: Real,
{
    debug_assert!(!v1.bulge_is_zero(), "expected arc");

    if v1.pos().fuzzy_eq(v2.pos()) {
        return AABB::new(v1.x, v1.y, v1.x, v1.y);
    }

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    let start_angle = angle(arc_center, v1.pos());
    let end_angle = angle(arc_center, v2.pos());
    let sweep_angle = delta_angle_signed(start_angle, end_angle, v1.bulge_is_neg());
    let (min_point, max_point) = arc_sweep_extents(
        v1.pos(),
        v2.pos(),
        arc_center,
        arc_radius,
        start_angle,
        sweep_angle,
    );

    AABB::new(min_point.x, min_point.y, max_point.x, max_point.y)
}

/// Computes the axis aligned bounding box of a polyline segment defined by `v1` to `v2`.
///
/// This function is quite a bit slower than [`seg_fast_approx_bounding_box`] when given an arc.
pub fn seg_bounding_box<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> AABB<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        line_seg_bounding_box(v1.pos(), v2.pos())
    } else {
        arc_seg_bounding_box(v1, v2)
    }
}

/// Calculate the path length of the polyline segment defined by `v1` to `v2`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::polyline::*;
/// # use cavalier_contours::core::traits::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// // arc radius = 1 so length should be PI
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// assert!(seg_length(v1, v2).fuzzy_eq(std::f64::consts::PI));
/// ```
///
/// Also works with line segments.
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::polyline::*;
/// // line segment going from (2, 2) to (4, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 0.0);
/// let v2 = PlineVertex::new(4.0, 4.0, 0.0);
/// assert!(seg_length(v1, v2).fuzzy_eq(2.0 * 2.0f64.sqrt()));
/// ```
pub fn seg_length<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> T
where
    T: Real,
{
    if v1.fuzzy_eq(v2) {
        return T::zero();
    }

    if v1.bulge_is_zero() {
        return dist_squared(v1.pos(), v2.pos()).sqrt();
    }

    let abs_bulge = v1.bulge.abs();
    let chord_length = (v2.pos() - v1.pos()).length();
    let arc_radius = chord_length * (abs_bulge * abs_bulge + T::one()) / (T::four() * abs_bulge);
    arc_radius * angle_from_bulge(v1.bulge).abs()
}

/// Returns the forward distance from `v1` to `point` along the segment from `v1` to `v2`.
///
/// Assumes `point` lies on the segment from `v1` to `v2`.
///
/// Endpoints snap with `pos_equal_eps`. For lines, it uses distance along the line. For arcs, it
/// uses the radius times the forward angle, which is the smaller radial angle because supported
/// arcs sweep at most a half circle.
pub fn dist_from_segment_start<T>(
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    point: Vector2<T>,
    pos_equal_eps: T,
) -> T
where
    T: Real,
{
    if point.fuzzy_eq_eps(v1.pos(), pos_equal_eps) {
        return T::zero();
    }
    if point.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
        return seg_length(v1, v2);
    }
    if v1.bulge_is_zero() {
        let direction = v2.pos() - v1.pos();
        let length = direction.length();
        if length == T::zero() {
            T::zero()
        } else {
            (point - v1.pos()).dot(direction) / length
        }
    } else {
        let bulge = v1.bulge;
        let bulge_squared = bulge * bulge;
        let chord = v2.pos() - v1.pos();
        let radius = chord.length() * (bulge_squared + T::one()) / (T::four() * bulge.abs());

        // These are the start and point radius vectors scaled by `4 * bulge`. Deriving them
        // directly from the chord avoids constructing a distant center for shallow arcs. Their
        // common scale does not change the angle between them.
        let start_radius =
            chord.perp().scale(bulge_squared - T::one()) - chord.scale(T::two() * bulge);
        let point_radius = start_radius + (point - v1.pos()).scale(T::four() * bulge);
        let sweep = T::atan2(
            start_radius.perp_dot(point_radius).abs(),
            start_radius.dot(point_radius),
        );
        radius * sweep
    }
}

/// Find the midpoint for the polyline segment defined by `v1` to `v2`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::polyline::*;
/// # use cavalier_contours::core::math::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// assert!(seg_midpoint(v1, v2).fuzzy_eq(Vector2::new(3.0, 1.0)));
/// ```
///
/// Also works with line segments.
///
/// ```
/// # use cavalier_contours::polyline::*;
/// # use cavalier_contours::core::math::*;
/// // line segment going from (2, 2) to (4, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 0.0);
/// let v2 = PlineVertex::new(4.0, 4.0, 0.0);
/// assert!(seg_midpoint(v1, v2).fuzzy_eq(Vector2::new(3.0, 3.0)));
/// ```
pub fn seg_midpoint<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> Vector2<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        return midpoint(v1.pos(), v2.pos());
    }

    let chord_midpoint = midpoint(v1.pos(), v2.pos());
    let half_bulge = v1.bulge / T::two();
    Vector2::new(
        chord_midpoint.x + half_bulge * (v2.y - v1.y),
        chord_midpoint.y - half_bulge * (v2.x - v1.x),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::traits::FuzzyEq;

    #[test]
    fn seg_arc_radius_and_center_returns_expected_geometry() {
        let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
        for (bulge, expected_radius, expected_center_y) in [
            (1.0, 2.0, 0.0),
            (-1.0, 2.0, 0.0),
            (quarter_circle_bulge, 2.0f64.sqrt() * 2.0, 2.0),
            (-quarter_circle_bulge, 2.0f64.sqrt() * 2.0, -2.0),
            (1e-7, 10_000_000.0000001, 9_999_999.9999999),
            (-1e-7, 10_000_000.0000001, -9_999_999.9999999),
        ] {
            let v1 = PlineVertex::new(0.0, 0.0, bulge);
            let v2 = PlineVertex::new(4.0, 0.0, 0.0);
            let (radius, center) = seg_arc_radius_and_center(v1, v2);
            assert!((radius - expected_radius).abs() <= expected_radius * 2e-15);
            assert!(center.fuzzy_eq_eps(
                Vector2::new(2.0, expected_center_y),
                expected_radius * 2e-15
            ));
        }
    }

    #[test]
    fn seg_tangent_vector_preserves_arc_radius_magnitude() {
        for bulge in [
            -1.0,
            -std::f64::consts::FRAC_PI_8.tan(),
            -1e-7,
            1e-7,
            std::f64::consts::FRAC_PI_8.tan(),
            1.0,
        ] {
            let sweep = 4.0 * bulge.atan();
            let v1 = PlineVertex::new(1.0, 0.0, bulge);
            let v2 = PlineVertex::new(sweep.cos(), sweep.sin(), 0.0);
            let point = Vector2::new((sweep / 2.0).cos(), (sweep / 2.0).sin());
            let tangent = seg_tangent_vector(v1, v2, point);
            let expected = if bulge < 0.0 {
                -point.perp()
            } else {
                point.perp()
            };
            assert!(
                tangent.fuzzy_eq_eps(expected, 2e-9),
                "{bulge}: {tangent:?} != {expected:?}"
            );
        }
    }

    #[test]
    fn seg_midpoint_returns_expected_arc_midpoints() {
        let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
        let sqrt_half = std::f64::consts::FRAC_1_SQRT_2;
        let cases = [
            (
                PlineVertex::new(0.0, 0.0, 1.0),
                PlineVertex::new(4.0, 0.0, 0.0),
                Vector2::new(2.0, -2.0),
            ),
            (
                PlineVertex::new(0.0, 0.0, -1.0),
                PlineVertex::new(4.0, 0.0, 0.0),
                Vector2::new(2.0, 2.0),
            ),
            (
                PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
                PlineVertex::new(0.0, 1.0, 0.0),
                Vector2::new(sqrt_half, sqrt_half),
            ),
            (
                PlineVertex::new(0.0, 0.0, 1e-7),
                PlineVertex::new(4.0, 0.0, 0.0),
                Vector2::new(2.0, -2e-7),
            ),
        ];

        for (v1, v2, expected) in cases {
            assert!(seg_midpoint(v1, v2).fuzzy_eq_eps(expected, 1e-14));
        }
    }

    #[test]
    fn seg_split_at_point_returns_expected_subarcs() {
        for bulge in [
            -1.0,
            -std::f64::consts::FRAC_PI_8.tan(),
            -1e-7,
            1e-7,
            std::f64::consts::FRAC_PI_8.tan(),
            1.0,
        ] {
            let sweep = 4.0 * bulge.atan();
            let v1 = PlineVertex::new(1.0, 0.0, bulge);
            let v2 = PlineVertex::new(sweep.cos(), sweep.sin(), 0.0);
            for fraction in [1e-6, 0.1, 0.5, 0.9, 1.0 - 1e-6] {
                let split_angle = sweep * fraction;
                let point = Vector2::new(split_angle.cos(), split_angle.sin());
                let result = seg_split_at_point(v1, v2, point, 1e-15);
                let expected1 = (split_angle / 4.0).tan();
                let expected2 = ((sweep - split_angle) / 4.0).tan();

                assert!(
                    result.updated_start.bulge.fuzzy_eq_eps(expected1, 2e-14),
                    "{bulge}, {fraction}: {} != {expected1}",
                    result.updated_start.bulge
                );
                assert!(
                    result.split_vertex.bulge.fuzzy_eq_eps(expected2, 2e-14),
                    "{bulge}, {fraction}: {} != {expected2}",
                    result.split_vertex.bulge
                );
                assert_eq!(result.updated_start.pos(), v1.pos());
                assert_eq!(result.split_vertex.pos(), point);
            }
        }
    }

    #[test]
    fn seg_length_returns_expected_arc_lengths() {
        for bulge in [
            -1.0,
            -std::f64::consts::FRAC_PI_8.tan(),
            -1e-7,
            1e-7,
            std::f64::consts::FRAC_PI_8.tan(),
            1.0,
        ] {
            let sweep = 4.0 * bulge.atan();
            let v1 = PlineVertex::new(1.0, 0.0, bulge);
            let v2 = PlineVertex::new(sweep.cos(), sweep.sin(), 0.0);
            assert!(
                seg_length(v1, v2).fuzzy_eq_eps(sweep.abs(), 2e-14),
                "{bulge}: {} != {}",
                seg_length(v1, v2),
                sweep.abs()
            );
        }
    }

    #[test]
    fn dist_from_segment_start_snaps_near_arc_endpoints() {
        let v1 = PlineVertex::new(-1.0f64, 0.0, -1.0);
        let v2 = PlineVertex::new(1.0, 0.0, 0.0);

        assert!(dist_from_segment_start(v1, v2, Vector2::new(-1.0, -1e-6), 1e-5).abs() <= 1e-10);
        assert!(
            (dist_from_segment_start(v1, v2, Vector2::new(1.0, 1e-6), 1e-5) - std::f64::consts::PI)
                .abs()
                <= 1e-10
        );
    }

    #[test]
    fn dist_from_segment_start_matches_arc_fractions_in_both_directions() {
        let radius = 3.25;
        let start_angle = 0.37;
        for bulge in [
            -1.0,
            -std::f64::consts::FRAC_PI_8.tan(),
            -1e-7,
            1e-7,
            std::f64::consts::FRAC_PI_8.tan(),
            1.0,
        ] {
            let sweep = 4.0 * bulge.atan();
            let point_at_angle = |angle: f64| {
                let (sin, cos) = angle.sin_cos();
                Vector2::new(radius * cos, radius * sin)
            };
            let start = point_at_angle(start_angle);
            let end = point_at_angle(start_angle + sweep);
            let v1 = PlineVertex::from_vector2(start, bulge);
            let v2 = PlineVertex::from_vector2(end, 0.0);
            let mut previous = 0.0;

            for fraction in [1e-6, 0.1, 0.5, 0.9, 1.0 - 1e-6] {
                let point = point_at_angle(start_angle + sweep * fraction);
                let distance = dist_from_segment_start(v1, v2, point, 1e-15);
                let expected = radius * sweep.abs() * fraction;
                assert!(
                    (distance - expected).abs() <= 2e-13 * radius,
                    "{bulge}, {fraction}: {distance} != {expected}"
                );
                assert!(
                    distance > previous,
                    "{bulge}, {fraction}: {distance} <= {previous}"
                );
                previous = distance;
            }
        }
    }
}
