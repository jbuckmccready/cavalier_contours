use super::PlineVertex;
use crate::core::{
    math::{
        angle, angle_is_within_sweep, bulge_from_angle, delta_angle, delta_angle_signed,
        dist_squared, line_seg_closest_point, midpoint, min_max, point_on_circle,
        point_within_arc_sweep, Vector2,
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

    // compute radius
    let abs_bulge = v1.bulge.abs();
    let chord_v = v2.pos() - v1.pos();
    let chord_len = chord_v.length();
    let radius = chord_len * (abs_bulge * abs_bulge + T::one()) / (T::four() * abs_bulge);

    // compute center
    let s = abs_bulge * chord_len / T::two();
    let m = radius - s;
    let mut offs_x = -m * chord_v.y / chord_len;
    let mut offs_y = m * chord_v.x / chord_len;
    if v1.bulge_is_neg() {
        offs_x = -offs_x;
        offs_y = -offs_y;
    }

    let center = Vector2::new(
        v1.x + chord_v.x / T::two() + offs_x,
        v1.y + chord_v.y / T::two() + offs_y,
    );

    (radius, center)
}

/// Result from splitting a segment using [seg_split_at_point].
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

    let (_, arc_center) = seg_arc_radius_and_center(v1, v2);

    let point_pos_angle = angle(arc_center, point_on_seg);

    let arc_start_angle = angle(arc_center, v1.pos());
    let theta1 = delta_angle_signed(arc_start_angle, point_pos_angle, v1.bulge_is_neg());
    let bulge1 = bulge_from_angle(theta1);

    let arc_end_angle = angle(arc_center, v2.pos());
    let theta2 = delta_angle_signed(point_pos_angle, arc_end_angle, v1.bulge_is_neg());
    let bulge2 = bulge_from_angle(theta2);

    let updated_start = PlineVertex::new(v1.x, v1.y, bulge1);
    let split_vertex = PlineVertex::new(point_on_seg.x, point_on_seg.y, bulge2);

    SplitResult {
        updated_start,
        split_vertex,
    }
}

/// Find the tangent direction vector on a polyline segment defined by `v1` to `v2` at `point_on_seg`.
///
/// Note: The vector returned is just the direction vector, add the `point_on_seg` position to
/// get the actual tangent vector.
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

    let (_, arc_center) = seg_arc_radius_and_center(v1, v2);
    if v1.bulge_is_pos() {
        // ccw, rotate vector from center to point_on_seg 90 degrees
        return Vector2::new(
            -(point_on_seg.y - arc_center.y),
            point_on_seg.x - arc_center.x,
        );
    }

    // cw, rotate vector from center to pointOnCurve -90 degrees
    Vector2::new(
        point_on_seg.y - arc_center.y,
        -(point_on_seg.x - arc_center.x),
    )
}

/// Find the closest point on a polyline segment defined by `v1` to `v2` to `point` given.
/// If there are multiple closest points then one is chosen (which is chosen is not defined).
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::*;
/// # use cavalier_contours::polyline::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// assert!(seg_closest_point(v1, v2, Vector2::new(3.0, 0.0)).fuzzy_eq(Vector2::new(3.0, 1.0)));
/// assert!(seg_closest_point(v1, v2, Vector2::new(3.0, 1.2)).fuzzy_eq(Vector2::new(3.0, 1.0)));
/// assert!(seg_closest_point(v1, v2, v1.pos()).fuzzy_eq(v1.pos()));
/// assert!(seg_closest_point(v1, v2, v2.pos()).fuzzy_eq(v2.pos()));
/// ```
pub fn seg_closest_point<T>(v1: PlineVertex<T>, v2: PlineVertex<T>, point: Vector2<T>) -> Vector2<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        return line_seg_closest_point(v1.pos(), v2.pos(), point);
    }

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    if point.fuzzy_eq(arc_center) {
        // avoid normalizing zero length vector (point is at center, just return start point)
        return v1.pos();
    }

    if point_within_arc_sweep(arc_center, v1.pos(), v2.pos(), v1.bulge_is_neg(), point) {
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

/// Computes a fast approximate axis aligned bounding box of a polyline segment defined by `v1` to `v2`.
///
/// The bounding box may be larger than the true bounding box for the segment (but is never smaller).
/// For the true axis aligned bounding box use [seg_bounding_box] but this function is faster for arc
/// segments.
pub fn seg_fast_approx_bounding_box<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> AABB<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        // line segment
        let (min_x, max_x) = min_max(v1.x, v2.x);
        let (min_y, max_y) = min_max(v1.y, v2.y);
        return AABB::new(min_x, min_y, max_x, max_y);
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

    let crosses_angle = |angle| angle_is_within_sweep(angle, start_angle, sweep_angle);

    let min_x = if crosses_angle(T::pi()) {
        // crosses PI
        arc_center.x - arc_radius
    } else {
        num_traits::real::Real::min(v1.x, v2.x)
    };

    let min_y = if crosses_angle(T::from(1.5).unwrap() * T::pi()) {
        // crosses 3PI/2
        arc_center.y - arc_radius
    } else {
        num_traits::real::Real::min(v1.y, v2.y)
    };

    let max_x = if crosses_angle(T::zero()) {
        // crosses 2PI
        arc_center.x + arc_radius
    } else {
        num_traits::real::Real::max(v1.x, v2.x)
    };

    let max_y = if crosses_angle(T::from(0.5).unwrap() * T::pi()) {
        // crosses PI/2
        arc_center.y + arc_radius
    } else {
        num_traits::real::Real::max(v1.y, v2.y)
    };

    AABB::new(min_x, min_y, max_x, max_y)
}

/// Computes the axis aligned bounding box of a polyline segment defined by `v1` to `v2`.
///
/// This function is quite a bit slower than [seg_fast_approx_bounding_box] when given an arc.
pub fn seg_bounding_box<T>(v1: PlineVertex<T>, v2: PlineVertex<T>) -> AABB<T>
where
    T: Real,
{
    if v1.bulge_is_zero() {
        // line segment
        let (min_x, max_x) = min_max(v1.x, v2.x);
        let (min_y, max_y) = min_max(v1.y, v2.y);
        AABB::new(min_x, min_y, max_x, max_y)
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

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    let start_angle = angle(arc_center, v1.pos());
    let end_angle = angle(arc_center, v2.pos());
    arc_radius * delta_angle(start_angle, end_angle).abs()
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

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    let angle1 = angle(arc_center, v1.pos());
    let angle2 = angle(arc_center, v2.pos());
    let angle_offset = delta_angle_signed(angle1, angle2, v1.bulge_is_neg()) / T::two();
    let mid_angle = angle1 + angle_offset;
    point_on_circle(arc_radius, arc_center, mid_angle)
}
