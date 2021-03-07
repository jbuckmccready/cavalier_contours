use crate::{
    base_math::{
        angle, bulge_from_angle, delta_angle, dist_squared, line_seg_closest_point, midpoint,
        min_max, point_on_circle, point_within_arc_sweep,
    },
    core_math::is_left,
    PlineVertex, Real, Vector2, AABB,
};

/// Get the arc radius and center of an arc polyline segment defined by `v1` to `v2`.
/// Behavior undefined (may panic or return without error) if v1.bulge is zero.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
/// // arc half circle arc segment going from (0, 0) to (1, 0) counter clockwise
/// let v1 = PlineVertex::new(0.0, 0.0, 1.0);
/// let v2 = PlineVertex::new(1.0, 0.0, 0.0);
/// let (arc_radius, arc_center) =seg_arc_radius_and_center(v1, v2);
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
    let b = v1.bulge.abs();
    let v = v2.pos() - v1.pos();
    let d = v.length();
    let r = d * (b * b + T::one()) / (T::four() * b);

    // compute center
    let s = b * d / T::two();
    let m = r - s;
    let mut offs_x = -m * v.y / d;
    let mut offs_y = m * v.x / d;
    if v1.bulge_is_neg() {
        offs_x = -offs_x;
        offs_y = -offs_y;
    }

    let c = Vector2::new(
        v1.x + v.x / T::two() + offs_x,
        v1.y + v.y / T::two() + offs_y,
    );

    (r, c)
}

/// Result from splitting a segment using [seg_split_at_point].
#[derive(Debug, Copy, Clone)]
pub struct SplitResult<T = f64>
where
    T: Real,
{
    pub updated_start: PlineVertex<T>,
    pub split_vertex: PlineVertex<T>,
}

/// Splits a polyline segment defined by `v1` to `v2` at the `point_on_seg` given. Assumes the `point_on_seg` lies on the segment.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
    let theta1 = delta_angle(arc_start_angle, point_pos_angle);
    let bulge1 = bulge_from_angle(theta1);

    let arc_end_angle = angle(arc_center, v2.pos());
    let theta2 = delta_angle(point_pos_angle, arc_end_angle);
    let bulge2 = bulge_from_angle(theta2);

    let updated_start = PlineVertex::new(v1.x, v1.y, bulge1);
    let split_vertex = PlineVertex::new(point_on_seg.x, point_on_seg.y, bulge2);

    return SplitResult {
        updated_start,
        split_vertex,
    };
}

/// Find the tangent direction vector on a polyline segment defined by `v1` to `v2` at `point_on_seg`.
///
/// Note: The vector returned is just the direction vector, add the `point_on_seg` position to
/// get the actual tangent vector.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
    // initialize the arc extents with the chord extents
    let (chord_min_x, chord_max_x) = min_max(v1.x, v2.x);
    let (chord_min_y, chord_max_y) = min_max(v1.y, v2.y);
    let mut arc_extents = AABB::new(chord_min_x, chord_min_y, chord_max_x, chord_max_y);

    let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
    let circle_max_x_pt = Vector2::new(arc_center.x + arc_radius, arc_center.y);
    let circle_max_y_pt = Vector2::new(arc_center.x, arc_center.y + arc_radius);

    enum Quadrant {
        I,
        II,
        III,
        IV,
    }

    // helper function to determine quadrant a point resides in (with arc_center being the origin)
    let quadrant = |pt: Vector2<T>| -> Quadrant {
        // is_left wont work properly for the boundary case
        debug_assert!(
            !(pt.x.fuzzy_eq(arc_center.x) || pt.y.fuzzy_eq(arc_center.y)),
            "should have already checked this boundary case!"
        );

        if is_left(arc_center, circle_max_x_pt, pt) {
            // quadrant 1 or 2
            if is_left(arc_center, circle_max_y_pt, pt) {
                // quadrant 2
                return Quadrant::II;
            }

            // else quadrant 1
            return Quadrant::I;
        }
        // else quadrant 3 or 4
        if is_left(arc_center, circle_max_y_pt, pt) {
            // quadrant 3
            return Quadrant::III;
        }
        // else quadrant 4
        Quadrant::IV
    };

    let arc_is_ccw = v1.bulge_is_pos();

    // must check if arc is an axis aligned half circle because the is_left checks in the quadrant function will not work properly
    if v1.x.fuzzy_eq(arc_center.x) {
        // y axis aligned half circle
        if (v1.y > v2.y) == arc_is_ccw {
            // update min x (half circle bulges out in the negative x direction)
            arc_extents.min_x = arc_center.x - arc_radius;
        } else {
            // update max x (half circle bulges out in the positive x direction)
            arc_extents.max_x = arc_center.x + arc_radius;
        }
    } else if v1.y.fuzzy_eq(arc_center.y) {
        // x axis aligned half circle
        if (v1.x > v2.x) == arc_is_ccw {
            // update max y (half circle bulges out in the y positive direction)
            arc_extents.max_y = arc_center.y + arc_radius;
        } else {
            // update min y (half circle bulges out in the y negative direction)
            arc_extents.min_y = arc_center.y - arc_radius;
        }
    } else {
        // not axis aligned half circle, use quadrant function to find quadrant crossings
        let start_pt_quad = quadrant(v1.pos());
        let end_pt_quad = quadrant(v2.pos());
        // determine crossing from quadrants
        // note in some quadrant pair cases there is only one possible crossing due the arc
        // never having a sweep angle greater than PI, in other cases we must check the arc direction
        match (start_pt_quad, end_pt_quad) {
            (Quadrant::I, Quadrant::II) => {
                // crosses min y
                arc_extents.max_y = circle_max_y_pt.y;
            }
            (Quadrant::I, Quadrant::III) => {
                if arc_is_ccw {
                    // crosses max y then min x
                    arc_extents.max_y = circle_max_y_pt.y;
                    arc_extents.min_x = arc_center.x - arc_radius;
                } else {
                    // crosses max x then min y
                    arc_extents.max_x = circle_max_x_pt.x;
                    arc_extents.min_y = arc_center.y - arc_radius;
                }
            }
            (Quadrant::I, Quadrant::IV) => {
                // crosses max x
                arc_extents.max_x = circle_max_x_pt.x;
            }
            (Quadrant::II, Quadrant::I) => {
                // crosses max y
                arc_extents.max_y = circle_max_y_pt.y;
            }
            (Quadrant::II, Quadrant::III) => {
                // crosses min x
                arc_extents.min_x = arc_center.x - arc_radius;
            }
            (Quadrant::II, Quadrant::IV) => {
                if arc_is_ccw {
                    // crosses min x then min y
                    arc_extents.min_x = arc_center.x - arc_radius;
                    arc_extents.min_y = arc_center.y - arc_radius;
                } else {
                    // crosses max y then max x
                    arc_extents.max_y = circle_max_y_pt.y;
                    arc_extents.max_x = circle_max_x_pt.x;
                }
            }
            (Quadrant::III, Quadrant::I) => {
                if arc_is_ccw {
                    // crosses min y then max x
                    arc_extents.min_y = arc_center.y - arc_radius;
                    arc_extents.max_x = circle_max_x_pt.x;
                } else {
                    // crosses min x then max y
                    arc_extents.min_x = arc_center.x - arc_radius;
                    arc_extents.max_y = circle_max_y_pt.y;
                }
            }
            (Quadrant::III, Quadrant::II) => {
                // crosses min x
                arc_extents.min_x = arc_center.x - arc_radius;
            }
            (Quadrant::III, Quadrant::IV) => {
                // crosses min y
                arc_extents.min_y = arc_center.y - arc_radius;
            }
            (Quadrant::IV, Quadrant::I) => {
                // crosses max x
                arc_extents.max_x = circle_max_x_pt.x;
            }
            (Quadrant::IV, Quadrant::II) => {
                if arc_is_ccw {
                    // crosses max x then max y
                    arc_extents.max_x = circle_max_x_pt.x;
                    arc_extents.max_y = circle_max_y_pt.y;
                } else {
                    // crosses min y then min x
                    arc_extents.min_y = arc_center.y - arc_radius;
                    arc_extents.min_x = arc_center.x - arc_radius;
                }
            }
            (Quadrant::IV, Quadrant::III) => {
                // crosses min y
                arc_extents.min_y = arc_center.y - arc_radius;
            }

            // remaining cases not possible for the arc
            _ => {}
        }
    }

    arc_extents
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
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
/// // counter clockwise half circle arc going from (2, 2) to (2, 4)
/// let v1 = PlineVertex::new(2.0, 2.0, 1.0);
/// let v2 = PlineVertex::new(4.0, 2.0, 0.0);
/// assert!(seg_midpoint(v1, v2).fuzzy_eq(Vector2::new(3.0, 1.0)));
/// ```
///
/// Also works with line segments.
///
/// ```
/// # use cavalier_contours::*;
/// # use cavalier_contours::core_math::*;
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
    let angle_offset = delta_angle(angle1, angle2).abs() / T::two();
    let mid_angle = if v1.bulge_is_pos() {
        angle1 + angle_offset
    } else {
        angle1 - angle_offset
    };
    point_on_circle(arc_radius, arc_center, mid_angle)
}
