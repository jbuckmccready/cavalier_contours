use super::{
    internal::pline_offset::parallel_offset,
    pline_seg::{
        arc_seg_bounding_box, seg_arc_radius_and_center, seg_closest_point,
        seg_fast_approx_bounding_box, seg_length,
    },
    seg_bounding_box, PlineVertex,
};
use crate::core::{
    math::{
        angle, angle_from_bulge, delta_angle, dist_squared, is_left, is_left_or_equal,
        point_on_circle, Vector2,
    },
    traits::Real,
};
use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder, AABB};
use std::{
    ops::{Index, IndexMut},
    slice::Windows,
};

/// Polyline represented by a sequence of [PlineVertex](crate::polyline::PlineVertex) and a bool
/// indicating whether the polyline is open (last vertex is end of polyline) or closed (last vertex
/// forms segment with first vertex).
#[derive(Debug, Clone)]
pub struct Polyline<T = f64> {
    vertex_data: Vec<PlineVertex<T>>,
    is_closed: bool,
}

impl<T> Default for Polyline<T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Polyline<T>
where
    T: Real,
{
    /// Create a new empty [Polyline] with `is_closed` set to false.
    pub fn new() -> Self {
        Polyline {
            vertex_data: Vec::new(),
            is_closed: false,
        }
    }

    /// Create a new empty [Polyline] with `is_closed` set to true.
    pub fn new_closed() -> Self {
        Polyline {
            vertex_data: Vec::new(),
            is_closed: true,
        }
    }

    /// Construct a new empty [Polyline] with `is_closed` set to false and some reserved capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Polyline {
            vertex_data: Vec::with_capacity(capacity),
            is_closed: false,
        }
    }

    /// Returns the number of vertexes currently in the polyline.
    pub fn len(&self) -> usize {
        self.vertex_data.len()
    }

    /// Returns true if `self.len() == 0`.
    pub fn is_empty(&self) -> bool {
        self.vertex_data.is_empty()
    }

    /// Reserves capacity for at least `additional` more elements.
    pub fn reserve(&mut self, additional: usize) {
        self.vertex_data.reserve(additional);
    }

    /// Add a vertex to the polyline by giving the `x`, `y`, and `bulge` values of the vertex.
    pub fn add(&mut self, x: T, y: T, bulge: T) {
        self.vertex_data.push(PlineVertex::new(x, y, bulge));
    }

    /// Add vertex from array data (index 0 = x, 1 = y, 2 = bulge).
    pub fn add_from_array(&mut self, data: [T; 3]) {
        self.add(data[0], data[1], data[2]);
    }

    /// Add a vertex if it's position is not fuzzy equal to the last vertex in the polyline.
    ///
    /// If the vertex position is fuzzy equal then just update the bulge of the last vertex with
    /// the bulge given.
    pub(crate) fn add_or_replace(&mut self, x: T, y: T, bulge: T, pos_equal_eps: T) {
        let ln = self.len();
        if ln == 0 {
            self.add(x, y, bulge);
            return;
        }

        let last_vert = &mut self.vertex_data[ln - 1];
        if last_vert.x.fuzzy_eq_eps(x, pos_equal_eps) && last_vert.y.fuzzy_eq_eps(y, pos_equal_eps)
        {
            last_vert.bulge = bulge;
            return;
        }

        self.add(x, y, bulge);
    }

    /// Add a vertex if it's position is not fuzzy equal to the last vertex in the polyline.
    ///
    /// If the vertex position is fuzzy equal then just update the bulge of the last vertex with
    /// the bulge given.
    pub(crate) fn add_or_replace_vertex(&mut self, vertex: PlineVertex<T>, pos_equal_eps: T) {
        self.add_or_replace(vertex.x, vertex.y, vertex.bulge, pos_equal_eps)
    }

    /// Returns the next wrapping vertex index for the polyline.
    ///
    /// If `i + 1 >= self.len()` then 0 is returned, otherwise `i + 1` is returned.
    pub fn next_wrapping_index(&self, i: usize) -> usize {
        let next = i + 1;
        if next >= self.len() {
            0
        } else {
            next
        }
    }

    /// Returns the previous wrapping vertex index for the polyline.
    ///
    /// If `i == 0` then `self.len() - 1` is returned, otherwise `i - 1` is returned.
    pub fn prev_wrapping_index(&self, i: usize) -> usize {
        if i == 0 {
            self.len() - 1
        } else {
            i - 1
        }
    }

    /// Add a vertex to the polyline by giving a [PlineVertex](crate::polyline::PlineVertex).
    pub fn add_vertex(&mut self, vertex: PlineVertex<T>) {
        self.vertex_data.push(vertex);
    }

    /// Copy all vertexes from other to the end of this polyline.
    pub fn extend_vertexes(&mut self, other: &Polyline<T>) {
        self.vertex_data.extend(other.vertex_data.iter());
    }

    /// Remove vertex at index.
    pub fn remove(&mut self, index: usize) -> PlineVertex<T> {
        self.vertex_data.remove(index)
    }

    /// Remove last vertex.
    pub fn remove_last(&mut self) -> PlineVertex<T> {
        self.remove(self.len() - 1)
    }

    /// Clear all vertexes.
    pub fn clear(&mut self) {
        self.vertex_data.clear();
    }

    /// Returns true if the polyline is closed, false if it is open.
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    /// Allows modifying whether the polyline is closed or not.
    pub fn set_is_closed(&mut self, is_closed: bool) {
        self.is_closed = is_closed;
    }

    pub fn last(&self) -> Option<&PlineVertex<T>> {
        self.vertex_data.last()
    }

    pub fn last_mut(&mut self) -> Option<&mut PlineVertex<T>> {
        self.vertex_data.last_mut()
    }

    /// Set the vertex data at a given index of the polyline.
    pub fn set_vertex(&mut self, index: usize, x: T, y: T, bulge: T) {
        self.vertex_data[index].x = x;
        self.vertex_data[index].y = y;
        self.vertex_data[index].bulge = bulge;
    }

    /// Fuzzy equal comparison with another polyline using `fuzzy_epsilon` given.
    pub fn fuzzy_eq_eps(&self, other: &Self, fuzzy_epsilon: T) -> bool {
        self.vertex_data
            .iter()
            .zip(&other.vertex_data)
            .all(|(v1, v2)| v1.fuzzy_eq_eps(*v2, fuzzy_epsilon))
    }

    /// Fuzzy equal comparison with another vertex using T::fuzzy_epsilon().
    pub fn fuzzy_eq(&self, other: &Self) -> bool {
        self.fuzzy_eq_eps(other, T::fuzzy_epsilon())
    }

    /// Invert/reverse the direction of the polyline in place.
    ///
    /// This method works by simply reversing the order of the vertexes, shifting by 1 position all
    /// the vertexes, and inverting the sign of all the bulge values. E.g. after reversing the
    /// vertex the bulge at index 0 becomes negative bulge at index 1. The end result for a closed
    /// polyline is the direction will be changed from clockwise to counter clockwise or vice versa.
    pub fn invert_direction(&mut self) {
        let ln = self.len();
        if ln < 2 {
            return;
        }

        self.vertex_data.reverse();

        let first_bulge = self[0].bulge;
        for i in 1..ln {
            self[i - 1].bulge = -self[i].bulge;
        }

        if self.is_closed {
            self[ln - 1].bulge = -first_bulge;
        }
    }

    /// Uniformly scale the polyline in the xy plane by `scale_factor`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(4.0, 4.0, 1.0);
    /// polyline.scale(2.0);
    /// let mut expected = Polyline::new();
    /// expected.add(4.0, 4.0, 0.5);
    /// expected.add(8.0, 8.0, 1.0);
    /// assert!(polyline.fuzzy_eq(&expected));
    /// ```
    pub fn scale(&mut self, scale_factor: T) {
        for v in self.iter_mut() {
            v.x = v.x * scale_factor;
            v.y = v.y * scale_factor;
        }
    }

    /// Translate the polyline by some `x_offset` and `y_offset`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(4.0, 4.0, 1.0);
    /// polyline.translate(-3.0, 1.0);
    /// let mut expected = Polyline::new();
    /// expected.add(-1.0, 3.0, 0.5);
    /// expected.add(1.0, 5.0, 1.0);
    /// assert!(polyline.fuzzy_eq(&expected));
    /// ```
    pub fn translate(&mut self, x_offset: T, y_offset: T) {
        for v in self.iter_mut() {
            v.x = v.x + x_offset;
            v.y = v.y + y_offset;
        }
    }

    /// Compute the XY extents of the polyline.
    ///
    /// Returns `None` if polyline is empty. If polyline has only one vertex then
    /// `min_x = max_x = polyline[0].x` and `min_y = max_y = polyline[0].y`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// let mut polyline = Polyline::new();
    /// assert_eq!(polyline.extents(), None);
    /// polyline.add(1.0, 1.0, 1.0);
    /// let pt_extents = polyline.extents().unwrap();
    /// assert!(pt_extents.min_x.fuzzy_eq(1.0));
    /// assert!(pt_extents.min_y.fuzzy_eq(1.0));
    /// assert!(pt_extents.max_x.fuzzy_eq(1.0));
    /// assert!(pt_extents.max_y.fuzzy_eq(1.0));
    ///
    /// polyline.add(3.0, 1.0, 1.0);
    /// let extents = polyline.extents().unwrap();
    /// assert!(extents.min_x.fuzzy_eq(1.0));
    /// assert!(extents.min_y.fuzzy_eq(0.0));
    /// assert!(extents.max_x.fuzzy_eq(3.0));
    /// assert!(extents.max_y.fuzzy_eq(1.0));
    ///
    /// polyline.set_is_closed(true);
    /// let extents = polyline.extents().unwrap();
    /// assert!(extents.min_x.fuzzy_eq(1.0));
    /// assert!(extents.min_y.fuzzy_eq(0.0));
    /// assert!(extents.max_x.fuzzy_eq(3.0));
    /// assert!(extents.max_y.fuzzy_eq(2.0));
    /// ```
    pub fn extents(&self) -> Option<AABB<T>> {
        if self.is_empty() {
            return None;
        }

        let mut result = AABB::new(self[0].x, self[0].y, self[0].x, self[0].y);

        for (v1, v2) in self.iter_segments() {
            if v1.bulge_is_zero() {
                // line segment, just look at end of line point
                if v2.x < result.min_x {
                    result.min_x = v2.x;
                } else if v2.x > result.max_x {
                    result.max_x = v2.x;
                }

                if v2.y < result.min_y {
                    result.min_y = v2.y;
                } else if v2.y > result.max_y {
                    result.max_y = v2.y;
                }

                continue;
            }
            // else arc segment
            let arc_extents = arc_seg_bounding_box(v1, v2);

            result.min_x = num_traits::real::Real::min(result.min_x, arc_extents.min_x);
            result.min_y = num_traits::real::Real::min(result.min_y, arc_extents.min_y);
            result.max_x = num_traits::real::Real::max(result.max_x, arc_extents.max_x);
            result.max_y = num_traits::real::Real::max(result.max_y, arc_extents.max_y);
        }

        Some(result)
    }

    /// Creates a fast approximate spatial index of all the polyline segments.
    ///
    /// The starting vertex index position is used as the key to the segment bounding box in the
    /// `StaticAABB2DIndex`. The bounding boxes are guaranteed to be no smaller than the actual
    /// bounding box of the segment but may be larger, this is done for performance. If you want the
    /// actual bounding box index use [Polyline::create_spatial_index] instead.
    ///
    /// Returns `None` if polyline vertex count is less than 2 or an error occurs in constructing
    /// the spatial index.
    pub fn create_approx_spatial_index(&self) -> Option<StaticAABB2DIndex<T>> {
        let ln = self.len();
        if ln < 2 {
            return None;
        }

        let seg_count = if self.is_closed { ln } else { ln - 1 };

        let mut builder = StaticAABB2DIndexBuilder::new(seg_count);

        for (v1, v2) in self.iter_segments() {
            let approx_aabb = seg_fast_approx_bounding_box(v1, v2);
            builder.add(
                approx_aabb.min_x,
                approx_aabb.min_y,
                approx_aabb.max_x,
                approx_aabb.max_y,
            );
        }

        builder.build().ok()
    }

    /// Creates a spatial index of all the polyline segments.
    ///
    /// The starting vertex index position is used as the key to the segment bounding box in the
    /// `StaticAABB2DIndex`. The bounding boxes are the actual bounding box of the segment, for
    /// performance reasons you may want to use [Polyline::create_approx_spatial_index].
    ///
    /// Returns `None` if polyline vertex count is less than 2 or an error occurs in constructing
    /// the spatial index.
    pub fn create_spatial_index(&self) -> Option<StaticAABB2DIndex<T>> {
        let ln = self.len();
        if ln < 2 {
            return None;
        }

        let seg_count = if self.is_closed { ln } else { ln - 1 };

        let mut builder = StaticAABB2DIndexBuilder::new(seg_count);

        for (v1, v2) in self.iter_segments() {
            let approx_aabb = seg_bounding_box(v1, v2);
            builder.add(
                approx_aabb.min_x,
                approx_aabb.min_y,
                approx_aabb.max_x,
                approx_aabb.max_y,
            );
        }

        builder.build().ok()
    }

    /// Visit all the polyline segments (represented as polyline vertex pairs, starting at indexes
    /// (0, 1)) with a function/closure.
    ///
    /// This is equivalent to [Polyline::iter_segments] but uses a visiting function rather than an
    /// iterator.
    pub fn visit_segments<F>(&self, visitor: &mut F)
    where
        F: FnMut(PlineVertex<T>, PlineVertex<T>) -> bool,
    {
        let ln = self.vertex_data.len();
        if ln < 2 {
            return;
        }

        let mut windows = self.vertex_data.windows(2);
        while let Some(&[v1, v2]) = windows.next() {
            if !visitor(v1, v2) {
                return;
            }
        }

        if self.is_closed {
            let v1 = self.vertex_data[ln - 1];
            let v2 = self.vertex_data[0];
            visitor(v1, v2);
        }
    }

    /// Iterate through all the vertexes in the polyline.
    pub fn iter(&self) -> impl Iterator<Item = &PlineVertex<T>> + Clone {
        self.vertex_data.iter()
    }

    /// Iterate through all the vertexes in the polyline as mutable references.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut PlineVertex<T>> {
        self.vertex_data.iter_mut()
    }

    /// Iterate through all the polyline segments (represented as polyline vertex pairs, starting at
    /// indexes (0, 1)).
    ///
    /// This is equivalent to [Polyline::visit_segments] but returns an iterator rather than
    /// accepting a visiting function.
    pub fn iter_segments(
        &self,
    ) -> impl Iterator<Item = (PlineVertex<T>, PlineVertex<T>)> + Clone + '_ {
        PlineSegIterator::new(&self)
    }

    /// Iterate through all the polyline segment vertex positional indexes.
    ///
    /// Segments are represented by polyline vertex pairs, for each vertex there is an associated
    /// positional index in the polyline, this method iterates through those positional indexes as
    /// segment pairs starting at (0, 1) and ending at (n-2, n-1) if open polyline or (n-1, 0) if
    /// closed polyline where n is the number of vertexes.
    pub fn iter_segment_indexes(&self) -> impl Iterator<Item = (usize, usize)> {
        PlineSegIndexIterator::new(self.vertex_data.len(), self.is_closed)
    }

    /// Compute the parallel offset polylines of the polyline.
    ///
    /// `offset` determines what offset polylines generated, if it is positive then the direction of
    /// the offset is to the left of the polyline segment tangent vectors otherwise it is to the
    /// right.
    ///
    /// `spatial_index` is a spatial index of all the polyline segment bounding boxes (or boxes no
    /// smaller, e.g. using [Polyline::create_approx_spatial_index] is valid). If `None` is given
    /// then it will be computed internally. [Polyline::create_approx_spatial_index] or
    /// [Polyline::create_spatial_index] may be used to create the spatial index, the only
    /// restriction is that the spatial index bounding boxes must be at least big enough to contain
    /// the segments.
    ///
    /// `options` is a struct that holds parameters for tweaking the behavior of the algorithm, if
    /// `None is given then `PlineOffsetOptions::default()` will be used. See
    /// [PlineOffsetOptions](crate::polyline::PlineOffsetOptions) for specific parameters.
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// // using the options struct to inform the algorithm that there may be self intersects
    /// // in the polyline to be offset
    /// let options = PlineOffsetOptions { handle_self_intersects: true, .. Default::default() };
    /// let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// // passing in None for the spatial_index
    /// let offset_plines = pline.parallel_offset(0.2, None, Some(options));
    /// assert_eq!(offset_plines.len(), 1);
    /// let offset_pline = &offset_plines[0];
    /// assert!(offset_pline[0].fuzzy_eq(PlineVertex::new(0.2, 0.0, 1.0)));
    /// assert!(offset_pline[1].fuzzy_eq(PlineVertex::new(0.8, 0.0, 1.0)));
    /// ```
    pub fn parallel_offset(
        &self,
        offset: T,
        spatial_index: Option<&StaticAABB2DIndex<T>>,
        options: Option<PlineOffsetOptions<T>>,
    ) -> Vec<Polyline<T>> {
        parallel_offset(self, offset, spatial_index, options)
    }

    /// Compute the closed signed area of the polyline.
    ///
    /// If [Polyline::is_closed] is false (open polyline) then 0.0 is always returned.
    /// The area is signed such that if the polyline direction is counter clockwise
    /// then the area is positive, otherwise it is negative.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// let mut polyline: Polyline = Polyline::new();
    /// assert!(polyline.area().fuzzy_eq(0.0));
    /// polyline.add(1.0, 1.0, 1.0);
    /// assert!(polyline.area().fuzzy_eq(0.0));
    ///
    /// polyline.add(3.0, 1.0, 1.0);
    /// // polyline is still open so area is 0
    /// assert!(polyline.area().fuzzy_eq(0.0));
    /// polyline.set_is_closed(true);
    /// assert!(polyline.area().fuzzy_eq(std::f64::consts::PI));
    /// polyline.invert_direction();
    /// assert!(polyline.area().fuzzy_eq(-std::f64::consts::PI));
    /// ```
    pub fn area(&self) -> T {
        if !self.is_closed {
            return T::zero();
        }

        // Implementation notes:
        // Using the shoelace formula (https://en.wikipedia.org/wiki/Shoelace_formula) modified to
        // support arcs defined by a bulge value. The shoelace formula returns a negative value for
        // clockwise oriented polygons and positive value for counter clockwise oriented polygons.
        // The area of each circular segment defined by arcs is then added if it is a counter
        // clockwise arc or subtracted if it is a clockwise arc. The area of the circular segments
        // are computed by finding the area of the arc sector minus the area of the triangle
        // defined by the chord and center of circle.
        // See https://en.wikipedia.org/wiki/Circular_segment
        let mut double_total_area = T::zero();

        for (v1, v2) in self.iter_segments() {
            double_total_area = double_total_area + v1.x * v2.y - v1.y * v2.x;
            if !v1.bulge_is_zero() {
                // add arc segment area
                let b = v1.bulge.abs();
                let sweep_angle = angle_from_bulge(b);
                let triangle_base = (v2.pos() - v1.pos()).length();
                let radius = triangle_base * ((b * b + T::one()) / (T::four() * b));
                let sagitta = b * triangle_base / T::two();
                let triangle_height = radius - sagitta;
                let double_sector_area = sweep_angle * radius * radius;
                let double_triangle_area = triangle_base * triangle_height;
                let mut double_arc_area = double_sector_area - double_triangle_area;
                if v1.bulge_is_neg() {
                    double_arc_area = -double_arc_area;
                }

                double_total_area = double_total_area + double_arc_area;
            }
        }

        double_total_area / T::two()
    }

    /// Find the closest segment point on a polyline to a `point` given.
    ///
    /// If the polyline is empty then `None` is returned.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline: Polyline = Polyline::new();
    /// assert!(matches!(polyline.closest_point(Vector2::zero()), None));
    /// polyline.add(1.0, 1.0, 1.0);
    /// let result = polyline.closest_point(Vector2::new(1.0, 0.0)).unwrap();
    /// assert_eq!(result.seg_start_index, 0);
    /// assert!(result.seg_point.fuzzy_eq(polyline[0].pos()));
    /// assert!(result.distance.fuzzy_eq(1.0));
    /// ```
    pub fn closest_point(&self, point: Vector2<T>) -> Option<ClosestPointResult<T>> {
        if self.is_empty() {
            return None;
        }

        let mut result = ClosestPointResult {
            seg_start_index: 0,
            seg_point: self[0].pos(),
            distance: Real::max_value(),
        };

        if self.len() == 1 {
            result.distance = (result.seg_point - point).length();
            return Some(result);
        }

        let mut dist_squared = Real::max_value();

        for (i, j) in self.iter_segment_indexes() {
            let v1 = self[i];
            let v2 = self[j];
            let cp = seg_closest_point(v1, v2, point);
            let diff_v = point - cp;
            let dist2 = diff_v.length_squared();
            if dist2 < dist_squared {
                result.seg_start_index = i;
                result.seg_point = cp;
                dist_squared = dist2;
            }
        }

        result.distance = dist_squared.sqrt();

        Some(result)
    }

    /// Returns the total path length of the polyline.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// let mut polyline: Polyline = Polyline::new();
    /// // open polyline half circle
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(2.0, 0.0, 1.0);
    /// assert!(polyline.path_length().fuzzy_eq(std::f64::consts::PI));
    /// // close into full circle
    /// polyline.set_is_closed(true);
    /// assert!(polyline.path_length().fuzzy_eq(2.0 * std::f64::consts::PI));
    /// ```
    pub fn path_length(&self) -> T {
        self.iter_segments()
            .fold(T::zero(), |acc, (v1, v2)| acc + seg_length(v1, v2))
    }

    /// Helper function for processing a line segment when computing the winding number.
    fn process_line_winding(v1: PlineVertex<T>, v2: PlineVertex<T>, point: Vector2<T>) -> i32 {
        let mut result = 0;
        if v1.y <= point.y {
            if v2.y > point.y && is_left(v1.pos(), v2.pos(), point) {
                // left and upward crossing
                result += 1;
            }
        } else if v2.y <= point.y && !is_left(v1.pos(), v2.pos(), point) {
            // right an downward crossing
            result -= 1;
        }

        result
    }

    /// Helper function for processing an arc segment when computing the winding number.
    fn process_arc_winding(v1: PlineVertex<T>, v2: PlineVertex<T>, point: Vector2<T>) -> i32 {
        let is_ccw = v1.bulge_is_pos();
        let point_is_left = if is_ccw {
            is_left(v1.pos(), v2.pos(), point)
        } else {
            is_left_or_equal(v1.pos(), v2.pos(), point)
        };

        let dist_to_arc_center_less_than_radius = || {
            let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
            let dist2 = dist_squared(arc_center, point);
            dist2 < arc_radius * arc_radius
        };

        let mut result = 0;

        if v1.y <= point.y {
            if v2.y > point.y {
                // upward crossing of arc chord
                if is_ccw {
                    if point_is_left {
                        // counter clockwise arc left of chord
                        result += 1;
                    } else {
                        // counter clockwise arc right of chord
                        if dist_to_arc_center_less_than_radius() {
                            result += 1;
                        }
                    }
                } else if point_is_left {
                    // clockwise arc left of chord
                    if !dist_to_arc_center_less_than_radius() {
                        result += 1;
                    }
                    // else clockwise arc right of chord, no crossing
                }
            } else {
                // not crossing arc chord and chord is below, check if point is inside arc sector
                if is_ccw
                    && !point_is_left
                    && v2.x < point.x
                    && point.x < v1.x
                    && dist_to_arc_center_less_than_radius()
                {
                    result += 1;
                } else if !is_ccw
                    && point_is_left
                    && v1.x < point.x
                    && point.x < v2.x
                    && dist_to_arc_center_less_than_radius()
                {
                    result -= 1;
                }
            }
        } else if v2.y <= point.y {
            // downward crossing of arc chord
            if is_ccw {
                if !point_is_left {
                    // counter clockwise arc right of chord
                    if dist_to_arc_center_less_than_radius() {
                        result -= 1;
                    }
                }
            // else counter clockwise arc left of chord, no crossing
            } else if point_is_left {
                // clockwise arc left of chord
                if dist_to_arc_center_less_than_radius() {
                    result -= 1;
                }
            } else {
                // clockwise arc right of chord
                result -= 1;
            }
        } else {
            // not crossing arc chord and chord is above, check if point is inside arc sector
            if is_ccw
                && !point_is_left
                && v1.x < point.x
                && point.x < v2.x
                && dist_to_arc_center_less_than_radius()
            {
                result += 1;
            } else if v2.x < point.x && point.x < v1.x && dist_to_arc_center_less_than_radius() {
                result -= 1;
            }
        }

        result
    }

    /// Calculate the winding number for a `point` relative to the polyline.
    ///
    /// The winding number calculates the number of turns/windings around a point
    /// that the polyline path makes. For a closed polyline without self intersects
    /// there are only three possibilities:
    ///
    /// * -1 (winds around point clockwise)
    /// * 0 (point is outside the polyline)
    /// * 1 (winds around the point counter clockwise).
    ///
    /// This function always returns 0 if polyline [Polyline::is_closed] is false.
    ///
    /// If the point lies directly on top of one of the polyline segments the result
    /// is not defined.
    ///
    /// # Examples
    ///
    /// ### Polyline without self intersects
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline: Polyline = Polyline::new_closed();
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(2.0, 0.0, 1.0);
    /// assert_eq!(polyline.winding_number(Vector2::new(1.0, 0.0)), 1);
    /// assert_eq!(polyline.winding_number(Vector2::new(0.0, 2.0)), 0);
    /// polyline.invert_direction();
    /// assert_eq!(polyline.winding_number(Vector2::new(1.0, 0.0)), -1);
    /// ```
    ///
    /// ### Multiple windings with self intersecting polyline
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline: Polyline = Polyline::new_closed();
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(2.0, 0.0, 1.0);
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(4.0, 0.0, 1.0);
    /// assert_eq!(polyline.winding_number(Vector2::new(1.0, 0.0)), 2);
    /// assert_eq!(polyline.winding_number(Vector2::new(-1.0, 0.0)), 0);
    /// polyline.invert_direction();
    /// assert_eq!(polyline.winding_number(Vector2::new(1.0, 0.0)), -2);
    /// ```
    pub fn winding_number(&self, point: Vector2<T>) -> i32 {
        if !self.is_closed || self.len() < 2 {
            return 0;
        }

        let mut winding = 0;

        for (v1, v2) in self.iter_segments() {
            if v1.bulge_is_zero() {
                winding += Self::process_line_winding(v1, v2, point);
            } else {
                winding += Self::process_arc_winding(v1, v2, point);
            }
        }

        winding
    }

    /// Returns a new polyline with all arc segments converted to line segments with some
    /// `error_distance` or None if T fails to cast to or from usize.
    ///
    /// `error_distance` is the maximum distance from any line segment to the arc it is
    /// approximating. Line segments are circumscribed by the arc (all line end points lie on the
    /// arc path).
    pub fn arcs_to_approx_lines(&self, error_distance: T) -> Option<Self> {
        let mut result = Polyline::new();
        result.set_is_closed(self.is_closed);

        // catch case where polyline is empty since we may index into the last vertex later
        if self.is_empty() {
            return Some(result);
        }

        let abs_error = error_distance.abs();

        for (v1, v2) in self.iter_segments() {
            if v1.bulge_is_zero() {
                result.add_vertex(v1);
                continue;
            }

            let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
            if arc_radius.fuzzy_lt(error_distance) {
                result.add(v1.x, v1.y, T::zero());
                continue;
            }

            let start_angle = angle(arc_center, v1.pos());
            let end_angle = angle(arc_center, v2.pos());
            let angle_diff = delta_angle(start_angle, end_angle).abs();

            let seg_sub_angle = T::two() * (T::one() - abs_error / arc_radius).acos().abs();
            let seg_count = (angle_diff / seg_sub_angle).ceil();
            // create angle offset such that all lines have an equal part of the arc
            let seg_angle_offset = if v1.bulge_is_neg() {
                -angle_diff / seg_count
            } else {
                angle_diff / seg_count
            };

            // add start vertex
            result.add(v1.x, v1.y, T::zero());
            let usize_count = seg_count.to_usize()?;
            // add all vertex points along arc
            for i in 1..usize_count {
                let angle_pos = T::from(i)?;
                let angle = angle_pos * seg_angle_offset + start_angle;
                let pos = point_on_circle(arc_radius, arc_center, angle);
                result.add(pos.x, pos.y, T::zero());
            }
        }

        if !self.is_closed {
            // add the final missing vertex in the case that the polyline is not closed
            result.add_vertex(self[self.len() - 1]);
        }

        Some(result)
    }
}

/// Result from calling [Polyline::closest_point].
#[derive(Debug, Copy, Clone)]
pub struct ClosestPointResult<T>
where
    T: Real,
{
    /// The start vertex index of the closest segment.
    pub seg_start_index: usize,
    /// The closest point on the closest segment.
    pub seg_point: Vector2<T>,
    /// The distance between the points.
    pub distance: T,
}

/// Struct to hold options parameters passed to [Polyline::parallel_offset].
#[derive(Debug, Clone)]
pub struct PlineOffsetOptions<T = f64>
where
    T: Real,
{
    pub pos_equal_eps: T,
    pub slice_join_eps: T,
    pub offset_dist_eps: T,
    pub handle_self_intersects: bool,
}

impl<T> Default for PlineOffsetOptions<T>
where
    T: Real,
{
    fn default() -> Self {
        PlineOffsetOptions {
            pos_equal_eps: T::from(1e-5).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
            offset_dist_eps: T::from(1e-4).unwrap(),
            handle_self_intersects: false,
        }
    }
}

impl<T> Index<usize> for Polyline<T> {
    type Output = PlineVertex<T>;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.vertex_data[index]
    }
}

impl<T> IndexMut<usize> for Polyline<T> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.vertex_data[index]
    }
}

/// An iterator that traverses all segment vertex pairs.
#[derive(Debug, Clone)]
struct PlineSegIterator<'a, T>
where
    T: Real,
{
    polyline: &'a Polyline<T>,
    vertex_windows: Windows<'a, PlineVertex<T>>,
    wrap_not_exhausted: bool,
}

impl<'a, T> PlineSegIterator<'a, T>
where
    T: Real,
{
    fn new(polyline: &'a Polyline<T>) -> PlineSegIterator<'a, T> {
        let vertex_windows = polyline.vertex_data.windows(2);
        let wrap_not_exhausted = if polyline.vertex_data.len() < 2 {
            false
        } else {
            polyline.is_closed
        };
        PlineSegIterator {
            polyline,
            vertex_windows,
            wrap_not_exhausted,
        }
    }
}

impl<'a, T> Iterator for PlineSegIterator<'a, T>
where
    T: Real,
{
    type Item = (PlineVertex<T>, PlineVertex<T>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(&[v1, v2]) = self.vertex_windows.next() {
            Some((v1, v2))
        } else if self.wrap_not_exhausted {
            self.wrap_not_exhausted = false;
            let ln = self.polyline.len();
            Some((self.polyline[ln - 1], self.polyline[0]))
        } else {
            None
        }
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        let windows_hint = self.vertex_windows.size_hint();
        if self.wrap_not_exhausted {
            (windows_hint.0 + 1, windows_hint.1.map(|h| h + 1))
        } else {
            windows_hint
        }
    }
}

/// An iterator that traverses all segment vertex pair index positions.
struct PlineSegIndexIterator {
    pos: usize,
    remaining: usize,
    is_closed: bool,
}

impl PlineSegIndexIterator {
    fn new(vertex_count: usize, is_closed: bool) -> PlineSegIndexIterator {
        let remaining = if vertex_count < 2 {
            0
        } else if is_closed {
            vertex_count
        } else {
            vertex_count - 1
        };
        PlineSegIndexIterator {
            pos: 0,
            remaining,
            is_closed,
        }
    }
}

impl Iterator for PlineSegIndexIterator {
    type Item = (usize, usize);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }

        self.remaining -= 1;

        if self.remaining == 0 && self.is_closed {
            return Some((self.pos, 0));
        }

        let pos = self.pos;
        self.pos += 1;
        Some((pos, pos + 1))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}
