use super::{
    internal::{pline_boolean::polyline_boolean, pline_offset::parallel_offset},
    pline_seg::{
        arc_seg_bounding_box, seg_arc_radius_and_center, seg_closest_point,
        seg_fast_approx_bounding_box, seg_length,
    },
    seg_bounding_box, PlineVertex,
};
use crate::core::{
    math::{
        angle, angle_from_bulge, bulge_from_angle, delta_angle, dist_squared, is_left,
        is_left_or_equal, point_on_circle, Vector2,
    },
    traits::Real,
};
use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder, AABB};
use std::{
    borrow::Cow,
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

    /// Create a new [Polyline] with vertexes given by an iterator.
    pub fn from_iter<I>(iter: I, is_closed: bool) -> Self
    where
        I: IntoIterator<Item = PlineVertex<T>>,
    {
        Polyline {
            vertex_data: iter.into_iter().collect(),
            is_closed,
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

    /// Copy all vertexes from `other` to the end of this polyline.
    pub fn extend(&mut self, other: &Polyline<T>) {
        self.vertex_data.extend(other.vertex_data.iter());
    }

    /// Add all vertexes to the end of this polyline.
    pub fn extend_vertexes<I>(&mut self, vertexes: I)
    where
        I: IntoIterator<Item = PlineVertex<T>>,
    {
        self.vertex_data.extend(vertexes);
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

    /// Remove all repeat position vertexes from the polyline.
    ///
    /// A `Cow<Polyline>` is returned to avoid allocation and copy in the case that no vertexes are
    /// removed.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use std::borrow::Cow;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(2.0, 2.0, 1.0);
    /// polyline.add(3.0, 3.0, 1.0);
    /// polyline.add(3.0, 3.0, 0.5);
    /// let result = polyline.remove_repeat_pos(1e-5);
    /// // result will be owned since vertexes were removed
    /// assert!(matches!(result, Cow::Owned(_)));
    /// assert_eq!(result.len(), 2);
    /// assert!(result[0].fuzzy_eq(PlineVertex::new(2.0, 2.0, 1.0)));
    /// assert!(result[1].fuzzy_eq(PlineVertex::new(3.0, 3.0, 0.5)));
    /// ```
    pub fn remove_repeat_pos(&self, pos_equal_eps: T) -> Cow<Polyline<T>> {
        if self.len() < 2 {
            return Cow::Borrowed(self);
        }

        let mut result: Option<Polyline<T>> = None;
        let mut prev_pos = self[0].pos();
        for (i, &v) in self.iter().enumerate().skip(1) {
            let is_repeat = v.pos().fuzzy_eq_eps(prev_pos, pos_equal_eps);

            if is_repeat {
                result
                    .get_or_insert_with(|| {
                        Polyline::from_iter(self.iter().take(i).copied(), self.is_closed)
                    })
                    .last_mut()
                    .unwrap()
                    .bulge = v.bulge;
            } else {
                if let Some(ref mut r) = result {
                    // not repeat position and result is initialized
                    r.add_vertex(v);
                }
                // else not repeat position and result is not initialized, do nothing
            }

            prev_pos = v.pos();
        }

        // check if closed and last repeats position on first
        if self.is_closed()
            && self
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(self[0].pos(), pos_equal_eps)
        {
            result
                .get_or_insert_with(|| Polyline::from_iter(self.iter().copied(), self.is_closed))
                .remove_last();
        }

        result.map_or_else(|| Cow::Borrowed(self), |r| Cow::Owned(r))
    }

    /// Remove all redundant vertexes from the polyline.
    ///
    /// Redundant vertexes can arise with multiple vertexes on top of each other, along a straight
    /// line, or along an arc with sweep angle less than or equal to PI.
    ///
    /// A `Cow<Polyline>` is returned to avoid allocation and copy in the case that no vertexes are
    /// removed.
    ///
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use std::borrow::Cow;
    /// // removing repeat vertexes along a line
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(2.0, 2.0, 0.0);
    /// polyline.add(3.0, 3.0, 0.0);
    /// polyline.add(3.0, 3.0, 0.0);
    /// polyline.add(4.0, 4.0, 0.0);
    /// polyline.add(2.0, 4.0, 0.0);
    /// let result = polyline.remove_redundant(1e-5);
    /// assert!(matches!(result, Cow::Owned(_)));
    /// assert_eq!(result.len(), 3);
    /// assert!(result.is_closed());
    /// assert!(result[0].fuzzy_eq(PlineVertex::new(2.0, 2.0, 0.0)));
    /// assert!(result[1].fuzzy_eq(PlineVertex::new(4.0, 4.0, 0.0)));
    /// assert!(result[2].fuzzy_eq(PlineVertex::new(2.0, 4.0, 0.0)));
    /// ```
    pub fn remove_redundant(&self, pos_equal_eps: T) -> Cow<Polyline<T>> {
        if self.len() < 2 {
            return Cow::Borrowed(self);
        }

        if self.len() == 2 {
            if self[0].pos().fuzzy_eq_eps(self[1].pos(), pos_equal_eps) {
                let mut result = Polyline::new();
                result.add_vertex(self[0]);
                result.set_is_closed(self.is_closed);
                return Cow::Owned(result);
            }
            return Cow::Borrowed(self);
        }

        // helper to test if v1->v2->v3 are collinear and all going in the same direction
        let is_collinear_same_dir = |v1: &PlineVertex<T>,
                                     v2: &PlineVertex<T>,
                                     v3: &PlineVertex<T>| {
            if v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps) {
                return true;
            }

            let collinear = (v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y))
                .fuzzy_eq_zero();

            if !collinear {
                return false;
            }

            let same_direction = (v3.pos() - v2.pos()).dot(v2.pos() - v1.pos()) > T::zero();

            same_direction
        };

        let mut result: Option<Polyline<T>> = None;
        let mut v1 = self[0];
        let mut v2 = self[1];
        let mut v1_v2_arc: Option<(T, Vector2<T>)> = None;
        let mut v1_bulge_is_zero = v1.bulge_is_zero();
        let mut v2_bulge_is_zero = v2.bulge_is_zero();
        let mut v1_bulge_is_pos = v1.bulge_is_pos();
        let mut v2_bulge_is_pos = v2.bulge_is_pos();

        // loop through processing/considering to discard the middle vertex v2
        for (i, &v3) in self.iter().cycle().enumerate().skip(2).take(self.len() - 1) {
            use RemoveRedundantCase::*;
            let state: RemoveRedundantCase<T> = if v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                // repeat position, just update bulge
                UpdateV1BulgeForRepeatPos
            } else if v1_bulge_is_zero && v2_bulge_is_zero {
                // two line segments in a row, check if collinear
                let is_final_vertex_for_open = !self.is_closed() && i == self.len();
                if !is_final_vertex_for_open && is_collinear_same_dir(&v1, &v2, &v3) {
                    DiscardVertex
                } else {
                    IncludeVertex
                }
            } else if !v1_bulge_is_zero
                && !v2_bulge_is_zero
                && (v1_bulge_is_pos == v2_bulge_is_pos)
                && !v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps)
            {
                // two arc segments in a row with same orientation, check if v2 can be removed by
                // updating v1 bulge
                let &mut (arc_radius1, arc_center1) =
                    v1_v2_arc.get_or_insert_with(|| seg_arc_radius_and_center(v1, v2));

                let (arc_radius2, arc_center2) = seg_arc_radius_and_center(v2, v3);

                if arc_radius1.fuzzy_eq(arc_radius2) && arc_center1.fuzzy_eq(arc_center2) {
                    let angle1 = angle(arc_center1, v1.pos());
                    let angle2 = angle(arc_center1, v2.pos());
                    let angle3 = angle(arc_center1, v3.pos());
                    let total_sweep =
                        delta_angle(angle1, angle2).abs() + delta_angle(angle2, angle3).abs();
                    if total_sweep <= T::pi() {
                        let bulge = if v1_bulge_is_pos {
                            bulge_from_angle(total_sweep)
                        } else {
                            -bulge_from_angle(total_sweep)
                        };
                        UpdateV1BulgeForArc(bulge)
                    } else {
                        IncludeVertex
                    }
                } else {
                    IncludeVertex
                }
            } else {
                IncludeVertex
            };

            let copy_self = |discard_last| {
                let count = if discard_last { i - 1 } else { i };
                if self[0].pos().fuzzy_eq_eps(self[1].pos(), pos_equal_eps) {
                    Polyline::from_iter(
                        self.iter().skip(1).take(count - 1).copied(),
                        self.is_closed,
                    )
                } else {
                    Polyline::from_iter(self.iter().take(count).copied(), self.is_closed)
                }
            };

            match state {
                IncludeVertex => {
                    if let Some(ref mut r) = result {
                        r.add_vertex(v2);
                    }
                    v1 = v2;
                    v2 = v3;
                    v1_v2_arc = None;
                    v1_bulge_is_zero = v2_bulge_is_zero;
                    v2_bulge_is_zero = v3.bulge_is_zero();
                    v1_bulge_is_pos = v2_bulge_is_pos;
                    v2_bulge_is_pos = v3.bulge_is_pos();
                }
                DiscardVertex => {
                    if result.is_none() {
                        result = Some(copy_self(true));
                    }

                    v2 = v3;
                }
                UpdateV1BulgeForRepeatPos => {
                    let p = result.get_or_insert_with(|| copy_self(false));
                    p.last_mut().unwrap().bulge = v2.bulge;
                    v1.bulge = v2.bulge;
                    v2 = v3;
                    v1_v2_arc = None;
                    v1_bulge_is_zero = v2_bulge_is_zero;
                    v2_bulge_is_zero = v3.bulge_is_zero();
                    v1_bulge_is_pos = v2_bulge_is_pos;
                    v2_bulge_is_pos = v3.bulge_is_pos();
                }
                UpdateV1BulgeForArc(bulge) => {
                    let p = result.get_or_insert_with(|| copy_self(true));
                    p.last_mut().unwrap().bulge = bulge;
                    v1.bulge = bulge;
                    v2 = v3;
                    v1_bulge_is_zero = v2_bulge_is_zero;
                    v2_bulge_is_zero = v3.bulge_is_zero();
                    v1_bulge_is_pos = v2_bulge_is_pos;
                    v2_bulge_is_pos = v3.bulge_is_pos();
                }
            }
        }

        if self.is_closed() {
            // handle wrap around middle vertex at start
            if self
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(self[0].pos(), pos_equal_eps)
            {
                // last repeats position on first
                result
                    .get_or_insert_with(|| {
                        Polyline::from_iter(self.iter().copied(), self.is_closed)
                    })
                    .remove_last();
            } else {
                // v1 => self.last()
                // v2 => self[0]
                let v3 = self[1];
                if v1_bulge_is_zero && v2_bulge_is_zero && is_collinear_same_dir(&v1, &v2, &v3) {
                    // first vertex is in middle of line
                    let p = result.get_or_insert_with(|| {
                        Polyline::from_iter(self.iter().copied(), self.is_closed)
                    });
                    p[0] = p.remove_last();
                } else if !v1_bulge_is_zero
                    && !v2_bulge_is_zero
                    && (v1_bulge_is_pos == v2_bulge_is_pos)
                    && !v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps)
                {
                    // check if arc can be simplified by removing first vertex
                    let &mut (arc_radius1, arc_center1) =
                        v1_v2_arc.get_or_insert_with(|| seg_arc_radius_and_center(v1, v2));

                    let (arc_radius2, arc_center2) = seg_arc_radius_and_center(v2, v3);

                    if arc_radius1.fuzzy_eq(arc_radius2) && arc_center1.fuzzy_eq(arc_center2) {
                        let angle1 = angle(arc_center1, v1.pos());
                        let angle2 = angle(arc_center1, v2.pos());
                        let angle3 = angle(arc_center1, v3.pos());
                        let total_sweep =
                            delta_angle(angle1, angle2).abs() + delta_angle(angle2, angle3).abs();
                        if total_sweep <= T::pi() {
                            let bulge = if v1_bulge_is_pos {
                                bulge_from_angle(total_sweep)
                            } else {
                                -bulge_from_angle(total_sweep)
                            };
                            let p = result.get_or_insert_with(|| {
                                Polyline::from_iter(self.iter().copied(), self.is_closed)
                            });
                            p[0] = p.remove_last();
                            p[0].bulge = bulge;
                        }
                    }
                }
            }
        }
        result.map_or_else(|| Cow::Borrowed(self), |r| Cow::Owned(r))
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

    /// Compute the parallel offset polylines of the polyline using default options.
    ///
    /// `offset` determines what offset polylines are generated, if it is positive then the
    /// direction of the offset is to the left of the polyline segment tangent vectors otherwise it
    /// is to the right.
    ///
    /// Algorithm will use [PlineOffsetOptions::default] for algorithm options.
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// let offset_plines = pline.parallel_offset(0.2);
    /// assert_eq!(offset_plines.len(), 1);
    /// let offset_pline = &offset_plines[0];
    /// assert!(offset_pline[0].fuzzy_eq(PlineVertex::new(0.2, 0.0, 1.0)));
    /// assert!(offset_pline[1].fuzzy_eq(PlineVertex::new(0.8, 0.0, 1.0)));
    /// ```
    pub fn parallel_offset(&self, offset: T) -> Vec<Polyline<T>> {
        self.parallel_offset_opt(offset, &Default::default())
    }

    /// Compute the parallel offset polylines of the polyline with options given.
    ///
    /// `offset` determines what offset polylines are generated, if it is positive then the
    /// direction of the offset is to the left of the polyline segment tangent vectors otherwise it
    /// is to the right.
    ///
    /// `options` is a struct that holds optional parameters. See
    /// [PlineOffsetOptions](crate::polyline::PlineOffsetOptions) for specific parameters.
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// let spatial_index = pline.create_approx_spatial_index().unwrap();
    /// let options = PlineOffsetOptions {
    ///     // setting option to handle possible self intersects in the polyline
    ///     handle_self_intersects: true,
    ///     // passing in existing spatial index of the polyline segments
    ///     spatial_index: Some(&spatial_index),
    ///     ..Default::default()
    /// };
    /// let offset_plines = pline.parallel_offset_opt(0.2, &options);
    /// assert_eq!(offset_plines.len(), 1);
    /// let offset_pline = &offset_plines[0];
    /// assert!(offset_pline[0].fuzzy_eq(PlineVertex::new(0.2, 0.0, 1.0)));
    /// assert!(offset_pline[1].fuzzy_eq(PlineVertex::new(0.8, 0.0, 1.0)));
    /// ```
    pub fn parallel_offset_opt(
        &self,
        offset: T,
        options: &PlineOffsetOptions<T>,
    ) -> Vec<Polyline<T>> {
        parallel_offset(self, offset, options)
    }

    /// Perform a boolean `operation` between this polyline and another using default options.
    ///
    pub fn boolean(&self, other: &Polyline<T>, operation: BooleanOp) -> BooleanResult<T> {
        self.boolean_opt(other, operation, &Default::default())
    }

    /// Perform a boolean `operation` between this polyline and another with options provided.
    pub fn boolean_opt(
        &self,
        other: &Polyline<T>,
        operation: BooleanOp,
        options: &PlineBooleanOptions<T>,
    ) -> BooleanResult<T> {
        polyline_boolean(self, other, operation, &options)
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

    /// Returns the orientation of the polyline.
    ///
    /// This method just uses the [Polyline::area] function to determine directionality of a closed
    /// polyline which may not yield a useful result if the polyline has self intersects.
    pub fn orientation(&self) -> PlineOrientation {
        if !self.is_closed {
            return PlineOrientation::Open;
        }

        if self.area() < T::zero() {
            PlineOrientation::Clockwise
        } else {
            PlineOrientation::CounterClockwise
        }
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
                    if !dist_to_arc_center_less_than_radius() {
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
            } else if !is_ccw
                && point_is_left
                && v2.x < point.x
                && point.x < v1.x
                && dist_to_arc_center_less_than_radius()
            {
                result -= 1;
            }
        }

        result
    }

    /// Calculate the winding number for a `point` relative to the polyline.
    ///
    /// The winding number calculates the number of turns/windings around a point that the polyline
    /// path makes. For a closed polyline without self intersects there are only three
    /// possibilities:
    ///
    /// * -1 (polyline winds around point clockwise)
    /// * 0 (point is outside the polyline)
    /// * 1 (polyline winds around the point counter clockwise).
    ///
    /// For a self intersecting closed polyline the winding number may be less than -1 (if the
    /// polyline winds around the point more than once in the counter clockwise direction) or
    /// greater than 1 (if the polyline winds around the point more than once in the clockwise
    /// direction).
    ///
    /// This function always returns 0 if polyline [Polyline::is_closed] is false.
    ///
    /// If the point lies directly on top of one of the polyline segments the result is not defined
    /// (it may return any integer). To handle the case of the point lying directly on the polyline
    /// [Polyline::closest_point] may be used to check if the distance from the point to the
    /// polyline is zero.
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

/// Internal type used by [Polyline::remove_redundant].
enum RemoveRedundantCase<T>
where
    T: Real,
{
    /// Include the vertex in the result.
    IncludeVertex,
    /// Discard the current vertex.
    DiscardVertex,
    /// Discard the current vertex and update the previous vertex bulge.
    UpdateV1BulgeForRepeatPos,
    /// Discard the current vertex and update the previous vertex bulge with the value computed.
    UpdateV1BulgeForArc(T),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Represents the orientation of a polyline.
pub enum PlineOrientation {
    /// Polyline is open.
    Open,
    /// Polyline is closed and directionally clockwise.
    Clockwise,
    /// Polyline is closed and directionally counter clockwise.
    CounterClockwise,
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

/// Struct to hold options parameters when performing polyline offset.
#[derive(Debug, Clone)]
pub struct PlineOffsetOptions<'a, T>
where
    T: Real,
{
    /// Spatial index of all the polyline segment bounding boxes (or boxes no smaller, e.g. using
    /// [Polyline::create_approx_spatial_index] is valid). If `None` is given then it will be
    /// computed internally. [Polyline::create_approx_spatial_index] or
    /// [Polyline::create_spatial_index] may be used to create the spatial index, the only
    /// restriction is that the spatial index bounding boxes must be at least big enough to contain
    /// the segments.
    pub spatial_index: Option<&'a StaticAABB2DIndex<T>>,
    /// If true then self intersects will be properly handled by the offset algorithm, if false then
    /// self intersecting polylines may not offset correctly. Handling self intersects of closed
    /// polylines requires more memory and computation.
    pub handle_self_intersects: bool,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// Fuzzy comparison epsilon used for determining if two positions are equal when stitching
    /// polyline slices together.
    pub slice_join_eps: T,
    /// Fuzzy comparison epsilon used when testing distance of slices to original polyline for
    /// validity.
    pub offset_dist_eps: T,
    /// Any offset polyline loop with length less than this value is culled (determined invalid).
    /// This can arise due to other epsilon values combined with the input given not catching very
    /// small invalid offsets.
    pub min_length_loop_cull: T,
}

impl<'a, T> PlineOffsetOptions<'a, T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            spatial_index: None,
            handle_self_intersects: false,
            pos_equal_eps: T::from(1e-5).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
            offset_dist_eps: T::from(1e-4).unwrap(),
            min_length_loop_cull: T::from(1e-2).unwrap(),
        }
    }
}

impl<'a, T> Default for PlineOffsetOptions<'a, T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Boolean operation to apply to polylines.
pub enum BooleanOp {
    /// Return the union of the polylines.
    OR,
    /// Return the intersection of the polylines.
    AND,
    /// Return the exclusion of a polyline from another.
    NOT,
    /// Exclusive OR between polylines.
    XOR,
}

/// Result of performing a boolean operation between two polylines.
pub struct BooleanResult<T> {
    /// Positive remaining space polylines.
    pub pos_plines: Vec<Polyline<T>>,
    /// Negative subtracted space polylines.
    pub neg_plines: Vec<Polyline<T>>,
}

impl<T> BooleanResult<T> {
    pub fn new() -> Self {
        Self {
            pos_plines: Vec::new(),
            neg_plines: Vec::new(),
        }
    }
}

impl<T> Default for BooleanResult<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct PlineBooleanOptions<'a, T>
where
    T: Real,
{
    pub pline1_spatial_index: Option<&'a StaticAABB2DIndex<T>>,
    pub pos_equal_eps: T,
    pub slice_join_eps: T,
}

impl<'a, T> PlineBooleanOptions<'a, T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            pline1_spatial_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
        }
    }
}

impl<'a, T> Default for PlineBooleanOptions<'a, T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
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
