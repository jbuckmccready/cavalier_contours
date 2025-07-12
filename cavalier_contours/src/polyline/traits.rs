use static_aabb2d_index::{
    AABB, IndexableNum, StaticAABB2DIndex, StaticAABB2DIndexBuildError, StaticAABB2DIndexBuilder,
};

use crate::{
    core::{
        Control,
        math::{
            Vector2, angle, angle_from_bulge, bulge_from_angle, delta_angle, dist_squared, is_left,
            is_left_or_equal, point_on_circle,
        },
        traits::{ControlFlow, FuzzyEq, FuzzyOrd, Real},
    },
    polyline::{
        PlineContainsOptions, PlineContainsResult, PlineIntersect, SelfIntersectsInclude,
        TwoPlinesIntersectVisitor, seg_arc_radius_and_center,
    },
};

use super::{
    BooleanOp, BooleanResult, ClosestPointResult, FindIntersectsOptions, PlineBooleanOptions,
    PlineIntersectVisitor, PlineIntersectsCollection, PlineOffsetOptions, PlineOrientation,
    PlineSelfIntersectOptions, PlineVertex, arc_seg_bounding_box,
    internal::{
        pline_boolean::polyline_boolean,
        pline_contains::polyline_contains,
        pline_intersects::{
            find_intersects, visit_global_self_intersects, visit_intersects,
            visit_local_self_intersects,
        },
        pline_offset::parallel_offset,
    },
    seg_bounding_box, seg_closest_point, seg_fast_approx_bounding_box, seg_length,
    seg_split_at_point,
};
use num_traits::One;
use num_traits::ToPrimitive;
use num_traits::Zero;
use num_traits::cast::NumCast;

/// Trait representing a readonly source of polyline data. This trait has all the methods and
/// operations that can be performed on a readonly polyline.
///
/// A polyline is a sequence of vertexes and a bool indicating whether the polyline is closed (last
/// vertex forms segment with first vertex) or open (no segment between last and first vertex).
/// Polylines can represent complex 2D shapes including straight line segments and circular arc
/// segments defined by bulge values. For related traits see [PlineSourceMut] and [PlineCreation].
///
/// Each vertex has a 2d xy position and bulge value. The bulge value determines the curvature of
/// the segment from this vertex to the next:
/// - A bulge of 0.0 creates a straight line segment
/// - A positive bulge creates a counter-clockwise arc
/// - A negative bulge creates a clockwise arc
/// - The magnitude of the bulge determines the arc's curvature
///
/// See [PlineVertex] for more information about vertex structure and bulge calculations.
pub trait PlineSource {
    /// Numeric type used for the polyline.
    type Num: Real;

    /// Type used for output when invoking methods that return a new polyline.
    type OutputPolyline: PlineCreation<Num = Self::Num>;

    /// Returns the number of user data values stored with this polyline.
    ///
    /// User data values are 64-bit unsigned integers that can be associated with polylines
    /// for storing custom application-specific data.
    fn get_userdata_count(&self) -> usize;

    /// Returns an iterator over all user data values stored with this polyline.
    ///
    /// User data values are 64-bit unsigned integers that can be associated with polylines
    /// for storing custom application-specific data.
    fn get_userdata_values(&self) -> impl Iterator<Item = u64> + '_;

    /// Total number of vertexes.
    fn vertex_count(&self) -> usize;

    /// Whether the polyline is closed (true) or open (false).
    fn is_closed(&self) -> bool;

    /// Get the vertex at given `index` position. Returns `None` if `index` out of bounds.
    fn get(&self, index: usize) -> Option<PlineVertex<Self::Num>>;

    /// Same as [PlineSource::get] but panics if `index` is out of bounds.
    ///
    /// # Panics
    ///
    /// Panics if `index` is out of bounds.
    fn at(&self, index: usize) -> PlineVertex<Self::Num>;

    /// Return iterator to iterate over all the polyline segments.
    #[inline]
    fn iter_segments(&self) -> SegmentIter<'_, Self> {
        SegmentIter::new(self)
    }

    /// Return iterator to iterate over all the polyline vertexes.
    #[inline]
    fn iter_vertexes(&self) -> VertexIter<'_, Self> {
        VertexIter::new(self)
    }

    /// Returns true if vertex count is 0.
    #[inline]
    fn is_empty(&self) -> bool {
        self.vertex_count() == 0
    }

    /// Fuzzy compare with another polyline using `eps` epsilon value for fuzzy comparison of
    /// vertexes.
    #[inline]
    fn fuzzy_eq_eps<P>(&self, other: &P, eps: Self::Num) -> bool
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.is_closed() == other.is_closed()
            && self.vertex_count() == other.vertex_count()
            && self
                .iter_vertexes()
                .zip(other.iter_vertexes())
                .all(|(v1, v2)| v1.fuzzy_eq_eps(v2, eps))
    }

    /// Same as [PlineSource::fuzzy_eq_eps] but uses default `Self::Num::fuzzy_epsilon()`.
    #[inline]
    fn fuzzy_eq<P>(&self, other: &P) -> bool
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.fuzzy_eq_eps(other, Self::Num::fuzzy_epsilon())
    }

    /// Get the last vertex of the polyline or `None` if polyline is empty.
    #[inline]
    fn last(&self) -> Option<PlineVertex<Self::Num>> {
        self.get(self.vertex_count() - 1)
    }

    /// Total number of segments in the polyline.
    #[inline]
    fn segment_count(&self) -> usize {
        let vc = self.vertex_count();
        if vc < 2 {
            0
        } else if self.is_closed() {
            vc
        } else {
            vc - 1
        }
    }

    /// Iterate through all the polyline segment vertex positional indexes.
    ///
    /// Segments are represented by polyline vertex pairs, for each vertex there is an associated
    /// positional index in the polyline, this method iterates through those positional indexes as
    /// segment pairs starting at (0, 1) and ending at (n-2, n-1) if open polyline or (n-1, 0) if
    /// closed polyline where n is the number of vertexes.
    #[inline]
    fn iter_segment_indexes(&self) -> PlineSegIndexIterator {
        PlineSegIndexIterator::new(self.vertex_count(), self.is_closed())
    }

    /// Returns the next wrapping vertex index for the polyline.
    ///
    /// This method treats the polyline as circular, so after the last vertex index,
    /// it wraps around to index 0. This is useful for traversing polylines in a
    /// circular manner regardless of whether they are closed or open.
    ///
    /// If `i + 1 >= self.vertex_count()` then 0 is returned, otherwise `i + 1` is returned.
    #[inline]
    fn next_wrapping_index(&self, i: usize) -> usize {
        let next = i + 1;
        if next >= self.vertex_count() { 0 } else { next }
    }

    /// Returns the previous wrapping vertex index for the polyline.
    ///
    /// This method treats the polyline as circular, so before the first vertex index (0),
    /// it wraps around to the last vertex index. This is useful for traversing polylines
    /// in a circular manner regardless of whether they are closed or open.
    ///
    /// If `i == 0` then `self.vertex_count() - 1` is returned, otherwise `i - 1` is returned.
    #[inline]
    fn prev_wrapping_index(&self, i: usize) -> usize {
        if i == 0 {
            self.vertex_count() - 1
        } else {
            i - 1
        }
    }

    /// Returns the forward wrapping distance between two vertex indexes.
    ///
    /// Assumes `start_index` is valid, debug asserts `start_index < self.len()`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// assert_eq!(polyline.fwd_wrapping_dist(0, 2), 2);
    /// assert_eq!(polyline.fwd_wrapping_dist(3, 1), 2);
    /// ```
    #[inline]
    fn fwd_wrapping_dist(&self, start_index: usize, end_index: usize) -> usize {
        let vc = self.vertex_count();

        debug_assert!(
            start_index < vc,
            "start_index is out of polyline range bounds"
        );

        if start_index <= end_index {
            end_index - start_index
        } else {
            vc - start_index + end_index
        }
    }

    /// Returns the vertex index after applying `offset` to `start_index` in a wrapping manner.
    ///
    /// Assumes `start_index` is valid, debug asserts `start_index < self.len()`.
    /// Assumes `offset` does not wrap multiple times, debug asserts `offset <= self.len()`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 0.0, 0.0);
    /// assert_eq!(polyline.fwd_wrapping_index(0, 2), 2);
    /// assert_eq!(polyline.fwd_wrapping_index(1, 2), 3);
    /// assert_eq!(polyline.fwd_wrapping_index(1, 3), 0);
    /// assert_eq!(polyline.fwd_wrapping_index(2, 3), 1);
    /// ```
    #[inline]
    fn fwd_wrapping_index(&self, start_index: usize, offset: usize) -> usize {
        let vc = self.vertex_count();

        debug_assert!(
            start_index < vc,
            "start_index is out of polyline range bounds"
        );

        debug_assert!(offset <= vc, "offset wraps multiple times");

        let sum = start_index + offset;
        if sum < vc { sum } else { sum - vc }
    }

    /// Compute the XY extents of the polyline.
    ///
    /// Returns `None` if polyline has less than 2 vertexes.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// let mut polyline = Polyline::new();
    /// assert_eq!(polyline.extents(), None);
    /// polyline.add(1.0, 1.0, 1.0);
    /// assert_eq!(polyline.extents(), None);
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
    fn extents(&self) -> Option<AABB<Self::Num>> {
        if self.segment_count() == 0 {
            return None;
        }

        let v1 = self.at(0);
        let mut result = AABB::new(v1.x, v1.y, v1.x, v1.y);

        for (v1, v2) in self.iter_segments() {
            if v1.bulge_is_zero() {
                // line segment, just look at end of line point (result seeded with first point)
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
    #[inline]
    fn path_length(&self) -> Self::Num {
        self.iter_segments()
            .fold(Self::Num::zero(), |acc, (v1, v2)| acc + seg_length(v1, v2))
    }

    /// Compute the closed signed area of the polyline.
    ///
    /// If [PlineSource::is_closed] is false (open polyline) then 0.0 is always returned.
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
    /// polyline.invert_direction_mut();
    /// assert!(polyline.area().fuzzy_eq(-std::f64::consts::PI));
    /// ```
    fn area(&self) -> Self::Num {
        use num_traits::real::Real;
        if !self.is_closed() {
            return Self::Num::zero();
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
        let mut double_total_area = Self::Num::zero();

        for (v1, v2) in self.iter_segments() {
            double_total_area = double_total_area + v1.x * v2.y - v1.y * v2.x;
            if !v1.bulge_is_zero() {
                // add arc segment area
                let b = v1.bulge.abs();
                let sweep_angle = angle_from_bulge(b);
                let triangle_base = (v2.pos() - v1.pos()).length();
                let radius = triangle_base * ((b * b + Self::Num::one()) / (Self::Num::four() * b));
                let sagitta = b * triangle_base / Self::Num::two();
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

        double_total_area / Self::Num::two()
    }

    /// Returns the orientation of the polyline.
    ///
    /// This method just uses the [PlineSource::area] function to determine directionality of a closed
    /// polyline which may not yield a useful result if the polyline has self intersects.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(1.0, 0.0, 1.0);
    /// assert_eq!(polyline.orientation(), PlineOrientation::CounterClockwise);
    /// polyline.invert_direction_mut();
    /// assert_eq!(polyline.orientation(), PlineOrientation::Clockwise);
    /// ```
    fn orientation(&self) -> PlineOrientation {
        if !self.is_closed() {
            return PlineOrientation::Open;
        }

        if self.area() < Self::Num::zero() {
            PlineOrientation::Clockwise
        } else {
            PlineOrientation::CounterClockwise
        }
    }

    /// Remove all repeat position vertexes from the polyline.
    ///
    /// Returns `None` to avoid allocation and copy in the case that no vertexes are removed.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(2.0, 2.0, 1.0);
    /// polyline.add(3.0, 3.0, 1.0);
    /// polyline.add(3.0, 3.0, 0.5);
    /// let result = polyline.remove_repeat_pos(1e-5).expect("repeat position vertexes were removed");
    /// assert_eq!(result.vertex_count(), 2);
    /// assert!(result[0].fuzzy_eq(PlineVertex::new(2.0, 2.0, 1.0)));
    /// assert!(result[1].fuzzy_eq(PlineVertex::new(3.0, 3.0, 0.5)));
    /// ```
    fn remove_repeat_pos(&self, pos_equal_eps: Self::Num) -> Option<Self::OutputPolyline> {
        if self.vertex_count() < 2 {
            return None;
        }

        let mut result: Option<Self::OutputPolyline> = None;
        let mut prev_pos = self.at(0).pos();
        for (i, v) in self.iter_vertexes().enumerate().skip(1) {
            let is_repeat = v.pos().fuzzy_eq_eps(prev_pos, pos_equal_eps);

            if is_repeat {
                // repeat position just update bulge (remove vertex by not adding it to result)
                let r = result.get_or_insert_with(|| {
                    Self::OutputPolyline::from_iter(self.iter_vertexes().take(i), self.is_closed())
                });
                let last = r.last().unwrap();
                r.set_last(last.with_bulge(v.bulge));
            } else {
                if let Some(ref mut r) = result {
                    // not repeat position and result is initialized
                    r.add_vertex(v);
                }
                // else not repeat position and result is not initialized, do nothing

                // update previous position for next iteration
                prev_pos = v.pos();
            }
        }

        // check if is_closed and last repeats position on first
        if self.is_closed()
            && self
                .last()
                .unwrap()
                .pos()
                .fuzzy_eq_eps(self.at(0).pos(), pos_equal_eps)
        {
            result
                .get_or_insert_with(|| {
                    Self::OutputPolyline::from_iter(self.iter_vertexes(), self.is_closed())
                })
                .remove_last();
        }

        result
    }

    /// Remove all redundant vertexes from the polyline.
    ///
    /// Redundant vertexes can arise with multiple vertexes on top of each other, along a straight
    /// line, or forming a concentric arc with sweep angle less than or equal to PI.
    ///
    /// Returns `None` to avoid allocation and copy in the case that no vertexes are removed.
    ///
    /// # Examples
    ///
    /// ### Removing repeat vertexes along a line
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(2.0, 2.0, 0.0);
    /// polyline.add(3.0, 3.0, 0.0);
    /// polyline.add(3.0, 3.0, 0.0);
    /// polyline.add(4.0, 4.0, 0.0);
    /// polyline.add(2.0, 4.0, 0.0);
    /// let result = polyline.remove_redundant(1e-5).expect("redundant vertexes were removed");
    /// assert_eq!(result.vertex_count(), 3);
    /// assert!(result.is_closed());
    /// assert!(result[0].fuzzy_eq(PlineVertex::new(2.0, 2.0, 0.0)));
    /// assert!(result[1].fuzzy_eq(PlineVertex::new(4.0, 4.0, 0.0)));
    /// assert!(result[2].fuzzy_eq(PlineVertex::new(2.0, 4.0, 0.0)));
    /// ```
    ///
    /// ### Simplifying a circle defined by 5 vertexes
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let bulge = (std::f64::consts::PI / 8.0).tan();
    /// let mut polyline = Polyline::new_closed();
    /// polyline.add(-0.5, 0.0, bulge);
    /// polyline.add(0.0, -0.5, bulge);
    /// // repeat vertex thrown in
    /// polyline.add(0.0, -0.5, bulge);
    /// polyline.add(0.5, 0.0, bulge);
    /// polyline.add(0.0, 0.5, bulge);
    /// let result = polyline.remove_redundant(1e-5).expect("redundant vertexes were removed");
    /// assert_eq!(result.vertex_count(), 2);
    /// assert!(result.is_closed());
    /// assert!(result[0].fuzzy_eq(PlineVertex::new(-0.5, 0.0, 1.0)));
    /// assert!(result[1].fuzzy_eq(PlineVertex::new(0.5, 0.0, 1.0)));
    /// ```
    fn remove_redundant(&self, pos_equal_eps: Self::Num) -> Option<Self::OutputPolyline> {
        use num_traits::real::Real;
        let vc = self.vertex_count();
        if vc < 2 {
            return None;
        }

        if vc == 2 {
            let v1 = self.at(0);
            let v2 = self.at(1);
            if v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                let mut result = Self::OutputPolyline::with_capacity(1, self.is_closed());
                result.add_vertex(v2); // take bulge from last vertex
                return Some(result);
            }
            return None;
        }

        // helper to test if v1->v2->v3 are collinear and all going in the same direction
        let is_collinear_same_dir =
            |v1: &PlineVertex<Self::Num>,
             v2: &PlineVertex<Self::Num>,
             v3: &PlineVertex<Self::Num>| {
                // check if v2 on top of v3 (considered collinear for the purposes of discarding v2)
                if v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps) {
                    return true;
                }

                let collinear =
                    (v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y))
                        .fuzzy_eq_zero_eps(pos_equal_eps);
                let same_direction =
                    (v3.pos() - v2.pos()).dot(v2.pos() - v1.pos()) > -pos_equal_eps;

                collinear && same_direction
            };

        let mut v1 = self.at(0);
        let mut v2 = self.at(1);

        // remove all repeat positions at the start
        let mut i = 2;
        while v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
            v1.bulge = v2.bulge;
            // check for reaching the end of polyline
            if i >= vc {
                break;
            }
            v2 = self.at(i);
            i += 1;
        }

        let mut result: Option<Self::OutputPolyline> = if i == 2 {
            None
        } else {
            let mut pl = Self::OutputPolyline::with_capacity(1, self.is_closed());
            pl.add_vertex(v1);
            Some(pl)
        };
        // if end is reached return polyline with the only vertex
        if i >= vc {
            return result;
        }

        let mut v1_v2_arc: Option<(Self::Num, Vector2<Self::Num>)> = None;
        let mut v1_bulge_is_zero = v1.bulge_is_zero();
        let mut v2_bulge_is_zero = v2.bulge_is_zero();
        let mut v1_bulge_is_pos = v1.bulge_is_pos();
        let mut v2_bulge_is_pos = v2.bulge_is_pos();

        let iter_count = if self.is_closed() { vc - 1 } else { vc - 2 };

        enum RemoveRedundantCase<U>
        where
            U: Real,
        {
            /// Include the vertex in the result.
            IncludeVertex,
            /// Discard the current vertex.
            DiscardVertex,
            /// Discard the current vertex and update the previous vertex bulge with the value computed.
            UpdateV1BulgeForArc(U),
        }

        // loop through processing/considering to discard the middle vertex v2
        for (i, v3) in self
            .iter_vertexes()
            .cycle()
            .enumerate()
            .skip(i)
            .take(iter_count)
        {
            use RemoveRedundantCase::*;
            let state: RemoveRedundantCase<Self::Num> =
                if v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps) {
                    // repeat position, just update bulge
                    DiscardVertex
                } else if v1_bulge_is_zero && v2_bulge_is_zero {
                    // two line segments in a row, check if collinear
                    let is_final_vertex_for_open = !self.is_closed() && i == vc;
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

                    if arc_radius1.fuzzy_eq_eps(arc_radius2, pos_equal_eps)
                        && arc_center1.fuzzy_eq_eps(arc_center2, pos_equal_eps)
                    {
                        let angle1 = angle(arc_center1, v1.pos());
                        let angle2 = angle(arc_center1, v2.pos());
                        let angle3 = angle(arc_center1, v3.pos());
                        let total_sweep =
                            delta_angle(angle1, angle2).abs() + delta_angle(angle2, angle3).abs();

                        let avg_radius = (arc_radius1 + arc_radius2) / Self::Num::two();

                        // can only combine vertexes if total sweep will still be less than PI
                        // multiplying by average radius for fuzzy compare to have numbers in scale
                        // of epsilon
                        if (avg_radius * total_sweep)
                            .fuzzy_lt_eps(avg_radius * Self::Num::pi(), pos_equal_eps)
                        {
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

            let copy_self = || {
                Self::OutputPolyline::from_iter(self.iter_vertexes().take(i - 1), self.is_closed())
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
                        result = Some(copy_self());
                    }

                    v2 = v3;
                    v1_v2_arc = None;
                    v2_bulge_is_zero = v3.bulge_is_zero();
                    v2_bulge_is_pos = v3.bulge_is_pos();
                }
                UpdateV1BulgeForArc(bulge) => {
                    let p = result.get_or_insert_with(copy_self);
                    let last = p.last().unwrap();
                    p.set_last(last.with_bulge(bulge));
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
            match result.as_mut() {
                Some(pl) => {
                    if pl
                        .last()
                        .unwrap()
                        .pos()
                        .fuzzy_eq_eps(pl.at(0).pos(), pos_equal_eps)
                    {
                        pl.remove_last();
                    }
                }
                None => {
                    if self
                        .last()
                        .unwrap()
                        .pos()
                        .fuzzy_eq_eps(self.at(0).pos(), pos_equal_eps)
                    {
                        // last repeats position on first
                        result
                            .get_or_insert_with(|| {
                                Self::OutputPolyline::from_iter(
                                    self.iter_vertexes(),
                                    self.is_closed(),
                                )
                            })
                            .remove_last();
                    }
                }
            }

            // v1 => last
            // v2 => first
            // v3 => second
            let v3 = match result.as_ref() {
                Some(pl) => pl.at(1),
                None => self.at(1),
            };
            if v1_bulge_is_zero && v2_bulge_is_zero && is_collinear_same_dir(&v1, &v2, &v3) {
                // first vertex is in middle of line
                let p = result.get_or_insert_with(|| {
                    Self::OutputPolyline::from_iter(self.iter_vertexes(), self.is_closed())
                });
                let last = p.remove_last();
                p.set_vertex(0, last);
            } else if !v1_bulge_is_zero
                && !v2_bulge_is_zero
                && (v1_bulge_is_pos == v2_bulge_is_pos)
                && !v2.pos().fuzzy_eq_eps(v3.pos(), pos_equal_eps)
            {
                // check if arc can be simplified by removing first vertex
                let &mut (arc_radius1, arc_center1) =
                    v1_v2_arc.get_or_insert_with(|| seg_arc_radius_and_center(v1, v2));

                let (arc_radius2, arc_center2) = seg_arc_radius_and_center(v2, v3);

                if arc_radius1.fuzzy_eq_eps(arc_radius2, pos_equal_eps)
                    && arc_center1.fuzzy_eq_eps(arc_center2, pos_equal_eps)
                {
                    let angle1 = angle(arc_center1, v1.pos());
                    let angle2 = angle(arc_center1, v2.pos());
                    let angle3 = angle(arc_center1, v3.pos());
                    let total_sweep =
                        delta_angle(angle1, angle2).abs() + delta_angle(angle2, angle3).abs();

                    let avg_radius = (arc_radius1 + arc_radius2) / Self::Num::two();
                    if (avg_radius * total_sweep)
                        .fuzzy_lt_eps(avg_radius * Self::Num::pi(), pos_equal_eps)
                    {
                        let bulge = if v1_bulge_is_pos {
                            bulge_from_angle(total_sweep)
                        } else {
                            -bulge_from_angle(total_sweep)
                        };
                        let p = result.get_or_insert_with(|| {
                            Self::OutputPolyline::from_iter(self.iter_vertexes(), self.is_closed())
                        });
                        let last = p.remove_last();
                        p.set_vertex(0, last.with_bulge(bulge));
                    }
                }
            }
        } else {
            // handle adding last vertex
            match result.as_mut() {
                Some(pl) => {
                    pl.add_or_replace_vertex(self.last().unwrap(), pos_equal_eps);
                }
                None => {
                    if self.at(vc - 2).fuzzy_eq_eps(self.at(vc - 1), pos_equal_eps) {
                        result
                            .get_or_insert_with(|| {
                                Self::OutputPolyline::from_iter(
                                    self.iter_vertexes(),
                                    self.is_closed(),
                                )
                            })
                            .remove_last();
                    }
                }
            }
        }

        result
    }

    /// Rotates the vertexes in a closed polyline such that the first vertex's position is at
    /// `point`. `start_index` indicates which segment `point` lies on before rotation. This does
    /// not change the shape of the polyline curve. `pos_equal_eps` is epsilon value used for
    /// comparing the positions of points. `None` is returned if the polyline is not closed, the
    /// polyline length is less than 2, or the `start_index` is out of bounds.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline = Polyline::new_closed();
    /// assert!(matches!(polyline.rotate_start(0, Vector2::new(0.0, 0.0), 1e-5), None));
    /// polyline.add(0.0, 0.0, 0.0);
    /// assert!(matches!(polyline.rotate_start(0, Vector2::new(0.0, 0.0), 1e-5), None));
    /// polyline.add(1.0, 0.0, 0.0);
    /// polyline.add(1.0, 1.0, 0.0);
    /// polyline.add(0.0, 1.0, 0.0);
    ///
    /// let rot = polyline.rotate_start(0, Vector2::new(0.5, 0.0), 1e-5).unwrap();
    /// let mut expected_rot = Polyline::new_closed();
    /// expected_rot.add(0.5, 0.0, 0.0);
    /// expected_rot.add(1.0, 0.0, 0.0);
    /// expected_rot.add(1.0, 1.0, 0.0);
    /// expected_rot.add(0.0, 1.0, 0.0);
    /// expected_rot.add(0.0, 0.0, 0.0);
    /// assert!(rot.fuzzy_eq(&expected_rot));
    /// ```
    fn rotate_start(
        &self,
        start_index: usize,
        point: Vector2<Self::Num>,
        pos_equal_eps: Self::Num,
    ) -> Option<Self::OutputPolyline> {
        let vc = self.vertex_count();
        if !self.is_closed() || vc < 2 || start_index > vc - 1 {
            return None;
        }

        let wrapping_vertexes_starting_at = |start: usize| {
            self.iter_vertexes()
                .skip(start)
                .take(vc - start)
                .chain(self.iter_vertexes().take(start))
        };

        let start_v = self.at(start_index);
        // Note: using with_capacity to ensure exact allocation required for the end result (avoids
        // over allocating and resize allocations)
        let result = if start_v.pos().fuzzy_eq_eps(point, pos_equal_eps) {
            // point lies on top of start index vertex
            let mut r = Self::OutputPolyline::with_capacity(vc, true);
            r.extend_vertexes(wrapping_vertexes_starting_at(start_index));
            r
        } else {
            // check if it's at the end of the segment, if it is then use that next index
            let next_index = self.next_wrapping_index(start_index);
            if point.fuzzy_eq_eps(self.at(next_index).pos(), pos_equal_eps) {
                let mut r = Self::OutputPolyline::with_capacity(vc, true);
                r.extend_vertexes(wrapping_vertexes_starting_at(next_index));
                r
            } else {
                // must split at the point
                let mut r = Self::OutputPolyline::with_capacity(vc + 1, true);
                let split = seg_split_at_point(
                    self.at(start_index),
                    self.at(next_index),
                    point,
                    pos_equal_eps,
                );
                r.add_vertex(split.split_vertex);
                r.extend_vertexes(wrapping_vertexes_starting_at(next_index));
                r.set_last(split.updated_start);
                r
            }
        };

        Some(result)
    }

    /// Creates a fast approximate spatial index of all the polyline segments.
    ///
    /// The starting vertex index position is used as the key to the segment bounding box in the
    /// `StaticAABB2DIndex`. The bounding boxes are guaranteed to be no smaller than the actual
    /// bounding box of the segment but may be larger, this is done for performance. If you want the
    /// actual bounding box index use [PlineSource::create_aabb_index] instead.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16`.
    fn create_approx_aabb_index(&self) -> StaticAABB2DIndex<Self::Num> {
        let vc = self.vertex_count();
        if vc < 2 {
            return unwrap_spatial_index(StaticAABB2DIndexBuilder::new(0));
        }

        let seg_count = if self.is_closed() { vc } else { vc - 1 };

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

        unwrap_spatial_index(builder)
    }

    /// Creates a spatial index of all the polyline segments.
    ///
    /// The starting vertex index position is used as the key to the segment bounding box in the
    /// `StaticAABB2DIndex`. The bounding boxes are the actual bounding box of the segment, for
    /// performance reasons you may want to use [PlineSource::create_approx_aabb_index].
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16`.
    fn create_aabb_index(&self) -> StaticAABB2DIndex<Self::Num> {
        let vc = self.vertex_count();
        if vc < 2 {
            return unwrap_spatial_index(StaticAABB2DIndexBuilder::new(0));
        }

        let seg_count = if self.is_closed() { vc } else { vc - 1 };

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

        unwrap_spatial_index(builder)
    }

    /// Find the closest segment point on a polyline to a `point` given.
    ///
    /// If the polyline is empty then `None` is returned.
    ///
    /// `pos_equal_eps` is epsilon value used for fuzzy float comparisons.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline: Polyline = Polyline::new();
    /// assert!(matches!(polyline.closest_point(Vector2::zero(), 1e-5), None));
    /// polyline.add(1.0, 1.0, 1.0);
    /// let result = polyline.closest_point(Vector2::new(1.0, 0.0), 1e-5).unwrap();
    /// assert_eq!(result.seg_start_index, 0);
    /// assert!(result.seg_point.fuzzy_eq(polyline[0].pos()));
    /// assert!(result.distance.fuzzy_eq(1.0));
    /// ```
    fn closest_point(
        &self,
        point: Vector2<Self::Num>,
        pos_equal_eps: Self::Num,
    ) -> Option<ClosestPointResult<Self::Num>> {
        use num_traits::real::Real;
        if self.is_empty() {
            return None;
        }

        let mut result = ClosestPointResult {
            seg_start_index: 0,
            seg_point: self.at(0).pos(),
            distance: Real::max_value(),
        };

        if self.vertex_count() == 1 {
            result.distance = (result.seg_point - point).length();
            return Some(result);
        }

        let mut dist_squared = Real::max_value();

        for (i, j) in self.iter_segment_indexes() {
            let v1 = self.at(i);
            let v2 = self.at(j);
            let cp = seg_closest_point(v1, v2, point, pos_equal_eps);
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
    /// This function always returns 0 if polyline [PlineSource::is_closed] is false.
    ///
    /// If the point lies directly on top of one of the polyline segments the result is not defined
    /// (it may return any integer). To handle the case of the point lying directly on the polyline
    /// [PlineSource::closest_point] may be used to check if the distance from the point to the
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
    /// polyline.invert_direction_mut();
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
    /// polyline.invert_direction_mut();
    /// assert_eq!(polyline.winding_number(Vector2::new(1.0, 0.0)), -2);
    /// ```
    fn winding_number(&self, point: Vector2<Self::Num>) -> i32 {
        if !self.is_closed() || self.vertex_count() < 2 {
            return 0;
        }

        // Helper function for processing a line segment when computing the winding number.
        let process_line_winding =
            |v1: PlineVertex<Self::Num>, v2: PlineVertex<Self::Num>, point: Vector2<Self::Num>| {
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
            };

        // Helper function for processing an arc segment when computing the winding number.
        let process_arc_winding =
            |v1: PlineVertex<Self::Num>, v2: PlineVertex<Self::Num>, point: Vector2<Self::Num>| {
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
            };

        let mut winding = 0;

        for (v1, v2) in self.iter_segments() {
            if v1.bulge_is_zero() {
                winding += process_line_winding(v1, v2, point);
            } else {
                winding += process_arc_winding(v1, v2, point);
            }
        }

        winding
    }

    /// Returns a new polyline with all arc segments converted to line segments with some
    /// `error_distance` or `None` if `Self::Num` fails to cast to or from usize.
    ///
    /// `error_distance` is the maximum distance from any line segment to the arc it is
    /// approximating. Line segments are circumscribed by the arc (all line end points lie on the
    /// arc path).
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// // half circle
    /// polyline.add(0.0, 0.0, 1.0);
    /// polyline.add(2.0, 0.0, 0.0);
    /// let lines = polyline.arcs_to_approx_lines(0.1).unwrap();
    /// assert!(lines.vertex_count() > 2);
    /// assert!(lines.iter_vertexes().all(|v| v.bulge == 0.0));
    /// ```
    fn arcs_to_approx_lines(&self, error_distance: Self::Num) -> Option<Self::OutputPolyline> {
        use num_traits::real::Real;
        let mut result = Self::OutputPolyline::with_capacity(0, self.is_closed());

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
                result.add(v1.x, v1.y, Self::Num::zero());
                continue;
            }

            let start_angle = angle(arc_center, v1.pos());
            let end_angle = angle(arc_center, v2.pos());
            let angle_diff = delta_angle(start_angle, end_angle).abs();

            let seg_sub_angle =
                Self::Num::two() * (Self::Num::one() - abs_error / arc_radius).acos().abs();
            let seg_count = (angle_diff / seg_sub_angle).ceil();
            // create angle offset such that all lines have an equal part of the arc
            let seg_angle_offset = if v1.bulge_is_neg() {
                -angle_diff / seg_count
            } else {
                angle_diff / seg_count
            };

            // add start vertex
            result.add(v1.x, v1.y, Self::Num::zero());
            let usize_count = seg_count.to_usize()?;
            // add all vertex points along arc
            for i in 1..usize_count {
                let angle_pos = <Self::Num as NumCast>::from(i)?;
                let angle = angle_pos * seg_angle_offset + start_angle;
                let pos = point_on_circle(arc_radius, arc_center, angle);
                result.add(pos.x, pos.y, Self::Num::zero());
            }
        }

        if !self.is_closed() {
            // add the final missing vertex in the case that the polyline is not closed
            result.add_vertex(self.last().unwrap());
        }

        Some(result)
    }

    /// Visit self intersects of the polyline using default options.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 2.0, 0.0);
    /// polyline.add(1.0, 1.0, 0.0);
    /// polyline.add(-1.0, 1.0, 0.0);
    ///
    /// let mut visited_intersects = 0;
    /// // NOTE: FnMut(PlineIntersect) implements PlineInteresectVisitor trait
    /// polyline.visit_self_intersects(&mut |intersect: PlineIntersect<f64>| {
    ///    visited_intersects += 1;
    ///    match intersect {
    ///        PlineIntersect::Basic(intr) => assert!(intr.point.fuzzy_eq_eps(Vector2::new(0.0, 1.0), 1e-5)),
    ///        PlineIntersect::Overlapping(_) => panic!("Unexpected overlapping intersection"),
    ///    }
    ///    // stop visiting intersects on first intersect found by returning Control::Break
    ///    // NOTE: use Control::Continue or return () to continue visiting
    ///    Control::Break(())
    /// });
    ///
    /// assert_eq!(visited_intersects, 1);
    /// ```
    #[inline]
    fn visit_self_intersects<C, V>(&self, visitor: &mut V) -> C
    where
        C: ControlFlow,
        V: PlineIntersectVisitor<Self::Num, C>,
    {
        self.visit_self_intersects_opt(visitor, &Default::default())
    }

    /// Visit self intersects of the polyline using options provided.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    fn visit_self_intersects_opt<C, V>(
        &self,
        visitor: &mut V,
        options: &PlineSelfIntersectOptions<Self::Num>,
    ) -> C
    where
        C: ControlFlow,
        V: PlineIntersectVisitor<Self::Num, C>,
    {
        if self.vertex_count() < 2 {
            return C::continuing();
        }

        if options.include == SelfIntersectsInclude::Local {
            // local intersects only
            return visit_local_self_intersects(self, visitor, options.pos_equal_eps);
        }

        let constructed_index;
        let index = if let Some(x) = options.aabb_index {
            x
        } else {
            constructed_index = self.create_approx_aabb_index();
            &constructed_index
        };

        if options.include == SelfIntersectsInclude::Global {
            // global intersects only
            return visit_global_self_intersects(self, index, visitor, options.pos_equal_eps);
        }

        // else all intersects
        try_cf!(visit_local_self_intersects(
            self,
            visitor,
            options.pos_equal_eps
        ));

        visit_global_self_intersects(self, index, visitor, options.pos_equal_eps)
    }

    /// Visit all intersects between two polylines using default options.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    #[inline]
    fn visit_intersects<P, V, C>(&self, other: &P, visitor: &mut V)
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
        V: TwoPlinesIntersectVisitor<Self::Num, C>,
        C: ControlFlow,
    {
        self.visit_intersects_opt(other, visitor, &Default::default());
    }

    /// Visit all intersects between two polylines using the options provided.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    #[inline]
    fn visit_intersects_opt<P, V, C>(
        &self,
        other: &P,
        visitor: &mut V,
        options: &FindIntersectsOptions<Self::Num>,
    ) where
        P: PlineSource<Num = Self::Num> + ?Sized,
        V: TwoPlinesIntersectVisitor<Self::Num, C>,
        C: ControlFlow,
    {
        visit_intersects(self, other, visitor, options);
    }

    /// Scan for self intersects using default options.
    /// Returns true on the first one found; false if there are none.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 2.0, 0.0);
    /// polyline.add(1.0, 1.0, 0.0);
    /// polyline.add(-1.0, 1.0, 0.0);
    ///
    /// assert!(polyline.scan_for_self_intersect());
    /// ```
    #[inline]
    fn scan_for_self_intersect(&self) -> bool {
        self.scan_for_self_intersect_opt(&Default::default())
    }

    /// Scan for self intersects using options provided.
    /// Returns true on the first one found; false if there are none.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::*;
    /// # use cavalier_contours::core::math::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(0.0, 2.0, 0.0);
    /// polyline.add(1.0, 1.0, 0.0);
    /// polyline.add(-1.0, 1.0, 0.0);
    ///
    /// assert!(polyline.scan_for_self_intersect_opt(&Default::default()));
    /// ```
    fn scan_for_self_intersect_opt(&self, options: &PlineSelfIntersectOptions<Self::Num>) -> bool {
        let mut found_intersects = false;
        self.visit_self_intersects_opt(
            &mut |_intersect: PlineIntersect<Self::Num>| {
                found_intersects = true;
                Control::Break(())
            },
            options,
        );
        found_intersects
    }

    /// Find all intersects between two polylines using default options.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    #[inline]
    fn find_intersects<P>(&self, other: &P) -> PlineIntersectsCollection<Self::Num>
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.find_intersects_opt(other, &Default::default())
    }

    /// Find all intersects between two polylines using the options provided.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    #[inline]
    fn find_intersects_opt<P>(
        &self,
        other: &P,
        options: &FindIntersectsOptions<Self::Num>,
    ) -> PlineIntersectsCollection<Self::Num>
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        find_intersects(self, other, options)
    }

    /// Compute the parallel offset polylines of the polyline using default options.
    ///
    /// `offset` determines what offset polylines are generated, if it is positive then the
    /// direction of the offset is to the left of the polyline segment tangent vectors otherwise it
    /// is to the right.
    ///
    /// Algorithm will use [PlineOffsetOptions::default] for algorithm options.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
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
    fn parallel_offset(&self, offset: Self::Num) -> Vec<Self::OutputPolyline> {
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
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// let aabb_index = pline.create_approx_aabb_index();
    /// let options = PlineOffsetOptions {
    ///     // setting option to handle possible self intersects in the polyline
    ///     handle_self_intersects: true,
    ///     // passing in existing spatial index of the polyline segments
    ///     aabb_index: Some(&aabb_index),
    ///     ..Default::default()
    /// };
    /// let offset_plines = pline.parallel_offset_opt(0.2, &options);
    /// assert_eq!(offset_plines.len(), 1);
    /// let offset_pline = &offset_plines[0];
    /// assert!(offset_pline[0].fuzzy_eq(PlineVertex::new(0.2, 0.0, 1.0)));
    /// assert!(offset_pline[1].fuzzy_eq(PlineVertex::new(0.8, 0.0, 1.0)));
    /// ```
    fn parallel_offset_opt(
        &self,
        offset: Self::Num,
        options: &PlineOffsetOptions<Self::Num>,
    ) -> Vec<Self::OutputPolyline> {
        parallel_offset(self, offset, options)
    }

    /// Perform a boolean `operation` between this polyline and another using default options.
    ///
    /// See [PlineSource::boolean_opt] for more information.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// let rectangle = pline_closed![
    ///     (-1.0, -2.0, 0.0),
    ///     (3.0, -2.0, 0.0),
    ///     (3.0, 2.0, 0.0),
    ///     (-1.0, 2.0, 0.0),
    /// ];
    /// let circle = pline_closed![(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)];
    /// let results = rectangle.boolean(&circle, BooleanOp::Not);
    /// // since the circle is inside the rectangle we get back 1 positive polyline and 1 negative
    /// // polyline where the positive polyline is the rectangle and the negative polyline is the
    /// // circle
    /// assert_eq!(results.pos_plines.len(), 1);
    /// assert_eq!(results.neg_plines.len(), 1);
    /// assert!(matches!(results.result_info, BooleanResultInfo::Pline2InsidePline1));
    /// assert!(results.pos_plines[0].pline.area().fuzzy_eq(rectangle.area()));
    /// assert!(results.neg_plines[0].pline.area().fuzzy_eq(circle.area()));
    /// ```
    fn boolean<P>(&self, other: &P, operation: BooleanOp) -> BooleanResult<Self::OutputPolyline>
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.boolean_opt(other, operation, &Default::default())
    }

    /// Perform a boolean `operation` between this polyline and another with options provided.
    ///
    /// Returns the boolean result polylines and their associated slices that were stitched together
    /// end to end to form them. For the result `pline1` refers to `self`, and `pline2` refers to
    /// `other`.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// let rectangle = pline_closed![
    ///     (-1.0, -2.0, 0.0),
    ///     (3.0, -2.0, 0.0),
    ///     (3.0, 2.0, 0.0),
    ///     (-1.0, 2.0, 0.0),
    /// ];
    /// let circle = pline_closed![(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)];
    /// let aabb_index = rectangle.create_approx_aabb_index();
    /// let options = PlineBooleanOptions {
    ///     // passing in existing spatial index of the polyline segments for the first polyline
    ///     pline1_aabb_index: Some(&aabb_index),
    ///     ..Default::default()
    /// };
    /// let results = rectangle.boolean_opt(&circle, BooleanOp::Not, &options);
    /// // since the circle is inside the rectangle we get back 1 positive polyline and 1 negative
    /// // polyline where the positive polyline is the rectangle and the negative polyline is the
    /// // circle
    /// assert_eq!(results.pos_plines.len(), 1);
    /// assert_eq!(results.neg_plines.len(), 1);
    /// assert!(matches!(results.result_info, BooleanResultInfo::Pline2InsidePline1));
    /// assert!(results.pos_plines[0].pline.area().fuzzy_eq(rectangle.area()));
    /// assert!(results.neg_plines[0].pline.area().fuzzy_eq(circle.area()));
    /// ```
    fn boolean_opt<P>(
        &self,
        other: &P,
        operation: BooleanOp,
        options: &PlineBooleanOptions<Self::Num>,
    ) -> BooleanResult<Self::OutputPolyline>
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        polyline_boolean(self, other, operation, options)
    }

    /// Determine if this polyline fully contains another using default options.
    ///
    /// Caution: Polylines with self-intersections may generate unexpected results.
    /// Use scan_for_self_intersect() to find and reject self-intersecting polylines
    /// if this is a possibility for your input data.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// # use cavalier_contours::polyline::PlineContainsResult::*;
    /// let rectangle = pline_closed![
    ///     (-2.0, -2.0, 0.0),
    ///     (2.0, -2.0, 0.0),
    ///     (2.0, 2.0, 0.0),
    ///     (-2.0, 2.0, 0.0),
    /// ];
    /// let circle = pline_closed![(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// let triangle = pline_closed![(3.1340, 4.5, 0.0), (4.0, 3.0, 0.0), (4.8660, 4.5, 0.0)];
    ///
    /// // since the circle is inside the rectangle we get back Pline2InsidePline1
    /// assert_eq!(rectangle.contains(&circle), Pline2InsidePline1);
    /// // since the rectangle is outside the circle, but containing, it we get back Pline1InsidePline2
    /// assert_eq!(circle.contains(&rectangle), Pline1InsidePline2);
    /// // since the triangle is outside the rectangle, and not containing it, we get back Disjoint
    /// assert_eq!(rectangle.contains(&triangle), Disjoint);
    /// ```
    fn contains<P>(&self, other: &P) -> PlineContainsResult
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.contains_opt(other, &Default::default())
    }

    /// Determine if this polyline fully contains another with options provided.
    ///
    /// Caution: Polylines with self-intersections may generate unexpected results.
    /// Use scan_for_self_intersect() to find and reject self-intersecting polylines
    /// if this is a possibility for your input data.
    ///
    /// # Panics
    ///
    /// Panics if `Self::Num` type fails to cast to/from a `u16` (required for spatial index).
    ///
    /// # Examples
    /// ```
    /// # use cavalier_contours::core::traits::*;
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::pline_closed;
    /// # use cavalier_contours::polyline::{PlineContainsOptions, PlineContainsResult::*};
    /// let rectangle = pline_closed![
    ///     (-2.0, -2.0, 0.0),
    ///     (2.0, -2.0, 0.0),
    ///     (2.0, 2.0, 0.0),
    ///     (-2.0, 2.0, 0.0),
    /// ];
    /// let circle = pline_closed![(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0)];
    /// let triangle = pline_closed![(3.1340, 4.5, 0.0), (4.0, 3.0, 0.0), (4.8660, 4.5, 0.0)];
    ///
    /// let rectangle_aabb_index = rectangle.create_approx_aabb_index();
    /// let rectangle_options = PlineContainsOptions {
    ///     // passing in existing spatial index of the polyline segments for the first polyline
    ///     pline1_aabb_index: Some(&rectangle_aabb_index),
    ///     ..Default::default()
    /// };
    /// // since the circle is inside the rectangle we get back Pline2InsidePline1
    /// assert_eq!(rectangle.contains_opt(&circle, &rectangle_options), Pline2InsidePline1);
    ///
    /// let circle_aabb_index = circle.create_approx_aabb_index();
    /// let circle_options = PlineContainsOptions {
    ///     // passing in existing spatial index of the polyline segments for the first polyline
    ///     pline1_aabb_index: Some(&circle_aabb_index),
    ///     ..Default::default()
    /// };
    /// // since the rectangle is outside the circle, but containing, it we get back Pline1InsidePline2
    /// assert_eq!(circle.contains_opt(&rectangle, &circle_options), Pline1InsidePline2);
    ///
    /// // since the triangle is outside the rectangle, and not containing it, we get back Disjoint
    /// assert_eq!(rectangle.contains_opt(&triangle, &rectangle_options), Disjoint);
    ///
    /// ```
    fn contains_opt<P>(
        &self,
        other: &P,
        options: &PlineContainsOptions<Self::Num>,
    ) -> PlineContainsResult
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        polyline_contains(self, other, options)
    }

    /// Find the segment index and point on the polyline corresponding to the path length given.
    ///
    /// Returns `Ok((0, first_vertex_position))` if `target_path_length` is negative.
    ///
    /// Returns `Ok((seg_index, point))` if `target_path_length` is less than or equal to the
    /// polyline's total path length. Where `seg_index` is the index of the segment the point lies
    /// on, e.g. if point is on the second segment of the polyline then `seg_index = 1`.
    ///
    /// Returns `Err((total_path_length))` if `target_path_length` is greater than total path
    /// length of the polyline.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// # use cavalier_contours::core::math::Vector2;
    /// # use cavalier_contours::core::traits::FuzzyEq;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.add(10.0, 0.0, 0.0);
    /// let (seg_index, point) = polyline.find_point_at_path_length(5.0).unwrap();
    /// assert_eq!(seg_index, 0);
    /// assert!(point.fuzzy_eq(Vector2::new(5.0, 0.0)));
    /// ```
    fn find_point_at_path_length(
        &self,
        target_path_length: Self::Num,
    ) -> Result<(usize, Vector2<Self::Num>), Self::Num> {
        if target_path_length <= Self::Num::zero() {
            return Ok((0, self.at(0).pos()));
        }

        let mut acc_length = Self::Num::zero();
        for (i, (v1, v2)) in self.iter_segments().enumerate() {
            let seg_len = seg_length(v1, v2);
            let sum_len = acc_length + seg_len;
            if sum_len < target_path_length {
                acc_length = sum_len;
                continue;
            }

            // parametric value (from 0 to 1) along the segment where the point lies
            let t = (target_path_length - acc_length) / seg_len;

            if v1.bulge_is_zero() {
                // line segment
                let pt = v1.pos() + (v2.pos() - v1.pos()).scale(t);
                return Ok((i, pt));
            } else {
                // arc segment
                let (radius, center) = seg_arc_radius_and_center(v1, v2);
                let start_angle = angle(center, v1.pos());
                let total_sweep_angle = angle_from_bulge(v1.bulge);
                let target_angle = start_angle + total_sweep_angle * t;

                let pt = point_on_circle(radius, center, target_angle);
                return Ok((i, pt));
            }
        }

        Err(acc_length)
    }
}

/// Trait representing a mutable source of polyline data. This trait has all the methods and
/// operations that can be performed on a mutable polyline.
///
/// See other core polyline traits: [PlineSource] and [PlineCreation] for more information.
pub trait PlineSourceMut: PlineSource {
    /// Clears all existing user data values and replaces them with the provided values.
    ///
    /// User data values are 64-bit unsigned integers that can be associated with polylines
    /// for storing custom application-specific data.
    ///
    /// # Parameters
    ///
    /// * `values` - An iterator of `u64` values to set as the new user data
    fn set_userdata_values(&mut self, values: impl IntoIterator<Item = u64>);

    /// Appends additional user data values to the existing user data storage.
    ///
    /// User data values are 64-bit unsigned integers that can be associated with polylines
    /// for storing custom application-specific data.
    ///
    /// # Parameters
    ///
    /// * `values` - An iterator of `u64` values to append to the existing user data
    fn add_userdata_values(&mut self, values: impl IntoIterator<Item = u64>);

    /// Set the vertex data at the given `index` position of the polyline.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// polyline.set_vertex(0, PlineVertex::new(1.0, 1.0, 1.0));
    /// assert!(polyline.at(0).fuzzy_eq(PlineVertex::new(1.0, 1.0, 1.0)));
    /// ```
    fn set_vertex(&mut self, index: usize, vertex: PlineVertex<Self::Num>);

    /// Same as [PlineSourceMut::set_vertex] but accepts each component of the vertex rather than a
    /// vertex structure.
    #[inline]
    fn set(&mut self, index: usize, x: Self::Num, y: Self::Num, bulge: Self::Num) {
        self.set_vertex(index, PlineVertex::new(x, y, bulge))
    }

    /// Set the last vertex of the polyline.
    ///
    /// # Panics
    ///
    /// Panics if polyline is empty.
    #[inline]
    fn set_last(&mut self, vertex: PlineVertex<Self::Num>) {
        self.set_vertex(self.vertex_count() - 1, vertex);
    }

    /// Insert a new vertex into the polyline at the given `index` position.
    fn insert_vertex(&mut self, index: usize, vertex: PlineVertex<Self::Num>);

    /// Same as [PlineSourceMut::insert_vertex] but accepts each component of the vertex rather than
    /// a vertex structure.
    #[inline]
    fn insert(&mut self, index: usize, x: Self::Num, y: Self::Num, bulge: Self::Num) {
        self.insert_vertex(index, PlineVertex::new(x, y, bulge));
    }

    /// Remove vertex at the given `index` position and return it.
    fn remove(&mut self, index: usize) -> PlineVertex<Self::Num>;

    /// Remove the last vertex from the polyline and return it.
    ///
    /// # Panics
    ///
    /// Panics if polyline is empty.
    #[inline]
    fn remove_last(&mut self) -> PlineVertex<Self::Num> {
        self.remove(self.vertex_count() - 1)
    }

    /// Clear all vertexes of the polyline.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.0);
    /// assert_eq!(polyline.vertex_count(), 1);
    /// polyline.clear();
    /// assert_eq!(polyline.vertex_count(), 0);
    /// ```
    fn clear(&mut self);

    /// Add a vertex to the end of the polyline.
    fn add_vertex(&mut self, vertex: PlineVertex<Self::Num>);

    /// Same as [PlineSourceMut::add_vertex] but accepts each component of the vertex rather than a
    /// vertex structure.
    #[inline]
    fn add(&mut self, x: Self::Num, y: Self::Num, bulge: Self::Num) {
        self.add_vertex(PlineVertex::new(x, y, bulge));
    }

    /// Same as [PlineSourceMut::add_vertex] but accepts each component as elements in an array,
    /// 0 = x, 1 = y, 2 = bulge.
    #[inline]
    fn add_from_array(&mut self, data: [Self::Num; 3]) {
        self.add(data[0], data[1], data[2]);
    }

    /// Append all vertexes from an iterator to the end of this polyline.
    fn extend_vertexes<I>(&mut self, vertexes: I)
    where
        I: IntoIterator<Item = PlineVertex<Self::Num>>;

    /// Copy all vertexes from `other` to the end of this polyline.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline1 = Polyline::new();
    /// polyline1.add(0.0, 0.0, 0.0);
    /// let mut polyline2 = Polyline::new();
    /// polyline2.add(1.0, 1.0, 0.0);
    /// polyline1.extend(&polyline2);
    /// assert_eq!(polyline1.vertex_count(), 2);
    /// assert!(polyline1.at(1).fuzzy_eq(PlineVertex::new(1.0, 1.0, 0.0)));
    /// ```
    #[inline]
    fn extend<P>(&mut self, other: &P)
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.extend_vertexes(other.iter_vertexes());
    }

    /// Same as [PlineSourceMut::extend] but removes any consecutive repeat position vertexes in the
    /// process of copying (using `pos_equal_eps` for compare).
    #[inline]
    fn extend_remove_repeat<P>(&mut self, other: &P, pos_equal_eps: Self::Num)
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        self.reserve(other.vertex_count());
        for v in other.iter_vertexes() {
            self.add_or_replace_vertex(v, pos_equal_eps)
        }
    }

    /// Reserves capacity for at least `additional` more vertexes.
    fn reserve(&mut self, additional: usize);

    /// Add a vertex if it's position is not fuzzy equal to the last vertex in the polyline.
    ///
    /// If the vertex position is fuzzy equal then just update the bulge of the last vertex with
    /// the bulge given.
    #[inline]
    fn add_or_replace_vertex(&mut self, vertex: PlineVertex<Self::Num>, pos_equal_eps: Self::Num) {
        let vc = self.vertex_count();
        if vc == 0 {
            self.add_vertex(vertex);
            return;
        }

        let last = self.at(vc - 1);
        if last.pos().fuzzy_eq_eps(vertex.pos(), pos_equal_eps) {
            self.set_vertex(vc - 1, last.with_bulge(vertex.bulge));
            return;
        }

        self.add_vertex(vertex);
    }

    /// Same as [PlineSourceMut::add_or_replace_vertex] but accepts each component of the vertex
    /// rather than a vertex structure.
    #[inline]
    fn add_or_replace(
        &mut self,
        x: Self::Num,
        y: Self::Num,
        bulge: Self::Num,
        pos_equal_eps: Self::Num,
    ) {
        self.add_or_replace_vertex(PlineVertex::new(x, y, bulge), pos_equal_eps);
    }

    /// Set whether the polyline is closed (`is_closed = true`) or open (`is_closed = false`).
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline: Polyline = Polyline::new();
    /// assert!(!polyline.is_closed());
    /// polyline.set_is_closed(true);
    /// assert!(polyline.is_closed());
    /// ```
    fn set_is_closed(&mut self, is_closed: bool);

    /// Uniformly scale the polyline (mutably) in the xy plane by `scale_factor`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(4.0, 4.0, 1.0);
    /// polyline.scale_mut(2.0);
    /// let mut expected = Polyline::new();
    /// expected.add(4.0, 4.0, 0.5);
    /// expected.add(8.0, 8.0, 1.0);
    /// assert!(polyline.fuzzy_eq(&expected));
    /// ```
    fn scale_mut(&mut self, scale_factor: Self::Num) {
        for i in 0..self.vertex_count() {
            let v = self.at(i);
            self.set(i, scale_factor * v.x, scale_factor * v.y, v.bulge);
        }
    }

    /// Translate the polyline (mutably) by some `x_offset` and `y_offset`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(2.0, 2.0, 0.5);
    /// polyline.add(4.0, 4.0, 1.0);
    /// polyline.translate_mut(-3.0, 1.0);
    /// let mut expected = Polyline::new();
    /// expected.add(-1.0, 3.0, 0.5);
    /// expected.add(1.0, 5.0, 1.0);
    /// assert!(polyline.fuzzy_eq(&expected));
    /// ```
    fn translate_mut(&mut self, x: Self::Num, y: Self::Num) {
        for i in 0..self.vertex_count() {
            let v = self.at(i);
            self.set(i, v.x + x, v.y + y, v.bulge);
        }
    }

    /// Invert/reverse the direction of the polyline in place (mutably).
    ///
    /// This method works by simply reversing the order of the vertexes, shifting by 1 position all
    /// the vertexes, and inverting the sign of all the bulge values. E.g. after reversing the
    /// vertex the bulge at index 0 becomes negative bulge at index 1. The end result for a is_closed
    /// polyline is the direction will be changed from clockwise to counter clockwise or vice versa.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::polyline::*;
    /// let mut polyline = Polyline::new();
    /// polyline.add(0.0, 0.0, 0.5);
    /// polyline.add(1.0, 1.0, 0.0);
    /// polyline.invert_direction_mut();
    /// let mut expected = Polyline::new();
    /// expected.add(1.0, 1.0, -0.5);
    /// expected.add(0.0, 0.0, 0.5);
    /// assert!(polyline.fuzzy_eq(&expected));
    /// ```
    fn invert_direction_mut(&mut self) {
        let vc = self.vertex_count();
        if vc < 2 {
            return;
        }

        let mut start = 0;
        let mut end = vc - 1;
        while start < end {
            let s = self.at(start);
            let e = self.at(end);
            self.set_vertex(start, e);
            self.set_vertex(end, s);
            start += 1;
            end -= 1;
        }

        let first_bulge = self.at(0).bulge;
        for i in 1..vc {
            let b = -self.at(i).bulge;
            self.set_vertex(i - 1, self.at(i - 1).with_bulge(b));
        }

        if self.is_closed() {
            self.set_vertex(vc - 1, self.at(vc - 1).with_bulge(-first_bulge));
        }
    }
}

/// Trait representing a creatable source of polyline data. This trait acts as a mutable polyline
/// source and also exposes associated functions for construction. This trait is used when new
/// polylines need to be returned from a function.
///
/// See other core polyline traits: [PlineSource] and [PlineSourceMut] for more information.
pub trait PlineCreation: PlineSourceMut + Sized {
    /// Create a new empty polyline with `capacity` given and `is_closed` indicating whether it is
    /// a closed or open polyline.
    fn with_capacity(capacity: usize, is_closed: bool) -> Self;

    /// Create a new polyline by constructing from vertexes given by an iterator, `is_closed` sets
    /// whether the created polyline is closed or open.
    fn from_iter<I>(iter: I, is_closed: bool) -> Self
    where
        I: Iterator<Item = PlineVertex<Self::Num>>;

    /// Create a new polyline by cloning from an existing polyline.
    #[inline]
    fn create_from<P>(pline: &P) -> Self
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        let mut result = Self::from_iter(pline.iter_vertexes(), pline.is_closed());

        result.set_userdata_values(pline.get_userdata_values());
        result
    }

    /// Same as [PlineCreation::create_from] but removes any repeat position vertexes in the
    /// process using `pos_equal_eps` for positional comparisons.
    #[inline]
    fn create_from_remove_repeat<P>(pline: &P, pos_equal_eps: Self::Num) -> Self
    where
        P: PlineSource<Num = Self::Num> + ?Sized,
    {
        let mut result = Self::with_capacity(pline.vertex_count(), pline.is_closed());
        for v in pline.iter_vertexes() {
            result.add_or_replace_vertex(v, pos_equal_eps);
        }

        if pline.is_closed() && result.vertex_count() >= 2 {
            // catch last position overlapping first for closed polyline case
            let last = result.last().unwrap();
            if last.pos().fuzzy_eq_eps(result.at(0).pos(), pos_equal_eps) {
                result.remove_last();
            }
        }

        result.set_userdata_values(pline.get_userdata_values());
        result
    }

    /// Create empty polyline with `is_closed` set to false.
    #[inline]
    fn empty() -> Self {
        Self::with_capacity(0, false)
    }
}

/// An iterator that traverses polyline vertexes.
#[derive(Debug)]
pub struct VertexIter<'a, P>
where
    P: ?Sized,
{
    pline: &'a P,
    pos: usize,
    end: usize,
}

impl<'a, P> VertexIter<'a, P>
where
    P: PlineSource + ?Sized,
{
    #[inline]
    pub fn new(pline: &'a P) -> Self {
        Self {
            pline,
            pos: 0,
            end: pline.vertex_count(),
        }
    }
}

impl<P> Clone for VertexIter<'_, P>
where
    P: ?Sized,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            pline: self.pline,
            pos: self.pos,
            end: self.end,
        }
    }
}

impl<P> Iterator for VertexIter<'_, P>
where
    P: PlineSource + ?Sized,
{
    type Item = PlineVertex<P::Num>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.pos == self.end {
            return None;
        }
        let r = self.pline.get(self.pos);
        self.pos += 1;
        r
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.end - self.pos;
        (remaining, Some(remaining))
    }
}

impl<P> ExactSizeIterator for VertexIter<'_, P>
where
    P: PlineSource,
{
    #[inline]
    fn len(&self) -> usize {
        let (lower, upper) = self.size_hint();
        assert_eq!(upper, Some(lower));
        lower
    }
}

impl<P> DoubleEndedIterator for VertexIter<'_, P>
where
    P: PlineSource + ?Sized,
{
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.pos == self.end {
            return None;
        }

        let r = self.pline.get(self.end - 1);
        self.end -= 1;
        r
    }
}

/// An iterator that traverses polyline segments (as pairs of vertexes).
#[derive(Debug)]
pub struct SegmentIter<'a, P>
where
    P: ?Sized,
{
    pline: &'a P,
    pos: usize,
    exhausted: bool,
}

impl<'a, P> SegmentIter<'a, P>
where
    P: PlineSource + ?Sized,
{
    #[inline]
    pub fn new(pline: &'a P) -> Self {
        Self {
            pline,
            pos: 0,
            exhausted: pline.vertex_count() < 2,
        }
    }
}

impl<P> Clone for SegmentIter<'_, P>
where
    P: ?Sized,
{
    #[inline]
    fn clone(&self) -> Self {
        Self {
            pline: self.pline,
            pos: self.pos,
            exhausted: self.exhausted,
        }
    }
}

impl<P> Iterator for SegmentIter<'_, P>
where
    P: PlineSource + ?Sized,
{
    type Item = (PlineVertex<P::Num>, PlineVertex<P::Num>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.exhausted {
            return None;
        }

        let vc = self.pline.vertex_count();
        if self.pos == vc - 1 {
            if self.pline.is_closed() {
                self.exhausted = true;
                return Some((self.pline.at(vc - 1), self.pline.at(0)));
            } else {
                return None;
            }
        }

        let r = (self.pline.at(self.pos), self.pline.at(self.pos + 1));
        self.pos += 1;
        Some(r)
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        if self.exhausted {
            (0, Some(0))
        } else {
            let remaining = if self.pline.is_closed() {
                self.pline.vertex_count() - self.pos
            } else {
                self.pline.vertex_count() - self.pos - 1
            };
            (remaining, Some(remaining))
        }
    }
}

impl<P> ExactSizeIterator for SegmentIter<'_, P>
where
    P: PlineSource,
{
    #[inline]
    fn len(&self) -> usize {
        let (lower, upper) = self.size_hint();
        assert_eq!(upper, Some(lower));
        lower
    }
}
/// An iterator that traverses all segment vertex pair index positions.
pub struct PlineSegIndexIterator {
    pos: usize,
    remaining: usize,
    is_closed: bool,
}

impl PlineSegIndexIterator {
    #[inline]
    pub fn new(vertex_count: usize, is_closed: bool) -> PlineSegIndexIterator {
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

/// Helper function to unwrap a spatial index from a builder or panic for the unexpected case of
/// failure.
fn unwrap_spatial_index<T>(builder: StaticAABB2DIndexBuilder<T>) -> StaticAABB2DIndex<T>
where
    T: IndexableNum,
{
    match builder.build() {
        Ok(x) => x,
        Err(e) => match e {
            StaticAABB2DIndexBuildError::ItemCountError { .. } => {
                unreachable!("internal library error: count mismatch when building spatial index")
            }
            StaticAABB2DIndexBuildError::NumericCastError => {
                panic!("failed to cast Self::Num type: {e}")
            }
        },
    }
}
