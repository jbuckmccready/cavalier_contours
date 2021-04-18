//! Supporting public types used in [Polyline] methods.

use super::{PlineVertex, Polyline};
use crate::core::{math::Vector2, traits::Real};
use static_aabb2d_index::StaticAABB2DIndex;

/// Represents the orientation of a polyline.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
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
    /// [Polyline::create_approx_aabb_index] is valid). If `None` is given then it will be
    /// computed internally. [Polyline::create_approx_aabb_index] or
    /// [Polyline::create_aabb_index] may be used to create the spatial index, the only
    /// restriction is that the spatial index bounding boxes must be at least big enough to contain
    /// the segments.
    pub aabb_index: Option<&'a StaticAABB2DIndex<T>>,
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
}

impl<'a, T> PlineOffsetOptions<'a, T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            aabb_index: None,
            handle_self_intersects: false,
            pos_equal_eps: T::from(1e-5).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
            offset_dist_eps: T::from(1e-4).unwrap(),
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
    Or,
    /// Return the intersection of the polylines.
    And,
    /// Return the exclusion of a polyline from another.
    Not,
    /// Exclusive OR between polylines.
    Xor,
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
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    pub pos_equal_eps: T,
    pub slice_join_eps: T,
}

impl<'a, T> PlineBooleanOptions<'a, T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            pline1_aabb_index: None,
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

/// Enum to control which self intersects to include.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SelfIntersectsInclude {
    /// Include all (local and global) self intersects.
    All,
    /// Include only local self intersects (defined as being between two adjacent polyline
    /// segments).
    Local,
    /// Include only global self intersects (defined as being between two non-adjacent polyline
    /// segments).
    Global,
}

#[derive(Debug)]
pub struct PlineSelfIntersectOptions<'a, T>
where
    T: Real,
{
    pub aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    pub pos_equal_eps: T,
    pub include: SelfIntersectsInclude,
}

impl<'a, T> PlineSelfIntersectOptions<'a, T>
where
    T: Real,
{
    pub fn new() -> Self {
        Self {
            aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
            include: SelfIntersectsInclude::All,
        }
    }
}

impl<'a, T> Default for PlineSelfIntersectOptions<'a, T>
where
    T: Real,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Represents a polyline intersect at a single point.
#[derive(Debug, Clone, Copy)]
pub struct PlineBasicIntersect<T> {
    /// Starting vertex index of the first polyline segment involved in the intersect.
    pub start_index1: usize,
    /// Starting vertex index of the second polyline segment involved in the intersect.
    pub start_index2: usize,
    /// Point at which the intersect occurs.
    pub point: Vector2<T>,
}

impl<T> PlineBasicIntersect<T> {
    pub fn new(start_index1: usize, start_index2: usize, point: Vector2<T>) -> Self {
        Self {
            start_index1,
            start_index2,
            point,
        }
    }
}

/// Represents an overlapping polyline intersect segment.
#[derive(Debug, Clone, Copy)]
pub struct PlineOverlappingIntersect<T> {
    /// Starting vertex index of the first polyline segment involved in the overlapping intersect.
    pub start_index1: usize,
    /// Starting vertex index of the second polyline segment involved in the intersect.
    pub start_index2: usize,
    /// First end point of the overlapping intersect (closest to the second segment start).
    pub point1: Vector2<T>,
    /// Second end point of the overlapping intersect (furthest from the second segment start).
    pub point2: Vector2<T>,
}

impl<T> PlineOverlappingIntersect<T> {
    pub fn new(
        start_index1: usize,
        start_index2: usize,
        point1: Vector2<T>,
        point2: Vector2<T>,
    ) -> Self {
        Self {
            start_index1,
            start_index2,
            point1,
            point2,
        }
    }
}

/// Represents a polyline intersect that may be either a [PlineBasicIntersect] or
/// [PlineOverlappingIntersect].
#[derive(Debug, Clone, Copy)]
pub enum PlineIntersect<T> {
    Basic(PlineBasicIntersect<T>),
    Overlapping(PlineOverlappingIntersect<T>),
}

impl<T> PlineIntersect<T> {
    pub fn new_basic(start_index1: usize, start_index2: usize, point: Vector2<T>) -> Self {
        PlineIntersect::Basic(PlineBasicIntersect::new(start_index1, start_index2, point))
    }

    pub fn new_overlapping(
        start_index1: usize,
        start_index2: usize,
        point1: Vector2<T>,
        point2: Vector2<T>,
    ) -> Self {
        PlineIntersect::Overlapping(PlineOverlappingIntersect::new(
            start_index1,
            start_index2,
            point1,
            point2,
        ))
    }
}

/// Represents a collection of basic and overlapping polyline intersects.
#[derive(Debug, Clone)]
pub struct PlineIntersectsCollection<T> {
    pub basic_intersects: Vec<PlineBasicIntersect<T>>,
    pub overlapping_intersects: Vec<PlineOverlappingIntersect<T>>,
}

impl<T> PlineIntersectsCollection<T> {
    pub fn new(
        basic_intersects: Vec<PlineBasicIntersect<T>>,
        overlapping_intersects: Vec<PlineOverlappingIntersect<T>>,
    ) -> Self {
        Self {
            basic_intersects,
            overlapping_intersects,
        }
    }
    pub fn new_empty() -> Self {
        Self::new(Vec::new(), Vec::new())
    }
}

/// Holds information regarding a slice from some source polyline.
#[derive(Debug, Clone)]
pub struct PlineSlice<T> {
    /// Source polyline start segment index.
    pub start_index: usize,
    /// Source polyline end segment index.
    pub end_index: usize,
    /// First vertex of the slice (positioned somewhere along the `start_index` segment with bulge
    /// and position updated).
    pub updated_start: PlineVertex<T>,
    /// Second to last vertex of the slice (positioned somewhere along the `end_index` segment with
    /// bulge and position updated).
    pub updated_end: PlineVertex<T>,
    /// Final end point of the slice.
    pub end_point: Vector2<T>,
}

impl<T> PlineSlice<T>
where
    T: Real,
{
    pub fn new(
        start_index: usize,
        end_index: usize,
        updated_start: PlineVertex<T>,
        updated_end: PlineVertex<T>,
        end_point: Vector2<T>,
    ) -> Self {
        Self {
            start_index,
            end_index,
            updated_start,
            updated_end,
            end_point,
        }
    }

    /// Helper method used to check if collapsed/singularity slice.
    pub fn is_collapsed(&self, pos_equal_eps: T) -> bool {
        self.start_index == self.end_index
            && self
                .updated_start
                .pos()
                .fuzzy_eq_eps(self.end_point, pos_equal_eps)
    }

    /// Total index distance traversed by the slice (forward wrapping distance from start_index to
    /// end_index based on source polyline).
    pub fn get_index_dist(&self, source: &Polyline<T>) -> usize {
        source.forward_wrapping_dist(self.start_index, self.end_index)
    }

    /// Sufficient vertex count required to build the slice into a polyline (may be one more than
    /// actually in the slice).
    pub fn max_vertex_count(&self, source: &Polyline<T>) -> usize {
        self.get_index_dist(source) + 2
    }

    /// Construct an open polyline that represents this slice (source reference required for vertex
    /// data).
    pub fn to_polyline(&self, source: &Polyline<T>, pos_equal_eps: T) -> Polyline<T> {
        let vertex_count = self.max_vertex_count(source);
        let mut result = Polyline::with_capacity(vertex_count, false);

        let mut visitor = |v| {
            result.add_vertex(v);
            true
        };
        self.visit_vertexes(source, pos_equal_eps, &mut visitor);
        result
    }

    /// Visit all vertexes in this slice (source reference required for vertex data) until the slice
    /// has been fully traversed or the visitor returns false.
    pub fn visit_vertexes<F>(&self, source: &Polyline<T>, pos_equal_eps: T, visitor: &mut F)
    where
        F: FnMut(PlineVertex<T>) -> bool,
    {
        let index_dist = self.get_index_dist(source);

        if index_dist == 0 {
            if !visitor(self.updated_start) {
                return;
            }
            if !visitor(PlineVertex::from_vector2(self.end_point, T::zero())) {
                return;
            }
        } else {
            if !visitor(self.updated_start) {
                return;
            }

            let iter = source
                .iter()
                .cycle()
                .skip(self.start_index + 1)
                .take(index_dist - 1);

            for v in iter {
                if !visitor(*v) {
                    return;
                }
            }

            if self
                .updated_end
                .pos()
                .fuzzy_eq_eps(self.end_point, pos_equal_eps)
            {
                visitor(PlineVertex::from_vector2(self.end_point, T::zero()));
            } else {
                if !visitor(self.updated_end) {
                    return;
                }
                visitor(PlineVertex::from_vector2(self.end_point, T::zero()));
            }
        }
    }
}