//! Supporting public types used in the core polyline trait methods.

use super::{internal::pline_intersects::OverlappingSlice, PlineVertex, PlineView, PlineViewData};
use crate::{
    core::{
        math::Vector2,
        traits::{ControlFlow, Real},
    },
    polyline::{PlineCreation, PlineSource, ViewDataValidation},
};
use static_aabb2d_index::StaticAABB2DIndex;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

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

/// Result from calling [PlineSource::closest_point].
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
    /// [PlineSource::create_approx_aabb_index] is valid). If `None` is given then it will be
    /// computed internally. [PlineSource::create_approx_aabb_index] or
    /// [PlineSource::create_aabb_index] may be used to create the spatial index, the only
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
    #[inline]
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
    #[inline]
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

#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(rename_all = "camelCase"),
    serde(bound(
        serialize = "P: Serialize, P::Num: Serialize",
        deserialize = "P: Deserialize<'de>, P::Num: Deserialize<'de>",
    ))
)]
/// Represents one of the polyline results from a boolean operation between two polylines.
#[derive(Debug, Clone, Default)]
pub struct BooleanResultPline<P>
where
    P: PlineCreation,
{
    /// Resultant polyline.
    pub pline: P,
    /// Indexes of the slices that were stitched together to form the `pline`.
    pub subslices: Vec<BooleanPlineSlice<P::Num>>,
}

impl<P> BooleanResultPline<P>
where
    P: PlineCreation,
{
    #[inline]
    pub fn new(pline: P, subslices: Vec<BooleanPlineSlice<P::Num>>) -> Self {
        Self { pline, subslices }
    }
}

#[derive(Debug, Clone)]
/// Result of performing a boolean operation between two polylines.
pub struct BooleanResult<P>
where
    P: PlineCreation,
{
    /// Positive remaining space polylines and associated slice indexes.
    pub pos_plines: Vec<BooleanResultPline<P>>,
    /// Negative subtracted space polylines and associated slice indexes.
    pub neg_plines: Vec<BooleanResultPline<P>>,
}

impl<P> BooleanResult<P>
where
    P: PlineCreation,
{
    #[inline]
    pub fn new(
        pos_plines: Vec<BooleanResultPline<P>>,
        neg_plines: Vec<BooleanResultPline<P>>,
    ) -> Self {
        Self {
            pos_plines,
            neg_plines,
        }
    }

    #[inline]
    pub fn empty() -> Self {
        Self::new(Vec::new(), Vec::new())
    }

    #[inline]
    pub fn from_whole_plines<I>(pos_plines: I, neg_plines: I) -> Self
    where
        I: IntoIterator<Item = P>,
    {
        Self {
            pos_plines: pos_plines
                .into_iter()
                .map(|p| BooleanResultPline::new(p, Vec::new()))
                .collect(),
            neg_plines: neg_plines
                .into_iter()
                .map(|p| BooleanResultPline::new(p, Vec::new()))
                .collect(),
        }
    }
}

#[derive(Debug)]
pub struct PlineBooleanOptions<'a, T>
where
    T: Real,
{
    /// Spatial index for `self` or first polyline argument for the boolean operation.
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// Fuzzy comparison epsilon used for determining if two positions are equal when stitching
    /// polyline slices together.
    pub slice_join_eps: T,
}

impl<'a, T> PlineBooleanOptions<'a, T>
where
    T: Real,
{
    #[inline]
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
    #[inline]
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
    /// Spatial index for the polyline.
    pub aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// Controls whether to include all (local + global), only local, or only global self
    /// intersects.
    pub include: SelfIntersectsInclude,
}

impl<'a, T> PlineSelfIntersectOptions<'a, T>
where
    T: Real,
{
    #[inline]
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
    #[inline]

    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct FindIntersectsOptions<'a, T>
where
    T: Real,
{
    /// Spatial index for `self` or first polyline argument to find intersects.
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
}

impl<'a, T> FindIntersectsOptions<'a, T>
where
    T: Real,
{
    #[inline]
    pub fn new() -> Self {
        Self {
            pline1_aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
        }
    }
}

impl<'a, T> Default for FindIntersectsOptions<'a, T>
where
    T: Real,
{
    #[inline]
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
    #[inline]
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
    #[inline]
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
    #[inline]
    pub fn new_basic(start_index1: usize, start_index2: usize, point: Vector2<T>) -> Self {
        PlineIntersect::Basic(PlineBasicIntersect::new(start_index1, start_index2, point))
    }

    #[inline]
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

/// Trait for visiting polyline intersects.
pub trait PlineIntersectVisitor<T, C>
where
    T: Real,
    C: ControlFlow,
{
    fn visit_basic_intr(&mut self, intr: PlineBasicIntersect<T>) -> C;
    fn visit_overlapping_intr(&mut self, intr: PlineOverlappingIntersect<T>) -> C;
}

impl<T, C, F> PlineIntersectVisitor<T, C> for F
where
    T: Real,
    C: ControlFlow,
    F: FnMut(PlineIntersect<T>) -> C,
{
    #[inline]
    fn visit_basic_intr(&mut self, intr: PlineBasicIntersect<T>) -> C {
        self(PlineIntersect::Basic(intr))
    }

    #[inline]
    fn visit_overlapping_intr(&mut self, intr: PlineOverlappingIntersect<T>) -> C {
        self(PlineIntersect::Overlapping(intr))
    }
}

/// Trait for visiting polyline vertexes.
pub trait PlineVertexVisitor<T, C>
where
    T: Real,
    C: ControlFlow,
{
    fn visit_vertex(&mut self, vertex: PlineVertex<T>) -> C;
}

impl<T, C, F> PlineVertexVisitor<T, C> for F
where
    T: Real,
    C: ControlFlow,
    F: FnMut(PlineVertex<T>) -> C,
{
    #[inline]
    fn visit_vertex(&mut self, vertex: PlineVertex<T>) -> C {
        self(vertex)
    }
}

/// Trait for visiting polyline segments (two consecutive vertexes).
pub trait PlineSegVisitor<T, C>
where
    T: Real,
    C: ControlFlow,
{
    fn visit_seg(&mut self, v1: PlineVertex<T>, v2: PlineVertex<T>) -> C;
}

impl<T, C, F> PlineSegVisitor<T, C> for F
where
    T: Real,
    C: ControlFlow,
    F: FnMut(PlineVertex<T>, PlineVertex<T>) -> C,
{
    #[inline]
    fn visit_seg(&mut self, v1: PlineVertex<T>, v2: PlineVertex<T>) -> C {
        self(v1, v2)
    }
}

/// Represents a collection of basic and overlapping polyline intersects.
#[derive(Debug, Clone)]
pub struct PlineIntersectsCollection<T> {
    pub basic_intersects: Vec<PlineBasicIntersect<T>>,
    pub overlapping_intersects: Vec<PlineOverlappingIntersect<T>>,
}

impl<T> PlineIntersectsCollection<T> {
    #[inline]
    pub fn new(
        basic_intersects: Vec<PlineBasicIntersect<T>>,
        overlapping_intersects: Vec<PlineOverlappingIntersect<T>>,
    ) -> Self {
        Self {
            basic_intersects,
            overlapping_intersects,
        }
    }

    #[inline]
    pub fn new_empty() -> Self {
        Self::new(Vec::new(), Vec::new())
    }
}

#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(rename_all = "camelCase")
)]
/// Open polyline slice created in the process of performing a polyline boolean operation.
#[derive(Debug, Copy, Clone)]
pub struct BooleanPlineSlice<T = f64> {
    /// View data for the slice (source polyline is always pline2 in boolean operation).
    pub view_data: PlineViewData<T>,
    /// If true then the source polyline for this slice is pline1 from the boolean operation
    /// otherwise it is pline2.
    pub source_is_pline1: bool,
    /// Whether the slice is an overlapping slice or not (both polylines in the boolean operation
    /// overlapped along this slice).
    pub overlapping: bool,
}

impl<T> BooleanPlineSlice<T>
where
    T: Real,
{
    #[inline]
    pub fn from_open_pline_slice(
        data: &PlineViewData<T>,
        source_is_pline1: bool,
        inverted: bool,
    ) -> Self {
        Self {
            view_data: PlineViewData {
                start_index: data.start_index,
                end_index_offset: data.end_index_offset,
                updated_start: data.updated_start,
                updated_end_bulge: data.updated_end_bulge,
                end_point: data.end_point,
                inverted_direction: inverted,
            },
            source_is_pline1,
            overlapping: false,
        }
    }

    #[inline]
    pub fn from_overlapping<P>(
        source: &P,
        overlapping_slice: &OverlappingSlice<T>,
        inverted: bool,
    ) -> Self
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        let result = Self {
            view_data: PlineViewData {
                start_index: overlapping_slice.start_indexes.1,
                end_index_offset: overlapping_slice.view_data.end_index_offset,
                updated_start: overlapping_slice.view_data.updated_start,
                updated_end_bulge: overlapping_slice.view_data.updated_end_bulge,
                end_point: overlapping_slice.view_data.end_point,
                inverted_direction: inverted,
            },
            source_is_pline1: false,
            overlapping: true,
        };
        debug_assert_eq!(
            result.view_data.validate_for_source(source),
            ViewDataValidation::IsValid
        );
        result
    }

    #[inline]
    pub fn view<'a, P>(&self, source: &'a P) -> PlineView<'a, P, T>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        self.view_data.view(source)
    }
}
