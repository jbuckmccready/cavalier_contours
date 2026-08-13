//! Supporting public types used in the core polyline trait methods.

use super::{PlineVertex, PlineView, PlineViewData};
use crate::{
    core::{
        math::Vector2,
        traits::{ControlFlow, Real},
    },
    polyline::{
        PlineCreation, PlineSegIntr, PlineSource, ViewDataValidation,
        internal::pline_intersects::OverlappingSlice,
    },
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

/// Result from calling [`PlineSource::closest_point`].
#[derive(Debug, Copy, Clone)]
pub struct ClosestPointResult<T = f64>
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
pub struct PlineOffsetOptions<'a, T = f64>
where
    T: Real,
{
    /// Spatial index of all the polyline segment bounding boxes (or boxes no smaller, e.g. using
    /// [`PlineSource::create_approx_aabb_index`] is valid). If `None` is given then it will be
    /// computed internally. [`PlineSource::create_approx_aabb_index`] or
    /// [`PlineSource::create_aabb_index`] may be used to create the spatial index, the only
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

impl<T> PlineOffsetOptions<'_, T>
where
    T: Real,
{
    #[inline]
    #[must_use]
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

impl<T> Default for PlineOffsetOptions<'_, T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// The containment functions use the same underlying mechinsims as the boolean functions.
/// Information about what happened during the boolean operation.
#[derive(Debug, Clone, PartialEq)]
pub enum PlineContainsResult {
    /// Input was not valid to perform containment test operation.
    InvalidInput,
    /// Pline1 entirely inside of pline2 with no intersects.
    Pline1InsidePline2,
    /// Pline2 entirely inside of pline1 with no intersects.
    Pline2InsidePline1,
    /// Pline1 is disjoint from pline2 (no intersects and neither polyline is inside of the other).
    Disjoint,
    /// Pline1 intersects with pline2 in at least one place.
    Intersected,
}

#[derive(Debug)]
pub struct PlineContainsOptions<'a, T = f64>
where
    T: Real,
{
    /// Spatial index for `self`
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
}

impl<T> PlineContainsOptions<'_, T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            pline1_aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
        }
    }
}

impl<T> Default for PlineContainsOptions<'_, T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Boolean operation to apply to polylines.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
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
    /// Slices that were stitched together to form the `pline` result. If boolean result info is not
    /// [`BooleanResultInfo::Intersected`] this collection may be empty.
    pub subslices: Vec<BooleanPlineSlice<P::Num>>,
}

impl<P> BooleanResultPline<P>
where
    P: PlineCreation,
{
    #[inline]
    #[must_use]
    pub fn new(pline: P, subslices: Vec<BooleanPlineSlice<P::Num>>) -> Self {
        Self { pline, subslices }
    }
}

/// Information about what happened during the boolean operation.
#[derive(Debug, Clone)]
pub enum BooleanResultInfo {
    /// Input was not valid to perform boolean operation.
    InvalidInput,
    /// Pline1 entirely inside of pline2 with no intersects.
    Pline1InsidePline2,
    /// Pline2 entirely inside of pline1 with no intersects.
    Pline2InsidePline1,
    /// Pline1 is disjoint from pline2 (no intersects and neither polyline is inside of the other).
    Disjoint,
    /// Pline1 exactly overlaps pline2 (same geometric path).
    Overlapping,
    /// Pline1 intersects with pline2 but is not exactly overlapping with the same geometric path.
    Intersected,
}

#[derive(Debug, Clone)]
/// Result of performing a boolean operation between two polylines.
pub struct BooleanResult<P>
where
    P: PlineCreation,
{
    /// Positive remaining space polylines.
    pub pos_plines: Vec<BooleanResultPline<P>>,
    /// Negative subtracted space polylines.
    pub neg_plines: Vec<BooleanResultPline<P>>,
    /// Information about what happened during the boolean operation.
    pub result_info: BooleanResultInfo,
}

impl<P> BooleanResult<P>
where
    P: PlineCreation,
{
    #[inline]
    #[must_use]
    pub fn new(
        pos_plines: Vec<BooleanResultPline<P>>,
        neg_plines: Vec<BooleanResultPline<P>>,
        result_info: BooleanResultInfo,
    ) -> Self {
        Self {
            pos_plines,
            neg_plines,
            result_info,
        }
    }

    #[inline]
    #[must_use]
    pub fn empty(result_info: BooleanResultInfo) -> Self {
        Self::new(Vec::new(), Vec::new(), result_info)
    }

    #[inline]
    #[must_use]
    pub fn from_whole_plines<I>(
        pos_plines: I,
        neg_plines: I,
        result_info: BooleanResultInfo,
    ) -> Self
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
            result_info,
        }
    }
}

#[derive(Debug)]
pub struct PlineBooleanOptions<'a, T = f64>
where
    T: Real,
{
    /// Spatial index for `self` or first polyline argument for the boolean operation.
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// If Some then this epsilon value is used to determine if a result polyline is collapsed, that
    /// is has no area according to abs(area) < eps. Polylines that are collapsed will not be
    /// included in the result. This is useful to avoid inconsistent results due to floating point
    /// thresholding, or if you just don't want ever want collapsed polylines in the result.
    pub collapsed_area_eps: Option<T>,
}

impl<T> PlineBooleanOptions<'_, T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            pline1_aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
            collapsed_area_eps: None,
        }
    }
}

impl<T> Default for PlineBooleanOptions<'_, T>
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
pub struct PlineSelfIntersectOptions<'a, T = f64>
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

impl<T> PlineSelfIntersectOptions<'_, T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
            include: SelfIntersectsInclude::All,
        }
    }
}

impl<T> Default for PlineSelfIntersectOptions<'_, T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct FindIntersectsOptions<'a, T = f64>
where
    T: Real,
{
    /// Spatial index for `self` or first polyline argument to find intersects.
    pub pline1_aabb_index: Option<&'a StaticAABB2DIndex<T>>,
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
}

impl<T> FindIntersectsOptions<'_, T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            pline1_aabb_index: None,
            pos_equal_eps: T::from(1e-5).unwrap(),
        }
    }
}

impl<T> Default for FindIntersectsOptions<'_, T>
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
pub struct PlineBasicIntersect<T = f64> {
    /// Starting vertex index of the first polyline segment involved in the intersect.
    pub start_index1: usize,
    /// Starting vertex index of the second polyline segment involved in the intersect.
    pub start_index2: usize,
    /// Point at which the intersect occurs.
    pub point: Vector2<T>,
}

impl<T> PlineBasicIntersect<T> {
    #[inline]
    #[must_use]
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
pub struct PlineOverlappingIntersect<T = f64> {
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
    #[must_use]
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

/// Represents a polyline intersect that may be either a [`PlineBasicIntersect`] or
/// [`PlineOverlappingIntersect`].
#[derive(Debug, Clone, Copy)]
pub enum PlineIntersect<T = f64> {
    Basic(PlineBasicIntersect<T>),
    Overlapping(PlineOverlappingIntersect<T>),
}

impl<T> PlineIntersect<T> {
    #[inline]
    #[must_use]
    pub fn new_basic(start_index1: usize, start_index2: usize, point: Vector2<T>) -> Self {
        PlineIntersect::Basic(PlineBasicIntersect {
            start_index1,
            start_index2,
            point,
        })
    }

    #[inline]
    #[must_use]
    pub fn new_overlapping(
        start_index1: usize,
        start_index2: usize,
        point1: Vector2<T>,
        point2: Vector2<T>,
    ) -> Self {
        PlineIntersect::Overlapping(PlineOverlappingIntersect {
            start_index1,
            start_index2,
            point1,
            point2,
        })
    }
}

/// Identifies an item considered during self-intersection traversal.
///
/// Filter implementations may use this distinction to map compact spatial-index item IDs back to
/// polyline segment indexes while handling local segment indexes directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlineIntersectFilterItem {
    /// Start index of a segment used in a local self-intersection test.
    LocalSegment(usize),
    /// Item ID from the spatial index used for global self-intersection tests.
    GlobalAabbItem(usize),
}

impl PlineIntersectFilterItem {
    /// Returns the segment index or spatial-index item ID carried by this value.
    #[inline]
    #[must_use]
    pub fn index(self) -> usize {
        match self {
            Self::LocalSegment(index) | Self::GlobalAabbItem(index) => index,
        }
    }
}

/// Trait for filtering and visiting polyline self-intersections.
pub trait PlineIntersectVisitor<T, C>
where
    T: Real,
    C: ControlFlow,
{
    /// Resolves a traversal item to a polyline segment start index.
    ///
    /// Return `None` to exclude the item before its detailed intersection tests. The default
    /// implementation treats the item ID as its segment index and includes it. A
    /// [`PlineIntersectFilterItem::LocalSegment`] already contains its segment index and should
    /// only be accepted or rejected; a [`PlineIntersectFilterItem::GlobalAabbItem`] may require
    /// mapping when the spatial index contains only a subset of segments.
    #[inline]
    fn filter_map(&self, item: PlineIntersectFilterItem) -> Option<usize> {
        Some(item.index())
    }

    /// Visits a basic or overlapping self-intersection.
    fn visit(&mut self, intersect: PlineIntersect<T>) -> C;
}

impl<T, C, F> PlineIntersectVisitor<T, C> for F
where
    T: Real,
    C: ControlFlow,
    F: FnMut(PlineIntersect<T>) -> C,
{
    #[inline]
    fn visit(&mut self, intersect: PlineIntersect<T>) -> C {
        self(intersect)
    }
}

/// Segment context supplied when visiting an intersection between two polylines.
#[derive(Default, Copy, Clone)]
pub struct PlineIntersectVisitContext<T> {
    /// Start vertex index of the segment.
    pub vertex_index: usize,
    /// Segment start vertex.
    pub v1: PlineVertex<T>,
    /// Segment end vertex.
    pub v2: PlineVertex<T>,
}

/// Identifies an item considered while finding intersections between two polylines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwoPlinesIntersectFilterItem {
    /// Item ID from the first polyline's spatial index.
    Pline1AabbItem(usize),
    /// Start index of a segment from the directly iterated second polyline.
    Pline2Segment(usize),
}

impl TwoPlinesIntersectFilterItem {
    /// Returns the segment index or spatial-index item ID carried by this value.
    #[inline]
    #[must_use]
    pub fn index(self) -> usize {
        match self {
            Self::Pline1AabbItem(index) | Self::Pline2Segment(index) => index,
        }
    }
}

/// Trait for filtering and visiting intersections between two polylines.
pub trait TwoPlinesIntersectVisitor<T, C>
where
    T: Real,
    C: ControlFlow,
{
    /// Resolves a traversal item to a segment start index in its source polyline.
    ///
    /// Return `None` to exclude the item. Second-polyline segments are filtered before their
    /// spatial query, while first-polyline items are filtered before detailed intersection tests.
    /// A [`TwoPlinesIntersectFilterItem::Pline2Segment`] already contains its segment index and
    /// should only be accepted or rejected; a
    /// [`TwoPlinesIntersectFilterItem::Pline1AabbItem`] may require mapping when the spatial index
    /// contains a compact subset of segments. The default implementation treats the item ID as its
    /// segment index and includes it.
    #[inline]
    fn filter(&self, item: TwoPlinesIntersectFilterItem) -> Option<usize> {
        Some(item.index())
    }

    /// Visits the detailed intersection result and contexts for the resolved segment pair.
    fn visit(
        &mut self,
        intersect: PlineSegIntr<T>,
        pline1_context: &PlineIntersectVisitContext<T>,
        pline2_context: &PlineIntersectVisitContext<T>,
    ) -> C;
}

impl<T, C, F> TwoPlinesIntersectVisitor<T, C> for F
where
    T: Real,
    C: ControlFlow,
    F: FnMut(PlineSegIntr<T>, &PlineIntersectVisitContext<T>, &PlineIntersectVisitContext<T>) -> C,
{
    #[inline]
    fn visit(
        &mut self,
        intersect: PlineSegIntr<T>,
        pline1_context: &PlineIntersectVisitContext<T>,
        pline2_context: &PlineIntersectVisitContext<T>,
    ) -> C {
        self(intersect, pline1_context, pline2_context)
    }
}

/// Represents a collection of basic and overlapping polyline intersects.
#[derive(Debug, Clone)]
pub struct PlineIntersectsCollection<T = f64> {
    pub basic_intersects: Vec<PlineBasicIntersect<T>>,
    pub overlapping_intersects: Vec<PlineOverlappingIntersect<T>>,
}

impl<T> PlineIntersectsCollection<T> {
    #[inline]
    #[must_use]
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
    #[must_use]
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
    /// View data for the slice, can be used with source polyline to form a view of the vertexes for
    /// the slice.
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
    #[must_use]
    pub fn view<'a, P>(&self, source: &'a P) -> PlineView<'a, P>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        self.view_data.view(source)
    }

    #[inline]
    #[must_use]
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
    #[must_use]
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
}
