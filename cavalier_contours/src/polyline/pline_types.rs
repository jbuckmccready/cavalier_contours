//! Supporting public types used in [Polyline] methods.

use super::{
    internal::pline_intersects::OverlappingSlice, seg_arc_radius_and_center, seg_closest_point,
    seg_length, seg_split_at_point, PlineVertex, Polyline,
};
use crate::core::{
    math::{angle, angle_from_bulge, point_on_circle, Vector2},
    traits::{ControlFlow, Real},
    Control,
};
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

/// Represents one of the polyline results from a boolean operation between two polylines.
#[derive(Debug, Clone, Default)]
pub struct BooleanResultPline<T>
where
    T: Real,
{
    /// Resultant polyline.
    pub pline: Polyline<T>,
    /// Indexes of the slices that were stitched together to form the `pline`.
    pub subslices: Vec<BooleanPlineSlice<T>>,
}

impl<T> BooleanResultPline<T>
where
    T: Real,
{
    pub fn new(pline: Polyline<T>, subslices: Vec<BooleanPlineSlice<T>>) -> Self {
        Self { pline, subslices }
    }
}

#[derive(Debug, Clone, Default)]
/// Result of performing a boolean operation between two polylines.
pub struct BooleanResult<T>
where
    T: Real,
{
    /// Positive remaining space polylines and associated slice indexes.
    pub pos_plines: Vec<BooleanResultPline<T>>,
    /// Negative subtracted space polylines and associated slice indexes.
    pub neg_plines: Vec<BooleanResultPline<T>>,
}

impl<T> BooleanResult<T>
where
    T: Real,
{
    pub fn new(
        pos_plines: Vec<BooleanResultPline<T>>,
        neg_plines: Vec<BooleanResultPline<T>>,
    ) -> Self {
        Self {
            pos_plines,
            neg_plines,
        }
    }

    pub fn from_whole_plines<I>(pos_plines: I, neg_plines: I) -> Self
    where
        I: IntoIterator<Item = Polyline<T>>,
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

fn visit_slice_vertexes_impl<T, S, C, V>(slice: &S, source: &Polyline<T>, visitor: &mut V) -> C
where
    T: Real,
    S: PolylineSlice<T> + ?Sized,
    C: ControlFlow,
    V: PlineVertexVisitor<T, C>,
{
    try_cf!(visitor.visit_vertex(slice.updated_start()));

    let offset = slice.end_index_offset();
    if offset == 0 {
        return visitor.visit_vertex(PlineVertex::from_vector2(slice.end_point(), T::zero()));
    }

    // skip to vertex after start segment vertex
    let mut iter = source.iter().cycle().skip(slice.start_index() + 1);

    // vertex count to traverse before second to last vertex with updated bulge
    let between_count = offset - 1;
    for v in iter.by_ref().take(between_count) {
        try_cf!(visitor.visit_vertex(*v));
    }

    // iter cycles so unwrap will never fail
    let v = iter.next().unwrap();
    try_cf!(visitor.visit_vertex(v.with_bulge(slice.updated_end_bulge())));
    visitor.visit_vertex(PlineVertex::from_vector2(slice.end_point(), T::zero()))
}

fn visit_inverted_slice_vertexes_impl<T, S, C, V>(
    slice: &S,
    source: &Polyline<T>,
    visitor: &mut V,
) -> C
where
    T: Real,
    S: PolylineSlice<T> + ?Sized,
    C: ControlFlow,
    V: PlineVertexVisitor<T, C>,
{
    // Visit the slice vertexes in inverted direction by going in the reverse order and using the
    // next vertexes bulge negated
    let end_point = slice.end_point();
    let updated_start = PlineVertex::from_vector2(end_point, -slice.updated_end_bulge());
    try_cf!(visitor.visit_vertex(updated_start));

    let offset = slice.end_index_offset();
    if offset == 0 {
        return visitor.visit_vertex(slice.updated_start().with_bulge(T::zero()));
    }

    let skip_count = source.len() - source.fwd_wrapping_index(slice.start_index(), offset) - 1;
    let mut iter = source.iter().rev().cycle().skip(skip_count);

    let mut prev_vertex = iter.next().unwrap();
    for v in iter.by_ref().take(offset - 1) {
        try_cf!(visitor.visit_vertex(prev_vertex.with_bulge(-v.bulge)));
        prev_vertex = v;
    }

    try_cf!(visitor.visit_vertex(prev_vertex.with_bulge(-slice.updated_start().bulge)));
    visitor.visit_vertex(slice.updated_start().with_bulge(T::zero()))
}

/// Enum used for slice validation debugging and asserting.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SliceValidation<T> {
    OffsetOutOfRange {
        offset: usize,
        source_length: usize,
    },
    UpdatedStartNotOnSegment {
        start_point: Vector2<T>,
    },
    EndPointNotOnSegment {
        end_point: Vector2<T>,
    },
    EndPointOnFinalOffsetVertex {
        end_point: Vector2<T>,
        final_offset_vertex: PlineVertex<T>,
    },
    UpdatedBulgeDoesNotMatch {
        updated_bulge: T,
        expected: T,
    },
    IsValid,
}

/// Trait for working with a sub slice of a polyline.
///
/// A [PolylineSlice] has all the information required to construct a complete polyline that
/// represents the contiguous subpart of a source polyline.
///
/// Anytime the source polyline is required it is explicitly passed in to avoid lifetime
/// dependencies. This means it is up to caller to ensure the source polyline given actually matches
/// the associated slice being used.
pub trait PolylineSlice<T>
where
    T: Real,
{
    /// Source polyline start segment index.
    fn start_index(&self) -> usize;
    /// Wrapping offset from `start_index` to reach the last segment index in the source polyline.
    fn end_index_offset(&self) -> usize;
    /// First vertex of the slice (positioned somewhere along the `start_index` segment with bulge
    /// and position updated).
    fn updated_start(&self) -> PlineVertex<T>;
    /// Updated bulge value to be used in the end_index segment.
    fn updated_end_bulge(&self) -> T;
    /// Final end point of the slice.
    fn end_point(&self) -> Vector2<T>;
    /// Whether the slice direction is inverted or not, note this just affects the way vertexes are
    /// constructed from the source polyline, all properties stay oriented/defined the same.
    fn inverted_direction(&self) -> bool;

    /// Number of vertexes that will exist if this slice is constructed or visited.
    #[inline]
    fn vertex_count(&self) -> usize {
        2 + self.end_index_offset()
    }

    /// Construct an open polyline that represents this slice (source reference required for vertex
    /// data).
    ///
    /// `pos_equal_eps` is used to prevent repeat position vertexes.
    fn to_polyline(&self, source: &Polyline<T>, pos_equal_eps: T) -> Polyline<T> {
        let vertex_count = self.vertex_count();
        let mut result = Polyline::with_capacity(vertex_count, false);

        let mut visitor = |v: PlineVertex<T>| {
            result.add_or_replace_vertex(v, pos_equal_eps);
        };

        self.visit_vertexes(source, &mut visitor);
        debug_assert!(
            result.len() <= vertex_count,
            "reserved capacity was not large enough"
        );
        debug_assert_eq!(
            result.remove_repeat_pos(T::from(1e-5).unwrap()).len(),
            result.len(),
            "should not have repeat positions"
        );
        result
    }

    /// Visit all vertexes in this slice (source reference required for vertex data) until the slice
    /// has been fully traversed or the visitor breaks.
    fn visit_vertexes<V, C>(&self, source: &Polyline<T>, visitor: &mut V) -> C
    where
        C: ControlFlow,
        V: PlineVertexVisitor<T, C>,
    {
        if self.inverted_direction() {
            visit_inverted_slice_vertexes_impl(self, source, visitor)
        } else {
            visit_slice_vertexes_impl(self, source, visitor)
        }
    }

    /// Visit all polyline segments in this slice (source reference required for vertex data) until
    /// the slice has been fully traversed or the visitor breaks.
    fn visit_segs<V, C>(&self, source: &Polyline<T>, visitor: &mut V) -> C
    where
        C: ControlFlow,
        V: PlineSegVisitor<T, C>,
    {
        let mut prev_vertex = None;
        let mut visitor = |v| {
            let r = if let Some(pv) = prev_vertex {
                visitor.visit_seg(pv, v)
            } else {
                C::continuing()
            };
            prev_vertex = Some(v);
            r
        };

        self.visit_vertexes(source, &mut visitor)
    }

    /// Stitch this slice onto a target polyline by appending all its vertexes onto the target.
    ///
    /// `pos_equal_eps` is used to prevent repeat position vertexes.
    fn stitch_onto(&self, source: &Polyline<T>, target: &mut Polyline<T>, pos_equal_eps: T) {
        target.reserve(self.vertex_count());
        let mut visitor = |v: PlineVertex<T>| {
            target.add_or_replace_vertex(v, pos_equal_eps);
        };

        self.visit_vertexes(source, &mut visitor);
    }

    /// Find the point on the slice corresponding to the path length given.
    ///
    /// Returns `Ok((segment_index, point))` if `target_path_length` is less than the slice's total
    /// path length, `Ok((0, first_vertex))` if `target_path_length` is negative, otherwise returns
    /// `Err((slice_total_path_length))`.
    fn find_point_at_path_length(
        &self,
        source: &Polyline<T>,
        target_path_length: T,
    ) -> Result<(usize, Vector2<T>), T> {
        if target_path_length < T::zero() {
            return Ok((0, source[0].pos()));
        }

        let mut seg_index = 0;
        let mut acc_length = T::zero();
        let mut visitor = |v1, v2| {
            let seg_len = seg_length(v1, v2);
            let sum_len = acc_length + seg_len;
            if sum_len < target_path_length {
                acc_length = sum_len;
                seg_index += 1;
                return Control::Continue;
            }

            // parametric value (from 0 to 1) along the segment where the point lies
            let t = (target_path_length - acc_length) / seg_len;

            if v1.bulge_is_zero() {
                // line segment
                let pt = v1.pos() + (v2.pos() - v1.pos()).scale(t);
                Control::Break((seg_index, pt))
            } else {
                // arc segment
                let (radius, center) = seg_arc_radius_and_center(v1, v2);
                let start_angle = angle(center, v1.pos());
                let total_sweep_angle = angle_from_bulge(v1.bulge);
                let target_angle = if v1.bulge_is_pos() {
                    start_angle + total_sweep_angle * t
                } else {
                    start_angle - total_sweep_angle * t
                };

                let pt = point_on_circle(radius, center, target_angle);
                Control::Break((seg_index, pt))
            }
        };

        match self.visit_segs(source, &mut visitor) {
            Control::Continue => Err(acc_length),
            Control::Break((seg_index, pt)) => Ok((seg_index, pt)),
        }
    }

    /// Epsilon value to be used by [PolylineSlice::validate_for_source].
    const VALIDATION_EPS: f64 = 1e-5;

    /// Epsilon value to be used by [PolylineSlice::validate_for_source] when testing if positions
    /// are fuzzy equal.
    const VALIDATION_POINT_ON_SEG_EPS: f64 = 1e-3;

    /// Function mostly used for debugging and asserts, checks that this slice's properties are
    /// valid for the source polyline provided.
    fn validate_for_source(&self, source: &Polyline<T>) -> SliceValidation<T> {
        if self.end_index_offset() > source.len() {
            return SliceValidation::OffsetOutOfRange {
                offset: self.end_index_offset(),
                source_length: source.len(),
            };
        }

        let point_is_on_segment = |seg_index, point: Vector2<T>| {
            let on_seg_eps = T::from(Self::VALIDATION_POINT_ON_SEG_EPS).unwrap();
            let v1 = source[seg_index];
            let v2 = source[source.next_wrapping_index(seg_index)];
            if point.fuzzy_eq_eps(v1.pos(), on_seg_eps) || point.fuzzy_eq_eps(v2.pos(), on_seg_eps)
            {
                return true;
            }
            let closest_point = seg_closest_point(v1, v2, point);
            closest_point.fuzzy_eq_eps(point, on_seg_eps)
        };
        // check that updated start lies on the source polyline according to start index segment
        if !point_is_on_segment(self.start_index(), self.updated_start().pos()) {
            return SliceValidation::UpdatedStartNotOnSegment {
                start_point: self.updated_start().pos(),
            };
        }

        // check that end point lies on the source polyline according to end index segment
        let end_index = source.fwd_wrapping_index(self.start_index(), self.end_index_offset());
        if !point_is_on_segment(end_index, self.end_point()) {
            return SliceValidation::EndPointNotOnSegment {
                end_point: self.end_point(),
            };
        }

        let validation_eps = T::from(Self::VALIDATION_EPS).unwrap();
        // end point should never lie directly on top of end index segment start
        if self
            .end_point()
            .fuzzy_eq_eps(source[end_index].pos(), validation_eps)
        {
            return SliceValidation::EndPointOnFinalOffsetVertex {
                end_point: self.end_point(),
                final_offset_vertex: source[end_index],
            };
        }

        if self.end_index_offset() == 0 {
            // end point on start index segment, check that updated bulge matches updated start
            // bulge
            if !self
                .updated_end_bulge()
                .fuzzy_eq_eps(self.updated_start().bulge, validation_eps)
            {
                return SliceValidation::UpdatedBulgeDoesNotMatch {
                    updated_bulge: self.updated_end_bulge(),
                    expected: self.updated_start().bulge,
                };
            }
        }

        SliceValidation::IsValid
    }
}

/// Struct for a simple open polyline slice. See the [PolylineSlice] trait for more information.
#[derive(Debug, Copy, Clone)]
pub struct OpenPlineSlice<T> {
    /// Source polyline start segment index.
    pub start_index: usize,
    /// Wrapping offset from `start_index` to reach the last segment index in the source polyline.
    pub end_index_offset: usize,
    /// First vertex of the slice (positioned somewhere along the `start_index` segment with bulge
    /// and position updated).
    pub updated_start: PlineVertex<T>,
    /// Updated bulge value to be used in the end_index segment.
    pub updated_end_bulge: T,
    /// Final end point of the slice.
    pub end_point: Vector2<T>,
}

impl<T> OpenPlineSlice<T>
where
    T: Real,
{
    /// Create OpenPlineSlice from source polyline that exists on a single segment.
    ///
    /// Returns `None` if `slice_start_vertex` is on top of `end_intersect` (collapsed slice).
    pub fn create_on_single_segment(
        source: &Polyline<T>,
        start_index: usize,
        updated_start: PlineVertex<T>,
        end_intersect: Vector2<T>,
        pos_equal_eps: T,
    ) -> Option<Self> {
        if updated_start
            .pos()
            .fuzzy_eq_eps(end_intersect, pos_equal_eps)
        {
            return None;
        }
        let slice = Self {
            start_index,
            end_index_offset: 0,
            updated_start,
            updated_end_bulge: updated_start.bulge,
            end_point: end_intersect,
        };

        debug_assert_eq!(slice.validate_for_source(source), SliceValidation::IsValid);
        Some(slice)
    }

    /// Create OpenPlineSlice from source polyline and parameters.
    ///
    /// # Panics
    ///
    /// This function panics if `traverse_count == 0`. Use
    /// [OpenPlineSlice::create_on_single_segment] if slice exists on a single segment.
    pub fn create(
        source: &Polyline<T>,
        start_index: usize,
        end_intersect: Vector2<T>,
        intersect_index: usize,
        updated_start: PlineVertex<T>,
        traverse_count: usize,
        pos_equal_eps: T,
    ) -> Self {
        assert!(traverse_count != 0,
            "traverse_count must be greater than 1, use different constructor if slice is all on one segment"
        );

        let current_vertex = source[intersect_index];
        let (end_index_offset, updated_end_bulge) =
            if end_intersect.fuzzy_eq_eps(current_vertex.pos(), pos_equal_eps) {
                // intersect lies on top of vertex at start of segment
                let offset = traverse_count - 1;
                let updated_end_bulge = if offset != 0 {
                    source[source.prev_wrapping_index(intersect_index)].bulge
                } else {
                    updated_start.bulge
                };
                (offset, updated_end_bulge)
            } else {
                // trim bulge to intersect position
                let next_index = source.next_wrapping_index(intersect_index);
                let split = seg_split_at_point(
                    current_vertex,
                    source[next_index],
                    end_intersect,
                    pos_equal_eps,
                );
                (traverse_count, split.updated_start.bulge)
            };

        let slice = Self {
            start_index,
            end_index_offset,
            updated_start,
            updated_end_bulge,
            end_point: end_intersect,
        };

        debug_assert_eq!(slice.validate_for_source(source), SliceValidation::IsValid);

        slice
    }

    /// Construct slice representing an entire polyline.
    ///
    /// # Panics
    ///
    /// This function panics if `source` has less than 2 vertexes.
    pub fn from_entire_pline(source: &Polyline<T>) -> Self {
        let ln = source.len();
        assert!(
            ln >= 2,
            "source must have at least 2 vertexes to form slice"
        );

        let slice = if source.is_closed() {
            Self {
                start_index: 0,
                end_index_offset: ln - 1,
                updated_start: source[0],
                updated_end_bulge: source.last().unwrap().bulge,
                end_point: source[0].pos(),
            }
        } else {
            Self {
                start_index: 0,
                end_index_offset: ln - 2,
                updated_start: source[0],
                updated_end_bulge: source[ln - 2].bulge,
                end_point: source[ln - 1].pos(),
            }
        };

        debug_assert_eq!(slice.validate_for_source(source), SliceValidation::IsValid);

        slice
    }

    /// Construct slice that is contiguous between two points on a source polyline (start and end of
    /// source polyline are trimmed).
    pub fn from_slice_points(
        source: &Polyline<T>,
        start_point: Vector2<T>,
        start_index: usize,
        end_point: Vector2<T>,
        end_index: usize,
        pos_equal_eps: T,
    ) -> Option<Self> {
        debug_assert!(
            start_index <= end_index || source.is_closed(),
            "start index should be less than or equal to end index if polyline is open"
        );

        // catch if start_point is at end of first segment
        let (start_index, start_point_at_seg_end) = {
            if !source.is_closed() && start_index >= end_index {
                // not possible to wrap index forward
                (start_index, false)
            } else {
                let next_index = source.next_wrapping_index(start_index);
                if source[next_index]
                    .pos()
                    .fuzzy_eq_eps(start_point, pos_equal_eps)
                {
                    (next_index, true)
                } else {
                    (start_index, false)
                }
            }
        };

        let traverse_count = source.fwd_wrapping_dist(start_index, end_index);

        // compute updated start vertex
        let updated_start = {
            let start_v1 = source[start_index];
            let start_v2 = source[source.next_wrapping_index(start_index)];
            if start_point_at_seg_end {
                // start point on top of vertex no need to split using start_point
                if traverse_count == 0 {
                    // start and end point on same segment, split at end point
                    let split = seg_split_at_point(start_v1, start_v2, end_point, pos_equal_eps);
                    split.updated_start
                } else {
                    start_v1
                }
            } else {
                // split at start point
                let start_split =
                    seg_split_at_point(start_v1, start_v2, start_point, pos_equal_eps);
                let updated_for_start = start_split.split_vertex;
                if traverse_count == 0 {
                    // start and end point on same segment, split at end point
                    let split =
                        seg_split_at_point(updated_for_start, start_v2, end_point, pos_equal_eps);
                    split.updated_start
                } else {
                    updated_for_start
                }
            }
        };

        if traverse_count == 0 {
            Self::create_on_single_segment(
                source,
                start_index,
                updated_start,
                end_point,
                pos_equal_eps,
            )
        } else {
            Some(Self::create(
                source,
                start_index,
                end_point,
                end_index,
                updated_start,
                traverse_count,
                pos_equal_eps,
            ))
        }
    }
}

impl<T> PolylineSlice<T> for OpenPlineSlice<T>
where
    T: Real,
{
    #[inline]
    fn start_index(&self) -> usize {
        self.start_index
    }

    #[inline]
    fn end_index_offset(&self) -> usize {
        self.end_index_offset
    }

    #[inline]
    fn updated_start(&self) -> PlineVertex<T> {
        self.updated_start
    }

    #[inline]
    fn updated_end_bulge(&self) -> T {
        self.updated_end_bulge
    }

    #[inline]
    fn end_point(&self) -> Vector2<T> {
        self.end_point
    }

    #[inline]
    fn inverted_direction(&self) -> bool {
        false
    }
}

/// Open polyline slice created in the process of performing a polyline boolean operation.
#[derive(Debug, Copy, Clone)]
pub struct BooleanPlineSlice<T> {
    /// Source polyline start segment index (always pline2 in boolean operation).
    pub start_index: usize,
    /// Wrapping offset from `start_index` to reach the last segment index in the source polyline.
    pub end_index_offset: usize,
    /// First vertex of the slice (positioned somewhere along the `start_index` segment with bulge
    /// and position updated).
    pub updated_start: PlineVertex<T>,
    /// Updated bulge value to be used in the end_index segment.
    pub updated_end_bulge: T,
    /// Final end point of the slice.
    pub end_point: Vector2<T>,
    /// If true then the source polyline for this slice is pline1 from the boolean operation
    /// otherwise it is pline2.
    pub source_is_pline1: bool,
    /// Whether the slice direction is inverted or not before being stitched together for final
    /// boolean result polyline.
    pub inverted: bool,
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
        slice: &OpenPlineSlice<T>,
        source_is_pline1: bool,
        inverted: bool,
    ) -> Self {
        Self {
            start_index: slice.start_index,
            end_index_offset: slice.end_index_offset,
            updated_start: slice.updated_start,
            updated_end_bulge: slice.updated_end_bulge,
            end_point: slice.end_point,
            source_is_pline1,
            inverted,
            overlapping: false,
        }
    }

    #[inline]
    pub fn from_overlapping(
        source: &Polyline<T>,
        overlapping_slice: &OverlappingSlice<T>,
        inverted: bool,
    ) -> Self {
        let result = Self {
            start_index: overlapping_slice.start_indexes.1,
            end_index_offset: overlapping_slice.end_index_offset,
            updated_start: overlapping_slice.updated_start,
            updated_end_bulge: overlapping_slice.updated_end_bulge,
            end_point: overlapping_slice.end_point,
            source_is_pline1: false,
            inverted,
            overlapping: true,
        };
        debug_assert_eq!(result.validate_for_source(source), SliceValidation::IsValid);
        result
    }
}

impl<T> PolylineSlice<T> for BooleanPlineSlice<T>
where
    T: Real,
{
    #[inline]
    fn start_index(&self) -> usize {
        self.start_index
    }

    #[inline]
    fn end_index_offset(&self) -> usize {
        self.end_index_offset
    }

    #[inline]
    fn updated_start(&self) -> PlineVertex<T> {
        self.updated_start
    }

    #[inline]
    fn updated_end_bulge(&self) -> T {
        self.updated_end_bulge
    }

    #[inline]
    fn end_point(&self) -> Vector2<T> {
        self.end_point
    }

    #[inline]
    fn inverted_direction(&self) -> bool {
        self.inverted
    }
}
