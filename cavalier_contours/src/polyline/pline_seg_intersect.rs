use super::{PlineVertex, Segment, SegmentIntersections, intersect_segments};
use crate::core::traits::Real;

/// Intersects two finite polyline segments.
///
/// Segments are defined by `seg1_start->seg1_end` and `seg2_start->seg2_end`. The returned point
/// is authoritative, and each hit includes normalized parameters on both segments. Two-point and
/// overlap results follow the second segment's direction.
#[must_use]
pub fn pline_seg_intr<T>(
    seg1_start: PlineVertex<T>,
    seg1_end: PlineVertex<T>,
    seg2_start: PlineVertex<T>,
    seg2_end: PlineVertex<T>,
    pos_equal_eps: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    intersect_segments(
        Segment::from_vertices(seg1_start, seg1_end),
        Segment::from_vertices(seg2_start, seg2_end),
        pos_equal_eps,
    )
}
