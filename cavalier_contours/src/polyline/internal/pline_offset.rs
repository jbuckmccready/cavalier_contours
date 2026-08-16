//! Internal stages and intermediate types for the polyline offset algorithm.
//!
//! These items are public only so workspace visualization, benchmarks, and tests can inspect
//! intermediate results.
//!
//! The algorithm creates a primary raw offset, records usable contacts, splits it into validated
//! dissected slices, and joins those slices through their contact occurrences. A `ContactNode`
//! stores one contact point and its occurrences. A `ContactOccurrence` identifies one primary raw
//! segment at that node. A `ContactRelation` allows routing between two occurrences. An
//! `OffsetSlice` stores one validated raw offset span with its boundary occurrences.
//!
//! The primary raw offset is checked for local construction failures, minimum distance from the
//! source, and source intersections. Each valid slice keeps occurrence IDs at its boundaries.
//! Stitching first follows an unvisited slice whose start occurrence matches the current ending
//! occurrence, then one whose start occurrence is related to the current ending occurrence. It
//! closes a component only when the contact records permit it. Preserve mode keeps raw offset
//! traversal through contacts.
//! Separate mode follows an unambiguous touch relation before same-occurrence continuation, so
//! touching loops can become separate components. Crossings always use preserve routing.
//!
//! `pos_equal_eps` groups nearby contact points and removes repeated positions while copying
//! geometry. It does not find stitch connections. Connections come only from occurrence identity
//! and recorded relations.

use crate::{
    core::{
        Control,
        math::{
            CircleCircleIntr, LineCircleIntr, Vector2, angle, circle_circle_intr,
            delta_angle_signed, dist_squared, line_circle_intr, min_max, point_from_parametric,
            point_within_arc_sweep,
        },
        traits::Real,
    },
    polyline::{
        CoincidentSegmentBehavior, FindIntersectsOptions, PlineBasicIntersect, PlineCreation,
        PlineIntersect, PlineIntersectFilterItem, PlineIntersectVisitor, PlineOffsetOptions,
        PlineOverlappingIntersect, PlineSegIntr, PlineSource, PlineVertex, PlineViewData,
        TouchingLoopBehavior, TwoPlinesIntersectFilterItem,
        internal::{
            pline_intersects::{
                find_intersects, find_intersects_filtered, visit_global_self_intersects,
                visit_local_self_intersects,
            },
            raw_pline_offset::{RawOffsetResult, create_raw_offset},
        },
        pline_seg_intr, seg_arc_radius_and_center, seg_closest_point, seg_fast_approx_bounding_box,
        seg_length, seg_midpoint, seg_tangent_vector,
    },
};
use static_aabb2d_index::{Control as AabbControl, StaticAABB2DIndex, StaticAABB2DIndexBuilder};
/// Returns whether `point` is farther from every source segment than the allowed offset distance.
/// Invalid raw segment flags handle local folds, so `offset_tol` applies only to global distance
/// checks.
#[inline]
pub fn point_valid_for_offset<P, T>(
    polyline: &P,
    offset: T,
    aabb_index: &StaticAABB2DIndex<T>,
    point: Vector2<T>,
    query_stack: &mut Vec<usize>,
    pos_equal_eps: T,
    offset_tol: T,
) -> bool
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let abs_offset = offset.abs() - offset_tol;
    let min_dist = abs_offset * abs_offset;
    let mut point_valid = true;
    let mut visitor = |i: usize| {
        let j = polyline.next_wrapping_index(i);
        let closest_point = seg_closest_point(polyline.at(i), polyline.at(j), point, pos_equal_eps);
        let dist = dist_squared(closest_point, point);
        point_valid = dist > min_dist;
        if point_valid {
            AabbControl::Continue
        } else {
            AabbControl::Break(())
        }
    };

    aabb_index.visit_query_with_stack(
        point.x - abs_offset,
        point.y - abs_offset,
        point.x + abs_offset,
        point.y + abs_offset,
        &mut visitor,
        query_stack,
    );
    point_valid
}

/// Returns whether the slice includes a raw segment marked locally invalid.
///
/// `invalid_segments` is sorted. A non-wrapping slice needs one range lookup. A wrapping closed
/// slice checks the end and start ranges separately.
fn slice_contains_invalid_segment<T>(
    slice: &PlineViewData<T>,
    raw_is_closed: bool,
    segment_count: usize,
    invalid_segments: &[usize],
) -> bool {
    if invalid_segments.is_empty() {
        return false;
    }

    let range_contains_invalid = |start: usize, end: usize| {
        let first = invalid_segments.partition_point(|&index| index < start);
        invalid_segments
            .get(first)
            .is_some_and(|&index| index <= end)
    };

    let end = slice.start_index + slice.end_index_offset;
    if !raw_is_closed || end < segment_count {
        return range_contains_invalid(slice.start_index, end);
    }

    debug_assert!(end < 2 * segment_count);
    range_contains_invalid(slice.start_index, segment_count - 1)
        || range_contains_invalid(0, end - segment_count)
}

/// Shared data for validating raw offset slices against the source polyline.
struct OffsetSliceValidator<'a, P, R, T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    original_polyline: &'a P,
    raw_offset_polyline: &'a R,
    orig_polyline_index: &'a StaticAABB2DIndex<T>,
    invalid_segments: &'a [usize],
    offset: T,
    pos_equal_eps: T,
    offset_dist_eps: T,
}

impl<'a, P, R, T> OffsetSliceValidator<'a, P, R, T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    fn new(
        original_polyline: &'a P,
        raw_offset_polyline: &'a R,
        orig_polyline_index: &'a StaticAABB2DIndex<T>,
        invalid_segments: &'a [usize],
        offset: T,
        options: &PlineOffsetOptions<T>,
    ) -> Self {
        Self {
            original_polyline,
            raw_offset_polyline,
            orig_polyline_index,
            invalid_segments,
            offset,
            pos_equal_eps: options.pos_equal_eps,
            offset_dist_eps: options.offset_dist_eps,
        }
    }

    /// Returns whether `point` is far enough from every source segment for this offset.
    fn point_is_valid(&self, point: Vector2<T>, query_stack: &mut Vec<usize>) -> bool {
        point_valid_for_offset(
            self.original_polyline,
            self.offset,
            self.orig_polyline_index,
            point,
            query_stack,
            self.pos_equal_eps,
            self.offset_dist_eps,
        )
    }

    /// Uses the source index to check whether the segment from `v1` to `v2` intersects any source
    /// segment.
    fn intersects_original(
        &self,
        v1: PlineVertex<T>,
        v2: PlineVertex<T>,
        query_stack: &mut Vec<usize>,
    ) -> bool {
        let approx_bb = seg_fast_approx_bounding_box(v1, v2);
        let original_polyline = self.original_polyline;
        let pos_equal_eps = self.pos_equal_eps;
        let mut has_intersect = false;
        let mut visitor = |i: usize| {
            let j = original_polyline.next_wrapping_index(i);
            has_intersect = !matches!(
                pline_seg_intr(
                    v1,
                    v2,
                    original_polyline.at(i),
                    original_polyline.at(j),
                    pos_equal_eps
                ),
                PlineSegIntr::NoIntersect
            );
            if has_intersect {
                AabbControl::Break(())
            } else {
                AabbControl::Continue
            }
        };

        let fuzz = T::fuzzy_epsilon();
        self.orig_polyline_index.visit_query_with_stack(
            approx_bb.min_x - fuzz,
            approx_bb.min_y - fuzz,
            approx_bb.max_x + fuzz,
            approx_bb.max_y + fuzz,
            &mut visitor,
            query_stack,
        );
        has_intersect
    }

    /// Returns whether `slice` has valid raw segments, stays far enough from the source, and does
    /// not intersect the source.
    fn slice_is_valid(&self, slice: &PlineViewData<T>, query_stack: &mut Vec<usize>) -> bool {
        let raw_offset_polyline = self.raw_offset_polyline;
        if slice_contains_invalid_segment(
            slice,
            raw_offset_polyline.is_closed(),
            raw_offset_polyline.segment_count(),
            self.invalid_segments,
        ) {
            return false;
        }

        if slice.end_index_offset == 0 {
            // slice all on one segment, test start, end, midpoint, and if it intersects the
            // original
            let v1 = slice.updated_start;
            if !self.point_is_valid(v1.pos(), query_stack) {
                return false;
            }
            let v2 = PlineVertex::from_vector2(slice.end_point, T::zero());
            if !self.point_is_valid(v2.pos(), query_stack) {
                return false;
            }
            let midpoint = seg_midpoint(v1, v2);
            if !self.point_is_valid(midpoint, query_stack) {
                return false;
            }

            return !self.intersects_original(v1, v2, query_stack);
        }

        // slice not all on one segment, start by checking midpoints of first and last segment of
        // the slice
        let start_seg_midpoint = seg_midpoint(
            slice.updated_start,
            raw_offset_polyline.at(raw_offset_polyline.next_wrapping_index(slice.start_index)),
        );

        if !self.point_is_valid(start_seg_midpoint, query_stack) {
            return false;
        }

        let end_index =
            raw_offset_polyline.fwd_wrapping_index(slice.start_index, slice.end_index_offset);
        let end_seg_midpoint = seg_midpoint(
            raw_offset_polyline
                .at(end_index)
                .with_bulge(slice.updated_end_bulge),
            PlineVertex::from_vector2(slice.end_point, T::zero()),
        );

        if !self.point_is_valid(end_seg_midpoint, query_stack) {
            return false;
        }

        // test all segments
        for (v1, v2) in slice.view(raw_offset_polyline).iter_segments() {
            // test start point
            if !self.point_is_valid(v1.pos(), query_stack) {
                return false;
            }

            // test intersection with original polyline
            if self.intersects_original(v1, v2, query_stack) {
                return false;
            }
        }
        // check final end point (loop checks only start point and intersection)
        self.point_is_valid(slice.end_point, query_stack)
    }
}

/// Creates a spatial index for polyline segments. When invalid segments are dense, it leaves them
/// out and returns a map from index item IDs to raw segment indexes.
///
/// When invalid segments are sparse, the full index is cheaper than rebuilding it and checking
/// extra candidates.
fn create_segment_index<P, T>(
    polyline: &P,
    invalid_segments: &[bool],
    invalid_count: usize,
) -> (StaticAABB2DIndex<T>, Option<Vec<usize>>)
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let segment_count = invalid_segments.len();
    if invalid_count * 4 < segment_count {
        return (polyline.create_approx_aabb_index(), None);
    }

    let valid_count = segment_count - invalid_count;
    let mut builder = StaticAABB2DIndexBuilder::new(valid_count);
    let mut item_to_segment = Vec::with_capacity(valid_count);
    for (index, (v1, v2)) in polyline.iter_segments().enumerate() {
        if invalid_segments[index] {
            continue;
        }
        let bb = seg_fast_approx_bounding_box(v1, v2);
        builder.add(bb.min_x, bb.min_y, bb.max_x, bb.max_y);
        item_to_segment.push(index);
    }
    (builder.build().unwrap(), Some(item_to_segment))
}

/// Maps a spatial-index item to a raw segment and rejects locally invalid geometry before it can
/// create a contact boundary.
fn resolve_valid_segment(
    item: usize,
    item_to_segment: Option<&[usize]>,
    invalid_segments: &[bool],
) -> Option<usize> {
    let index = item_to_segment.map_or(item, |mapping| mapping[item]);
    (!invalid_segments[index]).then_some(index)
}

/// Index of a contact node in the node list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct ContactNodeId(usize);

/// Index of a contact occurrence in the occurrence list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct OccurrenceId(usize);

/// Kind of contact between two raw-segment occurrences.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactRelationKind {
    /// A contact between nonparallel segments, or one whose tangent is too short to classify.
    Crossing,
    /// A contact between segments with parallel or opposite tangents. Separate-touch routing may
    /// transfer between the two occurrences.
    Touch,
    /// An endpoint of a nonzero overlap. It takes precedence over other contact kinds.
    OverlapBoundary,
}

/// One primary raw segment at a contact node.
///
/// The forward distance orders occurrences on the segment. A `(node, raw_segment)` pair is unique,
/// but an occurrence can take part in several contact relations when more than two segments meet
/// at one contact node.
struct ContactOccurrence<T> {
    node_id: ContactNodeId,
    raw_segment: usize,
    distance_from_segment_start: T,
}

/// An unordered pair of primary raw occurrences at a contact node.
///
/// A contact relation allows routing from one occurrence to the other. Sharing a contact node does
/// not connect two occurrences by itself.
struct ContactRelation {
    occurrence1: OccurrenceId,
    occurrence2: OccurrenceId,
    kind: ContactRelationKind,
}

/// One contact point and its primary raw occurrences and contact relations.
///
/// Contact reports within `pos_equal_eps` are grouped into one node for storage. Routing still
/// requires the same occurrence or an explicit contact relation.
struct ContactNode<T> {
    point: Vector2<T>,
    occurrences: Vec<OccurrenceId>,
    relations: Vec<ContactRelation>,
}

impl<T: Copy> ContactOccurrence<T> {
    fn point(&self, nodes: &[ContactNode<T>]) -> Vector2<T> {
        nodes[self.node_id.0].point
    }
}

/// One validated forward span of the primary raw offset with optional occurrence IDs at its
/// boundaries.
struct OffsetSlice<T> {
    /// Geometry selected from the primary raw offset.
    view_data: PlineViewData<T>,
    /// Occurrence at the start of the slice. `None` only for an open prefix or a complete path with
    /// no contacts.
    start_occurrence: Option<OccurrenceId>,
    /// Occurrence at the end of the slice. `None` only for an open suffix or a complete path with
    /// no contacts.
    end_occurrence: Option<OccurrenceId>,
}

/// Validated slices and the contact data used to route between them.
struct OffsetSliceSet<T> {
    slices: Vec<OffsetSlice<T>>,
    nodes: Vec<ContactNode<T>>,
    occurrences: Vec<ContactOccurrence<T>>,
}

impl<T> OffsetSliceSet<T> {
    fn empty() -> Self {
        Self {
            slices: Vec::new(),
            nodes: Vec::new(),
            occurrences: Vec::new(),
        }
    }
}

/// Forward distance interval covered by a nonzero overlap on one raw segment.
#[derive(Clone, Copy)]
struct OverlapInterval<T> {
    start: T,
    end: T,
}

/// Sorted occurrence IDs grouped into ranges for the raw segments that have contacts.
struct DissectionPoints {
    occurrence_ids: Vec<OccurrenceId>,
}

impl DissectionPoints {
    fn new() -> Self {
        Self {
            occurrence_ids: Vec::new(),
        }
    }

    fn push(&mut self, occurrence_id: OccurrenceId) {
        self.occurrence_ids.push(occurrence_id);
    }

    fn is_empty(&self) -> bool {
        self.occurrence_ids.is_empty()
    }

    fn finish<T>(&mut self, occurrences: &[ContactOccurrence<T>])
    where
        T: Real,
    {
        self.occurrence_ids.sort_unstable_by(|a, b| {
            let occurrence_a = &occurrences[a.0];
            let occurrence_b = &occurrences[b.0];
            occurrence_a
                .raw_segment
                .cmp(&occurrence_b.raw_segment)
                .then_with(|| {
                    occurrence_a
                        .distance_from_segment_start
                        .total_cmp(&occurrence_b.distance_from_segment_start)
                })
                .then_with(|| occurrence_a.node_id.cmp(&occurrence_b.node_id))
                .then_with(|| a.cmp(b))
        });
    }

    fn first<T>(&self, occurrences: &[ContactOccurrence<T>]) -> Option<(usize, &[OccurrenceId])> {
        let raw_segment = occurrences.get(self.occurrence_ids.first()?.0)?.raw_segment;
        let end = self.range_end(0, raw_segment, occurrences);
        Some((raw_segment, &self.occurrence_ids[..end]))
    }

    fn iter<'a, T>(
        &'a self,
        occurrences: &'a [ContactOccurrence<T>],
    ) -> DissectionPointsIter<'a, T> {
        DissectionPointsIter {
            occurrence_ids: &self.occurrence_ids,
            occurrences,
            next_start: 0,
        }
    }

    fn range_end<T>(
        &self,
        start: usize,
        raw_segment: usize,
        occurrences: &[ContactOccurrence<T>],
    ) -> usize {
        self.occurrence_ids[start..]
            .iter()
            .position(|occurrence_id| occurrences[occurrence_id.0].raw_segment != raw_segment)
            .map_or(self.occurrence_ids.len(), |offset| start + offset)
    }
}

struct DissectionPointsIter<'a, T> {
    occurrence_ids: &'a [OccurrenceId],
    occurrences: &'a [ContactOccurrence<T>],
    next_start: usize,
}

impl<'a, T> Iterator for DissectionPointsIter<'a, T> {
    type Item = (usize, &'a [OccurrenceId]);

    fn next(&mut self) -> Option<Self::Item> {
        let raw_segment = self
            .occurrences
            .get(self.occurrence_ids.get(self.next_start)?.0)?
            .raw_segment;
        let start = self.next_start;
        let end = self.occurrence_ids[start..]
            .iter()
            .position(|occurrence_id| self.occurrences[occurrence_id.0].raw_segment != raw_segment)
            .map_or(self.occurrence_ids.len(), |offset| start + offset);
        self.next_start = end;
        Some((raw_segment, &self.occurrence_ids[start..end]))
    }
}

/// Collects contact data and temporary data used to split the primary raw offset.
///
/// `dissection_points` stores occurrence IDs in one flat list sorted by raw segment in
/// `finish_intersections`. Iteration creates ranges for active raw segments without allocating one
/// container per segment. Overlap coverage is stored only in discard mode.
struct OffsetTopologyBuilder<T> {
    nodes: Vec<ContactNode<T>>,
    occurrences: Vec<ContactOccurrence<T>>,
    dissection_points: DissectionPoints,
    /// Overlap coverage for each raw segment. `Some` contains one interval list per segment and is
    /// used only in discard mode. `None` means preserve mode; overlap coverage is not collected or
    /// checked.
    overlap_intervals: Option<Vec<Vec<OverlapInterval<T>>>>,
}

impl<T> OffsetTopologyBuilder<T>
where
    T: Real,
{
    /// Creates an empty builder for `segment_count` primary raw segments.
    ///
    /// Preserve mode stores only overlap boundaries. Discard mode also allocates an interval list
    /// for each raw segment.
    fn new(segment_count: usize, collect_overlaps: bool) -> Self {
        Self {
            nodes: Vec::new(),
            occurrences: Vec::new(),
            dissection_points: DissectionPoints::new(),
            overlap_intervals: collect_overlaps.then(|| vec![Vec::new(); segment_count]),
        }
    }

    /// Returns the first contact node within `pos_equal_eps` of `point`, or creates one that stores
    /// `point`.
    fn find_or_create_node(&mut self, point: Vector2<T>, pos_equal_eps: T) -> ContactNodeId {
        if let Some(index) = self
            .nodes
            .iter()
            .position(|node| node.point.fuzzy_eq_eps(point, pos_equal_eps))
        {
            return ContactNodeId(index);
        }

        let id = ContactNodeId(self.nodes.len());
        self.nodes.push(ContactNode {
            point,
            occurrences: Vec::new(),
            relations: Vec::new(),
        });
        id
    }

    /// Returns the unique occurrence for this node and raw segment.
    ///
    /// A report at a segment end uses the outgoing segment when one exists. The occurrence stores
    /// its forward distance and is added to `dissection_points`.
    fn add_occurrence<P>(
        &mut self,
        raw_offset: &P,
        node_id: ContactNodeId,
        segment_index: usize,
        pos_equal_eps: T,
    ) -> OccurrenceId
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        let point = self.nodes[node_id.0].point;
        let segment_index =
            canonical_segment_index(raw_offset, segment_index, point, pos_equal_eps);
        if let Some(id) = self.nodes[node_id.0]
            .occurrences
            .iter()
            .copied()
            .find(|id| self.occurrences[id.0].raw_segment == segment_index)
        {
            return id;
        }

        let occurrence_id = OccurrenceId(self.occurrences.len());
        self.occurrences.push(ContactOccurrence {
            node_id,
            raw_segment: segment_index,
            distance_from_segment_start: distance_from_segment_start(
                raw_offset,
                segment_index,
                point,
                pos_equal_eps,
            ),
        });
        self.nodes[node_id.0].occurrences.push(occurrence_id);
        self.dissection_points.push(occurrence_id);
        occurrence_id
    }

    /// Adds an unordered contact relation or strengthens the existing relation for the same pair.
    ///
    /// If a pair is reported more than once, the relation keeps `OverlapBoundary` over `Crossing`,
    /// and `Crossing` over `Touch`.
    fn add_relation(
        &mut self,
        node_id: ContactNodeId,
        occurrence1: OccurrenceId,
        occurrence2: OccurrenceId,
        kind: ContactRelationKind,
    ) {
        if occurrence1 == occurrence2 {
            return;
        }

        let relations = &mut self.nodes[node_id.0].relations;
        if let Some(relation) = relations.iter_mut().find(|relation| {
            (relation.occurrence1 == occurrence1 && relation.occurrence2 == occurrence2)
                || (relation.occurrence1 == occurrence2 && relation.occurrence2 == occurrence1)
        }) {
            relation.kind = match (relation.kind, kind) {
                (ContactRelationKind::OverlapBoundary, _)
                | (_, ContactRelationKind::OverlapBoundary) => ContactRelationKind::OverlapBoundary,
                (ContactRelationKind::Crossing, _) | (_, ContactRelationKind::Crossing) => {
                    ContactRelationKind::Crossing
                }
                (ContactRelationKind::Touch, ContactRelationKind::Touch) => {
                    ContactRelationKind::Touch
                }
            };
            return;
        }

        relations.push(ContactRelation {
            occurrence1,
            occurrence2,
            kind,
        });
    }

    /// Records the two primary raw occurrences for a basic self-contact and relates them using
    /// their tangent directions.
    fn add_self_intersection<P>(
        &mut self,
        raw_offset: &P,
        intersect: PlineBasicIntersect<T>,
        pos_equal_eps: T,
    ) where
        P: PlineSource<Num = T> + ?Sized,
    {
        let node_id = self.find_or_create_node(intersect.point, pos_equal_eps);
        let occurrence1 =
            self.add_occurrence(raw_offset, node_id, intersect.start_index1, pos_equal_eps);
        let occurrence2 =
            self.add_occurrence(raw_offset, node_id, intersect.start_index2, pos_equal_eps);
        self.add_relation(
            node_id,
            occurrence1,
            occurrence2,
            classify_contact(
                raw_offset,
                self.occurrences[occurrence1.0].raw_segment,
                self.occurrences[occurrence2.0].raw_segment,
                intersect.point,
                pos_equal_eps,
            ),
        );
    }

    /// Adds a primary raw occurrence for a clipping contact without a contact relation.
    ///
    /// Primary/dual intersections and open-end circle contacts split the primary raw offset only.
    /// They cannot route onto the clipping geometry.
    fn add_clip<P>(
        &mut self,
        raw_offset: &P,
        segment_index: usize,
        point: Vector2<T>,
        pos_equal_eps: T,
    ) where
        P: PlineSource<Num = T> + ?Sized,
    {
        let node_id = self.find_or_create_node(point, pos_equal_eps);
        self.add_occurrence(raw_offset, node_id, segment_index, pos_equal_eps);
    }

    /// Records both overlap endpoints and their occurrences on the reported primary raw segments.
    ///
    /// In discard mode, it also records the covered interval on each segment so later checks can
    /// remove every coincident copy instead of keeping one.
    fn add_overlap<P>(
        &mut self,
        raw_offset: &P,
        overlap: PlineOverlappingIntersect<T>,
        pos_equal_eps: T,
    ) where
        P: PlineSource<Num = T> + ?Sized,
    {
        for point in [overlap.point1, overlap.point2] {
            let node_id = self.find_or_create_node(point, pos_equal_eps);
            let occurrence1 =
                self.add_occurrence(raw_offset, node_id, overlap.start_index1, pos_equal_eps);
            let occurrence2 =
                self.add_occurrence(raw_offset, node_id, overlap.start_index2, pos_equal_eps);
            self.add_relation(
                node_id,
                occurrence1,
                occurrence2,
                ContactRelationKind::OverlapBoundary,
            );
        }

        let Some(overlap_intervals) = self.overlap_intervals.as_mut() else {
            return;
        };
        for segment_index in [overlap.start_index1, overlap.start_index2] {
            let distance1 = distance_from_segment_start(
                raw_offset,
                segment_index,
                overlap.point1,
                pos_equal_eps,
            );
            let distance2 = distance_from_segment_start(
                raw_offset,
                segment_index,
                overlap.point2,
                pos_equal_eps,
            );
            let (start, end) = min_max(distance1, distance2);
            if end > start {
                overlap_intervals[segment_index].push(OverlapInterval { start, end });
            }
        }
    }

    /// Sorts contact and overlap data after all contacts have been collected.
    ///
    /// Occurrences on each raw segment are sorted by forward distance, node ID, and occurrence ID.
    /// In discard mode, overlap intervals are sorted and merged when they overlap or are within
    /// `pos_equal_eps` of each other.
    fn finish_intersections(&mut self, pos_equal_eps: T) {
        if self.dissection_points.is_empty() {
            return;
        }
        self.dissection_points.finish(&self.occurrences);

        let Some(overlap_intervals) = &mut self.overlap_intervals else {
            return;
        };
        for intervals in overlap_intervals {
            intervals.sort_unstable_by(|a, b| a.start.total_cmp(&b.start));
            let mut merged_len = 0;
            for index in 0..intervals.len() {
                let interval = intervals[index];
                if merged_len != 0
                    && interval.start <= intervals[merged_len - 1].end + pos_equal_eps
                {
                    if interval.end > intervals[merged_len - 1].end {
                        intervals[merged_len - 1].end = interval.end;
                    }
                } else {
                    intervals[merged_len] = interval;
                    merged_len += 1;
                }
            }
            intervals.truncate(merged_len);
        }
    }

    /// Returns an `OffsetSliceSet` with the supplied slices and contact data, dropping temporary
    /// dissection and overlap data.
    fn into_slice_set(self, slices: Vec<OffsetSlice<T>>) -> OffsetSliceSet<T> {
        OffsetSliceSet {
            slices,
            nodes: self.nodes,
            occurrences: self.occurrences,
        }
    }
}

/// Converts local and global self-intersection reports into primary contact data.
struct SelfIntersectTopologyVisitor<'a, P, T, F>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    raw_offset: &'a P,
    topology: &'a mut OffsetTopologyBuilder<T>,
    filter: F,
    pos_equal_eps: T,
}

impl<P, T, F> PlineIntersectVisitor<T, Control> for SelfIntersectTopologyVisitor<'_, P, T, F>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    F: Fn(PlineIntersectFilterItem) -> Option<usize>,
{
    fn filter_map(&self, item: PlineIntersectFilterItem) -> Option<usize> {
        (self.filter)(item)
    }

    fn visit(&mut self, intersect: PlineIntersect<T>) -> Control {
        match intersect {
            PlineIntersect::Basic(intersect) => {
                self.topology
                    .add_self_intersection(self.raw_offset, intersect, self.pos_equal_eps);
            }
            PlineIntersect::Overlapping(overlap) => {
                self.topology
                    .add_overlap(self.raw_offset, overlap, self.pos_equal_eps);
            }
        }
        Control::Continue
    }
}

/// Visits local and global self-intersections with one filter and an `OffsetTopologyBuilder`.
///
/// The filter resolves each intersection item to a raw segment index and rejects invalid segments
/// before reports can become slice boundaries.
fn collect_self_intersection_topology<P, T, F>(
    raw_offset: &P,
    raw_offset_index: &StaticAABB2DIndex<T>,
    topology: &mut OffsetTopologyBuilder<T>,
    filter: F,
    pos_equal_eps: T,
) where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    F: Fn(PlineIntersectFilterItem) -> Option<usize>,
{
    let mut visitor = SelfIntersectTopologyVisitor {
        raw_offset,
        topology,
        filter,
        pos_equal_eps,
    };
    visit_local_self_intersects(raw_offset, &mut visitor, pos_equal_eps);
    visit_global_self_intersects(raw_offset, raw_offset_index, &mut visitor, pos_equal_eps);
}

/// Assigns a contact at a shared raw vertex to the outgoing segment when possible.
///
/// This gives the contact one stable identity for forward traversal: distance zero on the outgoing
/// segment. The last endpoint of an open path stays on its incoming segment.
fn canonical_segment_index<P, T>(
    polyline: &P,
    segment_index: usize,
    point: Vector2<T>,
    pos_equal_eps: T,
) -> usize
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let next_index = polyline.next_wrapping_index(segment_index);
    if !polyline
        .at(next_index)
        .pos()
        .fuzzy_eq_eps(point, pos_equal_eps)
    {
        return segment_index;
    }

    if polyline.is_closed() || next_index < polyline.segment_count() {
        next_index
    } else {
        segment_index
    }
}

/// Returns the forward distance from the start of one raw segment to `point`, for ordering
/// occurrences.
///
/// Endpoints snap with `pos_equal_eps`. For lines, it uses distance along the line. For arcs, it
/// uses the radius times the directed angle along the arc.
fn distance_from_segment_start<P, T>(
    polyline: &P,
    segment_index: usize,
    point: Vector2<T>,
    pos_equal_eps: T,
) -> T
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let v1 = polyline.at(segment_index);
    let v2 = polyline.at(polyline.next_wrapping_index(segment_index));
    if point.fuzzy_eq_eps(v1.pos(), pos_equal_eps) {
        return T::zero();
    }
    if point.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
        return seg_length(v1, v2);
    }
    if v1.bulge_is_zero() {
        let direction = v2.pos() - v1.pos();
        let length = direction.length();
        if length == T::zero() {
            T::zero()
        } else {
            (point - v1.pos()).dot(direction) / length
        }
    } else {
        let (radius, center) = seg_arc_radius_and_center(v1, v2);
        let start_angle = angle(center, v1.pos());
        let point_angle = angle(center, point);
        radius * delta_angle_signed(start_angle, point_angle, v1.bulge_is_neg()).abs()
    }
}

/// Classifies a basic self-contact from the tangents of its two raw segments.
///
/// Parallel or opposite tangents produce `Touch`; other tangents produce `Crossing`. A tangent no
/// longer than `pos_equal_eps` produces `Crossing` so routing keeps preserve behavior.
fn classify_contact<P, T>(
    polyline: &P,
    segment1: usize,
    segment2: usize,
    point: Vector2<T>,
    pos_equal_eps: T,
) -> ContactRelationKind
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let tangent1 = seg_tangent_vector(
        polyline.at(segment1),
        polyline.at(polyline.next_wrapping_index(segment1)),
        point,
    );
    let tangent2 = seg_tangent_vector(
        polyline.at(segment2),
        polyline.at(polyline.next_wrapping_index(segment2)),
        point,
    );
    let length1_squared = tangent1.length_squared();
    let length2_squared = tangent2.length_squared();
    let position_eps_squared = pos_equal_eps * pos_equal_eps;
    if length1_squared <= position_eps_squared || length2_squared <= position_eps_squared {
        return ContactRelationKind::Crossing;
    }

    let angular_epsilon = T::from(1e-5).unwrap();
    let cross = tangent1.perp_dot(tangent2);
    if cross * cross <= angular_epsilon * angular_epsilon * length1_squared * length2_squared {
        ContactRelationKind::Touch
    } else {
        ContactRelationKind::Crossing
    }
}

/// Returns whether overlap coverage contains every nonzero raw-segment span in `slice`.
///
/// It compares forward distances along each raw segment, so it works for lines and arcs. A
/// zero-length span does not need coverage.
fn slice_is_fully_coincident<P, T>(
    slice: &PlineViewData<T>,
    raw_offset: &P,
    intervals: &[Vec<OverlapInterval<T>>],
    pos_equal_eps: T,
) -> bool
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    for offset in 0..=slice.end_index_offset {
        let segment_index = raw_offset.fwd_wrapping_index(slice.start_index, offset);
        let start_point = if offset == 0 {
            slice.updated_start.pos()
        } else {
            raw_offset.at(segment_index).pos()
        };
        let end_point = if offset == slice.end_index_offset {
            slice.end_point
        } else {
            raw_offset
                .at(raw_offset.next_wrapping_index(segment_index))
                .pos()
        };
        if start_point.fuzzy_eq_eps(end_point, pos_equal_eps) {
            continue;
        }

        let distance1 =
            distance_from_segment_start(raw_offset, segment_index, start_point, pos_equal_eps);
        let distance2 =
            distance_from_segment_start(raw_offset, segment_index, end_point, pos_equal_eps);
        let (start, end) = min_max(distance1, distance2);
        if !intervals[segment_index].iter().any(|interval| {
            start + pos_equal_eps >= interval.start && end <= interval.end + pos_equal_eps
        }) {
            return false;
        }
    }
    true
}

/// Adds a proposed dissected slice if it passes validation and, in discard mode, is not fully
/// covered by overlaps.
///
/// Rejection leaves its boundary occurrences in the `OffsetTopologyBuilder`. Routing can then see
/// that no valid slice starts at the contact instead of choosing a replacement.
fn push_valid_offset_slice<P, R, T>(
    result: &mut Vec<OffsetSlice<T>>,
    view_data: PlineViewData<T>,
    start_occurrence: Option<OccurrenceId>,
    end_occurrence: Option<OccurrenceId>,
    validator: &OffsetSliceValidator<'_, P, R, T>,
    overlap_intervals: Option<&[Vec<OverlapInterval<T>>]>,
    query_stack: &mut Vec<usize>,
) where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    if !validator.slice_is_valid(&view_data, query_stack) {
        return;
    }

    if overlap_intervals.is_some_and(|intervals| {
        slice_is_fully_coincident(
            &view_data,
            validator.raw_offset_polyline,
            intervals,
            validator.pos_equal_eps,
        )
    }) {
        return;
    }

    result.push(OffsetSlice {
        view_data,
        start_occurrence,
        end_occurrence,
    });
}

/// Builds each forward dissected slice from `OffsetTopologyBuilder::dissection_points`.
///
/// For each raw segment with contact occurrences, it builds slices between consecutive occurrence
/// IDs, then between the last occurrence on that segment and the first occurrence on the next raw
/// segment with contact occurrences. For an open primary raw offset, it also builds the prefix and
/// suffix without occurrence boundaries. For a closed primary raw offset, it connects the last
/// occurrence back to the first. Each slice keeps its exact boundary occurrence IDs.
fn build_dissected_slices<P, R, T>(
    raw_offset: &R,
    topology: &OffsetTopologyBuilder<T>,
    validator: &OffsetSliceValidator<'_, P, R, T>,
    query_stack: &mut Vec<usize>,
) -> Vec<OffsetSlice<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut result = Vec::new();
    let (first_segment_index, first_points) = topology
        .dissection_points
        .first(&topology.occurrences)
        .expect("dissected slices require at least one dissection point");
    let pos_equal_eps = validator.pos_equal_eps;
    let mut push_slice =
        |start_point, start_segment, end_point, end_segment, start_occurrence, end_occurrence| {
            let Some(view_data) = PlineViewData::from_slice_points(
                raw_offset,
                start_point,
                start_segment,
                end_point,
                end_segment,
                pos_equal_eps,
            ) else {
                return;
            };
            push_valid_offset_slice(
                &mut result,
                view_data,
                start_occurrence,
                end_occurrence,
                validator,
                topology.overlap_intervals.as_deref(),
                query_stack,
            );
        };

    if !raw_offset.is_closed() {
        let end_occurrence = first_points[0];
        push_slice(
            raw_offset.at(0).pos(),
            0,
            topology.occurrences[end_occurrence.0].point(&topology.nodes),
            first_segment_index,
            None,
            Some(end_occurrence),
        );
    }

    let mut entries = topology
        .dissection_points
        .iter(&topology.occurrences)
        .peekable();
    while let Some((segment_index, points)) = entries.next() {
        for window in points.windows(2) {
            let start_occurrence = window[0];
            let end_occurrence = window[1];
            push_slice(
                topology.occurrences[start_occurrence.0].point(&topology.nodes),
                segment_index,
                topology.occurrences[end_occurrence.0].point(&topology.nodes),
                segment_index,
                Some(start_occurrence),
                Some(end_occurrence),
            );
        }

        let start_occurrence = *points.last().unwrap();
        let (end_point, end_segment, end_occurrence) =
            if let Some((next_segment_index, next_points)) = entries.peek().copied() {
                let end_occurrence = next_points[0];
                (
                    topology.occurrences[end_occurrence.0].point(&topology.nodes),
                    next_segment_index,
                    Some(end_occurrence),
                )
            } else if raw_offset.is_closed() {
                let end_occurrence = first_points[0];
                (
                    topology.occurrences[end_occurrence.0].point(&topology.nodes),
                    first_segment_index,
                    Some(end_occurrence),
                )
            } else {
                (
                    raw_offset.last().unwrap().pos(),
                    raw_offset.vertex_count() - 1,
                    None,
                )
            };
        push_slice(
            topology.occurrences[start_occurrence.0].point(&topology.nodes),
            segment_index,
            end_point,
            end_segment,
            Some(start_occurrence),
            end_occurrence,
        );
    }

    result
}

/// Converts a completed `OffsetTopologyBuilder` into an `OffsetSliceSet`.
///
/// If there are no dissection points, any locally invalid segment rejects the whole primary raw
/// offset. If all segments are valid, checking the first raw vertex is enough to keep the whole
/// path as one slice without occurrence boundaries. With dissection points, it validates each
/// dissected slice separately.
fn into_validated_slice_set<P, R, T>(
    original_polyline: &P,
    raw_offset: &RawOffsetResult<R>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
    topology: OffsetTopologyBuilder<T>,
    query_stack: &mut Vec<usize>,
) -> OffsetSliceSet<T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    let validator = OffsetSliceValidator::new(
        original_polyline,
        raw_offset_polyline,
        orig_polyline_index,
        &raw_offset.invalid_segment_indexes,
        offset,
        options,
    );
    if topology.dissection_points.is_empty() {
        if !raw_offset.invalid_segment_indexes.is_empty()
            || !validator.point_is_valid(raw_offset_polyline.at(0).pos(), query_stack)
        {
            return topology.into_slice_set(Vec::new());
        }

        return topology.into_slice_set(vec![OffsetSlice {
            view_data: PlineViewData::from_entire_pline(raw_offset_polyline),
            start_occurrence: None,
            end_occurrence: None,
        }]);
    }

    let slices = build_dissected_slices(raw_offset_polyline, &topology, &validator, query_stack);
    topology.into_slice_set(slices)
}

/// Primary raw contact data and the spatial index used by clipping queries.
struct PrimaryOffsetTopology<T: Real> {
    topology: OffsetTopologyBuilder<T>,
    aabb_index: StaticAABB2DIndex<T>,
    item_to_segment: Option<Vec<usize>>,
}

/// Indexes valid primary raw segments and collects contact data for primary raw self-intersections.
///
/// It filters locally invalid segments from local and global reports, so they cannot create slice
/// boundaries or routing targets. It records overlap coverage only in discard mode.
fn build_primary_offset_topology<R, T>(
    raw_offset: &RawOffsetResult<R>,
    options: &PlineOffsetOptions<T>,
) -> Option<PrimaryOffsetTopology<T>>
where
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    let invalid_segments = &raw_offset.invalid_segments;
    let invalid_count = raw_offset.invalid_segment_indexes.len();
    if raw_offset_polyline.vertex_count() < 2 || invalid_count == invalid_segments.len() {
        return None;
    }

    let (aabb_index, item_to_segment) = if invalid_count == 0 {
        (raw_offset_polyline.create_approx_aabb_index(), None)
    } else {
        create_segment_index(raw_offset_polyline, invalid_segments, invalid_count)
    };
    let mut topology = OffsetTopologyBuilder::new(
        raw_offset_polyline.segment_count(),
        options.coincident_segment_behavior == CoincidentSegmentBehavior::Discard,
    );
    collect_self_intersection_topology(
        raw_offset_polyline,
        &aabb_index,
        &mut topology,
        |item| match item {
            PlineIntersectFilterItem::LocalSegment(index) => {
                resolve_valid_segment(index, None, invalid_segments)
            }
            PlineIntersectFilterItem::GlobalAabbItem(item) => {
                resolve_valid_segment(item, item_to_segment.as_deref(), invalid_segments)
            }
        },
        options.pos_equal_eps,
    );

    Some(PrimaryOffsetTopology {
        topology,
        aabb_index,
        item_to_segment,
    })
}

/// Builds validated dissected slices for a closed source using only primary raw self-contacts.
fn slices_from_raw_offset<P, R, T>(
    original_polyline: &P,
    raw_offset: &RawOffsetResult<R>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> OffsetSliceSet<T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    debug_assert!(
        raw_offset_polyline.is_closed(),
        "only supports closed polylines, use slices_from_dual_raw_offsets for open polylines"
    );

    let Some(PrimaryOffsetTopology { mut topology, .. }) =
        build_primary_offset_topology(raw_offset, options)
    else {
        return OffsetSliceSet::empty();
    };
    let pos_equal_eps = options.pos_equal_eps;
    topology.finish_intersections(pos_equal_eps);

    let mut query_stack = Vec::new();
    into_validated_slice_set(
        original_polyline,
        raw_offset,
        orig_polyline_index,
        offset,
        options,
        topology,
        &mut query_stack,
    )
}

/// Circle query geometry and spatial index.
struct CircleIntersectQuery<'a, T>
where
    T: Real,
{
    center: Vector2<T>,
    radius: T,
    aabb_index: &'a StaticAABB2DIndex<T>,
    pos_equal_eps: T,
}

/// Visits circle intersections with polyline segments after resolving each spatial-index item with
/// `filter`.
///
/// `filter` returns a polyline segment index or `None`. Rejected items skip the line-circle or
/// circle-circle intersection test.
fn visit_circle_intersects<P, T, F, V>(
    pline: &P,
    query: &CircleIntersectQuery<'_, T>,
    query_stack: &mut Vec<usize>,
    filter: &F,
    visitor: &mut V,
) where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    F: Fn(usize) -> Option<usize>,
    V: FnMut(usize, Vector2<T>),
{
    let circle_center = query.center;
    let circle_radius = query.radius;
    let aabb_index = query.aabb_index;
    let pos_equal_eps = query.pos_equal_eps;

    let is_valid_line_intr = |t: T| -> bool {
        // skip false intersects and intersects at start of seg
        t >= T::zero() && t <= T::one() && t.abs() > pos_equal_eps
    };

    let is_valid_arc_intr = |arc_center: Vector2<T>,
                             arc_start: Vector2<T>,
                             arc_end: Vector2<T>,
                             bulge: T,
                             intr: Vector2<T>|
     -> bool {
        // skip false intersects and intersects at start of seg
        !arc_start.fuzzy_eq_eps(intr, pos_equal_eps)
            && point_within_arc_sweep(
                arc_center,
                arc_start,
                arc_end,
                bulge < T::zero(),
                intr,
                pos_equal_eps,
            )
    };

    let mut query_visitor = |item: usize| {
        let Some(start_index) = filter(item) else {
            return;
        };
        let v1 = pline.at(start_index);
        let v2 = pline.at(pline.next_wrapping_index(start_index));
        if v1.bulge_is_zero() {
            match line_circle_intr(
                v1.pos(),
                v2.pos(),
                circle_radius,
                circle_center,
                pos_equal_eps,
            ) {
                LineCircleIntr::NoIntersect => {}
                LineCircleIntr::TangentIntersect { t0 } => {
                    if is_valid_line_intr(t0) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t0));
                    }
                }
                LineCircleIntr::TwoIntersects { t0, t1 } => {
                    if is_valid_line_intr(t0) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t0));
                    }
                    if is_valid_line_intr(t1) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t1));
                    }
                }
            }
        } else {
            let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
            match circle_circle_intr(
                arc_radius,
                arc_center,
                circle_radius,
                circle_center,
                pos_equal_eps,
            ) {
                CircleCircleIntr::NoIntersect | CircleCircleIntr::Overlapping => {}
                CircleCircleIntr::TangentIntersect { point } => {
                    if is_valid_arc_intr(arc_center, v1.pos(), v2.pos(), v1.bulge, point) {
                        visitor(start_index, point);
                    }
                }
                CircleCircleIntr::TwoIntersects { point1, point2 } => {
                    if is_valid_arc_intr(arc_center, v1.pos(), v2.pos(), v1.bulge, point1) {
                        visitor(start_index, point1);
                    }
                    if is_valid_arc_intr(arc_center, v1.pos(), v2.pos(), v1.bulge, point2) {
                        visitor(start_index, point2);
                    }
                }
            }
        }
    };

    aabb_index.visit_query_with_stack(
        circle_center.x - circle_radius,
        circle_center.y - circle_radius,
        circle_center.x + circle_radius,
        circle_center.y + circle_radius,
        &mut query_visitor,
        query_stack,
    );
}

/// Builds validated dissected slices when the opposite-side raw offset supplies clipping contacts.
///
/// Primary/dual intersections and open-end circle contacts add only primary occurrences. They split
/// the primary raw offset but add no relation, so routing cannot move onto clipping geometry.
/// Primary/dual overlap reports from collapsed regions do not add primary contact data.
fn slices_from_dual_raw_offsets<P, R, T>(
    original_polyline: &P,
    raw_offset: &RawOffsetResult<R>,
    dual_raw_offset: &RawOffsetResult<R>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> OffsetSliceSet<T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    let invalid_segments = &raw_offset.invalid_segments;
    let dual_raw_offset_polyline = &dual_raw_offset.polyline;
    let dual_invalid_segments = &dual_raw_offset.invalid_segments;

    let Some(PrimaryOffsetTopology {
        mut topology,
        aabb_index: raw_offset_index,
        item_to_segment: raw_index_to_segment,
    }) = build_primary_offset_topology(raw_offset, options)
    else {
        return OffsetSliceSet::empty();
    };
    let pos_equal_eps = options.pos_equal_eps;

    let has_invalid = !raw_offset.invalid_segment_indexes.is_empty();
    let dual_has_invalid = !dual_raw_offset.invalid_segment_indexes.is_empty();

    let dual_intrs = if has_invalid || dual_has_invalid {
        find_intersects_filtered(
            raw_offset_polyline,
            dual_raw_offset_polyline,
            &raw_offset_index,
            |item| match item {
                TwoPlinesIntersectFilterItem::Pline1AabbItem(item) => {
                    resolve_valid_segment(item, raw_index_to_segment.as_deref(), invalid_segments)
                }
                TwoPlinesIntersectFilterItem::Pline2Segment(index) => {
                    resolve_valid_segment(index, None, dual_invalid_segments)
                }
            },
            pos_equal_eps,
        )
    } else {
        find_intersects(
            raw_offset_polyline,
            dual_raw_offset_polyline,
            &FindIntersectsOptions {
                pline1_aabb_index: Some(&raw_offset_index),
                pos_equal_eps,
            },
        )
    };

    let mut query_stack = Vec::with_capacity(8);

    if !original_polyline.is_closed() {
        // add intersects between circles generated at original open polyline end points and raw
        // offset polyline
        let circle_radius = offset.abs();
        let include_segment =
            |item| resolve_valid_segment(item, raw_index_to_segment.as_deref(), invalid_segments);
        for center in [
            original_polyline.at(0).pos(),
            original_polyline.last().unwrap().pos(),
        ] {
            let query = CircleIntersectQuery {
                center,
                radius: circle_radius,
                aabb_index: &raw_offset_index,
                pos_equal_eps,
            };
            let mut add_intr = |segment_index, point| {
                topology.add_clip(raw_offset_polyline, segment_index, point, pos_equal_eps);
            };
            visit_circle_intersects(
                raw_offset_polyline,
                &query,
                &mut query_stack,
                &include_segment,
                &mut add_intr,
            );
        }
    }

    // Only the first segment index belongs to the primary raw offset.
    for intr in dual_intrs.basic_intersects {
        debug_assert!(intr.start_index1 < raw_offset_polyline.segment_count());
        debug_assert!(intr.start_index2 < dual_raw_offset_polyline.segment_count());
        topology.add_clip(
            raw_offset_polyline,
            intr.start_index1,
            intr.point,
            pos_equal_eps,
        );
    }
    // Primary/dual overlap reports arise only from collapsed regions and do not define primary
    // topology.
    topology.finish_intersections(pos_equal_eps);

    into_validated_slice_set(
        original_polyline,
        raw_offset,
        orig_polyline_index,
        offset,
        options,
        topology,
        &mut query_stack,
    )
}

/// Routing result after adding one dissected slice.
enum Continuation {
    /// Continue with the indexed unvisited dissected slice.
    Next(usize),
    /// End the component and close it.
    Close,
    /// End the component without closing it.
    Stop,
}

/// Iterates over contact relations at the contact node for `occurrence_id`.
fn incident_relations<T>(
    topology: &OffsetSliceSet<T>,
    occurrence_id: OccurrenceId,
) -> impl Iterator<Item = &ContactRelation> {
    let node_id = topology.occurrences[occurrence_id.0].node_id;
    topology.nodes[node_id.0]
        .relations
        .iter()
        .filter(move |relation| {
            relation.occurrence1 == occurrence_id || relation.occurrence2 == occurrence_id
        })
}

/// Returns the other occurrence in a contact relation for `occurrence_id`.
fn relation_partner(relation: &ContactRelation, occurrence_id: OccurrenceId) -> OccurrenceId {
    if relation.occurrence1 == occurrence_id {
        relation.occurrence2
    } else {
        debug_assert_eq!(relation.occurrence2, occurrence_id);
        relation.occurrence1
    }
}

/// Orders two occurrences by raw segment, forward distance, and occurrence ID.
fn occurrence_order<T>(
    topology: &OffsetSliceSet<T>,
    occurrence1_id: OccurrenceId,
    occurrence2_id: OccurrenceId,
) -> std::cmp::Ordering
where
    T: Real,
{
    let occurrence1 = &topology.occurrences[occurrence1_id.0];
    let occurrence2 = &topology.occurrences[occurrence2_id.0];
    occurrence1
        .raw_segment
        .cmp(&occurrence2.raw_segment)
        .then_with(|| {
            occurrence1
                .distance_from_segment_start
                .total_cmp(&occurrence2.distance_from_segment_start)
        })
        .then_with(|| occurrence1_id.cmp(&occurrence2_id))
}

/// Returns the unvisited dissected slice that starts at `occurrence`, if any.
fn unvisited_start(
    occurrence: OccurrenceId,
    starts_by_occurrence: &[Option<usize>],
    visited: &[bool],
) -> Option<usize> {
    starts_by_occurrence[occurrence.0].filter(|&index| !visited[index])
}

/// Chooses the next slice while preserving raw offset traversal through contacts.
///
/// It tries, in order: an unvisited slice starting at the ending occurrence, an unvisited slice
/// starting at a related occurrence, closing at the component's initial occurrence, then stopping.
/// It tries closing last so touching loops and repeated slices are consumed first.
fn choose_preserve_continuation<T>(
    topology: &OffsetSliceSet<T>,
    end_occurrence: OccurrenceId,
    initial_occurrence: Option<OccurrenceId>,
    starts_by_occurrence: &[Option<usize>],
    visited: &[bool],
    coincident_segment_behavior: CoincidentSegmentBehavior,
) -> Continuation
where
    T: Real,
{
    if let Some(next) = unvisited_start(end_occurrence, starts_by_occurrence, visited) {
        return Continuation::Next(next);
    }

    // A discarded overlap leaves no same-occurrence slice through its boundary. Preserve must not
    // use a Touch at that node as a substitute; Separate handles that relation explicitly.
    let discard_overlap_at_occurrence = coincident_segment_behavior
        == CoincidentSegmentBehavior::Discard
        && incident_relations(topology, end_occurrence)
            .any(|relation| relation.kind == ContactRelationKind::OverlapBoundary);
    let relation_can_continue = |relation: &ContactRelation| {
        !(discard_overlap_at_occurrence && relation.kind == ContactRelationKind::Touch)
    };
    let next = incident_relations(topology, end_occurrence)
        .filter(|relation| relation_can_continue(relation))
        .map(|relation| relation_partner(relation, end_occurrence))
        .filter_map(|occurrence| {
            unvisited_start(occurrence, starts_by_occurrence, visited)
                .map(|slice_index| (occurrence, slice_index))
        })
        .min_by(|(occurrence1, _), (occurrence2, _)| {
            occurrence_order(topology, *occurrence1, *occurrence2)
        });
    if let Some((_, slice_index)) = next {
        return Continuation::Next(slice_index);
    }

    if initial_occurrence.is_some_and(|initial| {
        initial == end_occurrence
            || incident_relations(topology, end_occurrence)
                .filter(|relation| relation_can_continue(relation))
                .any(|relation| relation_partner(relation, end_occurrence) == initial)
    }) {
        return Continuation::Close;
    }

    Continuation::Stop
}

/// Applies separate-touch rules when the ending occurrence has a `Touch` relation; otherwise uses
/// preserve routing.
///
/// In separate mode, it closes first when a `Touch` returns to the initial occurrence, follows the
/// only unvisited `Touch` partner, stops when none remains, and uses preserve order when several
/// partners remain. A `Crossing` relation alone does not use separate-touch routing.
fn choose_continuation<T>(
    topology: &OffsetSliceSet<T>,
    end_occurrence: Option<OccurrenceId>,
    initial_occurrence: Option<OccurrenceId>,
    starts_by_occurrence: &[Option<usize>],
    visited: &[bool],
    touching_loop_behavior: TouchingLoopBehavior,
    coincident_segment_behavior: CoincidentSegmentBehavior,
) -> Continuation
where
    T: Real,
{
    let Some(end_occurrence) = end_occurrence else {
        return Continuation::Stop;
    };

    if touching_loop_behavior == TouchingLoopBehavior::Separate {
        let has_touch = incident_relations(topology, end_occurrence)
            .any(|relation| relation.kind == ContactRelationKind::Touch);
        if has_touch {
            if initial_occurrence.is_some_and(|initial| {
                initial == end_occurrence
                    || incident_relations(topology, end_occurrence)
                        .filter(|relation| relation.kind == ContactRelationKind::Touch)
                        .any(|relation| relation_partner(relation, end_occurrence) == initial)
            }) {
                return Continuation::Close;
            }

            let mut paired_candidates = incident_relations(topology, end_occurrence)
                .filter(|relation| relation.kind == ContactRelationKind::Touch)
                .filter_map(|relation| {
                    unvisited_start(
                        relation_partner(relation, end_occurrence),
                        starts_by_occurrence,
                        visited,
                    )
                });
            return match (paired_candidates.next(), paired_candidates.next()) {
                (Some(next), None) => Continuation::Next(next),
                (None, _) => Continuation::Stop,
                (Some(_), Some(_)) => {
                    // Pair relations do not identify one continuation at an ambiguous multi-way
                    // touch. Preserve primary raw traversal rather than guessing from tangent angle
                    // or position.
                    choose_preserve_continuation(
                        topology,
                        end_occurrence,
                        initial_occurrence,
                        starts_by_occurrence,
                        visited,
                        coincident_segment_behavior,
                    )
                }
            };
        }
    }

    choose_preserve_continuation(
        topology,
        end_occurrence,
        initial_occurrence,
        starts_by_occurrence,
        visited,
        coincident_segment_behavior,
    )
}

/// Checks that the selected slices meet at the same contact node and that their boundary points
/// match.
///
/// This checks an invariant; it does not search for another connection. A failed check stops
/// assembly in release builds.
fn selected_connection_is_valid<T>(
    topology: &OffsetSliceSet<T>,
    current_slice: &OffsetSlice<T>,
    next_slice: &OffsetSlice<T>,
    pos_equal_eps: T,
) -> bool
where
    T: Real,
{
    let (Some(end_occurrence), Some(start_occurrence)) =
        (current_slice.end_occurrence, next_slice.start_occurrence)
    else {
        return false;
    };
    let end = &topology.occurrences[end_occurrence.0];
    let start = &topology.occurrences[start_occurrence.0];
    if end.node_id != start.node_id {
        return false;
    }
    let contact_point = topology.nodes[end.node_id.0].point;
    current_slice
        .view_data
        .end_point
        .fuzzy_eq_eps(contact_point, pos_equal_eps)
        && next_slice
            .view_data
            .updated_start
            .pos()
            .fuzzy_eq_eps(contact_point, pos_equal_eps)
}

/// Checks that nodes own their occurrences, relations stay within one node, and slice boundaries
/// match before routing.
#[cfg(debug_assertions)]
fn debug_validate_topology<T>(topology: &OffsetSliceSet<T>, pos_equal_eps: T)
where
    T: Real,
{
    for (index, occurrence) in topology.occurrences.iter().enumerate() {
        let occurrence_id = OccurrenceId(index);
        debug_assert!(occurrence.node_id.0 < topology.nodes.len());
        debug_assert!(
            topology.nodes[occurrence.node_id.0]
                .occurrences
                .contains(&occurrence_id)
        );
    }
    for (node_index, node) in topology.nodes.iter().enumerate() {
        let node_id = ContactNodeId(node_index);
        for occurrence in &node.occurrences {
            debug_assert_eq!(topology.occurrences[occurrence.0].node_id, node_id);
        }
        for relation in &node.relations {
            debug_assert_ne!(relation.occurrence1, relation.occurrence2);
            debug_assert_eq!(
                topology.occurrences[relation.occurrence1.0].node_id,
                node_id
            );
            debug_assert_eq!(
                topology.occurrences[relation.occurrence2.0].node_id,
                node_id
            );
        }
    }
    for slice in &topology.slices {
        if let Some(occurrence) = slice.start_occurrence {
            debug_assert!(slice.view_data.updated_start.pos().fuzzy_eq_eps(
                topology.occurrences[occurrence.0].point(&topology.nodes),
                pos_equal_eps
            ));
        }
        if let Some(occurrence) = slice.end_occurrence {
            debug_assert!(slice.view_data.end_point.fuzzy_eq_eps(
                topology.occurrences[occurrence.0].point(&topology.nodes),
                pos_equal_eps
            ));
        }
    }
}

/// Routes all validated dissected slices into output components using their contact occurrences.
///
/// A dense table maps each start occurrence to its slice, avoiding a position search. It copies
/// slice geometry and removes repeated positions after choosing the route. A component
/// closes only when the contact records allow it, except for a whole closed primary raw offset
/// with no contacts.
fn stitch_slices_together<P, T, O>(
    raw_offset_pline: &P,
    topology: &OffsetSliceSet<T>,
    is_closed: bool,
    options: &PlineOffsetOptions<T>,
) -> Vec<O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    if topology.slices.is_empty() {
        return Vec::new();
    }

    let pos_equal_eps = options.pos_equal_eps;
    #[cfg(debug_assertions)]
    debug_validate_topology(topology, pos_equal_eps);

    let mut result = Vec::new();
    let mut starts_by_occurrence = vec![None; topology.occurrences.len()];
    let mut visited = vec![false; topology.slices.len()];
    for (index, slice) in topology.slices.iter().enumerate() {
        if let Some(occurrence) = slice.start_occurrence {
            let start = &mut starts_by_occurrence[occurrence.0];
            debug_assert!(
                start.is_none(),
                "one raw occurrence cannot start more than one dissected slice"
            );
            *start = Some(index);
        }
    }

    for i in 0..topology.slices.len() {
        if visited[i] {
            continue;
        }

        let mut current_pline = O::empty();
        let mut current_index = i;
        let initial_occurrence = topology.slices[i].start_occurrence;
        let mut close_component = is_closed
            && topology.slices.len() == 1
            && initial_occurrence.is_none()
            && topology.slices[i].end_occurrence.is_none();
        loop {
            debug_assert!(!visited[current_index]);
            visited[current_index] = true;
            let current_slice = &topology.slices[current_index];
            current_pline.extend_remove_repeat(
                &current_slice.view_data.view(raw_offset_pline),
                pos_equal_eps,
            );

            match choose_continuation(
                topology,
                current_slice.end_occurrence,
                initial_occurrence,
                &starts_by_occurrence,
                &visited,
                options.touching_loop_behavior,
                options.coincident_segment_behavior,
            ) {
                Continuation::Next(next_index) => {
                    let connection_is_valid = selected_connection_is_valid(
                        topology,
                        current_slice,
                        &topology.slices[next_index],
                        pos_equal_eps,
                    );
                    debug_assert!(
                        connection_is_valid,
                        "selected offset slices do not share one canonical contact"
                    );
                    if !connection_is_valid {
                        break;
                    }
                    current_index = next_index;
                }
                Continuation::Close => {
                    close_component = true;
                    break;
                }
                Continuation::Stop => break,
            }
        }

        if close_component && is_closed && current_pline.vertex_count() > 1 {
            let endpoints_match = current_pline
                .at(0)
                .pos()
                .fuzzy_eq_eps(current_pline.last().unwrap().pos(), pos_equal_eps);
            debug_assert!(
                endpoints_match,
                "closed offset topology has mismatched geometric endpoints"
            );
            if endpoints_match {
                current_pline.remove_last();
                if current_pline.vertex_count() > 1 {
                    current_pline.set_is_closed(true);
                }
            }
        }
        if current_pline.vertex_count() > 1 {
            result.push(current_pline);
        }
    }

    result
}

/// Runs the offset stages for a prepared source polyline and its spatial index.
///
/// A closed source without self-intersection handling uses only primary raw contacts. An open
/// source, or one with self-intersection handling enabled, also builds the opposite-side raw offset
/// for clipping contacts.
fn parallel_offset_for_source<P, T, O>(
    polyline: &P,
    offset: T,
    options: &PlineOffsetOptions<T>,
    allow_external_index: bool,
) -> Vec<O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    let constructed_index;
    let index = if allow_external_index {
        if let Some(x) = options.aabb_index {
            x
        } else {
            constructed_index = polyline.create_approx_aabb_index();
            &constructed_index
        }
    } else {
        constructed_index = polyline.create_approx_aabb_index();
        &constructed_index
    };

    let raw_offset: RawOffsetResult<O> = create_raw_offset(polyline, offset, options.pos_equal_eps);
    if raw_offset.polyline.is_empty() {
        Vec::new()
    } else if polyline.is_closed() && !options.handle_self_intersects {
        let slices = slices_from_raw_offset(polyline, &raw_offset, index, offset, options);
        stitch_slices_together(&raw_offset.polyline, &slices, true, options)
    } else {
        let dual_raw_offset: RawOffsetResult<O> =
            create_raw_offset(polyline, -offset, options.pos_equal_eps);
        let slices = slices_from_dual_raw_offsets(
            polyline,
            &raw_offset,
            &dual_raw_offset,
            index,
            offset,
            options,
        );

        stitch_slices_together(&raw_offset.polyline, &slices, polyline.is_closed(), options)
    }
}

/// Builds polyline parallel offsets by creating a raw offset, splitting it into validated
/// dissected slices, and stitching those slices.
///
/// A zero offset returns a copy. A nonzero offset first removes repeated source positions. If that
/// changes segment indexes, it rebuilds the spatial index instead of using the caller's index.
pub fn parallel_offset<P, T, O>(polyline: &P, offset: T, options: &PlineOffsetOptions<T>) -> Vec<O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    if polyline.vertex_count() < 2 {
        return Vec::new();
    }

    if offset == T::zero() {
        return vec![O::create_from(polyline)];
    }

    // Sanitize repeat positions to prevent unstable/degenerate segments.
    let mut result = if let Some(cleaned) = polyline.remove_repeat_pos(options.pos_equal_eps) {
        if cleaned.vertex_count() < 2 {
            Vec::<O>::new()
        } else {
            // user-provided aabb index is tied to the original polyline, rebuild for cleaned source
            parallel_offset_for_source(&cleaned, offset, options, false)
        }
    } else {
        parallel_offset_for_source(polyline, offset, options, true)
    };

    for pline in &mut result {
        pline.set_userdata_values(polyline.get_userdata_values());
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polyline::{PlineSourceMut, Polyline};

    fn open(vertexes: &[(f64, f64, f64)]) -> Polyline<f64> {
        let mut result = Polyline::new();
        for &(x, y, bulge) in vertexes {
            result.add(x, y, bulge);
        }
        result
    }

    #[test]
    fn slices_with_invalid_indexes_are_rejected() {
        let source = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]);
        let slice = PlineViewData::from_entire_pline(&source);
        assert!(slice_contains_invalid_segment(&slice, false, 1, &[0]));

        let wrapped_slice = PlineViewData {
            start_index: 3,
            end_index_offset: 1,
            ..slice
        };
        assert!(slice_contains_invalid_segment(
            &wrapped_slice,
            true,
            4,
            &[0]
        ));
        assert!(!slice_contains_invalid_segment(
            &wrapped_slice,
            true,
            4,
            &[1, 2]
        ));
    }

    #[test]
    fn segment_index_compacts_only_for_dense_invalidity() {
        let polyline = open(&[
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (5.0, 0.0, 0.0),
        ]);

        let (sparse_index, sparse_mapping) =
            create_segment_index(&polyline, &[false, true, false, false, false], 1);
        assert_eq!(sparse_index.item_indices().len(), 5);
        assert!(sparse_mapping.is_none());

        let (dense_index, dense_mapping) =
            create_segment_index(&polyline, &[true, false, true, false, true], 3);
        assert_eq!(dense_index.item_indices().len(), 2);
        assert_eq!(dense_mapping.unwrap(), [1, 3]);
    }

    fn routing_topology(
        raw_segments: &[usize],
        relations: &[(usize, usize, ContactRelationKind)],
    ) -> OffsetSliceSet<f64> {
        let point = Vector2::zero();
        let occurrences = raw_segments
            .iter()
            .map(|raw_segment| ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment: *raw_segment,
                distance_from_segment_start: 0.0,
            })
            .collect::<Vec<_>>();
        let relations = relations
            .iter()
            .map(|&(occurrence1, occurrence2, kind)| ContactRelation {
                occurrence1: OccurrenceId(occurrence1),
                occurrence2: OccurrenceId(occurrence2),
                kind,
            })
            .collect::<Vec<_>>();
        let nodes = vec![ContactNode {
            point,
            occurrences: (0..occurrences.len()).map(OccurrenceId).collect(),
            relations,
        }];
        OffsetSliceSet {
            slices: Vec::new(),
            nodes,
            occurrences,
        }
    }

    fn builder_with_occurrences(raw_segments: &[usize]) -> OffsetTopologyBuilder<f64> {
        let mut topology = OffsetTopologyBuilder::new(0, false);
        topology.nodes.push(ContactNode {
            point: Vector2::zero(),
            occurrences: Vec::new(),
            relations: Vec::new(),
        });
        for &raw_segment in raw_segments {
            let occurrence_id = OccurrenceId(topology.occurrences.len());
            topology.occurrences.push(ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment,
                distance_from_segment_start: 0.0,
            });
            topology.nodes[0].occurrences.push(occurrence_id);
        }
        topology
    }

    #[test]
    fn dissection_points_sort_into_segment_ranges() {
        let occurrences = vec![
            ContactOccurrence {
                node_id: ContactNodeId(1),
                raw_segment: 1,
                distance_from_segment_start: 2.0,
            },
            ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment: 0,
                distance_from_segment_start: 2.0,
            },
            ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment: 0,
                distance_from_segment_start: 1.0,
            },
            ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment: 1,
                distance_from_segment_start: 1.0,
            },
        ];
        let mut points = DissectionPoints::new();
        for occurrence_id in (0..occurrences.len()).map(OccurrenceId) {
            points.push(occurrence_id);
        }

        points.finish(&occurrences);

        let groups = points
            .iter(&occurrences)
            .map(|(raw_segment, occurrence_ids)| (raw_segment, occurrence_ids.to_vec()))
            .collect::<Vec<_>>();
        assert_eq!(
            groups,
            vec![
                (0, vec![OccurrenceId(2), OccurrenceId(1)]),
                (1, vec![OccurrenceId(3), OccurrenceId(0)]),
            ]
        );
    }

    #[test]
    fn repeated_clip_reports_share_one_occurrence() {
        let polyline = open(&[(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]);
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
        let point = Vector2::new(2.0, 0.0);

        topology.add_clip(&polyline, 0, point, 1e-5);
        topology.add_clip(&polyline, 0, point, 1e-5);

        assert_eq!(topology.nodes.len(), 1);
        assert_eq!(topology.occurrences.len(), 1);
        assert_eq!(topology.nodes[0].occurrences, [OccurrenceId(0)]);
        assert_eq!(topology.dissection_points.occurrence_ids, [OccurrenceId(0)]);
    }

    #[test]
    fn repeated_self_contact_reports_share_one_relation() {
        let polyline = open(&[
            (-2.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, -2.0, 0.0),
            (0.0, 2.0, 0.0),
        ]);
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
        let intersect = PlineBasicIntersect::new(0, 2, Vector2::zero());

        topology.add_self_intersection(&polyline, intersect, 1e-5);
        topology.add_self_intersection(&polyline, intersect, 1e-5);

        assert_eq!(topology.nodes.len(), 1);
        assert_eq!(topology.occurrences.len(), 2);
        assert_eq!(topology.nodes[0].relations.len(), 1);
    }

    #[test]
    fn reversed_relation_reports_merge_and_strengthen_one_pair() {
        let mut topology = builder_with_occurrences(&[0, 1]);
        let node_id = ContactNodeId(0);
        let occurrence0 = OccurrenceId(0);
        let occurrence1 = OccurrenceId(1);

        topology.add_relation(
            node_id,
            occurrence0,
            occurrence1,
            ContactRelationKind::Touch,
        );
        topology.add_relation(
            node_id,
            occurrence1,
            occurrence0,
            ContactRelationKind::Crossing,
        );
        topology.add_relation(
            node_id,
            occurrence0,
            occurrence1,
            ContactRelationKind::OverlapBoundary,
        );

        assert_eq!(topology.nodes[0].relations.len(), 1);
        assert_eq!(
            topology.nodes[0].relations[0].kind,
            ContactRelationKind::OverlapBoundary
        );
    }

    #[test]
    fn shared_endpoint_reports_use_one_occurrence_in_both_directions() {
        fn occurrence_segment_at_shared_endpoint(polyline: &Polyline<f64>) -> usize {
            let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
            let point = polyline.at(1).pos();
            topology.add_clip(polyline, 0, point, 1e-5);
            topology.add_clip(polyline, 1, point, 1e-5);
            assert_eq!(topology.occurrences.len(), 1);
            topology.occurrences[0].raw_segment
        }

        let polyline = open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]);
        let mut inverted = polyline.clone();
        inverted.invert_direction_mut();

        assert_eq!(
            occurrence_segment_at_shared_endpoint(&polyline),
            occurrence_segment_at_shared_endpoint(&inverted)
        );
    }

    #[test]
    fn preserve_routing_visits_related_slice_before_closing() {
        let topology = routing_topology(
            &[0, 1, 2],
            &[
                (0, 1, ContactRelationKind::Crossing),
                (0, 2, ContactRelationKind::Crossing),
            ],
        );
        let starts = vec![None, None, Some(0)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                Some(OccurrenceId(1)),
                &starts,
                &[false],
                TouchingLoopBehavior::Preserve,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Next(0)
        ));
        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                Some(OccurrenceId(1)),
                &starts,
                &[true],
                TouchingLoopBehavior::Preserve,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Close
        ));
    }

    #[test]
    fn separate_behavior_keeps_crossing_closure_order() {
        let topology = routing_topology(&[0, 1], &[(0, 1, ContactRelationKind::Crossing)]);
        let starts = vec![None, Some(0)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                Some(OccurrenceId(0)),
                &starts,
                &[false],
                TouchingLoopBehavior::Separate,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Next(0)
        ));
    }

    #[test]
    fn separate_routing_prioritizes_relation_to_initial_occurrence() {
        let topology = routing_topology(
            &[0, 1, 2],
            &[
                (0, 1, ContactRelationKind::Touch),
                (0, 2, ContactRelationKind::Touch),
            ],
        );
        let starts = vec![None, None, Some(0)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                Some(OccurrenceId(1)),
                &starts,
                &[false],
                TouchingLoopBehavior::Separate,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Close
        ));
    }

    #[test]
    fn ambiguous_multiway_touch_falls_back_to_preserve_order() {
        let topology = routing_topology(
            &[0, 1, 2],
            &[
                (0, 1, ContactRelationKind::Touch),
                (0, 2, ContactRelationKind::Touch),
            ],
        );
        let starts = vec![Some(0), Some(1), Some(2)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                None,
                &starts,
                &[false, false, false],
                TouchingLoopBehavior::Separate,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Next(0)
        ));
    }

    #[test]
    fn discarded_overlap_blocks_preserve_touch_routing_at_shared_boundary() {
        let topology = routing_topology(
            &[0, 1, 2],
            &[
                (0, 1, ContactRelationKind::OverlapBoundary),
                (0, 2, ContactRelationKind::Touch),
            ],
        );
        let starts = vec![None, None, Some(0)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                None,
                &starts,
                &[false],
                TouchingLoopBehavior::Preserve,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Next(0)
        ));
        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                None,
                &starts,
                &[false],
                TouchingLoopBehavior::Preserve,
                CoincidentSegmentBehavior::Discard,
            ),
            Continuation::Stop
        ));
        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                None,
                &starts,
                &[false],
                TouchingLoopBehavior::Separate,
                CoincidentSegmentBehavior::Discard,
            ),
            Continuation::Next(0)
        ));
    }

    #[test]
    fn unrelated_nodes_do_not_connect_by_position() {
        let point = Vector2::zero();
        let nodes = vec![
            ContactNode {
                point,
                occurrences: vec![OccurrenceId(0)],
                relations: Vec::new(),
            },
            ContactNode {
                point: Vector2::new(1e-8, 0.0),
                occurrences: vec![OccurrenceId(1)],
                relations: Vec::new(),
            },
        ];
        let occurrences = vec![
            ContactOccurrence {
                node_id: ContactNodeId(0),
                raw_segment: 0,
                distance_from_segment_start: 0.0,
            },
            ContactOccurrence {
                node_id: ContactNodeId(1),
                raw_segment: 1,
                distance_from_segment_start: 0.0,
            },
        ];
        let topology = OffsetSliceSet {
            slices: Vec::new(),
            nodes,
            occurrences,
        };
        let starts = vec![None, Some(0)];

        assert!(matches!(
            choose_continuation(
                &topology,
                Some(OccurrenceId(0)),
                None,
                &starts,
                &[false],
                TouchingLoopBehavior::Preserve,
                CoincidentSegmentBehavior::Preserve,
            ),
            Continuation::Stop
        ));
    }

    #[test]
    fn segment_end_occurrences_are_canonicalized_to_outgoing_segment() {
        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, 0.0, 0.0);
        polyline.add(1.0, 0.0, 0.0);
        polyline.add(1.0, 1.0, 0.0);
        polyline.add(0.0, 1.0, 0.0);
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);

        topology.add_self_intersection(
            &polyline,
            PlineBasicIntersect::new(0, 2, Vector2::new(1.0, 0.0)),
            1e-5,
        );

        assert!(
            !topology
                .occurrences
                .iter()
                .any(|occurrence| occurrence.raw_segment == 0)
        );
        assert!(
            topology
                .occurrences
                .iter()
                .any(|occurrence| occurrence.raw_segment == 1)
        );
        assert_eq!(
            topology.occurrences[0].raw_segment, 1,
            "the occurrence must use the segment leaving the vertex"
        );
    }

    #[test]
    fn raw_order_snaps_near_arc_endpoints_with_position_epsilon() {
        let polyline = open(&[(-1.0, 0.0, -1.0), (1.0, 0.0, 0.0)]);

        assert!(
            distance_from_segment_start(&polyline, 0, Vector2::new(-1.0, -1e-6), 1e-5).abs()
                <= 1e-10
        );
        assert!(
            (distance_from_segment_start(&polyline, 0, Vector2::new(1.0, 1e-6), 1e-5)
                - std::f64::consts::PI)
                .abs()
                <= 1e-10
        );
    }

    #[test]
    fn short_tangent_is_classified_as_crossing() {
        let polyline = open(&[(0.0, 0.0, 0.0), (1e-6, 0.0, 0.0), (0.0, 1.0, 0.0)]);

        assert_eq!(
            classify_contact(&polyline, 0, 1, polyline.at(1).pos(), 1e-5),
            ContactRelationKind::Crossing
        );
    }

    #[test]
    fn one_occurrence_can_participate_in_several_relations() {
        let polyline = open(&[
            (-2.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, -2.0, 0.0),
            (0.0, 2.0, 0.0),
            (-2.0, -2.0, 0.0),
            (2.0, 2.0, 0.0),
        ]);
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
        let point = Vector2::zero();
        topology.add_self_intersection(&polyline, PlineBasicIntersect::new(0, 2, point), 1e-5);
        topology.add_self_intersection(&polyline, PlineBasicIntersect::new(0, 4, point), 1e-5);

        assert_eq!(topology.nodes.len(), 1);
        assert_eq!(topology.nodes[0].occurrences.len(), 3);
        assert_eq!(topology.nodes[0].relations.len(), 2);
        let occurrence0 = topology.nodes[0]
            .occurrences
            .iter()
            .find(|occurrence| topology.occurrences[occurrence.0].raw_segment == 0)
            .unwrap();
        assert_eq!(
            topology.nodes[0]
                .relations
                .iter()
                .filter(|relation| {
                    relation.occurrence1 == *occurrence0 || relation.occurrence2 == *occurrence0
                })
                .count(),
            2
        );
    }

    #[test]
    fn one_segment_pair_can_have_two_distinct_contact_nodes() {
        let polyline = open(&[
            (-3.0, 1.0, 0.0),
            (3.0, 1.0, 0.0),
            (-2.0, 0.0, -1.0),
            (2.0, 0.0, 0.0),
        ]);
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
        let x = 3.0_f64.sqrt();
        topology.add_self_intersection(
            &polyline,
            PlineBasicIntersect::new(0, 2, Vector2::new(-x, 1.0)),
            1e-5,
        );
        topology.add_self_intersection(
            &polyline,
            PlineBasicIntersect::new(0, 2, Vector2::new(x, 1.0)),
            1e-5,
        );

        assert_eq!(topology.nodes.len(), 2);
        assert!(topology.nodes.iter().all(|node| node.relations.len() == 1));
    }

    #[test]
    fn self_intersection_collection_records_full_arc_overlap() {
        let polyline = open(&[
            (-1.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
        ]);
        let index = polyline.create_approx_aabb_index();
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), true);
        collect_self_intersection_topology(
            &polyline,
            &index,
            &mut topology,
            |item| Some(item.index()),
            1e-5,
        );
        topology.finish_intersections(1e-5);

        let intervals = topology.overlap_intervals.unwrap();
        assert!(intervals[0].iter().any(|interval| {
            interval.start.abs() <= 1e-5 && (interval.end - std::f64::consts::PI).abs() <= 1e-5
        }));
    }

    #[test]
    fn preserve_collection_records_overlap_boundaries_without_coverage() {
        let polyline = open(&[
            (-1.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, -1.0),
            (1.0, 0.0, 0.0),
        ]);
        let index = polyline.create_approx_aabb_index();
        let mut topology = OffsetTopologyBuilder::new(polyline.segment_count(), false);
        collect_self_intersection_topology(
            &polyline,
            &index,
            &mut topology,
            |item| Some(item.index()),
            1e-5,
        );

        assert!(topology.overlap_intervals.is_none());
        assert!(topology.nodes.iter().any(|node| {
            node.relations
                .iter()
                .any(|relation| relation.kind == ContactRelationKind::OverlapBoundary)
        }));
    }

    #[test]
    fn coincident_coverage_can_span_lines_and_arcs() {
        let line_polyline = open(&[(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (20.0, 0.0, 0.0)]);
        let line_slice = PlineViewData::from_entire_pline(&line_polyline);
        let line_intervals = vec![
            vec![OverlapInterval {
                start: 0.0,
                end: 10.0,
            }],
            vec![OverlapInterval {
                start: 0.0,
                end: 10.0,
            }],
        ];
        assert!(slice_is_fully_coincident(
            &line_slice,
            &line_polyline,
            &line_intervals,
            1e-5
        ));

        let partial_intervals = vec![
            line_intervals[0].clone(),
            vec![OverlapInterval {
                start: 0.0,
                end: 5.0,
            }],
        ];
        assert!(!slice_is_fully_coincident(
            &line_slice,
            &line_polyline,
            &partial_intervals,
            1e-5
        ));

        let arc_polyline = open(&[(-1.0, 0.0, 1.0), (1.0, 0.0, 0.0)]);
        let arc_slice = PlineViewData::from_entire_pline(&arc_polyline);
        let arc_intervals = vec![vec![OverlapInterval {
            start: 0.0,
            end: std::f64::consts::PI,
        }]];
        assert!(slice_is_fully_coincident(
            &arc_slice,
            &arc_polyline,
            &arc_intervals,
            1e-5
        ));
    }
}
