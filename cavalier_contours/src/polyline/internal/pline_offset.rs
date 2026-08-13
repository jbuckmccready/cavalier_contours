use crate::{
    core::{
        math::{
            CircleCircleIntr, LineCircleIntr, Vector2, circle_circle_intr, dist_squared,
            line_circle_intr, point_from_parametric, point_within_arc_sweep,
        },
        traits::Real,
    },
    polyline::{
        FindIntersectsOptions, PlineCreation, PlineIntersectFilterItem, PlineOffsetOptions,
        PlineSegIntr, PlineSource, PlineVertex, PlineViewData, TwoPlinesIntersectFilterItem,
        internal::{
            pline_intersects::{
                all_self_intersects_as_basic, all_self_intersects_as_basic_filtered,
                find_intersects, find_intersects_filtered,
            },
            raw_pline_offset::{RawOffsetResult, create_raw_offset},
        },
        pline_seg_intr, seg_arc_radius_and_center, seg_closest_point, seg_fast_approx_bounding_box,
        seg_midpoint,
    },
};
use static_aabb2d_index::{Control, StaticAABB2DIndex, StaticAABB2DIndexBuilder};
use std::collections::BTreeMap;

/// Tests global distance interference. Local folds are rejected separately through raw source-span
/// validity so `offset_tol` can remain a relaxed global distance tolerance.
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
            Control::Continue
        } else {
            Control::Break(())
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

/// Creates a spatial index for offset segments and, when useful, a mapping from its compact item
/// IDs to polyline segment indexes.
///
/// Sparse invalidity keeps the normal full index so rebuilding it costs less than the extra AABB
/// candidates. Dense invalidity omits invalid segment boxes and returns the mapping required to
/// resolve compact item IDs.
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

pub fn slices_from_raw_offset<P, R, T>(
    original_polyline: &P,
    raw_offset: &RawOffsetResult<R>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> Vec<PlineViewData<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    let invalid_segments = &raw_offset.invalid_segments;
    debug_assert!(
        raw_offset_polyline.is_closed(),
        "only supports closed polylines, use slices_from_dual_raw_offsets for open polylines"
    );

    let mut result = Vec::new();
    if raw_offset_polyline.vertex_count() < 2 {
        return result;
    }

    let pos_equal_eps = options.pos_equal_eps;
    let offset_dist_eps = options.offset_dist_eps;

    let invalid_segment_indexes = &raw_offset.invalid_segment_indexes;
    let has_invalid = !invalid_segment_indexes.is_empty();
    if invalid_segment_indexes.len() == invalid_segments.len() {
        return result;
    }
    let raw_offset_index;
    let self_intrs = if has_invalid {
        let raw_index_to_segment;
        (raw_offset_index, raw_index_to_segment) = create_segment_index(
            raw_offset_polyline,
            invalid_segments,
            invalid_segment_indexes.len(),
        );
        all_self_intersects_as_basic_filtered(
            raw_offset_polyline,
            &raw_offset_index,
            |item| {
                let index = match item {
                    PlineIntersectFilterItem::LocalSegment(index) => index,
                    PlineIntersectFilterItem::GlobalAabbItem(item) => raw_index_to_segment
                        .as_deref()
                        .map_or(item, |mapping| mapping[item]),
                };
                (!invalid_segments[index]).then_some(index)
            },
            false,
            pos_equal_eps,
        )
    } else {
        raw_offset_index = raw_offset_polyline.create_approx_aabb_index();
        all_self_intersects_as_basic(raw_offset_polyline, &raw_offset_index, false, pos_equal_eps)
    };

    let mut intersects_lookup = BTreeMap::<usize, Vec<Vector2<T>>>::new();
    for si in &self_intrs {
        intersects_lookup
            .entry(si.start_index1)
            .or_default()
            .push(si.point);
        intersects_lookup
            .entry(si.start_index2)
            .or_default()
            .push(si.point);
    }
    let mut query_stack = Vec::new();
    if intersects_lookup.is_empty() {
        if has_invalid {
            return result;
        }

        let slice = PlineViewData::from_entire_pline(raw_offset_polyline);

        // no self intersects, test point on polyline is valid
        if !point_valid_for_offset(
            original_polyline,
            offset,
            orig_polyline_index,
            raw_offset_polyline.at(0).pos(),
            &mut query_stack,
            pos_equal_eps,
            offset_dist_eps,
        ) {
            // not valid
            return result;
        }

        // is valid
        result.push(slice);
        return result;
    }

    // sort intersects by distance from segment start vertex
    for (&i, intr_list) in &mut intersects_lookup {
        let start_pos = raw_offset_polyline.at(i).pos();
        intr_list.sort_unstable_by(|&si1, &si2| {
            let dist1 = dist_squared(si1, start_pos);
            let dist2 = dist_squared(si2, start_pos);
            dist1.total_cmp(&dist2)
        });
    }

    let intersects_original_pline =
        |v1: PlineVertex<T>, v2: PlineVertex<T>, query_stack: &mut Vec<usize>| -> bool {
            let approx_bb = seg_fast_approx_bounding_box(v1, v2);
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
                    Control::Break(())
                } else {
                    Control::Continue
                }
            };

            let fuzz = T::fuzzy_epsilon();
            orig_polyline_index.visit_query_with_stack(
                approx_bb.min_x - fuzz,
                approx_bb.min_y - fuzz,
                approx_bb.max_x + fuzz,
                approx_bb.max_y + fuzz,
                &mut visitor,
                query_stack,
            );
            has_intersect
        };

    let point_valid_dist = |point: Vector2<T>, query_stack: &mut Vec<usize>| -> bool {
        point_valid_for_offset(
            original_polyline,
            offset,
            orig_polyline_index,
            point,
            query_stack,
            pos_equal_eps,
            offset_dist_eps,
        )
    };

    let slice_is_valid = |slice: &PlineViewData<T>, query_stack: &mut Vec<usize>| -> bool {
        if slice_contains_invalid_segment(
            slice,
            true,
            raw_offset_polyline.segment_count(),
            invalid_segment_indexes,
        ) {
            return false;
        }

        if slice.end_index_offset == 0 {
            // slice all on one segment, test start, end, midpoint, and if it intersects the
            // original
            let v1 = slice.updated_start;
            if !point_valid_dist(v1.pos(), query_stack) {
                return false;
            }
            let v2 = PlineVertex::from_vector2(slice.end_point, T::zero());
            if !point_valid_dist(v2.pos(), query_stack) {
                return false;
            }
            let midpoint = seg_midpoint(v1, v2);
            if !point_valid_dist(midpoint, query_stack) {
                return false;
            }

            return !intersects_original_pline(v1, v2, query_stack);
        }

        // slice not all on one segment, start by checking midpoints of first and last segment of
        // the slice
        let start_seg_midpoint = seg_midpoint(
            slice.updated_start,
            raw_offset_polyline.at(raw_offset_polyline.next_wrapping_index(slice.start_index)),
        );

        if !point_valid_dist(start_seg_midpoint, query_stack) {
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

        if !point_valid_dist(end_seg_midpoint, query_stack) {
            return false;
        }

        // test all segments
        for (v1, v2) in slice.view(raw_offset_polyline).iter_segments() {
            // test start point
            if !point_valid_dist(v1.pos(), query_stack) {
                return false;
            }

            // test intersection with original polyline
            if intersects_original_pline(v1, v2, query_stack) {
                return false;
            }
        }
        // check final end point (loop checks only start point and intersection)
        point_valid_dist(slice.end_point, query_stack)
    };

    for (&start_index, intr_list) in &intersects_lookup {
        for intrs in intr_list.windows(2) {
            let slice = PlineViewData::from_slice_points(
                raw_offset_polyline,
                intrs[0],
                start_index,
                intrs[1],
                start_index,
                pos_equal_eps,
            );

            if let Some(s) = slice
                && slice_is_valid(&s, &mut query_stack)
            {
                result.push(s);
            }
        }

        // build the slice between the last intersect in the intr_list and the next intersect found
        let next_index = raw_offset_polyline.next_wrapping_index(start_index);

        let (found_index, next_intr_list) =
            if let Some(list) = intersects_lookup.range(next_index..).next() {
                list
            } else {
                // wrap around polyline
                intersects_lookup.range(..=start_index).next().unwrap()
            };

        let slice = PlineViewData::from_slice_points(
            raw_offset_polyline,
            *intr_list.last().unwrap(),
            start_index,
            next_intr_list[0],
            *found_index,
            pos_equal_eps,
        );

        if let Some(s) = slice
            && slice_is_valid(&s, &mut query_stack)
        {
            result.push(s);
        }
    }

    result
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

/// Visits circle intersections after resolving each spatial-index item with `filter`.
///
/// `filter` returns the corresponding polyline segment index or `None` to reject the item before
/// the detailed line-circle or circle-circle test.
fn visit_circle_intersects<P, T, F, V>(
    pline: &P,
    query: &CircleIntersectQuery<'_, T>,
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

    let query_results = aabb_index.query(
        circle_center.x - circle_radius,
        circle_center.y - circle_radius,
        circle_center.x + circle_radius,
        circle_center.y + circle_radius,
    );

    for item in query_results {
        let Some(start_index) = filter(item) else {
            continue;
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
    }
}

pub fn slices_from_dual_raw_offsets<P, R, T>(
    original_polyline: &P,
    raw_offset: &RawOffsetResult<R>,
    dual_raw_offset: &RawOffsetResult<R>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> Vec<PlineViewData<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T>,
    T: Real,
{
    let raw_offset_polyline = &raw_offset.polyline;
    let invalid_segments = &raw_offset.invalid_segments;
    let dual_raw_offset_polyline = &dual_raw_offset.polyline;
    let dual_invalid_segments = &dual_raw_offset.invalid_segments;

    let mut result = Vec::new();
    if raw_offset_polyline.vertex_count() < 2 {
        return result;
    }

    let pos_equal_eps = options.pos_equal_eps;
    let offset_dist_eps = options.offset_dist_eps;

    let invalid_segment_indexes = &raw_offset.invalid_segment_indexes;
    let has_invalid = !invalid_segment_indexes.is_empty();
    let dual_has_invalid = !dual_raw_offset.invalid_segment_indexes.is_empty();
    if invalid_segment_indexes.len() == invalid_segments.len() {
        return result;
    }
    let (raw_offset_index, raw_index_to_segment);
    let self_intrs = if has_invalid {
        (raw_offset_index, raw_index_to_segment) = create_segment_index(
            raw_offset_polyline,
            invalid_segments,
            invalid_segment_indexes.len(),
        );
        all_self_intersects_as_basic_filtered(
            raw_offset_polyline,
            &raw_offset_index,
            |item| {
                let index = match item {
                    PlineIntersectFilterItem::LocalSegment(index) => index,
                    PlineIntersectFilterItem::GlobalAabbItem(item) => raw_index_to_segment
                        .as_deref()
                        .map_or(item, |mapping| mapping[item]),
                };
                (!invalid_segments[index]).then_some(index)
            },
            false,
            pos_equal_eps,
        )
    } else {
        raw_offset_index = raw_offset_polyline.create_approx_aabb_index();
        raw_index_to_segment = None;
        all_self_intersects_as_basic(raw_offset_polyline, &raw_offset_index, false, pos_equal_eps)
    };

    let dual_intrs = if has_invalid || dual_has_invalid {
        find_intersects_filtered(
            raw_offset_polyline,
            dual_raw_offset_polyline,
            &raw_offset_index,
            |item| match item {
                TwoPlinesIntersectFilterItem::Pline1AabbItem(item) => {
                    let index = raw_index_to_segment
                        .as_deref()
                        .map_or(item, |mapping| mapping[item]);
                    (!invalid_segments[index]).then_some(index)
                }
                TwoPlinesIntersectFilterItem::Pline2Segment(index) => {
                    (!dual_invalid_segments[index]).then_some(index)
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

    // using BTreeMap rather than  HashMap since we want to construct the slices in vertex index
    // order and we do so by looping through all intersects (required later when slices are stitched
    // together, because slices may not all form closed loops/polylines so must go in order of
    // indexes to ensure longest stitched results are formed)
    let mut intersects_lookup = BTreeMap::<usize, Vec<Vector2<T>>>::new();

    // Invalid requested spans cannot own geometric slice boundaries.
    let mut add_intr = |start_index: usize, intr: Vector2<T>| {
        intersects_lookup.entry(start_index).or_default().push(intr);
    };

    if !original_polyline.is_closed() {
        // add intersects between circles generated at original open polyline end points and raw
        // offset polyline
        let circle_radius = offset.abs();
        let include_segment = |item: usize| {
            let index = raw_index_to_segment
                .as_deref()
                .map_or(item, |mapping| mapping[item]);
            (!invalid_segments[index]).then_some(index)
        };
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
            if has_invalid {
                visit_circle_intersects(
                    raw_offset_polyline,
                    &query,
                    &include_segment,
                    &mut add_intr,
                );
            } else {
                visit_circle_intersects(
                    raw_offset_polyline,
                    &query,
                    &Some,
                    &mut add_intr,
                );
            }
        }
    }

    // Add all self intersects.
    for &si in &self_intrs {
        add_intr(si.start_index1, si.point);
        add_intr(si.start_index2, si.point);
    }

    // Only add intersects using start_index1 (corresponds to the the raw offset polyline).
    for &intr in &dual_intrs.basic_intersects {
        debug_assert!(intr.start_index1 < raw_offset_polyline.segment_count());
        debug_assert!(intr.start_index2 < dual_raw_offset_polyline.segment_count());
        add_intr(intr.start_index1, intr.point);
    }
    // Note not adding any overlapping intersects (they can only arise due to collapsing regions)

    let mut query_stack = Vec::with_capacity(8);

    if intersects_lookup.is_empty() {
        if has_invalid {
            return result;
        }

        let slice = PlineViewData::from_entire_pline(raw_offset_polyline);
        // test a point on raw offset polyline
        if !point_valid_for_offset(
            original_polyline,
            offset,
            orig_polyline_index,
            raw_offset_polyline.at(0).pos(),
            &mut query_stack,
            pos_equal_eps,
            offset_dist_eps,
        ) {
            return result;
        }

        // is valid
        result.push(slice);
        return result;
    }

    // sort intersects by distance from segment start vertex
    for (&i, intr_list) in &mut intersects_lookup {
        let start_pos = raw_offset_polyline.at(i).pos();
        intr_list.sort_unstable_by(|&si1, &si2| {
            let dist1 = dist_squared(si1, start_pos);
            let dist2 = dist_squared(si2, start_pos);
            dist1.total_cmp(&dist2)
        });
    }

    let intersects_original_pline =
        |v1: PlineVertex<T>, v2: PlineVertex<T>, query_stack: &mut Vec<usize>| -> bool {
            let approx_bb = seg_fast_approx_bounding_box(v1, v2);
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
                    Control::Break(())
                } else {
                    Control::Continue
                }
            };

            let fuzz = T::fuzzy_epsilon();
            orig_polyline_index.visit_query_with_stack(
                approx_bb.min_x - fuzz,
                approx_bb.min_y - fuzz,
                approx_bb.max_x + fuzz,
                approx_bb.max_y + fuzz,
                &mut visitor,
                query_stack,
            );
            has_intersect
        };

    let point_valid_dist = |point: Vector2<T>, query_stack: &mut Vec<usize>| -> bool {
        point_valid_for_offset(
            original_polyline,
            offset,
            orig_polyline_index,
            point,
            query_stack,
            pos_equal_eps,
            offset_dist_eps,
        )
    };

    let slice_is_valid = |slice: &PlineViewData<T>, query_stack: &mut Vec<usize>| -> bool {
        if slice_contains_invalid_segment(
            slice,
            raw_offset_polyline.is_closed(),
            raw_offset_polyline.segment_count(),
            invalid_segment_indexes,
        ) {
            return false;
        }

        if slice.end_index_offset == 0 {
            // slice all on one segment, test start, end, midpoint, and if it intersects the
            // original
            let v1 = slice.updated_start;
            if !point_valid_dist(v1.pos(), query_stack) {
                return false;
            }
            let v2 = PlineVertex::from_vector2(slice.end_point, T::zero());
            if !point_valid_dist(v2.pos(), query_stack) {
                return false;
            }
            let midpoint = seg_midpoint(v1, v2);
            if !point_valid_dist(midpoint, query_stack) {
                return false;
            }

            return !intersects_original_pline(v1, v2, query_stack);
        }

        // slice not all on one segment, start by checking midpoints of first and last segment of
        // the slice
        let start_seg_midpoint = seg_midpoint(
            slice.updated_start,
            raw_offset_polyline.at(raw_offset_polyline.next_wrapping_index(slice.start_index)),
        );

        if !point_valid_dist(start_seg_midpoint, query_stack) {
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

        if !point_valid_dist(end_seg_midpoint, query_stack) {
            return false;
        }

        // test all segments
        for (v1, v2) in slice.view(raw_offset_polyline).iter_segments() {
            // test start point
            if !point_valid_dist(v1.pos(), query_stack) {
                return false;
            }

            // test intersection with original polyline
            if intersects_original_pline(v1, v2, query_stack) {
                return false;
            }
        }
        // check final end point (loop checks only start point and intersection)
        point_valid_dist(slice.end_point, query_stack)
    };

    if !original_polyline.is_closed() {
        // build first slice that ends at the first intersect since we will not wrap back to
        // capture it as in the case of a closed polyline
        let (intr_idx, intr_list) = intersects_lookup.iter().next().unwrap();
        let intr = intr_list[0];
        let slice = PlineViewData::from_slice_points(
            raw_offset_polyline,
            raw_offset_polyline.at(0).pos(),
            0,
            intr,
            *intr_idx,
            pos_equal_eps,
        );

        if let Some(s) = slice
            && slice_is_valid(&s, &mut query_stack)
        {
            result.push(s);
        }
    }

    for (&start_index, intr_list) in &intersects_lookup {
        for intrs in intr_list.windows(2) {
            let slice = PlineViewData::from_slice_points(
                raw_offset_polyline,
                intrs[0],
                start_index,
                intrs[1],
                start_index,
                pos_equal_eps,
            );

            if let Some(s) = slice
                && slice_is_valid(&s, &mut query_stack)
            {
                result.push(s);
            }
        }

        // build the slice between the last intersect in the intr_list and the next intersect found
        let next_index = raw_offset_polyline.next_wrapping_index(start_index);

        let (found_index, next_intr_list) =
            if let Some(list) = intersects_lookup.range(next_index..).next() {
                list
            } else if original_polyline.is_closed() {
                // wrap around polyline
                intersects_lookup.range(..=start_index).next().unwrap()
            } else {
                // open polyline and didn't find next intersect, we're done
                let slice = PlineViewData::from_slice_points(
                    raw_offset_polyline,
                    *intr_list.last().unwrap(),
                    start_index,
                    raw_offset_polyline.last().unwrap().pos(),
                    raw_offset_polyline.vertex_count() - 1,
                    pos_equal_eps,
                );
                if let Some(s) = slice
                    && slice_is_valid(&s, &mut query_stack)
                {
                    result.push(s);
                }
                return result;
            };

        let slice = PlineViewData::from_slice_points(
            raw_offset_polyline,
            *intr_list.last().unwrap(),
            start_index,
            next_intr_list[0],
            *found_index,
            pos_equal_eps,
        );

        if let Some(s) = slice
            && slice_is_valid(&s, &mut query_stack)
        {
            result.push(s);
        }
    }

    result
}

pub fn stitch_slices_together<P, T, O>(
    raw_offset_pline: &P,
    slices: &[PlineViewData<T>],
    is_closed: bool,
    orig_max_index: usize,
    options: &PlineOffsetOptions<T>,
) -> Vec<O>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    let mut result = Vec::new();
    if slices.is_empty() {
        return result;
    }

    let join_eps = options.slice_join_eps;
    let pos_equal_eps = options.pos_equal_eps;

    if slices.len() == 1 {
        // Use join_eps for removing repeat vertices to be consistent with how slice connections
        // are detected (prevents tiny segments at slice boundaries)
        let mut pline = O::create_from_remove_repeat(&slices[0].view(raw_offset_pline), join_eps);

        if is_closed
            && pline
                .at(0)
                .pos()
                .fuzzy_eq_eps(pline.last().unwrap().pos(), join_eps)
        {
            pline.set_is_closed(true);
            pline.remove_last();
        }

        result.push(pline);

        return result;
    }

    let aabb_index = {
        let mut builder = StaticAABB2DIndexBuilder::new(slices.len());
        for slice in slices {
            let start_point = slice.updated_start.pos();
            builder.add(
                start_point.x - join_eps,
                start_point.y - join_eps,
                start_point.x + join_eps,
                start_point.y + join_eps,
            );
        }
        builder.build().unwrap()
    };

    let mut visited_indexes = vec![false; slices.len()];
    let mut query_results = Vec::new();
    let mut query_stack = Vec::with_capacity(8);

    for i in 0..slices.len() {
        if visited_indexes[i] {
            continue;
        }

        visited_indexes[i] = true;

        let mut current_pline = O::empty();
        let mut current_index = i;
        let initial_start_point = slices[i].updated_start.pos();
        let mut loop_count = 0;
        let max_loop_count = slices.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                unreachable!("loop_count exceeded max_loop_count while stitching slices together");
            }
            loop_count += 1;

            // append current slice to current pline
            // Use join_eps for removing repeat vertices to be consistent with how slice connections
            // are detected (prevents tiny segments at slice boundaries)
            let current_slice = &slices[current_index];

            current_pline.extend_remove_repeat(&current_slice.view(raw_offset_pline), join_eps);

            let current_loop_start_index = current_slice.start_index;
            let current_end_point = current_slice.end_point;

            query_results.clear();
            let mut aabb_index_visitor = |i: usize| {
                if !visited_indexes[i] {
                    query_results.push(i);
                }
            };
            aabb_index.visit_query_with_stack(
                current_end_point.x - join_eps,
                current_end_point.y - join_eps,
                current_end_point.x + join_eps,
                current_end_point.y + join_eps,
                &mut aabb_index_visitor,
                &mut query_stack,
            );

            let get_index_dist = |i: usize| -> usize {
                let slice = &slices[i];
                if current_loop_start_index <= slice.start_index {
                    slice.start_index - current_loop_start_index
                } else {
                    // forward wrapping distance (distance to end + distance to index)
                    orig_max_index - current_loop_start_index + slice.start_index
                }
            };

            let end_connects_to_start = |i: usize| -> bool {
                let end_point = slices[i].end_point;
                end_point.fuzzy_eq_eps(initial_start_point, pos_equal_eps)
            };

            let Some(next_index) = query_results.iter().copied().min_by(|a, b| {
                // Choose by index distance, then by the end of the slice connecting to the initial
                // start. This ordering ensures overlapping slices are retained in stitching.
                get_index_dist(*a)
                    .cmp(&get_index_dist(*b))
                    .then_with(|| end_connects_to_start(*a).cmp(&end_connects_to_start(*b)))
            }) else {
                // done stitching current polyline
                if current_pline.vertex_count() > 1 {
                    let current_pline_sp = current_pline.at(0).pos();
                    let current_pline_ep = current_pline.last().unwrap().pos();
                    // Use join_eps for consistency with slice connection detection
                    if is_closed && current_pline_sp.fuzzy_eq_eps(current_pline_ep, join_eps) {
                        current_pline.remove_last();
                        current_pline.set_is_closed(true);
                    }

                    result.push(current_pline);
                }
                break;
            };

            // else continue stitching
            visited_indexes[next_index] = true;
            current_pline.remove_last();
            current_index = next_index;
        }
    }

    result
}

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
        stitch_slices_together(
            &raw_offset.polyline,
            &slices,
            true,
            raw_offset.polyline.vertex_count() - 1,
            options,
        )
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

        stitch_slices_together(
            &raw_offset.polyline,
            &slices,
            polyline.is_closed(),
            raw_offset.polyline.vertex_count(),
            options,
        )
    }
}

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
}
