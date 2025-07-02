use crate::{
    core::{
        Control,
        math::{Vector2, dist_squared},
        traits::{ControlFlow, Real},
    },
    polyline::{
        FindIntersectsOptions, PlineBasicIntersect, PlineIntersectVisitor,
        PlineIntersectsCollection, PlineOverlappingIntersect, PlineSegIntr, PlineSource, PlineView,
        PlineViewData, pline_seg_intr, seg_fast_approx_bounding_box, seg_split_at_point,
        seg_tangent_vector,
    },
};
use static_aabb2d_index as aabb_index;
use static_aabb2d_index::StaticAABB2DIndex;
use std::collections::HashSet;

/// Visits all local self intersects of the polyline. Local self intersects are defined as between
/// two polyline segments that share a vertex.
pub fn visit_local_self_intersects<P, T, C, V>(polyline: &P, visitor: &mut V, pos_equal_eps: T) -> C
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    C: ControlFlow,
    V: PlineIntersectVisitor<T, C>,
{
    let vc = polyline.vertex_count();
    if vc < 2 {
        return C::continuing();
    }

    if vc == 2 {
        if polyline.is_closed() {
            // check if entirely overlaps self
            if polyline.at(0).bulge.fuzzy_eq(-polyline.at(1).bulge) {
                // overlapping
                return visitor.visit_overlapping_intr(PlineOverlappingIntersect::new(
                    0,
                    1,
                    polyline.at(0).pos(),
                    polyline.at(1).pos(),
                ));
            }
        }
        return C::continuing();
    }

    let mut visit_indexes = |i: usize, j: usize, k: usize| {
        let v1 = polyline.at(i);
        let v2 = polyline.at(j);
        let v3 = polyline.at(k);

        // testing for intersection between v1->v2 and v2->v3 segments
        if v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
            // singularity
            try_cf!(
                visitor.visit_overlapping_intr(PlineOverlappingIntersect::new(
                    i,
                    j,
                    v1.pos(),
                    v2.pos()
                ))
            );
        } else {
            match pline_seg_intr(v1, v2, v2, v3, pos_equal_eps) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !point.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        try_cf!(visitor.visit_basic_intr(PlineBasicIntersect::new(i, j, point)));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !point1.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        try_cf!(visitor.visit_basic_intr(PlineBasicIntersect::new(i, j, point1)));
                    }

                    if !point2.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        pline_seg_intr(v1, v2, v2, v3, pos_equal_eps);
                        try_cf!(visitor.visit_basic_intr(PlineBasicIntersect::new(i, j, point2)));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    try_cf!(
                        visitor.visit_overlapping_intr(PlineOverlappingIntersect::new(
                            i, j, point1, point2
                        ))
                    );
                }
            }
        }

        C::continuing()
    };

    for i in 2..vc {
        try_cf!(visit_indexes(i - 2, i - 1, i));
    }

    if polyline.is_closed() {
        // we tested for intersect between segments at indexes 0->1, 1->2 and everything up to and
        // including (count-3)->(count-2), (count-2)->(count-1), polyline is closed so now test
        // [(count-2)->(count-1), (count-1)->0] and [(count-1)->0, 0->1]
        try_cf!(visit_indexes(vc - 2, vc - 1, 0));
        try_cf!(visit_indexes(vc - 1, 0, 1));
    }
    C::continuing()
}

/// Visits all global self intersects of the polyline. Global self intersects are defined as between
/// two polyline segments that do not share a vertex.
///
/// In the case of two intersects on one segment the intersects will be added as two
/// [PlineBasicIntersect] in the order of distance from the start of the second segment.
///
/// In the case of an intersect at the very start of a polyline segment the vertex index of the
/// start of that segment is recorded (unless the polyline is open and the intersect is at the very
/// end of the polyline, then the second to last vertex index is used to maintain that it represents
/// the start of a polyline segment).
pub fn visit_global_self_intersects<P, T, C, V>(
    polyline: &P,
    aabb_index: &StaticAABB2DIndex<T>,
    visitor: &mut V,
    pos_equal_eps: T,
) -> C
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    C: ControlFlow,
    V: PlineIntersectVisitor<T, C>,
{
    let vc = polyline.vertex_count();

    if vc < 3 {
        return C::continuing();
    }

    let mut visited_pairs = HashSet::with_capacity(vc);
    let mut query_stack = Vec::with_capacity(8);

    // iterate all segment bounding boxes in the spatial index querying itself to test for self
    // intersects
    let mut cf = C::continuing();
    for (&i, aabb) in aabb_index
        .item_indices()
        .iter()
        .zip(aabb_index.item_boxes().iter())
    {
        let j = polyline.next_wrapping_index(i);
        let v1 = polyline.at(i);
        let v2 = polyline.at(j);
        let mut query_visitor = |hit_i: usize| {
            let hit_j = polyline.next_wrapping_index(hit_i);
            // skip local segments
            if i == hit_i || i == hit_j || j == hit_i || j == hit_j {
                return aabb_index::Control::Continue;
            }

            // skip already visited pairs (reverse index pair order for lookup to work, e.g. we
            // visit (1, 2) then (2, 1) and we only want to visit the segment pair once)
            if visited_pairs.contains(&(hit_i, i)) {
                return aabb_index::Control::Continue;
            }

            // add pair being visited
            visited_pairs.insert((i, hit_i));

            let u1 = polyline.at(hit_i);
            let u2 = polyline.at(hit_j);
            let skip_intr_at_end = |intr: Vector2<T>| -> bool {
                // skip intersect if it is at end point of either pline segment since it will be
                // found again by another segment with the intersect at its start point (this is
                // true even for an open polyline since we're finding self intersects)
                v2.pos().fuzzy_eq_eps(intr, pos_equal_eps)
                    && u2.pos().fuzzy_eq_eps(intr, pos_equal_eps)
            };

            match pline_seg_intr(v1, v2, u1, u2, pos_equal_eps) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !skip_intr_at_end(point) {
                        cf = visitor.visit_basic_intr(PlineBasicIntersect::new(i, hit_i, point));
                        if cf.should_break() {
                            return aabb_index::Control::Break(());
                        }
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !skip_intr_at_end(point1) {
                        cf = visitor.visit_basic_intr(PlineBasicIntersect::new(i, hit_i, point1));
                        if cf.should_break() {
                            return aabb_index::Control::Break(());
                        }
                    }

                    if !skip_intr_at_end(point2) {
                        cf = visitor.visit_basic_intr(PlineBasicIntersect::new(i, hit_i, point2));
                        if cf.should_break() {
                            return aabb_index::Control::Break(());
                        }
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    if !skip_intr_at_end(point1) {
                        cf = visitor.visit_overlapping_intr(PlineOverlappingIntersect::new(
                            i, hit_i, point1, point2,
                        ));
                        if cf.should_break() {
                            return aabb_index::Control::Break(());
                        }
                    }
                }
            };

            aabb_index::Control::Continue
        };

        aabb_index.visit_query_with_stack(
            aabb.min_x - pos_equal_eps,
            aabb.min_y - pos_equal_eps,
            aabb.max_x + pos_equal_eps,
            aabb.max_y + pos_equal_eps,
            &mut query_visitor,
            &mut query_stack,
        );

        if cf.should_break() {
            break;
        }
    }

    cf
}

/// Find all self intersects of a polyline. If `include_overlapping` is `true` then overlapping
/// intersects are returned as two basic intersects, one at each end of the overlap. If
/// `include_overlapping` is `false` then overlapping intersects are not returned.
pub fn all_self_intersects_as_basic<P, T>(
    polyline: &P,
    aabb_index: &StaticAABB2DIndex<T>,
    include_overlapping: bool,
    pos_equal_eps: T,
) -> Vec<PlineBasicIntersect<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    struct Visitor<U> {
        intrs: Vec<PlineBasicIntersect<U>>,
        include_overlapping: bool,
    }

    impl<U> PlineIntersectVisitor<U, Control> for Visitor<U>
    where
        U: Real,
    {
        fn visit_basic_intr(&mut self, intr: PlineBasicIntersect<U>) -> Control {
            self.intrs.push(intr);
            ControlFlow::continuing()
        }

        fn visit_overlapping_intr(&mut self, intr: PlineOverlappingIntersect<U>) -> Control {
            if self.include_overlapping {
                self.intrs.push(PlineBasicIntersect::new(
                    intr.start_index1,
                    intr.start_index2,
                    intr.point1,
                ));

                self.intrs.push(PlineBasicIntersect::new(
                    intr.start_index1,
                    intr.start_index2,
                    intr.point2,
                ));
            }

            ControlFlow::continuing()
        }
    }

    let mut visitor = Visitor {
        intrs: Vec::new(),
        include_overlapping,
    };

    visit_local_self_intersects(polyline, &mut visitor, pos_equal_eps);
    visit_global_self_intersects(polyline, aabb_index, &mut visitor, pos_equal_eps);

    visitor.intrs
}

/// Find all intersects between two polylines.
///
/// In the case of overlapping intersects `point1` is always closest to the start of the second
/// segment (`start_index2`) and `point2` furthest from the start of the second segment.
///
/// In the case of two intersects on one segment the intersects will be added as two
/// [PlineBasicIntersect] in the order of distance from the start of the second segment.
///
/// In the case of an intersect at the very start of a polyline segment the vertex index of the
/// start of that segment is recorded (unless the polyline is open and the intersect is at the very
/// end of the polyline, then the second to last vertex index is used to maintain that it represents
/// the start of a polyline segment).
pub fn find_intersects<P, O, T>(
    pline1: &P,
    pline2: &O,
    options: &FindIntersectsOptions<T>,
) -> PlineIntersectsCollection<T>
where
    P: PlineSource<Num = T> + ?Sized,
    O: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut result = PlineIntersectsCollection::new_empty();
    if pline1.vertex_count() < 2 || pline2.vertex_count() < 2 {
        return result;
    }

    // extract option parameters
    let pos_equal_eps = options.pos_equal_eps;
    let constructed_index1;
    let pline1_aabb_index = if let Some(x) = options.pline1_aabb_index {
        x
    } else {
        constructed_index1 = pline1.create_approx_aabb_index();
        &constructed_index1
    };

    // hash sets used to keep track of possible duplicate intersects being recorded due to
    // overlapping segments
    let mut possible_duplicates1 = HashSet::<usize>::new();
    let mut possible_duplicates2 = HashSet::<usize>::new();

    // last polyline segment starting indexes for open polylines (used to check when skipping
    // intersects at end points of polyline segments)
    let open1_last_idx = pline1.vertex_count() - 2;
    let open2_last_idx = pline2.vertex_count() - 2;

    for (i2, j2) in pline2.iter_segment_indexes() {
        let p2v1 = pline2.at(i2);
        let p2v2 = pline2.at(j2);
        let mut query_visitor = |i1: usize| {
            let j1 = pline1.next_wrapping_index(i1);
            let p1v1 = pline1.at(i1);
            let p1v2 = pline1.at(j1);

            let skip_intr_at_end = |intr: Vector2<T>| -> bool {
                // skip intersect at end point of pline segment since it will be found again by the
                // segment with it as its start point (unless the polyline is open and we're looking
                // at the very end point of the polyline, then include the intersect)
                (p1v2.pos().fuzzy_eq_eps(intr, pos_equal_eps)
                    && (pline1.is_closed() || i1 != open1_last_idx))
                    || (p2v2.pos().fuzzy_eq_eps(intr, pos_equal_eps)
                        && (pline2.is_closed() || i2 != open2_last_idx))
            };

            match pline_seg_intr(p1v1, p1v2, p2v1, p2v2, pos_equal_eps) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !skip_intr_at_end(point) {
                        result
                            .basic_intersects
                            .push(PlineBasicIntersect::new(i1, i2, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !skip_intr_at_end(point1) {
                        result
                            .basic_intersects
                            .push(PlineBasicIntersect::new(i1, i2, point1));
                    }
                    if !skip_intr_at_end(point2) {
                        result
                            .basic_intersects
                            .push(PlineBasicIntersect::new(i1, i2, point2));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    result
                        .overlapping_intersects
                        .push(PlineOverlappingIntersect::new(i1, i2, point1, point2));

                    if p1v2.pos().fuzzy_eq_eps(point1, pos_equal_eps)
                        || p1v2.pos().fuzzy_eq_eps(point2, pos_equal_eps)
                    {
                        possible_duplicates1.insert(pline1.next_wrapping_index(i1));
                    }
                    if p2v2.pos().fuzzy_eq_eps(point1, pos_equal_eps)
                        || p2v2.pos().fuzzy_eq_eps(point2, pos_equal_eps)
                    {
                        possible_duplicates2.insert(pline2.next_wrapping_index(i2));
                    }
                }
            }
        };

        let bb = seg_fast_approx_bounding_box(p2v1, p2v2);

        let mut query_stack = Vec::with_capacity(8);
        pline1_aabb_index.visit_query_with_stack(
            bb.min_x - pos_equal_eps,
            bb.min_y - pos_equal_eps,
            bb.max_x + pos_equal_eps,
            bb.max_y + pos_equal_eps,
            &mut query_visitor,
            &mut query_stack,
        );
    }

    if possible_duplicates1.is_empty() && possible_duplicates2.is_empty() {
        return result;
    }

    // remove any duplicate points caused by end point intersects + overlapping
    let mut final_basic_intrs = Vec::with_capacity(result.basic_intersects.len());

    for intr in result.basic_intersects.iter() {
        if possible_duplicates1.contains(&intr.start_index1) {
            let start_pt1 = pline1.at(intr.start_index1).pos();
            if intr.point.fuzzy_eq_eps(start_pt1, pos_equal_eps) {
                // skip including the intersect
                continue;
            }
        }

        if possible_duplicates2.contains(&intr.start_index2) {
            let start_pt2 = pline2.at(intr.start_index2).pos();
            if intr.point.fuzzy_eq_eps(start_pt2, pos_equal_eps) {
                // skip including the intersect
                continue;
            }
        }

        final_basic_intrs.push(*intr);
    }

    result.basic_intersects = final_basic_intrs;
    result
}

/// Find if two polylines have any intersections.
///
/// Any overlapping segments will be treated as an intersection and cause
/// scan_for_intersection() to return true.
pub fn scan_for_intersection<P, O, T>(
    pline1: &P,
    pline2: &O,
    options: &FindIntersectsOptions<T>,
) -> bool
where
    P: PlineSource<Num = T> + ?Sized,
    O: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut found_intersection = false;
    if pline1.vertex_count() < 2 || pline2.vertex_count() < 2 {
        return false;
    }

    // extract option parameters
    let pos_equal_eps = options.pos_equal_eps;
    let constructed_index1;
    let pline1_aabb_index = if let Some(x) = options.pline1_aabb_index {
        x
    } else {
        constructed_index1 = pline1.create_approx_aabb_index();
        &constructed_index1
    };

    for (i2, j2) in pline2.iter_segment_indexes() {
        let p2v1 = pline2.at(i2);
        let p2v2 = pline2.at(j2);
        let mut query_visitor = |i1: usize| {
            let j1 = pline1.next_wrapping_index(i1);
            let p1v1 = pline1.at(i1);
            let p1v2 = pline1.at(j1);

            match pline_seg_intr(p1v1, p1v2, p2v1, p2v2, pos_equal_eps) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point: _ }
                | PlineSegIntr::OneIntersect { point: _ } => {
                    found_intersection = true;
                    return aabb_index::Control::Break(());
                }
                PlineSegIntr::TwoIntersects {
                    point1: _,
                    point2: _,
                } => {
                    found_intersection = true;
                    return aabb_index::Control::Break(());
                }
                PlineSegIntr::OverlappingLines {
                    point1: _,
                    point2: _,
                }
                | PlineSegIntr::OverlappingArcs {
                    point1: _,
                    point2: _,
                } => {
                    found_intersection = true;
                    return aabb_index::Control::Break(());
                }
            }
            aabb_index::Control::Continue
        };

        let bb = seg_fast_approx_bounding_box(p2v1, p2v2);

        let mut query_stack = Vec::with_capacity(8);
        pline1_aabb_index.visit_query_with_stack(
            bb.min_x - pos_equal_eps,
            bb.min_y - pos_equal_eps,
            bb.max_x + pos_equal_eps,
            bb.max_y + pos_equal_eps,
            &mut query_visitor,
            &mut query_stack,
        );
    }
    found_intersection
}

/// Represents an open polyline slice where there was overlap between polylines across one or more
/// segments.
///
/// `source` polyline for `view_data` is always the second polyline.
#[derive(Debug, Copy, Clone)]
pub struct OverlappingSlice<T> {
    /// Start vertex indexes of the slice according to the original polylines that overlapped.
    pub start_indexes: (usize, usize),
    /// End vertex indexes of the slice according to the original polylines that overlapped.
    pub end_indexes: (usize, usize),
    /// View data for the slice, source is always the second polyline.
    pub view_data: PlineViewData<T>,
    /// If true then overlapping slice forms a closed loop on itself, otherwise it does not.
    pub is_loop: bool,
    /// If true then the overlapping slice was formed by segments that have opposing directions.
    pub opposing_directions: bool,
}

impl<T> OverlappingSlice<T>
where
    T: Real,
{
    pub fn new<P, R>(
        pline1: &P,
        pline2: &R,
        start_intr: &PlineOverlappingIntersect<T>,
        end_intr: Option<&PlineOverlappingIntersect<T>>,
        pos_equal_eps: T,
    ) -> Self
    where
        P: PlineSource<Num = T> + ?Sized,
        R: PlineSource<Num = T> + ?Sized,
    {
        let start_v1 = pline1.at(start_intr.start_index1);
        let start_v2 = pline1.at(pline1.next_wrapping_index(start_intr.start_index1));
        let start_u1 = pline2.at(start_intr.start_index2);
        let start_u2 = pline2.at(pline2.next_wrapping_index(start_intr.start_index2));
        let opposing_directions = {
            // tangent vectors are either going same direction or opposite direction, just test dot
            // product sign to determine if going same direction
            let t1 = seg_tangent_vector(start_v1, start_v2, start_intr.point1);
            let t2 = seg_tangent_vector(start_u1, start_u2, start_intr.point1);
            t1.dot(t2) < T::zero()
        };

        let start_indexes = (start_intr.start_index1, start_intr.start_index2);

        let create_updated_start = || {
            // create updated start by using point1 for position and determining bulge required
            // to form subsegment to point2
            let split1 = seg_split_at_point(start_u1, start_u2, start_intr.point1, pos_equal_eps);
            let split2 = seg_split_at_point(
                split1.split_vertex,
                start_u2,
                start_intr.point2,
                pos_equal_eps,
            );
            split2.updated_start
        };

        match end_intr {
            None => {
                // slice created from single overlapping intersect
                let updated_start = create_updated_start();
                let updated_end_bulge = updated_start.bulge;
                let end_point = start_intr.point2;
                let end_index_offset = 0;

                Self {
                    start_indexes,
                    end_indexes: start_indexes,
                    view_data: PlineViewData {
                        start_index: start_indexes.1,
                        end_index_offset,
                        updated_start,
                        updated_end_bulge,
                        end_point,
                        inverted_direction: false,
                    },
                    is_loop: false,
                    opposing_directions,
                }
            }
            Some(end_intr) => {
                // slice created from multiple intersects joined together end to start

                // check if end_intr forms closed loop back to start_intr
                if end_intr
                    .point2
                    .fuzzy_eq_eps(start_intr.point1, pos_equal_eps)
                {
                    // slice forms closed loop
                    Self {
                        start_indexes,
                        end_indexes: start_indexes,
                        view_data: PlineViewData {
                            start_index: start_indexes.1,
                            end_index_offset: pline2.vertex_count() - 1,
                            updated_start: start_u1,
                            updated_end_bulge: pline2.at(pline2.vertex_count() - 1).bulge,
                            end_point: end_intr.point2,
                            inverted_direction: false,
                        },
                        is_loop: true,
                        opposing_directions,
                    }
                } else {
                    // slice does not form closed loop
                    let end_point = end_intr.point2;
                    let end_indexes = (end_intr.start_index1, end_intr.start_index2);
                    let end_index_offset =
                        pline2.fwd_wrapping_dist(start_indexes.1, end_intr.start_index2);

                    // check if all on one pline2 segment or not
                    if start_intr.start_index2 == end_intr.start_index2 {
                        // slice is all on one pline2 segment
                        // updated_start positioned at start_intr.point1 and connects with end_point
                        // updated_end == updated_start
                        // end_point positioned at end_intr.point2
                        let updated_start = {
                            let split1 = seg_split_at_point(
                                start_u1,
                                start_u2,
                                start_intr.point1,
                                pos_equal_eps,
                            );
                            let split2 = seg_split_at_point(
                                split1.split_vertex,
                                start_u2,
                                end_intr.point2,
                                pos_equal_eps,
                            );
                            split2.updated_start
                        };

                        let updated_end_bulge = updated_start.bulge;

                        Self {
                            start_indexes,
                            end_indexes,
                            view_data: PlineViewData {
                                start_index: start_indexes.1,
                                end_index_offset,
                                updated_start,
                                updated_end_bulge,
                                end_point,
                                inverted_direction: false,
                            },
                            is_loop: false,
                            opposing_directions,
                        }
                    } else {
                        // slice is not on one pline2 segment
                        // updated_start positioned at start_intr.point1 and connects with start_u2
                        // updated_end positioned at end_intr.point1 and connects with end_intr.point2
                        // end_point positioned at end_intr.point2
                        let updated_start = {
                            let split1 = seg_split_at_point(
                                start_u1,
                                start_u2,
                                start_intr.point1,
                                pos_equal_eps,
                            );
                            split1.split_vertex
                        };

                        let updated_end = {
                            let end_u1 = pline2.at(end_intr.start_index2);
                            let end_u2 =
                                pline2.at(pline2.next_wrapping_index(end_intr.start_index2));

                            let split1 =
                                seg_split_at_point(end_u1, end_u2, end_intr.point1, pos_equal_eps);
                            let split2 = seg_split_at_point(
                                split1.split_vertex,
                                end_u2,
                                end_intr.point2,
                                pos_equal_eps,
                            );
                            split2.updated_start
                        };

                        Self {
                            start_indexes,
                            end_indexes,
                            view_data: PlineViewData {
                                start_index: start_indexes.1,
                                end_index_offset,
                                updated_start,
                                updated_end_bulge: updated_end.bulge,
                                end_point,
                                inverted_direction: false,
                            },
                            is_loop: false,
                            opposing_directions,
                        }
                    }
                }
            }
        }
    }

    #[inline]
    pub fn view<'a, P>(&self, source: &'a P) -> PlineView<'a, P>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        PlineView {
            source,
            data: self.view_data,
        }
    }
}

/// Sorts the overlapping `intersects` given according to `pline2` direction and vertex indexes
/// and returns all the overlapping `intersects` joined together into slices.
///
/// This function assumes the intersects given follow the convention that `point1` is closest to the
/// pline2's segment start and `point2` is furthest from the start of pline2's segment start.
pub fn sort_and_join_overlapping_intersects<P, R, T>(
    intersects: &mut [PlineOverlappingIntersect<T>],
    pline1: &P,
    pline2: &R,
    pos_equal_eps: T,
) -> Vec<OverlappingSlice<T>>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut result = Vec::new();

    if intersects.is_empty() {
        return result;
    }

    debug_assert!(
        intersects
            .iter()
            .all(|intr: &PlineOverlappingIntersect<T>| {
                let start = pline2.at(intr.start_index2).pos();
                let dist1 = dist_squared(start, intr.point1);
                let dist2 = dist_squared(start, intr.point2);
                dist1 <= dist2
            }),
        "intersect point1 and point2 expected to be sorted according to pline2 direction!"
    );

    // sort the intersects according to pline2 direction (points within the intersects
    // are already sorted with point1 closer to start of the pline2 segment than point2)
    intersects.sort_unstable_by(|intr_a, intr_b| {
        intr_a.start_index2.cmp(&intr_b.start_index2).then_with(|| {
            // equal start_index2 so sort by distance from start
            let start = pline2.at(intr_a.start_index2).pos();
            let dist1 = dist_squared(start, intr_a.point1);
            let dist2 = dist_squared(start, intr_b.point1);
            dist1.total_cmp(&dist2)
        })
    });

    let mut start_intr = &intersects[0];
    let mut end_intr = None;
    let mut current_end_point = start_intr.point2;

    // skip first intr (already processed by setting start_intr)
    for intr in intersects.iter().skip(1) {
        // check if intr start point connects with end_intr end point
        if !intr.point1.fuzzy_eq_eps(current_end_point, pos_equal_eps) {
            // intr does not join with previous intr, cap off slice and add to result
            let slice = OverlappingSlice::new(pline1, pline2, start_intr, end_intr, pos_equal_eps);
            result.push(slice);

            start_intr = intr;
            end_intr = None;
        } else {
            end_intr = Some(intr);
        }

        current_end_point = intr.point2;
    }

    // cap off final slice and add to result
    let slice = OverlappingSlice::new(pline1, pline2, start_intr, end_intr, pos_equal_eps);
    result.push(slice);

    if result.len() > 1 {
        // check if last overlapping slice connects with first
        let last_slice_end = result.last().unwrap().view_data.end_point;
        let first_slice_begin = result[0].view_data.updated_start.pos();
        if last_slice_end.fuzzy_eq_eps(first_slice_begin, pos_equal_eps) {
            // they do connect, join them together by updating the first slice and removing the last
            let last_slice = result.pop().unwrap();
            let first_slice = &mut result[0];
            first_slice.start_indexes = last_slice.start_indexes;
            first_slice.view_data.updated_start = last_slice.view_data.updated_start;
            first_slice.view_data.end_index_offset += last_slice.view_data.end_index_offset;

            if last_slice
                .view_data
                .end_point
                .fuzzy_eq_eps(pline2.at(0).pos(), pos_equal_eps)
            {
                // add one to offset to capture pline2[0] vertex (it is at point of connection)
                first_slice.view_data.end_index_offset += 1;
            }
        }
    }

    result
}

#[cfg(test)]
mod local_self_intersect_tests {
    use super::*;
    use crate::{
        core::math::bulge_from_angle,
        polyline::{PlineIntersect, PlineSourceMut, Polyline},
    };

    fn local_self_intersects<T>(
        polyline: &Polyline<T>,
        pos_equal_eps: T,
    ) -> PlineIntersectsCollection<T>
    where
        T: Real,
    {
        let mut intrs = Vec::new();
        let mut overlapping_intrs = Vec::new();
        let mut visitor = |intr: PlineIntersect<T>| match intr {
            PlineIntersect::Basic(b) => {
                intrs.push(b);
            }
            PlineIntersect::Overlapping(o) => {
                overlapping_intrs.push(o);
            }
        };

        visit_local_self_intersects(polyline, &mut visitor, pos_equal_eps);

        PlineIntersectsCollection::new(intrs, overlapping_intrs)
    }

    #[test]
    fn empty_polyline() {
        let pline = Polyline::<f64>::new();
        let intrs = local_self_intersects(&pline, 1e-5);

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn single_vertex() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, 1.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn circle_no_intersects() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn half_circle_overlapping_self() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, -1.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects[0].start_index1, 0);
        assert_eq!(intrs.overlapping_intersects[0].start_index2, 1);
        assert_fuzzy_eq!(intrs.overlapping_intersects[0].point1, pline[0].pos());
        assert_fuzzy_eq!(intrs.overlapping_intersects[0].point2, pline[1].pos());
    }

    #[test]
    fn short_open_polyline_circle() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        pline.add(0.0, 0.0, 0.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 1);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, pline[2].pos());
    }

    #[test]
    fn long_open_polyline_circle() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, -1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(2.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, 1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(0.0, 0.0, 0.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }
}

#[cfg(test)]
mod global_self_intersect_tests {
    use super::*;
    use crate::{
        core::math::bulge_from_angle,
        polyline::{PlineIntersect, PlineSourceMut, Polyline},
    };

    fn global_self_intersects<T>(
        polyline: &Polyline<T>,
        aabb_index: &StaticAABB2DIndex<T>,
    ) -> PlineIntersectsCollection<T>
    where
        T: Real,
    {
        let mut intrs = Vec::new();
        let mut overlapping_intrs = Vec::new();
        let mut visitor = |intr: PlineIntersect<T>| match intr {
            PlineIntersect::Basic(b) => {
                intrs.push(b);
            }
            PlineIntersect::Overlapping(o) => {
                overlapping_intrs.push(o);
            }
        };

        visit_global_self_intersects(polyline, aabb_index, &mut visitor, T::from(1e-5).unwrap());

        PlineIntersectsCollection::new(intrs, overlapping_intrs)
    }

    #[test]
    fn circle_no_intersects() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_aabb_index());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        let pline_as_lines = pline.arcs_to_approx_lines(1e-2).unwrap();
        let intrs =
            global_self_intersects(&pline_as_lines, &pline_as_lines.create_approx_aabb_index());

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn half_circle_overlapping_self() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, -1.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_aabb_index());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn short_open_polyline_circle() {
        // does self intersect at end but is local self intersect
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        pline.add(0.0, 0.0, 0.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_aabb_index());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        // self intersect at end point is returned since not local self intersect
        let pline_as_lines = pline.arcs_to_approx_lines(1e-2).unwrap();
        let intrs =
            global_self_intersects(&pline_as_lines, &pline_as_lines.create_approx_aabb_index());

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(
            intrs.basic_intersects[0].start_index2,
            pline_as_lines.vertex_count() - 2
        );

        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(0.0, 0.0));
    }

    #[test]
    fn long_open_polyline_circle() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, -1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(2.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, 1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(0.0, 0.0, 0.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_aabb_index());
        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 3);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, pline[4].pos(), 1e-5);
    }
}

#[cfg(test)]
mod find_intersects_tests {
    use crate::polyline::{PlineSourceMut, Polyline};

    use super::*;

    #[test]
    fn open_polylines_end_touch_start() {
        // two open polylines end point touching start point
        let mut pline1 = Polyline::new();
        pline1.add(0.0, 0.0, 0.0);
        pline1.add(1.0, 1.0, 0.0);

        let mut pline2 = Polyline::new();
        pline2.add(-1.0, -1.0, 0.0);
        pline2.add(0.0, 0.0, 0.0);

        let intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 0);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(0.0, 0.0));
    }

    #[test]
    fn open_polylines_end_touch_start_flipped() {
        let mut pline1 = Polyline::new();
        pline1.add(-1.0, -1.0, 0.0);
        pline1.add(0.0, 0.0, 0.0);

        let mut pline2 = Polyline::new();
        pline2.add(0.0, 0.0, 0.0);
        pline2.add(1.0, 1.0, 0.0);

        let intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 0);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(0.0, 0.0));
    }

    #[test]
    fn open_polylines_start_points_touch() {
        // two open polylines start point touching start point
        let mut pline1 = Polyline::new();
        pline1.add(0.0, 0.0, 0.0);
        pline1.add(1.0, 1.0, 0.0);

        let mut pline2 = Polyline::new();
        pline2.add(0.0, 0.0, 0.0);
        pline2.add(-1.0, -1.0, 0.0);

        let intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 0);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(0.0, 0.0));
    }

    #[test]
    fn circles_touching() {
        // two closed circles touching
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(1.0, 0.0, 1.0);
        pline2.add(2.0, 0.0, 1.0);

        let intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        let intr = intrs.basic_intersects[0];
        assert_eq!(intr.start_index1, 1);
        assert_eq!(intr.start_index2, 0);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(1.0, 0.0));
    }

    #[test]
    fn circles_overlapping_same_direction() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let pline2 = pline1.clone();

        let mut intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 2);

        // sort for retrieval for asserts
        intrs
            .overlapping_intersects
            .sort_unstable_by_key(|oi| oi.start_index1);

        let intr1 = intrs.overlapping_intersects[0];
        assert_eq!(intr1.start_index1, 0);
        assert_eq!(intr1.start_index2, 0);
        assert_fuzzy_eq!(intr1.point1, pline1[0].pos());
        assert_fuzzy_eq!(intr1.point2, pline1[1].pos());

        let intr2 = intrs.overlapping_intersects[1];
        assert_eq!(intr2.start_index1, 1);
        assert_eq!(intr2.start_index2, 1);
        assert_fuzzy_eq!(intr2.point1, pline1[1].pos());
        assert_fuzzy_eq!(intr2.point2, pline1[0].pos());
    }

    #[test]
    fn circles_overlapping_opposing_direction() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(0.0, 0.0, -1.0);
        pline2.add(1.0, 0.0, -1.0);

        let mut intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 2);

        // sort for retrieval for asserts
        intrs
            .overlapping_intersects
            .sort_unstable_by_key(|oi| oi.start_index2);

        let intr1 = intrs.overlapping_intersects[0];
        assert_eq!(intr1.start_index1, 1);
        assert_eq!(intr1.start_index2, 0);
        assert_fuzzy_eq!(intr1.point1, pline2[0].pos());
        assert_fuzzy_eq!(intr1.point2, pline2[1].pos());

        let intr2 = intrs.overlapping_intersects[1];
        assert_eq!(intr2.start_index1, 0);
        assert_eq!(intr2.start_index2, 1);
        assert_fuzzy_eq!(intr2.point1, pline2[1].pos());
        assert_fuzzy_eq!(intr2.point2, pline2[0].pos());
    }

    #[test]
    fn circles_overlapping_opposing_direction_flipped() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, -1.0);
        pline1.add(1.0, 0.0, -1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(0.0, 0.0, 1.0);
        pline2.add(1.0, 0.0, 1.0);

        let mut intrs = find_intersects(&pline1, &pline2, &Default::default());

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 2);

        // sort for retrieval for asserts
        intrs
            .overlapping_intersects
            .sort_unstable_by_key(|oi| oi.start_index2);

        let intr1 = intrs.overlapping_intersects[0];
        assert_eq!(intr1.start_index1, 1);
        assert_eq!(intr1.start_index2, 0);
        assert_fuzzy_eq!(intr1.point1, pline2[0].pos());
        assert_fuzzy_eq!(intr1.point2, pline2[1].pos());

        let intr2 = intrs.overlapping_intersects[1];
        assert_eq!(intr2.start_index1, 0);
        assert_eq!(intr2.start_index2, 1);
        assert_fuzzy_eq!(intr2.point1, pline2[1].pos());
        assert_fuzzy_eq!(intr2.point2, pline2[0].pos());
    }

    #[test]
    fn uses_pos_equal_eps() {
        // test that pos_equal_eps passed in options is used
        let eps = 1e-5;
        let mut pline1 = Polyline::new();
        pline1.add(0.5, 0.0, 0.0);
        pline1.add(0.5, 1.0 - 0.99 * eps, 0.0);

        let mut pline2 = Polyline::new();
        pline2.add(0.0, 1.0, 0.0);
        pline2.add(1.0, 1.0, 0.0);

        let opts = FindIntersectsOptions {
            pos_equal_eps: eps,
            ..Default::default()
        };

        let intrs = find_intersects(&pline1, &pline2, &opts);
        assert_eq!(intrs.basic_intersects.len(), 1);
        assert!(intrs.overlapping_intersects.is_empty());
        let intr = intrs.basic_intersects[0];
        assert_fuzzy_eq!(intr.point, Vector2::new(0.5, 1.0));
    }
}

#[cfg(test)]
mod sort_and_join_overlapping_intersects_tests {
    use super::*;
    use crate::core::math::bulge_from_angle;
    use crate::polyline::{PlineCreation, PlineSourceMut, PlineVertex, Polyline};

    #[test]
    fn overlapping_circles_same_dir() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(0.0, 0.0, 1.0);
        pline2.add(1.0, 0.0, 1.0);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 3);
        assert_fuzzy_eq!(slice_pline[0], pline2[0]);
        assert_fuzzy_eq!(slice_pline[1], pline2[1]);
        assert_fuzzy_eq!(slice_pline[2], pline2[0].with_bulge(0.0));

        assert_eq!(slices[0].start_indexes, (0, 0));
        assert_eq!(slices[0].end_indexes, (0, 0));
        assert!(!slices[0].opposing_directions);
    }

    #[test]
    fn overlapping_circles_same_dir_flipped_index() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(1.0, 0.0, 1.0);
        pline2.add(0.0, 0.0, 1.0);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 3);
        assert_fuzzy_eq!(slice_pline[0], pline2[0]);
        assert_fuzzy_eq!(slice_pline[1], pline2[1]);
        assert_fuzzy_eq!(slice_pline[2], pline2[0].with_bulge(0.0));

        assert_eq!(slices[0].start_indexes, (1, 0));
        assert_eq!(slices[0].end_indexes, (1, 0));
        assert!(!slices[0].opposing_directions);
    }

    #[test]
    fn overlapping_circles_opposing_dir() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(0.0, 0.0, -1.0);
        pline2.add(1.0, 0.0, -1.0);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 3);
        assert_fuzzy_eq!(slice_pline[0], pline2[0]);
        assert_fuzzy_eq!(slice_pline[1], pline2[1]);
        assert_fuzzy_eq!(slice_pline[2], pline2[0].with_bulge(0.0));

        assert_eq!(slices[0].start_indexes, (1, 0));
        assert_eq!(slices[0].end_indexes, (1, 0));
        assert!(slices[0].opposing_directions);
    }

    #[test]
    fn overlapping_circles_perpendicular_vertexes() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let mut pline2 = Polyline::new_closed();
        pline2.add(0.5, -0.5, 1.0);
        pline2.add(0.5, 0.5, 1.0);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 3);
        assert_fuzzy_eq!(slice_pline[0], pline2[0]);
        assert_fuzzy_eq!(slice_pline[1], pline2[1]);
        assert_fuzzy_eq!(slice_pline[2], pline2[0].with_bulge(0.0));

        assert_eq!(slices[0].start_indexes, (0, 0));
        assert_eq!(slices[0].end_indexes, (0, 0));
        assert!(!slices[0].opposing_directions);
    }

    #[test]
    fn overlapping_arcs() {
        // full circle composed of 10 vertexes
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);
        let radius = 1.0;

        let vertexes = (0..count)
            .map(|i| (i as f64) * sub_angle)
            .map(|angle| PlineVertex::new(radius * angle.cos(), radius * angle.sin(), bulge));

        let pline1 = Polyline::from_iter(vertexes, true);

        // half circle composed of two vertexes
        let mut pline2 = Polyline::new();
        pline2.add(-radius, 0.0, 1.0);
        pline2.add(radius, 0.0, 0.0);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 2);
        assert_fuzzy_eq!(slice_pline[0], pline2[0]);
        assert_fuzzy_eq!(slice_pline[1], pline2[1]);

        let data = &slices[0].view_data;
        assert_fuzzy_eq!(data.updated_start, PlineVertex::new(-radius, 0.0, 1.0));
        assert_fuzzy_eq!(data.updated_end_bulge, 1.0);
        assert_fuzzy_eq!(data.end_point, Vector2::new(radius, 0.0));
        assert_eq!(slices[0].start_indexes, (5, 0));
        assert_eq!(slices[0].end_indexes, (9, 0));
        assert!(!slices[0].opposing_directions);
    }
    #[test]
    fn overlapping_arcs_flipped() {
        let radius = 1.0;

        // half circle composed of two vertexes
        let mut pline1 = Polyline::new();
        pline1.add(-radius, 0.0, 1.0);
        pline1.add(radius, 0.0, 0.0);

        // full circle composed of 10 vertexes
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let vertexes = (0..count)
            .map(|i| (i as f64) * sub_angle)
            .map(|angle| PlineVertex::new(radius * angle.cos(), radius * angle.sin(), bulge));

        let pline2 = Polyline::from_iter(vertexes, true);

        let mut intersects = find_intersects(&pline1, &pline2, &Default::default());

        let slices = sort_and_join_overlapping_intersects(
            &mut intersects.overlapping_intersects,
            &pline1,
            &pline2,
            1e-5,
        );

        assert_eq!(slices.len(), 1);
        let slice_pline = Polyline::create_from(&slices[0].view(&pline2));
        assert_eq!(slice_pline.vertex_count(), 6);
        assert_fuzzy_eq!(slice_pline[0], pline2[5]);
        assert_fuzzy_eq!(slice_pline[1], pline2[6]);
        assert_fuzzy_eq!(slice_pline[2], pline2[7]);
        assert_fuzzy_eq!(slice_pline[3], pline2[8]);
        assert_fuzzy_eq!(slice_pline[4], pline2[9]);
        assert_fuzzy_eq!(slice_pline[5], pline2[0].with_bulge(0.0));

        let data = &slices[0].view_data;
        assert_fuzzy_eq!(data.updated_start, pline2[5]);
        assert_fuzzy_eq!(data.updated_end_bulge, pline2[9].bulge);
        assert_fuzzy_eq!(data.end_point, Vector2::new(radius, 0.0));
        assert_eq!(slices[0].start_indexes, (0, 5));
        assert_eq!(slices[0].end_indexes, (0, 9));
        assert!(!slices[0].opposing_directions);
    }
}
