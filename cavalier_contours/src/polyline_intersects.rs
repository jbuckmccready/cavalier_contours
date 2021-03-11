use std::collections::HashSet;

use static_aabb2d_index::StaticAABB2DIndex;

use crate::{
    core_math::seg_fast_approx_bounding_box, intersects::PlineSegIntr,
    pline_seg_intersect::pline_seg_intr, Polyline, Real, Vector2,
};

/// Represents a basic polyline intersect at a single point.
///
/// `start_index1` is the starting vertex index position of the first polyline segment involved in the intersect.
/// `start_index2` is the starting vertex index position of the second polyline segment involved in the intersect.
/// `point` is the point at which the intersect occurs.
#[derive(Debug, Clone, Copy)]
pub struct PlineBasicIntersect<T> {
    pub start_index1: usize,
    pub start_index2: usize,
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

/// Represents a overlapping polyline intersect segment.
///
/// `start_index1` is the starting vertex index position of the first polyline segment involved in the intersect.
/// `start_index2` is the starting vertex index position of the second polyline segment involved in the intersect.
/// `point1` and `point2` are the end cap points of the overlapping intersect segment.
#[derive(Debug, Clone, Copy)]
pub struct PlineOverlappingIntersect<T> {
    pub start_index1: usize,
    pub start_index2: usize,
    pub point1: Vector2<T>,
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

/// Represents a polyline intersect that may be either a [PlineBasicIntersect] or [PlineOverlappingIntersect].
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

/// Visits all local self intersects of the polyline. Local self intersects are defined as between
/// two polyline segments that share a vertex.
pub fn visit_local_self_intersects<T, F>(polyline: &Polyline<T>, visitor: &mut F, pos_equal_eps: T)
where
    T: Real,
    F: FnMut(PlineIntersect<T>) -> bool,
{
    let ln = polyline.len();
    if ln < 2 {
        return;
    }

    if ln == 2 {
        if polyline.is_closed() {
            // check if entirely overlaps self
            if polyline[0].bulge.fuzzy_eq(-polyline[1].bulge) {
                // overlapping
                visitor(PlineIntersect::new_overlapping(
                    0,
                    1,
                    polyline[0].pos(),
                    polyline[1].pos(),
                ));
            }
        }
        return;
    }

    let mut visit_indexes = |i: usize, j: usize, k: usize| {
        let v1 = polyline[i];
        let v2 = polyline[j];
        let v3 = polyline[k];

        let mut continue_visiting = true;

        // testing for intersection between v1->v2 and v2->v3 segments
        if v1.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
            // singularity
            continue_visiting = visitor(PlineIntersect::new_overlapping(i, j, v1.pos(), v2.pos()));
        } else {
            match pline_seg_intr(v1, v2, v2, v3) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !point.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        continue_visiting = visitor(PlineIntersect::new_basic(i, j, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !point1.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        continue_visiting = visitor(PlineIntersect::new_basic(i, j, point1));
                    }

                    if continue_visiting && !point2.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        pline_seg_intr(v1, v2, v2, v3);
                        continue_visiting = visitor(PlineIntersect::new_basic(i, j, point2));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    continue_visiting =
                        visitor(PlineIntersect::new_overlapping(i, j, point1, point2));
                }
            }
        }

        continue_visiting
    };

    // bool to track state of whether to continue visiting segments or not
    let mut continue_visiting = true;
    for i in 2..ln {
        continue_visiting = visit_indexes(i - 2, i - 1, i);
        if !continue_visiting {
            break;
        }
    }

    if continue_visiting && polyline.is_closed() {
        // we tested for intersect between segments at indexes 0->1, 1->2 and everything up to and
        // including (count-3)->(count-2), (count-2)->(count-1), polyline is closed so now test
        // [(count-2)->(count-1), (count-1)->0] and [(count-1)->0, 0->1]
        visit_indexes(ln - 2, ln - 1, 0);
        visit_indexes(ln - 1, 0, 1);
    }
}

/// Visits all global self intersects of the polyline. Global self intersects are defined as between
/// two polyline segments that do not share a vertex.
pub fn visit_global_self_intersects<T, F>(
    polyline: &Polyline<T>,
    spatial_index: &StaticAABB2DIndex<T>,
    visitor: &mut F,
) where
    T: Real,
    F: FnMut(PlineIntersect<T>) -> bool,
{
    let ln = polyline.len();

    if ln < 3 {
        return;
    }

    let mut visited_pairs = HashSet::with_capacity(ln);
    let mut query_stack = Vec::with_capacity(8);
    let fuzz = T::fuzzy_epsilon();

    // iterate all segment bounding boxes in spatial index querying itself to test for self intersects
    let mut break_loop = false;
    for (box_index, aabb) in spatial_index.item_boxes().iter().enumerate() {
        let i = spatial_index.map_all_boxes_index(box_index);
        let j = polyline.next_wrapping_index(i);
        let v1 = polyline[i];
        let v2 = polyline[j];
        let mut query_visitor = |hit_i: usize| -> bool {
            let hit_j = polyline.next_wrapping_index(hit_i);
            // skip local segments
            if i == hit_i || i == hit_j || j == hit_i || j == hit_j {
                return true;
            }

            // skip already visited pairs (reverse index pair order for lookup to work)
            if visited_pairs.contains(&(hit_i, i)) {
                return true;
            }

            // add pair being visited
            visited_pairs.insert((i, hit_i));

            let u1 = polyline[hit_i];
            let u2 = polyline[hit_j];
            let intr_at_start =
                |intr: Vector2<T>| -> bool { v1.pos().fuzzy_eq(intr) || u1.pos().fuzzy_eq(intr) };

            let mut continue_visiting = true;
            match pline_seg_intr(v1, v2, u1, u2) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !intr_at_start(point) {
                        continue_visiting = visitor(PlineIntersect::new_basic(i, hit_i, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !intr_at_start(point1) {
                        continue_visiting = visitor(PlineIntersect::new_basic(i, hit_i, point1));
                    }

                    if continue_visiting && !intr_at_start(point2) {
                        continue_visiting = visitor(PlineIntersect::new_basic(i, hit_i, point2));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    if !intr_at_start(point1) {
                        continue_visiting =
                            visitor(PlineIntersect::new_overlapping(i, hit_i, point1, point2));
                    }
                }
            };
            break_loop = !continue_visiting;
            continue_visiting
        };

        spatial_index.visit_query_with_stack(
            aabb.min_x - fuzz,
            aabb.min_y - fuzz,
            aabb.max_x + fuzz,
            aabb.max_y + fuzz,
            &mut query_visitor,
            &mut query_stack,
        );

        if break_loop {
            break;
        }
    }
}

/// Find all self intersects of a polyline, returning any overlapping intersects as basic intersects at each end point of overlap segment.
pub fn all_self_intersects_as_basic<T>(
    polyline: &Polyline<T>,
    spatial_index: &StaticAABB2DIndex<T>,
    pos_equal_eps: T,
) -> Vec<PlineBasicIntersect<T>>
where
    T: Real,
{
    // dbg!(local_self_intersects(polyline, pos_equal_eps));
    // dbg!(global_self_intersects(polyline, spatial_index));
    let mut intrs = Vec::new();
    let mut visitor = |intr: PlineIntersect<T>| {
        match intr {
            PlineIntersect::Basic(b) => {
                intrs.push(b);
            }
            PlineIntersect::Overlapping(o) => {
                intrs.push(PlineBasicIntersect::new(
                    o.start_index1,
                    o.start_index2,
                    o.point1,
                ));
                intrs.push(PlineBasicIntersect::new(
                    o.start_index1,
                    o.start_index2,
                    o.point2,
                ));
            }
        }
        true
    };

    visit_local_self_intersects(polyline, &mut visitor, pos_equal_eps);
    visit_global_self_intersects(polyline, spatial_index, &mut visitor);

    intrs
}

/// Find all intersects between two polylines.
///
/// In the case of overlapping intersects `point1` is always closest to the start of
/// the second segment (`start_index2`) and `point2` furthest from the start of the second segment.
///
/// Also in the case of two intersects on one segment the intersects will be added as
/// two PlineBasicIntersect in the order of distance from the start of the second segment.
pub fn find_intersects<T>(
    pline1: &Polyline<T>,
    pline2: &Polyline<T>,
    pline1_spatial_index: &StaticAABB2DIndex<T>,
) -> PlineIntersectsCollection<T>
where
    T: Real,
{
    let mut result = PlineIntersectsCollection::new_empty();
    let mut possible_duplicates = HashSet::<(usize, usize)>::new();

    let pline1_is_open = !pline1.is_closed();
    let pline2_is_open = !pline2.is_closed();

    for (i2, j2) in pline2.iter_segment_indexes() {
        let p2v1 = pline2[i2];
        let p2v2 = pline2[j2];
        let mut query_visitor = |i1: usize| {
            let j1 = pline1.next_wrapping_index(i1);
            let p1v1 = pline1[i1];
            let p1v2 = pline1[j1];

            let skip_intr_at_start = |intr: Vector2<T>| -> bool {
                // skip intersect at start position of pline segment since it will be found
                // again for the end point that connects to the start position (unless the polyline is
                // open and we're looking at the very start, then include the intersect)
                (p1v1.pos().fuzzy_eq(intr) && !(pline1_is_open && i1 == 0))
                    || (p2v1.pos().fuzzy_eq(intr) && !(pline2_is_open && i2 == 0))
            };

            match pline_seg_intr(p1v1, p1v2, p2v1, p2v2) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !skip_intr_at_start(point) {
                        result
                            .basic_intersects
                            .push(PlineBasicIntersect::new(i1, i2, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !skip_intr_at_start(point1) {
                        result
                            .basic_intersects
                            .push(PlineBasicIntersect::new(i1, i2, point1));
                    }
                    if !skip_intr_at_start(point2) {
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

                    if p1v1.pos().fuzzy_eq(point1) || p1v1.pos().fuzzy_eq(point2) {
                        possible_duplicates.insert((pline1.prev_wrapping_index(i1), i2));
                    }
                    if p2v1.pos().fuzzy_eq(point1) || p2v1.pos().fuzzy_eq(point2) {
                        possible_duplicates.insert((i1, pline2.prev_wrapping_index(i2)));
                    }
                }
            }

            // visit all query results
            true
        };

        let bb = seg_fast_approx_bounding_box(p2v1, p2v2);
        let fuzz = T::fuzzy_epsilon();

        let mut query_stack = Vec::with_capacity(8);
        pline1_spatial_index.visit_query_with_stack(
            bb.min_x - fuzz,
            bb.min_y - fuzz,
            bb.max_x + fuzz,
            bb.max_y + fuzz,
            &mut query_visitor,
            &mut query_stack,
        );
    }

    if possible_duplicates.is_empty() {
        return result;
    }

    // remove any duplicate points caused by end point intersects + overlapping
    let mut final_basic_intrs = Vec::with_capacity(result.basic_intersects.len());

    for intr in result.basic_intersects.iter() {
        if possible_duplicates.contains(&(intr.start_index1, intr.start_index2)) {
            let end_pt1 = pline1[pline1.next_wrapping_index(intr.start_index1)].pos();
            if intr.point.fuzzy_eq(end_pt1) {
                // skip including the intersect
                continue;
            }

            let end_pt2 = pline2[pline2.next_wrapping_index(intr.start_index2)].pos();
            if intr.point.fuzzy_eq(end_pt2) {
                // skip including the intersect
                continue;
            }
        }

        final_basic_intrs.push(*intr);
    }

    result.basic_intersects = final_basic_intrs;
    result
}

#[cfg(test)]
mod local_self_intersect_tests {
    use super::*;
    use crate::core_math::bulge_from_angle;

    fn local_self_intersects<T>(
        polyline: &Polyline<T>,
        pos_equal_eps: T,
    ) -> PlineIntersectsCollection<T>
    where
        T: Real,
    {
        let mut intrs = Vec::new();
        let mut overlapping_intrs = Vec::new();
        let mut visitor = |intr: PlineIntersect<T>| {
            match intr {
                PlineIntersect::Basic(b) => {
                    intrs.push(b);
                }
                PlineIntersect::Overlapping(o) => {
                    overlapping_intrs.push(o);
                }
            }
            true
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
        pline.add(1.0, 1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(2.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, -1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(0.0, 0.0, 0.0);
        let intrs = local_self_intersects(&pline, 1e-5);
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }
}

#[cfg(test)]
mod global_self_intersect_tests {
    use super::*;
    use crate::core_math::bulge_from_angle;

    fn global_self_intersects<T>(
        polyline: &Polyline<T>,
        spatial_index: &StaticAABB2DIndex<T>,
    ) -> PlineIntersectsCollection<T>
    where
        T: Real,
    {
        let mut intrs = Vec::new();
        let mut overlapping_intrs = Vec::new();
        let mut visitor = |intr: PlineIntersect<T>| {
            match intr {
                PlineIntersect::Basic(b) => {
                    intrs.push(b);
                }
                PlineIntersect::Overlapping(o) => {
                    overlapping_intrs.push(o);
                }
            }
            true
        };

        visit_global_self_intersects(polyline, spatial_index, &mut visitor);

        PlineIntersectsCollection::new(intrs, overlapping_intrs)
    }

    #[test]
    fn circle_no_intersects() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_spatial_index().unwrap());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        let pline_as_lines = pline.arcs_to_approx_lines(1e-2).unwrap();
        let intrs = global_self_intersects(
            &pline_as_lines,
            &pline_as_lines.create_approx_spatial_index().unwrap(),
        );

        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn half_circle_overlapping_self() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, -1.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_spatial_index().unwrap());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn short_open_polyline_circle() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, 1.0);
        pline.add(2.0, 0.0, 1.0);
        pline.add(0.0, 0.0, 0.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_spatial_index().unwrap());
        assert_eq!(intrs.basic_intersects.len(), 0);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
    }

    #[test]
    fn long_open_polyline_circle() {
        let mut pline = Polyline::new();
        pline.add(0.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, 1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(2.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(1.0, -1.0, bulge_from_angle(std::f64::consts::FRAC_PI_2));
        pline.add(0.0, 0.0, 0.0);
        let intrs = global_self_intersects(&pline, &pline.create_approx_spatial_index().unwrap());
        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);
        assert_eq!(intrs.basic_intersects[0].start_index1, 0);
        assert_eq!(intrs.basic_intersects[0].start_index2, 3);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, pline[4].pos(), 1e-5);
    }
}

#[cfg(test)]
mod find_intersects_tests {
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

        let intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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

        let intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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

        let intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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

        let intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

        assert_eq!(intrs.basic_intersects.len(), 1);
        assert_eq!(intrs.overlapping_intersects.len(), 0);

        let intr = intrs.basic_intersects[0];
        assert_eq!(intr.start_index1, 0);
        assert_eq!(intr.start_index2, 1);
        assert_fuzzy_eq!(intrs.basic_intersects[0].point, Vector2::new(1.0, 0.0));
    }

    #[test]
    fn circles_overlapping_same_direction() {
        let mut pline1 = Polyline::new_closed();
        pline1.add(0.0, 0.0, 1.0);
        pline1.add(1.0, 0.0, 1.0);

        let pline2 = pline1.clone();

        let mut intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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

        let mut intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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

        let mut intrs = find_intersects(
            &pline1,
            &pline2,
            &pline1.create_approx_spatial_index().unwrap(),
        );

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
}
