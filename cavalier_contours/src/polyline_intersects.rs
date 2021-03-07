use std::collections::HashSet;

use static_aabb2d_index::StaticAABB2DIndex;

use crate::{
    intersects::PlineSegIntr, pline_seg_intersect::pline_seg_intr, Polyline, Real, Vector2,
};

#[derive(Debug, Clone, Copy)]
pub struct PlineIntersect<T> {
    pub start_index1: usize,
    pub start_index2: usize,
    pub position: Vector2<T>,
}

impl<T> PlineIntersect<T> {
    pub fn new(start_index1: usize, start_index2: usize, point: Vector2<T>) -> PlineIntersect<T> {
        PlineIntersect {
            start_index1,
            start_index2,
            position: point,
        }
    }
}

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
    ) -> PlineOverlappingIntersect<T> {
        PlineOverlappingIntersect {
            start_index1,
            start_index2,
            point1,
            point2,
        }
    }
}

/// Visits all local self intersects of the polyline. Local self intersects are defined as between
/// two polyline segments that share a vertex.
pub fn visit_local_self_intersects<T, F, G>(
    polyline: &Polyline<T>,
    visitor: &mut F,
    overlap_visitor: &mut G,
    pos_equal_eps: T,
) where
    T: Real,
    F: FnMut(PlineIntersect<T>) -> bool,
    G: FnMut(PlineOverlappingIntersect<T>) -> bool,
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
                overlap_visitor(PlineOverlappingIntersect::new(
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
            continue_visiting =
                overlap_visitor(PlineOverlappingIntersect::new(i, j, v1.pos(), v2.pos()));
        } else {
            match pline_seg_intr(v1, v2, v2, v3) {
                PlineSegIntr::NoIntersect => {}
                PlineSegIntr::TangentIntersect { point } | PlineSegIntr::OneIntersect { point } => {
                    if !point.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        continue_visiting = visitor(PlineIntersect::new(i, j, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !point1.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        continue_visiting = visitor(PlineIntersect::new(i, j, point1));
                    }

                    if continue_visiting && !point2.fuzzy_eq_eps(v2.pos(), pos_equal_eps) {
                        continue_visiting = visitor(PlineIntersect::new(i, j, point2));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    continue_visiting =
                        overlap_visitor(PlineOverlappingIntersect::new(i, j, point1, point2));
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

pub fn local_self_intersects<T>(
    polyline: &Polyline<T>,
    pos_equal_eps: T,
) -> (Vec<PlineIntersect<T>>, Vec<PlineOverlappingIntersect<T>>)
where
    T: Real,
{
    let mut intrs = Vec::new();
    let mut overlapping_intrs = Vec::new();
    let mut intr_visitor = |intr: PlineIntersect<T>| {
        intrs.push(intr);
        true
    };
    let mut overlapping_visitor = |overlapping_intr: PlineOverlappingIntersect<T>| {
        overlapping_intrs.push(overlapping_intr);
        true
    };

    visit_local_self_intersects(
        polyline,
        &mut intr_visitor,
        &mut overlapping_visitor,
        pos_equal_eps,
    );

    (intrs, overlapping_intrs)
}

pub fn visit_global_self_intersects<T, F, G>(
    polyline: &Polyline<T>,
    spatial_index: &StaticAABB2DIndex<T>,
    visitor: &mut F,
    overlap_visitor: &mut G,
) where
    T: Real,
    F: FnMut(PlineIntersect<T>) -> bool,
    G: FnMut(PlineOverlappingIntersect<T>) -> bool,
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
                        continue_visiting = visitor(PlineIntersect::new(i, hit_i, point));
                    }
                }
                PlineSegIntr::TwoIntersects { point1, point2 } => {
                    if !intr_at_start(point1) {
                        continue_visiting = visitor(PlineIntersect::new(i, hit_i, point1));
                    }

                    if continue_visiting && !intr_at_start(point2) {
                        continue_visiting = visitor(PlineIntersect::new(i, hit_i, point2));
                    }
                }
                PlineSegIntr::OverlappingLines { point1, point2 }
                | PlineSegIntr::OverlappingArcs { point1, point2 } => {
                    if !intr_at_start(point1) {
                        continue_visiting = overlap_visitor(PlineOverlappingIntersect::new(
                            i, hit_i, point1, point2,
                        ));
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

pub fn global_self_intersects<T>(
    polyline: &Polyline<T>,
    spatial_index: &StaticAABB2DIndex<T>,
) -> (Vec<PlineIntersect<T>>, Vec<PlineOverlappingIntersect<T>>)
where
    T: Real,
{
    let mut intrs = Vec::new();
    let mut overlapping_intrs = Vec::new();
    let mut intr_visitor = |intr: PlineIntersect<T>| {
        intrs.push(intr);
        true
    };
    let mut overlapping_visitor = |overlapping_intr: PlineOverlappingIntersect<T>| {
        overlapping_intrs.push(overlapping_intr);
        true
    };

    visit_global_self_intersects(
        polyline,
        spatial_index,
        &mut intr_visitor,
        &mut overlapping_visitor,
    );

    (intrs, overlapping_intrs)
}

pub fn all_self_intersects<T>(
    polyline: &Polyline<T>,
    spatial_index: &StaticAABB2DIndex<T>,
    pos_equal_eps: T,
) -> Vec<PlineIntersect<T>>
where
    T: Real,
{
    let mut intrs = Vec::new();
    let mut overlapping_intrs = Vec::new();
    let mut visitor = |intr: PlineIntersect<T>| {
        intrs.push(intr);
        true
    };
    let mut overlap_visitor = |overlapping_intr: PlineOverlappingIntersect<T>| {
        overlapping_intrs.push(overlapping_intr);
        true
    };

    visit_local_self_intersects(polyline, &mut visitor, &mut overlap_visitor, pos_equal_eps);

    visit_global_self_intersects(polyline, spatial_index, &mut visitor, &mut overlap_visitor);
    intrs.reserve(2 * overlapping_intrs.len());
    for overlapping_intr in &overlapping_intrs {
        intrs.push(PlineIntersect::new(
            overlapping_intr.start_index1,
            overlapping_intr.start_index2,
            overlapping_intr.point1,
        ));
        intrs.push(PlineIntersect::new(
            overlapping_intr.start_index1,
            overlapping_intr.start_index2,
            overlapping_intr.point2,
        ));
    }

    intrs
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn local_self_intersects_tests() {
        {
            // empty polyline
            let pline = Polyline::<f64>::new();
            let (intrs, overlapping_intrs) = local_self_intersects(&pline, 1e-5);
            assert_eq!(intrs.len(), 0);
            assert_eq!(overlapping_intrs.len(), 0);
        }

        {
            // single vertex
            let mut pline = Polyline::new();
            pline.add(0.0, 0.0, 1.0);
            let (intrs, overlapping_intrs) = local_self_intersects(&pline, 1e-5);
            assert_eq!(intrs.len(), 0);
            assert_eq!(overlapping_intrs.len(), 0);
        }

        {
            // circle no intersects
            let mut pline = Polyline::new_closed();
            pline.add(0.0, 0.0, 1.0);
            pline.add(2.0, 0.0, 1.0);
            let (intrs, overlapping_intrs) = local_self_intersects(&pline, 1e-5);
            assert_eq!(intrs.len(), 0);
            assert_eq!(overlapping_intrs.len(), 0);
        }

        {
            // half circle overlapping self
            let mut pline = Polyline::new_closed();
            pline.add(0.0, 0.0, 1.0);
            pline.add(2.0, 0.0, -1.0);
            let (intrs, overlapping_intrs) = local_self_intersects(&pline, 1e-5);
            assert_eq!(intrs.len(), 0);
            assert_eq!(overlapping_intrs.len(), 1);
            assert_eq!(overlapping_intrs[0].start_index1, 0);
            assert_eq!(overlapping_intrs[0].start_index2, 1);
            assert_fuzzy_eq!(overlapping_intrs[0].point1, pline[0].pos());
            assert_fuzzy_eq!(overlapping_intrs[0].point2, pline[1].pos());
        }

        {
            // circle with line back to start
            let mut pline = Polyline::new();
            pline.add(0.0, 0.0, 1.0);
            pline.add(2.0, 0.0, 1.0);
            pline.add(0.0, 0.0, 0.0);
            let (intrs, overlapping_intrs) = local_self_intersects(&pline, 1e-5);
            assert_eq!(intrs.len(), 1);
            assert_eq!(overlapping_intrs.len(), 0);
            assert_eq!(intrs[0].start_index1, 0);
            assert_eq!(intrs[0].start_index2, 1);
            assert_fuzzy_eq!(intrs[0].position, pline[2].pos());
        }
    }
}
