use crate::{
    core::math::dist_squared,
    polyline::{
        seg_midpoint, seg_split_at_point, BooleanOp, BooleanResult, PlineBasicIntersect,
        PlineBooleanOptions, PlineVertex,
    },
};
use std::collections::HashMap;

use super::pline_intersects::{
    find_intersects, sort_and_join_overlapping_intersects, OverlappingSlice,
};
use crate::{
    core::{math::Vector2, traits::Real},
    polyline::{PlineOrientation, Polyline},
};
use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder};

pub struct ProcessForBooleanResult<T> {
    pub overlapping_slices: Vec<OverlappingSlice<T>>,
    pub intersects: Vec<PlineBasicIntersect<T>>,
    pub pline1_orientation: PlineOrientation,
    pub pline2_orientation: PlineOrientation,
}

impl<T> ProcessForBooleanResult<T>
where
    T: Real,
{
    pub fn completely_overlapping(&self, pos_equal_eps: T) -> bool {
        self.overlapping_slices.len() == 1
            && self.overlapping_slices[0].polyline[0].pos().fuzzy_eq_eps(
                self.overlapping_slices[0].polyline.last().unwrap().pos(),
                pos_equal_eps,
            )
    }

    pub fn opposing_directions(&self) -> bool {
        self.pline1_orientation != self.pline2_orientation
    }

    pub fn any_intersects(&self) -> bool {
        !self.intersects.is_empty() || !self.overlapping_slices.is_empty()
    }
}

pub fn process_for_boolean<T>(
    pline1: &Polyline<T>,
    pline2: &Polyline<T>,
    pline1_aabb_index: &StaticAABB2DIndex<T>,
    pos_equal_eps: T,
) -> ProcessForBooleanResult<T>
where
    T: Real,
{
    let mut intrs = find_intersects(pline1, pline2, pline1_aabb_index);
    let overlapping_slices = sort_and_join_overlapping_intersects(
        &mut intrs.overlapping_intersects,
        pline1,
        pline2,
        pos_equal_eps,
    );

    let pline1_orientation = pline1.orientation();
    let pline2_orientation = pline2.orientation();

    ProcessForBooleanResult {
        overlapping_slices,
        intersects: intrs.basic_intersects,
        pline1_orientation,
        pline2_orientation,
    }
}

#[derive(Debug, Clone, Copy)]
struct SlicePoint<T> {
    pos: Vector2<T>,
    is_start_of_overlapping_slice: bool,
}

impl<T> SlicePoint<T> {
    fn new(pos: Vector2<T>, is_start_of_overlapping_slice: bool) -> Self {
        Self {
            pos,
            is_start_of_overlapping_slice,
        }
    }
}

/// Slice the given pline at all of its intersects for boolean operations.
///
/// If `use_second_index` is true then the second index of the intersect types is used to correspond
/// with pline, otherwise the first index is used. `point_on_slice_pred` is called on at least one
/// point from each slice, if it returns true then the slice is kept, otherwise it is discarded.
/// `output` is populated with open polylines that represent all the slices.
pub fn slice_at_intersects<T, F>(
    pline: &Polyline<T>,
    boolean_info: &ProcessForBooleanResult<T>,
    use_second_index: bool,
    point_on_slice_pred: &mut F,
    output_slices: &mut Vec<Polyline<T>>,
    pos_equal_eps: T,
) where
    T: Real,
    F: FnMut(Vector2<T>) -> bool,
{
    let mut intersects_lookup = HashMap::<usize, Vec<SlicePoint<T>>>::with_capacity(
        boolean_info.intersects.len() + 2 * boolean_info.overlapping_slices.len(),
    );

    if use_second_index {
        // using start_index2 from intersects
        for intr in boolean_info.intersects.iter() {
            intersects_lookup
                .entry(intr.start_index2)
                .or_default()
                .push(SlicePoint::new(intr.point, false));
        }

        for overlapping_slice in boolean_info.overlapping_slices.iter() {
            let sp = overlapping_slice.polyline[0].pos();
            let ep = overlapping_slice.polyline.last().unwrap().pos();
            let sp_idx = overlapping_slice.start_indexes.1;
            let ep_idx = overlapping_slice.end_indexes.1;
            intersects_lookup
                .entry(sp_idx)
                .or_default()
                .push(SlicePoint::new(sp, true));
            intersects_lookup
                .entry(ep_idx)
                .or_default()
                .push(SlicePoint::new(ep, false));
        }
    } else {
        // use start_index1 from intersects
        for intr in boolean_info.intersects.iter() {
            intersects_lookup
                .entry(intr.start_index1)
                .or_default()
                .push(SlicePoint::new(intr.point, false));
        }

        for overlapping_slice in boolean_info.overlapping_slices.iter() {
            let sp = overlapping_slice.polyline[0].pos();
            let ep = overlapping_slice.polyline.last().unwrap().pos();
            let sp_idx = overlapping_slice.start_indexes.0;
            let ep_idx = overlapping_slice.end_indexes.0;
            // overlapping slices are always constructed following the direction of pline2 so if
            // pline1 has opposing direction then sp becomes slice end point and ep becomes slice
            // start point
            let sp_is_slice_start = !overlapping_slice.opposing_directions;
            intersects_lookup
                .entry(sp_idx)
                .or_default()
                .push(SlicePoint::new(sp, sp_is_slice_start));
            intersects_lookup
                .entry(ep_idx)
                .or_default()
                .push(SlicePoint::new(ep, !sp_is_slice_start));
        }
    }

    // sort intersects by distance from segment start vertex
    for (&i, intr_list) in intersects_lookup.iter_mut() {
        let start_pos = pline[i].pos();
        intr_list.sort_unstable_by(|intr1, intr2| {
            let dist1 = dist_squared(intr1.pos, start_pos);
            let dist2 = dist_squared(intr2.pos, start_pos);
            dist1.partial_cmp(&dist2).unwrap()
        });
    }

    for (&start_index, intrs_list) in intersects_lookup.iter() {
        let next_index = pline.next_wrapping_index(start_index);
        let start_vertex = pline[start_index];
        let end_vertex = pline[next_index];

        if intrs_list.len() != 1 {
            // build all the slices between the N intersects in intr_list (N > 1), skipping the
            // first slice (to be processed at the end)
            let first_split =
                seg_split_at_point(start_vertex, end_vertex, intrs_list[0].pos, pos_equal_eps);
            let mut prev_vertex = first_split.split_vertex;
            for i in 1..intrs_list.len() {
                let split =
                    seg_split_at_point(prev_vertex, end_vertex, intrs_list[i].pos, pos_equal_eps);
                // update prev_vertex for next loop iteration
                prev_vertex = split.split_vertex;

                if intrs_list[i - 1].is_start_of_overlapping_slice {
                    // skip overlapping slices
                    continue;
                }

                if split
                    .updated_start
                    .pos()
                    .fuzzy_eq_eps(split.split_vertex.pos(), pos_equal_eps)
                {
                    // slice end points overlap each other, skip slice
                    continue;
                }

                let midpoint = seg_midpoint(split.updated_start, split.split_vertex);
                if !point_on_slice_pred(midpoint) {
                    // failed predicate, skip slice
                    continue;
                }

                let mut slice = Polyline::new();
                slice.add_vertex(split.updated_start);
                slice.add_vertex(split.split_vertex);
                output_slices.push(slice);
            }
        }

        let last_intr = intrs_list.last().unwrap();

        if last_intr.is_start_of_overlapping_slice {
            // skip overlapping slices
            continue;
        }

        let slice_start_point = last_intr.pos;

        // build the slice between the last intersect in the intr_list and the next intersect found
        let split = seg_split_at_point(start_vertex, end_vertex, slice_start_point, pos_equal_eps);

        let mut slice = Polyline::new();
        slice.add_vertex(split.split_vertex);

        let mut index = next_index;
        let mut loop_count = 0;
        let max_loop_count = pline.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!("loop_count exceeded max_loop_count while creating slices from intersects");
            }
            loop_count += 1;

            slice.add_or_replace_vertex(pline[index], pos_equal_eps);

            // check if segment that starts at current vertex just added to slice has an intersect
            if let Some(next_intr_list) = intersects_lookup.get(&index) {
                // there is an intersect, slice is done
                let intersect_point = next_intr_list[0].pos;

                // trim last added vertex and add final intersect position
                let next_index = pline.next_wrapping_index(index);
                let split = seg_split_at_point(
                    *slice.last().unwrap(),
                    pline[next_index],
                    intersect_point,
                    pos_equal_eps,
                );
                *slice.last_mut().unwrap() = split.updated_start;

                let end_vertex = PlineVertex::from_vector2(intersect_point, T::zero());
                slice.add_or_replace_vertex(end_vertex, pos_equal_eps);
                break;
            }
            // else there is not an intersect, increment index and continue
            index = pline.next_wrapping_index(index);
        }

        // check that slice length greater than 1 and 2nd/last vertex isn't on top of the first
        if slice.len() > 1 && !slice[0].pos().fuzzy_eq_eps(slice[1].pos(), pos_equal_eps) {
            let midpoint = seg_midpoint(slice[0], slice[1]);
            if point_on_slice_pred(midpoint) {
                output_slices.push(slice);
            }
        }
    }
}

/// Holds all the slices after pruning them for the boolean operation performed. These slices can
/// then be stitched together to form the final result.
pub struct PrunedSlices<T> {
    /// Remaining slices to be stitched together.
    ///
    /// This Vec holds all the slices ordered according to their source and type: first block is
    /// pline1 non-overlapping slices, next block starting at `start_of_pline2_slices` index
    /// position is non-overlapping slices from pline2, next block starting at
    /// `start_of_pline1_overlapping_slices` is pline1 overlapping slices,
    /// and finally the last block starting at `start_of_pline1_overlapping_slices` holds pline2
    /// overlapping slices.
    pub slices_remaining: Vec<Polyline<T>>,
    pub start_of_pline2_slices: usize,
    pub start_of_pline1_overlapping_slices: usize,
    pub start_of_pline2_overlapping_slices: usize,
}

pub fn prune_slices<T, F, U>(
    pline1: &Polyline<T>,
    pline2: &Polyline<T>,
    boolean_info: &ProcessForBooleanResult<T>,
    pline1_point_on_slice_pred: &mut F,
    pline2_point_on_slice_pred: &mut U,
    set_opposing_direction: bool,
    pos_equal_eps: T,
) -> PrunedSlices<T>
where
    T: Real,
    F: FnMut(Vector2<T>) -> bool,
    U: FnMut(Vector2<T>) -> bool,
{
    let mut slices_remaining = Vec::new();
    // slice pline1
    slice_at_intersects(
        pline1,
        boolean_info,
        false,
        pline1_point_on_slice_pred,
        &mut slices_remaining,
        pos_equal_eps,
    );

    let start_of_pline2_slices = slices_remaining.len();
    // slice pline2
    slice_at_intersects(
        pline2,
        boolean_info,
        true,
        pline2_point_on_slice_pred,
        &mut slices_remaining,
        pos_equal_eps,
    );

    let start_of_pline1_overlapping_slices = slices_remaining.len();

    // reserve space for set of overlapping slices from both polylines
    slices_remaining.reserve(2 * boolean_info.overlapping_slices.len());
    // add pline1 overlapping slices
    for overlapping_slice in boolean_info.overlapping_slices.iter() {
        let mut s = overlapping_slice.polyline.clone();
        if overlapping_slice.opposing_directions {
            // invert pline1 overlapping slices to match original pline1 orientation
            s.invert_direction();
        }
        slices_remaining.push(s);
    }

    let start_of_pline2_overlapping_slices = slices_remaining.len();

    // add pline2 overlapping slices (note they are already oriented the same as pline2)
    slices_remaining.extend(
        boolean_info
            .overlapping_slices
            .iter()
            .map(|s| s.polyline.clone()),
    );

    if set_opposing_direction != boolean_info.opposing_directions() {
        // invert pline1 directions to match request to set opposing direction
        slices_remaining[0..start_of_pline2_slices]
            .iter_mut()
            .for_each(|s| s.invert_direction());
    }

    PrunedSlices {
        slices_remaining,
        start_of_pline2_slices,
        start_of_pline1_overlapping_slices,
        start_of_pline2_overlapping_slices,
    }
}
pub trait StitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize>;
}

#[derive(Debug, Clone)]
pub struct OrAndStitchSelector {
    start_of_pline2_slices: usize,
    start_of_pline1_overlapping_slices: usize,
    start_of_pline2_overlapping_slices: usize,
}

impl OrAndStitchSelector {
    pub fn new(
        start_of_pline2_slices: usize,
        start_of_pline1_overlapping_slices: usize,
        start_of_pline2_overlapping_slices: usize,
    ) -> Self {
        Self {
            start_of_pline2_slices,
            start_of_pline1_overlapping_slices,
            start_of_pline2_overlapping_slices,
        }
    }

    pub fn from_pruned_slices<T>(pruned_slices: &PrunedSlices<T>) -> Self {
        Self::new(
            pruned_slices.start_of_pline2_slices,
            pruned_slices.start_of_pline1_overlapping_slices,
            pruned_slices.start_of_pline2_overlapping_slices,
        )
    }
}

impl StitchSelector for OrAndStitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize> {
        let is_pline1_idx = current_slice_idx < self.start_of_pline2_slices
            || (current_slice_idx >= self.start_of_pline1_overlapping_slices
                && current_slice_idx < self.start_of_pline2_overlapping_slices);

        let first_available = || Some(available_idx[0]);

        let available = || available_idx.iter().copied();

        if is_pline1_idx {
            // attempt to stitch to non-overlapping pline2 slice
            available()
                .find(|&i| {
                    i >= self.start_of_pline2_slices && i < self.start_of_pline1_overlapping_slices
                })
                // attempt to stitch to non-overlapping pline1 slice
                .or_else(|| available().find(|&i| i < self.start_of_pline2_slices))
                // just use first available
                .or_else(first_available)
        } else {
            // attempt to stitch to non-overlapping pline1 slice
            available()
                .find(|&i| i < self.start_of_pline2_slices)
                // attempt to stitch to non-overlapping pline2 slice
                .or_else(|| {
                    available().find(|&i| {
                        i >= self.start_of_pline2_slices
                            && i < self.start_of_pline1_overlapping_slices
                    })
                })
                // just use first available
                .or_else(first_available)
        }
    }
}

pub struct NotXorStitchSelector {
    start_of_pline2_slices: usize,
    start_of_pline1_overlapping_slices: usize,
    start_of_pline2_overlapping_slices: usize,
}

impl NotXorStitchSelector {
    pub fn new(
        start_of_pline2_slices: usize,
        start_of_pline1_overlapping_slices: usize,
        start_of_pline2_overlapping_slices: usize,
    ) -> Self {
        Self {
            start_of_pline2_slices,
            start_of_pline1_overlapping_slices,
            start_of_pline2_overlapping_slices,
        }
    }

    pub fn from_pruned_slices<T>(pruned_slices: &PrunedSlices<T>) -> Self {
        Self::new(
            pruned_slices.start_of_pline2_slices,
            pruned_slices.start_of_pline1_overlapping_slices,
            pruned_slices.start_of_pline2_overlapping_slices,
        )
    }

    fn idx_for_pline1_slice(&self, available_idx: &[usize]) -> Option<usize> {
        available_idx
            .iter()
            .copied()
            .find(|&i| i < self.start_of_pline2_slices)
    }

    fn idx_for_pline2_slice(&self, available_idx: &[usize]) -> Option<usize> {
        available_idx.iter().copied().find(|&i| {
            i >= self.start_of_pline2_slices && i < self.start_of_pline1_overlapping_slices
        })
    }
}

impl StitchSelector for NotXorStitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize> {
        if current_slice_idx >= self.start_of_pline1_overlapping_slices {
            // current slice is overlapping
            if current_slice_idx < self.start_of_pline2_overlapping_slices {
                // current overlapping slice is from pline1
                // attempt to stitch to slice from pline2 then to
                // pline1 and if both fail then return None (stitching overlapping to overlapping is
                // never valid)
                return self
                    .idx_for_pline2_slice(available_idx)
                    .or_else(|| self.idx_for_pline1_slice(available_idx));
            }
            // else current overlapping slice is from pline2
            // attempt to stitch to slice from pline1 then to slice from pline2 and if both fail
            // then return None (stitching overlapping to overlapping is never valid)
            return self
                .idx_for_pline1_slice(available_idx)
                .or_else(|| self.idx_for_pline2_slice(available_idx));
        }

        // else current slice is not overlapping
        if current_slice_idx < self.start_of_pline2_slices {
            // current slice is from pline1, attempt to stitch to slice from pline2 and if not
            // possible then just return first available
            return self
                .idx_for_pline2_slice(available_idx)
                .or_else(|| Some(available_idx[0]));
        }

        // else current slice is from pline2, attempt to stitch to slice from pline1 and if not
        // possible then just return first available
        self.idx_for_pline1_slice(available_idx)
            .or_else(|| Some(available_idx[0]))
    }
}

/// Stitches open polyline slices together into closed polylines. The open polylines must be
/// ordered/agree on direction (every start point connects with an end point). `stitch_selector` is
/// used to determine priority of stitching in the case multiple possibilities exist.
pub fn stitch_slices_into_closed_polylines<T, S>(
    slices: &[Polyline<T>],
    stitch_selector: &S,
    slice_join_eps: T,
) -> Vec<Polyline<T>>
where
    T: Real,
    S: StitchSelector,
{
    let mut result = Vec::new();
    if slices.is_empty() {
        return result;
    }

    // load all the slice start points into spatial index
    let aabb_index = {
        let mut builder = StaticAABB2DIndexBuilder::new(slices.len());

        for slice in slices.iter() {
            let pt = slice[0].pos();
            builder.add(
                pt.x - slice_join_eps,
                pt.y - slice_join_eps,
                pt.x + slice_join_eps,
                pt.y + slice_join_eps,
            );
        }

        builder.build().unwrap()
    };

    let mut visited_slice_idx = vec![false; slices.len()];

    let mut close_pline = |mut pline: Polyline<T>| {
        if pline.len() < 3 {
            // skip slice in case of just two vertexes on top of each other
            return;
        }

        pline.remove_last();
        pline.set_is_closed(true);
        result.push(pline);
    };

    let mut query_results = Vec::new();
    let mut query_stack = Vec::with_capacity(8);

    // loop through all slice indexes
    for i in 0..slices.len() {
        if visited_slice_idx[i] {
            continue;
        }
        visited_slice_idx[i] = true;

        let mut current_pline = slices[i].clone();

        let beginning_slice_idx = i;
        let mut current_slice_idx = i;
        let mut loop_count = 0;
        let max_loop_count = slices.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!(
                    "loop_count exceeded max_loop_count while creating closed polylines from slices"
                );
            }
            loop_count += 1;

            query_results.clear();
            let mut query_visitor = |i: usize| -> bool {
                // skip already visited
                if i == beginning_slice_idx || !visited_slice_idx[i] {
                    query_results.push(i);
                }
                true
            };

            let ep = current_pline.last().unwrap().pos();
            aabb_index.visit_query_with_stack(
                ep.x - slice_join_eps,
                ep.y - slice_join_eps,
                ep.x + slice_join_eps,
                ep.y + slice_join_eps,
                &mut query_visitor,
                &mut query_stack,
            );

            if query_results.is_empty() {
                // may arrive here due to epsilon/thresholds around overlapping segments,
                // discard the pline
                break;
            }

            match stitch_selector.select(current_slice_idx, &query_results) {
                None => {
                    // discard current polyline
                    break;
                }
                Some(connected_slice_idx) if connected_slice_idx == beginning_slice_idx => {
                    // connected back to beginning, close pline and add to result
                    close_pline(current_pline);
                    break;
                }
                Some(connected_slice_idx) => {
                    current_pline.remove_last();
                    current_pline.extend(&slices[connected_slice_idx]);
                    visited_slice_idx[connected_slice_idx] = true;

                    // continue stitching slices to current pline, using last stitched index to find
                    // next
                    current_slice_idx = connected_slice_idx;
                }
            }
        }
    }

    result
}

/// Perform boolean operation between two polylines using parameters given.
pub fn polyline_boolean<T>(
    pline1: &Polyline<T>,
    pline2: &Polyline<T>,
    operation: BooleanOp,
    options: &PlineBooleanOptions<T>,
) -> BooleanResult<T>
where
    T: Real,
{
    if pline1.len() < 2 {
        return BooleanResult::new();
    }

    let constructed_index;
    let pline1_aabb_index = if let Some(x) = options.pline1_aabb_index {
        x
    } else {
        constructed_index = pline1.create_approx_aabb_index().unwrap();
        &constructed_index
    };

    let boolean_info =
        process_for_boolean(pline1, pline2, pline1_aabb_index, options.pos_equal_eps);

    // helper functions to test if point is inside pline1 and pline2
    let mut point_in_pline1 = |point: Vector2<T>| pline1.winding_number(point) != 0;
    let mut point_in_pline2 = |point: Vector2<T>| pline2.winding_number(point) != 0;

    // helper functions (assuming no intersects between pline1 and pline2)
    let is_pline1_in_pline2 = || point_in_pline2(pline1[0].pos());
    let is_pline2_in_pline1 = || point_in_pline1(pline2[0].pos());

    let mut pos_plines = Vec::new();
    let mut neg_plines = Vec::new();

    let pos_equal_eps = options.pos_equal_eps;
    let slice_join_eps = options.slice_join_eps;

    match operation {
        BooleanOp::Or => {
            if boolean_info.completely_overlapping(pos_equal_eps) {
                // pline1 completely overlapping pline2 just return pline2
                pos_plines.push(pline2.clone());
            } else if !boolean_info.any_intersects() {
                // no intersects, returning only one pline if one is inside other or both if they
                // are completely disjoint
                if is_pline1_in_pline2() {
                    pos_plines.push(pline2.clone());
                } else if is_pline2_in_pline1() {
                    pos_plines.push(pline1.clone());
                } else {
                    pos_plines.push(pline1.clone());
                    pos_plines.push(pline2.clone());
                }
            } else {
                // keep all slices of pline1 that are not in pline2 and all slices of pline2 that
                // are not in pline1
                let pruned_slices = prune_slices(
                    &pline1,
                    &pline2,
                    &boolean_info,
                    &mut |pt| !point_in_pline2(pt),
                    &mut |pt| !point_in_pline1(pt),
                    false,
                    pos_equal_eps,
                );

                let stitch_selector = OrAndStitchSelector::from_pruned_slices(&pruned_slices);

                let remaining = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    &stitch_selector,
                    slice_join_eps,
                );

                for pline in remaining {
                    let orientation = pline.orientation();
                    if orientation != boolean_info.pline2_orientation {
                        // orientation inverted from original, therefore it represents negative
                        // space
                        neg_plines.push(pline);
                    } else {
                        // orientation stayed the same, therefore it represents positive space
                        pos_plines.push(pline);
                    }
                }
            }
        }
        BooleanOp::And => {
            if boolean_info.completely_overlapping(pos_equal_eps) {
                // pline1 completely overlapping pline2 just return pline2
                pos_plines.push(pline2.clone());
            } else if !boolean_info.any_intersects() {
                // no intersects, returning only one pline if one is inside other or none if they
                // are completely disjoint
                if is_pline1_in_pline2() {
                    pos_plines.push(pline1.clone());
                } else if is_pline2_in_pline1() {
                    pos_plines.push(pline2.clone());
                }
                // else none
            } else {
                // keep all slices from pline1 that are in pline2 and all slices from pline2 that
                // are in pline1
                let pruned_slices = prune_slices(
                    &pline1,
                    &pline2,
                    &boolean_info,
                    &mut point_in_pline2,
                    &mut point_in_pline1,
                    false,
                    pos_equal_eps,
                );

                let stitch_selector = OrAndStitchSelector::from_pruned_slices(&pruned_slices);
                let remaining = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    &stitch_selector,
                    slice_join_eps,
                );
                pos_plines.extend(remaining)
            }
        }
        BooleanOp::Not => {
            if boolean_info.completely_overlapping(pos_equal_eps) {
                // completely overlapping, nothing is left
            } else if !boolean_info.any_intersects() {
                if is_pline1_in_pline2() {
                    // everything is subtracted (nothing left)
                } else if is_pline2_in_pline1() {
                    // negative space island created inside pline1
                    pos_plines.push(pline1.clone());
                    neg_plines.push(pline2.clone());
                } else {
                    // disjoint
                    pos_plines.push(pline1.clone());
                }
            } else {
                // keep all slices from pline1 that are not in pline2 and all slices on pline2 that
                // are in pline1
                let pruned_slices = prune_slices(
                    &pline1,
                    &pline2,
                    &boolean_info,
                    &mut |pt| !point_in_pline2(pt),
                    &mut point_in_pline1,
                    true,
                    pos_equal_eps,
                );

                let stitch_selector = NotXorStitchSelector::from_pruned_slices(&pruned_slices);

                let remaining = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    &stitch_selector,
                    slice_join_eps,
                );
                pos_plines.extend(remaining);
            }
        }
        BooleanOp::Xor => {
            if boolean_info.completely_overlapping(pos_equal_eps) {
                // completely overlapping, nothing is left
            } else if !boolean_info.any_intersects() {
                if is_pline1_in_pline2() {
                    pos_plines.push(pline2.clone());
                    neg_plines.push(pline1.clone());
                } else if is_pline2_in_pline1() {
                    pos_plines.push(pline1.clone());
                    neg_plines.push(pline2.clone());
                } else {
                    // disjoint
                    pos_plines.push(pline1.clone());
                    pos_plines.push(pline2.clone());
                }
            } else {
                // collect pline1 NOT pline2 results
                {
                    let pruned_slices = prune_slices(
                        &pline1,
                        &pline2,
                        &boolean_info,
                        &mut |pt| !point_in_pline2(pt),
                        &mut point_in_pline1,
                        true,
                        pos_equal_eps,
                    );

                    let stitch_selector = NotXorStitchSelector::from_pruned_slices(&pruned_slices);
                    let remaining = stitch_slices_into_closed_polylines(
                        &pruned_slices.slices_remaining,
                        &stitch_selector,
                        slice_join_eps,
                    );

                    pos_plines.extend(remaining);
                }

                // collect pline2 NOT pline1 results
                {
                    let pruned_slices = prune_slices(
                        &pline1,
                        &pline2,
                        &boolean_info,
                        &mut point_in_pline2,
                        &mut |pt| !point_in_pline1(pt),
                        true,
                        pos_equal_eps,
                    );

                    let stitch_selector = NotXorStitchSelector::from_pruned_slices(&pruned_slices);
                    let remaining = stitch_slices_into_closed_polylines(
                        &pruned_slices.slices_remaining,
                        &stitch_selector,
                        slice_join_eps,
                    );

                    pos_plines.extend(remaining);
                }
            }
        }
    }

    BooleanResult {
        pos_plines,
        neg_plines,
    }
}
