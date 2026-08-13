//! Internal stages of the polyline Boolean algorithm.
//!
//! The public items in this module are exposed only so workspace visualization and tests can
//! inspect intermediate algorithm results.

use crate::{
    core::math::dist_squared,
    polyline::{
        BooleanOp, BooleanPlineSlice, BooleanResult, BooleanResultInfo, BooleanResultPline,
        FindIntersectsOptions, PlineBasicIntersect, PlineBooleanOptions, PlineCreation,
        PlineSource, PlineViewData, seg_midpoint, seg_split_at_point,
    },
};
use std::collections::BTreeMap;

use super::pline_intersects::{
    OverlappingSlice, find_intersects, sort_and_join_overlapping_intersects,
};
use crate::{
    core::{math::Vector2, traits::Real},
    polyline::PlineOrientation,
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
    #[must_use]
    pub fn completely_overlapping(&self) -> bool {
        self.overlapping_slices.len() == 1 && self.overlapping_slices[0].is_loop
    }

    #[must_use]
    pub fn opposing_directions(&self) -> bool {
        self.pline1_orientation != self.pline2_orientation
    }

    #[must_use]
    pub fn any_intersects(&self) -> bool {
        !self.intersects.is_empty() || !self.overlapping_slices.is_empty()
    }
}

pub fn process_for_boolean<P, R, T>(
    pline1: &P,
    pline2: &R,
    pline1_aabb_index: &StaticAABB2DIndex<T>,
    pos_equal_eps: T,
) -> ProcessForBooleanResult<T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut intrs = find_intersects(
        pline1,
        pline2,
        &FindIntersectsOptions {
            pline1_aabb_index: Some(pline1_aabb_index),
            pos_equal_eps,
        },
    );
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

/// Builds intersection points keyed by segment start index and sorted along each segment.
fn build_intersects_lookup<P, T>(
    pline: &P,
    boolean_info: &ProcessForBooleanResult<T>,
    use_second_index: bool,
    pos_equal_eps: T,
) -> BTreeMap<usize, Vec<SlicePoint<T>>>
where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut intersects_lookup = BTreeMap::<usize, Vec<SlicePoint<T>>>::new();

    // helper function to adjust overlapping slice start point and endpoint indexes for lookup
    let adjust_sp_ep_indexes =
        |sp_idx: &mut usize, sp: Vector2<T>, ep_idx: &mut usize, ep: Vector2<T>| {
            // checking if positioned at end of segment in which case the point should use the next
            // index to match convention used for intersects
            let sp_idx_next = pline.next_wrapping_index(*sp_idx);
            if sp.fuzzy_eq_eps(pline.at(sp_idx_next).pos(), pos_equal_eps) {
                *sp_idx = sp_idx_next;
            }
            let ep_idx_next = pline.next_wrapping_index(*ep_idx);
            if ep.fuzzy_eq_eps(pline.at(ep_idx_next).pos(), pos_equal_eps) {
                *ep_idx = ep_idx_next;
            }
        };

    if use_second_index {
        // using start_index2 from intersects
        for intr in &boolean_info.intersects {
            intersects_lookup
                .entry(intr.start_index2)
                .or_default()
                .push(SlicePoint::new(intr.point, false));
        }

        for overlapping_slice in &boolean_info.overlapping_slices {
            let sp = overlapping_slice.view_data.updated_start.pos();
            let ep = overlapping_slice.view_data.end_point;
            let mut sp_idx = overlapping_slice.start_indexes.1;
            let mut ep_idx = overlapping_slice.end_indexes.1;
            adjust_sp_ep_indexes(&mut sp_idx, sp, &mut ep_idx, ep);

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
        for intr in &boolean_info.intersects {
            intersects_lookup
                .entry(intr.start_index1)
                .or_default()
                .push(SlicePoint::new(intr.point, false));
        }

        for overlapping_slice in &boolean_info.overlapping_slices {
            let sp = overlapping_slice.view_data.updated_start.pos();
            let ep = overlapping_slice.view_data.end_point;
            let mut sp_idx = overlapping_slice.start_indexes.0;
            let mut ep_idx = overlapping_slice.end_indexes.0;
            adjust_sp_ep_indexes(&mut sp_idx, sp, &mut ep_idx, ep);

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
    for (&i, intr_list) in &mut intersects_lookup {
        let start_pos = pline.at(i).pos();
        intr_list.sort_unstable_by(|intr1, intr2| {
            let dist1 = dist_squared(intr1.pos, start_pos);
            let dist2 = dist_squared(intr2.pos, start_pos);
            dist1.total_cmp(&dist2)
        });
    }

    intersects_lookup
}

/// Slice the given pline at all of its intersects for boolean operations.
///
/// If `use_second_index` is true then the second index of the intersect types is used to correspond
/// with pline, otherwise the first index is used. `point_on_slice_pred` is called on at least one
/// point from each slice, if it returns true then the slice is kept, otherwise it is discarded.
/// `output` is populated with open polylines that represent all the slices.
pub fn slice_at_intersects<P, T, F>(
    pline: &P,
    boolean_info: &ProcessForBooleanResult<T>,
    use_second_index: bool,
    point_on_slice_pred: &mut F,
    output_slices: &mut Vec<BooleanPlineSlice<T>>,
    pos_equal_eps: T,
) where
    P: PlineSource<Num = T> + ?Sized,
    T: Real,
    F: FnMut(Vector2<T>) -> bool,
{
    let intersects_lookup =
        build_intersects_lookup(pline, boolean_info, use_second_index, pos_equal_eps);

    for (&start_index, intrs_list) in &intersects_lookup {
        let next_index = pline.next_wrapping_index(start_index);
        let start_vertex = pline.at(start_index);
        let end_vertex = pline.at(next_index);

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

                let opl = PlineViewData::create_on_single_segment(
                    pline,
                    start_index,
                    split.updated_start,
                    split.split_vertex.pos(),
                    pos_equal_eps,
                );

                if let Some(s) = opl {
                    output_slices.push(BooleanPlineSlice::from_open_pline_slice(
                        &s,
                        !use_second_index,
                        false,
                    ));
                }
            }
        }

        let last_intr = intrs_list.last().unwrap();

        if last_intr.is_start_of_overlapping_slice {
            // skip overlapping slices
            continue;
        }

        // build the slice between the last intersect in the intr_list and the next intersect found

        let slice_start_vertex = {
            let slice_start_point = last_intr.pos;
            let split =
                seg_split_at_point(start_vertex, end_vertex, slice_start_point, pos_equal_eps);
            split.split_vertex
        };

        let mut index = next_index;
        let mut loop_count = 0;
        let max_loop_count = pline.vertex_count();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                unreachable!(
                    "loop_count exceeded max_loop_count while creating slices from intersects"
                );
            }
            loop_count += 1;

            // check if segment that starts at current vertex just added to slice has an intersect
            if let Some(next_intr_list) = intersects_lookup.get(&index) {
                // there is an intersect, slice is done
                let intersect_point = next_intr_list[0].pos;

                let slice = BooleanPlineSlice::from_open_pline_slice(
                    &PlineViewData::create(
                        pline,
                        start_index,
                        intersect_point,
                        index,
                        slice_start_vertex,
                        loop_count,
                        pos_equal_eps,
                    ),
                    !use_second_index,
                    false,
                );

                let midpoint = seg_midpoint(
                    slice.view_data.updated_start,
                    pline.at(pline.next_wrapping_index(slice.view_data.start_index)),
                );
                if point_on_slice_pred(midpoint) {
                    output_slices.push(slice);
                }

                break;
            }
            // else there is not an intersect, increment index and continue
            index = pline.next_wrapping_index(index);
        }
    }
}

/// Start indices for the ordered groups in [`PrunedSlices::slices_remaining`].
#[derive(Debug, Clone, Copy)]
pub struct SliceStarts {
    /// Start of the non-overlapping slices from the second polyline.
    pub pline2: usize,
    /// Start of the overlapping slices from the first polyline.
    pub pline1_overlapping: usize,
    /// Start of the overlapping slices from the second polyline.
    pub pline2_overlapping: usize,
}

/// Holds all the slices after pruning them for the boolean operation performed. These slices can
/// then be stitched together to form the final result.
pub struct PrunedSlices<T> {
    /// Remaining slices to be stitched together.
    ///
    /// This Vec holds all the slices ordered according to their source and type: first block is
    /// pline1 non-overlapping slices, next block starting at `starts.pline2` is non-overlapping
    /// slices from pline2, next block starting at `starts.pline1_overlapping` is pline1 overlapping
    /// slices, and finally the last block starting at `starts.pline2_overlapping` holds pline2
    /// overlapping slices.
    pub slices_remaining: Vec<BooleanPlineSlice<T>>,
    /// Start indices for each slice group.
    pub starts: SliceStarts,
}

/// Controls which non-overlapping polyline slices are kept during pruning.
///
/// Overlapping slice candidates are retained in every mode for stitch selection.
#[derive(Debug, Clone, Copy)]
pub enum PruneMode {
    /// Keeps each polyline's slices that lie outside the other polyline.
    Union,
    /// Keeps each polyline's slices that lie inside the other polyline.
    Intersection,
    /// Keeps first-polyline slices outside the second and second-polyline slices inside the first.
    FirstMinusSecond,
    /// Keeps first-polyline slices inside the second and second-polyline slices outside the first.
    SecondMinusFirst,
}

/// Prunes slices from polylines based on the specified pruning mode.
///
/// This function slices both polylines at their intersection points and filters the resulting
/// slices based on the pruning mode's requirements.
///
/// The resulting slices are organized into categories:
/// 1. Non-overlapping slices from pline1
/// 2. Non-overlapping slices from pline2
/// 3. Overlapping slices from pline1
/// 4. Overlapping slices from pline2
///
/// These categorized slices can then be stitched together to form the final boolean result.
///
/// # Arguments
///
/// - `pline1`: First polyline
/// - `pline2`: Second polyline
/// - `boolean_info`: Precomputed intersection and overlap information
/// - `mode`: The slice pruning mode to apply
/// - `pos_equal_eps`: Epsilon for position equality comparisons
///
/// # Returns
///
/// A `PrunedSlices` struct containing the filtered slices organized by category.
///
/// # Public Visibility
///
/// This function is public for visualization and testing purposes.
pub fn prune_slices<P, R, T>(
    pline1: &P,
    pline2: &R,
    boolean_info: &ProcessForBooleanResult<T>,
    mode: PruneMode,
    pos_equal_eps: T,
) -> PrunedSlices<T>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    let mut slices_remaining = Vec::new();

    let mut point_in_pline1 = |pt: Vector2<T>| pline1.winding_number(pt) != 0;
    let mut point_in_pline2 = |pt: Vector2<T>| pline2.winding_number(pt) != 0;

    // slice pline1
    match mode {
        PruneMode::Union | PruneMode::FirstMinusSecond => slice_at_intersects(
            pline1,
            boolean_info,
            false,
            &mut |pt: Vector2<T>| !point_in_pline2(pt),
            &mut slices_remaining,
            pos_equal_eps,
        ),
        PruneMode::Intersection | PruneMode::SecondMinusFirst => slice_at_intersects(
            pline1,
            boolean_info,
            false,
            &mut point_in_pline2,
            &mut slices_remaining,
            pos_equal_eps,
        ),
    }

    let pline2_start = slices_remaining.len();

    // slice pline2
    match mode {
        PruneMode::Union | PruneMode::SecondMinusFirst => slice_at_intersects(
            pline2,
            boolean_info,
            true,
            &mut |pt: Vector2<T>| !point_in_pline1(pt),
            &mut slices_remaining,
            pos_equal_eps,
        ),
        PruneMode::Intersection | PruneMode::FirstMinusSecond => slice_at_intersects(
            pline2,
            boolean_info,
            true,
            &mut point_in_pline1,
            &mut slices_remaining,
            pos_equal_eps,
        ),
    }

    let pline1_overlapping = slices_remaining.len();

    // reserve space for set of overlapping slices from both polylines
    slices_remaining.reserve(2 * boolean_info.overlapping_slices.len());

    // add pline1 overlapping slices
    slices_remaining.extend(
        boolean_info
            .overlapping_slices
            .iter()
            .map(|s| BooleanPlineSlice::from_overlapping(pline2, s, s.opposing_directions)),
    );

    let pline2_overlapping = slices_remaining.len();

    // add pline2 overlapping slices (note they are already oriented with same direction as pline2)
    slices_remaining.extend(
        boolean_info
            .overlapping_slices
            .iter()
            .map(|s| BooleanPlineSlice::from_overlapping(pline2, s, false)),
    );

    // Determine set_opposing_direction based on mode
    let set_opposing_direction = match mode {
        PruneMode::Union | PruneMode::Intersection => false,
        PruneMode::FirstMinusSecond | PruneMode::SecondMinusFirst => true,
    };

    if set_opposing_direction != boolean_info.opposing_directions() {
        // invert pline1 directions to match request to set opposing direction
        slices_remaining[0..pline2_start]
            .iter_mut()
            .for_each(|s| s.view_data.inverted_direction = true);
    }

    let starts = SliceStarts {
        pline2: pline2_start,
        pline1_overlapping,
        pline2_overlapping,
    };
    PrunedSlices {
        slices_remaining,
        starts,
    }
}
pub trait StitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize>;
}

#[derive(Debug, Clone)]
pub struct OrAndStitchSelector {
    starts: SliceStarts,
}

impl OrAndStitchSelector {
    #[must_use]
    pub fn new(starts: SliceStarts) -> Self {
        Self { starts }
    }

    #[must_use]
    pub fn from_pruned_slices<T>(pruned_slices: &PrunedSlices<T>) -> Self {
        Self::new(pruned_slices.starts)
    }
}

impl StitchSelector for OrAndStitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize> {
        let is_pline1_idx = current_slice_idx < self.starts.pline2
            || (current_slice_idx >= self.starts.pline1_overlapping
                && current_slice_idx < self.starts.pline2_overlapping);

        let first_available = || Some(available_idx[0]);

        let available = || available_idx.iter().copied();

        if is_pline1_idx {
            // attempt to stitch to non-overlapping pline2 slice
            available()
                .find(|&i| i >= self.starts.pline2 && i < self.starts.pline1_overlapping)
                // attempt to stitch to non-overlapping pline1 slice
                .or_else(|| available().find(|&i| i < self.starts.pline2))
                // just use first available
                .or_else(first_available)
        } else {
            // attempt to stitch to non-overlapping pline1 slice
            available()
                .find(|&i| i < self.starts.pline2)
                // attempt to stitch to non-overlapping pline2 slice
                .or_else(|| {
                    available()
                        .find(|&i| i >= self.starts.pline2 && i < self.starts.pline1_overlapping)
                })
                // just use first available
                .or_else(first_available)
        }
    }
}

pub struct NotXorStitchSelector {
    starts: SliceStarts,
}

impl NotXorStitchSelector {
    #[must_use]
    pub fn new(starts: SliceStarts) -> Self {
        Self { starts }
    }

    #[must_use]
    pub fn from_pruned_slices<T>(pruned_slices: &PrunedSlices<T>) -> Self {
        Self::new(pruned_slices.starts)
    }

    fn idx_for_pline1_slice(&self, available_idx: &[usize]) -> Option<usize> {
        available_idx
            .iter()
            .copied()
            .find(|&i| i < self.starts.pline2)
    }

    fn idx_for_pline2_slice(&self, available_idx: &[usize]) -> Option<usize> {
        available_idx
            .iter()
            .copied()
            .find(|&i| i >= self.starts.pline2 && i < self.starts.pline1_overlapping)
    }
}

impl StitchSelector for NotXorStitchSelector {
    fn select(&self, current_slice_idx: usize, available_idx: &[usize]) -> Option<usize> {
        if current_slice_idx >= self.starts.pline1_overlapping {
            // current slice is overlapping
            if current_slice_idx < self.starts.pline2_overlapping {
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
        if current_slice_idx < self.starts.pline2 {
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
pub fn stitch_slices_into_closed_polylines<P, R, T, S, O>(
    slices: &[BooleanPlineSlice<T>],
    source_pline1: &P,
    source_pline2: &R,
    stitch_selector: &S,
    pos_equal_eps: T,
    collapsed_area_eps: Option<T>,
) -> Vec<BooleanResultPline<O>>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
    S: StitchSelector,
    O: PlineCreation<Num = T>,
{
    let mut result = Vec::new();
    if slices.is_empty() {
        return result;
    }

    // load all the slice start points into spatial index
    let aabb_index = {
        let mut builder = StaticAABB2DIndexBuilder::new(slices.len());

        for slice in slices {
            let pt = if slice.view_data.inverted_direction {
                slice.view_data.end_point
            } else {
                slice.view_data.updated_start.pos()
            };
            builder.add(pt.x, pt.y, pt.x, pt.y);
        }

        builder.build().unwrap()
    };

    let mut visited_slice_idx = vec![false; slices.len()];

    let mut close_pline = |mut pline: O, subslices: Vec<BooleanPlineSlice<T>>| {
        // sanity assert (start should connect back with end)
        debug_assert!(
            pline
                .at(0)
                .pos()
                .fuzzy_eq_eps(pline.last().unwrap().pos(), pos_equal_eps)
        );

        if pline.vertex_count() < 3 {
            // skip slice in case of just two vertexes on top of each other
            return;
        }
        pline.remove_last();
        pline.set_is_closed(true);
        if let Some(collapsed_area) = collapsed_area_eps
            && pline.area().abs() < collapsed_area
        {
            // skip slice with area less than collapsed_area_eps
            return;
        }
        result.push(BooleanResultPline::new(pline, subslices));
    };

    let mut query_results = Vec::new();
    let mut query_stack = Vec::with_capacity(8);

    let slice_to_pline = |s: &BooleanPlineSlice<T>| {
        if s.source_is_pline1 {
            O::create_from_remove_repeat(&s.view(source_pline1), pos_equal_eps)
        } else {
            O::create_from_remove_repeat(&s.view(source_pline2), pos_equal_eps)
        }
    };

    let stitch_slice_onto = |s: &BooleanPlineSlice<T>, target: &mut O| {
        if s.source_is_pline1 {
            target.extend_remove_repeat(&s.view(source_pline1), pos_equal_eps);
        } else {
            target.extend_remove_repeat(&s.view(source_pline2), pos_equal_eps);
        }
    };

    // loop through all slice indexes
    for i in 0..slices.len() {
        if visited_slice_idx[i] {
            continue;
        }
        visited_slice_idx[i] = true;

        let s = slices[i];
        let mut current_pline: O = slice_to_pline(&s);
        let mut subslices = vec![s];

        let beginning_slice_idx = i;
        let mut current_slice_idx = i;
        let mut loop_count = 0;
        let max_loop_count = slices.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                unreachable!(
                    "loop_count exceeded max_loop_count while creating closed polylines from slices"
                );
            }
            loop_count += 1;

            query_results.clear();
            let mut query_visitor = |i: usize| {
                // skip already visited
                if i == beginning_slice_idx || !visited_slice_idx[i] {
                    query_results.push(i);
                }
            };

            let ep = current_pline.last().unwrap().pos();
            aabb_index.visit_query_with_stack(
                ep.x - pos_equal_eps,
                ep.y - pos_equal_eps,
                ep.x + pos_equal_eps,
                ep.y + pos_equal_eps,
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
                    close_pline(current_pline, subslices);
                    break;
                }
                Some(connected_slice_idx) => {
                    let s = slices[connected_slice_idx];
                    current_pline.remove_last();
                    stitch_slice_onto(&s, &mut current_pline);
                    visited_slice_idx[connected_slice_idx] = true;
                    subslices.push(s);

                    // continue stitching slices to current pline, using last stitched index to find
                    // next
                    current_slice_idx = connected_slice_idx;
                }
            }
        }
    }

    let mut composite_userdata: Vec<u64> = Vec::new();
    composite_userdata.extend(source_pline1.get_userdata_values());
    composite_userdata.extend(source_pline2.get_userdata_values());

    for result_item in &mut result {
        result_item
            .pline
            .set_userdata_values(composite_userdata.iter().copied());
    }

    result
}

/// Perform boolean operation between two polylines using parameters given.
pub fn polyline_boolean<P, R, O, T>(
    pline1: &P,
    pline2: &R,
    operation: BooleanOp,
    options: &PlineBooleanOptions<T>,
) -> BooleanResult<O>
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
    O: PlineCreation<Num = T>,
{
    if pline1.vertex_count() < 2
        || !pline1.is_closed()
        || pline2.vertex_count() < 2
        || !pline2.is_closed()
    {
        return BooleanResult::empty(BooleanResultInfo::InvalidInput);
    }

    let constructed_index;
    let pline1_aabb_index = if let Some(x) = options.pline1_aabb_index {
        x
    } else {
        constructed_index = pline1.create_approx_aabb_index();
        &constructed_index
    };

    let boolean_info =
        process_for_boolean(pline1, pline2, pline1_aabb_index, options.pos_equal_eps);

    // helper functions (assuming no intersects between pline1 and pline2)
    let is_pline1_in_pline2 = || pline2.winding_number(pline1.at(0).pos()) != 0;
    let is_pline2_in_pline1 = || pline1.winding_number(pline2.at(0).pos()) != 0;

    let pos_equal_eps = options.pos_equal_eps;
    let collapsed_area_eps = options.collapsed_area_eps;

    match operation {
        BooleanOp::Or => {
            if boolean_info.completely_overlapping() {
                // pline1 completely overlapping pline2 just return pline2
                BooleanResult::from_whole_plines(
                    vec![O::create_from(pline2)],
                    Vec::new(),
                    BooleanResultInfo::Overlapping,
                )
            } else if !boolean_info.any_intersects() {
                // no intersects, returning only one pline if one is inside other or both if they
                // are completely disjoint
                if is_pline1_in_pline2() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline2)],
                        Vec::new(),
                        BooleanResultInfo::Pline1InsidePline2,
                    )
                } else if is_pline2_in_pline1() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1)],
                        Vec::new(),
                        BooleanResultInfo::Pline2InsidePline1,
                    )
                } else {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1), O::create_from(pline2)],
                        Vec::new(),
                        BooleanResultInfo::Disjoint,
                    )
                }
            } else {
                // keep all slices of pline1 that are not in pline2 and all slices of pline2 that
                // are not in pline1
                let pruned_slices = prune_slices(
                    pline1,
                    pline2,
                    &boolean_info,
                    PruneMode::Union,
                    pos_equal_eps,
                );

                let stitch_selector = OrAndStitchSelector::from_pruned_slices(&pruned_slices);

                let remaining: Vec<BooleanResultPline<O>> = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    pline1,
                    pline2,
                    &stitch_selector,
                    pos_equal_eps,
                    collapsed_area_eps,
                );

                let mut pos_plines = Vec::new();
                let mut neg_plines = Vec::new();

                for result_pline in remaining {
                    let orientation = result_pline.pline.orientation();
                    if orientation == boolean_info.pline2_orientation {
                        // orientation stayed the same, therefore it represents positive space
                        pos_plines.push(result_pline);
                    } else {
                        // orientation inverted from original, therefore it represents negative
                        // space
                        neg_plines.push(result_pline);
                    }
                }

                BooleanResult::new(pos_plines, neg_plines, BooleanResultInfo::Intersected)
            }
        }
        BooleanOp::And => {
            if boolean_info.completely_overlapping() {
                // pline1 completely overlapping pline2 just return pline2
                BooleanResult::from_whole_plines(
                    vec![O::create_from(pline2)],
                    Vec::new(),
                    BooleanResultInfo::Overlapping,
                )
            } else if !boolean_info.any_intersects() {
                // no intersects, returning only one pline if one is inside other or none if they
                // are completely disjoint
                if is_pline1_in_pline2() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1)],
                        Vec::new(),
                        BooleanResultInfo::Pline1InsidePline2,
                    )
                } else if is_pline2_in_pline1() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline2)],
                        Vec::new(),
                        BooleanResultInfo::Pline2InsidePline1,
                    )
                } else {
                    BooleanResult::empty(BooleanResultInfo::Disjoint)
                }
            } else {
                // keep all slices from pline1 that are in pline2 and all slices from pline2 that
                // are in pline1
                let pruned_slices = prune_slices(
                    pline1,
                    pline2,
                    &boolean_info,
                    PruneMode::Intersection,
                    pos_equal_eps,
                );

                let stitch_selector = OrAndStitchSelector::from_pruned_slices(&pruned_slices);
                let pos_plines = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    pline1,
                    pline2,
                    &stitch_selector,
                    pos_equal_eps,
                    collapsed_area_eps,
                );

                BooleanResult::new(pos_plines, Vec::new(), BooleanResultInfo::Intersected)
            }
        }
        BooleanOp::Not => {
            if boolean_info.completely_overlapping() {
                // completely overlapping, nothing is left
                BooleanResult::empty(BooleanResultInfo::Overlapping)
            } else if !boolean_info.any_intersects() {
                if is_pline1_in_pline2() {
                    // everything is subtracted (nothing left)
                    BooleanResult::empty(BooleanResultInfo::Pline1InsidePline2)
                } else if is_pline2_in_pline1() {
                    // negative space island created inside pline1
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1)],
                        vec![O::create_from(pline2)],
                        BooleanResultInfo::Pline2InsidePline1,
                    )
                } else {
                    // disjoint
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1)],
                        Vec::new(),
                        BooleanResultInfo::Disjoint,
                    )
                }
            } else {
                // keep all slices from pline1 that are not in pline2 and all slices on pline2 that
                // are in pline1
                let pruned_slices = prune_slices(
                    pline1,
                    pline2,
                    &boolean_info,
                    PruneMode::FirstMinusSecond,
                    pos_equal_eps,
                );

                let stitch_selector = NotXorStitchSelector::from_pruned_slices(&pruned_slices);

                let pos_plines = stitch_slices_into_closed_polylines(
                    &pruned_slices.slices_remaining,
                    pline1,
                    pline2,
                    &stitch_selector,
                    pos_equal_eps,
                    collapsed_area_eps,
                );

                BooleanResult::new(pos_plines, Vec::new(), BooleanResultInfo::Intersected)
            }
        }
        BooleanOp::Xor => {
            if boolean_info.completely_overlapping() {
                BooleanResult::empty(BooleanResultInfo::Overlapping)
            } else if !boolean_info.any_intersects() {
                if is_pline1_in_pline2() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline2)],
                        vec![O::create_from(pline1)],
                        BooleanResultInfo::Pline1InsidePline2,
                    )
                } else if is_pline2_in_pline1() {
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1)],
                        vec![O::create_from(pline2)],
                        BooleanResultInfo::Pline2InsidePline1,
                    )
                } else {
                    // disjoint
                    BooleanResult::from_whole_plines(
                        vec![O::create_from(pline1), O::create_from(pline2)],
                        Vec::new(),
                        BooleanResultInfo::Disjoint,
                    )
                }
            } else {
                // collect pline1 NOT pline2 results
                let pruned_slices1 = prune_slices(
                    pline1,
                    pline2,
                    &boolean_info,
                    PruneMode::FirstMinusSecond,
                    pos_equal_eps,
                );

                let stitch_selector1 = NotXorStitchSelector::from_pruned_slices(&pruned_slices1);
                let mut remaining1 = stitch_slices_into_closed_polylines(
                    &pruned_slices1.slices_remaining,
                    pline1,
                    pline2,
                    &stitch_selector1,
                    pos_equal_eps,
                    collapsed_area_eps,
                );

                // collect pline2 NOT pline1 results
                let pruned_slices2 = prune_slices(
                    pline1,
                    pline2,
                    &boolean_info,
                    PruneMode::SecondMinusFirst,
                    pos_equal_eps,
                );

                let stitch_selector2 = NotXorStitchSelector::from_pruned_slices(&pruned_slices2);
                let remaining2 = stitch_slices_into_closed_polylines(
                    &pruned_slices2.slices_remaining,
                    pline1,
                    pline2,
                    &stitch_selector2,
                    pos_equal_eps,
                    collapsed_area_eps,
                );

                remaining1.extend(remaining2);
                BooleanResult::new(remaining1, Vec::new(), BooleanResultInfo::Intersected)
            }
        }
    }
}
