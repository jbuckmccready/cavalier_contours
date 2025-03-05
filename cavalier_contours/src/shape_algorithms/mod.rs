use std::collections::{BTreeMap, BTreeSet};

use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder};

use crate::{
    core::{
        math::{Vector2, dist_squared},
        traits::Real,
    },
    polyline::{
        BooleanOp, BooleanResultInfo, FindIntersectsOptions, PlineBasicIntersect, PlineInversionView,
        PlineOffsetOptions, PlineOrientation, PlineSource, PlineSourceMut, PlineViewData, Polyline,
        internal::pline_offset::point_valid_for_offset, seg_midpoint,
    },
};

struct OffsetLoop<T: Real> {
    pub parent_loop_idx: usize,
    pub indexed_pline: IndexedPolyline<T>,
}

impl<T> Default for OffsetLoop<T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self {
            parent_loop_idx: Default::default(),
            indexed_pline: IndexedPolyline::new(Polyline::new()),
        }
    }
}

#[derive(Debug, Clone)]
pub struct IndexedPolyline<T: Real> {
    pub polyline: Polyline<T>,
    pub spatial_index: StaticAABB2DIndex<T>,
}

impl<T> IndexedPolyline<T>
where
    T: Real,
{
    pub fn new(polyline: Polyline<T>) -> Self {
        let spatial_index = polyline.create_approx_aabb_index();
        Self {
            polyline,
            spatial_index,
        }
    }

    fn parallel_offset_for_shape(
        &self,
        offset: T,
        options: &ShapeOffsetOptions<T>,
    ) -> Vec<Polyline<T>> {
        let opts = PlineOffsetOptions {
            aabb_index: Some(&self.spatial_index),
            handle_self_intersects: false,
            pos_equal_eps: options.pos_equal_eps,
            slice_join_eps: options.slice_join_eps,
            offset_dist_eps: options.offset_dist_eps,
        };

        self.polyline.parallel_offset_opt(offset, &opts)
    }
}

/// Shape represented by positive area counter clockwise polylines, `ccw_plines` and negative/hole
/// area clockwise polylines, `cw_plines`.
#[derive(Debug, Clone)]
pub struct Shape<T: Real> {
    /// Positive/filled area counter clockwise polylines.
    pub ccw_plines: Vec<IndexedPolyline<T>>,
    /// Negative/hole area clockwise polylines.
    pub cw_plines: Vec<IndexedPolyline<T>>,
    /// Spatial index of all the polyline area bounding boxes, index positions correspond to in
    /// order all the counter clockwise polylines followed by all the clockwise polylines. E.g., if
    /// there is 1 `ccw_plines` and 2 `cw_plines` then index position 0 is the bounding box for the
    /// ccw pline and index positions 1 and 2 correspond to the first and second cw plines.
    pub plines_index: StaticAABB2DIndex<T>,
}

/// Struct to hold options parameters when performing shape offset.
#[derive(Debug, Clone)]
pub struct ShapeOffsetOptions<T> {
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// Fuzzy comparison epsilon used when testing distance of slices to original polyline for
    /// validity.
    pub offset_dist_eps: T,
    /// Fuzzy comparison epsilon used for determining if two positions are equal when stitching
    /// polyline slices together.
    pub slice_join_eps: T,
}

/// A builder type for performing multiple transformations on a `Shape`
/// without rebuilding indexes each time. The transforms are recorded
/// and only applied in the final `build` step.
pub struct ShapeTransformBuilder<T: Real> {
    // We can either store an owned shape or a borrowed shape. Typically
    // you might store an owned shape if you want to produce a new shape
    // at the end and let the builder consume the old shape.
    shape: Shape<T>,

    // We can store the transformations in a small struct of
    // "accumulated transform" if we only want uniform scale, rotate, etc.
    // But we also want "invert direction" logic, which cannot be captured by a matrix alone.
    // So let's store each step in an enum, or keep separate booleans/accumulations.
    scale_factor: T,
    // For non-uniform scale:
    // scale_x: T,
    // scale_y: T,
    rotate_angle: T,
    rotate_anchor: Option<(T, T)>,
    translate_x: T,
    translate_y: T,

    // If we want to invert direction or do multiple times, we only need to track an invert bool:
    invert_direction: bool,
    // Possibly track repeated rotates or merges them. But let's keep it simple:
    // We'll just store these as "accumulated transformations" for scale/rotate/translate,
    // plus a boolean for invert direction. More complicated steps can be added if needed.
}

impl<T> ShapeOffsetOptions<T>
where
    T: Real,
{
    #[inline]
    pub fn new() -> Self {
        Self {
            pos_equal_eps: T::from(1e-5).unwrap(),
            offset_dist_eps: T::from(1e-4).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
        }
    }
}

impl<T> Default for ShapeOffsetOptions<T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Shape<T>
where
    T: Real,
{
    pub fn from_plines<I>(plines: I) -> Self
    where
        I: IntoIterator<Item = Polyline<T>>,
    {
        let mut ccw_plines = Vec::new();
        let mut cw_plines = Vec::new();
        // skip empty polylines
        for pl in plines.into_iter().filter(|p| p.vertex_count() > 1) {
            if pl.orientation() == PlineOrientation::CounterClockwise {
                ccw_plines.push(IndexedPolyline::new(pl));
            } else {
                cw_plines.push(IndexedPolyline::new(pl));
            }
        }

        let plines_index = {
            let mut b = StaticAABB2DIndexBuilder::new(ccw_plines.len() + cw_plines.len());

            let mut add_all_bounds = |plines: &[IndexedPolyline<T>]| {
                for pline in plines.iter() {
                    let bounds = pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_plines);
            add_all_bounds(&cw_plines);

            b.build().unwrap()
        };

        Self {
            ccw_plines,
            cw_plines,
            plines_index,
        }
    }

    /// Return an empty shape (0 polylines).
    #[inline]
    pub fn empty() -> Self {
        Self {
            ccw_plines: Vec::new(),
            cw_plines: Vec::new(),
            plines_index: StaticAABB2DIndexBuilder::new(0).build().unwrap(),
        }
    }

    pub fn parallel_offset(&self, offset: T, options: ShapeOffsetOptions<T>) -> Self {
        let pos_equal_eps = options.pos_equal_eps;
        let offset_dist_eps = options.offset_dist_eps;
        let slice_join_eps = options.slice_join_eps;

        // generate offset loops
        let mut ccw_offset_loops = Vec::new();
        let mut cw_offset_loops = Vec::new();
        let mut parent_idx = 0;
        for pline in self.ccw_plines.iter() {
            for offset_pline in pline.parallel_offset_for_shape(offset, &options) {
                // check if orientation inverted (due to collapse of very narrow or small input)
                // skip if inversion happened (ccw became cw while offsetting inward)
                let area = offset_pline.area();
                if offset > T::zero() && area < T::zero() {
                    continue;
                }

                let offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline),
                };

                if area < T::zero() {
                    cw_offset_loops.push(offset_loop);
                } else {
                    ccw_offset_loops.push(offset_loop);
                }
            }

            parent_idx += 1;
        }

        for pline in self.cw_plines.iter() {
            for offset_pline in pline.parallel_offset_for_shape(offset, &options) {
                let area = offset_pline.area();

                // check if orientation inverted (due to collapse of very narrow or small input)
                // skip if inversion happened (cw became ccw while offsetting inward)
                if offset < T::zero() && area > T::zero() {
                    continue;
                }

                let offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline),
                };

                if area < T::zero() {
                    cw_offset_loops.push(offset_loop);
                } else {
                    ccw_offset_loops.push(offset_loop);
                }
            }
            parent_idx += 1;
        }

        let offset_loop_count = ccw_offset_loops.len() + cw_offset_loops.len();
        if offset_loop_count == 0 {
            // no offsets remaining
            return Self::empty();
        }

        // build spatial index of offset loop approximate bounding boxes
        let offset_loops_index = {
            let mut b =
                StaticAABB2DIndexBuilder::new(ccw_offset_loops.len() + cw_offset_loops.len());

            let mut add_all_bounds = |loops: &[OffsetLoop<T>]| {
                for l in loops.iter() {
                    let bounds = l
                        .indexed_pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_offset_loops);
            add_all_bounds(&cw_offset_loops);

            b.build()
                .expect("failed to build spatial index of offset loop bounds")
        };

        let mut ccw_plines_result = Vec::new();
        let mut cw_plines_result = Vec::new();

        // find all intersects between all offsets to create slice points
        let mut slice_point_sets = Vec::new();
        let mut slice_points_lookup = BTreeMap::<usize, Vec<usize>>::new();
        let mut visited_loop_pairs = BTreeSet::<(usize, usize)>::new();
        let mut query_stack = Vec::new();

        for i in 0..offset_loop_count {
            let loop1 = Self::get_loop(i, &ccw_offset_loops, &cw_offset_loops);
            let spatial_idx1 = &loop1.indexed_pline.spatial_index;
            let bounds = spatial_idx1.bounds().expect("expect non-empty polyline");
            let query_results = offset_loops_index.query_with_stack(
                bounds.min_x,
                bounds.min_y,
                bounds.max_x,
                bounds.max_y,
                &mut query_stack,
            );

            for j in query_results {
                if i == j {
                    // skip same index (no self intersects among the offset loops)
                    continue;
                }

                if visited_loop_pairs.contains(&(j, i)) {
                    // skip reversed index order (would end up comparing the same loops in another
                    // iteration)
                    continue;
                }

                visited_loop_pairs.insert((i, j));

                let loop2 = Self::get_loop(j, &ccw_offset_loops, &cw_offset_loops);

                let intrs_opts = FindIntersectsOptions {
                    pline1_aabb_index: Some(spatial_idx1),
                    pos_equal_eps,
                };

                let intersects = loop1
                    .indexed_pline
                    .polyline
                    .find_intersects_opt(&loop2.indexed_pline.polyline, &intrs_opts);

                if intersects.basic_intersects.is_empty()
                    && intersects.overlapping_intersects.is_empty()
                {
                    continue;
                }

                let mut slice_points = Vec::new();

                for intr in intersects.basic_intersects {
                    slice_points.push(intr);
                }

                // add overlapping start and end points
                for overlap_intr in intersects.overlapping_intersects {
                    let start_index1 = overlap_intr.start_index1;
                    let start_index2 = overlap_intr.start_index2;
                    slice_points.push(PlineBasicIntersect {
                        start_index1,
                        start_index2,
                        point: overlap_intr.point1,
                    });
                    slice_points.push(PlineBasicIntersect {
                        start_index1,
                        start_index2,
                        point: overlap_intr.point2,
                    });
                }

                let slice_point_set = SlicePointSet {
                    loop_idx1: i,
                    loop_idx2: j,
                    slice_points,
                };

                slice_points_lookup
                    .entry(i)
                    .or_default()
                    .push(slice_point_sets.len());

                slice_points_lookup
                    .entry(j)
                    .or_default()
                    .push(slice_point_sets.len());

                slice_point_sets.push(slice_point_set);
            }
        }

        // create slices from slice points
        let mut sorted_intrs = Vec::new();
        let mut slices_data = Vec::new();

        let create_slice = |pt1: &DissectionPoint<T>,
                            pt2: &DissectionPoint<T>,
                            offset_loop: &Polyline<T>|
         -> Option<PlineViewData<T>> {
            PlineViewData::from_slice_points(
                offset_loop,
                pt1.pos,
                pt1.seg_idx,
                pt2.pos,
                pt2.seg_idx,
                pos_equal_eps,
            )
        };

        let is_slice_valid = |v_data: &PlineViewData<T>,
                              offset_loop: &Polyline<T>,
                              parent_idx: usize,
                              query_stack: &mut Vec<usize>|
         -> bool {
            let slice_view = v_data.view(offset_loop);
            let midpoint = seg_midpoint(slice_view.at(0), slice_view.at(1));
            // loop through input polylines and check if slice is too close (skipping parent
            // polyline since it's never too close)
            for input_loop_idx in
                (0..(self.ccw_plines.len() + self.cw_plines.len())).filter(|i| *i != parent_idx)
            {
                let parent_loop = if input_loop_idx < self.ccw_plines.len() {
                    &self.ccw_plines[input_loop_idx]
                } else {
                    &self.cw_plines[input_loop_idx - self.ccw_plines.len()]
                };

                if !point_valid_for_offset(
                    &parent_loop.polyline,
                    offset,
                    &parent_loop.spatial_index,
                    midpoint,
                    query_stack,
                    pos_equal_eps,
                    offset_dist_eps,
                ) {
                    return false;
                }
            }

            true
        };

        for loop_idx in 0..offset_loop_count {
            sorted_intrs.clear();
            let curr_loop = Self::get_loop(loop_idx, &ccw_offset_loops, &cw_offset_loops);

            if let Some(slice_point_set_idxs) = slice_points_lookup.get(&loop_idx) {
                // gather all the intersects for the current loop
                sorted_intrs.extend(slice_point_set_idxs.iter().flat_map(|set_idx| {
                    let set = &slice_point_sets[*set_idx];
                    debug_assert!(set.loop_idx1 == loop_idx || set.loop_idx2 == loop_idx);
                    let loop_is_first_index = set.loop_idx1 == loop_idx;
                    set.slice_points.iter().map(move |intr_pt| {
                        let seg_idx = if loop_is_first_index {
                            intr_pt.start_index1
                        } else {
                            intr_pt.start_index2
                        };
                        DissectionPoint {
                            seg_idx,
                            pos: intr_pt.point,
                        }
                    })
                }));

                // sort the intersect points along direction of polyline
                sorted_intrs.sort_unstable_by(|a, b| {
                    // sort by the segment index, then if both intersects on the same segment sort
                    // by distance from start of segment
                    a.seg_idx.cmp(&b.seg_idx).then_with(|| {
                        let seg_start = curr_loop.indexed_pline.polyline.at(a.seg_idx).pos();
                        let dist1 = dist_squared(a.pos, seg_start);
                        let dist2 = dist_squared(b.pos, seg_start);
                        dist1.total_cmp(&dist2)
                    })
                });

                // construct valid slices to later be stitched together
                if sorted_intrs.len() == 1 {
                    // treat whole loop as slice
                    let v_data =
                        PlineViewData::from_entire_pline(&curr_loop.indexed_pline.polyline);
                    if is_slice_valid(
                        &v_data,
                        &curr_loop.indexed_pline.polyline,
                        curr_loop.parent_loop_idx,
                        &mut query_stack,
                    ) {
                        slices_data.push(DissectedSlice {
                            source_idx: loop_idx,
                            v_data,
                        });
                    }
                } else {
                    // create slices from adjacent points
                    let mut windows = sorted_intrs.windows(2);
                    while let Some([pt1, pt2]) = windows.next() {
                        let v_data = create_slice(pt1, pt2, &curr_loop.indexed_pline.polyline);
                        if let Some(v_data) = v_data {
                            if is_slice_valid(
                                &v_data,
                                &curr_loop.indexed_pline.polyline,
                                curr_loop.parent_loop_idx,
                                &mut query_stack,
                            ) {
                                slices_data.push(DissectedSlice {
                                    source_idx: loop_idx,
                                    v_data,
                                });
                            }
                        }
                    }

                    // collect slice from last to start
                    let pt1 = sorted_intrs.last().unwrap();
                    let pt2 = &sorted_intrs[0];
                    let v_data = create_slice(pt1, pt2, &curr_loop.indexed_pline.polyline);
                    if let Some(v_data) = v_data {
                        if is_slice_valid(
                            &v_data,
                            &curr_loop.indexed_pline.polyline,
                            curr_loop.parent_loop_idx,
                            &mut query_stack,
                        ) {
                            slices_data.push(DissectedSlice {
                                source_idx: loop_idx,
                                v_data,
                            });
                        }
                    }
                }
            } else {
                // no intersects but still must test distance of one vertex position since it may be
                // inside another offset (completely eclipsed by island offset)
                let v_data = PlineViewData::from_entire_pline(&curr_loop.indexed_pline.polyline);
                if is_slice_valid(
                    &v_data,
                    &curr_loop.indexed_pline.polyline,
                    curr_loop.parent_loop_idx,
                    &mut query_stack,
                ) {
                    // Take/consume the loop to avoid allocation and copy required to clone
                    if loop_idx < ccw_offset_loops.len() {
                        let r = std::mem::take(&mut ccw_offset_loops[loop_idx]).indexed_pline;
                        ccw_plines_result.push(r);
                    } else {
                        let i = loop_idx - ccw_offset_loops.len();
                        let r = std::mem::take(&mut cw_offset_loops[i]).indexed_pline;
                        cw_plines_result.push(r)
                    }
                }
            }
        }

        // stitch slices together
        let slice_starts_aabb_index = {
            let mut builder = StaticAABB2DIndexBuilder::new(slices_data.len());
            for slice in slices_data.iter() {
                let start_point = slice.v_data.updated_start.pos();
                builder.add(
                    start_point.x - slice_join_eps,
                    start_point.y - slice_join_eps,
                    start_point.x + slice_join_eps,
                    start_point.y + slice_join_eps,
                );
            }
            builder.build().unwrap()
        };
        let mut visited_slices_idxs = vec![false; slices_data.len()];
        let mut query_results = Vec::new();
        for slice_idx in 0..slices_data.len() {
            if visited_slices_idxs[slice_idx] {
                continue;
            }
            visited_slices_idxs[slice_idx] = true;

            let mut current_index = slice_idx;
            let mut loop_count = 0;
            let max_loop_count = slices_data.len();
            let mut current_pline = Polyline::new();
            loop {
                if loop_count > max_loop_count {
                    // prevent infinite loop
                    unreachable!(
                        "loop_count exceeded max_loop_count while stitching slices together"
                    );
                }
                loop_count += 1;

                let curr_slice = &slices_data[current_index];
                let source_loop =
                    Self::get_loop(curr_slice.source_idx, &ccw_offset_loops, &cw_offset_loops);
                let slice_view = curr_slice.v_data.view(&source_loop.indexed_pline.polyline);
                current_pline.extend_remove_repeat(&slice_view, pos_equal_eps);

                query_results.clear();
                let slice_end_point = curr_slice.v_data.end_point;
                let mut aabb_index_visitor = |i: usize| {
                    if !visited_slices_idxs[i] {
                        query_results.push(i);
                    }
                };
                slice_starts_aabb_index.visit_query_with_stack(
                    slice_end_point.x - slice_join_eps,
                    slice_end_point.y - slice_join_eps,
                    slice_end_point.x + slice_join_eps,
                    slice_end_point.y + slice_join_eps,
                    &mut aabb_index_visitor,
                    &mut query_stack,
                );

                if query_results.is_empty() {
                    if current_pline.vertex_count() > 2 {
                        current_pline.remove_last();
                        current_pline.set_is_closed(true);
                    }
                    let is_ccw = current_pline.orientation() == PlineOrientation::CounterClockwise;
                    if is_ccw {
                        ccw_plines_result.push(IndexedPolyline::new(current_pline));
                    } else {
                        cw_plines_result.push(IndexedPolyline::new(current_pline));
                    }
                    break;
                }

                current_index = query_results
                    .iter()
                    .find_map(|i| {
                        let slice = &slices_data[*i];
                        if slice.source_idx == curr_slice.source_idx {
                            Some(*i)
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| query_results[0]);

                visited_slices_idxs[current_index] = true;
            }
        }

        let plines_index = {
            let mut b =
                StaticAABB2DIndexBuilder::new(ccw_plines_result.len() + cw_plines_result.len());

            let mut add_all_bounds = |plines: &[IndexedPolyline<T>]| {
                for pline in plines.iter() {
                    let bounds = pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_plines_result);
            add_all_bounds(&cw_plines_result);

            b.build().unwrap()
        };

        Shape {
            ccw_plines: ccw_plines_result,
            cw_plines: cw_plines_result,
            plines_index,
        }
    }

    fn get_loop<'a>(
        i: usize,
        s1: &'a [OffsetLoop<T>],
        s2: &'a [OffsetLoop<T>],
    ) -> &'a OffsetLoop<T> {
        if i < s1.len() {
            &s1[i]
        } else {
            &s2[i - s1.len()]
        }
    }

    /// Perform a boolean operation between `self` and `other` producing a new `Shape<T>`.
    /// The `op` can be `BooleanOp::Or`, `BooleanOp::And`, `BooleanOp::Not`, or `BooleanOp::Xor`.
    pub fn boolean(&self, other: &Self, op: BooleanOp) -> Self {
        // 1) Collect all polylines from self & other in a "signed" fashion:
        //    - ccw_plines => "positive"
        //    - cw_plines => "negative" or invert them, depending on how
        //      the library’s polyline boolean expects holes.
        // (We can do a naive approach: treat CW as simply "inverted" polylines.)

        let mut all_results = Vec::new();

        // Helper for checking bounding-box overlap
        #[inline]
        fn boxes_overlap<T: Real>(
            b1: &static_aabb2d_index::AABB<T>,
            b2: &static_aabb2d_index::AABB<T>,
        ) -> bool {
            b1.min_x <= b2.max_x
                && b1.max_x >= b2.min_x
                && b1.min_y <= b2.max_y
                && b1.max_y >= b2.min_y
        }

        // Bookkeeping: track which polylines from self/other participated
        let mut self_used_ccw = vec![false; self.ccw_plines.len()];
        let mut self_used_cw = vec![false; self.cw_plines.len()];
        let mut othr_used_ccw = vec![false; other.ccw_plines.len()];
        let mut othr_used_cw = vec![false; other.cw_plines.len()];

        // For each loop in self vs each loop in other, we run the boolean operation,
        // wrapping clockwise polylines in a PlineInversionView (instead of mutating them)

        // 1) ccw_plines vs ccw_plines
        for (i, ip) in self.ccw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            for (j, jp) in other.ccw_plines.iter().enumerate() {
                let b2 = match jp.spatial_index.bounds() {
                    Some(bb) => bb,
                    None => continue,
                };        
                if !boxes_overlap(&b1, &b2) {
                    continue;
                } else {
                    // bounding box says "maybe they intersect"
                
                    // Actually do the boolean op to confirm real intersection:
                    let result = ip.polyline.boolean(&jp.polyline, op);
                
                    // If the result is disjoint or empty, we consider them not used
                    if !matches!(result.result_info, BooleanResultInfo::Disjoint)
                        || !result.pos_plines.is_empty()
                        || !result.neg_plines.is_empty()
                    {
                        // If they do overlap, mark them used
                        self_used_ccw[i] = true;
                        othr_used_ccw[j] = true;
                        all_results.push(result);
                    }
                } 
            }
        }

        // 2) ccw_plines vs cw_plines
        for (i, ip) in self.ccw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };

            for (j, jp) in other.cw_plines.iter().enumerate() {
                let b2 = match jp.spatial_index.bounds() {
                    Some(bb) => bb,
                    None => continue,
                };

                if !boxes_overlap(&b1, &b2) {
                    continue;
                } else {
                    // bounding box says "maybe they intersect"
                
                    // Actually do the boolean op to confirm real intersection:
                    let jp_inverted = PlineInversionView::new(&jp.polyline);
                    let result = ip.polyline.boolean(&jp_inverted, op);
                
                    // If the result is disjoint or empty, we consider them not used
                    if !matches!(result.result_info, BooleanResultInfo::Disjoint)
                        || !result.pos_plines.is_empty()
                        || !result.neg_plines.is_empty()
                    {
                        // If they do overlap, mark them used
                        self_used_ccw[i] = true;
                        othr_used_cw[j] = true;
                        all_results.push(result);
                    }
                }
            }
        }

        // 3) cw_plines vs ccw_plines
        for (i, ip) in self.cw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            let ip_inverted = PlineInversionView::new(&ip.polyline);

            for (j, jp) in other.ccw_plines.iter().enumerate() {
                let b2 = match jp.spatial_index.bounds() {
                    Some(bb) => bb,
                    None => continue,
                };

                if !boxes_overlap(&b1, &b2) {
                    continue;
                } else {
                    // bounding box says "maybe they intersect"
                
                    // Actually do the boolean op to confirm real intersection:
                    let result = ip_inverted.boolean(&jp.polyline, op);
                
                    // If the result is disjoint or empty, we consider them not used
                    if !matches!(result.result_info, BooleanResultInfo::Disjoint)
                        || !result.pos_plines.is_empty()
                        || !result.neg_plines.is_empty()
                    {
                        // If they do overlap, mark them used
                        self_used_cw[i] = true;
                        othr_used_ccw[j] = true;
                        all_results.push(result);
                    }
                }
            }
        }

        // 4) cw_plines vs cw_plines
        for (i, ip) in self.cw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            let ip_inverted = PlineInversionView::new(&ip.polyline);

            for (j, jp) in other.cw_plines.iter().enumerate() {
                let b2 = match jp.spatial_index.bounds() {
                    Some(bb) => bb,
                    None => continue,
                };

                if !boxes_overlap(&b1, &b2) {
                    continue;
                } else {
                    // bounding box says "maybe they intersect"
                
                    // Actually do the boolean op to confirm real intersection:
                    let jp_inverted = PlineInversionView::new(&jp.polyline);
                    let result = ip_inverted.boolean(&jp_inverted, op);
                
                    // If the result is disjoint or empty, we consider them not used
                    if !matches!(result.result_info, BooleanResultInfo::Disjoint)
                        || !result.pos_plines.is_empty()
                        || !result.neg_plines.is_empty()
                    {
                        // If they do overlap, mark them used
                        self_used_cw[i] = true;
                        othr_used_cw[j] = true;
                        all_results.push(result);
                    }
                }
            }
        }

        // At this point, we have a bunch of BooleanResult<Pline<_>>.
        // We'll classify each returned polyline by its signed area
        // to figure out which are ccw (outer loops) vs cw (holes).
        let mut final_ccw = Vec::new();
        let mut final_cw = Vec::new();

        for boolean_result in all_results {
            // each BooleanResult can have pos_plines or neg_plines
            for rp in boolean_result.pos_plines {
                let area = rp.pline.area();
                if area < T::zero() {
                    // negative area → cw loop
                    final_cw.push(rp.pline);
                } else {
                    // zero or positive area → ccw loop
                    final_ccw.push(rp.pline);
                }
            }
            for rp in boolean_result.neg_plines {
                let area = rp.pline.area();
                if area < T::zero() {
                    final_cw.push(rp.pline);
                } else {
                    final_ccw.push(rp.pline);
                }
            }
        }

        // For union or XOR, also include any "unused" loops from self & other
        match op {
            BooleanOp::Or | BooleanOp::Xor => {
                // all self.ccw that never overlapped anything
                for (i, used) in self_used_ccw.iter().enumerate() {
                    if !used {
                        // Add it as-is (ccw stays ccw)
                        final_ccw.push(self.ccw_plines[i].polyline.clone());
                    }
                }
                // all self.cw that never overlapped anything
                for (i, used) in self_used_cw.iter().enumerate() {
                    if !used {
                        final_cw.push(self.cw_plines[i].polyline.clone());
                    }
                }
                // same for other
                for (j, used) in othr_used_ccw.iter().enumerate() {
                    if !used {
                        final_ccw.push(other.ccw_plines[j].polyline.clone());
                    }
                }
                for (j, used) in othr_used_cw.iter().enumerate() {
                    if !used {
                        final_cw.push(other.cw_plines[j].polyline.clone());
                    }
                }
            }
            BooleanOp::And => {
                // For intersection, leftover loops do not appear
                // ...
            }
            BooleanOp::Not => {
                // For difference: leftover loops from self remain,
                // leftover loops from other do NOT appear.
                for (i, used) in self_used_ccw.iter().enumerate() {
                    if !used {
                        final_ccw.push(self.ccw_plines[i].polyline.clone());
                    }
                }
                for (i, used) in self_used_cw.iter().enumerate() {
                    if !used {
                        final_cw.push(self.cw_plines[i].polyline.clone());
                    }
                }
            }
        }

        // 3) Optionally combine/merge identical or overlapping loops if necessary

        // 4) Build new shape from these final CCW / CW polylines:
        Shape::from_plines(final_ccw.into_iter().chain(final_cw.into_iter()))
    }

    /// Union
    pub fn union(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Or)
    }

    /// Intersection
    pub fn intersection(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::And)
    }

    /// Difference: shape1 minus shape2
    pub fn difference(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Not)
    }

    /// Symmetric difference (xor)
    pub fn xor(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Xor)
    }

    /// Translates all polylines by `(dx, dy)`.
    pub fn translate_mut(&mut self, dx: T, dy: T) {
        for loop_poly in &mut self.ccw_plines {
            loop_poly.polyline.translate_mut(dx, dy);
            // rebuild each loop's index
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            loop_poly.polyline.translate_mut(dx, dy);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        // rebuild the shape’s overall index
        self.plines_index = self.build_plines_index();
    }

    /// Scales all polylines by `scale_factor` about the origin `(0, 0)`.
    /// (You might want an overload that takes an `origin` parameter.)
    pub fn scale_mut(&mut self, scale_factor: T) {
        for loop_poly in &mut self.ccw_plines {
            loop_poly.polyline.scale_mut(scale_factor);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            loop_poly.polyline.scale_mut(scale_factor);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Rotates all polylines by `theta` radians about `(0, 0)`.
    /// (You might want an overload for a custom center.)
    pub fn rotate_mut(&mut self, theta: T) {
        // We treat rotation as: (x, y) -> (x cos θ – y sin θ, x sin θ + y cos θ)
        // Each polyline's `translate_mut` or manual iteration can do that.
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                let rx = px * cos_theta - py * sin_theta;
                let ry = px * sin_theta + py * cos_theta;
                v.x = rx;
                v.y = ry;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                let rx = px * cos_theta - py * sin_theta;
                let ry = px * sin_theta + py * cos_theta;
                v.x = rx;
                v.y = ry;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Mirrors (reflects) all polylines about the X-axis (example).
    /// For a more general "mirror across a line," you can parameterize the reflection formula.
    /// This example just flips `y -> -y`.
    pub fn mirror_x_mut(&mut self) {
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                // reflect y about the x-axis
                v.y = -v.y;
                // if you want to invert bulge as well, do so if it's an arc:
                // (since reflection can flip arc orientation)
                v.bulge = -v.bulge;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                v.y = -v.y;
                v.bulge = -v.bulge;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Re-centers the shape to place its bounding box center at the origin (0, 0).
    pub fn center_mut(&mut self) {
        if let Some(shape_aabb) = self.plines_index.bounds() {
            let cx = (shape_aabb.min_x + shape_aabb.max_x) / T::two();
            let cy = (shape_aabb.min_y + shape_aabb.max_y) / T::two();
            // Translate the shape so that bounding box center -> (0, 0)
            self.translate_mut(-cx, -cy);
        }
    }

    /// Applies a full 2D transform if you want a custom matrix approach.
    /// transform matrix is `[ [a, b], [c, d] ]` plus a translation `(tx, ty)`.
    /// the mapping is: (x, y) -> (a x + b y + tx, c x + d y + ty).
    pub fn transform_mut(
        &mut self,
        scale_x: T,
        scale_y: T,
        rot_cos: T,
        rot_sin: T,
        trans_x: T,
        trans_y: T,
    ) {
        // apply transform to polylines
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                v.x = scale_x * px + scale_y * py + trans_x;
                v.y = rot_cos * px + rot_sin * py + trans_y;
                // If arcs are used, you might need more logic to handle orientation, etc.
                // But for simple transform, bulge isn't changed, though reflection inverts sign, etc.
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                v.x = scale_x * px + scale_y * py + trans_x;
                v.y = rot_cos * px + rot_sin * py + trans_y;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }

        // now rebuild the shape's top-level index
        self.plines_index = self.build_plines_index();
    }

    /// Helper to rebuild the shape’s own bounding index after polylines are mutated.
    fn build_plines_index(&self) -> StaticAABB2DIndex<T> {
        let total_len = self.ccw_plines.len() + self.cw_plines.len();
        let mut builder = StaticAABB2DIndexBuilder::new(total_len);
        for loop_poly in &self.ccw_plines {
            let bounds = loop_poly
                .spatial_index
                .bounds()
                .expect("empty pline unexpected");
            builder.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
        }
        for loop_poly in &self.cw_plines {
            let bounds = loop_poly
                .spatial_index
                .bounds()
                .expect("empty pline unexpected");
            builder.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
        }
        builder.build().unwrap()
    }
}

impl<T: Real> ShapeTransformBuilder<T> {
    /// Creates a builder from an existing shape (consumes the shape).
    pub fn new(shape: Shape<T>) -> Self {
        // default no transformation
        ShapeTransformBuilder {
            shape,
            scale_factor: T::one(),
            rotate_angle: T::zero(),
            rotate_anchor: None,
            translate_x: T::zero(),
            translate_y: T::zero(),
            invert_direction: false,
        }
    }

    /// Apply a uniform scale factor (accumulated). If you want x/y separate,
    /// you'd keep scale_x and scale_y, etc.
    pub fn scale_mut(&mut self, factor: T) -> &mut Self {
        self.scale_factor = self.scale_factor * factor;
        self
    }

    /// Translate by (dx, dy).
    pub fn translate_mut(&mut self, dx: T, dy: T) -> &mut Self {
        self.translate_x = self.translate_x + dx;
        self.translate_y = self.translate_y + dy;
        self
    }

    /// Invert directions of all polylines once we apply transforms
    pub fn invert_direction_mut(&mut self) -> &mut Self {
        self.invert_direction = !self.invert_direction;
        self
    }

    /// Rotate about an anchor point with some angle in radians
    pub fn rotate_about_mut(&mut self, anchor: Vector2<T>, angle: T) -> &mut Self {
        // If you want multiple rotates about different anchors, you'd store them in a list
        // or accumulate them with the existing rotate anchor. We'll do the simple approach:
        self.rotate_angle = self.rotate_angle + angle;
        self.rotate_anchor = Some((anchor.x, anchor.y));
        self
    }

    /// Final build step: apply all transformations in the recorded order
    /// to each polyline, then rebuild the shape-level indexes once.
    pub fn build(mut self) -> Shape<T> {
        // apply transformations to all polylines
        // We'll do them in the order: invert => scale => rotate => translate
        // or the user might want a different order. Typically you'd define the order or let them specify.

        let do_invert = self.invert_direction;
        let s = self.scale_factor;
        let angle = self.rotate_angle;
        let (rc, rs) = (angle.cos(), angle.sin());
        let (tx, ty) = (self.translate_x, self.translate_y);

        let anchor = match self.rotate_anchor {
            None => (T::zero(), T::zero()),
            Some(a) => a,
        };

        // We'll do the transformations in that order for each polyline:
        // invert direction (bulges), scale, then rotate about anchor, then translate.

        // We'll update self.shape in place
        for ip in &mut self.shape.ccw_plines {
            let p = &mut ip.polyline;
            if do_invert {
                p.invert_direction_mut();
            }
            for i in 0..p.vertex_count() {
                let v = p.at(i);

                // invert bulge is done by invert_direction_mut above, so skip if that's the user approach
                let (mut x, mut y) = (v.x, v.y);

                // scale
                x = x * s;
                y = y * s;

                // rotate about anchor
                // translate so anchor is origin => rotate => move back
                x = x - anchor.0;
                y = y - anchor.1;
                let rx = x * rc - y * rs;
                let ry = x * rs + y * rc;
                x = rx + anchor.0;
                y = ry + anchor.1;

                // translate
                x = x + tx;
                y = y + ty;

                p.set(i, x, y, v.bulge);
            }
            // now rebuild polyline's own index
            ip.spatial_index = p.create_aabb_index();
        }
        for ip in &mut self.shape.cw_plines {
            let p = &mut ip.polyline;
            if do_invert {
                p.invert_direction_mut();
            }
            for i in 0..p.vertex_count() {
                let v = p.at(i);
                let (mut x, mut y) = (v.x, v.y);
                // scale
                x = x * s;
                y = y * s;
                // rotate about anchor
                x = x - anchor.0;
                y = y - anchor.1;
                let rx = x * rc - y * rs;
                let ry = x * rs + y * rc;
                x = rx + anchor.0;
                y = ry + anchor.1;
                // translate
                x = x + tx;
                y = y + ty;
                p.set(i, x, y, v.bulge);
            }
            ip.spatial_index = p.create_aabb_index();
        }

        // now rebuild shape's top-level index
        let mut builder =
            StaticAABB2DIndexBuilder::new(self.shape.ccw_plines.len() + self.shape.cw_plines.len());
        for ip in &self.shape.ccw_plines {
            if let Some(b) = ip.spatial_index.bounds() {
                builder.add(b.min_x, b.min_y, b.max_x, b.max_y);
            }
        }
        for ip in &self.shape.cw_plines {
            if let Some(b) = ip.spatial_index.bounds() {
                builder.add(b.min_x, b.min_y, b.max_x, b.max_y);
            }
        }
        self.shape.plines_index = builder.build().unwrap();

        // Return the transformed shape
        self.shape
    }
}

// intersects between two offset loops
#[derive(Debug, Clone)]
pub struct SlicePointSet<T> {
    loop_idx1: usize,
    loop_idx2: usize,
    slice_points: Vec<PlineBasicIntersect<T>>,
}

#[derive(Debug, Clone, Copy)]
struct DissectionPoint<T> {
    seg_idx: usize,
    pos: Vector2<T>,
}

#[derive(Debug, Clone, Copy)]
struct DissectedSlice<T> {
    source_idx: usize,
    v_data: PlineViewData<T>,
}
