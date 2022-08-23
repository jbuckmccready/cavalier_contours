use core::slice;
use std::{
    collections::{BTreeMap, BTreeSet},
    ops::Index,
};

use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder};

use crate::{
    core::{math::Vector2, traits::Real},
    polyline::{
        FindIntersectsOptions, PlineBasicIntersect, PlineIntersect, PlineOffsetOptions,
        PlineSelfIntersectOptions, PlineSource, PlineSourceMut, Polyline,
    },
};

pub struct OffsetLoop<T: Real> {
    pub parent_loop_idx: usize,
    pub indexed_pline: IndexedPolyline<T>,
}

pub struct ClosedPlineSet<T> {
    pub ccw_loops: Vec<Polyline<T>>,
    pub cw_loops: Vec<Polyline<T>>,
}

pub struct IndexedPolyline<T: Real> {
    pub polyline: Polyline<T>,
    pub spatial_index: StaticAABB2DIndex<T>,
}

impl<T> IndexedPolyline<T>
where
    T: Real,
{
    fn new(polyline: Polyline<T>) -> Option<Self> {
        let spatial_index = polyline.create_approx_aabb_index()?;
        Some(Self {
            polyline,
            spatial_index,
        })
    }

    fn parallel_offset(&self, offset: T) -> Vec<Polyline<T>> {
        let opts = PlineOffsetOptions {
            aabb_index: Some(&self.spatial_index),
            handle_self_intersects: false,
            ..Default::default()
        };

        self.polyline.parallel_offset_opt(offset, &opts)
    }
}
pub struct Shape<T: Real> {
    pub ccw_plines: Vec<IndexedPolyline<T>>,
    pub cw_plines: Vec<IndexedPolyline<T>>,
    pub plines_index: StaticAABB2DIndex<T>,
}

impl<T> Shape<T>
where
    T: Real,
{
    fn get_loop<'a>(
        i: usize,
        s1: &'a [OffsetLoop<T>],
        s2: &'a [OffsetLoop<T>],
    ) -> &'a OffsetLoop<T> {
        if i < s1.len() {
            &s1[i]
        } else {
            &s2[i]
        }
    }
    fn parallel_offset(&self, offset: T) -> Option<Self> {
        // generate offset loops
        let mut ccw_offset_loops = Vec::new();
        let mut cw_offset_loops = Vec::new();
        let mut parent_idx = 0;
        for pline in self.ccw_plines.iter() {
            for offset_pline in pline.parallel_offset(offset) {
                // must check if orientation inverted (due to collapse of very narrow or small input)
                if offset_pline.area() < T::zero() {
                    continue;
                }

                let ccw_offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline)
                        .expect("failed to offset shape polyline"),
                };
                ccw_offset_loops.push(ccw_offset_loop);
            }

            parent_idx += 1;
        }

        for pline in self.cw_plines.iter() {
            for offset_pline in pline.parallel_offset(offset) {
                let area = offset_pline.area();
                let offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline)
                        .expect("failed to offset shape polyline"),
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
            return None;
        }

        // build spatial index of offset loop approximate bounding boxes
        let offset_loops_index = {
            let mut b =
                StaticAABB2DIndexBuilder::new(ccw_offset_loops.len() + cw_offset_loops.len());
            for index in ccw_offset_loops
                .iter()
                .map(|p| &p.indexed_pline.spatial_index)
            {
                b.add(index.min_x(), index.min_y(), index.max_x(), index.max_y());
            }

            for index in cw_offset_loops
                .iter()
                .map(|p| &p.indexed_pline.spatial_index)
            {
                b.add(index.min_x(), index.min_y(), index.max_x(), index.max_y());
            }

            b.build()
                .expect("failed to build spatial index of offset loop bounds")
        };

        // find all intersects between all offsets to create slice points
        let mut slice_point_sets = Vec::new();
        let mut slice_points_lookup = BTreeMap::<usize, Vec<usize>>::new();
        let mut visited_loop_pairs = BTreeSet::<(usize, usize)>::new();
        let mut query_stack = Vec::new();

        for i in 0..offset_loop_count {
            let loop1 = Self::get_loop(i, &ccw_offset_loops, &cw_offset_loops);
            let index1 = &loop1.indexed_pline.spatial_index;
            let query_results = offset_loops_index.query_with_stack(
                index1.min_x(),
                index1.min_y(),
                index1.max_x(),
                index1.max_y(),
                &mut query_stack,
            );

            for j in query_results {
                if i == j {
                    // skip same index (no self intersects among the offset loops)
                    continue;
                }

                if visited_loop_pairs.contains(&(j, i)) {
                    // skip reversed index order (would end up comparing the same loops)
                    continue;
                }

                visited_loop_pairs.insert((i, j));

                let loop2 = Self::get_loop(j, &ccw_offset_loops, &cw_offset_loops);

                let intrs_opts = FindIntersectsOptions {
                    pline1_aabb_index: Some(index1),
                    ..Default::default()
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
        // remove slices which fail distance check
        // stitch slices together

        for i in 0..offset_loop_count {
            if let Some(slice_point_set) = slice_points_lookup.get(&i) {
                // create slices from loop using slice points
            } else {
                // no intersects but still must test distance of one vertex position since it may be
                // inside another offset (completely eclipsed by island offset)
            }
        }

        todo!()
    }
}

// fn stitch_slices_into_closed_polylines<

struct SlicePointSet<T> {
    loop_idx1: usize,
    loop_idx2: usize,
    slice_points: Vec<PlineBasicIntersect<T>>,
}

struct DissectionPoint<T> {
    other_loop_idx: usize,
    pos: Vector2<T>,
}

struct DissectedSlice<T> {
    polyline: Polyline<T>,
    slice_parent_idx: usize,
    start_loop_idx: usize,
    end_loop_idx: usize,
}

// fn create_offset_loops<T>(input_set: &ClosedPlineSet<T>, abs_offset: T)
// where
//     T: Real,
// {
//     let mut result = ClosedPlineSet {
//         ccw_loops: Vec::new(),
//         cw_loops: Vec::new(),
//     };

//     let mut parent_idx = 0;
//     for pline in input_set.ccw_loops.iter() {
//         for offset_pline in pline.parallel_offset(abs_offset) {
//             // must check if orientation inverted (due to collapse of very narrow or small input)
//             if offset_pline.area() < T::zero() {
//                 continue;
//             }

//             let spatial_index = offset_pline.create_approx_aabb_index();
//         }
//     }

//     result
// }
