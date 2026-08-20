use criterion::{criterion_group, criterion_main};

#[path = "modules/base_math.rs"]
mod base_math;
#[path = "modules/circle_circle_intersect.rs"]
mod circle_circle_intersect;
#[path = "modules/line_circle_intersect.rs"]
mod line_circle_intersect;
#[path = "modules/line_line_intersect.rs"]
mod line_line_intersect;
#[path = "modules/pline.rs"]
mod pline;
#[path = "modules/pline_intersects.rs"]
mod pline_intersects;
#[path = "modules/pline_offset.rs"]
mod pline_offset;
#[path = "modules/pline_seg.rs"]
mod pline_seg;
#[path = "modules/pline_seg_intersect.rs"]
mod pline_seg_intersect;
mod test_polylines;

use base_math::{delta_angle_group, point_within_arc_sweep_group};
use circle_circle_intersect::circle_circle_intr_group;
use line_circle_intersect::line_circle_intr_group;
use line_line_intersect::line_line_intr_group;
use pline::{polyline_area_group, polyline_winding_number_group};
use pline_intersects::polyline_find_intersects_duplicates_group;
use pline_offset::{
    polyline_offset_group, polyline_offset_topology_scaling_group, raw_offset_creation_group,
    raw_offset_round_join_group,
};
use pline_seg::{
    dist_from_segment_start_group, seg_arc_radius_and_center_group, seg_bounding_box_group,
    seg_closest_point_group, seg_distance_is_greater_than_group,
    seg_fast_approx_bounding_box_group, seg_length_group, seg_midpoint_group,
    seg_split_at_point_group, seg_tangent_vector_group,
};
use pline_seg_intersect::pline_seg_intr_group;

criterion_group!(
    benches,
    line_line_intr_group,
    line_circle_intr_group,
    circle_circle_intr_group,
    point_within_arc_sweep_group,
    delta_angle_group,
    pline_seg_intr_group,
    polyline_area_group,
    polyline_winding_number_group,
    seg_midpoint_group,
    seg_arc_radius_and_center_group,
    seg_tangent_vector_group,
    seg_fast_approx_bounding_box_group,
    seg_bounding_box_group,
    seg_closest_point_group,
    seg_distance_is_greater_than_group,
    seg_split_at_point_group,
    seg_length_group,
    dist_from_segment_start_group,
    raw_offset_round_join_group,
    raw_offset_creation_group,
    polyline_offset_group,
    polyline_offset_topology_scaling_group,
    polyline_find_intersects_duplicates_group,
);
criterion_main!(benches);
