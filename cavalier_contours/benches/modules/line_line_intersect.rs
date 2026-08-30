use std::hint::black_box;

use cavalier_contours::{
    core::math::Vector2,
    polyline::{LineSegment, intersect_line_segments},
};
use criterion::{BenchmarkId, Criterion};

pub fn intersect_line_segments_group(c: &mut Criterion) {
    let cases = [
        (
            "true_interior",
            Vector2::new(-1.0, -1.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(-1.0, 1.0),
            Vector2::new(1.0, -1.0),
        ),
        (
            "false_intersect",
            Vector2::new(-1.0, -1.0),
            Vector2::new(-0.5, -0.5),
            Vector2::new(-1.0, 1.0),
            Vector2::new(1.0, -1.0),
        ),
        (
            "parallel",
            Vector2::new(-2.0, -1.0),
            Vector2::new(2.0, -1.0),
            Vector2::new(-1.0, 5.0),
            Vector2::new(1.0, 5.0),
        ),
        (
            "collinear_disjoint",
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(3.0, 0.0),
        ),
        (
            "collinear_touch",
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 0.0),
        ),
        (
            "collinear_overlap",
            Vector2::new(-1.0, -1.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(0.5, 0.5),
        ),
        (
            "point_on_segment",
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(-1.0, 0.0),
            Vector2::new(1.0, 0.0),
        ),
        (
            "same_points",
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
        ),
        (
            "fuzzy_points",
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(5e-6, 0.0),
            Vector2::new(5e-6, 0.0),
        ),
        (
            "double_endpoint_snap",
            Vector2::new(-1.0, 0.0),
            Vector2::new(0.0, 0.0),
            Vector2::new(2.5e-6, 1.0000025),
            Vector2::new(2.5e-6, 2.5e-6),
        ),
    ];

    let mut group = c.benchmark_group("intersect_line_segments");
    for (name, v1, v2, u1, u2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, u1, u2),
            |b, &(v1, v2, u1, u2)| {
                b.iter(|| {
                    black_box(intersect_line_segments(
                        black_box(LineSegment { start: v1, end: v2 }),
                        black_box(LineSegment { start: u1, end: u2 }),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}
