use std::hint::black_box;

use cavalier_contours::core::math::{Vector2, line_line_intr};
use criterion::{BenchmarkId, Criterion};

pub fn line_line_intr_group(c: &mut Criterion) {
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
    ];

    let mut group = c.benchmark_group("line_line_intr");
    for (name, v1, v2, u1, u2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, u1, u2),
            |b, &(v1, v2, u1, u2)| {
                b.iter(|| {
                    black_box(line_line_intr(
                        black_box(v1),
                        black_box(v2),
                        black_box(u1),
                        black_box(u2),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}
