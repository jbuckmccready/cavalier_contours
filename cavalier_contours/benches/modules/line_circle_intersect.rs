use std::hint::black_box;

use cavalier_contours::core::math::{Circle, Line, Vector2, intersect_line_circle};
use criterion::{BenchmarkId, Criterion};

pub fn intersect_line_circle_group(c: &mut Criterion) {
    let near_tangent_center = |radius: f64, half_spacing: f64| {
        Vector2::new(0.0, (radius * radius - half_spacing * half_spacing).sqrt())
    };
    let epsilon = 1e-5;
    let half_epsilon = epsilon / 2.0;
    let endpoint_merge_half_spacing = 0.75 * epsilon;
    let cases = [
        (
            "oblique_miss",
            Vector2::new(-1.0, -1.0),
            Vector2::new(1.0, 1.0),
            0.5,
            Vector2::new(0.0, 5.0),
        ),
        (
            "oblique_two",
            Vector2::new(-4.0, -2.0),
            Vector2::new(5.0, 3.0),
            3.0,
            Vector2::new(0.5, 0.5),
        ),
        (
            "vertical_two",
            Vector2::new(0.0, -2.0),
            Vector2::new(0.0, 2.0),
            1.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "exact_tangent",
            Vector2::new(-1.0, 0.0),
            Vector2::new(1.0, 0.0),
            1.0,
            Vector2::new(0.0, -1.0),
        ),
        (
            "near_tangent_two",
            Vector2::new(-1.0, 0.0),
            Vector2::new(1.0, 0.0),
            100.0,
            near_tangent_center(100.0, 5e-4),
        ),
        (
            "near_tangent_merged",
            Vector2::new(-1.0, 0.0),
            Vector2::new(1.0, 0.0),
            100.0,
            near_tangent_center(100.0, 2e-6),
        ),
        (
            "endpoint_snap",
            Vector2::new(1.0 + half_epsilon, 0.0),
            Vector2::new(2.0, 0.0),
            1.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "endpoint_coalesced",
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            1.0,
            Vector2::new(
                -0.15 * epsilon,
                (1.0 - endpoint_merge_half_spacing * endpoint_merge_half_spacing).sqrt(),
            ),
        ),
        (
            "recovered_tangent",
            Vector2::new(-2.0, 0.0),
            Vector2::new(2.0, 0.0),
            1.0,
            Vector2::new(0.0, 1.0 + half_epsilon),
        ),
        (
            "recovered_tangent_endpoint_snap",
            Vector2::new(half_epsilon, 0.0),
            Vector2::new(2.0, 0.0),
            1.0,
            Vector2::new(0.0, 1.0 + half_epsilon),
        ),
        (
            "point_segment_hit",
            Vector2::new(0.0, 100.0 + half_epsilon),
            Vector2::new(0.0, 100.0 + half_epsilon),
            100.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "point_segment_miss",
            Vector2::new(0.0, 100.001),
            Vector2::new(0.0, 100.001),
            100.0,
            Vector2::new(0.0, 0.0),
        ),
    ];

    let mut group = c.benchmark_group("intersect_line_circle");
    for (name, p0, p1, radius, center) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(p0, p1, radius, center),
            |b, &(p0, p1, radius, center)| {
                b.iter(|| {
                    black_box(intersect_line_circle(
                        black_box(Line {
                            point: p0,
                            direction: p1 - p0,
                        }),
                        black_box(Circle { radius, center }),
                        black_box(epsilon),
                    ))
                });
            },
        );
    }
    group.finish();
}
