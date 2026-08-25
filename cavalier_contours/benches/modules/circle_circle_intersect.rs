use std::hint::black_box;

use cavalier_contours::core::math::{Vector2, circle_circle_intr};
use criterion::{BenchmarkId, Criterion};

pub fn circle_circle_intr_group(c: &mut Criterion) {
    let epsilon = 1e-5;
    let half_epsilon = epsilon / 2.0;
    let near_tangent_half_spacing = 2e-6f64;
    let cases = [
        (
            "overlapping",
            1.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "concentric_no_intersect",
            2.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "external_no_intersect",
            1.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(1.9, 0.8),
        ),
        (
            "internal_no_intersect",
            4.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(1.0, 0.25),
        ),
        (
            "external_tangent",
            1.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(2.0, 0.0),
        ),
        (
            "internal_tangent",
            3.0,
            Vector2::new(0.0, 1.0),
            4.0,
            Vector2::new(0.0, 0.0),
        ),
        (
            "recovered_external_tangent",
            1.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(2.0 + half_epsilon, 0.0),
        ),
        (
            "recovered_internal_tangent",
            2.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(1.0 - half_epsilon, 0.0),
        ),
        (
            "coalesced_intersections",
            1.0,
            Vector2::new(0.0, 0.0),
            1.0,
            Vector2::new(
                2.0 * (1.0 - near_tangent_half_spacing * near_tangent_half_spacing).sqrt(),
                0.0,
            ),
        ),
        (
            "two_intersects",
            3.0,
            Vector2::new(0.0, 1.0),
            4.0,
            Vector2::new(5.0, 5.0),
        ),
    ];

    let mut group = c.benchmark_group("circle_circle_intr");
    for (name, radius1, center1, radius2, center2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(radius1, center1, radius2, center2),
            |b, &(radius1, center1, radius2, center2)| {
                b.iter(|| {
                    black_box(circle_circle_intr(
                        black_box(radius1),
                        black_box(center1),
                        black_box(radius2),
                        black_box(center2),
                        black_box(epsilon),
                    ))
                });
            },
        );
    }
    group.finish();
}
