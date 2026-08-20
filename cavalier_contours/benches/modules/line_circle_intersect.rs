use std::hint::black_box;

use cavalier_contours::core::math::{Vector2, line_circle_intr};
use criterion::{BenchmarkId, Criterion};

pub fn line_circle_intr_group(c: &mut Criterion) {
    let near_tangent_center = |radius: f64, half_spacing: f64| {
        Vector2::new(0.0, (radius * radius - half_spacing * half_spacing).sqrt())
    };
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
            "point_segment",
            Vector2::new(0.0, 100.001),
            Vector2::new(0.0, 100.001),
            100.0,
            Vector2::new(0.0, 0.0),
        ),
    ];

    let mut group = c.benchmark_group("line_circle_intr");
    for (name, p0, p1, radius, center) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(p0, p1, radius, center),
            |b, &(p0, p1, radius, center)| {
                b.iter(|| {
                    black_box(line_circle_intr(
                        black_box(p0),
                        black_box(p1),
                        black_box(radius),
                        black_box(center),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}
