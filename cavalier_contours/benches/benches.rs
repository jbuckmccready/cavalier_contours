use std::hint::black_box;

use cavalier_contours::{
    core::math::Vector2,
    polyline::{
        PlineSource, PlineSourceMut, PlineVertex, Polyline,
        internal::raw_pline_offset::create_raw_offset, seg_length, seg_midpoint,
        seg_split_at_point,
    },
};
use criterion::{
    Bencher, BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main,
    measurement::WallTime,
};

mod test_polylines;

use test_polylines::{
    bezier_enclosure, floor_plan, involute_gear, involute_gear_with_arcs, mechanical_bracket,
    pathological1, pathological1_no_arcs, profile1, profile1_no_arcs, profile2, profile2_no_arcs,
    road_centerline, tapered_link_strip,
};

fn bench_polyline_area(b: &mut Bencher<'_>, polyline: &Polyline<f64>) {
    b.iter(|| polyline.area());
}

fn polyline_area_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_area");
    let vertex_counts = &[25, 250, 2500, 25000];
    for &i in vertex_counts {
        group.bench_with_input(BenchmarkId::new("pathological_area", i), &i, |b, i| {
            bench_polyline_area(b, &pathological1(*i));
        });
    }

    group.finish();
}

fn regular_polygon(vertex_count: u32, bulge: f64) -> Polyline<f64> {
    let mut result = Polyline::new_closed();
    result.reserve(usize::try_from(vertex_count).unwrap());
    for i in 0..vertex_count {
        let angle = f64::from(i) * std::f64::consts::TAU / f64::from(vertex_count);
        let (sin, cos) = angle.sin_cos();
        result.add(40.0 * cos, 40.0 * sin, bulge);
    }
    result
}

fn invalid_line_zigzag(vertex_count: u32) -> Polyline<f64> {
    let mut result = Polyline::new();
    result.reserve(usize::try_from(vertex_count).unwrap());
    for i in 0..vertex_count {
        result.add(f64::from(i), f64::from(i % 2), 0.0);
    }
    result
}

fn invalid_line_arc_zigzag(vertex_count: u32) -> Polyline<f64> {
    let mut result = Polyline::new();
    result.reserve(usize::try_from(vertex_count).unwrap());
    for i in 0..vertex_count {
        let bulge = if i % 2 == 0 { 0.0 } else { 0.5 };
        result.add(f64::from(i), f64::from(i % 2), bulge);
    }
    result
}

fn closed_invalid_runs(vertex_count: u32) -> Polyline<f64> {
    let mut result = Polyline::new_closed();
    result.reserve(usize::try_from(vertex_count).unwrap());
    for i in 0..vertex_count {
        let angle = f64::from(i) * std::f64::consts::TAU / f64::from(vertex_count);
        let radius = if i % 4 == 0 { 4.0 } else { 20.0 };
        let (sin, cos) = angle.sin_cos();
        result.add(radius * cos, radius * sin, 0.0);
    }
    result
}

fn polyline_winding_number_group(c: &mut Criterion) {
    const VERTEX_COUNT: u32 = 1000;
    let point = Vector2::new(0.0, 0.0);
    let lines = regular_polygon(VERTEX_COUNT, 0.0);
    let circle_arcs = regular_polygon(
        VERTEX_COUNT,
        (std::f64::consts::PI / (2.0 * f64::from(VERTEX_COUNT))).tan(),
    );

    // Each arc chord crosses the horizontal ray from `point` and takes the point-in-circle path.
    let mut crossing_arcs = Polyline::new_closed();
    crossing_arcs.reserve(usize::try_from(VERTEX_COUNT).unwrap());
    for i in 0..VERTEX_COUNT {
        if i % 2 == 0 {
            crossing_arcs.add(10.0, -1.0, -1.0);
        } else {
            crossing_arcs.add(10.0, 1.0, 1.0);
        }
    }

    let mut group = c.benchmark_group("polyline_winding_number");
    group.bench_function("lines", |b| {
        b.iter(|| black_box(lines.winding_number(black_box(point))));
    });
    group.bench_function("circle_arcs", |b| {
        b.iter(|| black_box(circle_arcs.winding_number(black_box(point))));
    });
    group.bench_function("arc_distance_checks", |b| {
        b.iter(|| black_box(crossing_arcs.winding_number(black_box(point))));
    });
    group.finish();
}

fn seg_midpoint_group(c: &mut Criterion) {
    let cases = [
        (
            "line",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "shallow_arc",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "quarter_circle",
            (
                PlineVertex::new(2.0, 2.0, std::f64::consts::FRAC_PI_8.tan()),
                PlineVertex::new(4.0, 4.0, 0.0),
            ),
        ),
        (
            "clockwise_semicircle",
            (
                PlineVertex::new(-123.5, 47.25, -1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_midpoint");
    for (name, vertices) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &vertices,
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_midpoint(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

fn seg_split_at_point_group(c: &mut Criterion) {
    let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
    let near_start_angle = std::f64::consts::PI * 1e-6;
    let cases = [
        (
            "line_interior",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
                Vector2::new(384.125, -127.625),
            ),
        ),
        (
            "arc_start",
            (
                PlineVertex::new(0.0, 0.0, 1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(0.0, 0.0),
            ),
        ),
        (
            "shallow_arc_midpoint",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
                Vector2::new(384.1249825125, -127.6250507625),
            ),
        ),
        (
            "quarter_circle_midpoint",
            (
                PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
                PlineVertex::new(0.0, 1.0, 0.0),
                Vector2::new(
                    std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ),
            ),
        ),
        (
            "semicircle_near_start",
            (
                PlineVertex::new(0.0, 0.0, 1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(1.0 - near_start_angle.cos(), -near_start_angle.sin()),
            ),
        ),
        (
            "clockwise_semicircle_midpoint",
            (
                PlineVertex::new(0.0, 0.0, -1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(1.0, 1.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_split_at_point");
    for (name, input) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &input,
            |b, &(v1, v2, point)| {
                b.iter(|| {
                    black_box(seg_split_at_point(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                        black_box(1e-12),
                    ))
                });
            },
        );
    }
    group.finish();
}

fn seg_length_group(c: &mut Criterion) {
    let cases = [
        (
            "zero_length",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(-123.5, 47.25, 0.0),
            ),
        ),
        (
            "line",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "shallow_arc",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "quarter_circle",
            (
                PlineVertex::new(1.0, 0.0, std::f64::consts::FRAC_PI_8.tan()),
                PlineVertex::new(0.0, 1.0, 0.0),
            ),
        ),
        (
            "semicircle",
            (
                PlineVertex::new(-123.5, 47.25, 1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "clockwise_semicircle",
            (
                PlineVertex::new(-123.5, 47.25, -1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_length");
    for (name, vertices) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &vertices,
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_length(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

fn raw_offset_round_join_group(c: &mut Criterion) {
    let open = |vertexes: &[(f64, f64, f64)]| {
        let mut polyline = Polyline::new();
        for &(x, y, bulge) in vertexes {
            polyline.add(x, y, bulge);
        }
        polyline
    };
    let cases = [
        (
            "line_line_outer",
            open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]),
            -0.2,
        ),
        (
            "line_line_shallow_outer",
            open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, -1e-4, 0.0)]),
            0.2,
        ),
        (
            "line_line_near_reversal",
            open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1e-6, 0.0)]),
            -0.2,
        ),
        (
            "line_line_reversal",
            open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]),
            0.2,
        ),
        (
            "line_arc_outer",
            open(&[(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)]),
            0.2,
        ),
        (
            "arc_line_outer",
            open(&[(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]),
            0.2,
        ),
        (
            "arc_arc_outer",
            open(&[(0.0, 0.0, 1.0), (1.0, 0.0, -1.0), (1.0, -1.0, 0.0)]),
            0.2,
        ),
    ];

    let mut group = c.benchmark_group("raw_offset_round_join");
    for (name, polyline, offset) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(polyline, offset),
            |b, (polyline, offset)| {
                b.iter(|| {
                    black_box(create_raw_offset::<_, _, Polyline<f64>>(
                        black_box(polyline),
                        black_box(*offset),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}

fn repeat_offsets(polyline: &Polyline<f64>, offset: f64, count: u32) {
    for i in 1..=count {
        let offset = f64::from(i) * offset;
        black_box(polyline.parallel_offset(black_box(offset)));
        black_box(polyline.parallel_offset(black_box(-offset)));
    }
}

fn repeat_raw_offsets(polyline: &Polyline<f64>, offset: f64, count: u32) {
    for i in 1..=count {
        let offset = f64::from(i) * offset;
        black_box(create_raw_offset::<_, _, Polyline<f64>>(
            polyline,
            black_box(offset),
            1e-5,
        ));
        black_box(create_raw_offset::<_, _, Polyline<f64>>(
            polyline,
            black_box(-offset),
            1e-5,
        ));
    }
}

fn add_offset_benchmarks(
    group: &mut BenchmarkGroup<'_, WallTime>,
    repeat: fn(&Polyline<f64>, f64, u32),
) {
    let profile1 = profile1();
    group.bench_function("profile1", |b| {
        b.iter(|| repeat(&profile1, 0.1, 40));
    });

    let profile2 = profile2();
    group.bench_function("profile2", |b| {
        b.iter(|| repeat(&profile2, 0.1, 40));
    });

    let profile1_no_arcs = profile1_no_arcs();
    group.bench_function("profile1_no_arcs", |b| {
        b.iter(|| repeat(&profile1_no_arcs, 0.1, 40));
    });

    let profile2_no_arcs = profile2_no_arcs();
    group.bench_function("profile2_no_arcs", |b| {
        b.iter(|| repeat(&profile2_no_arcs, 0.1, 40));
    });

    let floor_plan = floor_plan();
    group.bench_function("floor_plan", |b| {
        b.iter(|| repeat(&floor_plan, 0.25, 12));
    });

    let mechanical_bracket = mechanical_bracket();
    group.bench_function("mechanical_bracket", |b| {
        b.iter(|| repeat(&mechanical_bracket, 0.2, 10));
    });

    let road_centerline = road_centerline();
    group.bench_function("road_centerline", |b| {
        b.iter(|| repeat(&road_centerline, 0.5, 8));
    });

    let bezier_enclosure = bezier_enclosure();
    group.bench_function("bezier_enclosure", |b| {
        b.iter(|| repeat(&bezier_enclosure, 0.25, 8));
    });

    let involute_gear = involute_gear();
    group.bench_function("involute_gear", |b| {
        b.iter(|| repeat(&involute_gear, 0.2, 4));
    });

    let involute_gear_with_arcs = involute_gear_with_arcs();
    group.bench_function("involute_gear_with_arcs", |b| {
        b.iter(|| repeat(&involute_gear_with_arcs, 0.2, 4));
    });

    let pathological1 = pathological1(100);
    group.bench_function("pathological1", |b| {
        b.iter(|| repeat(&pathological1, 1.0, 30));
    });

    let pathological1_no_arcs = pathological1_no_arcs(100);
    group.bench_function("pathological1_no_arcs", |b| {
        b.iter(|| repeat(&pathological1_no_arcs, 1.0, 30));
    });

    let invalid_line_zigzag = invalid_line_zigzag(200);
    group.bench_function("invalid_line_zigzag", |b| {
        b.iter(|| repeat(&invalid_line_zigzag, 5.0, 3));
    });

    let invalid_line_arc_zigzag = invalid_line_arc_zigzag(200);
    group.bench_function("invalid_line_arc_zigzag", |b| {
        b.iter(|| repeat(&invalid_line_arc_zigzag, 5.0, 3));
    });

    let closed_invalid_runs = closed_invalid_runs(200);
    group.bench_function("closed_invalid_runs", |b| {
        b.iter(|| repeat(&closed_invalid_runs, 3.0, 3));
    });
}

fn raw_offset_creation_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("raw_offset_creation");
    add_offset_benchmarks(&mut group, repeat_raw_offsets);
    group.finish();
}

fn polyline_offset_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_offset");
    add_offset_benchmarks(&mut group, repeat_offsets);
    group.finish();
}

fn polyline_offset_topology_scaling_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_offset_topology_scaling");
    for link_count in [128, 512, 2048, 4096] {
        let polyline = tapered_link_strip(link_count);
        assert_eq!(
            polyline.parallel_offset(3.0).len(),
            usize::try_from(link_count + 1).unwrap()
        );
        group.bench_with_input(
            BenchmarkId::new("tapered_link_strip", link_count),
            &polyline,
            |b, polyline| {
                b.iter(|| black_box(polyline.parallel_offset(black_box(3.0))));
            },
        );
    }
    group.finish();
}

criterion_group!(
    benches,
    polyline_area_group,
    polyline_winding_number_group,
    seg_midpoint_group,
    seg_split_at_point_group,
    seg_length_group,
    raw_offset_round_join_group,
    raw_offset_creation_group,
    polyline_offset_group,
    polyline_offset_topology_scaling_group,
);
criterion_main!(benches);
