use std::hint::black_box;

use cavalier_contours::polyline::{FindIntersectsOptions, PlineSource, PlineSourceMut, Polyline};
use criterion::{BenchmarkGroup, Criterion, measurement::WallTime};

fn minimal_duplicate_intersects() -> (Polyline<f64>, Polyline<f64>) {
    // The shared segment records an overlap, then the diverging segments report its endpoint as a
    // basic intersect that duplicate cleanup removes.
    let mut pline1 = Polyline::new();
    pline1.add(0.0, 0.0, 0.0);
    pline1.add(1.0, 0.0, 0.0);
    pline1.add(2.0, 1.0, 0.0);

    let mut pline2 = Polyline::new();
    pline2.add(0.0, 0.0, 0.0);
    pline2.add(1.0, 0.0, 0.0);
    pline2.add(2.0, -1.0, 0.0);

    (pline1, pline2)
}

fn duplicate_intersect_chain(overlap_count: usize) -> (Polyline<f64>, Polyline<f64>) {
    // Alternate shared segments with diverging upper and lower paths. Each divergence reports one
    // basic intersect at the end of an overlap, and the final shared segment prevents an unrelated
    // basic intersect at the open endpoint.
    let mut pline1 = Polyline::new();
    let mut pline2 = Polyline::new();
    pline1.reserve(3 * overlap_count + 2);
    pline2.reserve(3 * overlap_count + 2);
    pline1.add(0.0, 0.0, 0.0);
    pline2.add(0.0, 0.0, 0.0);

    let mut x = 0.0;
    for _ in 0..overlap_count {
        pline1.add(x + 1.0, 0.0, 0.0);
        pline2.add(x + 1.0, 0.0, 0.0);
        pline1.add(x + 2.0, 1.0, 0.0);
        pline2.add(x + 2.0, -1.0, 0.0);
        pline1.add(x + 3.0, 0.0, 0.0);
        pline2.add(x + 3.0, 0.0, 0.0);
        x += 3.0;
    }
    pline1.add(x + 1.0, 0.0, 0.0);
    pline2.add(x + 1.0, 0.0, 0.0);

    (pline1, pline2)
}

pub fn polyline_find_intersects_duplicates_group(c: &mut Criterion) {
    const HEAVY_OVERLAP_COUNT: usize = 1024;

    fn add_benchmark(
        group: &mut BenchmarkGroup<'_, WallTime>,
        name: &str,
        pline1: &Polyline<f64>,
        pline2: &Polyline<f64>,
        expected_overlap_count: usize,
    ) {
        let pline1_aabb_index = pline1.create_approx_aabb_index();
        let options = FindIntersectsOptions {
            pline1_aabb_index: Some(&pline1_aabb_index),
            ..Default::default()
        };
        let intersects = pline1.find_intersects_opt(pline2, &options);
        assert!(intersects.basic_intersects.is_empty());
        assert_eq!(
            intersects.overlapping_intersects.len(),
            expected_overlap_count
        );

        group.bench_function(name, |b| {
            b.iter(|| {
                black_box(
                    black_box(pline1).find_intersects_opt(black_box(pline2), black_box(&options)),
                )
            });
        });
    }

    let mut group = c.benchmark_group("polyline_find_intersects_duplicates");
    let (minimal1, minimal2) = minimal_duplicate_intersects();
    add_benchmark(&mut group, "minimal_overlap", &minimal1, &minimal2, 1);

    let (heavy1, heavy2) = duplicate_intersect_chain(HEAVY_OVERLAP_COUNT);
    add_benchmark(
        &mut group,
        "heavy_overlap",
        &heavy1,
        &heavy2,
        HEAVY_OVERLAP_COUNT + 1,
    );
    group.finish();
}
