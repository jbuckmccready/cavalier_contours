use std::{
    hint::black_box,
    time::{Duration, Instant},
};

use cavalier_contours::{
    polyline::{BooleanOp, PlineSourceMut, Polyline},
    shape_algorithms::{Shape, ShapeView},
};

fn create_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(xmin, ymin, 0.0);
    pline.add(xmax, ymin, 0.0);
    pline.add(xmax, ymax, 0.0);
    pline.add(xmin, ymax, 0.0);
    pline
}

fn create_cw_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pline = create_rectangle(xmin, ymin, xmax, ymax);
    pline.invert_direction_mut();
    pline
}

fn shape_plines(dx: f64, dy: f64) -> Vec<Polyline<f64>> {
    let mut plines = Vec::new();
    for row in 0..4 {
        for col in 0..5 {
            let x = dx + col as f64 * 18.0;
            let y = dy + row as f64 * 18.0;
            plines.push(create_rectangle(x, y, x + 14.0, y + 14.0));
            if (row + col) % 2 == 0 {
                plines.push(create_cw_rectangle(x + 4.0, y + 4.0, x + 9.0, y + 9.0));
            }
        }
    }
    plines
}

fn solid_shape_plines(dx: f64, dy: f64) -> Vec<Polyline<f64>> {
    let mut plines = Vec::new();
    for row in 0..4 {
        for col in 0..5 {
            let x = dx + col as f64 * 18.0;
            let y = dy + row as f64 * 18.0;
            plines.push(create_rectangle(x, y, x + 14.0, y + 14.0));
        }
    }
    plines
}

fn report(name: &str, iterations: usize, elapsed: Duration) {
    let micros = elapsed.as_secs_f64() * 1_000_000.0 / iterations as f64;
    println!("{name:44} {micros:10.3} us/iter ({iterations} iters)");
}

fn bench<R>(name: &str, iterations: usize, mut f: impl FnMut() -> R) {
    for _ in 0..10 {
        black_box(f());
    }

    let start = Instant::now();
    for _ in 0..iterations {
        black_box(f());
    }
    report(name, iterations, start.elapsed());
}

fn main() {
    let a_plines = shape_plines(0.0, 0.0);
    let b_plines = shape_plines(6.0, 5.0);
    let solid_a_plines = solid_shape_plines(0.0, 0.0);
    let solid_b_plines = solid_shape_plines(6.0, 5.0);
    let shell_a_plines = vec![create_rectangle(0.0, 0.0, 100.0, 100.0)];
    let shell_b_plines = vec![create_rectangle(25.0, -15.0, 125.0, 75.0)];
    let owned_a = Shape::from_plines(a_plines.clone());
    let owned_b = Shape::from_plines(b_plines.clone());
    let view_a = ShapeView::from_plines(a_plines.iter());
    let view_b = ShapeView::from_plines(b_plines.iter());
    let solid_owned_a = Shape::from_plines(solid_a_plines.clone());
    let solid_owned_b = Shape::from_plines(solid_b_plines.clone());
    let solid_view_a = ShapeView::from_plines(solid_a_plines.iter());
    let solid_view_b = ShapeView::from_plines(solid_b_plines.iter());
    let shell_owned_a = Shape::from_plines(shell_a_plines.clone());
    let shell_owned_b = Shape::from_plines(shell_b_plines.clone());
    let shell_view_a = ShapeView::from_plines(shell_a_plines.iter());
    let shell_view_b = ShapeView::from_plines(shell_b_plines.iter());

    println!("shape borrowed API cost model");
    println!("--------------------------------");
    bench("owned construct from cloned Vec", 1_000, || {
        Shape::from_plines(black_box(a_plines.clone()))
    });
    bench("borrowed view construct from refs", 1_000, || {
        ShapeView::from_plines(black_box(&a_plines).iter())
    });
    bench("borrowed view materialize owned shape", 1_000, || {
        black_box(&view_a).to_owned_shape()
    });
    bench("owned reused-shape OR", 100, || {
        black_box(&owned_a).boolean(black_box(&owned_b), BooleanOp::Or)
    });
    bench("borrowed reused-view OR", 100, || {
        black_box(&view_a).boolean(black_box(&view_b), BooleanOp::Or)
    });
    bench("owned reused-shape solid OR", 100, || {
        black_box(&solid_owned_a).boolean(black_box(&solid_owned_b), BooleanOp::Or)
    });
    bench("borrowed reused-view solid OR", 100, || {
        black_box(&solid_view_a).boolean(black_box(&solid_view_b), BooleanOp::Or)
    });
    bench("owned reused-shape shell AND", 1_000, || {
        black_box(&shell_owned_a).boolean(black_box(&shell_owned_b), BooleanOp::And)
    });
    bench("borrowed reused-view shell AND", 1_000, || {
        black_box(&shell_view_a).boolean(black_box(&shell_view_b), BooleanOp::And)
    });
    bench("one-shot owned construct plus OR", 100, || {
        let a = Shape::from_plines(black_box(a_plines.clone()));
        let b = Shape::from_plines(black_box(b_plines.clone()));
        a.boolean(&b, BooleanOp::Or)
    });
    bench("one-shot borrowed construct plus OR", 100, || {
        let a = ShapeView::from_plines(black_box(&a_plines).iter());
        let b = ShapeView::from_plines(black_box(&b_plines).iter());
        a.boolean(&b, BooleanOp::Or)
    });
}
