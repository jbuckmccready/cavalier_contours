use cavalier_contours::{core_math::point_on_circle, Polyline, Real, Vector2};

pub fn pathological1<T>(vertex_count: usize) -> Polyline<T>
where
    T: Real,
{
    let radius = T::from(40.0).unwrap();
    let center = Vector2::zero();

    let mut result = Polyline::new_closed();

    for i in 0..vertex_count {
        let angle = T::from(i).unwrap() * T::tau() / T::from(vertex_count).unwrap();
        let point = point_on_circle(radius, center, angle);
        let bulge = if i % 2 == 0 { T::one() } else { -T::one() };
        result.add(point.x, point.y, bulge);
    }

    result
}
