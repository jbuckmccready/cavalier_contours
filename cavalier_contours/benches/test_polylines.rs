use cavalier_contours::{
    core::{
        math::{Vector2, point_on_circle},
        traits::Real,
    },
    pline_closed,
    polyline::{PlineSource, PlineSourceMut, Polyline},
};

#[must_use]
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

#[must_use]
pub fn pathological1_no_arcs<T>(vertex_count: usize) -> Polyline<T>
where
    T: Real,
{
    pathological1::<T>(vertex_count)
        .arcs_to_approx_lines(T::from(0.01).unwrap())
        .unwrap()
}

#[must_use]
pub fn profile1() -> Polyline<f64> {
    pline_closed![
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 1.0),
        (10.0, 0.0, -0.5),
        (10.0, 10.0, 0.5),
        (14.0, 20.0, -0.5),
        (0.0, 20.0, 0.0)
    ]
}

#[must_use]
pub fn profile1_no_arcs() -> Polyline<f64> {
    profile1().arcs_to_approx_lines(0.01).unwrap()
}

#[must_use]
pub fn profile2() -> Polyline<f64> {
    pline_closed![
        (0.0, 25.0, 1.0),
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 1.0),
        (10.0, 0.0, -0.5),
        (8.0, 9.0, 0.374794619217547),
        (21.0, 0.0, 0.0),
        (23.0, 0.0, 1.0),
        (32.0, 0.0, -0.5),
        (28.0, 0.0, 0.5),
        (39.0, 21.0, 0.0),
        (28.0, 12.0, 0.0)
    ]
}

#[must_use]
pub fn profile2_no_arcs() -> Polyline<f64> {
    profile2().arcs_to_approx_lines(0.01).unwrap()
}

/// Orthogonal building footprint with entrance, service, and terrace bays.
#[must_use]
pub fn floor_plan() -> Polyline<f64> {
    pline_closed![
        (0.0, 0.0, 0.0),
        (30.0, 0.0, 0.0),
        (30.0, -6.0, 0.0),
        (50.0, -6.0, 0.0),
        (50.0, 0.0, 0.0),
        (80.0, 0.0, 0.0),
        (80.0, 8.0, 0.0),
        (95.0, 8.0, 0.0),
        (95.0, 0.0, 0.0),
        (130.0, 0.0, 0.0),
        (130.0, 30.0, 0.0),
        (122.0, 30.0, 0.0),
        (122.0, 50.0, 0.0),
        (130.0, 50.0, 0.0),
        (130.0, 80.0, 0.0),
        (100.0, 80.0, 0.0),
        (100.0, 90.0, 0.0),
        (75.0, 90.0, 0.0),
        (75.0, 80.0, 0.0),
        (50.0, 80.0, 0.0),
        (50.0, 72.0, 0.0),
        (30.0, 72.0, 0.0),
        (30.0, 80.0, 0.0),
        (0.0, 80.0, 0.0),
        (0.0, 55.0, 0.0),
        (8.0, 55.0, 0.0),
        (8.0, 35.0, 0.0),
        (0.0, 35.0, 0.0)
    ]
}

/// Mixed line-and-arc mounting bracket with side lugs and a cable notch.
#[must_use]
pub fn mechanical_bracket() -> Polyline<f64> {
    const QUARTER_ARC_BULGE: f64 = 0.41421356237309503;

    pline_closed![
        (10.0, 0.0, 0.0),
        (90.0, 0.0, QUARTER_ARC_BULGE),
        (100.0, 10.0, 0.0),
        (100.0, 25.0, QUARTER_ARC_BULGE),
        (110.0, 35.0, 0.0),
        (110.0, 45.0, QUARTER_ARC_BULGE),
        (100.0, 55.0, 0.0),
        (100.0, 70.0, QUARTER_ARC_BULGE),
        (90.0, 80.0, 0.0),
        (65.0, 80.0, 0.0),
        (65.0, 65.0, 0.0),
        (55.0, 65.0, 0.0),
        (55.0, 80.0, 0.0),
        (10.0, 80.0, QUARTER_ARC_BULGE),
        (0.0, 70.0, 0.0),
        (0.0, 55.0, QUARTER_ARC_BULGE),
        (-10.0, 45.0, 0.0),
        (-10.0, 35.0, QUARTER_ARC_BULGE),
        (0.0, 25.0, 0.0),
        (0.0, 10.0, QUARTER_ARC_BULGE)
    ]
}

/// Laser-cut strip with wide pads joined by narrow tapered links.
///
/// An inward offset larger than each link's half-width splits the strip into one component per
/// pad. Each collapsed link creates two distinct raw-offset contacts, so contact count grows
/// linearly with `link_count`.
#[must_use]
pub fn tapered_link_strip(link_count: u32) -> Polyline<f64> {
    const HALF_PITCH: f64 = 12.0;
    const PAD_HALF_WIDTH: f64 = 10.0;
    const LINK_HALF_WIDTH: f64 = 2.0;

    let step_count = 2 * link_count;
    let mut result = Polyline::new_closed();
    result.reserve(2 * usize::try_from(step_count + 1).unwrap());
    let half_width = |i| {
        if i % 2 == 0 {
            PAD_HALF_WIDTH
        } else {
            LINK_HALF_WIDTH
        }
    };

    for i in 0..=step_count {
        result.add(f64::from(i) * HALF_PITCH, -half_width(i), 0.0);
    }
    for i in (0..=step_count).rev() {
        result.add(f64::from(i) * HALF_PITCH, half_width(i), 0.0);
    }
    result
}

/// Survey-style open road centerline sampled as short straight spans.
#[must_use]
pub fn road_centerline() -> Polyline<f64> {
    let mut result = Polyline::new();
    result.reserve(81);
    for i in 0..=80 {
        let x = f64::from(i) * 5.0;
        let y = 18.0 * (f64::from(i) * 0.11).sin() + 6.0 * (f64::from(i) * 0.037).sin();
        result.add(x, y, 0.0);
    }
    result
}

/// Closed product-enclosure outline sampled from eight cubic Bézier curves.
#[must_use]
pub fn bezier_enclosure() -> Polyline<f64> {
    const SAMPLES_PER_CURVE: u32 = 64;

    let curves = [
        [
            Vector2::new(0.0, 20.0),
            Vector2::new(8.0, 4.0),
            Vector2::new(25.0, 0.0),
            Vector2::new(45.0, 0.0),
        ],
        [
            Vector2::new(45.0, 0.0),
            Vector2::new(70.0, -4.0),
            Vector2::new(105.0, 0.0),
            Vector2::new(125.0, 8.0),
        ],
        [
            Vector2::new(125.0, 8.0),
            Vector2::new(145.0, 12.0),
            Vector2::new(162.0, 24.0),
            Vector2::new(165.0, 38.0),
        ],
        [
            Vector2::new(165.0, 38.0),
            Vector2::new(170.0, 55.0),
            Vector2::new(165.0, 70.0),
            Vector2::new(150.0, 78.0),
        ],
        [
            Vector2::new(150.0, 78.0),
            Vector2::new(132.0, 92.0),
            Vector2::new(108.0, 100.0),
            Vector2::new(90.0, 100.0),
        ],
        [
            Vector2::new(90.0, 100.0),
            Vector2::new(68.0, 101.0),
            Vector2::new(40.0, 98.0),
            Vector2::new(25.0, 92.0),
        ],
        [
            Vector2::new(25.0, 92.0),
            Vector2::new(8.0, 85.0),
            Vector2::new(-8.0, 72.0),
            Vector2::new(-10.0, 55.0),
        ],
        [
            Vector2::new(-10.0, 55.0),
            Vector2::new(-15.0, 38.0),
            Vector2::new(-8.0, 25.0),
            Vector2::new(0.0, 20.0),
        ],
    ];

    let mut result = Polyline::new_closed();
    result.reserve(curves.len() * usize::try_from(SAMPLES_PER_CURVE).unwrap());
    for [p0, p1, p2, p3] in curves {
        for i in 0..SAMPLES_PER_CURVE {
            let t = f64::from(i) / f64::from(SAMPLES_PER_CURVE);
            let u = 1.0 - t;
            let point = p0.scale(u * u * u)
                + p1.scale(3.0 * u * u * t)
                + p2.scale(3.0 * u * t * t)
                + p3.scale(t * t * t);
            result.add(point.x, point.y, 0.0);
        }
    }
    result
}

/// Forty-eight-tooth involute spur gear outline without root fillets.
#[must_use]
pub fn involute_gear() -> Polyline<f64> {
    const TOOTH_COUNT: u32 = 48;
    const FLANK_STEPS: u32 = 8;
    const TIP_STEPS: u32 = 4;
    const ROOT_STEPS: u32 = 4;
    const PITCH_RADIUS: f64 = 96.0;
    const ROOT_RADIUS: f64 = 91.0;
    const OUTER_RADIUS: f64 = 100.0;

    let pressure_angle = 20.0_f64.to_radians();
    let base_radius = PITCH_RADIUS * pressure_angle.cos();
    let involute = |radius: f64| {
        let pressure_angle_at_radius = (base_radius / radius).acos();
        pressure_angle_at_radius.tan() - pressure_angle_at_radius
    };
    let pitch_involute = involute(PITCH_RADIUS);
    let half_tooth_angle_at_pitch = std::f64::consts::PI / (2.0 * f64::from(TOOTH_COUNT));
    let flank_half_angle = |radius| half_tooth_angle_at_pitch + pitch_involute - involute(radius);
    let tooth_angle = std::f64::consts::TAU / f64::from(TOOTH_COUNT);
    let root_half_angle = flank_half_angle(ROOT_RADIUS);
    let outer_half_angle = flank_half_angle(OUTER_RADIUS);

    let points_per_tooth = FLANK_STEPS + 1 + TIP_STEPS + FLANK_STEPS + ROOT_STEPS - 1;
    let mut result = Polyline::new_closed();
    result.reserve(usize::try_from(TOOTH_COUNT * points_per_tooth).unwrap());
    for tooth in 0..TOOTH_COUNT {
        let center_angle = f64::from(tooth) * tooth_angle;
        let mut add_polar = |radius: f64, angle: f64| {
            let point = point_on_circle(radius, Vector2::zero(), angle);
            result.add(point.x, point.y, 0.0);
        };

        for step in 0..=FLANK_STEPS {
            let t = f64::from(step) / f64::from(FLANK_STEPS);
            let radius = ROOT_RADIUS + (OUTER_RADIUS - ROOT_RADIUS) * t;
            add_polar(radius, center_angle - flank_half_angle(radius));
        }

        for step in 1..=TIP_STEPS {
            let t = f64::from(step) / f64::from(TIP_STEPS);
            add_polar(
                OUTER_RADIUS,
                center_angle - outer_half_angle + 2.0 * outer_half_angle * t,
            );
        }

        for step in 1..=FLANK_STEPS {
            let t = f64::from(step) / f64::from(FLANK_STEPS);
            let radius = OUTER_RADIUS - (OUTER_RADIUS - ROOT_RADIUS) * t;
            add_polar(radius, center_angle + flank_half_angle(radius));
        }

        let next_root_angle = center_angle + tooth_angle - root_half_angle;
        for step in 1..ROOT_STEPS {
            let t = f64::from(step) / f64::from(ROOT_STEPS);
            add_polar(
                ROOT_RADIUS,
                center_angle
                    + root_half_angle
                    + (next_root_angle - center_angle - root_half_angle) * t,
            );
        }
    }
    result
}

/// Forty-eight-tooth involute spur gear with line-sampled flanks and circular tip and root arcs.
#[must_use]
pub fn involute_gear_with_arcs() -> Polyline<f64> {
    const TOOTH_COUNT: u32 = 48;
    const FLANK_STEPS: u32 = 8;
    const PITCH_RADIUS: f64 = 96.0;
    const ROOT_RADIUS: f64 = 91.0;
    const OUTER_RADIUS: f64 = 100.0;

    let pressure_angle = 20.0_f64.to_radians();
    let base_radius = PITCH_RADIUS * pressure_angle.cos();
    let involute = |radius: f64| {
        let pressure_angle_at_radius = (base_radius / radius).acos();
        pressure_angle_at_radius.tan() - pressure_angle_at_radius
    };
    let pitch_involute = involute(PITCH_RADIUS);
    let half_tooth_angle_at_pitch = std::f64::consts::PI / (2.0 * f64::from(TOOTH_COUNT));
    let flank_half_angle = |radius| half_tooth_angle_at_pitch + pitch_involute - involute(radius);
    let tooth_angle = std::f64::consts::TAU / f64::from(TOOTH_COUNT);
    let root_half_angle = flank_half_angle(ROOT_RADIUS);
    let outer_half_angle = flank_half_angle(OUTER_RADIUS);
    let tip_bulge = (outer_half_angle / 2.0).tan();
    let root_bulge = ((tooth_angle - 2.0 * root_half_angle) / 4.0).tan();

    let points_per_tooth = 2 * FLANK_STEPS + 2;
    let mut result = Polyline::new_closed();
    result.reserve(usize::try_from(TOOTH_COUNT * points_per_tooth).unwrap());
    for tooth in 0..TOOTH_COUNT {
        let center_angle = f64::from(tooth) * tooth_angle;
        let mut add_polar = |radius: f64, angle: f64, bulge: f64| {
            let point = point_on_circle(radius, Vector2::zero(), angle);
            result.add(point.x, point.y, bulge);
        };

        for step in 0..=FLANK_STEPS {
            let t = f64::from(step) / f64::from(FLANK_STEPS);
            let radius = ROOT_RADIUS + (OUTER_RADIUS - ROOT_RADIUS) * t;
            let bulge = if step == FLANK_STEPS { tip_bulge } else { 0.0 };
            add_polar(radius, center_angle - flank_half_angle(radius), bulge);
        }

        add_polar(OUTER_RADIUS, center_angle + outer_half_angle, 0.0);

        for step in 1..=FLANK_STEPS {
            let t = f64::from(step) / f64::from(FLANK_STEPS);
            let radius = OUTER_RADIUS - (OUTER_RADIUS - ROOT_RADIUS) * t;
            let bulge = if step == FLANK_STEPS { root_bulge } else { 0.0 };
            add_polar(radius, center_angle + flank_half_angle(radius), bulge);
        }
    }
    result
}
