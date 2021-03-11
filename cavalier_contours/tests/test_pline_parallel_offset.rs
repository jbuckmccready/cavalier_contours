use cavalier_contours::{FuzzyEq, Polyline, AABB};

/// Fuzzy compare AABB values
fn aabb_fuzzy_eq_eps(a: &AABB<f64>, b: &AABB<f64>, eps: f64) -> bool {
    a.min_x.fuzzy_eq_eps(b.min_x, eps)
        && a.min_y.fuzzy_eq_eps(b.min_y, eps)
        && a.max_x.fuzzy_eq_eps(b.max_x, eps)
        && a.max_y.fuzzy_eq_eps(b.max_y, eps)
}

/// Holds a set of properties of a polyline for comparison in tests
#[derive(Debug, Copy, Clone)]
struct PolylineProperties {
    vertex_count: usize,
    area: f64,
    path_length: f64,
    extents: AABB<f64>,
}

impl PolylineProperties {
    fn new(
        vertex_count: usize,
        area: f64,
        path_length: f64,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
    ) -> Self {
        Self {
            vertex_count,
            area,
            path_length,
            extents: AABB::new(min_x, min_y, max_x, max_y),
        }
    }

    fn from_pline(pline: &Polyline<f64>, invert_area: bool) -> Self {
        let area = {
            let a = pline.area();
            if invert_area {
                -a
            } else {
                a
            }
        };

        Self {
            vertex_count: pline.len(),
            area,
            path_length: pline.path_length(),
            extents: pline.extents().unwrap(),
        }
    }

    fn fuzzy_eq_eps(&self, other: &Self, eps: f64) -> bool {
        self.vertex_count == other.vertex_count
            && self.area.fuzzy_eq_eps(other.area, eps)
            && self.path_length.fuzzy_eq_eps(other.path_length, eps)
            && aabb_fuzzy_eq_eps(&self.extents, &other.extents, eps)
    }
}

fn create_property_set(polylines: &[Polyline<f64>], invert_area: bool) -> Vec<PolylineProperties> {
    polylines
        .iter()
        .map(|pl| PolylineProperties::from_pline(pl, invert_area))
        .collect()
}

fn offset_into_properties_set(
    polyline: &Polyline<f64>,
    offset: f64,
    invert_area: bool,
) -> Vec<PolylineProperties> {
    create_property_set(&polyline.parallel_offset(offset, None, None), invert_area)
}

fn property_sets_match(
    result_set: &[PolylineProperties],
    expected_set: &[PolylineProperties],
) -> bool {
    let mut sets_match = true;
    if result_set.len() != expected_set.len() {
        sets_match = false;
    } else {
        // using simple N^2 comparisons to compare property sets (sets are always relatively small, e.g. N < 10)
        for properties_expected in expected_set {
            let match_count = result_set
                .iter()
                .filter(|properties_result| {
                    properties_expected.fuzzy_eq_eps(properties_result, 1e-5)
                })
                .count();

            if match_count != 1 {
                sets_match = false;
                break;
            }
        }
    }

    if !sets_match {
        dbg!(result_set);
        dbg!(expected_set);
    }

    sets_match
}

/// Takes a polyline and moves the 0 index forward in the vertex buffer
/// (only applicable to closed polylines)
fn cycle_start_index_forward(input: &Polyline<f64>, n: usize) -> Polyline<f64> {
    let mut result = Polyline::with_capacity(input.len());
    result.set_is_closed(true);

    for &v in input.iter().cycle().skip(n).take(input.len()) {
        result.add_vertex(v);
    }

    result
}

fn run_pline_offset_tests(
    input: &Polyline<f64>,
    offset: f64,
    expected_properties_set: &[PolylineProperties],
) {
    let offset_results = offset_into_properties_set(input, offset, false);
    assert!(
        property_sets_match(&offset_results, expected_properties_set),
        "property sets do not match!"
    );

    let mut inverted = input.clone();
    inverted.invert_direction();
    let inverted_results = offset_into_properties_set(&inverted, -offset, true);
    assert!(
        property_sets_match(&inverted_results, expected_properties_set),
        "property sets do not match after inverting direction!"
    );

    if input.is_closed() {
        for i in 0..input.len() - 1 {
            let cycled = cycle_start_index_forward(input, i);
            let cycled_offset_results = offset_into_properties_set(&cycled, offset, false);
            assert!(
                property_sets_match(&cycled_offset_results, expected_properties_set),
                "property sets do not match after cycling start index forward {} times",
                i
            );
        }
    }
}

macro_rules! declare_offset_tests {
    ($($name:ident { $($value:expr => $expected:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_pline_offset_tests(&$value.0, $value.1, &$expected);
                )+
            }
        )+
    };
}
mod test_simple {
    use super::*;
    use cavalier_contours::{pline_closed, pline_open};

    declare_offset_tests!(
        closed_rectangle_inward {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], 2.0) =>
            [PolylineProperties::new(4, 96.0, 44.0, 2.0, 2.0, 18.0, 8.0)]
        }
        closed_rectangle_outward {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], -2.0) =>
            [PolylineProperties::new(8, 332.56637061436, 72.566370614359, -2.0, -2.0, 22.0, 12.0)]
        }
        open_rectangle_inward {
            (pline_open![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 0.0) ], 2.0) =>
            [PolylineProperties::new(5, 0.0, 44.0, 2.0, 2.0, 18.0, 8.0)]
        }
        open_rectangle_outward {
            (pline_open![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 0.0) ], -2.0) =>
            [PolylineProperties::new(8, 0.0, 69.424777960769, -2.0, -2.0, 22.0, 12.0)]
        }
        closed_rectangle_into_overlapping_line {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], 5.0) =>
            [PolylineProperties::new(2, 0.0, 20.0, 5.0, 5.0, 15.0, 5.0)]
        }
        closed_diamond_offset_inward {
            (pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ], -5.0) =>
            [PolylineProperties::new(4, -17.157287525381, 16.568542494924, -2.9289321881345, -2.9289321881345, 2.9289321881345, 2.9289321881345)]
        }
        closed_diamond_offset_outward {
            (pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ], 5.0) =>
            [PolylineProperties::new(8, -561.38252881436, 87.984469030822, -15.0, -15.0, 15.0, 15.0)]
        }
        open_diamond_offset_inward {
            (pline_open![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0) ], -5.0) =>
            [PolylineProperties::new(5, 0.0, 16.568542494924, -2.9289321881345, -2.9289321881345, 2.9289321881345, 2.9289321881345)]
        }
        open_diamond_offset_outward {
            (pline_open![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0) ], 5.0) =>
            [PolylineProperties::new(8, 0.0, 80.130487396847, -13.535533905933, -15.0, 15.0, 15.0)]
        }
        closed_circle_offset_inward {
            (pline_closed![ (-5.0, 0.0, 1.0), (5.0, 0.0, 1.0) ], 3.0) =>
            [PolylineProperties::new(2, 12.566370614359, 12.566370614359, -2.0, -2.0, 2.0, 2.0)]
        }
        closed_circle_offset_outward {
            (pline_closed![ (-5.0, 0.0, 1.0), (5.0, 0.0, 1.0) ], -3.0) =>
            [PolylineProperties::new(2, 201.06192982975, 50.265482457437, -8.0, -8.0, 8.0, 8.0)]
        }
    );
}

mod test_specific {
    use super::*;
    use cavalier_contours::{pline_closed, pline_open};

    declare_offset_tests!(
        case1 {
            // offset arc just past line, in this case float epsilon values can cause failures
            (pline_closed![(27.804688, 1.0, 0.0),
                           (28.46842055794889, 0.3429054695163245, 0.0),
                           (32.34577133994935, 0.9269762697003898, 0.0),
                           (32.38116957207762, 1.451312562563487, 0.0),
                           (31.5, 1.0, -0.31783751349740424),
                           (30.79289310940682, 1.5, 0.0),
                           (29.20710689059337, 1.5, -0.31783754777018053),
                           (28.49999981323106, 1.00000000000007, 0.0)], 0.1) =>
            [PolylineProperties::new(4, 0.094833810726263, 1.8213211761499, 31.533345690439,
                                     0.90572346564886, 32.26949555256, 1.2817628453883),
             PolylineProperties::new(6, 1.7197931450343, 7.5140262005179, 28.047835685678,
                                     0.44926177903859, 31.495431966272, 1.4)]
        }
        case2 {
            // first vertex position is on top of intersect with second segment (leading to some edge cases
            // around the join between the last vertex and first vertex)
            (pline_closed![(27.804688, 1.0, 0.0),
                           (27.804688, 0.75, 0.0),
                           (32.195313, 0.75, 0.0),
                           (32.195313, 1.0, 0.0),
                           (31.5, 1.0, -0.3178375134974),
                           (30.792893109407, 1.5, 0.0),
                           (29.207106890593, 1.5, -0.31783754777018),
                           (28.499999813231, 1.0000000000001, 0.0)], 0.25) =>
            [PolylineProperties::new(4, 0.36247092523069, 3.593999211522, 29.16143806012, 1.0,
                                     30.838561906052, 1.25)]
        }
        case3 {
            // collapsed rectangle with raw offset polyline having no self intersects
            (pline_closed![(0.0, 0.0, 0.0), (120.0, 0.0, 0.0), (120.0, 40.0, 0.0), (0.0, 40.0, 0.0)], 30.0) => []
        }
        case4 {
            // three consecutive raw off segments intersect at the same point
            (pline_open![(30.123475382979791, -17.0, 0.0),
                         (42.0, -17.0, 0.0),
                         (42.0, 17.0, 0.0),
                         (30.123475382979798, 17.00000, -0.093311550024413187),
                         (30.500000000000000, 15.00000, 0.00000),
                         (30.500000000000000, -15.00000, -0.093311550024413409)], -2.0) =>
            [PolylineProperties::new(9, 0.0, 99.224754131592, 28.12347538298, -19.0, 44.0, 19.0)]
        }
    );
}
