mod test_utils;

use cavalier_contours::polyline::Polyline;
use test_utils::{
    create_property_set, property_sets_match, ModifiedPlineSet, ModifiedPlineSetVisitor,
    ModifiedPlineState, PlineProperties,
};

fn offset_into_properties_set(
    polyline: &Polyline<f64>,
    offset: f64,
    inverted: bool,
) -> Vec<PlineProperties> {
    let offset = if inverted { -offset } else { offset };
    create_property_set(&polyline.parallel_offset(offset), inverted)
}

struct PlineOffsetTestVisitor<'a> {
    offset: f64,
    expected_properties_set: &'a [PlineProperties],
}

impl<'a> ModifiedPlineSetVisitor for PlineOffsetTestVisitor<'a> {
    fn visit(&mut self, modified_pline: Polyline<f64>, pline_state: ModifiedPlineState) {
        let offset_results = offset_into_properties_set(
            &modified_pline,
            self.offset,
            pline_state.inverted_direction,
        );
        assert!(
            property_sets_match(&offset_results, self.expected_properties_set),
            "property sets do not match, modified state: {:?}",
            pline_state
        );
    }
}

fn run_pline_offset_tests(
    input: &Polyline<f64>,
    offset: f64,
    expected_properties_set: &[PlineProperties],
) {
    let mut visitor = PlineOffsetTestVisitor {
        offset,
        expected_properties_set,
    };

    let test_set = ModifiedPlineSet::new(input, true, true);
    test_set.accept(&mut visitor);
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

/// Simple/basic test cases for parallel offset (e.g. circles and rectangles).
mod test_simple {
    use super::*;
    use cavalier_contours::{pline_closed, pline_open};

    declare_offset_tests!(
        closed_rectangle_inward {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], 2.0) =>
            [PlineProperties::new(4, 96.0, 44.0, 2.0, 2.0, 18.0, 8.0)]
        }
        closed_rectangle_outward {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], -2.0) =>
            [PlineProperties::new(8, 332.56637061436, 72.566370614359, -2.0, -2.0, 22.0, 12.0)]
        }
        open_rectangle_inward {
            (pline_open![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 0.0) ], 2.0) =>
            [PlineProperties::new(5, 0.0, 44.0, 2.0, 2.0, 18.0, 8.0)]
        }
        open_rectangle_outward {
            (pline_open![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 0.0) ], -2.0) =>
            [PlineProperties::new(8, 0.0, 69.424777960769, -2.0, -2.0, 22.0, 12.0)]
        }
        closed_rectangle_into_overlapping_line {
            (pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ], 5.0) =>
            [PlineProperties::new(2, 0.0, 20.0, 5.0, 5.0, 15.0, 5.0)]
        }
        closed_diamond_offset_inward {
            (pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ], -5.0) =>
            [PlineProperties::new(4, -17.157287525381, 16.568542494924, -2.9289321881345, -2.9289321881345, 2.9289321881345, 2.9289321881345)]
        }
        closed_diamond_offset_outward {
            (pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ], 5.0) =>
            [PlineProperties::new(8, -561.38252881436, 87.984469030822, -15.0, -15.0, 15.0, 15.0)]
        }
        open_diamond_offset_inward {
            (pline_open![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0) ], -5.0) =>
            [PlineProperties::new(5, 0.0, 16.568542494924, -2.9289321881345, -2.9289321881345, 2.9289321881345, 2.9289321881345)]
        }
        open_diamond_offset_outward {
            (pline_open![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0) ], 5.0) =>
            [PlineProperties::new(8, 0.0, 80.130487396847, -13.535533905933, -15.0, 15.0, 15.0)]
        }
        closed_circle_offset_inward {
            (pline_closed![ (-5.0, 0.0, 1.0), (5.0, 0.0, 1.0) ], 3.0) =>
            [PlineProperties::new(2, 12.566370614359, 12.566370614359, -2.0, -2.0, 2.0, 2.0)]
        }
        closed_circle_offset_outward {
            (pline_closed![ (-5.0, 0.0, 1.0), (5.0, 0.0, 1.0) ], -3.0) =>
            [PlineProperties::new(2, 201.06192982975, 50.265482457437, -8.0, -8.0, 8.0, 8.0)]
        }
    );
}

/// Specific test cases for parallel offset that trigger edge case scenarios or specific code paths.
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
            [PlineProperties::new(4, 0.094833810726263, 1.8213211761499, 31.533345690439,
                                     0.90572346564886, 32.26949555256, 1.2817628453883),
             PlineProperties::new(6, 1.7197931450343, 7.5140262005179, 28.047835685678,
                                     0.44926177903859, 31.495431966272, 1.4)]
        }
        case2 {
            // first vertex position is on top of intersect with second segment (leading to some
            // edge cases around the join between the last vertex and first vertex)
            (pline_closed![(27.804688, 1.0, 0.0),
                           (27.804688, 0.75, 0.0),
                           (32.195313, 0.75, 0.0),
                           (32.195313, 1.0, 0.0),
                           (31.5, 1.0, -0.3178375134974),
                           (30.792893109407, 1.5, 0.0),
                           (29.207106890593, 1.5, -0.31783754777018),
                           (28.499999813231, 1.0000000000001, 0.0)], 0.25) =>
            [PlineProperties::new(4, 0.36247092523069, 3.593999211522, 29.16143806012, 1.0,
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
            [PlineProperties::new(9, 0.0, 99.224754131592, 28.12347538298, -19.0, 44.0, 19.0)]
        }
    );
}

/// Test cases that have failed or had issues in the past but are otherwise seemingly unremarkable.
mod test_past_failures {
    use super::*;
    use cavalier_contours::pline_open;

    declare_offset_tests!(
        small_open_pline {
            (pline_open![(8.25, 0.0, 0.0),
                         (8.25, 0.0625, -0.414214),
                         (8.5, 0.3125, 0.0)], 0.25) =>
            [PlineProperties::new(3, 0.0, 0.84789847066602, 7.9999999999999, 0.0, 8.5000001870958, 0.56250000000015)]
        }
    );
}
