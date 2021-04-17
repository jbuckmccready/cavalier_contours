mod test_utils;

use cavalier_contours::polyline::{PlineOffsetOptions, Polyline};
use test_utils::{
    create_property_set, property_sets_match, ModifiedPlineSet, ModifiedPlineSetVisitor,
    ModifiedPlineState, PlineProperties,
};

fn offset_into_properties_set(
    polyline: &Polyline<f64>,
    offset: f64,
    inverted: bool,
    handle_self_intersects: bool,
) -> Vec<PlineProperties> {
    let offset = if inverted { -offset } else { offset };
    let options = PlineOffsetOptions {
        handle_self_intersects,
        ..Default::default()
    };
    let offset_results = polyline.parallel_offset_opt(offset, &options);
    for r in offset_results.iter() {
        assert_eq!(
            r.remove_repeat_pos(1e-5).len(),
            r.len(),
            "offset result should not have repeat positioned vertexes"
        );
    }
    create_property_set(&offset_results, inverted)
}

struct PlineOffsetTestVisitor<'a> {
    offset: f64,
    expected_properties_set: &'a [PlineProperties],
    handle_self_intersects: bool,
}

impl<'a> ModifiedPlineSetVisitor for PlineOffsetTestVisitor<'a> {
    fn visit(&mut self, modified_pline: Polyline<f64>, pline_state: ModifiedPlineState) {
        let offset_results = offset_into_properties_set(
            &modified_pline,
            self.offset,
            pline_state.inverted_direction,
            self.handle_self_intersects,
        );
        assert!(
            property_sets_match(&offset_results, self.expected_properties_set),
            "property sets do not match, modified state: {:?}",
            pline_state
        );

        if modified_pline.is_closed() {
            let offset_results = offset_into_properties_set(
                &modified_pline,
                self.offset,
                pline_state.inverted_direction,
                true,
            );
            assert!(
            property_sets_match(&offset_results, self.expected_properties_set),
            "property sets do not match with handle_self_intersects set to true, modified state: {:?}",
            pline_state
        );
        }
    }
}

fn run_pline_offset_tests(
    input: &Polyline<f64>,
    offset: f64,
    expected_properties_set: &[PlineProperties],
    handle_self_intersects: bool,
) {
    let mut visitor = PlineOffsetTestVisitor {
        offset,
        expected_properties_set,
        handle_self_intersects,
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
                    run_pline_offset_tests(&$value.0, $value.1, &$expected, false);
                )+
            }
        )+
    };
}

macro_rules! declare_self_intersecting_offset_tests {
    ($($name:ident { $($value:expr => $expected:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_pline_offset_tests(&$value.0, $value.1, &$expected, true);
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
        circle_collapsed_into_point {
            (pline_closed![ (0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], 1.0) =>
            []
        }
        square_collapsed_into_point {
            (pline_closed![ (-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)], 1.0) =>
            []
        }
        circle_collapsed {
            (pline_closed![ (0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], 2.0) =>
            []
        }
        square_collapsed {
            (pline_closed![ (-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)], 2.0) =>
            []
        }
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
        case5 {
            // tests clipping circle at start of polyline works correctly (with collapsed arc at
            // start)
            (pline_open![(100.0, 100.0, -0.5),
                         (80.0, 90.0, 0.374794619217547),
                         (210.0, 0.0, 0.0),
                         (230.0, 0.0, 1.0),
                         (320.0, 0.0, -0.5),
                         (280.0, 0.0, 0.5),
                         (390.0, 210.0, 0.0),
                         (280.0, 120.0, 0.5)], -30.0) =>
            [PlineProperties::new(7, 0.0, 916.7498699472794, 50.000000000000014, -74.99999999999997, 434.41586988912127, 240.0)]
        }
        case6 {
            // tests line to line join where one of the lines is a collapsed arc and has there is no
            // intersection between them (they should be connected with an arc)
            (pline_open![(100.0, 100.0, -0.5),
                         (80.0, 90.0, 0.374794619217547),
                         (210.0, 0.0, 0.0),
                         (230.0, 0.0, 1.0),
                         (320.0, 0.0, -0.5),
                         (280.0, 0.0, 0.5),
                         (390.0, 210.0, 0.0),
                         (280.0, 120.0, 0.5)], 45.0) =>
            [PlineProperties::new(9, 0.0, 354.5924544050689, 137.36151283418917, 37.416573867739416, 357.2096858656279, 125.02881860280142)]
        }
        case7 {
            // tests line to line join where one of the lines is a collapsed arc and there is a
            // false intersect between them (they should be connected with an arc)
            (pline_open![(347.88382287598745, 269.85890289007887, -0.5),
                         (80.0, 90.0, 0.374794619217547),
                         (204.65318559134363, 55.01294696311311, 0.0),
                         (179.35722417454295, -56.42578188285236, 1.0),
                         (270.7403323676961, -93.94095261477841, -0.5),
                         (346.1511941991571, 157.81558178838168, 0.5),
                         (390.0, 210.0, 0.0),
                         (495.7348032988456, 68.8739763777561, 0.5)], 47.0) =>
            [PlineProperties::new(4, 0.0, 226.5117782356207, 394.0401535993119, 97.05525803008165, 533.3488347822203, 267.40547394194897)]
        }
        case8 {
            // almost collapsed adjacent arcs with true intersects
            (pline_closed![(30.0, 0.0, 1.0),
                           (30.0, 150.0, 0.0),
                           (-380.0, 0.0, 0.0),
                           (30.0, -150.0, 1.0)], 71.0) =>
            [PlineProperties::new(3, 31.563080748331117, 36.43002218023972, 17.851377192815367, 69.95291962376376, 34.00000003096393, 75.82916586272847),
             PlineProperties::new(3, 7211.747093261731, 504.5601794261032, -173.3532697788056, -61.27715478753268, -5.862380026216215, 61.27715478753261),
             PlineProperties::new(3, 31.56308032687207, 36.43002208996665, 17.851377192815107, -75.82916586272874, 34.000000000000675, -69.95291962376365)]
        }
        case9 {
            // almost collapsed adjacent arcs with false intersects
            (pline_closed![(30.0, 0.0, 1.0),
                           (30.0, 150.0, 0.0),
                           (-380.0, 0.0, 0.0),
                           (30.0, -150.0, 1.0)], 73.0) =>
            [PlineProperties::new(3, 6273.618943028112, 440.30207980349326, -167.53223512468745, -54.49913379476977, -18.567936085649954, 54.499133794769804)]
        }
        case10 {
            // collapsed adjacent arcs
            (pline_closed![(30.0, 0.0, 1.0),
                           (30.0, 150.0, 0.0),
                           (-380.0, 0.0, 0.0),
                           (30.0, -150.0, 1.0)], 77.0) =>
            [PlineProperties::new(3, 4682.865221417136, 359.74976552142584, -155.89016581645112, -45.203002912175684, -32.335291189837534, 45.203002912175705)]
        }
        case11 {
            // sequences of segments aligned along axis
            (pline_closed![(-225.0, 0.0, 0.0),
                           (-200.0, 0.0, 0.0),
                           (-175.0, 0.0, 1.0),
                           (-150.0, 0.0, 1.0),
                           (-125.0, 0.0, 1.0),
                           (-100.0, 0.0, 0.0),
                           (-75.0, 0.0, -1.0),
                           (-50.0, 0.0, -1.0),
                           (-25.0, 0.0, -1.0),
                           (0.0, 0.0, 0.0),
                           (25.0, 0.0, 1.0),
                           (50.0, 0.0, 0.0),
                           (75.0, 0.0, 1.0),
                           (100.0, 0.0, 0.0),
                           (125.0, 0.0, 1.0),
                           (150.0, 0.0, 1.0),
                           (165.0, 0.0, 1.0),
                           (190.0, 0.0, 0.0),
                           (215.0, 0.0, 1.0),
                           (230.0, 0.0, 1.0),
                           (255.0, 0.0, 1.0),
                           (270.0, 0.0, 0.0),
                           (280.0, 0.0, 0.0),
                           (390.0, 200.0, 0.0),
                           (365.0, 200.0, 1.0),
                           (340.0, 200.0, 1.0),
                           (352.5, 200.0, -1.0),
                           (290.0, 200.0, 0.0),
                           (310.0, 200.0, 1.0),
                           (270.0, 200.0, -1.0),
                           (280.0, 200.0, -1.0),
                           (225.0, 200.0, 1.0),
                           (200.0, 200.0, -1.0),
                           (175.0, 200.0, 1.0),
                           (150.0, 200.0, 0.0),
                           (-340.0, 200.0, 0.0)], -9.0) =>
            [PlineProperties::new(44, 141959.84850931115, 2052.5428168464014, -348.99999999999994, -21.5, 398.99999999999994, 229.0)]
        }
        case12 {
            // sequences of segments aligned along axis with some collapsed arcs
            (pline_closed![(-225.0, 0.0, 0.0),
                           (-200.0, 0.0, 0.0),
                           (-175.0, 0.0, 1.0),
                           (-150.0, 0.0, 1.0),
                           (-125.0, 0.0, 1.0),
                           (-100.0, 0.0, 0.0),
                           (-75.0, 0.0, -1.0),
                           (-50.0, 0.0, -1.0),
                           (-25.0, 0.0, -1.0),
                           (0.0, 0.0, 0.0),
                           (25.0, 0.0, 1.0),
                           (50.0, 0.0, 0.0),
                           (75.0, 0.0, 1.0),
                           (100.0, 0.0, 0.0),
                           (125.0, 0.0, 1.0),
                           (150.0, 0.0, 1.0),
                           (165.0, 0.0, 1.0),
                           (190.0, 0.0, 0.0),
                           (215.0, 0.0, 1.0),
                           (230.0, 0.0, 1.0),
                           (255.0, 0.0, 1.0),
                           (270.0, 0.0, 0.0),
                           (280.0, 0.0, 0.0),
                           (390.0, 200.0, 0.0),
                           (365.0, 200.0, 1.0),
                           (340.0, 200.0, 1.0),
                           (352.5, 200.0, -1.0),
                           (290.0, 200.0, 0.0),
                           (310.0, 200.0, 1.0),
                           (270.0, 200.0, -1.0),
                           (280.0, 200.0, -1.0),
                           (225.0, 200.0, 1.0),
                           (200.0, 200.0, -1.0),
                           (175.0, 200.0, 1.0),
                           (150.0, 200.0, 0.0),
                           (-340.0, 200.0, 0.0)], 9.0) =>
            [PlineProperties::new(45, 105309.44963383305, 1837.9627621817642, -324.4432552044466, -3.5, 374.77855901053806, 203.5),
             PlineProperties::new(4, 17.514629264722736, 24.09798450969452, 285.0, 208.2192186706253, 296.32455532033674, 211.00000000000003)]
        }
    );

    declare_self_intersecting_offset_tests!(
        self_intersecting_case1 {
            // tests clipping circle at start and end of polyline works correctly (with self
            // intersect between first and last segment)
            (pline_open![(305.8082007608764, 149.26270215110728, -0.5),
                         (80.0, 90.0, 0.374794619217547),
                         (210.0, 0.0, 0.0),
                         (230.0, 0.0, 1.0),
                         (320.0, 0.0, -0.5),
                         (280.0, 0.0, 0.5),
                         (390.0, 210.0, 0.0),
                         (280.0, 120.0, 0.5)], -30.0) =>
            [PlineProperties::new(3, 0.0, 24.810068463598633, 261.00286629228214, 143.21871897609964, 278.0250516267822, 160.58068007088974),
             PlineProperties::new(8, 0.0, 1047.5088824641757, 50.00000000000001, -74.99999999999997, 434.41586988912127, 240.0)]
        }
        self_intersecting_case2 {
            // self intersecting adjacent arcs
            (pline_closed![(-54.126705892111374, -9.012072327640396, 1.0),
                           (0.0, 200.0, 0.0),
                           (-200.0, 0.0, 0.0),
                           (0.0, -200.0, 1.0)], -9.0) =>
            [PlineProperties::new(7, 72784.07553221736, 1139.217753852123, -209.0, -208.99999999999997, 89.89004749763792, 209.0),
             PlineProperties::new(4, 0.0, 137.47770415252796, -63.126705892111374, -21.459436607513837, -0.0036782264819059662, 3.748798488343695)]
        }
    );
}

/// Test cases that have failed or had issues in the past but are otherwise seemingly unremarkable.
mod test_past_failures {
    use super::*;
    use cavalier_contours::pline_closed;
    use cavalier_contours::pline_open;

    declare_offset_tests!(
        open_pline1 {
            (pline_open![(8.25, 0.0, 0.0),
                         (8.25, 0.0625, -0.414214),
                         (8.5, 0.3125, 0.0)], 0.25) =>
            [PlineProperties::new(3, 0.0, 0.84789847066602, 7.9999999999999, 0.0, 8.5000001870958, 0.56250000000015)]
        }
        open_pline2 {
            (pline_open![(100.0, 100.0, -0.5),
                         (80.0, 90.0, 0.374794619217547),
                         (210.0, 0.0, 0.0),
                         (230.0, 0.0, 1.0),
                         (320.0, 0.0, -0.5),
                         (280.0, 0.0, 0.5),
                         (390.0, 210.0, 0.0),
                         (280.0, 120.0, 0.5)], 30.0) =>
            [PlineProperties::new(9, 0.0, 480.07132994083656, 119.08533878718923, 16.583123951777, 374.4158698891213, 158.00772717933913)]
        }
        closed_pline1 {
            (pline_closed![(100.0, 100.0, -0.5),
                           (80.0, 90.0, 0.374794619217547),
                           (100.0, 0.0, 1.0),
                           (225.0, 0.0, 1.0),
                           (320.0, 0.0, -0.5),
                           (280.0, 0.0, 0.5),
                           (390.0, 210.0, 0.0),
                           (280.0, 120.0, 0.5)], 26.0) =>
            [PlineProperties::new(12, 26880.50880023272, 879.9419421394236, 97.46410017370246, -36.5, 378.41586988912127, 165.65896506528978)]
        }
        closed_pline2 {
            (pline_closed![(112.41916161761486, 317.6090172318188, 0.374794619217547),
                           (283.91125822540016, 113.83906801254867, -1.0),
                           (320.0, 0.0, -0.5),
                           (416.19973184838693, -118.5880908230576, 0.5),
                           (390.0, 210.0, 0.0),
                           (280.0, 120.0, 0.5)], 11.0) =>
            [PlineProperties::new(4, 22967.88418361544, 725.8310555703592, 306.51750709258783, -88.76884688852556, 474.8986719957476, 196.35636086795802),
             PlineProperties::new(2, 14876.690185910866, 512.884459926642, 123.52142671939367, 127.7080000599289, 273.04351973980494, 306.89237970059116)]
        }
        closed_pline3 {
            (pline_closed![(-225.0, 0.0, 0.0),
                           (280.0, 0.0, 0.0),
                           (390.0, 200.0, 0.0),
                           (310.0, 200.0, 1.0),
                           (270.0, 200.0, -1.0),
                           (280.0, 200.0, -1.0),
                           (150.0, 200.0, 0.0),
                           (-340.0, 200.0, 0.0)], 16.0) =>
            [PlineProperties::new(7, 89881.66357519358, 1621.6223053868894, -312.34356480790507, 16.0, 362.93966046317865, 192.8544998953781)]
        }
        closed_pline4 {
            (pline_closed![(100.0, 100.0, -0.5),
                           (80.0, 90.0, 0.374794619217547),
                           (210.0, 0.0, 0.0),
                           (230.0, 0.0, 1.0),
                           (320.0, 0.0, -0.5),
                           (280.0, 0.0, 0.5),
                           (390.0, 210.0, 0.0),
                           (280.0, 120.0, 0.5)], -9.0) =>
            [PlineProperties::new(11, 53340.59364855598, 1008.1487200240091, 71.0, -54.00000000000001, 413.41586988912127, 219.0)]
        }
    );
}
