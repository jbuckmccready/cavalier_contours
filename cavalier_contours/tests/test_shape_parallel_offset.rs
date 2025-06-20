mod test_utils;

use cavalier_contours::{polyline::Polyline, shape_algorithms::Shape};
use test_utils::{PlineProperties, create_property_set};

use crate::test_utils::property_sets_match;

fn run_shape_offset_tests<I>(input: I, offset: f64, expected_properties_set: &[PlineProperties])
where
    I: IntoIterator<Item = Polyline>,
{
    let s = Shape::from_plines(input);
    let result = s.parallel_offset(offset, Default::default());
    let plines = result
        .ccw_plines
        .iter()
        .chain(result.cw_plines.iter())
        .map(|p| &p.polyline);
    let result_properties = create_property_set(plines, false);

    assert!(
        property_sets_match(&result_properties, expected_properties_set),
        "result property sets do not match"
    )
}

macro_rules! declare_offset_tests {
    ($($name:ident { $($value:expr => $expected:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_shape_offset_tests($value.0, $value.1, &$expected);
                )+
            }
        )+
    };
}
mod test_simple {
    use super::*;
    use cavalier_contours::pline_closed_userdata;

    declare_offset_tests!(
        empty_returns_empty {
          (Vec::<Polyline>::new(), 5.0) => []
        }
        set_of_empty_returns_empty {
          ([Polyline::<f64>::new_closed(), Polyline::new_closed()], 5.0) => []
        }
        rectangle_inside_shape {
            ([pline_closed_userdata![[4], (100.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
              pline_closed_userdata![[117], (150.0, 50.0, 0.0), (150.0, 100.0, 0.0), (200.0, 100.0, 0.0), (200.0, 50.0, 0.0)]], 3.0) =>
             [PlineProperties::new(12, 40977.79061358948, 998.5536075336107, 84.32384698504309, -41.99999999999997, 401.41586988912127, 205.22199935960901, vec![4]),
              PlineProperties::new(8, -3128.274333882308, 218.84955592153878, 147.0, 47.0, 203.0, 103.0, vec![117])]
        }
    );
}

mod test_specific {
    use super::*;
    use cavalier_contours::pline_closed_userdata;

    declare_offset_tests!(
        case1 {
            ([pline_closed_userdata![[4], (100.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
              pline_closed_userdata![[117], (150.0, 50.0, 0.0), (146.32758944101474, 104.13867601941358, 0.0), (200.0, 100.0, 0.0), (200.0, 50.0, 0.0)]], 17.0) =>
             [PlineProperties::new(22, 20848.93377998434, 1149.2701898185926, 102.79564651409214, -28.000000000000004, 387.41586988912127, 181.8843855860552, vec![4, 117])]
        }
        case2 {
            ([pline_closed_userdata![[4], (160.655879768138, 148.75471430537402, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
              pline_closed_userdata![[117], (150.0, 50.0, 0.0), (192.62381977774953, 130.82800839110848, 0.0), (200.0, 100.0, 0.0), (200.0, 50.0, 0.0)]], 17.0) =>
             [PlineProperties::new(20, 20135.256681247833, 1053.2414865948808, 105.64684517241575, -28.000000000000004, 387.41586988912127, 181.8843855860552, vec![4, 117]),
              PlineProperties::new(4, 2.091291658768, 9.557331573939933, 176.64810774674345, 136.97815392110508, 178.9335673169721, 140.906549335123, vec![4, 117])]
        }
    );
}
