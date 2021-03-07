use cavalier_contours_ffi::{
    cavc_pline, cavc_pline_add, cavc_pline_clear, cavc_pline_create, cavc_pline_eval_area,
    cavc_pline_eval_path_length, cavc_pline_f, cavc_pline_get_is_closed, cavc_pline_get_vertex,
    cavc_pline_get_vertex_count, cavc_pline_get_vertex_data, cavc_pline_remove,
    cavc_pline_set_is_closed, cavc_pline_set_vertex_data, cavc_vertex,
};
use std::ptr;

mod macros;

fn create_pline(vertexes: &[(f64, f64, f64)], is_closed: bool) -> *mut cavc_pline {
    let mut buffer = Vec::with_capacity(vertexes.len());
    for &(x, y, bulge) in vertexes {
        buffer.push(cavc_vertex {
            data: [x, y, bulge],
        });
    }

    let mut result = ptr::null_mut();
    let err = unsafe {
        cavc_pline_create(
            buffer.as_ptr(),
            buffer.len() as u32,
            if is_closed { 1 } else { 0 },
            &mut result,
        )
    };
    assert_eq!(err, 0);

    result
}

#[test]
fn pline_data_manipulation() {
    let pline = create_pline(&[], true);
    let null_ptr = ptr::null_mut();
    unsafe {
        // test pline is closed
        let mut is_closed: u8 = 0;
        assert_eq!(cavc_pline_get_is_closed(pline, &mut is_closed), 0);
        assert_ne!(is_closed, 0);

        // set pline to be open
        assert_eq!(cavc_pline_set_is_closed(pline, 0), 0);
        assert_eq!(cavc_pline_get_is_closed(pline, &mut is_closed), 0);
        assert_eq!(is_closed, 0);

        // set vertex data
        let vertex_data = [
            cavc_vertex::new(-1.0, -2.0, 0.0),
            cavc_vertex::new(-3.0, -4.0, -1.0),
        ];

        assert_eq!(
            cavc_pline_set_vertex_data(pline, vertex_data.as_ptr(), vertex_data.len() as u32),
            0
        );

        let mut count: u32 = 0;
        assert_eq!(cavc_pline_get_vertex_count(pline, &mut count), 0);
        assert_eq!(count, 2);

        // read all vertex data
        let mut data_out = [cavc_vertex::new(0.0, 0.0, 0.0); 2];
        assert_eq!(cavc_pline_get_vertex_data(pline, data_out.as_mut_ptr()), 0);
        assert_eq!(data_out[0].x(), -1.0);
        assert_eq!(data_out[0].y(), -2.0);
        assert_eq!(data_out[0].bulge(), 0.0);
        assert_eq!(data_out[1].x(), -3.0);
        assert_eq!(data_out[1].y(), -4.0);
        assert_eq!(data_out[1].bulge(), -1.0);

        // clear vertexes
        assert_eq!(cavc_pline_clear(pline), 0);
        let mut count: u32 = 0;
        assert_eq!(cavc_pline_get_vertex_count(pline, &mut count), 0);
        assert_eq!(count, 0);

        // add vertexes
        assert_eq!(cavc_pline_add(null_ptr, 0.0, 0.0, 0.0), 1);
        assert_eq!(cavc_pline_add(pline, 1.0, 2.0, 0.0), 0);
        assert_eq!(cavc_pline_add(pline, 3.0, 4.0, 1.0), 0);

        // get vertex count
        let mut count: u32 = 0;
        assert_eq!(cavc_pline_get_vertex_count(null_ptr, &mut count), 1);
        assert_eq!(cavc_pline_get_vertex_count(pline, &mut count), 0);
        assert_eq!(count, 2);

        // read vertex at positions
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(null_ptr, 0, &mut v), 1);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.bulge(), 0.0);

        assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
        assert_eq!(v.x(), 3.0);
        assert_eq!(v.y(), 4.0);
        assert_eq!(v.bulge(), 1.0);

        // index position out of bounds
        assert_eq!(cavc_pline_get_vertex(pline, 3, &mut v), 2);

        // remove vertex at position
        assert_eq!(cavc_pline_remove(pline, 0), 0);
        let mut count: u32 = 0;
        assert_eq!(cavc_pline_get_vertex_count(pline, &mut count), 0);
        assert_eq!(count, 1);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x(), 3.0);
        assert_eq!(v.y(), 4.0);
        assert_eq!(v.bulge(), 1.0);
    }
}

mod test_area {
    use super::*;

    macro_rules! declare_area_tests {
        ($($name:ident { $($value:expr => $expected:expr,)+ })*) => {
            $(
                #[test]
                fn $name() {
                    $(
                        let pline = create_pline(&$value, true);
                        let mut a = std::f64::NAN;
                        unsafe {
                            assert_eq!(cavc_pline_eval_area(pline, &mut a), 0);
                        }
                        assert_fuzzy_eq!(a, $expected);
                        unsafe {
                            cavc_pline_f(pline);
                        }
                    )+
                }
            )+
        };
    }

    declare_area_tests!(
        empty_pline {
            [] => 0.0,
        }
        single_vertex {
            [(1.0, 1.0, 0.0)] => 0.0,
        }
        ccw_circle {
            [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)] => std::f64::consts::PI,
        }
        cw_circle {
            [(0.0, 0.0, -1.0), (2.0, 0.0, -1.0)] => -std::f64::consts::PI,
        }
        ccw_half_circle {
            [(0.0, 0.0, 1.0), (-2.0, 0.0, 0.0)] => std::f64::consts::PI / 2.0,
        }
        cw_half_circle {
            [(0.0, 0.0, -1.0), (-2.0, 0.0, 0.0)] => -std::f64::consts::PI / 2.0,
        }
        ccw_rectangle {
            [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)] => 4.0,
        }
        cw_rectangle {
            [(-1.0, -1.0, 0.0), (-1.0, 1.0, 0.0), (1.0, 1.0, 0.0), (1.0, -1.0, 0.0)] => -4.0,
        }
    );
}

mod test_path_length {
    use super::*;

    macro_rules! declare_path_length_tests {
        ($($name:ident { $($is_closed:expr, $value:expr => $expected:expr,)+ })*) => {
            $(
                #[test]
                fn $name() {
                    $(
                        let pline = create_pline(&$value, $is_closed);
                        let mut path_length = std::f64::NAN;
                        unsafe {
                            assert_eq!(cavc_pline_eval_path_length(pline, &mut path_length), 0);
                        }
                        assert_fuzzy_eq!(path_length, $expected);
                        unsafe {
                            cavc_pline_f(pline);
                        }
                    )+
                }
            )+
        };
    }

    declare_path_length_tests!(
        empty_pline {
            false, [] => 0.0,
        }
        single_vertex {
            false, [(1.0, 1.0, 0.0)] => 0.0,
        }
        ccw_arc {
            false, [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)] => std::f64::consts::PI,
        }
        ccw_circle {
            true, [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)] => 2.0 * std::f64::consts::PI,
        }
        cw_arc {
            false, [(0.0, 0.0, -1.0), (2.0, 0.0, -1.0)] => std::f64::consts::PI,
        }
        cw_circle {
            true, [(0.0, 0.0, -1.0), (2.0, 0.0, -1.0)] => 2.0 * std::f64::consts::PI,
        }
        ccw_rectangle {
            true, [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)] => 8.0,
        }
        ccw_rectangle_open {
            false, [(-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0)] => 6.0,
        }
        cw_rectangle {
            true, [(-1.0, -1.0, 0.0), (-1.0, 1.0, 0.0), (1.0, 1.0, 0.0), (1.0, -1.0, 0.0)] => 8.0,
        }
        cw_rectangle_open {
            false, [(-1.0, -1.0, 0.0), (-1.0, 1.0, 0.0), (1.0, 1.0, 0.0), (1.0, -1.0, 0.0)] => 6.0,
        }
    );
}
