use cavalier_contours::assert_fuzzy_eq;
use cavalier_contours_ffi::*;
use std::ptr;

fn create_pline(vertexes: &[(f64, f64, f64)], is_closed: bool) -> *mut cavc_pline {
    let mut buffer = Vec::with_capacity(vertexes.len());
    for &(x, y, bulge) in vertexes {
        buffer.push(cavc_vertex::new(x, y, bulge));
    }

    let mut result = ptr::null();
    let err = unsafe {
        cavc_pline_create(
            buffer.as_ptr(),
            buffer.len() as u32,
            if is_closed { 1 } else { 0 },
            &mut result,
        )
    };
    assert_eq!(err, 0);

    result as *mut _
}

#[test]
fn pline_data_manipulation() {
    let pline = create_pline(&[], true);
    let null_ptr = ptr::null_mut();
    unsafe {
        // test reserve
        assert_eq!(cavc_pline_reserve(pline, 5), 0);
        assert_eq!(cavc_pline_reserve(null_ptr, 5), 1);

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

        // read all vertex data
        let mut data_out = [cavc_vertex::new(0.0, 0.0, 0.0); 2];
        assert_eq!(cavc_pline_get_vertex_data(pline, data_out.as_mut_ptr()), 0);
        assert_eq!(data_out[0].x, -1.0);
        assert_eq!(data_out[0].y, -2.0);
        assert_eq!(data_out[0].bulge, 0.0);
        assert_eq!(data_out[1].x, -3.0);
        assert_eq!(data_out[1].y, -4.0);
        assert_eq!(data_out[1].bulge, -1.0);

        // clone
        let mut cloned = ptr::null();
        assert_eq!(cavc_pline_clone(pline, &mut cloned), 0);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(cloned, 1, &mut v), 0);
        assert_eq!(v.x, -3.0);
        assert_eq!(v.y, -4.0);
        assert_eq!(v.bulge, -1.0);
        assert_eq!(cavc_pline_clone(null_ptr, &mut cloned), 1);
        cavc_pline_f(cloned as *mut _);

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
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.bulge, 0.0);

        assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
        assert_eq!(v.bulge, 1.0);

        // get index position out of bounds
        assert_eq!(cavc_pline_get_vertex(pline, 3, &mut v), 2);

        // set vertex at position
        assert_eq!(
            cavc_pline_set_vertex(pline, 0, cavc_vertex::new(8.0, 8.0, 0.55)),
            0
        );

        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x, 8.0);
        assert_eq!(v.y, 8.0);
        assert_eq!(v.bulge, 0.55);

        // set index position out of bounds
        assert_eq!(
            cavc_pline_set_vertex(pline, 3, cavc_vertex::new(0.0, 0.0, 0.0)),
            2
        );

        // remove vertex at position
        assert_eq!(cavc_pline_remove(pline, 0), 0);
        let mut count: u32 = 0;
        assert_eq!(cavc_pline_get_vertex_count(pline, &mut count), 0);
        assert_eq!(count, 1);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
        assert_eq!(v.bulge, 1.0);

        cavc_pline_f(pline);
    }
}

#[test]
fn pline_eval_path_length() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let mut l = std::f64::NAN;
    unsafe {
        assert_eq!(cavc_pline_eval_path_length(pline, &mut l), 0);
        assert_eq!(cavc_pline_eval_path_length(ptr::null_mut(), &mut l), 1);
    }
    assert_fuzzy_eq!(l, std::f64::consts::TAU);
    unsafe { cavc_pline_f(pline) }
}

#[test]
fn pline_eval_area() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let mut a = std::f64::NAN;
    unsafe {
        assert_eq!(cavc_pline_eval_area(pline, &mut a), 0);
        assert_eq!(cavc_pline_eval_area(ptr::null_mut(), &mut a), 1);
    }
    assert_fuzzy_eq!(a, std::f64::consts::PI);
    unsafe { cavc_pline_f(pline) }
}

#[test]
fn pline_eval_wn() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let (x, y) = (1.0, 0.0);
    let mut wn = i32::MAX;
    unsafe {
        assert_eq!(cavc_pline_eval_wn(pline, x, y, &mut wn), 0);
        assert_eq!(cavc_pline_eval_wn(ptr::null_mut(), x, y, &mut wn), 1);
    }
    assert_eq!(wn, 1);
    unsafe { cavc_pline_f(pline) }
}

#[test]
fn pline_invert_direction() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    unsafe {
        assert_eq!(cavc_pline_invert_direction(pline), 0);
        assert_eq!(cavc_pline_invert_direction(ptr::null_mut()), 1);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x, 2.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.bulge, -1.0);
        assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.bulge, -1.0);

        cavc_pline_f(pline)
    }
}

#[test]
fn pline_scale() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let scale_factor = 2.0;
    unsafe {
        assert_eq!(cavc_pline_scale(pline, scale_factor), 0);
        assert_eq!(cavc_pline_scale(ptr::null_mut(), scale_factor), 1);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.bulge, 1.0);
        assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
        assert_eq!(v.x, 4.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.bulge, 1.0);

        cavc_pline_f(pline)
    }
}

#[test]
fn pline_translate() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let (x_offset, y_offset) = (1.0, 1.0);
    unsafe {
        assert_eq!(cavc_pline_translate(pline, x_offset, y_offset), 0);
        assert_eq!(cavc_pline_translate(ptr::null_mut(), x_offset, y_offset), 1);
        let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
        assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 1.0);
        assert_eq!(v.bulge, 1.0);
        assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 1.0);
        assert_eq!(v.bulge, 1.0);

        cavc_pline_f(pline)
    }
}

#[test]
fn pline_remove_repeat_pos() {
    // no vertexes removed
    {
        let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
        let pos_equal_eps = 1e-5;
        unsafe {
            assert_eq!(cavc_pline_remove_repeat_pos(pline, pos_equal_eps), 0);
            assert_eq!(
                cavc_pline_remove_repeat_pos(ptr::null_mut(), pos_equal_eps),
                1
            );
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
            assert_eq!(v.x, 0.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 1.0);
            assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
            assert_eq!(v.x, 2.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 1.0);

            cavc_pline_f(pline)
        }
    }

    // vertex removed
    {
        let pline = create_pline(&[(0.0, 0.0, 1.0), (0.0, 0.0, 0.5), (2.0, 0.0, 1.0)], true);
        let pos_equal_eps = 1e-5;
        unsafe {
            assert_eq!(cavc_pline_remove_repeat_pos(pline, pos_equal_eps), 0);
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
            assert_eq!(v.x, 0.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 0.5);
            assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
            assert_eq!(v.x, 2.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 1.0);

            cavc_pline_f(pline)
        }
    }
}

#[test]
fn pline_remove_redundant() {
    // no vertexes removed
    {
        let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
        let pos_equal_eps = 1e-5;
        unsafe {
            assert_eq!(cavc_pline_remove_redundant(pline, pos_equal_eps), 0);
            assert_eq!(
                cavc_pline_remove_redundant(ptr::null_mut(), pos_equal_eps),
                1
            );
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
            assert_eq!(v.x, 0.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 1.0);
            assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
            assert_eq!(v.x, 2.0);
            assert_eq!(v.y, 0.0);
            assert_eq!(v.bulge, 1.0);

            cavc_pline_f(pline)
        }
    }

    // vertex removed
    {
        let bulge = (std::f64::consts::FRAC_PI_2 / 4.0).tan();
        let pline = create_pline(
            &[(0.0, 0.0, bulge), (1.0, -1.0, bulge), (2.0, 0.0, 1.0)],
            true,
        );
        let pos_equal_eps = 1e-5;
        unsafe {
            assert_eq!(cavc_pline_remove_redundant(pline, pos_equal_eps), 0);
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(pline, 0, &mut v), 0);
            assert_fuzzy_eq!(v.x, 0.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);
            assert_eq!(cavc_pline_get_vertex(pline, 1, &mut v), 0);
            assert_fuzzy_eq!(v.x, 2.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);

            cavc_pline_f(pline)
        }
    }
}

#[test]
fn pline_eval_extents() {
    let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
    let (mut min_x, mut min_y, mut max_x, mut max_y) =
        (std::f64::NAN, std::f64::NAN, std::f64::NAN, std::f64::NAN);
    unsafe {
        assert_eq!(
            cavc_pline_eval_extents(pline, &mut min_x, &mut min_y, &mut max_x, &mut max_y),
            0
        );
        assert_eq!(
            cavc_pline_eval_extents(
                ptr::null_mut(),
                &mut min_x,
                &mut min_y,
                &mut max_x,
                &mut max_y
            ),
            1
        );
    }
    assert_fuzzy_eq!(min_x, 0.0);
    assert_fuzzy_eq!(min_y, -1.0);
    assert_fuzzy_eq!(max_x, 2.0);
    assert_fuzzy_eq!(max_y, 1.0);

    unsafe { cavc_pline_f(pline) }
}

#[test]
fn pline_eval_parallel_offset() {
    // null options
    {
        let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
        let offset = -1.0;
        let mut results = ptr::null();
        unsafe {
            assert_eq!(
                cavc_pline_parallel_offset(pline, offset, ptr::null_mut(), &mut results),
                0
            );

            assert_eq!(
                cavc_pline_parallel_offset(ptr::null_mut(), offset, ptr::null_mut(), &mut results),
                1
            );

            let mut results_count = u32::MAX;
            assert_eq!(cavc_plinelist_get_count(results, &mut results_count), 0);
            assert_eq!(results_count, 1);

            let mut result_pline = ptr::null();
            assert_eq!(cavc_plinelist_get_pline(results, 0, &mut result_pline), 0);
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(result_pline, 0, &mut v), 0);
            assert_fuzzy_eq!(v.x, -1.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);
            assert_eq!(cavc_pline_get_vertex(result_pline, 1, &mut v), 0);
            assert_fuzzy_eq!(v.x, 3.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);

            cavc_plinelist_f(results as *mut _);

            cavc_pline_f(pline);
        }
    }

    // with options
    {
        let pline = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);
        let offset = -1.0;
        let mut options = cavc_pline_parallel_offset_o {
            aabb_index: std::ptr::null(),
            pos_equal_eps: std::f64::NAN,
            slice_join_eps: std::f64::NAN,
            offset_dist_eps: std::f64::NAN,
            handle_self_intersects: 0,
        };

        let mut results = ptr::null();
        unsafe {
            assert_eq!(cavc_pline_parallel_offset_o_init(&mut options), 0);
            assert!(!options.pos_equal_eps.is_nan());
            assert!(!options.slice_join_eps.is_nan());
            assert!(!options.offset_dist_eps.is_nan());

            let mut aabb_index = ptr::null();

            assert_eq!(
                cavc_pline_create_approx_aabbindex(pline, &mut aabb_index),
                0
            );

            options.aabb_index = aabb_index;

            assert_eq!(
                cavc_pline_parallel_offset(pline, offset, &options, &mut results),
                0
            );

            assert_eq!(
                cavc_pline_parallel_offset(ptr::null_mut(), offset, &options, &mut results),
                1
            );

            let mut results_count = u32::MAX;
            assert_eq!(cavc_plinelist_get_count(results, &mut results_count), 0);
            assert_eq!(results_count, 1);

            let mut result_pline = ptr::null();
            assert_eq!(cavc_plinelist_get_pline(results, 0, &mut result_pline), 0);
            let mut v = cavc_vertex::new(0.0, 0.0, 0.0);
            assert_eq!(cavc_pline_get_vertex(result_pline, 0, &mut v), 0);
            assert_fuzzy_eq!(v.x, -1.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);
            assert_eq!(cavc_pline_get_vertex(result_pline, 1, &mut v), 0);
            assert_fuzzy_eq!(v.x, 3.0);
            assert_fuzzy_eq!(v.y, 0.0);
            assert_fuzzy_eq!(v.bulge, 1.0);

            cavc_plinelist_f(results as *mut _);

            cavc_aabbindex_f(aabb_index as *mut _);

            cavc_pline_f(pline);
        }
    }
}

#[test]
fn pline_eval_boolean() {
    // null options
    {
        // 4x4 square
        let pline1 = create_pline(
            &[
                (-1.0, -2.0, 0.0),
                (3.0, -2.0, 0.0),
                (3.0, 2.0, 0.0),
                (-1.0, 2.0, 0.0),
            ],
            true,
        );
        // circle inside square with radius 1
        let pline2 = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);

        let mut pos_plines = ptr::null();
        let mut neg_plines = ptr::null();

        unsafe {
            assert_eq!(
                cavc_pline_boolean(
                    pline1,
                    pline2,
                    2,
                    ptr::null_mut(),
                    &mut pos_plines,
                    &mut neg_plines
                ),
                0
            );

            assert_eq!(
                cavc_pline_boolean(
                    ptr::null(),
                    ptr::null(),
                    2,
                    ptr::null_mut(),
                    &mut pos_plines,
                    &mut neg_plines
                ),
                1
            );
            assert_eq!(
                cavc_pline_boolean(
                    pline1,
                    ptr::null(),
                    2,
                    ptr::null_mut(),
                    &mut pos_plines,
                    &mut neg_plines
                ),
                1
            );
            assert_eq!(
                cavc_pline_boolean(
                    ptr::null(),
                    pline2,
                    2,
                    ptr::null_mut(),
                    &mut pos_plines,
                    &mut neg_plines
                ),
                1
            );

            let mut pos_plines_count = u32::MAX;
            assert_eq!(
                cavc_plinelist_get_count(pos_plines, &mut pos_plines_count),
                0
            );
            assert_eq!(pos_plines_count, 1);

            let mut neg_plines_count = u32::MAX;
            assert_eq!(
                cavc_plinelist_get_count(pos_plines, &mut neg_plines_count),
                0
            );
            assert_eq!(neg_plines_count, 1);

            let mut output_pline = ptr::null();
            assert_eq!(
                cavc_plinelist_get_pline(pos_plines, 0, &mut output_pline),
                0
            );

            let mut area = std::f64::NAN;
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, 16.0);

            assert_eq!(
                cavc_plinelist_get_pline(neg_plines, 0, &mut output_pline),
                0
            );
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, std::f64::consts::PI);

            // test take on the plinelist
            // null ptr
            assert_eq!(
                cavc_plinelist_take(ptr::null_mut(), 0, &mut output_pline),
                1
            );
            // index position out of range
            assert_eq!(
                cavc_plinelist_take(neg_plines as *mut _, 1, &mut output_pline),
                2
            );
            assert_eq!(
                cavc_plinelist_take(neg_plines as *mut _, 0, &mut output_pline),
                0
            );

            let mut area = 0.0;
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, std::f64::consts::PI);
            let mut count = u32::MAX;
            assert_eq!(cavc_plinelist_get_count(neg_plines, &mut count), 0);
            assert_eq!(count, 0);
            cavc_pline_f(output_pline as *mut _);

            // test pop on plinelist
            // null ptr
            assert_eq!(cavc_plinelist_pop(ptr::null_mut(), &mut output_pline), 1);

            assert_eq!(
                cavc_plinelist_pop(pos_plines as *mut _, &mut output_pline),
                0
            );
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, 16.0);
            let mut count = u32::MAX;
            assert_eq!(cavc_plinelist_get_count(pos_plines, &mut count), 0);
            assert_eq!(count, 0);
            cavc_pline_f(output_pline as *mut _);

            // empty plinelist
            assert_eq!(
                cavc_plinelist_pop(pos_plines as *mut _, &mut output_pline),
                2
            );

            cavc_plinelist_f(pos_plines as *mut _);
            cavc_plinelist_f(neg_plines as *mut _);

            cavc_pline_f(pline1);
        }
    }

    // with options
    {
        // 4x4 square
        let pline1 = create_pline(
            &[
                (-1.0, -2.0, 0.0),
                (3.0, -2.0, 0.0),
                (3.0, 2.0, 0.0),
                (-1.0, 2.0, 0.0),
            ],
            true,
        );
        // circle inside square with radius 1
        let pline2 = create_pline(&[(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)], true);

        let mut pos_plines = ptr::null();
        let mut neg_plines = ptr::null();

        let mut options = cavc_pline_boolean_o {
            pline1_aabb_index: std::ptr::null(),
            pos_equal_eps: std::f64::NAN,
        };

        unsafe {
            assert_eq!(cavc_pline_boolean_o_init(&mut options), 0);
            assert!(!options.pos_equal_eps.is_nan());

            let mut pline1_aabb_index = ptr::null();

            assert_eq!(
                cavc_pline_create_approx_aabbindex(pline1, &mut pline1_aabb_index),
                0
            );

            options.pline1_aabb_index = pline1_aabb_index;

            assert_eq!(
                cavc_pline_boolean(
                    pline1,
                    pline2,
                    2,
                    &options,
                    &mut pos_plines,
                    &mut neg_plines
                ),
                0
            );

            let mut pos_plines_count = u32::MAX;
            assert_eq!(
                cavc_plinelist_get_count(pos_plines, &mut pos_plines_count),
                0
            );
            assert_eq!(pos_plines_count, 1);

            let mut neg_plines_count = u32::MAX;
            assert_eq!(
                cavc_plinelist_get_count(neg_plines, &mut neg_plines_count),
                0
            );
            assert_eq!(neg_plines_count, 1);

            let mut output_pline = ptr::null();
            assert_eq!(
                cavc_plinelist_get_pline(pos_plines, 0, &mut output_pline),
                0
            );

            let mut area = std::f64::NAN;
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, 16.0);

            assert_eq!(
                cavc_plinelist_get_pline(neg_plines, 0, &mut output_pline),
                0
            );
            assert_eq!(cavc_pline_eval_area(output_pline, &mut area), 0);
            assert_fuzzy_eq!(area, std::f64::consts::PI);

            cavc_plinelist_f(pos_plines as *mut _);
            cavc_plinelist_f(neg_plines as *mut _);

            cavc_pline_f(pline1);
        }
    }
}
