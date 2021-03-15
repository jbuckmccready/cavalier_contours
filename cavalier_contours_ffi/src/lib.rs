//! This module contains the C foreign function interface for cavalier_contours.
#![allow(non_camel_case_types)]
mod error_handling;
use cavalier_contours::{PlineVertex, Polyline, Vector2};
use core::slice;
use error_handling::{set_last_error, LAST_ERROR};
use std::{convert::TryFrom, ffi::CStr, os::raw::c_char, panic};

/// Helper macro to catch unwind and return -1 if panic was caught otherwise returns whatever the
/// expression returned.
macro_rules! ffi_catch_unwind {
    ($body: expr) => {
        match panic::catch_unwind(move || $body) {
            Ok(r) => r,
            Err(_) => {
                set_last_error("Internal algorithm/library error occurred.", "");
                -1
            }
        }
    };
}
/// Represents a polyline vertex holding x, y, and bulge.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_vertex {
    pub data: [f64; 3],
}

impl cavc_vertex {
    pub fn new(x: f64, y: f64, bulge: f64) -> Self {
        cavc_vertex {
            data: [x, y, bulge],
        }
    }

    pub fn x(&self) -> f64 {
        self.data[0]
    }

    pub fn y(&self) -> f64 {
        self.data[1]
    }

    pub fn bulge(&self) -> f64 {
        self.data[2]
    }

    fn from_internal(v: PlineVertex<f64>) -> Self {
        cavc_vertex::new(v.x, v.y, v.bulge)
    }
}

/// Represents a basic 2D point holding x and y.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_point {
    pub data: [f64; 2],
}

/// Polyline object type.
#[derive(Debug, Clone)]
pub struct cavc_pline(Polyline<f64>);

/// Gets the last error message set as a null terminated c string.
///
/// The c string returned is in thread local storage and is only valid until another function that
/// can error is called!
pub extern "C" fn cavc_last_error_msg() -> *const c_char {
    // Safety: constructing CStr from static empty null terminated string, no null check required.
    let mut result = unsafe { CStr::from_bytes_with_nul_unchecked(b"0").as_ptr() };
    LAST_ERROR.with(|last_error| {
        if let Some(error_data) = last_error.borrow().as_ref() {
            // Safety: Constructing CStr from thread local null terminated string, no null check
            // required.
            unsafe {
                result =
                    CStr::from_bytes_with_nul_unchecked(&error_data.error_msg.as_bytes_with_nul())
                        .as_ptr();
            }
        }
    });

    result
}

/// Gets the last error report data set as a null terminated c string.
///
/// The c string returned is in thread local storage and is only valid until another function that
/// can error is called!
pub extern "C" fn cavc_last_error_report() -> *const c_char {
    // Safety: constructing CStr from static empty null terminated string, no null check required.
    let mut result = unsafe { CStr::from_bytes_with_nul_unchecked(b"0").as_ptr() };
    LAST_ERROR.with(|last_error| {
        if let Some(error_data) = last_error.borrow().as_ref() {
            // Safety: constructing CStr from static empty null terminated string, no null check
            // required.
            unsafe {
                result = CStr::from_bytes_with_nul_unchecked(
                    &error_data.error_report_data.as_bytes_with_nul(),
                )
                .as_ptr();
            }
        }
    });

    result
}

/// Create a new polyline object.
///
/// `vertexes` is an array of [cavc_vertex] to create the polyline with (may be null if `n_vertexes`
/// is 0).
/// `n_vertexes` contains the number of vertexes in the array.
/// `is_closed` sets the polyline to be closed if non-zero.
/// `pline` is an out parameter to hold the created polyline.
///
/// # Safety
///
/// `vertexes` may be null if `n_vertexes` is 0 or must point to a valid contiguous buffer of
/// [cavc_vertex] with length of at least `n_vertexes`.
/// `pline` must point to a pointer that can be safely assigned a reference to a cavc_pline.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_create(
    vertexes: *const cavc_vertex,
    n_vertexes: u32,
    is_closed: u8,
    pline: *mut *mut cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        let mut result = Polyline::new();
        if is_closed != 0 {
            result.set_is_closed(true);
        }

        if !vertexes.is_null() && n_vertexes != 0 {
            let data = slice::from_raw_parts(vertexes, n_vertexes as usize);
            for v in data {
                result.add_from_array(v.data);
            }
        }

        *pline = Box::into_raw(Box::new(cavc_pline(result)));
        0
    })
}

/// Free an existing polyline object.
///
/// Nothing happens if `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not already been freed.
#[no_mangle]
pub unsafe extern "C" fn cavc_pline_f(pline: *mut cavc_pline) {
    if !pline.is_null() {
        drop(Box::from_raw(pline))
    }
}

/// Get whether the polyline is closed or not.
///
/// `is_closed` is used as an out parameter to hold the whether `pline` is closed (non-zero) or not
/// (zero).
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_is_closed(
    pline: *const cavc_pline,
    is_closed: *mut u8,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        *is_closed = if (*pline).0.is_closed() { 1 } else { 0 };
        0
    })
}

/// Set whether the polyline is closed or not.
///
/// If `is_closed` is non-zero then `pline` is set to be closed, otherwise it is set to be open.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_set_is_closed(pline: *mut cavc_pline, is_closed: u8) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        (*pline).0.set_is_closed(is_closed != 0);
        0
    })
}

/// Get the vertex count of a polyline.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_vertex_count(
    pline: *const cavc_pline,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
        // prevent memory corruption/access errors but just panic as internal error if it does occur
        *count = u32::try_from((*pline).0.len()).unwrap();
        0
    })
}

/// Fills the buffer given with the vertex data of a polyline.
///
/// You must use [cavc_pline_get_vertex_count] to ensure the buffer given has adequate length
/// to be filled with all vertexes!
///
/// `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
/// overrun will happen.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_vertex_data(
    pline: *const cavc_pline,
    vertex_data: *mut cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let buffer = slice::from_raw_parts_mut(vertex_data, (*pline).0.len());
        for (i, v) in (*pline).0.iter().enumerate() {
            buffer[i] = cavc_vertex::new(v.x, v.y, v.bulge);
        }
        0
    })
}

/// Sets all of the vertexes of a polyline.
///
/// `vertex_data` is an array of vertexes to use for the polyline.
/// `n_vertexes` must specify the number of vertexes to be read from the
/// `vertex_data` array.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `vertex_data` must be a valid pointer to a buffer of at least `n_vertexes` of [cavc_vertex].
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_set_vertex_data(
    pline: *mut cavc_pline,
    vertex_data: *const cavc_vertex,
    n_vertexes: u32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        (*pline).0.clear();
        let buffer = slice::from_raw_parts(vertex_data, n_vertexes as usize);
        (*pline).0.reserve(buffer.len());
        for v in buffer {
            (*pline).0.add(v.x(), v.y(), v.bulge());
        }
        0
    })
}

/// Clears all of the vertexes of a polyline.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_clear(pline: *mut cavc_pline) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        (*pline).0.clear();
        0
    })
}

/// Add a vertex to a polyline `pline` with `x`, `y`, and `bulge`.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_add(pline: *mut cavc_pline, x: f64, y: f64, bulge: f64) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        (*pline).0.add(x, y, bulge);
        0
    })
}

/// Get a polyline vertex at a given index position.
///
/// `position` is is the index to get the vertex at.
/// `vertex` used as out parameter to hold the vertex retrieved.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `position` is out of bounds for the `pline` given.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `vertex` must point to valid place in memory for which a [cavc_vertex] can be written.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_vertex(
    pline: *const cavc_pline,
    position: u32,
    vertex: *mut cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        if position >= (*pline).0.len() as u32 {
            return 2;
        }

        let v = (*pline).0[position as usize];
        *vertex = cavc_vertex::from_internal(v);
        0
    })
}

/// Remove a vertex from a polyline at an index position.
///
/// `position` is the index of the vertex to be removed from the polyline.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `position` is out of bounds for the `pline` given.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_remove(pline: *mut cavc_pline, position: u32) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        if position as usize >= (*pline).0.len() {
            return 2;
        }

        (*pline).0.remove(position as usize);
        0
    })
}

/// Compute the path length of a polyline.
///
/// `path_length` is used as the out parameter to hold the computed path length.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_path_length(
    pline: *const cavc_pline,
    path_length: *mut f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        *path_length = (*pline).0.path_length();
        0
    })
}

/// Compute the signed area of a polyline.
///
/// If `pline` is an open polyline then the computed area is always 0.
/// If `pline` direction is counter clockwise then result is positive otherwise it is negative.
/// `area` is used as the out parameter to hold the computed area.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_area(pline: *const cavc_pline, area: *mut f64) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        *area = (*pline).0.area();
        0
    })
}

/// Compute the winding number for a closed polyline relative to a point.
///
/// If `pline` is an open polyline then 0 is always returned.
/// The winding number has a magnitude equal to the net number of times the `pline` winds around
/// the `point` given and its sign is positive if the windings are net counter clockwise or negative if
/// they are net clockwise.
///
/// `winding_number` is used as the out parameter to hold the computed winding number.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `winding_number` must point to a valid place in memory for which an i32 can be written.
#[no_mangle]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_wn(
    pline: *const cavc_pline,
    point: cavc_point,
    winding_number: *mut i32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        *winding_number = (*pline)
            .0
            .winding_number(Vector2::new(point.data[0], point.data[1]));
        0
    })
}
