//! This module contains the C foreign function interface for cavalier_contours.
#![allow(non_camel_case_types)]
use cavalier_contours::{
    core::math::Vector2,
    polyline::{
        BooleanOp, PlineBooleanOptions, PlineContainsOptions, PlineOffsetOptions,
        PlineSelfIntersectOptions, PlineSource, PlineSourceMut, PlineVertex, Polyline,
        SelfIntersectsInclude,
    },
    shape_algorithms::{Shape, ShapeOffsetOptions},
    static_aabb2d_index::StaticAABB2DIndex,
};
use core::slice;
use std::{convert::TryFrom, panic};

/// Helper macro to catch unwind and return -1 if panic was caught otherwise returns whatever the
/// expression returned.
macro_rules! ffi_catch_unwind {
    ($body: expr) => {
        match panic::catch_unwind(move || $body) {
            Ok(r) => r,
            Err(_) => -1,
        }
    };
}

/// Opaque type that wraps a [StaticAABB2DIndex].
///
/// Note the internal member is only public for composing in other Rust libraries wanting to use the
/// FFI opaque type as part of their FFI API.
#[derive(Debug, Clone)]
pub struct cavc_aabbindex(pub StaticAABB2DIndex<f64>);

/// Represents a simple 2D point with x and y coordinate values.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_point {
    pub x: f64,
    pub y: f64,
}

impl cavc_point {
    pub fn new(x: f64, y: f64) -> Self {
        cavc_point { x, y }
    }

    pub fn from_internal(v: Vector2<f64>) -> Self {
        cavc_point::new(v.x, v.y)
    }
}

/// Represents a polyline vertex holding x, y, and bulge.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_vertex {
    pub x: f64,
    pub y: f64,
    pub bulge: f64,
}

impl cavc_vertex {
    pub fn new(x: f64, y: f64, bulge: f64) -> Self {
        cavc_vertex { x, y, bulge }
    }

    pub fn from_internal(v: PlineVertex<f64>) -> Self {
        cavc_vertex::new(v.x, v.y, v.bulge)
    }
}

/// Opaque type that wraps a [Polyline].
///
/// Note the internal member is only public for composing in other Rust libraries wanting to use the
/// FFI opaque type as part of their FFI API.
#[derive(Debug, Clone)]
pub struct cavc_pline(pub Polyline<f64>);

/// FFI representation of [PlineOffsetOptions].
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_pline_parallel_offset_o {
    pub aabb_index: *const cavc_aabbindex,
    pub pos_equal_eps: f64,
    pub slice_join_eps: f64,
    pub offset_dist_eps: f64,
    pub handle_self_intersects: u8,
}

impl cavc_pline_parallel_offset_o {
    /// Convert FFI parallel offset options type to internal type.
    ///
    /// # Safety
    ///
    /// `aabb_index` field must be null or a valid pointer to a [cavc_aabbindex].
    pub unsafe fn to_internal(&self) -> PlineOffsetOptions<'_, f64> {
        let aabb_index = unsafe { self.aabb_index.as_ref().map(|w| &w.0) };
        PlineOffsetOptions {
            aabb_index,
            pos_equal_eps: self.pos_equal_eps,
            slice_join_eps: self.slice_join_eps,
            offset_dist_eps: self.offset_dist_eps,
            handle_self_intersects: self.handle_self_intersects != 0,
        }
    }
}

impl Default for cavc_pline_parallel_offset_o {
    fn default() -> Self {
        let d = PlineOffsetOptions::default();
        Self {
            aabb_index: std::ptr::null(),
            pos_equal_eps: d.pos_equal_eps,
            slice_join_eps: d.slice_join_eps,
            offset_dist_eps: d.offset_dist_eps,
            handle_self_intersects: d.handle_self_intersects as u8,
        }
    }
}

/// Create a new [cavc_pline_parallel_offset_o] object.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_parallel_offset_o_create(
    options: *mut *mut cavc_pline_parallel_offset_o,
) -> i32 {
    ffi_catch_unwind!({
        unsafe {
            let result = cavc_pline_parallel_offset_o::default();
            options.write(Box::into_raw(Box::new(result)));
        }
        0
    })
}

/// Free an existing [cavc_pline_parallel_offset_o] object.
///
/// Nothing happens if `options` is null.
///
/// Note that this does NOT free the aabb index that the [cavc_pline_parallel_offset_o] points to.
/// You need to do that by calling cavc_aabbindex_f() on the index pointer contained in the [cavc_pline_parallel_offset_o] object.
///
/// # Safety
///
/// `options` must be null or a valid cavc_pline_parallel_offset_o object that was created with [cavc_pline_parallel_offset_o_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_pline_parallel_offset_o_f(
    options: *mut cavc_pline_parallel_offset_o,
) {
    if !options.is_null() {
        unsafe {
            drop(Box::from_raw(options));
        }
    }
}

/// Write default option values to a [cavc_pline_parallel_offset_o].
///
/// ## Specific Error Codes
/// * 1 = `options` is null.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_parallel_offset_o_init(
    options: *mut cavc_pline_parallel_offset_o,
) -> i32 {
    ffi_catch_unwind!({
        if options.is_null() {
            return 1;
        }

        unsafe {
            options.write(Default::default());
        }
        0
    })
}

/// FFI representation of [PlineBooleanOptions].
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_pline_boolean_o {
    pub pline1_aabb_index: *const cavc_aabbindex,
    pub pos_equal_eps: f64,
    /// NOTE: optional parameter, set to NaN for None.
    pub collapsed_area_eps: f64,
}

impl cavc_pline_boolean_o {
    /// Convert FFI boolean options type to internal type.
    ///
    /// # Safety
    ///
    /// `pline1_aabb_index` field must be null or a valid pointer to a [cavc_aabbindex].
    pub unsafe fn to_internal(&self) -> PlineBooleanOptions<'_, f64> {
        let pline1_aabb_index = unsafe { self.pline1_aabb_index.as_ref().map(|w| &w.0) };
        PlineBooleanOptions {
            pline1_aabb_index,
            pos_equal_eps: self.pos_equal_eps,
            collapsed_area_eps: if self.collapsed_area_eps.is_nan() {
                None
            } else {
                Some(self.collapsed_area_eps)
            },
        }
    }
}

impl Default for cavc_pline_boolean_o {
    fn default() -> Self {
        let d = PlineBooleanOptions::default();
        Self {
            pline1_aabb_index: std::ptr::null(),
            pos_equal_eps: d.pos_equal_eps,
            collapsed_area_eps: f64::NAN,
        }
    }
}

/// Create a new [cavc_pline_boolean_o] object.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_boolean_o_create(
    options: *mut *mut cavc_pline_boolean_o,
) -> i32 {
    ffi_catch_unwind!({
        unsafe {
            let result = cavc_pline_boolean_o::default();
            options.write(Box::into_raw(Box::new(result)));
        }
        0
    })
}

/// Free an existing [cavc_pline_boolean_o] object.
///
/// Nothing happens if `options` is null.
///
/// Note that this does NOT free the aabb index that the [cavc_pline_boolean_o] points to.
/// You need to do that by calling cavc_aabbindex_f() on the index pointer contained in the [cavc_pline_boolean_o] object.
///
/// # Safety
///
/// `options` must be null or a valid cavc_pline_boolean_o object that was created with [cavc_pline_boolean_o_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_pline_boolean_o_f(options: *mut cavc_pline_boolean_o) {
    if !options.is_null() {
        unsafe {
            drop(Box::from_raw(options));
        }
    }
}

/// Write default option values to a [cavc_pline_boolean_o].
///
/// ## Specific Error Codes
/// * 1 = `options` is null.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_boolean_o_init(options: *mut cavc_pline_boolean_o) -> i32 {
    ffi_catch_unwind!({
        if options.is_null() {
            return 1;
        }

        unsafe {
            options.write(Default::default());
        }
        0
    })
}

fn boolean_op_from_u32(i: u32) -> Option<BooleanOp> {
    if i == 0 {
        Some(BooleanOp::Or)
    } else if i == 1 {
        Some(BooleanOp::And)
    } else if i == 2 {
        Some(BooleanOp::Not)
    } else if i == 3 {
        Some(BooleanOp::Xor)
    } else {
        None
    }
}

/// FFI representation of SelfIntersectsInclude enum
pub const CAVC_SELF_INTERSECTS_INCLUDE_ALL: u32 = 0;
pub const CAVC_SELF_INTERSECTS_INCLUDE_LOCAL: u32 = 1;
pub const CAVC_SELF_INTERSECTS_INCLUDE_GLOBAL: u32 = 2;

/// FFI representation of [PlineSelfIntersectOptions].
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_pline_self_intersect_o {
    pub pline_aabb_index: *const cavc_aabbindex,
    pub pos_equal_eps: f64,
    pub include: u32,
}

impl cavc_pline_self_intersect_o {
    /// Convert FFI self intersection options type to internal type.
    ///
    /// # Safety
    ///
    /// `pline_aabb_index` field must be null or a valid pointer to a [cavc_aabbindex].
    pub unsafe fn to_internal(&self) -> Option<PlineSelfIntersectOptions<'_, f64>> {
        let pline_aabb_index = unsafe { self.pline_aabb_index.as_ref().map(|w| &w.0) };
        let include_value = match self.include {
            0 => SelfIntersectsInclude::All,
            1 => SelfIntersectsInclude::Local,
            2 => SelfIntersectsInclude::Global,
            _ => {
                return None;
            }
        };

        Some(PlineSelfIntersectOptions {
            aabb_index: pline_aabb_index,
            pos_equal_eps: self.pos_equal_eps,
            include: include_value,
        })
    }
}

impl Default for cavc_pline_self_intersect_o {
    fn default() -> Self {
        let d = PlineSelfIntersectOptions::default();
        Self {
            pline_aabb_index: std::ptr::null(),
            pos_equal_eps: d.pos_equal_eps,
            include: d.include as u32,
        }
    }
}

/// Create a new [cavc_pline_self_intersect_o] object.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_self_intersect_o_create(
    options: *mut *mut cavc_pline_self_intersect_o,
) -> i32 {
    ffi_catch_unwind!({
        unsafe {
            let result = cavc_pline_self_intersect_o::default();
            options.write(Box::into_raw(Box::new(result)));
        }
        0
    })
}

/// Free an existing [cavc_pline_self_intersect_o] object.
///
/// Nothing happens if `options` is null.
///
/// Note that this does NOT free the aabb index that the [cavc_pline_self_intersect_o] points to.
/// You need to do that by calling cavc_aabbindex_f() on the index pointer contained in the [cavc_pline_self_intersect_o] object.
///
/// # Safety
///
/// `options` must be null or a valid cavc_pline_self_intersect_o object that was created with [cavc_pline_self_intersect_o_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_pline_self_intersect_o_f(options: *mut cavc_pline_self_intersect_o) {
    if !options.is_null() {
        unsafe {
            drop(Box::from_raw(options));
        }
    }
}

/// Write default option values to a [cavc_pline_self_intersect_o].
///
/// ## Specific Error Codes
/// * 1 = `options` is null.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_self_intersect_o_init(
    options: *mut cavc_pline_self_intersect_o,
) -> i32 {
    ffi_catch_unwind!({
        if options.is_null() {
            return 1;
        }

        unsafe {
            options.write(Default::default());
        }
        0
    })
}

/// FFI representation of [PlineContainsOptions].
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_pline_contains_o {
    pub pline1_aabb_index: *const cavc_aabbindex,
    pub pos_equal_eps: f64,
}

impl cavc_pline_contains_o {
    /// Convert FFI contains options type to internal type.
    ///
    /// # Safety
    ///
    /// `pline1_aabb_index` field must be null or a valid pointer to a [cavc_aabbindex].
    pub unsafe fn to_internal(&self) -> PlineContainsOptions<'_, f64> {
        let pline1_aabb_index = unsafe { self.pline1_aabb_index.as_ref().map(|w| &w.0) };
        PlineContainsOptions {
            pline1_aabb_index,
            pos_equal_eps: self.pos_equal_eps,
        }
    }
}

impl Default for cavc_pline_contains_o {
    fn default() -> Self {
        let d = PlineContainsOptions::default();
        Self {
            pline1_aabb_index: std::ptr::null(),
            pos_equal_eps: d.pos_equal_eps,
        }
    }
}

/// Create a new [cavc_pline_contains_o] object.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_contains_o_create(
    options: *mut *mut cavc_pline_contains_o,
) -> i32 {
    ffi_catch_unwind!({
        unsafe {
            let result = cavc_pline_contains_o::default();
            options.write(Box::into_raw(Box::new(result)));
        }
        0
    })
}

/// Free an existing [cavc_pline_contains_o] object.
///
/// Nothing happens if `options` is null.
///
/// Note that this does NOT free the aabb index that the [cavc_pline_contains_o] points to.
/// You need to do that by calling cavc_aabbindex_f() on the index pointer contained in the [cavc_pline_contains_o] object.
///
/// # Safety
///
/// `options` must be null or a valid cavc_pline_contains_o object that was created with [cavc_pline_contains_o_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_pline_contains_o_f(options: *mut cavc_pline_contains_o) {
    if !options.is_null() {
        unsafe {
            drop(Box::from_raw(options));
        }
    }
}

/// Write default option values to a [cavc_pline_contains_o].
///
/// ## Specific Error Codes
/// * 1 = `options` is null.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_contains_o_init(options: *mut cavc_pline_contains_o) -> i32 {
    ffi_catch_unwind!({
        if options.is_null() {
            return 1;
        }

        unsafe {
            options.write(Default::default());
        }
        0
    })
}

/// Opaque type that represents a list of [cavc_pline].
///
/// Note the internal member is only public for composing in other Rust libraries wanting to use the
/// FFI opaque type as part of their FFI API.
pub struct cavc_plinelist(pub Vec<*mut cavc_pline>);

impl Drop for cavc_plinelist {
    fn drop(&mut self) {
        // Free all contained cavc_pline pointers
        for pline_ptr in self.0.drain(..) {
            unsafe {
                cavc_pline_f(pline_ptr);
            }
        }
    }
}

impl cavc_plinelist {
    pub fn from_internal<I>(plines: I) -> *mut cavc_plinelist
    where
        I: IntoIterator<Item = Polyline>,
    {
        let r = plines
            .into_iter()
            .map(|pl| Box::into_raw(Box::new(cavc_pline(pl))))
            .collect();

        Box::into_raw(Box::new(cavc_plinelist(r)))
    }
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
/// `pline` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_create(
    vertexes: *const cavc_vertex,
    n_vertexes: u32,
    is_closed: u8,
    pline: *mut *const cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        let mut result = Polyline::new();
        if is_closed != 0 {
            result.set_is_closed(true);
        }

        if !vertexes.is_null() && n_vertexes != 0 {
            let data = unsafe { slice::from_raw_parts(vertexes, n_vertexes as usize) };
            result.reserve(data.len());
            for v in data {
                result.add(v.x, v.y, v.bulge);
            }
        }

        unsafe {
            pline.write(Box::into_raw(Box::new(cavc_pline(result))));
        }
        0
    })
}

/// Free an existing [cavc_pline] object.
///
/// Nothing happens if `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_pline_f(pline: *mut cavc_pline) {
    if !pline.is_null() {
        unsafe {
            drop(Box::from_raw(pline));
        }
    }
}

/// Set the userdata values of a pline
///
/// 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
/// 'count' is the number of u64 values to be stored; not the byte size.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `userdata_values` must point to a valid location to read from.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_set_userdata_values(
    pline: *mut cavc_pline,
    userdata_values: *const u64,
    count: u32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            (*pline).0.userdata.clear();

            if !userdata_values.is_null() && count != 0 {
                let data = slice::from_raw_parts(userdata_values, count as usize);
                (*pline).0.userdata.extend_from_slice(data);
            }
        }

        0
    })
}

/// Get the userdata value count of a polyline.
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
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_userdata_count(
    pline: *const cavc_pline,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            count.write(u32::try_from((*pline).0.userdata.len()).unwrap());
        }
        0
    })
}

/// Get the userdata values of a pline
///
/// 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
/// overrun will happen.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_userdata_values(
    pline: *const cavc_pline,
    userdata_values: *mut u64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            std::ptr::copy(
                (*pline).0.userdata.as_ptr(),
                userdata_values,
                (*pline).0.userdata.len(),
            );
        }
        0
    })
}

/// Reserve space for an `additional` number of vertexes in the [cavc_pline].
///
/// This function is used to avoid allocations when adding vertexes to the [cavc_pline].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_reserve(pline: *mut cavc_pline, additional: u32) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            (*pline).0.reserve(additional as usize);
        }
        0
    })
}

/// Clones the polyline.
///
/// `pline` is the polyline to be cloned.
/// `cloned` is used as an out parameter to hold the new polyline from cloning.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `cloned` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_clone(
    pline: *const cavc_pline,
    cloned: *mut *const cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            cloned.write(Box::into_raw(Box::new(cavc_pline((*pline).0.clone()))));
        }
        0
    })
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
/// `is_closed` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_is_closed(
    pline: *const cavc_pline,
    is_closed: *mut u8,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            is_closed.write((*pline).0.is_closed() as u8);
        }
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
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_set_is_closed(pline: *mut cavc_pline, is_closed: u8) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            (*pline).0.set_is_closed(is_closed != 0);
        }
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
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
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
        unsafe {
            count.write(u32::try_from((*pline).0.vertex_count()).unwrap());
        }
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
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_get_vertex_data(
    pline: *const cavc_pline,
    vertex_data: *mut cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let buffer = unsafe { slice::from_raw_parts_mut(vertex_data, (*pline).0.vertex_count()) };
        for (i, v) in unsafe { (*pline).0.iter_vertexes().enumerate() } {
            buffer[i] = cavc_vertex::from_internal(v);
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
#[unsafe(no_mangle)]
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

        unsafe {
            (*pline).0.clear();
            let buffer = slice::from_raw_parts(vertex_data, n_vertexes as usize);
            (*pline).0.reserve(buffer.len());
            for v in buffer {
                (*pline).0.add(v.x, v.y, v.bulge);
            }
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
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_clear(pline: *mut cavc_pline) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            (*pline).0.clear();
        }
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
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_add(pline: *mut cavc_pline, x: f64, y: f64, bulge: f64) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            (*pline).0.add(x, y, bulge);
        }
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
/// `vertex` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
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

        unsafe {
            if position >= (*pline).0.vertex_count() as u32 {
                return 2;
            }

            let v = (&(*pline).0)[position as usize];
            vertex.write(cavc_vertex::from_internal(v));
        }

        0
    })
}

/// Set a polyline vertex at a given index position.
///
/// `position` is is the index to set the vertex at.
/// `vertex` is the data to be set.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `position` is out of bounds for the `pline` given.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_set_vertex(
    pline: *mut cavc_pline,
    position: u32,
    vertex: cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            if position >= (*pline).0.vertex_count() as u32 {
                return 2;
            }

            (&mut (*pline).0)[position as usize] =
                PlineVertex::new(vertex.x, vertex.y, vertex.bulge);
        }

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
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_remove(pline: *mut cavc_pline, position: u32) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        unsafe {
            if position as usize >= (*pline).0.vertex_count() {
                return 2;
            }

            (*pline).0.remove(position as usize);
        }

        0
    })
}

/// Wraps [PlineSource::path_length].
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
/// `path_length` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_path_length(
    pline: *const cavc_pline,
    path_length: *mut f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            path_length.write((*pline).0.path_length());
        }
        0
    })
}

/// Wraps [PlineSource::area].
///
/// `area` is used as the out parameter to hold the computed area.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `area` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_area(pline: *const cavc_pline, area: *mut f64) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            area.write((*pline).0.area());
        }
        0
    })
}

/// Wraps [PlineSource::winding_number].
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
/// `winding_number` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_wn(
    pline: *const cavc_pline,
    x: f64,
    y: f64,
    winding_number: *mut i32,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            winding_number.write((*pline).0.winding_number(Vector2::new(x, y)));
        }
        0
    })
}

/// Wraps [PlineSourceMut::invert_direction_mut].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_invert_direction(pline: *mut cavc_pline) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            (*pline).0.invert_direction_mut();
        }
        0
    })
}

/// Wraps [PlineSourceMut::scale_mut].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_scale(pline: *mut cavc_pline, scale_factor: f64) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            (*pline).0.scale_mut(scale_factor);
        }
        0
    })
}

/// Wraps [PlineSourceMut::translate_mut].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_translate(
    pline: *mut cavc_pline,
    x_offset: f64,
    y_offset: f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }
        unsafe {
            (*pline).0.translate_mut(x_offset, y_offset);
        }
        0
    })
}

/// Wraps [PlineSource::remove_repeat_pos] but modifies in place rather than returning a result.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_remove_repeat_pos(
    pline: *mut cavc_pline,
    pos_equal_eps: f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &mut (*pline).0 };

        match pline.remove_repeat_pos(pos_equal_eps) {
            None => {
                // do nothing (no repeat positions, leave unchanged)
                0
            }
            Some(x) => {
                // update self with result
                *pline = x;
                0
            }
        }
    })
}

/// Wraps [PlineSource::remove_redundant] but modifies in place rather than returning a result.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_remove_redundant(
    pline: *mut cavc_pline,
    pos_equal_eps: f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &mut (*pline).0 };

        match pline.remove_redundant(pos_equal_eps) {
            None => {
                // do nothing (no repeat positions, leave unchanged)
                0
            }
            Some(x) => {
                // update self with result
                *pline = x;
                0
            }
        }
    })
}

/// Wraps [PlineSource::extents].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `pline` vertex count is less than 2.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `min_x`, `min_y`, `max_x`, and `max_y` must all point to a valid places in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_eval_extents(
    pline: *const cavc_pline,
    min_x: *mut f64,
    min_y: *mut f64,
    max_x: *mut f64,
    max_y: *mut f64,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &(*pline).0 };
        match pline.extents() {
            Some(aabb) => {
                unsafe {
                    min_x.write(aabb.min_x);
                    min_y.write(aabb.min_y);
                    max_x.write(aabb.max_x);
                    max_y.write(aabb.max_y);
                }
                0
            }
            None => 2,
        }
    })
}

/// Wraps [PlineSource::parallel_offset_opt].
///
/// `options` is allowed to be null (default options will be used).
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `result` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_parallel_offset(
    pline: *const cavc_pline,
    offset: f64,
    options: *const cavc_pline_parallel_offset_o,
    result: *mut *const cavc_plinelist,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &(*pline).0 };

        let results = if options.is_null() {
            pline.parallel_offset(offset)
        } else {
            let opts = unsafe { &(*options).to_internal() };
            pline.parallel_offset_opt(offset, opts)
        };

        unsafe {
            result.write(cavc_plinelist::from_internal(results));
        }
        0
    })
}

/// Wraps [PlineSource::boolean_opt].
///
/// `options` is allowed to be null (default options will be used).
///
/// Boolean operations are:
/// * 0 = [BooleanOp::Or]
/// * 1 = [BooleanOp::And]
/// * 2 = [BooleanOp::Not]
/// * 3 = [BooleanOp::Xor]
///
/// ## Specific Error Codes
/// * 1 = `pline1` and/or `pline2` is null.
/// * 2 = `operation` is unrecognized (must be one of the values listed).
///
/// # Safety
///
/// `pline1` and `pline2` must each be null or a valid cavc_pline object that was created with
/// [cavc_pline_create] and has not been freed.
/// `pos_plines` and `neg_plines` must both point to different valid places in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_boolean(
    pline1: *const cavc_pline,
    pline2: *const cavc_pline,
    operation: u32,
    options: *const cavc_pline_boolean_o,
    pos_plines: *mut *const cavc_plinelist,
    neg_plines: *mut *const cavc_plinelist,
) -> i32 {
    ffi_catch_unwind!({
        if pline1.is_null() || pline2.is_null() {
            return 1;
        }

        let op = {
            match boolean_op_from_u32(operation) {
                Some(op) => op,
                None => {
                    return 2;
                }
            }
        };

        let pline1 = unsafe { &(*pline1).0 };
        let pline2 = unsafe { &(*pline2).0 };

        let results = if options.is_null() {
            pline1.boolean(pline2, op)
        } else {
            let options = unsafe { &(*options).to_internal() };
            pline1.boolean_opt(pline2, op, options)
        };

        unsafe {
            pos_plines.write(cavc_plinelist::from_internal(
                results.pos_plines.into_iter().map(|p| p.pline),
            ));
            neg_plines.write(cavc_plinelist::from_internal(
                results.neg_plines.into_iter().map(|p| p.pline),
            ));
        }
        0
    })
}

/// Wraps [PlineSource::scan_for_self_intersect_opt].
///
/// `options` is allowed to be null (default options will be used).
///
/// ## Specific Error Codes
/// * 1 = `pline1` is null.
/// * 2 = `options` is invalid.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with
/// [cavc_pline_create] and has not been freed.
/// `is_self_intersecting` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_scan_for_self_intersect(
    pline: *const cavc_pline,
    options: *const cavc_pline_self_intersect_o,
    is_self_intersecting: *mut u8,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &(*pline).0 };

        let computed_result;
        if options.is_null() {
            computed_result = pline.scan_for_self_intersect();
        } else {
            let options = unsafe { &(*options).to_internal() };
            match options {
                None => return 2, // invalid options were passed in.
                Some(unpacked_options) => {
                    computed_result = pline.scan_for_self_intersect_opt(unpacked_options)
                }
            }
        };

        unsafe {
            is_self_intersecting.write(computed_result as u8);
        }

        0
    })
}

/// FFI Representation of PlineContainsResult enum
pub const CAVC_CONTAINS_RESULT_INVALID_INPUT: u32 = 0;
pub const CAVC_CONTAINS_RESULT_PLINE1_INSIDE_PLINE2: u32 = 1;
pub const CAVC_CONTAINS_RESULT_PLINE2_INSIDE_PLINE1: u32 = 2;
pub const CAVC_CONTAINS_RESULT_DISJOINT: u32 = 3;
pub const CAVC_CONTAINS_RESULT_INTERSECTED: u32 = 4;

/// Wraps [PlineSource::contains_opt].
///
/// `options` is allowed to be null (default options will be used).
///
/// Possible values returned in result:
///
/// CAVC_CONTAINS_RESULT_INVALID_INPUT: Input was not valid to perform operation.
/// CAVC_CONTAINS_RESULT_PLINE1_INSIDE_PLINE2: Pline1 entirely inside of pline2 with no intersects.
/// CAVC_CONTAINS_RESULT_PLINE2_INSIDE_PLINE1: Pline2 entirely inside of pline1 with no intersects.
/// CAVC_CONTAINS_RESULT_DISJOINT: Pline1 is disjoint from pline2 (no intersects and neither polyline is inside of the other).
/// CAVC_CONTAINS_RESULT_INTERSECTED: Pline1 intersects with pline2 in at least one place.
///
/// ## Specific Error Codes
/// * 1 = `pline1` and/or `pline2` is null. In case of an error, if result is not null it will be set to CAVC_CONTAINS_RESULT_INVALID_INPUT.
///
/// Caution: Polylines with self-intersections may generate unexpected results.
/// Use cavc_pline_scan_for_self_intersect() to find and reject self-intersecting polylines
/// if this is a possibility for your input data.
///
/// # Safety
///
/// `pline1` and `pline2` must each be null or a valid cavc_pline object that was created with
/// [cavc_pline_create] and has not been freed.
/// `result` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_contains(
    pline1: *const cavc_pline,
    pline2: *const cavc_pline,
    options: *const cavc_pline_contains_o,
    result: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if pline1.is_null() || pline2.is_null() {
            if !result.is_null() {
                unsafe { result.write(CAVC_CONTAINS_RESULT_INVALID_INPUT) };
            }
            return 1;
        }

        let pline1 = unsafe { &(*pline1).0 };
        let pline2 = unsafe { &(*pline2).0 };

        let computed_result = if options.is_null() {
            pline1.contains(pline2)
        } else {
            let options = unsafe { &(*options).to_internal() };
            pline1.contains_opt(pline2, options)
        };

        unsafe {
            result.write(computed_result as u32);
        }

        0
    })
}

/// Wraps [PlineSource::create_approx_aabb_index].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `aabbindex` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_create_approx_aabbindex(
    pline: *const cavc_pline,
    aabbindex: *mut *const cavc_aabbindex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &(*pline).0 };
        let result = pline.create_approx_aabb_index();
        unsafe {
            aabbindex.write(Box::into_raw(Box::new(cavc_aabbindex(result))));
        }

        0
    })
}

/// Wraps [PlineSource::create_aabb_index].
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `aabbindex` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_pline_create_aabbindex(
    pline: *const cavc_pline,
    aabbindex: *mut *const cavc_aabbindex,
) -> i32 {
    ffi_catch_unwind!({
        if pline.is_null() {
            return 1;
        }

        let pline = unsafe { &(*pline).0 };
        let result = pline.create_aabb_index();
        unsafe {
            aabbindex.write(Box::into_raw(Box::new(cavc_aabbindex(result))));
        }

        0
    })
}

/// Free an existing [cavc_aabbindex] object.
///
/// Nothing happens if `aabbindex` is null.
///
/// # Safety
///
/// `aabbindex` must be null or a valid [cavc_aabbindex] object.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_aabbindex_f(aabbindex: *mut cavc_aabbindex) {
    if !aabbindex.is_null() {
        unsafe { drop(Box::from_raw(aabbindex)) }
    }
}

/// Wraps the [`StaticAABB2DIndex::bounds`] method (gets total extents of the aabb index). Writes
/// NaNs if the index is empty.
///
/// ## Specific Error Codes
/// * 1 = `aabbindex` is null.
///
/// # Safety
///
/// `aabbindex` must be null or a valid [cavc_aabbindex] object.
/// `min_x`, `min_y`, `max_x`, and `max_y` must all point to a valid places in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_aabbindex_get_extents(
    aabbindex: *const cavc_aabbindex,
    min_x: *mut f64,
    min_y: *mut f64,
    max_x: *mut f64,
    max_y: *mut f64,
) -> i32 {
    ffi_catch_unwind!({
        if aabbindex.is_null() {
            return 1;
        }

        let aabbindex = unsafe { &(*aabbindex).0 };

        if let Some(bounds) = aabbindex.bounds() {
            unsafe {
                min_x.write(bounds.min_x);
                min_y.write(bounds.min_y);
                max_x.write(bounds.max_x);
                max_y.write(bounds.max_y);
            }
        } else {
            unsafe {
                min_x.write(f64::NAN);
                min_y.write(f64::NAN);
                max_x.write(f64::NAN);
                max_y.write(f64::NAN);
            }
        }
        0
    })
}

/// Create a new [cavc_plinelist] object.
///
/// `capacity` is the number of plines to pre-allocate space for. May be zero.
/// `plinelist` is an out parameter to hold the created shape.
///
/// # Safety
///
/// `plinelist` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_create(
    capacity: usize,
    plinelist: *mut *mut cavc_plinelist,
) -> i32 {
    ffi_catch_unwind!({
        unsafe {
            plinelist.write(Box::into_raw(Box::new(cavc_plinelist(Vec::with_capacity(
                capacity,
            )))));
        }
        0
    })
}

/// Free an existing [cavc_plinelist] object and all [cavc_pline] owned by it.
///
/// Nothing happens if `plinelist` is null.
///
/// # Safety
///
/// `plinelist` must be null or a valid [cavc_plinelist] object.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_plinelist_f(plinelist: *mut cavc_plinelist) {
    if !plinelist.is_null() {
        unsafe { drop(Box::from_raw(plinelist)) }
    }
}

/// Get the number of polylines inside a [cavc_plinelist].
///
/// `count` used as out parameter to hold the polyline count.
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
///
/// # Safety
///
/// `plinelist` must be null or a valid [cavc_plinelist] object.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_get_count(
    plinelist: *const cavc_plinelist,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }

        // using try_from to catch odd case of polyline count greater than u32::MAX to
        // prevent memory corruption/access errors but just panic as internal error if it does occur
        unsafe {
            count.write(u32::try_from((*plinelist).0.len()).unwrap());
        }
        0
    })
}

/// Get a polyline at the given index position in the [cavc_plinelist].
///
/// `pline` used as out parameter to hold the polyline pointer. NOTE: This does not release
/// ownership of the [cavc_pline] from the [cavc_plinelist], to do that use [cavc_plinelist_pop] or
/// [cavc_plinelist_take].
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
/// * 2 = `position` out of range for the [cavc_plinelist].
///
/// # Safety
///
/// `plinelist` must be null or a valid [cavc_plinelist] object.
/// `pline` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_get_pline(
    plinelist: *const cavc_plinelist,
    position: u32,
    pline: *mut *const cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }

        let plinelist = unsafe { &(*plinelist).0 };

        let pos = position as usize;

        match plinelist.get(pos) {
            Some(pl) => {
                unsafe {
                    pline.write(*pl);
                }
                0
            }
            None => 2,
        }
    })
}

/// Append a [cavc_pline] to the end of a [cavc_plinelist].
///
/// `plinelist` is the [cavc_plinelist] to append to.
/// `pline` is the [cavc_pline] to be appended.
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
/// * 2 = `pline` is null.
///
/// # Safety
///
/// `plinelist` must be a valid [cavc_plinelist] object.
/// `pline` must be a valid [cavc_pline] object.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_push(
    plinelist: *mut cavc_plinelist,
    pline: *mut cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }
        if pline.is_null() {
            return 2;
        }

        unsafe {
            (*plinelist).0.push(pline);
        }
        0
    })
}

/// Efficiently release and return the last [cavc_pline] from a [cavc_plinelist].
///
/// `pline` used as out parameter to hold the polyline pointer released from the [cavc_plinelist].
/// NOTE: The caller now must call [cavc_pline_f] at some point to free the released [cavc_pline].
/// If you pass null in `pline` you must already have another pointer to the released [cavc_pline]
/// in order to free it.
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
/// * 2 = `plinelist` is empty.
///
/// # Safety
///
/// `plinelist` must be null or a valid [cavc_plinelist] object.
/// `pline` must point to a valid place in memory to be written, or null if you don't need the pointer to the [cavc_pline].
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_pop(
    plinelist: *mut cavc_plinelist,
    pline: *mut *const cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }

        let plinelist = unsafe { &mut (*plinelist).0 };

        match plinelist.pop() {
            Some(p) => {
                unsafe {
                    if !pline.is_null() {
                        pline.write(p);
                    }
                }
                0
            }
            None => 2,
        }
    })
}

/// Release and return a [cavc_pline] from a [cavc_plinelist] at a given index position.
///
/// `pline` used as out parameter to hold the polyline pointer released from the [cavc_plinelist].
/// NOTE: The caller now must call [cavc_pline_f] at some point to free the released [cavc_pline].
/// If you pass null in `pline` you must already have another pointer to the released [cavc_pline]
/// in order to free it.
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
/// * 2 = `position` out of range for the [cavc_plinelist].
///
/// # Safety
///
/// `plinelist` must be null or a valid [cavc_plinelist] object.
/// `pline` must point to a valid place in memory to be written, or null if you don't need the pointer to the [cavc_pline].
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_plinelist_take(
    plinelist: *mut cavc_plinelist,
    position: u32,
    pline: *mut *const cavc_pline,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }

        let plinelist = unsafe { &mut (*plinelist).0 };
        let pos = position as usize;

        if pos >= plinelist.len() {
            return 2;
        }

        unsafe {
            let result = plinelist.remove(pos);
            if !pline.is_null() {
                pline.write(result);
            }
        }

        0
    })
}

/// FFI representation of [ShapeOffsetOptions].
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct cavc_shape_offset_o {
    pub pos_equal_eps: f64,
    pub offset_dist_eps: f64,
    pub slice_join_eps: f64,
}

impl cavc_shape_offset_o {
    /// Convert FFI shape offset options type to internal type.
    pub fn to_internal(&self) -> ShapeOffsetOptions<f64> {
        ShapeOffsetOptions {
            pos_equal_eps: self.pos_equal_eps,
            offset_dist_eps: self.offset_dist_eps,
            slice_join_eps: self.slice_join_eps,
        }
    }
}

impl Default for cavc_shape_offset_o {
    fn default() -> Self {
        let d = ShapeOffsetOptions::default();
        Self {
            pos_equal_eps: d.pos_equal_eps,
            offset_dist_eps: d.offset_dist_eps,
            slice_join_eps: d.slice_join_eps,
        }
    }
}

/// Write default option values to a [cavc_shape_offset_o].
///
/// ## Specific Error Codes
/// * 1 = `options` is null.
///
/// # Safety
///
/// `options` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_offset_o_init(options: *mut cavc_shape_offset_o) -> i32 {
    ffi_catch_unwind!({
        if options.is_null() {
            return 1;
        }

        unsafe {
            options.write(Default::default());
        }
        0
    })
}

/// Opaque type that wraps a [Shape].
///
/// Note the internal member is only public for composing in other Rust libraries wanting to use the
/// FFI opaque type as part of their FFI API.
#[derive(Debug, Clone)]
pub struct cavc_shape(pub Shape<f64>);

/// Create a new [cavc_shape] object.
///
/// `plinelist` is a [cavc_plinelist] containing the [cavc_pline] paths to create the shape from.
/// `shape` is an out parameter to hold the created shape.
///
/// ## Specific Error Codes
/// * 1 = `plinelist` is null.
///
/// # Safety
///
/// `shape` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_create(
    plinelist: *const cavc_plinelist,
    shape: *mut *mut cavc_shape,
) -> i32 {
    ffi_catch_unwind!({
        if plinelist.is_null() {
            return 1;
        }

        unsafe {
            let count: usize = (*plinelist).0.len();
            let mut v: Vec<Polyline<f64>> = Vec::with_capacity(count);

            for pline in (*plinelist).0.iter() {
                v.push((**pline).0.clone());
            }

            let s = Shape::from_plines(v);

            shape.write(Box::into_raw(Box::new(cavc_shape(s))));
        }
        0
    })
}

/// Free an existing [cavc_shape] object.
///
/// Nothing happens if `shape` is null.
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
/// has not already been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cavc_shape_f(shape: *mut cavc_shape) {
    if !shape.is_null() {
        unsafe { drop(Box::from_raw(shape)) }
    }
}

/// Wraps [Shape::parallel_offset].
///
/// `options` is allowed to be null (default options will be used).
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `result` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_parallel_offset(
    shape: *const cavc_shape,
    offset: f64,
    options: *const cavc_shape_offset_o,
    result: *mut *mut cavc_shape,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            let results = if options.is_null() {
                let default_options = ShapeOffsetOptions::new();
                (*shape).0.parallel_offset(offset, default_options)
            } else {
                (*shape).0.parallel_offset(offset, (*options).to_internal())
            };

            result.write(Box::into_raw(Box::new(cavc_shape(results))));
        }
        0
    })
}

/// Get the count of counter-clockwise polylines in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_count(
    shape: *const cavc_shape,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            count.write(u32::try_from((*shape).0.ccw_plines.len()).unwrap());
        }
        0
    })
}

/// Get the vertex count of a specific counter-clockwise polyline in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_polyline_count(
    shape: *const cavc_shape,
    polyline_index: u32,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.ccw_plines.len()) {
                return 2;
            }

            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            let ccw_plines = &(*shape).0.ccw_plines;
            let polyline = &ccw_plines[polyline_index as usize].polyline;
            count.write(u32::try_from(polyline.vertex_data.len()).unwrap());
        }
        0
    })
}

/// Get whether a specific counter-clockwise polyline in a shape is closed.
///
/// `is_closed` is used as an out parameter to hold the whether the polyline is closed (non-zero) or not
/// (zero).
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `is_closed` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_polyline_is_closed(
    shape: *const cavc_shape,
    polyline_index: u32,
    is_closed: *mut u8,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.ccw_plines.len()) {
                return 2;
            }

            let ccw_plines = &(*shape).0.ccw_plines;
            let polyline = &ccw_plines[polyline_index as usize].polyline;
            is_closed.write(u8::from(polyline.is_closed));
        }
        0
    })
}

/// Fills the buffer given with the vertex data of a ccw polyline in a shape.
///
/// You must use [cavc_shape_get_ccw_polyline_count] to ensure the buffer given has adequate length
/// to be filled with all vertexes!
///
/// `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
/// overrun will happen.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_polyline_vertex_data(
    shape: *const cavc_shape,
    polyline_index: u32,
    vertex_data: *mut cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.ccw_plines.len()) {
                return 2;
            }

            let ccw_plines = &(*shape).0.ccw_plines;
            let pline = &ccw_plines[polyline_index as usize].polyline;

            let buffer = slice::from_raw_parts_mut(vertex_data, pline.vertex_count());
            for (i, v) in pline.iter_vertexes().enumerate() {
                buffer[i] = cavc_vertex::from_internal(v);
            }
        }
        0
    })
}

/// Set the userdata values of a CCW polyline in a shape
///
/// 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
/// has not been freed.
/// `userdata_values` must point to a valid location to read from.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_set_ccw_pline_userdata_values(
    shape: *mut cavc_shape,
    polyline_index: u32,
    userdata_values: *const u64,
    count: u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.ccw_plines.len()) {
                return 2;
            }

            let ccw_plines = &mut (*shape).0.ccw_plines;
            let pline = &mut ccw_plines[polyline_index as usize].polyline;

            pline.userdata.clear();

            if !userdata_values.is_null() && count != 0 {
                let data = slice::from_raw_parts(userdata_values, count as usize);
                pline.userdata.extend_from_slice(data);
            }
        }
        0
    })
}

/// Get the userdata value count of a CCW polyline in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_pline_userdata_count(
    shape: *const cavc_shape,
    polyline_index: u32,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.ccw_plines.len()) {
                return 2;
            }

            let ccw_plines = &(*shape).0.ccw_plines;
            let pline = &ccw_plines[polyline_index as usize].polyline;

            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            count.write(u32::try_from(pline.userdata.len()).unwrap());
        }
        0
    })
}

/// Get the userdata values of a CCW pline in a shape
///
/// 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
/// overrun will happen.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_ccw_pline_userdata_values(
    shape: *const cavc_shape,
    polyline_index: u32,
    userdata_values: *mut u64,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            let ccw_plines = &(*shape).0.ccw_plines;
            let pline = &ccw_plines[polyline_index as usize].polyline;

            std::ptr::copy(
                pline.userdata.as_ptr(),
                userdata_values,
                pline.userdata.len(),
            );
        }
        0
    })
}

/// Get the count of clockwise polylines in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_count(shape: *const cavc_shape, count: *mut u32) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            count.write(u32::try_from((*shape).0.cw_plines.len()).unwrap());
        }
        0
    })
}

/// Get the vertex count of a specific clockwise polyline in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_polyline_count(
    shape: *const cavc_shape,
    polyline_index: u32,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.cw_plines.len()) {
                return 2;
            }

            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            let cw_plines = &(*shape).0.cw_plines;
            let polyline = &cw_plines[polyline_index as usize].polyline;
            count.write(u32::try_from(polyline.vertex_data.len()).unwrap());
        }
        0
    })
}

/// Get whether a specific clockwise polyline in a shape is closed.
///
/// `is_closed` is used as an out parameter to hold the whether the polyline is closed (non-zero) or not
/// (zero).
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
/// has not been freed.
/// `is_closed` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_polyline_is_closed(
    shape: *const cavc_shape,
    polyline_index: u32,
    is_closed: *mut u8,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            let shape = &(*shape).0;
            if polyline_index as usize >= shape.cw_plines.len() {
                return 2;
            }

            is_closed.write(u8::from(
                shape.cw_plines[polyline_index as usize].polyline.is_closed,
            ));
        }
        0
    })
}

/// Fills the buffer given with the vertex data of a cw polyline in a shape.
///
/// You must use [cavc_shape_get_cw_polyline_count] to ensure the buffer given has adequate length
/// to be filled with all vertexes!
///
/// `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
/// overrun will happen.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_polyline_vertex_data(
    shape: *const cavc_shape,
    polyline_index: u32,
    vertex_data: *mut cavc_vertex,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.cw_plines.len()) {
                return 2;
            }

            let cw_plines = &(*shape).0.cw_plines;
            let pline = &cw_plines[polyline_index as usize].polyline;

            let buffer = slice::from_raw_parts_mut(vertex_data, pline.vertex_count());
            for (i, v) in pline.iter_vertexes().enumerate() {
                buffer[i] = cavc_vertex::from_internal(v);
            }
        }
        0
    })
}

/// Set the userdata values of a CW polyline in a shape
///
/// 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
/// has not been freed.
/// `userdata_values` must point to a valid location to read from.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_set_cw_pline_userdata_values(
    shape: *mut cavc_shape,
    polyline_index: u32,
    userdata_values: *const u64,
    count: u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.cw_plines.len()) {
                return 2;
            }

            let cw_plines = &mut (*shape).0.cw_plines;
            let pline = &mut cw_plines[polyline_index as usize].polyline;

            pline.userdata.clear();

            if !userdata_values.is_null() && count != 0 {
                let data = slice::from_raw_parts(userdata_values, count as usize);
                pline.userdata.extend_from_slice(data);
            }
        }
        0
    })
}

/// Get the userdata value count of a CW polyline in a shape.
///
/// `count` used as out parameter to hold the vertex count.
///
/// ## Specific Error Codes
/// * 1 = `shape` is null.
/// * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
///
/// # Safety
///
/// `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
/// has not been freed.
/// `count` must point to a valid place in memory to be written.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_pline_userdata_count(
    shape: *const cavc_shape,
    polyline_index: u32,
    count: *mut u32,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            if polyline_index as usize >= ((*shape).0.cw_plines.len()) {
                return 2;
            }

            let cw_plines = &(*shape).0.cw_plines;
            let pline = &cw_plines[polyline_index as usize].polyline;

            // using try_from to catch odd case of polyline vertex count greater than u32::MAX to
            // prevent memory corruption/access errors but just panic as internal error if it does occur
            count.write(u32::try_from(pline.userdata.len()).unwrap());
        }
        0
    })
}

/// Get the userdata values of a CW pline in a shape
///
/// 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
///
/// ## Specific Error Codes
/// * 1 = `pline` is null.
///
/// # Safety
///
/// `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
/// has not been freed.
/// `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
/// overrun will happen.
#[unsafe(no_mangle)]
#[must_use]
pub unsafe extern "C" fn cavc_shape_get_cw_pline_userdata_values(
    shape: *const cavc_shape,
    polyline_index: u32,
    userdata_values: *mut u64,
) -> i32 {
    ffi_catch_unwind!({
        if shape.is_null() {
            return 1;
        }

        unsafe {
            let cw_plines = &(*shape).0.cw_plines;
            let pline = &cw_plines[polyline_index as usize].polyline;
            std::ptr::copy(
                pline.userdata.as_ptr(),
                userdata_values,
                pline.userdata.len(),
            );
        }
        0
    })
}
