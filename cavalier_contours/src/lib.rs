//! 2D geometry polyline/shape library for offsetting, combining, computing areas, path lengths,
//! winding numbers, etc.
#![forbid(unsafe_code)]

pub use static_aabb2d_index;

#[macro_use]
mod macros;
#[macro_use]
pub mod core;
pub mod polyline;
pub mod shape_algorithms;
