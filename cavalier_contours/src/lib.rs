//! 2D geometry polyline/shape library for offsetting, combining, computing areas, path lengths,
//! winding numbers, etc.

pub use static_aabb2d_index;

#[macro_use]
mod macros;
pub mod core;
pub mod polyline;
