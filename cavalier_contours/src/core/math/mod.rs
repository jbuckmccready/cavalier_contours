//! Core/common math functions for working with angles, 2D space, intersections, etc.
mod base_math;
mod circle_circle_intersect;
mod line_circle_intersect;
mod line_line_intersect;
mod vector2;

pub use base_math::*;
pub use circle_circle_intersect::{circle_circle_intr, CircleCircleIntr};
pub use line_circle_intersect::{line_circle_intr, LineCircleIntr};
pub use line_line_intersect::{line_line_intr, LineLineIntr};
pub use vector2::Vector2;
