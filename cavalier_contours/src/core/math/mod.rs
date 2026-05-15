//! Core/common math functions for working with angles, 2D space, intersections, etc.
mod base_math;
mod circle_circle_intersect;
mod line_circle_intersect;
mod line_line_intersect;
mod vector2;

pub use base_math::*;
pub use circle_circle_intersect::{CircleCircleIntr, circle_circle_intr};
pub use line_circle_intersect::{LineCircleIntr, line_circle_intr};
pub use line_line_intersect::{LineLineIntr, line_line_intr};
pub use vector2::{Vector2, vec2};
