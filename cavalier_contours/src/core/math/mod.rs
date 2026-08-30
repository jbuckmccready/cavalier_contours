//! Core/common math functions for working with angles, 2D space, intersections, etc.
mod base_math;
mod intersections;
mod vector2;

pub use base_math::*;
pub use intersections::{
    Circle, CurveIntersections, IntersectionPoint, Line, intersect_circles, intersect_line_circle,
    intersect_lines,
};
pub use vector2::{Vector2, vec2};
