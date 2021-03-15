extern crate static_aabb2d_index;

#[macro_use]
mod macros;
mod base_math;
mod base_traits;
mod pline_seg;
mod pline_vertex;
mod vector2;

mod circle_circle_intersect;
mod line_circle_intersect;
mod line_line_intersect;
mod pline_seg_intersect;

mod polyline;
mod polyline_intersects;
mod polyline_offset;

/// Internal modules made public for visualization, benchmarking, and testing purposes.
pub mod internal {
    pub mod polyline_intersects {
        pub use crate::polyline_intersects::*;
    }
    pub mod polyline_offset {
        pub use crate::polyline_offset::*;
    }
}

pub mod core_math {
    pub use crate::base_math::*;
    pub use crate::pline_seg::*;
}

pub mod intersects {
    pub use crate::circle_circle_intersect::*;
    pub use crate::line_circle_intersect::*;
    pub use crate::line_line_intersect::*;
    pub use crate::pline_seg_intersect::*;
}

pub use static_aabb2d_index::AABB;

pub use crate::base_traits::*;
pub use crate::pline_vertex::*;
pub use crate::polyline::*;
pub use crate::vector2::*;
