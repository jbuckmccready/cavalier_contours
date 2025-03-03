mod pline;
mod raw_pline_offset_segs;
mod shape;

pub use pline::*;
pub use raw_pline_offset_segs::*;
pub use shape::*;

use cavalier_contours::{core::math::Vector2, static_aabb2d_index::AABB};
use egui::epaint;
use egui_plot::PlotTransform;
use lyon::tessellation::{FillVertexConstructor, StrokeVertexConstructor};

/// Plot vertex radius (in pixels) for drawing vertices.
pub const PLOT_VERTEX_RADIUS: f32 = 4.0;

/// Convert cavalier contours Vector2 to lyon Point adjusted for plot using plo transform.
fn lyon_point(v: Vector2, transform: &PlotTransform) -> lyon::math::Point {
    lyon::math::point(
        transform.position_from_point_x(v.x),
        transform.position_from_point_y(v.y),
    )
}

fn aabb_to_plotbounds(aabb: &AABB) -> egui_plot::PlotBounds {
    egui_plot::PlotBounds::from_min_max([aabb.min_x, aabb.min_y], [aabb.max_x, aabb.max_y])
}

struct VertexConstructor {
    color: epaint::Color32,
}

impl FillVertexConstructor<epaint::Vertex> for VertexConstructor {
    fn new_vertex(&mut self, vertex: lyon::tessellation::FillVertex<'_>) -> epaint::Vertex {
        let p = vertex.position();
        let pos = epaint::Pos2::new(p.x, p.y);
        epaint::Vertex {
            pos,
            uv: epaint::WHITE_UV,
            color: self.color,
        }
    }
}

impl StrokeVertexConstructor<epaint::Vertex> for VertexConstructor {
    fn new_vertex(&mut self, vertex: lyon::tessellation::StrokeVertex<'_, '_>) -> epaint::Vertex {
        let p = vertex.position();
        let pos = epaint::Pos2::new(p.x, p.y);
        epaint::Vertex {
            pos,
            uv: epaint::WHITE_UV,
            color: self.color,
        }
    }
}
