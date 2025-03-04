mod plines;
mod raw_pline_offset_segs;

pub use plines::*;
pub use raw_pline_offset_segs::*;

use cavalier_contours::{core::math::Vector2, static_aabb2d_index::AABB};
use egui::epaint;
use egui_plot::PlotTransform;
use lyon::{
    path::{Path, PathEvent},
    tessellation::{FillVertexConstructor, StrokeVertexConstructor},
};

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

fn plotbounds_to_aabb(bounds: &egui_plot::PlotBounds) -> AABB {
    let [min_x, min_y] = bounds.min();
    let [max_x, max_y] = bounds.max();
    AABB::new(min_x, min_y, max_x, max_y)
}

fn empty_aabb() -> AABB {
    AABB::new(
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    )
}

/// Cull path events to only include those that are within the given bounds.
fn cull_path(path: &Path, bounds: &egui::Rect) -> impl Iterator<Item = PathEvent> {
    let mut path_events = path.iter();
    let mut event_queued = None;

    std::iter::from_fn(move || {
        if let Some(event) = event_queued.take() {
            return Some(event);
        }

        let event = path_events.next()?;
        match event {
            PathEvent::Begin { .. } => Some(event),
            PathEvent::Line { from, to } => {
                let line_bounds =
                    egui::Rect::from_two_pos(egui::pos2(from.x, from.y), egui::pos2(to.x, to.y));
                let keep = bounds.intersects(line_bounds);
                if !keep {
                    event_queued = Some(PathEvent::Begin { at: to });
                    return Some(PathEvent::End {
                        last: from,
                        first: from,
                        close: false,
                    });
                }
                Some(event)
            }
            PathEvent::Quadratic { from, ctrl, to } => {
                // using fast approximate AABB for quadratic bezier
                let min_x = from.x.min(ctrl.x).min(to.x);
                let min_y = from.y.min(ctrl.y).min(to.y);
                let max_x = from.x.max(ctrl.x).max(to.x);
                let max_y = from.y.max(ctrl.y).max(to.y);

                let quad_bounds =
                    egui::Rect::from_min_max(egui::pos2(min_x, min_y), egui::pos2(max_x, max_y));

                let keep = bounds.intersects(quad_bounds);
                if !keep {
                    event_queued = Some(PathEvent::Begin { at: to });
                    return Some(PathEvent::End {
                        last: from,
                        first: from,
                        close: false,
                    });
                }
                Some(event)
            }
            PathEvent::Cubic {
                from,
                ctrl1,
                ctrl2,
                to,
            } => {
                // using fast approximate AABB for cubic bezier
                let min_x = from.x.min(ctrl1.x).min(ctrl2.x).min(to.x);
                let min_y = from.y.min(ctrl1.y).min(ctrl2.y).min(to.y);
                let max_x = from.x.max(ctrl1.x).max(ctrl2.x).max(to.x);
                let max_y = from.y.max(ctrl1.y).max(ctrl2.y).max(to.y);

                let cubic_bounds =
                    egui::Rect::from_min_max(egui::pos2(min_x, min_y), egui::pos2(max_x, max_y));

                let keep = bounds.intersects(cubic_bounds);

                if !keep {
                    event_queued = Some(PathEvent::Begin { at: to });
                    return Some(PathEvent::End {
                        last: from,
                        first: from,
                        close: false,
                    });
                }
                Some(event)
            }
            PathEvent::End { .. } => Some(event),
        }
    })
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
