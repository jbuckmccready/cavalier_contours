use std::sync::Arc;

use cavalier_contours::{
    core::math::angle_from_bulge,
    polyline::{PlineSource as _, Polyline, seg_arc_radius_and_center},
    static_aabb2d_index::AABB,
};
use egui::epaint;
use egui_plot::{PlotItem, PlotPoint, PlotTransform};
use lyon::{
    path::Path,
    tessellation::{
        BuffersBuilder, FillOptions, FillTessellator, StrokeOptions, StrokeTessellator,
        VertexBuffers,
    },
};

use crate::plotting::plotbounds_to_aabb;

use super::{PLOT_VERTEX_RADIUS, VertexConstructor, aabb_to_plotbounds, cull_path, lyon_point};

pub struct PlinePlotItem<'a> {
    pub polyline: &'a Polyline,
    pub vertex_color: epaint::Color32,
    pub stroke_color: epaint::Color32,
    pub fill_color: epaint::Color32,
    id: Option<egui::Id>,
}

impl<'a> PlinePlotItem<'a> {
    pub fn new(polyline: &'a Polyline) -> Self {
        Self {
            polyline,
            vertex_color: epaint::Color32::TRANSPARENT,
            stroke_color: epaint::Color32::TRANSPARENT,
            fill_color: epaint::Color32::TRANSPARENT,
            id: None,
        }
    }

    pub fn vertex_color(mut self, color: epaint::Color32) -> Self {
        self.vertex_color = color;
        self
    }

    pub fn stroke_color(mut self, color: epaint::Color32) -> Self {
        self.stroke_color = color;
        self
    }

    pub fn fill_color(mut self, color: epaint::Color32) -> Self {
        self.fill_color = color;
        self
    }

    pub fn id(mut self, id: egui::Id) -> Self {
        self.id = Some(id);
        self
    }
}

impl PlotItem for PlinePlotItem<'_> {
    fn shapes(&self, _ui: &egui::Ui, transform: &PlotTransform, shapes: &mut Vec<egui::Shape>) {
        if self.polyline.is_empty() {
            return;
        }

        // scale using x value (assuming uniform scaling)
        let scaling = transform.dpos_dvalue_x();

        let mut builder = Path::svg_builder();

        let plot_bounds = plotbounds_to_aabb(transform.bounds());

        if self.stroke_color != epaint::Color32::TRANSPARENT
            || self.fill_color != epaint::Color32::TRANSPARENT
        {
            let mut initial = true;
            for (v1, v2) in self.polyline.iter_segments() {
                let p1 = lyon_point(v1.pos(), transform);
                let p2 = lyon_point(v2.pos(), transform);

                if initial {
                    builder.move_to(p1);
                    initial = false;
                }

                if v1.bulge_is_zero() {
                    builder.line_to(p2);
                } else {
                    let (r, c) = seg_arc_radius_and_center(v1, v2);

                    let radius = (scaling * r) as f32;
                    let sweep_angle = angle_from_bulge(v1.bulge);

                    builder.arc(
                        lyon_point(c, transform),
                        lyon::path::math::vector(radius, radius),
                        lyon::geom::Angle {
                            // negate the sweep angle because y axis is flipped
                            radians: -sweep_angle as f32,
                        },
                        lyon::geom::Angle { radians: 0.0 },
                    );
                }
            }
        }

        let path = builder.build();

        if self.fill_color != epaint::Color32::TRANSPARENT {
            let mut lyon_mesh: VertexBuffers<_, u32> = VertexBuffers::new();
            let mut fill_tess = FillTessellator::new();
            fill_tess
                .tessellate_path(
                    path.as_slice(),
                    &FillOptions::DEFAULT,
                    &mut BuffersBuilder::new(
                        &mut lyon_mesh,
                        VertexConstructor {
                            color: self.fill_color,
                        },
                    ),
                )
                .unwrap();

            let mesh = epaint::Mesh {
                vertices: lyon_mesh.vertices,
                indices: lyon_mesh.indices,
                texture_id: Default::default(),
            };
            shapes.push(egui::Shape::mesh(Arc::new(mesh)));
        }

        if self.stroke_color != epaint::Color32::TRANSPARENT {
            // screen bounds of the plot for culling
            let screen_plot_bounds = {
                let min_pt = transform.position_from_point(&PlotPoint::new(
                    plot_bounds.min_x as f64,
                    plot_bounds.min_y as f64,
                ));
                let max_pt = transform.position_from_point(&PlotPoint::new(
                    plot_bounds.max_x as f64,
                    plot_bounds.max_y as f64,
                ));

                // NOTE: y axis is flipped as ui coordinates have y positive going down
                AABB::new(
                    min_pt.x, max_pt.y, // y axis is flipped
                    max_pt.x, min_pt.y, // y axis is flipped
                )
            };

            // cull path to only include segments within the plot bounds, this is performance
            // benefit as it avoids tessellating stroke segments that are not visible which is
            // significant when zooming in as the number of triangles generated can be very large
            let stroke_path = cull_path(&path, &screen_plot_bounds);
            let mut lyon_mesh: VertexBuffers<_, u32> = VertexBuffers::new();
            let mut stroke_tess = StrokeTessellator::new();
            let line_width = 1.0;

            stroke_tess
                .tessellate(
                    stroke_path,
                    &StrokeOptions::DEFAULT.with_line_width(line_width),
                    &mut BuffersBuilder::new(
                        &mut lyon_mesh,
                        VertexConstructor {
                            color: self.stroke_color,
                        },
                    ),
                )
                .unwrap();
            let mesh = epaint::Mesh {
                vertices: lyon_mesh.vertices,
                indices: lyon_mesh.indices,
                texture_id: Default::default(),
            };
            shapes.push(egui::Shape::mesh(Arc::new(mesh)));
        }

        if self.vertex_color != epaint::Color32::TRANSPARENT {
            for v in self.polyline.iter_vertexes() {
                if v.x < plot_bounds.min_x
                    || v.x > plot_bounds.max_x
                    || v.y < plot_bounds.min_y
                    || v.y > plot_bounds.max_y
                {
                    // out of plot bounds cull from plotting
                    continue;
                }
                shapes.push(egui::Shape::circle_filled(
                    transform.position_from_point(&PlotPoint::new(v.x, v.y)),
                    PLOT_VERTEX_RADIUS,
                    self.vertex_color,
                ));
            }
        }
    }

    fn initialize(&mut self, _x_range: std::ops::RangeInclusive<f64>) {}

    fn name(&self) -> &str {
        "Polyline"
    }

    fn color(&self) -> egui::Color32 {
        self.fill_color
    }

    fn highlight(&mut self) {}

    fn highlighted(&self) -> bool {
        false
    }

    fn allow_hover(&self) -> bool {
        false
    }

    fn geometry(&self) -> egui_plot::PlotGeometry<'_> {
        unimplemented!()
    }

    fn bounds(&self) -> egui_plot::PlotBounds {
        if let Some(aabb) = self.polyline.extents() {
            aabb_to_plotbounds(&aabb)
        } else {
            egui_plot::PlotBounds::NOTHING
        }
    }

    fn id(&self) -> Option<egui::Id> {
        self.id
    }
}
