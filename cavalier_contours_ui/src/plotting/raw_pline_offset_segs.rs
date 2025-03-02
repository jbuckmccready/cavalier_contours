use std::sync::Arc;

use cavalier_contours::{
    core::math::angle_from_bulge,
    polyline::{
        internal::pline_offset::RawPlineOffsetSeg, seg_arc_radius_and_center, seg_bounding_box,
    },
};
use egui::epaint;
use egui_plot::PlotItem;
use lyon::{
    path::builder::WithSvg,
    tessellation::{BuffersBuilder, StrokeOptions, StrokeTessellator, VertexBuffers},
};

use super::{VertexConstructor, aabb_to_plotbounds, lyon_point};

pub struct RawPlineOffsetSegsPlotItem<'a> {
    pub segs: &'a [RawPlineOffsetSeg<f64>],
    pub color: epaint::Color32,
    pub collapsed_color: epaint::Color32,
    id: Option<egui::Id>,
}

impl<'a> RawPlineOffsetSegsPlotItem<'a> {
    pub fn new(segs: &'a [RawPlineOffsetSeg<f64>]) -> Self {
        Self {
            segs,
            color: epaint::Color32::PURPLE,
            collapsed_color: epaint::Color32::LIGHT_RED,
            id: None,
        }
    }

    pub fn color(mut self, color: epaint::Color32) -> Self {
        self.color = color;
        self
    }

    pub fn collapsed_color(mut self, color: epaint::Color32) -> Self {
        self.collapsed_color = color;
        self
    }

    pub fn id(mut self, id: egui::Id) -> Self {
        self.id = Some(id);
        self
    }
}

impl PlotItem for RawPlineOffsetSegsPlotItem<'_> {
    fn shapes(
        &self,
        _ui: &egui::Ui,
        transform: &egui_plot::PlotTransform,
        shapes: &mut Vec<egui::Shape>,
    ) {
        if self.segs.is_empty() {
            return;
        }

        if self.color == epaint::Color32::TRANSPARENT
            && self.collapsed_color == epaint::Color32::TRANSPARENT
        {
            return;
        }

        // scale using x value (assuming uniform scaling)
        let scaling = transform.dpos_dvalue_x();

        let mut lyon_mesh: VertexBuffers<_, u32> = VertexBuffers::new();
        let mut stroke_tess = StrokeTessellator::new();
        let line_width = 1.0;

        for seg in self.segs {
            let color = if seg.collapsed_arc {
                self.collapsed_color
            } else {
                self.color
            };

            if color == epaint::Color32::TRANSPARENT {
                continue;
            }

            let mut builder = WithSvg::<lyon::path::Builder>::new(lyon::path::Builder::new());
            let p1 = lyon_point(seg.v1.pos(), transform);
            let p2 = lyon_point(seg.v2.pos(), transform);

            builder.move_to(p1);
            if seg.v1.bulge_is_zero() {
                builder.line_to(p2);
            } else {
                let (r, c) = seg_arc_radius_and_center(seg.v1, seg.v2);

                let radius = (scaling * r) as f32;
                let sweep_angle = angle_from_bulge(seg.v1.bulge);

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

            let path = builder.build();

            stroke_tess
                .tessellate_path(
                    path.as_slice(),
                    &StrokeOptions::DEFAULT.with_line_width(line_width),
                    &mut BuffersBuilder::new(&mut lyon_mesh, VertexConstructor { color }),
                )
                .unwrap();
        }

        let mesh = epaint::Mesh {
            vertices: lyon_mesh.vertices,
            indices: lyon_mesh.indices,
            texture_id: Default::default(),
        };

        shapes.push(egui::Shape::mesh(Arc::new(mesh)));
    }

    fn initialize(&mut self, _x_range: std::ops::RangeInclusive<f64>) {}

    fn name(&self) -> &str {
        "RawPlineOffsetSegs"
    }

    fn color(&self) -> egui::Color32 {
        self.color
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
        self.segs
            .iter()
            .map(|s| aabb_to_plotbounds(&seg_bounding_box(s.v1, s.v2)))
            .fold(egui_plot::PlotBounds::NOTHING, |mut acc, bounds| {
                acc.merge(&bounds);
                acc
            })
    }

    fn id(&self) -> Option<egui::Id> {
        self.id
    }
}
