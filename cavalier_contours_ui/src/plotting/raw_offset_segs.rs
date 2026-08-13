use std::sync::Arc;

use cavalier_contours::polyline::internal::raw_pline_offset::RawOffsetSeg;
use egui::epaint;
use egui_plot::PlotItem;
use lyon::{
    path::builder::WithSvg,
    tessellation::{BuffersBuilder, StrokeOptions, StrokeTessellator, VertexBuffers},
};

use super::{VertexConstructor, aabb_to_plotbounds, cull_path, lyon_point, plot_bounds_valid};

pub struct RawOffsetSegsPlotItem<'a> {
    pub segs: &'a [RawOffsetSeg<f64>],
    pub color: epaint::Color32,
    pub collapsed_color: epaint::Color32,
    base: egui_plot::PlotItemBase,
}

impl<'a> RawOffsetSegsPlotItem<'a> {
    #[must_use]
    pub fn new(segs: &'a [RawOffsetSeg<f64>]) -> Self {
        Self {
            segs,
            color: epaint::Color32::GRAY,
            collapsed_color: epaint::Color32::DARK_GRAY,
            base: egui_plot::PlotItemBase::new("Raw Offset Segments".to_string()),
        }
    }

    #[must_use]
    pub fn color(mut self, color: epaint::Color32) -> Self {
        self.color = color;
        self
    }

    #[must_use]
    pub fn collapsed_color(mut self, color: epaint::Color32) -> Self {
        self.collapsed_color = color;
        self
    }
}

impl PlotItem for RawOffsetSegsPlotItem<'_> {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "lyon drawing types use f32"
    )]
    fn shapes(
        &self,
        _ui: &egui::Ui,
        transform: &egui_plot::PlotTransform,
        shapes: &mut Vec<egui::Shape>,
    ) {
        if !plot_bounds_valid(transform.bounds()) || self.segs.is_empty() {
            return;
        }

        if self.color == epaint::Color32::TRANSPARENT
            && self.collapsed_color == epaint::Color32::TRANSPARENT
        {
            return;
        }

        let scaling = transform.dpos_dvalue_x();
        let mut lyon_mesh: VertexBuffers<_, u32> = VertexBuffers::new();
        let mut stroke_tess = StrokeTessellator::new();

        for segment in self.segs {
            let color = if matches!(segment, RawOffsetSeg::Collapsed(_)) {
                self.collapsed_color
            } else {
                self.color
            };
            if color == epaint::Color32::TRANSPARENT {
                continue;
            }

            let mut builder = WithSvg::<lyon::path::Builder>::new(lyon::path::Builder::new());
            builder.move_to(lyon_point(segment.start(), transform));
            match segment {
                RawOffsetSeg::Line(line) => {
                    builder.line_to(lyon_point(line.end, transform));
                }
                RawOffsetSeg::Collapsed(arc) => {
                    builder.line_to(lyon_point(arc.end, transform));
                }
                RawOffsetSeg::Arc(arc) => {
                    let radius = (scaling * arc.radius) as f32;
                    builder.arc(
                        lyon_point(arc.center, transform),
                        lyon::path::math::vector(radius, radius),
                        lyon::geom::Angle {
                            radians: -arc.sweep as f32,
                        },
                        lyon::geom::Angle { radians: 0.0 },
                    );
                }
            }

            let path = builder.build();
            let stroke_path = cull_path(&path, transform.frame());
            stroke_tess
                .tessellate(
                    stroke_path,
                    &StrokeOptions::DEFAULT.with_line_width(1.0),
                    &mut BuffersBuilder::new(&mut lyon_mesh, VertexConstructor { color }),
                )
                .unwrap();
        }

        shapes.push(egui::Shape::mesh(Arc::new(epaint::Mesh {
            vertices: lyon_mesh.vertices,
            indices: lyon_mesh.indices,
            texture_id: epaint::TextureId::default(),
        })));
    }

    fn initialize(&mut self, _x_range: std::ops::RangeInclusive<f64>) {}

    fn name(&self) -> &'static str {
        "RawOffsetSegs"
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
            .map(RawOffsetSeg::bounding_box)
            .map(|bounds| aabb_to_plotbounds(&bounds))
            .fold(egui_plot::PlotBounds::NOTHING, |mut acc, bounds| {
                acc.merge(&bounds);
                acc
            })
    }

    fn base(&self) -> &egui_plot::PlotItemBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut egui_plot::PlotItemBase {
        &mut self.base
    }
}
