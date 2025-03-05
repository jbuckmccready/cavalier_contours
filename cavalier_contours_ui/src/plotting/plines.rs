use std::sync::Arc;

use cavalier_contours::{
    core::math::angle_from_bulge,
    polyline::{PlineSource, Polyline, seg_arc_radius_and_center},
    shape_algorithms::Shape,
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

use super::{
    PLOT_VERTEX_RADIUS, VertexConstructor, aabb_to_plotbounds, cull_path, empty_aabb, lyon_point,
    plot_bounds_valid,
};

/// Core trait for plotting polylines data, supports plotting multiple polylines.
pub trait PlinesPlotData {
    type Source: PlineSource<Num = f64>;

    // `visitor` is a closure that takes a polyline and its bounds used to plot the polyline (bounds used
    // to cull from scene). When implementing this trait, the visitor should be called for each
    // polyline (along with its bounds) to be plotted.
    fn plines<F>(&self, visitor: F)
    where
        F: FnMut(&Self::Source, AABB);

    // Total bounds of all polylines (used for zoom to fit and culling from scene).
    fn bounds(&self) -> AABB;
}

// blanket impl for references to PlinesPlotData (allows passing in both owned and borrowed data)
impl<T> PlinesPlotData for &T
where
    T: PlinesPlotData,
{
    type Source = T::Source;

    fn plines<F>(&self, visitor: F)
    where
        F: FnMut(&Self::Source, AABB),
    {
        (*self).plines(visitor);
    }

    fn bounds(&self) -> AABB {
        (*self).bounds()
    }
}

/// Simple wrapper to create plot data for a single polyline.
pub struct PlinePlotData<'a> {
    /// Reference to the polyline.
    pub pline: &'a Polyline,
    /// Bounds of the polyline.
    pub bounds: AABB,
}

impl<'a> PlinePlotData<'a> {
    pub fn new(pline: &'a Polyline) -> Self {
        let bounds = match pline.vertex_count() {
            0 => empty_aabb(),
            1 => {
                let v = pline[0];
                AABB::new(v.x, v.y, v.x, v.y)
            }
            _ => {
                let bounds = pline.extents();
                bounds.unwrap_or_else(empty_aabb)
            }
        };
        Self { pline, bounds }
    }
}

impl PlinesPlotData for PlinePlotData<'_> {
    type Source = Polyline;

    fn plines<F>(&self, mut visitor: F)
    where
        F: FnMut(&Self::Source, AABB),
    {
        visitor(self.pline, self.bounds);
    }

    fn bounds(&self) -> AABB {
        self.bounds
    }
}

impl PlinesPlotData for Shape<f64> {
    type Source = Polyline;

    fn plines<F>(&self, mut visitor: F)
    where
        F: FnMut(&Self::Source, AABB),
    {
        for indexed_pline in self.ccw_plines.iter().chain(self.cw_plines.iter()) {
            let pline = &indexed_pline.polyline;
            let bounds = indexed_pline
                .spatial_index
                .bounds()
                .unwrap_or_else(empty_aabb);

            visitor(pline, bounds);
        }
    }

    fn bounds(&self) -> AABB {
        self.plines_index.bounds().unwrap_or_else(empty_aabb)
    }
}

/// Plot item for plotting multiple polylines.
pub struct PlinesPlotItem<T> {
    data: T,
    vertex_color: epaint::Color32,
    stroke_color: epaint::Color32,
    fill_color: epaint::Color32,
    id: Option<egui::Id>,
}

impl<T> PlinesPlotItem<T> {
    pub fn new(data: T) -> Self {
        Self {
            data,
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

impl<T> PlotItem for PlinesPlotItem<T>
where
    T: PlinesPlotData,
{
    fn shapes(&self, _ui: &egui::Ui, transform: &PlotTransform, shapes: &mut Vec<egui::Shape>) {
        if !plot_bounds_valid(transform.bounds()) {
            return;
        }
        let plot_bounds = plotbounds_to_aabb(transform.bounds());
        if !plot_bounds.overlaps_aabb(&self.data.bounds()) {
            return;
        }

        // scale using x value (assuming uniform scaling)
        let scaling = transform.dpos_dvalue_x();

        let mut builder = Path::svg_builder();

        if self.stroke_color != epaint::Color32::TRANSPARENT
            || self.fill_color != epaint::Color32::TRANSPARENT
        {
            self.data.plines(|pline, bounds| {
                // cull plines that are not visible
                if !bounds.overlaps_aabb(&plot_bounds) {
                    return;
                }
                let mut initial = true;
                for (v1, v2) in pline.iter_segments() {
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
            });
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
            // cull path to only include segments within the plot bounds, this is performance
            // benefit as it avoids tessellating stroke segments that are not visible which is
            // significant when zooming in as the number of triangles generated can be very large
            let stroke_path = cull_path(&path, transform.frame());
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
            self.data.plines(|pline, _| {
                for v in pline.iter_vertexes() {
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
            });
        }
    }

    fn initialize(&mut self, _x_range: std::ops::RangeInclusive<f64>) {}

    fn name(&self) -> &str {
        "Polylines"
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
        aabb_to_plotbounds(&self.data.bounds())
    }

    fn id(&self) -> Option<egui::Id> {
        self.id
    }
}
