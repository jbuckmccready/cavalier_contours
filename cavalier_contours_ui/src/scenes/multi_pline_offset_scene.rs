use cavalier_contours::{
    pline_closed,
    polyline::{PlineSource, PlineSourceMut, Polyline},
    shape_algorithms::{Shape, ShapeOffsetOptions},
};
use eframe::egui::{CentralPanel, Color32, Rect, ScrollArea, SidePanel, Slider, Ui, Vec2};
use egui::Id;
use egui_plot::{Plot, PlotPoint};

use crate::plotting::ShapePlotItem;

use super::{super::plotting::PLOT_VERTEX_RADIUS, Scene};

pub struct MultiPlineOffsetScene {
    plines: Vec<Polyline>,
    max_offset_count: usize,
    offset: f64,
    interaction_state: InteractionState,
}

struct InteractionState {
    grabbed_vertex: Option<(usize, usize)>,
    dragging: bool,
    zoom_to_fit: bool,
}

impl Default for MultiPlineOffsetScene {
    fn default() -> Self {
        let plines = vec![
            pline_closed![
                (100.0, 100.0, -0.5),
                (80.0, 90.0, 0.374794619217547),
                (210.0, 0.0, 0.0),
                (230.0, 0.0, 1.0),
                (320.0, 0.0, -0.5),
                (280.0, 0.0, 0.5),
                (390.0, 210.0, 0.0),
                (280.0, 120.0, 0.5),
            ],
            pline_closed![
                (150.0, 50.0, 0.0),
                (150.0, 100.0, 0.0),
                (223.74732137849435, 142.16931273980475, 0.0),
                (199.491310072685, 52.51543504258919, 0.5),
            ],
            pline_closed![
                (261.11232783167395, 35.79686193615828, -1.0),
                (250.0, 100.0, -1.0),
            ],
            pline_closed![
                (320.5065990423979, 76.14222955572362, -1.0),
                (320.2986109239592, 103.52378781211337, 0.0),
            ],
            pline_closed![
                (273.6131273938006, -13.968608715397636, -0.3),
                (256.61336060995995, -25.49387433156079, 0.0),
                (249.69820124026208, 27.234215862385582, 0.0),
            ],
        ];

        Self {
            plines,
            max_offset_count: 25,
            offset: 2.0,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
        }
    }
}

impl Scene for MultiPlineOffsetScene {
    fn name(&self) -> &'static str {
        "Multi Polyline Offset"
    }

    fn ui(&mut self, ui: &mut Ui, init: bool) {
        let MultiPlineOffsetScene {
            plines,
            max_offset_count,
            offset,
            interaction_state,
        } = self;

        controls_panel(ui, max_offset_count, offset, interaction_state);

        interaction_state.zoom_to_fit |= init;
        plot_area(ui, plines, max_offset_count, offset, interaction_state);
    }
}

fn controls_panel(
    ui: &mut Ui,
    max_offset_count: &mut usize,
    offset: &mut f64,
    interaction_state: &mut InteractionState,
) {
    SidePanel::right("pline_offset_panel")
        .min_width(200.0)
        .default_width(20.0)
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                // state used to fill available width within the scroll area
                let last_others_width_id = Id::new("panel_width_state");
                let this_init_max_width = ui.max_rect().width();
                let last_others_width = ui.data(|data| {
                    data.get_temp(last_others_width_id)
                        .unwrap_or(this_init_max_width)
                });

                let this_target_width = this_init_max_width - last_others_width;
                ui.style_mut().spacing.slider_width = this_target_width;

                egui::Frame::default()
                    .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                    .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                    .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                    .show(ui, |ui| {
                        ui.label("Offset");
                        ui.add(Slider::new(offset, -100.0..=100.0).step_by(0.5));
                    });

                egui::Frame::default()
                    .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                    .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                    .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                    .show(ui, |ui| {
                        ui.label("Max Offset Count");
                        ui.add(
                            Slider::new(max_offset_count, 0..=100)
                                .integer()
                                .step_by(1.0),
                        );
                    });

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();

                ui.data_mut(|data| {
                    data.insert_temp(
                        last_others_width_id,
                        ui.min_rect().width() - this_target_width,
                    )
                });
            })
        });
}

fn plot_area(
    ui: &mut Ui,
    plines: &mut [Polyline],
    max_offset_count: &usize,
    offset: &f64,
    interaction_state: &mut InteractionState,
) {
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    // TODO: cache scene state to only update when necessary due to modified polyline or offset
    let (shape, offset_shapes) = build_offsets(plines, offset, max_offset_count);

    CentralPanel::default().show_inside(ui, |ui| {
        let plot = Plot::new("plot_area").data_aspect(1.0).allow_drag(false);

        plot.show(ui, |plot_ui| {
            plot_ui.set_auto_bounds([false, false]);

            if plot_ui.ctx().input(|i| i.pointer.any_released()) {
                if grabbed_vertex.is_none() {
                    *dragging = false;
                } else {
                    // release vertex when pointer released
                    *grabbed_vertex = None;
                }
            }

            if let Some((grabbed_pl, grabbed_idx)) = *grabbed_vertex {
                // move grabbed point by drag delta by offsetting point position
                let delta = plot_ui.pointer_coordinate_drag_delta();
                let pline = plines.get_mut(grabbed_pl).unwrap();
                let grabbed_vertex = pline.get(grabbed_idx).unwrap();
                pline.set(
                    grabbed_idx,
                    grabbed_vertex.x + delta.x as f64,
                    grabbed_vertex.y + delta.y as f64,
                    grabbed_vertex.bulge,
                );
            } else if *dragging {
                plot_ui.translate_bounds(-plot_ui.pointer_coordinate_drag_delta());
            } else if plot_ui.ctx().input(|i| i.pointer.any_pressed()) {
                // pointer pressed, check if point grabbed by iterating through points and checking
                // if point considered "hit"
                if let Some(coord) = plot_ui.ctx().pointer_interact_pos() {
                    let iter = plines.iter().enumerate().flat_map(|(pl_idx, pl)| {
                        // required for closure move
                        let plot_ui = &plot_ui;
                        pl.iter_vertexes().enumerate().map(move |(i, v)| {
                            (
                                // pline index
                                pl_idx,
                                // vertex index
                                i,
                                // vertex position
                                plot_ui.screen_from_plot(PlotPoint::new(v.x, v.y)),
                            )
                        })
                    });

                    for (pl_idx, i, pt) in iter {
                        let hit_size =
                            2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);

                        let hit_box = Rect::from_center_size(pt, Vec2::splat(hit_size));

                        if hit_box.contains(coord) {
                            // update grabbed point
                            *grabbed_vertex = Some((pl_idx, i));
                            break;
                        }
                    }

                    *dragging = grabbed_vertex.is_none();
                }
            }

            plot_ui.add(
                ShapePlotItem::new(&shape)
                    .stroke_color(Color32::GOLD)
                    .vertex_color(Color32::LIGHT_GREEN),
            );

            for shape in offset_shapes.iter() {
                plot_ui.add(ShapePlotItem::new(&shape).stroke_color(Color32::LIGHT_BLUE));
            }

            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });
}

fn build_offsets(
    plines: &[Polyline],
    offset: &f64,
    max_offset_count: &usize,
) -> (Shape<f64>, Vec<Shape<f64>>) {
    let shape = Shape::from_plines(plines.iter().cloned());
    let mut offset_shapes = Vec::new();
    if *max_offset_count == 0 {
        return (shape, offset_shapes);
    }

    let mut curr_offset = shape.parallel_offset(*offset, ShapeOffsetOptions::default());

    while !curr_offset.ccw_plines.is_empty() || !curr_offset.cw_plines.is_empty() {
        offset_shapes.push(curr_offset);
        if offset_shapes.len() >= *max_offset_count {
            break;
        }

        curr_offset = offset_shapes
            .last()
            .unwrap()
            .parallel_offset(*offset, ShapeOffsetOptions::default());
    }

    (shape, offset_shapes)
}
