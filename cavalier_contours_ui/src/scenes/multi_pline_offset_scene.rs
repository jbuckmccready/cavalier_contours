use cavalier_contours::{
    pline_closed,
    polyline::{PlineCreation, PlineSource, PlineSourceMut, Polyline},
    shape_algorithms::{Shape, ShapeOffsetOptions},
};
use eframe::egui::{Rect, ScrollArea, Slider, Ui, Vec2};
use egui::Id;
use egui_plot::{Plot, PlotPoint, PlotPoints};

use crate::editor::PolylineEditor;
use crate::plotting::{PlinePlotData, PlinesPlotItem};

use super::{
    super::plotting::PLOT_VERTEX_RADIUS, Scene, controls_side_panel, scene_settings::SceneSettings,
};

#[derive(Clone, Copy, PartialEq, Default)]
enum Mode {
    #[default]
    Offset,
    OffsetIntersects,
    Slices,
}

impl Mode {
    fn label(&self) -> &'static str {
        match self {
            Mode::Offset => "Offset",
            Mode::OffsetIntersects => "Offset Intersects",
            Mode::Slices => "Slices",
        }
    }
}

pub struct MultiPlineOffsetScene {
    plines: Vec<Polyline>,
    mode: Mode,
    max_offset_count: usize,
    offset: f64,
    interaction_state: InteractionState,
    polyline_editor: PolylineEditor,
}

struct InteractionState {
    grabbed_vertex: Option<(usize, usize)>,
    dragging: bool,
    zoom_to_fit: bool,
}

enum SceneState {
    Offset {
        shape: Shape<f64>,
        offset_shapes: Vec<Shape<f64>>,
    },
    OffsetIntersects {
        shape: Shape<f64>,
        offset_loops: Vec<Polyline>,
        intersects: Vec<PlotPoint>,
    },
    Slices {
        slice_plines: Vec<Polyline>,
    },
    NoOp,
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

        let mut polyline_editor = PolylineEditor::multi("Multi-Polyline Editor");
        polyline_editor.initialize_with_polylines(plines.clone());

        Self {
            plines,
            mode: Mode::default(),
            max_offset_count: 25,
            offset: 2.0,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
            polyline_editor,
        }
    }
}

impl Scene for MultiPlineOffsetScene {
    fn name(&self) -> &'static str {
        "Multi Polyline Offset"
    }

    fn ui(&mut self, ui: &mut Ui, settings: &SceneSettings, init: bool) {
        let MultiPlineOffsetScene {
            plines,
            mode,
            max_offset_count,
            offset,
            interaction_state,
            polyline_editor,
        } = self;

        controls_panel(
            ui,
            mode,
            max_offset_count,
            offset,
            interaction_state,
            polyline_editor,
        );

        interaction_state.zoom_to_fit |= init;

        // Show multi-polyline editor window if requested
        let colors = settings.colors(ui.ctx());
        polyline_editor.ui_show(ui.ctx(), plines, &colors);

        plot_area(
            ui,
            settings,
            plines,
            mode,
            max_offset_count,
            offset,
            interaction_state,
        );
    }
}

fn controls_panel(
    ui: &mut Ui,
    mode: &mut Mode,
    max_offset_count: &mut usize,
    offset: &mut f64,
    interaction_state: &mut InteractionState,
    polyline_editor: &mut PolylineEditor,
) {
    controls_side_panel("multi_pline_offset_panel")
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                ui.horizontal(|ui| {
                    ui.label("Mode:");
                    egui::ComboBox::from_id_salt("mode_combo")
                        .selected_text(mode.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(mode, Mode::Offset, Mode::Offset.label())
                                .on_hover_text("Final offset result");
                            ui.selectable_value(mode, Mode::OffsetIntersects, Mode::OffsetIntersects.label())
                                .on_hover_text("Shows offset for each polyline in shape and intersect points between the polylines");
                            ui.selectable_value(mode, Mode::Slices, Mode::Slices.label())
                                .on_hover_text("Shows slices remaining, each with different color, after discarding invalid slices");
                        });
                });

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
                        ui.label("Offset").on_hover_text("Parallel offset distance, positive value will offset to the left of curve direction");
                        ui.add(Slider::new(offset, -100.0..=100.0));
                    });

                if *mode == Mode::Offset {
                    egui::Frame::default()
                        .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                        .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                        .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                        .show(ui, |ui| {
                            ui.label("Max Offset Count").on_hover_text("Maximum number of parallel offsets to generate (stops early when orientation changes)");
                            ui.add(
                                Slider::new(max_offset_count, 0..=100)
                                    .integer()
                                    .step_by(1.0),
                            );
                        });
                }

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();

                // Multi-polyline editor button
                if ui.button("Edit Polylines").on_hover_text("Edit all polylines vertex data").clicked() {
                    polyline_editor.show_window();
                }

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
    settings: &SceneSettings,
    plines: &mut [Polyline],
    mode: &Mode,
    max_offset_count: &usize,
    offset: &f64,
    interaction_state: &mut InteractionState,
) {
    let colors = settings.colors(ui.ctx());
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    let scene_state = build_scene_state(plines, offset, max_offset_count, mode);
    let plot = settings
        .apply_to_plot(Plot::new("multi_pline_offset_scene"))
        .data_aspect(1.0)
        .allow_drag(false);

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

        match &scene_state {
            SceneState::Offset {
                shape,
                offset_shapes,
            } => {
                plot_ui.add(
                    PlinesPlotItem::new(shape)
                        .stroke_color(colors.accent_stroke)
                        .vertex_color(colors.vertex_color),
                );

                for shape in offset_shapes.iter() {
                    plot_ui.add(PlinesPlotItem::new(shape).stroke_color(colors.primary_stroke));
                }
            }
            SceneState::OffsetIntersects {
                shape,
                offset_loops,
                intersects,
            } => {
                plot_ui.add(
                    PlinesPlotItem::new(shape)
                        .stroke_color(colors.accent_stroke)
                        .vertex_color(colors.vertex_color),
                );
                for pline in offset_loops {
                    plot_ui.add(
                        PlinesPlotItem::new(PlinePlotData::new(pline))
                            .stroke_color(colors.primary_stroke),
                    );
                }
                // plot intersects
                let points = egui_plot::Points::new(
                    "Intersections",
                    PlotPoints::from(intersects.as_slice()),
                )
                .radius(PLOT_VERTEX_RADIUS)
                .color(colors.error_color);
                plot_ui.points(points);
            }
            SceneState::Slices { slice_plines } => {
                // plot slices
                for (i, slice_pline) in slice_plines.iter().enumerate() {
                    plot_ui.add(
                        PlinesPlotItem::new(PlinePlotData::new(slice_pline))
                            .stroke_color(colors.get_multi_color(i)),
                    );
                }
            }
            SceneState::NoOp => {
                // plot original plines directly
                for pline in plines {
                    plot_ui.add(
                        PlinesPlotItem::new(PlinePlotData::new(pline))
                            .stroke_color(colors.accent_stroke)
                            .vertex_color(colors.vertex_color),
                    );
                }
            }
        }

        if *zoom_to_fit {
            plot_ui.set_auto_bounds([true, true]);
        }
    });
}

fn build_scene_state(
    plines: &[Polyline],
    offset: &f64,
    max_offset_count: &usize,
    mode: &Mode,
) -> SceneState {
    let shape = Shape::from_plines(plines.iter().cloned());
    if *mode == Mode::Offset {
        let mut offset_shapes = Vec::new();
        if *max_offset_count == 0 {
            return SceneState::Offset {
                shape,
                offset_shapes,
            };
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
        return SceneState::Offset {
            shape,
            offset_shapes,
        };
    }

    let options = ShapeOffsetOptions::default();

    // Step 1: Create offset loops with spatial index
    let (ccw_offset_loops, cw_offset_loops, offset_loops_index) =
        shape.create_offset_loops_with_index(*offset, &options);

    if ccw_offset_loops.is_empty() && cw_offset_loops.is_empty() {
        return SceneState::NoOp;
    }

    // Step 2: Find intersects between offset loops
    let slice_point_sets = shape.find_intersects_between_offset_loops(
        &ccw_offset_loops,
        &cw_offset_loops,
        &offset_loops_index,
        options.pos_equal_eps,
    );

    // Convert offset loops to simple polylines for display purposes
    let mut offset_loops: Vec<Polyline> = ccw_offset_loops
        .iter()
        .map(|l| l.indexed_pline.polyline.clone())
        .collect();
    offset_loops.extend(
        cw_offset_loops
            .iter()
            .map(|l| l.indexed_pline.polyline.clone()),
    );

    if *mode == Mode::OffsetIntersects {
        let intersects = slice_point_sets
            .iter()
            .flat_map(|s| {
                s.slice_points
                    .iter()
                    .map(|p| PlotPoint::new(p.point.x, p.point.y))
            })
            .collect();
        return SceneState::OffsetIntersects {
            shape,
            offset_loops,
            intersects,
        };
    }

    // Step 3: Create valid slices from intersects
    let slices_data = shape.create_valid_slices_from_intersects(
        &ccw_offset_loops,
        &cw_offset_loops,
        &slice_point_sets,
        *offset,
        &options,
    );

    let slice_plines = slices_data
        .iter()
        .map(|slice| {
            let source_loop = if slice.source_idx < ccw_offset_loops.len() {
                &ccw_offset_loops[slice.source_idx].indexed_pline.polyline
            } else {
                &cw_offset_loops[slice.source_idx - ccw_offset_loops.len()]
                    .indexed_pline
                    .polyline
            };
            let slice_view = slice.v_data.view(source_loop);
            Polyline::create_from(&slice_view)
        })
        .collect();

    SceneState::Slices { slice_plines }
}
