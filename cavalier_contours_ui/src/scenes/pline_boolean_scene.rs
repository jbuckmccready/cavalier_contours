use cavalier_contours::{
    pline_closed,
    polyline::{
        BooleanOp, BooleanPlineSlice, BooleanResult, PlineCreation, PlineSource, PlineSourceMut,
        PlineViewData, Polyline,
        internal::pline_boolean::{process_for_boolean, prune_slices, slice_at_intersects},
    },
    shape_algorithms::Shape,
};
use eframe::egui::{CentralPanel, Rect, ScrollArea, Ui, Vec2};
use egui_plot::{Plot, PlotPoint, PlotPoints};

use crate::editor::PolylineEditor;
use crate::plotting::{PlinePlotData, PlinesPlotItem};

use super::{
    super::plotting::PLOT_VERTEX_RADIUS, Scene, controls_side_panel, scene_settings::SceneSettings,
};

pub struct PlineBooleanScene {
    plines: Vec<Polyline>,
    mode: Mode,
    fill: bool,
    show_vertexes: bool,
    show_pruned_slices: bool,
    interaction_state: InteractionState,
    polyline_editor: PolylineEditor,
}

#[derive(Default, Clone, Copy, PartialEq)]
enum Mode {
    #[default]
    None,
    Or,
    And,
    Not,
    Xor,
    Intersects,
    Slices,
}

impl Mode {
    fn label(&self) -> &'static str {
        match self {
            Mode::None => "None",
            Mode::Or => "Or",
            Mode::And => "And",
            Mode::Not => "Not",
            Mode::Xor => "Xor",
            Mode::Intersects => "Intersects",
            Mode::Slices => "Slices",
        }
    }

    fn supports_pruned_slices(&self) -> bool {
        matches!(
            self,
            Mode::None | Mode::Or | Mode::And | Mode::Not | Mode::Xor
        )
    }
}

struct InteractionState {
    grabbed_vertex: Option<(usize, usize)>,
    dragging: bool,
    zoom_to_fit: bool,
}

enum SceneState {
    NoOp,
    BooleanResult(BooleanResult<Polyline>),
    BooleanResultWithPrunedSlices {
        result: BooleanResult<Polyline>,
        pruned_slices: Vec<BooleanPlineSlice<f64>>,
        start_of_pline2_slices: usize,
        start_of_pline1_overlapping_slices: usize,
        start_of_pline2_overlapping_slices: usize,
    },
    Intersects {
        intersects: Vec<PlotPoint>,
        overlapping_slices: Vec<(PlineViewData<f64>, bool)>,
    },
    Slices {
        pline1_slices: Vec<BooleanPlineSlice<f64>>,
        pline2_slices: Vec<BooleanPlineSlice<f64>>,
    },
}

impl Default for PlineBooleanScene {
    fn default() -> Self {
        let pline1 = pline_closed![
            (10.0, 10.0, -0.5),
            (0.3, 1.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (39.0, 21.0, 0.0),
            (28.0, 12.0, 0.5),
        ];

        let mut pline2 = pline_closed![
            (10.0, 10.0, -0.5),
            (8.0, 9.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (38.0, 19.0, 0.0),
            (28.0, 12.0, 0.5),
        ];

        pline2.scale_mut(0.5);

        let plines = vec![pline1.clone(), pline2.clone()];
        let mut polyline_editor = PolylineEditor::dual("Polyline Editor");
        polyline_editor.initialize_with_polylines(plines.clone());

        Self {
            plines,
            mode: Mode::default(),
            fill: true,
            show_vertexes: true,
            show_pruned_slices: false,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
            polyline_editor,
        }
    }
}

impl Scene for PlineBooleanScene {
    fn name(&self) -> &'static str {
        "Polyline Boolean"
    }

    fn ui(&mut self, ui: &mut Ui, settings: &SceneSettings, init: bool) {
        let PlineBooleanScene {
            plines,
            mode,
            fill,
            show_vertexes,
            show_pruned_slices,
            interaction_state,
            polyline_editor,
        } = self;

        controls_panel(
            ui,
            mode,
            fill,
            show_vertexes,
            show_pruned_slices,
            interaction_state,
            polyline_editor,
        );

        interaction_state.zoom_to_fit |= init;
        plot_area(
            ui,
            settings,
            plines,
            mode,
            fill,
            show_vertexes,
            show_pruned_slices,
            interaction_state,
            polyline_editor,
        );
    }
}

fn controls_panel(
    ui: &mut Ui,
    mode: &mut Mode,
    fill: &mut bool,
    show_vertexes: &mut bool,
    show_pruned_slices: &mut bool,
    interaction_state: &mut InteractionState,
    polyline_editor: &mut PolylineEditor,
) {
    controls_side_panel("pline_boolean_controls")
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                ui.horizontal(|ui| {
                    ui.label("Mode:");
                    egui::ComboBox::from_id_salt("mode_combo")
                        .selected_text(mode.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(mode, Mode::None, Mode::None.label())
                                .on_hover_text("No boolean operation performed");
                            ui.separator();
                            ui.selectable_value(mode, Mode::Or, Mode::Or.label())
                                .on_hover_text("Union (OR) the closed polylines, keeping both areas");
                            ui.selectable_value(mode, Mode::And, Mode::And.label()).on_hover_text(
                                "Intersection (AND) the closed polylines, keeping only the overlapping area",
                            );
                            ui.selectable_value(mode, Mode::Not, Mode::Not.label())
                                .on_hover_text("Difference (NOT) the closed polylines, keeping only the area of the first polyline not overlapped by the second polyline");
                            ui.selectable_value(mode, Mode::Xor, Mode::Xor.label())
                                .on_hover_text("Symmetric Difference (XOR) the closed polylines, keeping only the areas not overlapped by both polylines");
                            ui.separator();
                            ui.selectable_value(mode, Mode::Intersects, Mode::Intersects.label())
                                .on_hover_text("Show intersection points and overlapping segments");
                            ui.selectable_value(mode, Mode::Slices, Mode::Slices.label())
                                .on_hover_text("Show polylines sliced at intersection points");
                        });
                });

                ui.checkbox(fill, "Fill").on_hover_text("Fill resulting polyline areas");
                ui.checkbox(show_vertexes, "Show Vertexes").on_hover_text("Show polyline vertexes");

                if mode.supports_pruned_slices() {
                    ui.checkbox(show_pruned_slices, "Show Pruned Slices")
                        .on_hover_text("Show slices after pruning based on the selected boolean operation");
                }

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();

                // Dual polyline editor button
                if ui.button("Edit Polylines").on_hover_text("Edit both polylines vertex data").clicked() {
                    polyline_editor.show_window();
                }
            })
        });
}

#[allow(clippy::too_many_arguments)]
fn plot_area(
    ui: &mut Ui,
    settings: &SceneSettings,
    plines: &mut Vec<Polyline>,
    mode: &Mode,
    fill: &bool,
    show_vertexes: &bool,
    show_pruned_slices: &bool,
    interaction_state: &mut InteractionState,
    polyline_editor: &mut PolylineEditor,
) {
    let colors = settings.colors(ui.ctx());
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    let [pline1, pline2] = plines.get_mut(0..2).expect("Expected two polylines") else {
        panic!("Expected two polylines, but found less");
    };

    CentralPanel::default().show_inside(ui, |ui| {
        let plot = settings
            .apply_to_plot(Plot::new("pline_boolean_scene"))
            .data_aspect(1.0)
            .allow_drag(false);

        // stack variables for plot items (required for lifetime)
        let mut shape = Shape::empty();
        let mut plines: Vec<Polyline> = Vec::new();
        let mut intrs = Vec::new();

        plot.show(ui, |plot_ui| {
            plot_ui.set_auto_bounds(false);
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
                if grabbed_pl == 1 {
                    let grabbed_vertex = pline1.get(grabbed_idx).unwrap();
                    pline1.set(
                        grabbed_idx,
                        grabbed_vertex.x + delta.x as f64,
                        grabbed_vertex.y + delta.y as f64,
                        grabbed_vertex.bulge,
                    );
                } else {
                    let grabbed_vertex = pline2.get(grabbed_idx).unwrap();
                    pline2.set(
                        grabbed_idx,
                        grabbed_vertex.x + delta.x as f64,
                        grabbed_vertex.y + delta.y as f64,
                        grabbed_vertex.bulge,
                    );
                }
            } else if *dragging {
                plot_ui.translate_bounds(-plot_ui.pointer_coordinate_drag_delta());
            } else if plot_ui.ctx().input(|i| i.pointer.any_pressed()) {
                // pointer pressed, check if point grabbed by iterating through points and checking
                // if point considered "hit"
                if let Some(coord) = plot_ui.ctx().pointer_interact_pos() {
                    let pline1_iter = pline1.iter_vertexes().enumerate().map(|(i, v)| (1, i, v));
                    let pline2_iter = pline2.iter_vertexes().enumerate().map(|(i, v)| (2, i, v));

                    let iter = pline1_iter.chain(pline2_iter).map(|(pl, i, pt)| {
                        (pl, i, plot_ui.screen_from_plot(PlotPoint::new(pt.x, pt.y)))
                    });

                    for (pl, i, pt) in iter {
                        let hit_size =
                            2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);

                        let hit_box = Rect::from_center_size(pt, Vec2::splat(hit_size));

                        if hit_box.contains(coord) {
                            // update grabbed point
                            *grabbed_vertex = Some((pl, i));
                            break;
                        }
                    }

                    *dragging = grabbed_vertex.is_none();
                }
            }

            // Build scene state after vertex dragging to ensure it uses updated polylines
            let scene_state = build_scene_state(pline1, pline2, mode, show_pruned_slices);

            // TODO: color pickers
            let color1 = colors.primary_stroke;
            let color2 = colors.secondary_stroke;
            let opacity = 0.8;
            let fill_color1 = color1.gamma_multiply(opacity);
            let fill_color2 = color2.gamma_multiply(opacity);

            let mut plot_item1 = PlinesPlotItem::new(PlinePlotData::new(pline1));
            let mut plot_item2 = PlinesPlotItem::new(PlinePlotData::new(pline2));

            if *show_vertexes {
                plot_item1 = plot_item1.vertex_color(fill_color1);
                plot_item2 = plot_item2.vertex_color(fill_color2);
            }

            match scene_state {
                SceneState::NoOp => {
                    if *fill {
                        plot_ui.add(plot_item1.fill_color(fill_color1));
                        plot_ui.add(plot_item2.fill_color(fill_color2));
                    } else {
                        plot_ui.add(plot_item1.stroke_color(color1));
                        plot_ui.add(plot_item2.stroke_color(color2));
                    }
                }
                SceneState::BooleanResult(result) => {
                    plot_ui.add(plot_item1);
                    plot_ui.add(plot_item2);

                    let all_plines = result
                        .pos_plines
                        .into_iter()
                        .chain(result.neg_plines)
                        .map(|pl| pl.pline);

                    shape = Shape::from_plines(all_plines);

                    let mut plot_item = PlinesPlotItem::new(&shape).stroke_color(color1);
                    if *fill {
                        plot_item = plot_item.fill_color(fill_color1);
                    }

                    plot_ui.add(plot_item);
                }
                SceneState::Intersects {
                    intersects,
                    overlapping_slices,
                } => {
                    intrs = intersects;
                    for (view_data, _) in overlapping_slices.iter() {
                        plines.push(Polyline::create_from(&view_data.view(pline1)));
                    }

                    // Draw original polylines
                    plot_ui.add(plot_item1.stroke_color(color1));
                    plot_ui.add(plot_item2.stroke_color(color2));

                    // Draw intersection points
                    if !intrs.is_empty() {
                        let points = egui_plot::Points::new(PlotPoints::from(intrs.as_slice()))
                            .radius(PLOT_VERTEX_RADIUS * 1.5)
                            .color(colors.error_color);
                        plot_ui.points(points);
                    }

                    // Draw overlapping segments
                    for (i, (_, opposing)) in overlapping_slices.iter().enumerate() {
                        let color = if *opposing {
                            colors.warning_color
                        } else {
                            colors.success_color
                        };
                        plot_ui.add(
                            PlinesPlotItem::new(PlinePlotData::new(&plines[i])).stroke_color(color),
                        );
                    }
                }
                SceneState::Slices {
                    pline1_slices,
                    pline2_slices,
                } => {
                    for slice in pline1_slices.iter() {
                        let slice_view = slice.view(pline1);
                        plines.push(Polyline::create_from(&slice_view));
                    }
                    for slice in pline2_slices.iter() {
                        let slice_view = slice.view(pline2);
                        plines.push(Polyline::create_from(&slice_view));
                    }

                    // Draw original polyline vertexes
                    if *show_vertexes {
                        plot_ui.add(plot_item1);
                        plot_ui.add(plot_item2);
                    }

                    // Draw all slices with different colors
                    for (i, pline) in plines.iter().enumerate() {
                        plot_ui.add(
                            PlinesPlotItem::new(PlinePlotData::new(pline))
                                .stroke_color(colors.get_multi_color(i)),
                        );
                    }
                }
                SceneState::BooleanResultWithPrunedSlices {
                    result,
                    pruned_slices,
                    start_of_pline2_slices,
                    start_of_pline1_overlapping_slices,
                    start_of_pline2_overlapping_slices,
                } => {
                    for slice in pruned_slices.iter() {
                        let slice_view = if slice.source_is_pline1 {
                            slice.view(pline1)
                        } else {
                            slice.view(pline2)
                        };
                        plines.push(Polyline::create_from(&slice_view));
                    }

                    // Draw original polylines faintly
                    plot_ui.add(plot_item1.stroke_color(color1.gamma_multiply(0.3)));
                    plot_ui.add(plot_item2.stroke_color(color2.gamma_multiply(0.3)));

                    // Draw the boolean result
                    let all_plines = result
                        .pos_plines
                        .iter()
                        .chain(result.neg_plines.iter())
                        .map(|pl| &pl.pline);

                    shape = Shape::from_plines(all_plines.cloned());

                    let mut plot_item = PlinesPlotItem::new(&shape).stroke_color(color1);
                    if *fill {
                        plot_item = plot_item.fill_color(fill_color1);
                    }
                    plot_ui.add(plot_item);

                    // Draw pruned slices with categorized colors (overlaid on top)
                    for (i, pline) in plines.iter().enumerate() {
                        let color = if i < start_of_pline2_slices {
                            // Pline1 non-overlapping slices
                            colors.primary_stroke
                        } else if i < start_of_pline1_overlapping_slices {
                            // Pline2 non-overlapping slices
                            colors.secondary_stroke
                        } else if i < start_of_pline2_overlapping_slices {
                            // Pline1 overlapping slices
                            colors.warning_color
                        } else {
                            // Pline2 overlapping slices
                            colors.success_color
                        };

                        plot_ui.add(
                            PlinesPlotItem::new(PlinePlotData::new(pline)).stroke_color(color),
                        );
                    }
                }
            }

            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });

    // Show dual polyline editor window if requested
    polyline_editor.ui_show(ui.ctx(), plines, &settings.colors(ui.ctx()));
}

fn build_scene_state(
    pline1: &Polyline,
    pline2: &Polyline,
    mode: &Mode,
    show_pruned_slices: &bool,
) -> SceneState {
    if pline1.vertex_count() < 2
        || pline2.vertex_count() < 2
        || !pline1.is_closed()
        || !pline2.is_closed()
    {
        return SceneState::NoOp;
    }

    let pos_equal_eps = 1e-5;

    match mode {
        Mode::None => SceneState::NoOp,
        Mode::Or | Mode::And | Mode::Not | Mode::Xor => {
            let op = match mode {
                Mode::Or => BooleanOp::Or,
                Mode::And => BooleanOp::And,
                Mode::Not => BooleanOp::Not,
                Mode::Xor => BooleanOp::Xor,
                _ => unreachable!(),
            };

            if *show_pruned_slices {
                let result = pline1.boolean(pline2, op);
                let pline1_aabb_index = pline1.create_approx_aabb_index();
                let boolean_info =
                    process_for_boolean(pline1, pline2, &pline1_aabb_index, pos_equal_eps);

                let pruned_slices = prune_slices(pline1, pline2, &boolean_info, op, pos_equal_eps);

                SceneState::BooleanResultWithPrunedSlices {
                    result,
                    pruned_slices: pruned_slices.slices_remaining,
                    start_of_pline2_slices: pruned_slices.start_of_pline2_slices,
                    start_of_pline1_overlapping_slices: pruned_slices
                        .start_of_pline1_overlapping_slices,
                    start_of_pline2_overlapping_slices: pruned_slices
                        .start_of_pline2_overlapping_slices,
                }
            } else {
                SceneState::BooleanResult(pline1.boolean(pline2, op))
            }
        }
        Mode::Intersects => {
            let pline1_aabb_index = pline1.create_approx_aabb_index();
            let boolean_info =
                process_for_boolean(pline1, pline2, &pline1_aabb_index, pos_equal_eps);

            let intersects = boolean_info
                .intersects
                .iter()
                .map(|i| PlotPoint::new(i.point.x, i.point.y))
                .collect();

            let overlapping_slices = boolean_info
                .overlapping_slices
                .iter()
                .map(|s| (s.view_data, s.opposing_directions))
                .collect();

            SceneState::Intersects {
                intersects,
                overlapping_slices,
            }
        }
        Mode::Slices => {
            let pline1_aabb_index = pline1.create_approx_aabb_index();
            let boolean_info =
                process_for_boolean(pline1, pline2, &pline1_aabb_index, pos_equal_eps);

            let mut pline1_slices = Vec::new();
            let mut pline2_slices = Vec::new();

            // Always accept all slices to show them all
            slice_at_intersects(
                pline1,
                &boolean_info,
                false,
                &mut |_| true,
                &mut pline1_slices,
                pos_equal_eps,
            );

            slice_at_intersects(
                pline2,
                &boolean_info,
                true,
                &mut |_| true,
                &mut pline2_slices,
                pos_equal_eps,
            );

            SceneState::Slices {
                pline1_slices,
                pline2_slices,
            }
        }
    }
}
