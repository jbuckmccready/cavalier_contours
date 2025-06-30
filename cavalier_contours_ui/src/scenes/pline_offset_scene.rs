use cavalier_contours::{
    pline_closed,
    polyline::{
        PlineOffsetOptions, PlineSource, PlineSourceMut, PlineVertex, Polyline,
        internal::pline_offset::{
            RawPlineOffsetSeg, create_raw_offset_polyline, create_untrimmed_raw_offset_segs,
        },
    },
};
use eframe::egui::{CentralPanel, Rect, ScrollArea, Slider, TextEdit, Ui, Vec2, Window};
use egui::Id;
use egui_plot::{Plot, PlotPoint};
use std::hash::{Hash, Hasher};
use std::{borrow::Cow, collections::hash_map::DefaultHasher};

use crate::plotting::{PlinePlotData, PlinesPlotItem, RawPlineOffsetSegsPlotItem};

use super::{
    super::plotting::PLOT_VERTEX_RADIUS, Scene, controls_side_panel, scene_settings::SceneSettings,
};

pub struct PlineOffsetScene {
    pline: Polyline,
    mode: Mode,
    offset: f64,
    interaction_state: InteractionState,
    json_editor: JsonEditor,
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Offset {
        handle_self_intersects: bool,
        max_offset_count: usize,
    },
    RawOffset,
    RawOffsetSegments,
}

impl Mode {
    fn label(&self) -> &'static str {
        match self {
            Mode::Offset { .. } => "Offset",
            Mode::RawOffset => "Raw Offset",
            Mode::RawOffsetSegments => "Raw Offset Segments",
        }
    }

    fn offset_default() -> Self {
        Mode::Offset {
            handle_self_intersects: true,
            max_offset_count: 10,
        }
    }
}

struct InteractionState {
    grabbed_vertex: Option<usize>,
    dragging: bool,
    zoom_to_fit: bool,
}

struct JsonEditor {
    show_window: bool,
    vertex_data: Vec<PlineVertex>,
    json_text: String,
    applied_json_text: String,
    json_error: Option<String>,
    last_pline_hash: u64,
    applied_vertex_data: Vec<PlineVertex>,
    active_tab: EditorTab,
}

#[derive(Clone, Copy, PartialEq)]
enum EditorTab {
    Table,
    Json,
}

enum SceneState {
    Offset {
        all_offset_plines: Vec<(Polyline, bool)>,
    },
    RawOffset {
        raw_offset_pline: Polyline,
    },
    RawOffsetSegments {
        segments: Vec<RawPlineOffsetSeg<f64>>,
    },
}

impl Default for PlineOffsetScene {
    fn default() -> Self {
        let pline = pline_closed![
            (10.0, 10.0, -0.5),
            (8.0, 9.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (39.0, 21.0, 0.0),
            (28.0, 12.0, 0.5),
        ];

        let json_text = serde_json::to_string_pretty(&pline).unwrap_or_default();
        let pline_hash = hash_polyline(&pline);
        let vertex_data: Vec<PlineVertex> = pline.iter_vertexes().collect();

        Self {
            pline,
            mode: Mode::Offset {
                handle_self_intersects: true,
                max_offset_count: 10,
            },
            offset: 1.0,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
            json_editor: JsonEditor {
                show_window: false,
                vertex_data: vertex_data.clone(),
                json_text: json_text.clone(),
                applied_json_text: json_text,
                json_error: None,
                last_pline_hash: pline_hash,
                applied_vertex_data: vertex_data,
                active_tab: EditorTab::Table,
            },
        }
    }
}

impl Scene for PlineOffsetScene {
    fn name(&self) -> &'static str {
        "Polyline Offset"
    }

    fn ui(&mut self, ui: &mut Ui, settings: &SceneSettings, init: bool) {
        let PlineOffsetScene {
            pline,
            mode,
            offset,
            interaction_state,
            json_editor,
        } = self;

        controls_panel(ui, mode, offset, interaction_state, json_editor);

        interaction_state.zoom_to_fit |= init;
        plot_area(
            ui,
            settings,
            pline,
            mode,
            offset,
            interaction_state,
            json_editor,
        );
    }
}

fn controls_panel(
    ui: &mut Ui,
    mode: &mut Mode,
    offset: &mut f64,
    interaction_state: &mut InteractionState,
    json_editor: &mut JsonEditor,
) {
    controls_side_panel("pline_offset_controls")
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                ui.horizontal(|ui| {
                    ui.label("Mode:");
                    egui::ComboBox::from_id_salt("mode_combo")
                        .selected_text(mode.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(mode, Mode::offset_default(), Mode::offset_default().label()).on_hover_text("Generate parallel offsets");
                            ui.selectable_value(mode, Mode::RawOffset, Mode::RawOffset.label()).on_hover_text("Generate single raw offset polyline");
                            ui.selectable_value(mode, Mode::RawOffsetSegments, Mode::RawOffsetSegments.label()).on_hover_text("Generate the raw offset polyline segments");
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
                        ui.add(Slider::new(offset, -100.0..=100.0).step_by(0.5));
                    });

                if let Mode::Offset {
                    handle_self_intersects,
                    max_offset_count,
                } = mode
                {
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

                    ui.add_space(ui.spacing().item_spacing.y);
                    ui.checkbox(handle_self_intersects, "Handle Self Intersects").on_hover_text("Handle self-intersecting polylines or not (small performance hit)");
                }

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();

                // Table editor button
                if ui.button("Edit Vertexes").on_hover_text("Edit the polyline vertex data").clicked() {
                    json_editor.show_window = true;
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
    pline: &mut Polyline,
    mode: &Mode,
    offset: &f64,
    interaction_state: &mut InteractionState,
    json_editor: &mut JsonEditor,
) {
    let colors = settings.colors(ui.ctx());
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    // TODO: cache scene state to only update when necessary due to modified polyline or offset
    let scene_state = match mode {
        Mode::Offset {
            handle_self_intersects,
            max_offset_count,
        } => build_offset(pline, offset, handle_self_intersects, max_offset_count),
        Mode::RawOffset => build_raw_offset(pline, offset),
        Mode::RawOffsetSegments => build_raw_offset_segments(pline, offset),
    };

    CentralPanel::default().show_inside(ui, |ui| {
        let plot = settings
            .apply_to_plot(Plot::new("pline_offset_scene"))
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

            if let Some(grabbed) = grabbed_vertex {
                // move grabbed point by drag delta by offsetting point position
                let delta = plot_ui.pointer_coordinate_drag_delta();
                let grabbed_vertex = pline.get(*grabbed).unwrap();
                pline.set(
                    *grabbed,
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
                    for (i, pt) in pline
                        .iter_vertexes()
                        .map(|v| plot_ui.screen_from_plot(PlotPoint::new(v.x, v.y)))
                        .enumerate()
                    {
                        let hit_size =
                            2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);

                        let hit_box = Rect::from_center_size(pt, Vec2::splat(hit_size));

                        if hit_box.contains(coord) {
                            // update grabbed point
                            *grabbed_vertex = Some(i);
                            break;
                        }
                    }

                    *dragging = grabbed_vertex.is_none();
                }
            }

            plot_ui.add(
                PlinesPlotItem::new(PlinePlotData::new(pline))
                    .stroke_color(colors.accent_stroke)
                    .vertex_color(colors.vertex_color),
            );

            // TODO: color pickers
            match &scene_state {
                SceneState::Offset { all_offset_plines } => {
                    for (pl, same_orientation) in all_offset_plines.iter() {
                        let color = if *same_orientation {
                            colors.primary_stroke
                        } else {
                            colors.secondary_stroke
                        };

                        plot_ui
                            .add(PlinesPlotItem::new(PlinePlotData::new(pl)).stroke_color(color));
                    }
                }
                SceneState::RawOffset { raw_offset_pline } => {
                    plot_ui.add(
                        PlinesPlotItem::new(PlinePlotData::new(raw_offset_pline))
                            .stroke_color(colors.primary_stroke),
                    );
                }
                SceneState::RawOffsetSegments { segments } => {
                    plot_ui.add(
                        RawPlineOffsetSegsPlotItem::new(&segments[..])
                            .color(colors.raw_offset_color)
                            .collapsed_color(colors.collapsed_color),
                    );
                }
            };

            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });

    // Show table editor window if requested
    show_table_editor_window(ui.ctx(), pline, json_editor, &colors);
}

fn build_offset(
    pline: &Polyline,
    offset: &f64,
    handle_self_intersects: &bool,
    max_offset_count: &usize,
) -> SceneState {
    let mut all_offset_plines = Vec::new();
    let offset_opt = PlineOffsetOptions {
        handle_self_intersects: *handle_self_intersects,
        ..Default::default()
    };

    // remove redundant vertices if necessary to avoid any problems with offsetting (sanitizing
    // input)
    let mut pline = Cow::Borrowed(pline);
    if let Some(pl) = pline.remove_redundant(offset_opt.pos_equal_eps) {
        pline = Cow::Owned(pl);
    }

    let orientation = pline.orientation();

    // current offset polylines
    let mut offset_plines = pline.parallel_offset_opt(*offset, &offset_opt);

    let mut same_orientation = Vec::new();
    let mut diff_orientation = Vec::new();

    // repeat offsets until max or collapsed entirely
    for _ in 1..*max_offset_count {
        // split offset plines by orientation
        for pl in offset_plines.drain(..) {
            if pl.orientation() == orientation {
                same_orientation.push(pl);
            } else {
                diff_orientation.push(pl);
            }
        }

        // repeat offset for same orientation ones
        for pl in same_orientation.iter() {
            offset_plines.extend(pl.parallel_offset_opt(*offset, &offset_opt));
        }

        // accumulate results
        all_offset_plines.extend(same_orientation.drain(..).zip(std::iter::repeat(true)));
        all_offset_plines.extend(diff_orientation.drain(..).zip(std::iter::repeat(false)));
    }

    // add last results
    for pl in offset_plines.drain(..) {
        if pl.orientation() == orientation {
            all_offset_plines.push((pl, true));
        } else {
            all_offset_plines.push((pl, false));
        }
    }
    SceneState::Offset { all_offset_plines }
}

fn build_raw_offset(pline: &Polyline, offset: &f64) -> SceneState {
    let offset_opt = PlineOffsetOptions::default();

    let raw_offset_pline: Polyline =
        create_raw_offset_polyline(pline, *offset, offset_opt.pos_equal_eps);

    SceneState::RawOffset { raw_offset_pline }
}

fn build_raw_offset_segments(pline: &Polyline, offset: &f64) -> SceneState {
    let raw_offset_segs: Vec<RawPlineOffsetSeg<f64>> =
        create_untrimmed_raw_offset_segs(pline, *offset);

    SceneState::RawOffsetSegments {
        segments: raw_offset_segs,
    }
}

fn hash_polyline(pline: &Polyline) -> u64 {
    let mut hasher = DefaultHasher::new();
    for vertex in pline.iter_vertexes() {
        vertex.x.to_bits().hash(&mut hasher);
        vertex.y.to_bits().hash(&mut hasher);
        vertex.bulge.to_bits().hash(&mut hasher);
    }
    pline.is_closed().hash(&mut hasher);
    hasher.finish()
}

fn update_editor_from_polyline(pline: &Polyline, json_editor: &mut JsonEditor) {
    let new_vertex_data: Vec<PlineVertex> = pline.iter_vertexes().collect();
    json_editor.vertex_data = new_vertex_data.clone();
    json_editor.applied_vertex_data = new_vertex_data;

    // Update JSON text
    if let Ok(json) = serde_json::to_string_pretty(pline) {
        json_editor.json_text = json.clone();
        json_editor.applied_json_text = json;
    }

    json_editor.last_pline_hash = hash_polyline(pline);
    json_editor.json_error = None;
}

fn show_table_editor_window(
    ctx: &egui::Context,
    pline: &mut Polyline,
    json_editor: &mut JsonEditor,
    colors: &crate::theme::ThemeColors,
) {
    let mut open = json_editor.show_window;

    // Auto-refresh table and JSON data when polyline changes
    let current_hash = hash_polyline(pline);
    if current_hash != json_editor.last_pline_hash {
        update_editor_from_polyline(pline, json_editor);
    }

    Window::new("Vertex Editor")
        .open(&mut open)
        .default_width(800.0)
        .default_height(500.0)
        .resizable(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(
                    &mut json_editor.active_tab,
                    EditorTab::Table,
                    "Table Editor",
                );
                ui.selectable_value(
                    &mut json_editor.active_tab,
                    EditorTab::Json,
                    "JSON Import/Export",
                );
            });

            ui.separator();

            // Show error if any
            if let Some(error) = &json_editor.json_error {
                ui.colored_label(colors.error_color, format!("Error: {error}"));
            }

            match json_editor.active_tab {
                EditorTab::Table => {
                    show_table_editor_tab(ui, pline, json_editor, colors);
                }
                EditorTab::Json => {
                    show_json_editor_tab(ui, pline, json_editor, colors);
                }
            }
        });

    json_editor.show_window = open;
}

fn show_table_editor_tab(
    ui: &mut Ui,
    pline: &mut Polyline,
    json_editor: &mut JsonEditor,
    colors: &crate::theme::ThemeColors,
) {
    ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
        ui.label("Edit polyline vertices:");

        ui.horizontal(|ui| {
            let has_pending_changes = json_editor.vertex_data != json_editor.applied_vertex_data;

            // Apply table changes button
            let apply_button = ui
                .button("Apply Changes")
                .on_hover_text("Update polyline from table data");

            if apply_button.clicked() {
                apply_table_changes_to_polyline(pline, json_editor);
            }

            ui.separator();

            // Add vertex button
            if ui.button("Add Vertex").clicked() {
                json_editor
                    .vertex_data
                    .push(PlineVertex::new(0.0, 0.0, 0.0));
            }

            // Show pending changes indicator and cancel button
            show_pending_changes_ui(
                ui,
                has_pending_changes,
                || {
                    json_editor.vertex_data = json_editor.applied_vertex_data.clone();
                },
                colors,
            );
        });

        ui.separator();

        // Vertex table
        ScrollArea::both().show(ui, |ui| {
            show_vertex_table(ui, json_editor);
        });
    });
}

fn show_json_editor_tab(
    ui: &mut Ui,
    pline: &mut Polyline,
    json_editor: &mut JsonEditor,
    colors: &crate::theme::ThemeColors,
) {
    ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
        ui.label("JSON Editor:");

        ui.horizontal(|ui| {
            let has_pending_changes = json_editor.json_text != json_editor.applied_json_text;

            // Apply JSON changes button
            let apply_button = ui
                .button("Apply")
                .on_hover_text("Apply JSON changes to polyline");

            if apply_button.clicked() {
                apply_json_changes_to_polyline(pline, json_editor);
            }

            // Show pending changes indicator and cancel button
            show_pending_changes_ui(
                ui,
                has_pending_changes,
                || {
                    json_editor.json_text = json_editor.applied_json_text.clone();
                },
                colors,
            );
        });

        ui.separator();

        ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
            let text_edit = TextEdit::multiline(&mut json_editor.json_text)
                .desired_rows(20)
                .font(egui::TextStyle::Monospace);
            ui.add_sized(ui.available_size(), text_edit);
        });
    });
}

fn show_vertex_table(ui: &mut Ui, json_editor: &mut JsonEditor) {
    use egui_extras::{Column, TableBuilder};

    TableBuilder::new(ui)
        .column(Column::auto().at_least(60.0)) // Index
        .column(Column::remainder().at_least(100.0)) // X
        .column(Column::remainder().at_least(100.0)) // Y
        .column(Column::remainder().at_least(100.0)) // Bulge
        .column(Column::auto().at_least(60.0)) // Delete
        .header(20.0, |mut header| {
            header.col(|ui| {
                ui.label("Index");
            });
            header.col(|ui| {
                ui.label("X");
            });
            header.col(|ui| {
                ui.label("Y");
            });
            header.col(|ui| {
                ui.label("Bulge");
            });
            header.col(|ui| {
                ui.label("Delete");
            });
        })
        .body(|mut body| {
            let mut to_delete = None;

            for (i, vertex) in json_editor.vertex_data.iter_mut().enumerate() {
                body.row(20.0, |mut row| {
                    row.col(|ui| {
                        ui.label(i.to_string());
                    });
                    row.col(|ui| {
                        ui.add(egui::DragValue::new(&mut vertex.x).speed(0.1));
                    });
                    row.col(|ui| {
                        ui.add(egui::DragValue::new(&mut vertex.y).speed(0.1));
                    });
                    row.col(|ui| {
                        ui.add(egui::DragValue::new(&mut vertex.bulge).speed(0.01));
                    });
                    row.col(|ui| {
                        if ui.button("🗑").on_hover_text("Delete vertex").clicked() {
                            to_delete = Some(i);
                        }
                    });
                });
            }

            if let Some(index) = to_delete {
                json_editor.vertex_data.remove(index);
            }
        });
}

fn apply_table_changes_to_polyline(pline: &mut Polyline, json_editor: &mut JsonEditor) {
    // Clear the existing polyline
    pline.clear();

    // Add vertices from table data
    for vertex in &json_editor.vertex_data {
        pline.add(vertex.x, vertex.y, vertex.bulge);
    }

    // Update all editor state from the modified polyline
    update_editor_from_polyline(pline, json_editor);
}

fn apply_json_changes_to_polyline(pline: &mut Polyline, json_editor: &mut JsonEditor) {
    match serde_json::from_str::<Polyline>(&json_editor.json_text) {
        Ok(new_pline) => {
            *pline = new_pline;

            // Update applied JSON text and all editor state
            json_editor.applied_json_text = json_editor.json_text.clone();
            update_editor_from_polyline(pline, json_editor);
        }
        Err(e) => {
            json_editor.json_error = Some(format!("Failed to parse JSON: {e}"));
        }
    }
}

fn show_pending_changes_ui<F>(
    ui: &mut Ui,
    has_pending_changes: bool,
    cancel_action: F,
    colors: &crate::theme::ThemeColors,
) where
    F: FnOnce(),
{
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        if has_pending_changes {
            // Cancel button
            if ui
                .button("Cancel")
                .on_hover_text("Discard changes and revert to applied state")
                .clicked()
            {
                cancel_action();
            }

            ui.separator();

            // Pending changes indicator
            ui.colored_label(colors.warning_color, "⚠ Changes pending");
        }
    });
}
