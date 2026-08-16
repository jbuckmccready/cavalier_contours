use cavalier_contours::{
    pline_closed,
    polyline::{
        CoincidentSegmentBehavior, PlineOffsetOptions, PlineSource, PlineSourceMut, Polyline,
        TouchingLoopBehavior,
        internal::raw_pline_offset::{
            RawOffsetSeg, create_raw_offset, create_untrimmed_raw_offset_segs,
        },
    },
};
use egui::{CentralPanel, DragValue, Rect, ScrollArea, Slider, Ui, Vec2};
use egui_plot::{Plot, PlotPoint};
use std::borrow::Cow;

use crate::editor::PolylineEditor;
use crate::plotting::{PlinePlotData, PlinesPlotItem, RawOffsetSegsPlotItem};
use crate::theme::ThemeColors;

use super::{
    super::plotting::PLOT_VERTEX_RADIUS, Scene, controls_side_panel, scene_settings::SceneSettings,
};

pub struct PlineOffsetScene {
    // NOTE: just one polyline but Vec used for passing into editor
    pline: Vec<Polyline>,
    mode: Mode,
    offset: f64,
    interaction_state: InteractionState,
    polyline_editor: PolylineEditor,
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Offset {
        handle_self_intersects: bool,
        max_offset_count: usize,
        touching_loop_behavior: TouchingLoopBehavior,
        coincident_segment_behavior: CoincidentSegmentBehavior,
    },
    RawOffset {
        show_invalid_segments: bool,
    },
    RawOffsetSegments,
}

impl Mode {
    fn label(&self) -> &'static str {
        match self {
            Mode::Offset { .. } => "Offset",
            Mode::RawOffset { .. } => "Raw Offset",
            Mode::RawOffsetSegments => "Raw Offset Segments",
        }
    }

    fn offset_default() -> Self {
        Mode::Offset {
            handle_self_intersects: true,
            max_offset_count: 10,
            touching_loop_behavior: TouchingLoopBehavior::Preserve,
            coincident_segment_behavior: CoincidentSegmentBehavior::Preserve,
        }
    }

    fn raw_offset_default() -> Self {
        Mode::RawOffset {
            show_invalid_segments: false,
        }
    }
}

struct InteractionState {
    grabbed_vertex: Option<usize>,
    dragging: bool,
    zoom_to_fit: bool,
}

enum SceneState {
    Offset {
        all_offset_plines: Vec<(Polyline, bool)>,
        first_offset_count: usize,
    },
    RawOffset {
        raw_offset_pline: Polyline,
        invalid_segments: Vec<Polyline>,
    },
    RawOffsetSegments {
        segments: Vec<RawOffsetSeg<f64>>,
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

        let pline = vec![pline];
        let mut polyline_editor = PolylineEditor::single("Vertex Editor");
        polyline_editor.initialize_with_polylines(pline.clone());

        Self {
            pline,
            mode: Mode::Offset {
                handle_self_intersects: true,
                max_offset_count: 10,
                touching_loop_behavior: TouchingLoopBehavior::Preserve,
                coincident_segment_behavior: CoincidentSegmentBehavior::Preserve,
            },
            offset: 1.0,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
            polyline_editor,
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
            polyline_editor,
        } = self;

        let scene_state = controls_panel(
            ui,
            pline
                .first()
                .expect("PlineOffsetScene should always have at least one polyline"),
            mode,
            offset,
            interaction_state,
            polyline_editor,
        );

        interaction_state.zoom_to_fit |= init;
        plot_area(
            ui,
            settings,
            pline,
            &scene_state,
            interaction_state,
            polyline_editor,
        );
    }
}

fn controls_panel(
    ui: &mut Ui,
    pline: &Polyline,
    mode: &mut Mode,
    offset: &mut f64,
    interaction_state: &mut InteractionState,
    polyline_editor: &mut PolylineEditor,
) -> SceneState {
    let mut scene_state = None;

    controls_side_panel("pline_offset_controls")
        .show(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                ui.horizontal(|ui| {
                    ui.label("Mode:");
                    egui::ComboBox::from_id_salt("mode_combo")
                        .selected_text(mode.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(mode, Mode::offset_default(), Mode::offset_default().label()).on_hover_text("Generate parallel offsets");
                            ui.selectable_value(mode, Mode::raw_offset_default(), Mode::raw_offset_default().label()).on_hover_text("Generate single raw offset polyline");
                            ui.selectable_value(mode, Mode::RawOffsetSegments, Mode::RawOffsetSegments.label()).on_hover_text("Generate the raw offset polyline segments");
                        });
                });

                egui::Frame::default()
                    .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                    .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                    .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                    .show(ui, |ui| {
                        ui.label("Offset").on_hover_text("Parallel offset distance, positive value will offset to the left of curve direction");
                        ui.style_mut().spacing.slider_width = ui.available_width().max(0.0);
                        ui.add(Slider::new(offset, -100.0..=100.0).show_value(false));
                        ui.label("Exact value:");
                        ui.add_sized(
                            [ui.available_width(), ui.spacing().interact_size.y],
                            DragValue::new(offset)
                                .range(-100.0..=100.0)
                                .speed(0.01)
                                .custom_formatter(|value, _| value.to_string()),
                        )
                        .on_hover_text("Exact parallel offset distance");
                    });

                if let Mode::RawOffset {
                    show_invalid_segments,
                } = mode
                {
                    ui.checkbox(show_invalid_segments, "Show Invalid Segments")
                        .on_hover_text(
                            "Highlight all locally invalid raw offset segments",
                        );
                }

                if let Mode::Offset {
                    handle_self_intersects,
                    max_offset_count,
                    touching_loop_behavior,
                    coincident_segment_behavior,
                } = mode
                {
                    egui::Frame::default()
                        .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                        .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                        .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                        .show(ui, |ui| {
                            ui.label("Max Offset Count").on_hover_text("Maximum number of parallel offsets to generate (stops early when orientation changes)");
                            let value_width =
                                ui.spacing().interact_size.x + ui.spacing().item_spacing.x;
                            ui.style_mut().spacing.slider_width =
                                (ui.available_width() - value_width).max(0.0);
                            ui.add(
                                Slider::new(max_offset_count, 0..=100)
                                    .integer()
                                    .step_by(1.0),
                            );
                        });

                    ui.add_space(ui.spacing().item_spacing.y);
                    ui.checkbox(handle_self_intersects, "Handle Self Intersects").on_hover_text("Handle self-intersecting polylines or not (small performance hit)");

                    egui::Frame::default()
                        .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                        .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                        .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                        .show(ui, |ui| {
                        ui.label("Touching Loops");
                        egui::ComboBox::from_id_salt("touching_loop_behavior")
                            .width(ui.available_width())
                            .selected_text(match touching_loop_behavior {
                                TouchingLoopBehavior::Preserve => "Preserve",
                                TouchingLoopBehavior::Separate => "Separate",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    touching_loop_behavior,
                                    TouchingLoopBehavior::Preserve,
                                    "Preserve",
                                );
                                ui.selectable_value(
                                    touching_loop_behavior,
                                    TouchingLoopBehavior::Separate,
                                    "Separate",
                                );
                            });
                    });

                    egui::Frame::default()
                        .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                        .corner_radius(ui.visuals().widgets.noninteractive.corner_radius)
                        .inner_margin(Vec2::splat(ui.spacing().item_spacing.x))
                        .show(ui, |ui| {
                        ui.label("Coincident Segments");
                        egui::ComboBox::from_id_salt("coincident_segment_behavior")
                            .width(ui.available_width())
                            .selected_text(match coincident_segment_behavior {
                                CoincidentSegmentBehavior::Preserve => "Preserve",
                                CoincidentSegmentBehavior::Discard => "Discard",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    coincident_segment_behavior,
                                    CoincidentSegmentBehavior::Preserve,
                                    "Preserve",
                                );
                                ui.selectable_value(
                                    coincident_segment_behavior,
                                    CoincidentSegmentBehavior::Discard,
                                    "Discard",
                                );
                            });
                    });
                }

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();

                // Table editor button
                if ui.button("Edit Vertexes").on_hover_text("Edit the polyline vertex data").clicked() {
                    polyline_editor.show_window();
                }

                // TODO: cache scene state to only update when the polyline or controls change.
                let current_scene_state = build_scene_state(pline, mode, *offset);
                if let SceneState::Offset {
                    first_offset_count,
                    ..
                } = &current_scene_state
                {
                    ui.separator();
                    ui.label("Offset Results");
                    ui.label(format!(
                        "First offset polyline count: {first_offset_count}"
                    ));
                }
                scene_state = Some(current_scene_state);
            })
        });

    scene_state.expect("controls panel contents should always be rendered")
}

fn build_scene_state(pline: &Polyline, mode: &Mode, offset: f64) -> SceneState {
    match mode {
        Mode::Offset {
            handle_self_intersects,
            max_offset_count,
            touching_loop_behavior,
            coincident_segment_behavior,
        } => build_offset(
            pline,
            offset,
            *handle_self_intersects,
            *max_offset_count,
            *touching_loop_behavior,
            *coincident_segment_behavior,
        ),
        Mode::RawOffset {
            show_invalid_segments,
        } => build_raw_offset(pline, offset, *show_invalid_segments),
        Mode::RawOffsetSegments => build_raw_offset_segments(pline, offset),
    }
}

fn plot_area(
    ui: &mut Ui,
    settings: &SceneSettings,
    plines: &mut Vec<Polyline>,
    scene_state: &SceneState,
    interaction_state: &mut InteractionState,
    polyline_editor: &mut PolylineEditor,
) {
    let colors = ThemeColors::from_context(ui.ctx());
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    let pline = plines
        .get_mut(0)
        .expect("PlineOffsetScene should always have at least one polyline");

    CentralPanel::default().show(ui, |ui| {
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
                    grabbed_vertex.x + f64::from(delta.x),
                    grabbed_vertex.y + f64::from(delta.y),
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
                        let hit_size = 2.0
                            * (plot_ui.ctx().input(egui::InputState::aim_radius)
                                + PLOT_VERTEX_RADIUS);

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
            match scene_state {
                SceneState::Offset {
                    all_offset_plines, ..
                } => {
                    for (pl, same_orientation) in all_offset_plines {
                        let color = if *same_orientation {
                            colors.primary_stroke
                        } else {
                            colors.secondary_stroke
                        };

                        plot_ui
                            .add(PlinesPlotItem::new(PlinePlotData::new(pl)).stroke_color(color));
                    }
                }
                SceneState::RawOffset {
                    raw_offset_pline,
                    invalid_segments,
                } => {
                    plot_ui.add(
                        PlinesPlotItem::new(PlinePlotData::new(raw_offset_pline))
                            .stroke_color(colors.primary_stroke),
                    );

                    for segment in invalid_segments {
                        plot_ui.add(
                            PlinesPlotItem::new(PlinePlotData::new(segment))
                                .stroke_color(colors.error_color),
                        );
                    }
                }
                SceneState::RawOffsetSegments { segments } => {
                    plot_ui.add(
                        RawOffsetSegsPlotItem::new(&segments[..])
                            .color(colors.raw_offset_color)
                            .collapsed_color(colors.collapsed_color),
                    );
                }
            }

            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });

    // Show table editor window if requested
    polyline_editor.ui_show(ui.ctx(), plines, &colors);
}

fn build_offset(
    pline: &Polyline,
    offset: f64,
    handle_self_intersects: bool,
    max_offset_count: usize,
    touching_loop_behavior: TouchingLoopBehavior,
    coincident_segment_behavior: CoincidentSegmentBehavior,
) -> SceneState {
    let mut all_offset_plines = Vec::new();
    if max_offset_count == 0 {
        return SceneState::Offset {
            all_offset_plines,
            first_offset_count: 0,
        };
    }

    let offset_opt = PlineOffsetOptions {
        handle_self_intersects,
        touching_loop_behavior,
        coincident_segment_behavior,
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
    let mut offset_plines = pline.parallel_offset_opt(offset, &offset_opt);
    let first_offset_count = offset_plines.len();

    let mut same_orientation = Vec::new();
    let mut diff_orientation = Vec::new();

    // repeat offsets until max or collapsed entirely
    for _ in 1..max_offset_count {
        // split offset plines by orientation
        for pl in offset_plines.drain(..) {
            if pl.orientation() == orientation {
                same_orientation.push(pl);
            } else {
                diff_orientation.push(pl);
            }
        }

        // repeat offset for same orientation ones
        for pl in &same_orientation {
            offset_plines.extend(pl.parallel_offset_opt(offset, &offset_opt));
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
    SceneState::Offset {
        all_offset_plines,
        first_offset_count,
    }
}

fn build_raw_offset(pline: &Polyline, offset: f64, show_invalid_segments: bool) -> SceneState {
    let offset_opt = PlineOffsetOptions::default();

    let raw_offset = create_raw_offset(pline, offset, offset_opt.pos_equal_eps);
    let invalid_segments = if show_invalid_segments {
        invalid_raw_offset_segments(&raw_offset.polyline, &raw_offset.invalid_segments)
    } else {
        Vec::new()
    };

    SceneState::RawOffset {
        raw_offset_pline: raw_offset.polyline,
        invalid_segments,
    }
}

fn invalid_raw_offset_segments(
    raw_offset_pline: &Polyline,
    invalid_segments: &[bool],
) -> Vec<Polyline> {
    invalid_segments
        .iter()
        .enumerate()
        .filter(|&(_, &invalid)| invalid)
        .map(|(index, _)| {
            let start = raw_offset_pline.at(index);
            let end = raw_offset_pline.at(raw_offset_pline.next_wrapping_index(index));
            let mut segment = Polyline::new();
            segment.add_vertex(start);
            segment.add(end.x, end.y, 0.0);
            segment
        })
        .collect()
}

fn build_raw_offset_segments(pline: &Polyline, offset: f64) -> SceneState {
    let raw_offset_segs: Vec<RawOffsetSeg<f64>> = create_untrimmed_raw_offset_segs(pline, offset);

    SceneState::RawOffsetSegments {
        segments: raw_offset_segs,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn partially_coincident_open_arcs() -> Polyline {
        let mut result = Polyline::new();
        for (x, y, bulge) in [
            (-5.0, 0.0, -1.0),
            (5.0, 0.0, 0.5773502691896257),
            (-2.5, 4.330127018922193, -1.0),
            (2.5, -4.330127018922193, 0.0),
        ] {
            result.add(x, y, bulge);
        }
        result
    }

    #[test]
    fn zero_max_offset_count_builds_no_offsets() {
        let result = build_offset(
            &partially_coincident_open_arcs(),
            1.0,
            true,
            0,
            TouchingLoopBehavior::Preserve,
            CoincidentSegmentBehavior::Preserve,
        );

        let SceneState::Offset {
            all_offset_plines,
            first_offset_count,
        } = result
        else {
            panic!("expected offset scene state");
        };
        assert!(all_offset_plines.is_empty());
        assert_eq!(first_offset_count, 0);
    }

    #[test]
    fn first_offset_count_matches_initial_offset_results() {
        let result = build_offset(
            &partially_coincident_open_arcs(),
            1.0,
            true,
            1,
            TouchingLoopBehavior::Preserve,
            CoincidentSegmentBehavior::Preserve,
        );

        let SceneState::Offset {
            all_offset_plines,
            first_offset_count,
        } = result
        else {
            panic!("expected offset scene state");
        };
        assert_eq!(all_offset_plines.len(), 2);
        assert_eq!(first_offset_count, 2);
    }
}
