use cavalier_contours::{
    pline_closed,
    polyline::{
        PlineOffsetOptions, PlineSource, PlineSourceMut, Polyline,
        internal::pline_offset::{
            RawPlineOffsetSeg, create_raw_offset_polyline, create_untrimmed_raw_offset_segs,
        },
    },
    static_aabb2d_index::AABB,
};
use eframe::egui::{CentralPanel, Color32, Rect, ScrollArea, SidePanel, Slider, Ui, Vec2};
use egui::Id;
use egui_plot::{Plot, PlotBounds, PlotPoint};

use crate::plotting::RawPlineOffsetSegsPlotItem;

use super::{
    super::plotting::{PLOT_VERTEX_RADIUS, PlinePlotItem},
    Scene, total_bounds,
};

pub struct PlineOffsetScene {
    pline: Polyline,
    mode: Mode,
    offset: f64,
    interaction_state: InteractionState,
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
            Mode::RawOffset { .. } => "Raw Offset",
            Mode::RawOffsetSegments { .. } => "Raw Offset Segments",
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
        //let pline = serde_json::from_str(DEER_PATH_STR).unwrap();

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
        }
    }
}

impl Scene for PlineOffsetScene {
    fn name(&self) -> &'static str {
        "Polyline Offset"
    }

    fn ui(&mut self, ui: &mut Ui, init: bool) {
        self.ui(ui, init)
    }
}

impl PlineOffsetScene {
    pub fn ui(&mut self, ui: &mut Ui, init: bool) {
        let PlineOffsetScene {
            pline,
            mode,
            offset,
            interaction_state,
        } = self;

        controls_panel(ui, mode, offset, interaction_state);

        interaction_state.zoom_to_fit |= init;
        plot_area(ui, pline, mode, offset, interaction_state);
    }
}

fn controls_panel(
    ui: &mut Ui,
    mode: &mut Mode,
    offset: &mut f64,
    interaction_state: &mut InteractionState,
) {
    SidePanel::right("pline_offset_panel")
        .min_width(150.0)
        .default_width(150.0)
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                egui::ComboBox::from_label("Mode")
                    .selected_text(mode.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(mode, Mode::offset_default(), "Offset");
                        ui.selectable_value(mode, Mode::RawOffset, "Raw Offset");
                        ui.selectable_value(mode, Mode::RawOffsetSegments, "Raw Offset Segments");
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
                        ui.label("Offset");
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
                            ui.label("Max Offset Count");
                            ui.add(
                                Slider::new(max_offset_count, 0..=100)
                                    .integer()
                                    .step_by(1.0),
                            );
                        });

                    ui.add_space(ui.spacing().item_spacing.y);
                    ui.checkbox(handle_self_intersects, "Handle Self Intersects");
                }

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
    pline: &mut Polyline,
    mode: &Mode,
    offset: &f64,
    interaction_state: &mut InteractionState,
) {
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
                PlinePlotItem::new(pline)
                    .stroke_color(Color32::GOLD)
                    //.fill_color(Color32::LIGHT_YELLOW)
                    .vertex_color(Color32::LIGHT_GREEN),
            );

            if *offset == 0.0 {
                return;
            }

            // TODO: color pickers
            let zoom_to_bounds = match &scene_state {
                SceneState::Offset { all_offset_plines } => {
                    for (pl, same_orientation) in all_offset_plines.iter() {
                        let color = if *same_orientation {
                            Color32::LIGHT_BLUE
                        } else {
                            Color32::LIGHT_RED
                        };

                        plot_ui.add(PlinePlotItem::new(pl).stroke_color(color));
                    }

                    zoom_to_fit
                        .then(|| {
                            let all_extents = all_offset_plines
                                .iter()
                                .map(|(pl, _)| pl.extents())
                                .chain(std::iter::once(pline.extents()));
                            total_bounds(all_extents.flatten())
                        })
                        .flatten()
                }
                SceneState::RawOffset { raw_offset_pline } => {
                    plot_ui.add(
                        PlinePlotItem::new(raw_offset_pline).stroke_color(Color32::LIGHT_GRAY),
                    );

                    zoom_to_fit
                        .then(|| {
                            total_bounds(
                                [pline.extents(), raw_offset_pline.extents()]
                                    .into_iter()
                                    .flatten(),
                            )
                        })
                        .flatten()
                }
                SceneState::RawOffsetSegments { segments } => {
                    plot_ui.add(
                        RawPlineOffsetSegsPlotItem::new(&segments[..])
                            .color(Color32::MAGENTA)
                            .collapsed_color(Color32::LIGHT_RED),
                    );

                    zoom_to_fit
                        .then(|| {
                            let seg_extents = segments.iter().map(|s| AABB {
                                min_x: s.v1.x.min(s.v2.x),
                                min_y: s.v1.y.min(s.v2.y),
                                max_x: s.v1.x.max(s.v2.x),
                                max_y: s.v1.y.max(s.v2.y),
                            });

                            let all_extents =
                                seg_extents.chain(std::iter::once(pline.extents()).flatten());

                            total_bounds(all_extents)
                        })
                        .flatten()
                }
            };

            if let Some(extents) = zoom_to_bounds {
                let mut bounds = PlotBounds::from_min_max(
                    [extents.min_x, extents.min_y],
                    [extents.max_x, extents.max_y],
                );
                let margin = egui::vec2(0.05, 0.05);
                bounds.add_relative_margin_x(margin);
                bounds.add_relative_margin_y(margin);
                plot_ui.set_plot_bounds(bounds);
            }
        });
    });
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
