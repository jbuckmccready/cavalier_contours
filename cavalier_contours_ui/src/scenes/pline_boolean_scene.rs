use cavalier_contours::{
    pline_closed,
    polyline::{BooleanOp, BooleanResult, PlineSource, PlineSourceMut, Polyline},
    shape_algorithms::Shape,
};
use eframe::egui::{CentralPanel, Color32, Rect, ScrollArea, SidePanel, Ui, Vec2};
use egui_plot::{Plot, PlotPoint};

use crate::plotting::ShapePlotItem;

use super::{
    super::plotting::{PLOT_VERTEX_RADIUS, PlinePlotItem},
    Scene,
};

pub struct PlineBooleanScene {
    pline1: Polyline,
    pline2: Polyline,
    mode: Mode,
    fill: bool,
    show_vertexes: bool,
    interaction_state: InteractionState,
}

#[derive(Default, Clone, Copy, PartialEq)]
enum Mode {
    #[default]
    None,
    Or,
    And,
    Not,
    Xor,
}

impl Mode {
    fn label(&self) -> &'static str {
        match self {
            Mode::None => "None",
            Mode::Or => "Or",
            Mode::And => "And",
            Mode::Not => "Not",
            Mode::Xor => "Xor",
        }
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

        Self {
            pline1,
            pline2,
            mode: Mode::default(),
            fill: true,
            show_vertexes: true,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
            },
        }
    }
}

impl Scene for PlineBooleanScene {
    fn name(&self) -> &'static str {
        "Polyline Boolean"
    }

    fn ui(&mut self, ui: &mut Ui, init: bool) {
        let PlineBooleanScene {
            pline1,
            pline2,
            mode,
            fill,
            show_vertexes,
            interaction_state,
        } = self;

        controls_panel(ui, mode, fill, show_vertexes, interaction_state);

        interaction_state.zoom_to_fit |= init;
        plot_area(
            ui,
            pline1,
            pline2,
            mode,
            fill,
            show_vertexes,
            interaction_state,
        );
    }
}

fn controls_panel(
    ui: &mut Ui,
    mode: &mut Mode,
    fill: &mut bool,
    show_vertexes: &mut bool,
    interaction_state: &mut InteractionState,
) {
    SidePanel::right("pline_offset_panel")
        .min_width(200.0)
        .default_width(200.0)
        .show_inside(ui, |ui| {
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                egui::ComboBox::from_label("Mode")
                    .selected_text(mode.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(mode, Mode::None, "None");
                        ui.selectable_value(mode, Mode::Or, "Or");
                        ui.selectable_value(mode, Mode::And, "And");
                        ui.selectable_value(mode, Mode::Not, "Not");
                        ui.selectable_value(mode, Mode::Xor, "Xor");
                    });

                ui.checkbox(fill, "Fill");
                ui.checkbox(show_vertexes, "Show Vertexes");

                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();
            })
        });
}

fn plot_area(
    ui: &mut Ui,
    pline1: &mut Polyline,
    pline2: &mut Polyline,
    mode: &Mode,
    fill: &bool,
    show_vertexes: &bool,
    interaction_state: &mut InteractionState,
) {
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
    } = interaction_state;

    // TODO: cache scene state to only update when necessary due to modified polylines
    let scene_state = build_boolean_result(pline1, pline2, mode);

    CentralPanel::default().show_inside(ui, |ui| {
        let plot = Plot::new("plot_area").data_aspect(1.0).allow_drag(false);

        // stack variable for shape used for plot item (required for lifetime)
        let mut shape = Shape::empty();
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

            // TODO: color pickers
            let color1 = Color32::LIGHT_BLUE;
            let color2 = Color32::LIGHT_RED;
            let opacity = 0.8;
            let fill_color1 = color1.gamma_multiply(opacity);
            let fill_color2 = color2.gamma_multiply(opacity);

            let mut plot_item1 = PlinePlotItem::new(pline1);
            let mut plot_item2 = PlinePlotItem::new(pline2);

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
                        .chain(result.neg_plines.into_iter())
                        .map(|pl| pl.pline);

                    shape = Shape::from_plines(all_plines);

                    plot_ui.add(
                        ShapePlotItem::new(&shape)
                            .fill_color(fill_color1)
                            .stroke_color(color1),
                    );
                }
            }

            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });
}

fn build_boolean_result(pline1: &Polyline, pline2: &Polyline, mode: &Mode) -> SceneState {
    if pline1.vertex_count() < 2
        || pline2.vertex_count() < 2
        || !pline1.is_closed()
        || !pline2.is_closed()
    {
        return SceneState::NoOp;
    }

    let op = match mode {
        Mode::None => return SceneState::NoOp,
        Mode::Or => BooleanOp::Or,
        Mode::And => BooleanOp::And,
        Mode::Not => BooleanOp::Not,
        Mode::Xor => BooleanOp::Xor,
    };

    SceneState::BooleanResult(pline1.boolean(pline2, op))
}
