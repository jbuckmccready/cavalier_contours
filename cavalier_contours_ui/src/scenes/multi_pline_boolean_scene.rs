use cavalier_contours::{
    pline_closed,
    polyline::{BooleanOp, PlineSource, PlineSourceMut},
    shape_algorithms::Shape,
};
use eframe::egui::{CentralPanel, Color32, Rect, ScrollArea, SidePanel, Ui, Vec2};
use egui_plot::{Plot, PlotPoint};
use egui::Pos2;

use crate::plotting::PlinesPlotItem;
use crate::scenes::SceneSettings;

use super::{super::plotting::PLOT_VERTEX_RADIUS, Scene};

/// A simple example scene demonstrating shape boolean operations.
pub struct MultiPlineBooleanScene {
    /// The first shape object.
    shape1: Shape<f64>,
    /// The second shape object.
    shape2: Shape<f64>,
    /// Local interactive state (which shape or vertex, if any, we have grabbed, etc.).
    interaction_state: InteractionState,
}

/// Simple container for our user interaction (dragging, etc.).
struct InteractionState {
    grabbed_vertex: Option<(usize, bool, usize, usize)>,
    dragging: bool,
    zoom_to_fit: bool,
    boolean_op: Option<BooleanOp>,
}

/// Provide a default starting scene with two shapes that partially overlap.
impl Default for MultiPlineBooleanScene {
    fn default() -> Self {
        // For demonstration: let both shapes start with the same polylines but
        // offset them so shape1 and shape2 partially overlap.
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
            // A couple smaller arcs or lines for variety
            pline_closed![
                (261.11232783167395, 35.79686193615828, -1.0),
                (250.0, 100.0, -1.0),
            ],
            pline_closed![
                (320.2986109239592, 103.52378781211337, 0.0),
                (320.5065990423979, 76.14222955572362, -1.0),
            ],
            pline_closed![
                (273.6131273938006, -13.968608715397636, -0.3),
                (256.61336060995995, -25.49387433156079, 0.0),
                (249.69820124026208, 27.234215862385582, 0.0),
            ],
        ];
        // Build two shapes from the same polylines, then translate them so they overlap
        let mut shape1 = Shape::from_plines(plines.clone());
        let mut shape2 = Shape::from_plines(plines);
        // Translate them so they partially overlap
        shape1.translate_mut(-20.0, -20.0);
        shape2.translate_mut(20.0, 20.0);

        Self {
            shape1,
            shape2,
            interaction_state: InteractionState {
                grabbed_vertex: None,
                dragging: false,
                zoom_to_fit: false,
                boolean_op: None,
            },
        }
    }
}

/// Implement the `Scene` trait for our shape boolean example.
impl Scene for MultiPlineBooleanScene {
    fn name(&self) -> &'static str {
        "Multi Polyline Boolean"
    }

    fn ui(&mut self, ui: &mut Ui, settings: &SceneSettings, init: bool) {
        // destructure for convenience
        let MultiPlineBooleanScene {
            shape1,
            shape2,
            interaction_state,
        } = self;

        // create the "controls" panel on the right side
        controls_panel(ui, interaction_state);

        // force a "Zoom to Fit" if this is the first frame
        interaction_state.zoom_to_fit |= init;

        // next, render the "plot" area
        plot_area(ui, settings, shape1, shape2, interaction_state);
    }
}

/// The panel with controls where the user can pick the boolean op, etc.
fn controls_panel(ui: &mut Ui, interaction_state: &mut InteractionState) {
    SidePanel::right("multi_pline_boolean_panel")
        .min_width(200.0)
        .default_width(200.0)
        .show_inside(ui, |ui| {
            // use a scroll area if you expect to have many controls
            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                ui.add_space(ui.spacing().item_spacing.y);

                // pick boolean op
                ui.label("Boolean Operation");
                ui.radio_value(&mut interaction_state.boolean_op, None, "None");
                ui.radio_value(&mut interaction_state.boolean_op, Some(BooleanOp::Or), "Or");
                ui.radio_value(
                    &mut interaction_state.boolean_op,
                    Some(BooleanOp::And),
                    "And",
                );
                ui.radio_value(
                    &mut interaction_state.boolean_op,
                    Some(BooleanOp::Not),
                    "Not",
                );
                ui.radio_value(
                    &mut interaction_state.boolean_op,
                    Some(BooleanOp::Xor),
                    "Xor",
                );

                // "Zoom to Fit" button
                interaction_state.zoom_to_fit = ui
                    .button("Zoom to Fit")
                    .on_hover_text("Zoom to fit contents")
                    .clicked();
            });
        });
}

/// A helper function to search for any vertex in the provided shape's CCW or CW polylines that
/// lies "near" a given screen coordinate. If found, returns `(shape_index, orientation, pline_idx, vertex_idx)`.
fn find_near_vertex(
    ui_coord: Pos2,
    plot_ui: &egui_plot::PlotUi,
    shape_index: usize,
    shape: &Shape<f64>,
) -> Option<(usize, bool, usize, usize)> {
    let _pointer_pos = plot_ui.plot_from_screen(ui_coord);

    // We'll check CCW polylines first (orientation = true), then CW polylines (orientation = false).
    // If you want the opposite priority, simply reverse the checks.
    // The first match we find, we return immediately (so we pick the "topmost" or first found).
    for (pline_idx, rpline) in shape.ccw_plines.iter().enumerate() {
        let pline = &rpline.polyline;
        for v_idx in 0..pline.vertex_count() {
            let v = pline.at(v_idx);
            let screen_v = plot_ui.screen_from_plot(PlotPoint::new(v.x, v.y));
            let hit_size =
                2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);
            let rect = Rect::from_center_size(screen_v, Vec2::splat(hit_size));
            if rect.contains(ui_coord) {
                return Some((shape_index, true, pline_idx, v_idx));
            }
        }
    }

    for (pline_idx, rpline) in shape.cw_plines.iter().enumerate() {
        let pline = &rpline.polyline;
        for v_idx in 0..pline.vertex_count() {
            let v = pline.at(v_idx);
            let screen_v = plot_ui.screen_from_plot(PlotPoint::new(v.x, v.y));
            let hit_size =
                2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);
            let rect = Rect::from_center_size(screen_v, Vec2::splat(hit_size));
            if rect.contains(ui_coord) {
                return Some((shape_index, false, pline_idx, v_idx));
            }
        }
    }

    None
}

/// Return a mutable reference to shape1 or shape2, based on `shape_idx`.
fn pick_shape_mut<'a>(
    shape_idx: usize,
    shape1: &'a mut Shape<f64>,
    shape2: &'a mut Shape<f64>,
) -> &'a mut Shape<f64> {
    if shape_idx == 0 {
        shape1
    } else {
        shape2
    }
}

/// The center "plot" area where we draw our shapes and do user interactions.
fn plot_area(
    ui: &mut Ui,
    settings: &SceneSettings,
    shape1: &mut Shape<f64>,
    shape2: &mut Shape<f64>,
    interaction_state: &mut InteractionState,
) {
    let InteractionState {
        grabbed_vertex,
        dragging,
        zoom_to_fit,
        boolean_op,
    } = interaction_state;

    let mut shape_result = Shape::empty();

    CentralPanel::default().show_inside(ui, |ui| {
        // set up the Plot widget
        let plot =
            settings.apply_to_plot(Plot::new("plot_area").data_aspect(1.0).allow_drag(false));

        // TODO: color pickers
        let color1 = Color32::LIGHT_BLUE;
        let color2 = Color32::LIGHT_RED;
        let color3 = Color32::LIGHT_GREEN;
        let opacity1 = 0.2;
        let opacity2 = 0.8;
        let fill_color1 = color1.gamma_multiply(opacity1);
        let fill_color2 = color2.gamma_multiply(opacity1);
        let fill_color3 = color3.gamma_multiply(opacity2);

        plot.show(ui, |plot_ui| {
            // disable auto-bounds by default; we can set them manually or on user action
            plot_ui.set_auto_bounds([false, false]);

            // check for pointer releases
            if plot_ui.ctx().input(|i| i.pointer.any_released()) {
                if grabbed_vertex.is_none() {
                    *dragging = false;
                } else {
                    // release vertex (stop dragging that vertex)
                    *grabbed_vertex = None;
                }
            }

            if let Some((shape_idx, is_ccw, pline_idx, v_idx)) = *grabbed_vertex {
                let shape = pick_shape_mut(shape_idx, shape1, shape2);
                let rpline = if is_ccw {
                    &mut shape.ccw_plines[pline_idx]
                } else {
                    &mut shape.cw_plines[pline_idx]
                };
                let pline = &mut rpline.polyline;
            
                // Apply the pointer delta
                let delta = plot_ui.pointer_coordinate_drag_delta();
                if v_idx < pline.vertex_count() {
                    let v = pline.at(v_idx);
                    pline.set(v_idx, v.x + delta.x as f64, v.y + delta.y as f64, v.bulge);
                }
                rpline.spatial_index = pline.create_aabb_index();
            } else if *dragging {
                // if we're not dragging a vertex, but the user is dragging the mouse, pan the plot
                plot_ui.translate_bounds(-plot_ui.pointer_coordinate_drag_delta());
            } else if plot_ui.ctx().input(|i| i.pointer.any_pressed()) {
                // pointer pressed, see if we clicked near any vertex from shape1 or shape2
                if let Some(coord) = plot_ui.ctx().pointer_interact_pos() {
                    let found_vertex = find_near_vertex(coord, plot_ui, 0, &*shape1)
                        .or_else(|| find_near_vertex(coord, plot_ui, 1, &*shape2));

                    if let Some(gv) = found_vertex {
                        *grabbed_vertex = Some(gv);
                    }

                    // If we didn't grab a vertex, treat the drag as a plot pan.
                    *dragging = grabbed_vertex.is_none();
                }
            }

            if boolean_op.is_none() {
                // draw shapes
                // 1) shape1
                plot_ui.add(
                    PlinesPlotItem::new(&*shape1)
                        .stroke_color(Color32::LIGHT_BLUE)
                        .fill_color(fill_color1)
                        .vertex_color(Color32::LIGHT_BLUE),
                );
                // 2) shape2
                plot_ui.add(
                    PlinesPlotItem::new(&*shape2)
                        .stroke_color(Color32::LIGHT_RED)
                        .fill_color(fill_color2)
                        .vertex_color(Color32::LIGHT_RED),
                );
            } else {
                // We have a boolean op; compute the shape and draw the result
                let bool_result = shape1.boolean(shape2, boolean_op.unwrap());
                // build a shape from the boolean result for plotting
                let all_plines = bool_result
                    .ccw_plines
                    .into_iter()
                    .chain(bool_result.cw_plines)
                    .map(|rp| rp.polyline); // ignoring sign for demonstration
                shape_result = Shape::from_plines(all_plines);

                // add the shape result to the plot as well
                plot_ui.add(
                    PlinesPlotItem::new(&shape_result)
                        .stroke_color(Color32::LIGHT_GREEN)
                        .fill_color(fill_color3)
                        .vertex_color(Color32::LIGHT_GREEN),
                );
            }

            // if user asked "zoom to fit," we do it
            if *zoom_to_fit {
                plot_ui.set_auto_bounds([true, true]);
            }
        });
    });
}
