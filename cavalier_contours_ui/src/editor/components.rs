use cavalier_contours::polyline::PlineVertex;
use eframe::egui::Ui;

pub fn show_vertex_table(ui: &mut Ui, vertex_data: &mut Vec<PlineVertex>, table_id: &str) -> bool {
    use egui_extras::{Column, TableBuilder};

    let mut changed = false;

    TableBuilder::new(ui)
        .id_salt(table_id)
        .striped(true) // Add alternating row colors for better readability
        .column(Column::auto().at_least(40.0).at_most(50.0)) // Index
        .column(Column::initial(80.0).at_least(60.0).at_most(120.0)) // X
        .column(Column::initial(80.0).at_least(60.0).at_most(120.0)) // Y
        .column(Column::initial(80.0).at_least(60.0).at_most(120.0)) // Bulge
        .column(Column::auto().at_least(40.0).at_most(50.0)) // Delete
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
            let mut to_delete: Option<usize> = None;

            for (i, vertex) in vertex_data.iter_mut().enumerate() {
                body.row(18.0, |mut row| {
                    row.col(|ui| {
                        ui.label(format!("{i}"));
                    });
                    row.col(|ui| {
                        changed |= ui
                            .add(egui::DragValue::new(&mut vertex.x).speed(0.1))
                            .changed();
                    });
                    row.col(|ui| {
                        changed |= ui
                            .add(egui::DragValue::new(&mut vertex.y).speed(0.1))
                            .changed();
                    });
                    row.col(|ui| {
                        changed |= ui
                            .add(egui::DragValue::new(&mut vertex.bulge).speed(0.01))
                            .changed();
                    });
                    row.col(|ui| {
                        ui.with_layout(
                            egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                            |ui| {
                                if ui
                                    .button("🗑")
                                    .on_hover_text(format!("Delete vertex {i}"))
                                    .clicked()
                                {
                                    to_delete = Some(i);
                                    changed = true;
                                }
                            },
                        );
                    });
                });
            }

            if let Some(index) = to_delete {
                vertex_data.remove(index);
            }
        });

    changed
}

pub fn show_pending_changes_ui<F>(
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
