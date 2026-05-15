use cavalier_contours::polyline::{PlineSource, PlineSourceMut, PlineVertex, Polyline};
use eframe::egui::{ScrollArea, TextEdit, Ui, Vec2, Window};

pub mod components;

const DEFAULT_VERTEX_X: f64 = 0.0;
const DEFAULT_VERTEX_Y: f64 = 0.0;
const DEFAULT_VERTEX_BULGE: f64 = 0.0;
const DEFAULT_ACCORDION_OPEN_COUNT: usize = 3;

/// Represents the active tab in the polyline editor window
#[derive(Clone, Copy, PartialEq, Default)]
pub enum EditorTab {
    #[default]
    Table,
    Json,
}

#[derive(Clone)]
pub struct PolylineEditorConfig {
    pub window_title: String,
    pub default_width: f32,
    pub default_height: f32,
    pub table_layout: TableLayout,
    pub json_format: JsonFormat,
}

/// Defines how polyline tables are laid out in the editor
#[derive(Clone)]
pub enum TableLayout {
    /// Single table for one polyline
    Single,
    /// Two tables side-by-side for two polylines
    SideBySide,
    /// Accordion layout for N polylines with collapsible sections
    Accordion,
}

/// Defines the JSON serialization format for polylines
#[derive(Clone, Debug)]
pub enum JsonFormat {
    /// Single polyline as root object
    Single,
    /// Two polylines in a combined object with custom keys
    Combined {
        pline1_key: String,
        pline2_key: String,
    },
    /// Multiple polylines in an array under "polylines" key
    Array,
}

/// A versatile polyline editor that supports editing 1 to N polylines with different UI layouts.
///
/// The editor provides:
/// - **Table editing**: Interactive vertex tables with drag values and delete buttons
/// - **JSON import/export**: Full polyline serialization with validation
/// - **Multiple layouts**: Single table, side-by-side, or accordion for N polylines
/// - **Smart vertex creation**: New vertices are positioned intelligently
/// - **Change tracking**: Pending changes indicator with apply/cancel functionality
pub struct PolylineEditor {
    show_window: bool,
    pending_state: Vec<Polyline>,
    current_json: String,
    pending_json: String,
    json_error: Option<String>,
    active_tab: EditorTab,
    config: PolylineEditorConfig,
    is_initialized: bool,
}

impl PolylineEditor {
    // ===== Public API for state operations =====

    /// Add a vertex to the specified polyline
    pub fn add_vertex(&mut self, polyline_index: usize) {
        self.ensure_polyline_capacity(polyline_index);
        let new_vertex = self.create_vertex_for_pline(polyline_index);
        self.pending_state[polyline_index].add(new_vertex.x, new_vertex.y, new_vertex.bulge);
    }

    /// Remove a vertex from the specified polyline
    pub fn remove_vertex(&mut self, polyline_index: usize, vertex_index: usize) {
        if polyline_index < self.pending_state.len() {
            let pline = &mut self.pending_state[polyline_index];
            if vertex_index < pline.vertex_count() {
                pline.remove(vertex_index);
            }
        }
    }

    /// Toggle the is_closed state for the specified polyline
    pub fn toggle_is_closed(&mut self, polyline_index: usize) {
        self.ensure_polyline_capacity(polyline_index);
        let current_closed = self.pending_state[polyline_index].is_closed();
        self.pending_state[polyline_index].set_is_closed(!current_closed);
    }

    /// Set the is_closed state for the specified polyline
    pub fn set_is_closed(&mut self, polyline_index: usize, is_closed: bool) {
        self.ensure_polyline_capacity(polyline_index);
        self.pending_state[polyline_index].set_is_closed(is_closed);
    }

    /// Delete the specified polyline
    pub fn delete_polyline(&mut self, polyline_index: usize) {
        if polyline_index < self.pending_state.len() {
            self.pending_state.remove(polyline_index);
        }
    }

    /// Cancel all pending changes, reverting to the current state
    pub fn cancel_changes(&mut self, polylines: &[Polyline]) {
        self.pending_state = polylines.to_vec();
        self.pending_json = self.current_json.clone();
        self.json_error = None;
    }

    /// Check if there are any pending changes in the table editor
    pub fn has_pending_table_changes(&self, polylines: &[Polyline]) -> bool {
        for (pl1, pl2) in polylines.iter().zip(self.pending_state.iter()) {
            if pl1.is_closed() != pl2.is_closed()
                || pl1.vertex_count() != pl2.vertex_count()
                || pl1
                    .iter_vertexes()
                    .zip(pl2.iter_vertexes())
                    .any(|(v1, v2)| v1 != v2)
            {
                return true;
            }
        }

        false
    }

    /// Check if there are any pending changes in the JSON editor
    pub fn has_pending_json_changes(&self) -> bool {
        self.current_json != self.pending_json
    }

    /// Get the current number of polylines being edited
    pub fn polyline_count(&self) -> usize {
        self.pending_state.len()
    }

    /// Get the vertex count for a specific polyline
    pub fn vertex_count(&self, polyline_index: usize) -> usize {
        self.pending_state
            .get(polyline_index)
            .map_or(0, |p| p.vertex_count())
    }

    /// Get the is_closed state for a specific polyline
    pub fn get_is_closed(&self, polyline_index: usize) -> bool {
        self.pending_state
            .get(polyline_index)
            .is_some_and(|p| p.is_closed())
    }

    /// Apply all pending table changes to the provided polylines
    pub fn apply_table_changes(&mut self, polylines: &mut Vec<Polyline>) {
        *polylines = self.pending_state.clone();
        self.current_json = self.serialize_polylines(polylines);
        self.pending_json = self.current_json.clone();
        self.json_error = None;
    }

    // ===== Constructor methods =====

    /// Creates a polyline editor configured for editing a single polyline
    pub fn single(title: &str) -> Self {
        Self::new(PolylineEditorConfig {
            window_title: title.to_string(),
            default_width: 800.0,
            default_height: 500.0,
            table_layout: TableLayout::Single,
            json_format: JsonFormat::Single,
        })
    }

    /// Creates a polyline editor configured for editing two polylines side-by-side
    pub fn dual(title: &str) -> Self {
        Self::new(PolylineEditorConfig {
            window_title: title.to_string(),
            default_width: 750.0,
            default_height: 550.0,
            table_layout: TableLayout::SideBySide,
            json_format: JsonFormat::Combined {
                pline1_key: "polyline1".to_string(),
                pline2_key: "polyline2".to_string(),
            },
        })
    }

    /// Creates a polyline editor configured for editing multiple polylines with accordion layout
    pub fn multi(title: &str) -> Self {
        Self::new(PolylineEditorConfig {
            window_title: title.to_string(),
            default_width: 900.0,
            default_height: 600.0,
            table_layout: TableLayout::Accordion,
            json_format: JsonFormat::Array,
        })
    }

    fn new(config: PolylineEditorConfig) -> Self {
        Self {
            show_window: false,
            pending_state: Vec::new(),
            current_json: String::new(),
            pending_json: String::new(),
            json_error: None,
            active_tab: EditorTab::Table,
            config,
            is_initialized: false,
        }
    }

    pub fn show_window(&mut self) {
        self.show_window = true;
    }

    pub fn is_window_open(&self) -> bool {
        self.show_window
    }

    pub fn initialize_with_polylines(&mut self, polylines: Vec<Polyline>) {
        self.pending_state = polylines;
        self.current_json = self.serialize_polylines(&self.pending_state);
        self.pending_json = self.current_json.clone();
        self.json_error = None;
    }

    pub fn ui_show(
        &mut self,
        ctx: &egui::Context,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        // Detect first opening and refresh state with passed in (current) state
        if self.show_window && !self.is_initialized {
            self.pending_state = polylines.clone();
            self.current_json = self.serialize_polylines(&self.pending_state);
            self.pending_json = self.current_json.clone();
            self.json_error = None;
            self.is_initialized = true;
        }

        let mut open = self.show_window;
        Window::new(&self.config.window_title)
            .open(&mut open)
            .default_width(self.config.default_width)
            .default_height(self.config.default_height)
            .resizable(true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.active_tab, EditorTab::Table, "Table Editor");
                    ui.selectable_value(
                        &mut self.active_tab,
                        EditorTab::Json,
                        "JSON Import/Export",
                    );
                });

                ui.separator();

                // Show error if any
                if let Some(error) = &self.json_error {
                    ui.colored_label(colors.error_color, format!("Error: {error}"));
                }

                match self.active_tab {
                    EditorTab::Table => {
                        self.show_table_tab(ui, polylines, colors);
                    }
                    EditorTab::Json => {
                        self.show_json_tab(ui, polylines, colors);
                    }
                }
            });

        self.show_window = open;

        // Reset initialization flag when window is closed so it refreshes on next open
        if !open {
            self.is_initialized = false;
        }
    }

    fn show_table_tab(
        &mut self,
        ui: &mut Ui,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        match self.config.table_layout {
            TableLayout::Single => {
                self.show_single_table_tab(ui, polylines, colors);
            }
            TableLayout::SideBySide => {
                self.show_dual_tables_tab(ui, polylines, colors);
            }
            TableLayout::Accordion => {
                self.show_accordion_tables_tab(ui, polylines, colors);
            }
        }
    }

    fn show_single_table_tab(
        &mut self,
        ui: &mut Ui,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
            ui.label("Edit polyline vertices:");

            ui.horizontal(|ui| {
                let has_pending_changes = self.has_pending_table_changes(polylines);

                // Apply table changes button
                let apply_button = ui
                    .button("Apply Changes")
                    .on_hover_text("Update polyline from table data");

                if apply_button.clicked() {
                    self.apply_table_changes(polylines);
                }

                ui.separator();

                // Add vertex button
                if ui.button("Add Vertex").clicked() {
                    self.add_vertex(0);
                }

                // Show pending changes indicator and cancel button
                components::show_pending_changes_ui(
                    ui,
                    has_pending_changes,
                    || {
                        self.cancel_changes(polylines);
                    },
                    colors,
                );
            });

            ui.separator();

            // Vertex table
            ScrollArea::both().show(ui, |ui| {
                if let Some(pline) = self.pending_state.get_mut(0) {
                    components::show_vertex_table(
                        ui,
                        &mut pline.vertex_data,
                        "single_vertex_table",
                    );
                }
            });
        });
    }

    fn show_dual_tables_tab(
        &mut self,
        ui: &mut Ui,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        ui.vertical(|ui| {
            // Controls section
            ui.horizontal(|ui| {
                let has_pending_changes = self.has_pending_table_changes(polylines);

                // Apply table changes button
                let apply_button = ui
                    .button("Apply Changes")
                    .on_hover_text("Update polylines from table data");

                if apply_button.clicked() {
                    self.apply_table_changes(polylines);
                }

                ui.separator();

                // Add vertex buttons
                if ui.button("Add Vertex to Polyline 1").clicked() {
                    self.add_vertex(0);
                }

                if ui.button("Add Vertex to Polyline 2").clicked() {
                    self.add_vertex(1);
                }

                // Show pending changes indicator and cancel button
                components::show_pending_changes_ui(
                    ui,
                    has_pending_changes,
                    || {
                        self.cancel_changes(polylines);
                    },
                    colors,
                );
            });

            ui.separator();

            // Tables section
            let remaining_height = ui.available_height();
            ui.horizontal(|ui| {
                // Polyline 1 table
                ui.allocate_ui_with_layout(
                    Vec2::new(ui.available_width() * 0.48, remaining_height),
                    egui::Layout::top_down(egui::Align::LEFT),
                    |ui| {
                        ui.heading("Polyline 1");
                        let table_height =
                            ui.available_height() - ui.spacing().item_spacing.y * 2.0;
                        ScrollArea::both()
                            .id_salt("pline1_table_scroll")
                            .max_height(table_height)
                            .show(ui, |ui| {
                                if let Some(pline) = self.pending_state.get_mut(0) {
                                    components::show_vertex_table(
                                        ui,
                                        &mut pline.vertex_data,
                                        "pline1_vertex_table",
                                    );
                                }
                            });
                    },
                );

                ui.separator();

                // Polyline 2 table
                ui.allocate_ui_with_layout(
                    Vec2::new(ui.available_width() * 0.96, remaining_height),
                    egui::Layout::top_down(egui::Align::LEFT),
                    |ui| {
                        ui.heading("Polyline 2");
                        let table_height =
                            ui.available_height() - ui.spacing().item_spacing.y * 2.0;
                        ScrollArea::both()
                            .id_salt("pline2_table_scroll")
                            .max_height(table_height)
                            .show(ui, |ui| {
                                if let Some(pline) = self.pending_state.get_mut(1) {
                                    components::show_vertex_table(
                                        ui,
                                        &mut pline.vertex_data,
                                        "pline2_vertex_table",
                                    );
                                }
                            });
                    },
                );
            });
        });
    }

    fn show_accordion_tables_tab(
        &mut self,
        ui: &mut Ui,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
            ui.label("Edit polylines vertices:");

            ui.horizontal(|ui| {
                let has_pending_changes = self.has_pending_table_changes(polylines);

                // Apply table changes button
                let apply_button = ui
                    .button("Apply Changes")
                    .on_hover_text("Update polylines from table data");

                if apply_button.clicked() {
                    self.apply_table_changes(polylines);
                }

                ui.separator();

                // Show pending changes indicator and cancel button
                components::show_pending_changes_ui(
                    ui,
                    has_pending_changes,
                    || {
                        self.cancel_changes(polylines);
                    },
                    colors,
                );
            });

            ui.separator();

            // Add new polyline button
            if ui.button("Add New Polyline").clicked() {
                let new_index = self.polyline_count();
                self.add_vertex(new_index);
            }

            ui.separator();

            // Accordion sections for each polyline
            for i in 0..self.polyline_count() {
                let polyline_title = format!("Polyline {}", i + 1);

                egui::CollapsingHeader::new(&polyline_title)
                    .default_open(i < DEFAULT_ACCORDION_OPEN_COUNT)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            // Add vertex button for this specific polyline
                            if ui.button("Add Vertex").clicked() {
                                self.add_vertex(i);
                            }

                            // Toggle is_closed for this polyline (track as pending change)
                            if let Some(pline) = self.pending_state.get_mut(i) {
                                let mut is_closed = pline.is_closed();
                                if ui.checkbox(&mut is_closed, "Closed").changed() {
                                    pline.set_is_closed(is_closed);
                                }
                            }

                            // Delete polyline button
                            if ui.button("Delete").clicked() {
                                self.delete_polyline(i);
                            }

                            // Vertex count info
                            let vertex_count = self.vertex_count(i);
                            ui.label(format!("Vertices: {vertex_count}"));
                        });

                        ui.separator();

                        // Vertex table for this polyline
                        if let Some(pline) = self.pending_state.get_mut(i) {
                            let table_id = format!("polyline_{i}_table");
                            components::show_vertex_table(ui, &mut pline.vertex_data, &table_id);
                        } else {
                            ui.label("No vertices");
                        }
                    });
            }
        });
    }

    fn show_json_tab(
        &mut self,
        ui: &mut Ui,
        polylines: &mut Vec<Polyline>,
        colors: &crate::theme::ThemeColors,
    ) {
        ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
            ui.label("JSON Editor:");

            ui.horizontal(|ui| {
                let has_pending_changes = self.has_pending_json_changes();

                // Apply JSON changes button
                let apply_button = ui
                    .button("Apply")
                    .on_hover_text("Apply JSON changes to polyline(s)");

                if apply_button.clicked() {
                    self.apply_json_changes_to_polylines(polylines);
                }

                // Show pending changes indicator and cancel button
                components::show_pending_changes_ui(
                    ui,
                    has_pending_changes,
                    || {
                        self.cancel_changes(polylines);
                    },
                    colors,
                );
            });

            ui.separator();

            ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                let text_edit = TextEdit::multiline(&mut self.pending_json)
                    .desired_rows(20)
                    .font(egui::TextStyle::Monospace);
                ui.add_sized(ui.available_size(), text_edit);
            });
        });
    }

    fn apply_json_changes_to_polylines(&mut self, polylines: &mut Vec<Polyline>) {
        match self.parse_polylines_from_json(&self.pending_json) {
            Ok(new_polylines) => {
                *polylines = new_polylines;
                self.pending_state = polylines.clone();
                self.current_json = self.serialize_polylines(polylines);
                self.pending_json = self.current_json.clone();
                self.json_error = None;
            }
            Err(e) => {
                self.json_error = Some(e);
            }
        }
    }

    /// Parses a JSON string into a vector of polylines based on the editor's config.
    fn parse_polylines_from_json(&self, json_str: &str) -> Result<Vec<Polyline>, String> {
        match &self.config.json_format {
            JsonFormat::Single => {
                let pline = serde_json::from_str(json_str)
                    .map_err(|e| format!("Failed to parse JSON: {e}"))?;
                Ok(vec![pline])
            }
            JsonFormat::Combined {
                pline1_key,
                pline2_key,
            } => {
                let combined: serde_json::Value = serde_json::from_str(json_str)
                    .map_err(|e| format!("Failed to parse JSON: {e}"))?;

                let p1_json = combined.get(pline1_key).ok_or_else(|| {
                    format!("JSON must contain '{pline1_key}' and '{pline2_key}' fields")
                })?;
                let p2_json = combined.get(pline2_key).ok_or_else(|| {
                    format!("JSON must contain '{pline1_key}' and '{pline2_key}' fields")
                })?;

                let pline1 = serde_json::from_value(p1_json.clone())
                    .map_err(|_| "Failed to parse polyline 1 from JSON".to_string())?;
                let pline2 = serde_json::from_value(p2_json.clone())
                    .map_err(|_| "Failed to parse polyline 2 from JSON".to_string())?;

                Ok(vec![pline1, pline2])
            }
            JsonFormat::Array => {
                let combined: serde_json::Value = serde_json::from_str(json_str)
                    .map_err(|e| format!("Failed to parse JSON: {e}"))?;

                let polylines_json = combined
                    .get("polylines")
                    .ok_or_else(|| "JSON must contain 'polylines' field".to_string())?;

                let polylines_array = polylines_json
                    .as_array()
                    .ok_or_else(|| "'polylines' field must be an array".to_string())?;

                polylines_array
                    .iter()
                    .map(|pline_json| {
                        serde_json::from_value(pline_json.clone())
                            .map_err(|_| "Failed to parse one or more polylines".to_string())
                    })
                    .collect()
            }
        }
    }

    /// Ensures pending_state has capacity for the given polyline index
    fn ensure_polyline_capacity(&mut self, polyline_index: usize) {
        while self.pending_state.len() <= polyline_index {
            self.pending_state.push(Polyline::new());
        }
    }

    /// Creates a new vertex at a reasonable position for the given polyline index
    fn create_vertex_for_pline(&self, polyline_index: usize) -> PlineVertex {
        if let Some(pline) = self.pending_state.get(polyline_index) {
            if pline.vertex_count() > 0 {
                let last_vertex = pline.iter_vertexes().next_back().unwrap();
                // Add the new vertex after the last existing vertex with a small offset
                PlineVertex::new(
                    last_vertex.x + 10.0,
                    last_vertex.y + 10.0,
                    DEFAULT_VERTEX_BULGE,
                )
            } else {
                // Default position if no existing vertices
                PlineVertex::new(DEFAULT_VERTEX_X, DEFAULT_VERTEX_Y, DEFAULT_VERTEX_BULGE)
            }
        } else {
            // Default position if polyline doesn't exist yet
            PlineVertex::new(DEFAULT_VERTEX_X, DEFAULT_VERTEX_Y, DEFAULT_VERTEX_BULGE)
        }
    }

    fn serialize_polylines(&self, polylines: &[Polyline]) -> String {
        match &self.config.json_format {
            JsonFormat::Single => {
                if !polylines.is_empty() {
                    serde_json::to_string_pretty(&polylines[0]).unwrap_or_default()
                } else {
                    String::new()
                }
            }
            JsonFormat::Combined {
                pline1_key,
                pline2_key,
            } => {
                if polylines.len() >= 2 {
                    let combined = serde_json::json!({
                        pline1_key: polylines[0],
                        pline2_key: polylines[1]
                    });
                    serde_json::to_string_pretty(&combined).unwrap_or_default()
                } else {
                    String::new()
                }
            }
            JsonFormat::Array => {
                let combined = serde_json::json!({
                    "polylines": polylines
                });
                serde_json::to_string_pretty(&combined).unwrap_or_default()
            }
        }
    }
}
