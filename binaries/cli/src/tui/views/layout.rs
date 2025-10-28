/// Layout Management for Component-Based Views
///
/// This module provides layout management for arranging components within views.
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use std::collections::HashMap;

use crate::tui::ComponentId;

/// Layout configuration for a view
#[derive(Debug, Clone)]
pub enum LayoutConfig {
    /// Single component fills entire area
    Single { component_id: ComponentId },

    /// Vertical split
    Vertical {
        components: Vec<(ComponentId, Constraint)>,
    },

    /// Horizontal split
    Horizontal {
        components: Vec<(ComponentId, Constraint)>,
    },

    /// Grid layout
    Grid {
        rows: usize,
        cols: usize,
        components: Vec<(ComponentId, usize, usize)>, // component_id, row, col
    },

    /// Custom layout with explicit areas
    Custom {
        component_areas: HashMap<ComponentId, Rect>,
    },
}

/// Layout manager for calculating component positions
pub struct LayoutManager {
    config: Option<LayoutConfig>,
}

impl LayoutManager {
    pub fn new() -> Self {
        Self { config: None }
    }

    pub fn set_layout(&mut self, config: LayoutConfig) {
        self.config = Some(config);
    }

    /// Calculate layout areas for all components
    pub fn calculate_layout(&self, area: Rect) -> HashMap<ComponentId, Rect> {
        let mut result = HashMap::new();

        if let Some(config) = &self.config {
            match config {
                LayoutConfig::Single { component_id } => {
                    result.insert(component_id.clone(), area);
                }

                LayoutConfig::Vertical { components } => {
                    let constraints: Vec<Constraint> = components
                        .iter()
                        .map(|(_, constraint)| *constraint)
                        .collect();

                    let chunks = Layout::default()
                        .direction(Direction::Vertical)
                        .constraints(constraints)
                        .split(area);

                    for (i, (component_id, _)) in components.iter().enumerate() {
                        if i < chunks.len() {
                            result.insert(component_id.clone(), chunks[i]);
                        }
                    }
                }

                LayoutConfig::Horizontal { components } => {
                    let constraints: Vec<Constraint> = components
                        .iter()
                        .map(|(_, constraint)| *constraint)
                        .collect();

                    let chunks = Layout::default()
                        .direction(Direction::Horizontal)
                        .constraints(constraints)
                        .split(area);

                    for (i, (component_id, _)) in components.iter().enumerate() {
                        if i < chunks.len() {
                            result.insert(component_id.clone(), chunks[i]);
                        }
                    }
                }

                LayoutConfig::Grid {
                    rows,
                    cols,
                    components,
                } => {
                    let row_height = area.height / *rows as u16;
                    let col_width = area.width / *cols as u16;

                    for (component_id, row, col) in components {
                        if row < rows && col < cols {
                            let x = area.x + (*col as u16 * col_width);
                            let y = area.y + (*row as u16 * row_height);

                            result.insert(
                                component_id.clone(),
                                Rect {
                                    x,
                                    y,
                                    width: col_width,
                                    height: row_height,
                                },
                            );
                        }
                    }
                }

                LayoutConfig::Custom { component_areas } => {
                    result = component_areas.clone();
                }
            }
        }

        result
    }

    /// Get layout for a specific component
    pub fn get_component_area(&self, area: Rect, component_id: &ComponentId) -> Option<Rect> {
        self.calculate_layout(area).get(component_id).copied()
    }
}

impl Default for LayoutManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_layout() {
        let mut manager = LayoutManager::new();
        let component_id = ComponentId::new("test");

        manager.set_layout(LayoutConfig::Single {
            component_id: component_id.clone(),
        });

        let area = Rect {
            x: 0,
            y: 0,
            width: 100,
            height: 50,
        };
        let layout = manager.calculate_layout(area);

        assert_eq!(layout.len(), 1);
        assert_eq!(layout.get(&component_id), Some(&area));
    }

    #[test]
    fn test_vertical_layout() {
        let mut manager = LayoutManager::new();
        let comp1 = ComponentId::new("comp1");
        let comp2 = ComponentId::new("comp2");

        manager.set_layout(LayoutConfig::Vertical {
            components: vec![
                (comp1.clone(), Constraint::Percentage(50)),
                (comp2.clone(), Constraint::Percentage(50)),
            ],
        });

        let area = Rect {
            x: 0,
            y: 0,
            width: 100,
            height: 100,
        };
        let layout = manager.calculate_layout(area);

        assert_eq!(layout.len(), 2);
        assert!(layout.contains_key(&comp1));
        assert!(layout.contains_key(&comp2));
    }

    #[test]
    fn test_horizontal_layout() {
        let mut manager = LayoutManager::new();
        let comp1 = ComponentId::new("left");
        let comp2 = ComponentId::new("right");

        manager.set_layout(LayoutConfig::Horizontal {
            components: vec![
                (comp1.clone(), Constraint::Percentage(30)),
                (comp2.clone(), Constraint::Percentage(70)),
            ],
        });

        let area = Rect {
            x: 0,
            y: 0,
            width: 100,
            height: 50,
        };
        let layout = manager.calculate_layout(area);

        assert_eq!(layout.len(), 2);
        assert!(layout.contains_key(&comp1));
        assert!(layout.contains_key(&comp2));
    }

    #[test]
    fn test_grid_layout() {
        let mut manager = LayoutManager::new();
        let comp1 = ComponentId::new("top-left");
        let comp2 = ComponentId::new("top-right");
        let comp3 = ComponentId::new("bottom-left");
        let comp4 = ComponentId::new("bottom-right");

        manager.set_layout(LayoutConfig::Grid {
            rows: 2,
            cols: 2,
            components: vec![
                (comp1.clone(), 0, 0),
                (comp2.clone(), 0, 1),
                (comp3.clone(), 1, 0),
                (comp4.clone(), 1, 1),
            ],
        });

        let area = Rect {
            x: 0,
            y: 0,
            width: 100,
            height: 100,
        };
        let layout = manager.calculate_layout(area);

        assert_eq!(layout.len(), 4);
        assert!(layout.contains_key(&comp1));
        assert!(layout.contains_key(&comp2));
        assert!(layout.contains_key(&comp3));
        assert!(layout.contains_key(&comp4));
    }
}
