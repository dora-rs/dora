use std::time::Duration;

use tui_interface::{InterfaceError, PreferencesStore, UserPreferencesSnapshot};

/// PreferencesStore implementation backed by Dora's UserPreferences.
#[derive(Debug, Default, Clone)]
pub struct CliPreferencesStore;

impl PreferencesStore for CliPreferencesStore {
    fn load(&self) -> Result<UserPreferencesSnapshot, InterfaceError> {
        let prefs = crate::config::preferences::UserPreferences::load_or_create()
            .map_err(|err| InterfaceError::from(err.to_string()))?;

        Ok(UserPreferencesSnapshot {
            theme: prefs.interface.tui.theme.clone(),
            auto_refresh_interval_secs: prefs.interface.tui.auto_refresh_interval.as_secs(),
            show_system_info: prefs.interface.hints.show_hints,
            default_view: Some(prefs.interface.tui.default_view.clone()),
        })
    }

    fn save(&self, snapshot: &UserPreferencesSnapshot) -> Result<(), InterfaceError> {
        let mut prefs = crate::config::preferences::UserPreferences::load_or_create()
            .map_err(|err| InterfaceError::from(err.to_string()))?;

        prefs.interface.tui.theme = snapshot.theme.clone();
        prefs.interface.tui.auto_refresh_interval =
            Duration::from_secs(snapshot.auto_refresh_interval_secs.max(1));
        prefs.interface.hints.show_hints = snapshot.show_system_info;

        if let Some(default_view) = snapshot.default_view.clone() {
            prefs.interface.tui.default_view = default_view;
        }

        prefs
            .save()
            .map_err(|err| InterfaceError::from(err.to_string()))
    }
}
