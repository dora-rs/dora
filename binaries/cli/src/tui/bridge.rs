use std::{sync::Arc, time::Duration};

use tui_interface::{
    CoordinatorClient, InterfaceError, LegacyCliService, PreferencesStore, TelemetryService,
    UserPreferencesSnapshot,
};

#[cfg(feature = "tui-cli-services")]
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, query_running_dataflows},
    config::preferences::UserPreferences,
    execute_legacy_command,
    tui::{app::dataflow_from_entry, metrics::MetricsCollector},
};

#[cfg(feature = "tui-cli-services")]
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;

#[cfg(feature = "tui-cli-services")]
use tui_interface::DataflowSummary;

#[cfg(feature = "tui-cli-services")]
pub struct ServiceBundle {
    pub preferences_store: Arc<dyn PreferencesStore>,
    pub coordinator_client: Arc<dyn CoordinatorClient>,
    pub telemetry_service: Arc<dyn TelemetryService>,
    pub legacy_cli_service: Arc<dyn LegacyCliService>,
}

#[cfg(feature = "tui-cli-services")]
pub fn default_service_bundle() -> ServiceBundle {
    ServiceBundle {
        preferences_store: Arc::new(CliPreferencesStore),
        coordinator_client: Arc::new(CliCoordinatorClient),
        telemetry_service: Arc::new(CliTelemetryService::default()),
        legacy_cli_service: Arc::new(CliLegacyCliService),
    }
}

#[cfg(feature = "tui-cli-services")]
#[derive(Debug, Default, Clone)]
struct CliPreferencesStore;

#[cfg(feature = "tui-cli-services")]
impl PreferencesStore for CliPreferencesStore {
    fn load(&self) -> Result<UserPreferencesSnapshot, InterfaceError> {
        let prefs = UserPreferences::load_or_create()
            .map_err(|err| InterfaceError::from(err.to_string()))?;

        Ok(UserPreferencesSnapshot {
            theme: prefs.interface.tui.theme.clone(),
            auto_refresh_interval_secs: prefs.interface.tui.auto_refresh_interval.as_secs(),
            show_system_info: prefs.interface.hints.show_hints,
            default_view: Some(prefs.interface.tui.default_view.clone()),
        })
    }

    fn save(&self, snapshot: &UserPreferencesSnapshot) -> Result<(), InterfaceError> {
        let mut prefs = UserPreferences::load_or_create()
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

#[cfg(feature = "tui-cli-services")]
#[derive(Default, Debug)]
struct CliCoordinatorClient;

#[cfg(feature = "tui-cli-services")]
impl CoordinatorClient for CliCoordinatorClient {
    fn list_dataflows(&self) -> Result<Vec<DataflowSummary>, InterfaceError> {
        let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
        let mut session = connect_to_coordinator(coordinator_addr)
            .map_err(|err| InterfaceError::from(err.to_string()))?;
        let list = query_running_dataflows(&mut *session)
            .map_err(|err| InterfaceError::from(err.to_string()))?;

        Ok(list.0.into_iter().map(dataflow_from_entry).collect())
    }
}

#[cfg(feature = "tui-cli-services")]
#[derive(Default, Debug)]
pub struct CliTelemetryService;

#[cfg(feature = "tui-cli-services")]
impl TelemetryService for CliTelemetryService {
    fn latest_metrics(&self) -> Result<tui_interface::SystemMetrics, InterfaceError> {
        let mut collector = MetricsCollector::new();
        collector
            .collect()
            .map_err(|err| InterfaceError::from(err.to_string()))
    }
}

#[cfg(feature = "tui-cli-services")]
#[derive(Default, Debug)]
struct CliLegacyCliService;

#[cfg(feature = "tui-cli-services")]
impl LegacyCliService for CliLegacyCliService {
    fn execute(
        &self,
        argv: &[String],
        working_dir: &std::path::Path,
    ) -> Result<(), InterfaceError> {
        execute_legacy_command(argv.iter().map(|s| s.as_str()), Some(working_dir))
            .map_err(|err| InterfaceError::from(err.to_string()))
    }
}
