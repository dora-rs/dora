use std::{
    path::{Path, PathBuf},
    sync::Mutex,
};

use crate::{
    CoordinatorClient, DataflowSummary, InterfaceError, LegacyCliService, PreferencesStore,
    SystemMetrics, TelemetryService, UserPreferencesSnapshot,
};

pub struct MockCoordinatorClient {
    response: Mutex<Result<Vec<DataflowSummary>, InterfaceError>>,
}

impl Default for MockCoordinatorClient {
    fn default() -> Self {
        Self {
            response: Mutex::new(Ok(Vec::new())),
        }
    }
}

impl MockCoordinatorClient {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_response(&self, response: Result<Vec<DataflowSummary>, InterfaceError>) {
        *self.response.lock().unwrap() = response;
    }
}

impl CoordinatorClient for MockCoordinatorClient {
    fn list_dataflows(&self) -> Result<Vec<DataflowSummary>, InterfaceError> {
        self.response.lock().unwrap().clone()
    }
}

pub struct MockLegacyCliService {
    pub calls: Mutex<Vec<(Vec<String>, PathBuf)>>,
    result: Mutex<Result<(), InterfaceError>>,
}

impl Default for MockLegacyCliService {
    fn default() -> Self {
        Self {
            calls: Mutex::new(Vec::new()),
            result: Mutex::new(Ok(())),
        }
    }
}

impl MockLegacyCliService {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_result(&self, result: Result<(), InterfaceError>) {
        *self.result.lock().unwrap() = result;
    }
}

impl LegacyCliService for MockLegacyCliService {
    fn execute(&self, argv: &[String], working_dir: &Path) -> Result<(), InterfaceError> {
        self.calls
            .lock()
            .unwrap()
            .push((argv.to_vec(), working_dir.to_path_buf()));
        self.result.lock().unwrap().clone()
    }
}

pub struct MockTelemetryService {
    response: Mutex<Result<SystemMetrics, InterfaceError>>,
}

impl Default for MockTelemetryService {
    fn default() -> Self {
        Self {
            response: Mutex::new(Ok(SystemMetrics::default())),
        }
    }
}

impl MockTelemetryService {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_response(&self, response: Result<SystemMetrics, InterfaceError>) {
        *self.response.lock().unwrap() = response;
    }
}

impl TelemetryService for MockTelemetryService {
    fn latest_metrics(&self) -> Result<SystemMetrics, InterfaceError> {
        self.response.lock().unwrap().clone()
    }
}

pub struct MockPreferencesStore {
    load_result: Mutex<Result<UserPreferencesSnapshot, InterfaceError>>,
    save_result: Mutex<Result<(), InterfaceError>>,
}

impl Default for MockPreferencesStore {
    fn default() -> Self {
        Self {
            load_result: Mutex::new(Ok(UserPreferencesSnapshot {
                theme: "dark".to_string(),
                auto_refresh_interval_secs: 5,
                show_system_info: true,
                default_view: None,
            })),
            save_result: Mutex::new(Ok(())),
        }
    }
}

impl MockPreferencesStore {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_load_result(&self, result: Result<UserPreferencesSnapshot, InterfaceError>) {
        *self.load_result.lock().unwrap() = result;
    }

    pub fn set_save_result(&self, result: Result<(), InterfaceError>) {
        *self.save_result.lock().unwrap() = result;
    }
}

impl PreferencesStore for MockPreferencesStore {
    fn load(&self) -> Result<UserPreferencesSnapshot, InterfaceError> {
        self.load_result.lock().unwrap().clone()
    }

    fn save(&self, _prefs: &UserPreferencesSnapshot) -> Result<(), InterfaceError> {
        self.save_result.lock().unwrap().clone()
    }
}
