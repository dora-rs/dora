/// Dashboard-specific components (Issue #24)
pub mod system_overview;
pub mod dataflow_summary;
pub mod performance_charts;
pub mod recent_activity;

pub use system_overview::SystemOverviewComponent;
pub use dataflow_summary::DataflowSummaryComponent;
pub use performance_charts::PerformanceChartsComponent;
pub use recent_activity::RecentActivityComponent;
