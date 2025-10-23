use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use lru::LruCache;
use std::num::NonZeroUsize;
use sysinfo::{System, SystemExt, CpuExt, NetworkExt, DiskExt};
use tokio::sync::RwLock;
use std::sync::Arc;
use eyre::Result;
use rand::Rng;

use crate::cli::daemon_client::DaemonClient;

/// Resource monitoring trait for different types of resources
#[async_trait]
pub trait ResourceMonitor: Send + Sync {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>>;
    fn monitor_type(&self) -> ResourceType;
    fn collection_interval(&self) -> Duration;
}

/// Types of resources that can be monitored
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum ResourceType {
    System,
    Dataflow,
    Node,
    Network,
    Custom(String),
}

/// Value types for metrics
#[derive(Debug, Clone)]
pub enum MetricValue {
    Integer(i64),
    Float(f32),
    Boolean(bool),
    String(String),
    TimeSeries(Vec<(DateTime<Utc>, f32)>),
}

impl MetricValue {
    pub fn as_float(&self) -> f32 {
        match self {
            MetricValue::Integer(i) => *i as f32,
            MetricValue::Float(f) => *f,
            MetricValue::Boolean(b) => if *b { 1.0 } else { 0.0 },
            MetricValue::String(_) => 0.0,
            MetricValue::TimeSeries(data) => {
                if let Some((_, latest)) = data.last() {
                    *latest
                } else {
                    0.0
                }
            }
        }
    }
    
    pub fn as_i64(&self) -> i64 {
        match self {
            MetricValue::Integer(i) => *i,
            MetricValue::Float(f) => *f as i64,
            MetricValue::Boolean(b) => if *b { 1 } else { 0 },
            MetricValue::String(_) => 0,
            MetricValue::TimeSeries(data) => {
                if let Some((_, latest)) = data.last() {
                    *latest as i64
                } else {
                    0
                }
            }
        }
    }
}

/// Dataflow-specific metrics
#[derive(Debug, Clone)]
pub struct DataflowMetrics {
    pub cpu_usage_percent: f32,
    pub memory_usage_bytes: u64,
    pub network_bytes_in_per_sec: f32,
    pub network_bytes_out_per_sec: f32,
    pub messages_per_second: f32,
    pub latency_p95_ms: f32,
    pub error_rate: f32,
    pub overall_node_health: f32,
    pub active_nodes: usize,
    pub total_messages_processed: u64,
    pub timestamp: DateTime<Utc>,
}

/// System-level resource monitor
#[derive(Debug)]
pub struct SystemResourceMonitor {
    system_info: System,
    last_collection: Option<Instant>,
    network_baseline: HashMap<String, (u64, u64)>,
}

impl SystemResourceMonitor {
    pub fn new() -> Self {
        Self {
            system_info: System::new_all(),
            last_collection: None,
            network_baseline: HashMap::new(),
        }
    }
}

#[async_trait]
impl ResourceMonitor for SystemResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        let mut metrics = HashMap::new();
        
        // Refresh system information
        self.system_info.refresh_all();
        
        // CPU metrics
        let cpu_usage = self.system_info.global_cpu_info().cpu_usage();
        metrics.insert("system.cpu_usage".to_string(), MetricValue::Float(cpu_usage));
        
        // Per-core CPU usage
        for (i, cpu) in self.system_info.cpus().iter().enumerate() {
            metrics.insert(
                format!("system.cpu_core_{}.usage", i),
                MetricValue::Float(cpu.cpu_usage()),
            );
        }
        
        // Memory metrics
        let total_memory = self.system_info.total_memory();
        let used_memory = self.system_info.used_memory();
        let free_memory = self.system_info.free_memory();
        let available_memory = self.system_info.available_memory();
        let memory_usage_percent = (used_memory as f32 / total_memory as f32) * 100.0;
        
        metrics.insert("system.memory_total".to_string(), MetricValue::Integer(total_memory as i64));
        metrics.insert("system.memory_used".to_string(), MetricValue::Integer(used_memory as i64));
        metrics.insert("system.memory_free".to_string(), MetricValue::Integer(free_memory as i64));
        metrics.insert("system.memory_available".to_string(), MetricValue::Integer(available_memory as i64));
        metrics.insert("system.memory_usage_percent".to_string(), MetricValue::Float(memory_usage_percent));
        
        // Swap metrics
        let total_swap = self.system_info.total_swap();
        let used_swap = self.system_info.used_swap();
        let swap_usage_percent = if total_swap > 0 {
            (used_swap as f32 / total_swap as f32) * 100.0
        } else {
            0.0
        };
        
        metrics.insert("system.swap_total".to_string(), MetricValue::Integer(total_swap as i64));
        metrics.insert("system.swap_used".to_string(), MetricValue::Integer(used_swap as i64));
        metrics.insert("system.swap_usage_percent".to_string(), MetricValue::Float(swap_usage_percent));
        
        // Disk metrics
        for disk in self.system_info.disks() {
            let disk_name = disk.name().to_string_lossy().replace("/", "_");
            let total_space = disk.total_space();
            let available_space = disk.available_space();
            let used_space = total_space - available_space;
            let usage_percent = if total_space > 0 {
                (used_space as f32 / total_space as f32) * 100.0
            } else {
                0.0
            };
            
            metrics.insert(
                format!("system.disk.{}.total_space", disk_name),
                MetricValue::Integer(total_space as i64),
            );
            metrics.insert(
                format!("system.disk.{}.used_space", disk_name),
                MetricValue::Integer(used_space as i64),
            );
            metrics.insert(
                format!("system.disk.{}.available_space", disk_name),
                MetricValue::Integer(available_space as i64),
            );
            metrics.insert(
                format!("system.disk.{}.usage_percent", disk_name),
                MetricValue::Float(usage_percent),
            );
        }
        
        // Network metrics
        let current_time = Instant::now();
        for (interface_name, network) in self.system_info.networks() {
            let received = network.received();
            let transmitted = network.transmitted();
            
            // Calculate rates if we have baseline data
            if let Some((prev_rx, prev_tx)) = self.network_baseline.get(interface_name) {
                if let Some(last_time) = self.last_collection {
                    let time_diff = current_time.duration_since(last_time).as_secs_f32();
                    if time_diff > 0.0 {
                        let rx_rate = (received - prev_rx) as f32 / time_diff;
                        let tx_rate = (transmitted - prev_tx) as f32 / time_diff;
                        
                        metrics.insert(
                            format!("system.network.{}.bytes_received_per_sec", interface_name),
                            MetricValue::Float(rx_rate),
                        );
                        metrics.insert(
                            format!("system.network.{}.bytes_transmitted_per_sec", interface_name),
                            MetricValue::Float(tx_rate),
                        );
                    }
                }
            }
            
            // Update baseline
            self.network_baseline.insert(interface_name.clone(), (received, transmitted));
            
            metrics.insert(
                format!("system.network.{}.bytes_received", interface_name),
                MetricValue::Integer(received as i64),
            );
            metrics.insert(
                format!("system.network.{}.bytes_transmitted", interface_name),
                MetricValue::Integer(transmitted as i64),
            );
        }
        
        // Process count
        let process_count = self.system_info.processes().len();
        metrics.insert("system.process_count".to_string(), MetricValue::Integer(process_count as i64));
        
        // Load average (on Unix systems)
        let load_avg = self.system_info.load_average();
        metrics.insert("system.load_1min".to_string(), MetricValue::Float(load_avg.one));
        metrics.insert("system.load_5min".to_string(), MetricValue::Float(load_avg.five));
        metrics.insert("system.load_15min".to_string(), MetricValue::Float(load_avg.fifteen));
        
        // Boot time and uptime
        let boot_time = self.system_info.boot_time();
        let uptime = self.system_info.uptime();
        metrics.insert("system.boot_time".to_string(), MetricValue::Integer(boot_time as i64));
        metrics.insert("system.uptime_seconds".to_string(), MetricValue::Integer(uptime as i64));
        
        self.last_collection = Some(current_time);
        Ok(metrics)
    }
    
    fn monitor_type(&self) -> ResourceType {
        ResourceType::System
    }
    
    fn collection_interval(&self) -> Duration {
        Duration::from_secs(10) // Collect every 10 seconds
    }
}

/// Dataflow-specific resource monitor
#[derive(Debug)]
pub struct DataflowResourceMonitor {
    daemon_client: Arc<RwLock<DaemonClient>>,
    monitored_dataflows: HashSet<String>,
    collection_cache: LruCache<String, DataflowMetrics>,
    last_collection: Option<Instant>,
}

impl DataflowResourceMonitor {
    pub fn new(daemon_client: DaemonClient) -> Self {
        Self {
            daemon_client: Arc::new(RwLock::new(daemon_client)),
            monitored_dataflows: HashSet::new(),
            collection_cache: LruCache::new(NonZeroUsize::new(100).unwrap()),
            last_collection: None,
        }
    }
    
    pub fn add_monitored_dataflow(&mut self, name: String) {
        self.monitored_dataflows.insert(name);
    }
    
    pub fn remove_monitored_dataflow(&mut self, name: &str) {
        self.monitored_dataflows.remove(name);
    }
    
    async fn collect_dataflow_metrics(&mut self, dataflow_name: &str) -> Result<DataflowMetrics> {
        // Check cache first
        if let Some(cached) = self.collection_cache.get(dataflow_name) {
            if let Some(last_time) = self.last_collection {
                if last_time.elapsed() < Duration::from_secs(5) {
                    return Ok(cached.clone());
                }
            }
        }
        
        // Collect fresh metrics
        let daemon_client = self.daemon_client.read().await;
        
        // In a real implementation, these would be actual daemon API calls
        // For now, we'll simulate realistic metrics
        let metrics = DataflowMetrics {
            cpu_usage_percent: self.simulate_cpu_usage(),
            memory_usage_bytes: self.simulate_memory_usage(),
            network_bytes_in_per_sec: self.simulate_network_in(),
            network_bytes_out_per_sec: self.simulate_network_out(),
            messages_per_second: self.simulate_message_rate(),
            latency_p95_ms: self.simulate_latency(),
            error_rate: self.simulate_error_rate(),
            overall_node_health: self.simulate_node_health(),
            active_nodes: self.simulate_active_nodes(),
            total_messages_processed: self.simulate_total_messages(),
            timestamp: Utc::now(),
        };
        
        // Update cache
        self.collection_cache.put(dataflow_name.to_string(), metrics.clone());
        
        Ok(metrics)
    }
    
    // Simulation methods - in a real implementation, these would query actual dataflow state
    fn simulate_cpu_usage(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(5.0..45.0) // 5-45% CPU usage
    }
    
    fn simulate_memory_usage(&self) -> u64 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(50_000_000..500_000_000) // 50MB to 500MB
    }
    
    fn simulate_network_in(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(1000.0..50000.0) // 1KB/s to 50KB/s
    }
    
    fn simulate_network_out(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(1000.0..50000.0) // 1KB/s to 50KB/s
    }
    
    fn simulate_message_rate(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(10.0..1000.0) // 10-1000 messages/sec
    }
    
    fn simulate_latency(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(1.0..50.0) // 1-50ms latency
    }
    
    fn simulate_error_rate(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(0.0..0.05) // 0-5% error rate
    }
    
    fn simulate_node_health(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(0.8..1.0) // 80-100% health
    }
    
    fn simulate_active_nodes(&self) -> usize {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(1..10) // 1-10 active nodes
    }
    
    fn simulate_total_messages(&self) -> u64 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(10000..1000000) // 10K-1M total messages
    }
}

#[async_trait]
impl ResourceMonitor for DataflowResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        let mut metrics = HashMap::new();
        
        // If no specific dataflows are monitored, get all active dataflows
        let dataflows_to_monitor = if self.monitored_dataflows.is_empty() {
            // In a real implementation, this would query the daemon for active dataflows
            vec!["example_dataflow".to_string(), "test_dataflow".to_string()]
        } else {
            self.monitored_dataflows.iter().cloned().collect()
        };
        
        for dataflow_name in dataflows_to_monitor {
            let dataflow_metrics = self.collect_dataflow_metrics(&dataflow_name).await?;
            
            // CPU metrics
            metrics.insert(
                format!("dataflow.{}.cpu_usage", dataflow_name),
                MetricValue::Float(dataflow_metrics.cpu_usage_percent),
            );
            
            // Memory metrics
            metrics.insert(
                format!("dataflow.{}.memory_usage", dataflow_name),
                MetricValue::Integer(dataflow_metrics.memory_usage_bytes as i64),
            );
            
            // Network metrics
            metrics.insert(
                format!("dataflow.{}.network_in", dataflow_name),
                MetricValue::Float(dataflow_metrics.network_bytes_in_per_sec),
            );
            
            metrics.insert(
                format!("dataflow.{}.network_out", dataflow_name),
                MetricValue::Float(dataflow_metrics.network_bytes_out_per_sec),
            );
            
            // Performance metrics
            metrics.insert(
                format!("dataflow.{}.throughput", dataflow_name),
                MetricValue::Float(dataflow_metrics.messages_per_second),
            );
            
            metrics.insert(
                format!("dataflow.{}.latency_p95", dataflow_name),
                MetricValue::Float(dataflow_metrics.latency_p95_ms),
            );
            
            // Health and error metrics
            metrics.insert(
                format!("dataflow.{}.error_rate", dataflow_name),
                MetricValue::Float(dataflow_metrics.error_rate),
            );
            
            metrics.insert(
                format!("dataflow.{}.node_health", dataflow_name),
                MetricValue::Float(dataflow_metrics.overall_node_health),
            );
            
            // Node metrics
            metrics.insert(
                format!("dataflow.{}.active_nodes", dataflow_name),
                MetricValue::Integer(dataflow_metrics.active_nodes as i64),
            );
            
            metrics.insert(
                format!("dataflow.{}.total_messages", dataflow_name),
                MetricValue::Integer(dataflow_metrics.total_messages_processed as i64),
            );
        }
        
        self.last_collection = Some(Instant::now());
        Ok(metrics)
    }
    
    fn monitor_type(&self) -> ResourceType {
        ResourceType::Dataflow
    }
    
    fn collection_interval(&self) -> Duration {
        Duration::from_secs(5) // Collect every 5 seconds
    }
}

/// Node-specific resource monitor
#[derive(Debug)]
pub struct NodeResourceMonitor {
    daemon_client: Arc<RwLock<DaemonClient>>,
    monitored_nodes: HashMap<String, String>, // node_id -> dataflow_name
    collection_cache: LruCache<String, NodeMetrics>,
    last_collection: Option<Instant>,
}

#[derive(Debug, Clone)]
pub struct NodeMetrics {
    pub cpu_usage_percent: f32,
    pub memory_usage_bytes: u64,
    pub message_processing_rate: f32,
    pub input_queue_size: usize,
    pub output_queue_size: usize,
    pub error_count: u64,
    pub health_status: NodeHealthStatus,
    pub last_heartbeat: DateTime<Utc>,
    pub processing_latency_ms: f32,
    pub timestamp: DateTime<Utc>,
}

#[derive(Debug, Clone)]
pub enum NodeHealthStatus {
    Healthy,
    Warning,
    Critical,
    Offline,
}

impl NodeResourceMonitor {
    pub fn new(daemon_client: DaemonClient) -> Self {
        Self {
            daemon_client: Arc::new(RwLock::new(daemon_client)),
            monitored_nodes: HashMap::new(),
            collection_cache: LruCache::new(NonZeroUsize::new(200).unwrap()),
            last_collection: None,
        }
    }
    
    pub fn add_monitored_node(&mut self, node_id: String, dataflow_name: String) {
        self.monitored_nodes.insert(node_id, dataflow_name);
    }
    
    pub fn remove_monitored_node(&mut self, node_id: &str) {
        self.monitored_nodes.remove(node_id);
    }
    
    async fn collect_node_metrics(&mut self, node_id: &str, dataflow_name: &str) -> Result<NodeMetrics> {
        // Check cache first
        let cache_key = format!("{}:{}", dataflow_name, node_id);
        if let Some(cached) = self.collection_cache.get(&cache_key) {
            if let Some(last_time) = self.last_collection {
                if last_time.elapsed() < Duration::from_secs(3) {
                    return Ok(cached.clone());
                }
            }
        }
        
        // Simulate node metrics collection
        let metrics = NodeMetrics {
            cpu_usage_percent: self.simulate_node_cpu(),
            memory_usage_bytes: self.simulate_node_memory(),
            message_processing_rate: self.simulate_processing_rate(),
            input_queue_size: self.simulate_queue_size(),
            output_queue_size: self.simulate_queue_size(),
            error_count: self.simulate_error_count(),
            health_status: self.simulate_health_status(),
            last_heartbeat: Utc::now() - chrono::Duration::seconds(rand::thread_rng().gen_range(1..30)),
            processing_latency_ms: self.simulate_processing_latency(),
            timestamp: Utc::now(),
        };
        
        // Update cache
        self.collection_cache.put(cache_key, metrics.clone());
        
        Ok(metrics)
    }
    
    // Simulation methods for node metrics
    fn simulate_node_cpu(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(2.0..30.0) // 2-30% CPU usage per node
    }
    
    fn simulate_node_memory(&self) -> u64 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(10_000_000..100_000_000) // 10MB to 100MB per node
    }
    
    fn simulate_processing_rate(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(1.0..200.0) // 1-200 messages/sec per node
    }
    
    fn simulate_queue_size(&self) -> usize {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(0..100) // 0-100 messages in queue
    }
    
    fn simulate_error_count(&self) -> u64 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(0..10) // 0-10 errors
    }
    
    fn simulate_health_status(&self) -> NodeHealthStatus {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..100) {
            0..=85 => NodeHealthStatus::Healthy,
            86..=95 => NodeHealthStatus::Warning,
            96..=98 => NodeHealthStatus::Critical,
            _ => NodeHealthStatus::Offline,
        }
    }
    
    fn simulate_processing_latency(&self) -> f32 {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen_range(0.5..10.0) // 0.5-10ms processing latency
    }
}

#[async_trait]
impl ResourceMonitor for NodeResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        let mut metrics = HashMap::new();
        
        for (node_id, dataflow_name) in self.monitored_nodes.clone() {
            let node_metrics = self.collect_node_metrics(&node_id, &dataflow_name).await?;
            
            let prefix = format!("node.{}.{}", dataflow_name, node_id);
            
            // Resource metrics
            metrics.insert(
                format!("{}.cpu_usage", prefix),
                MetricValue::Float(node_metrics.cpu_usage_percent),
            );
            
            metrics.insert(
                format!("{}.memory_usage", prefix),
                MetricValue::Integer(node_metrics.memory_usage_bytes as i64),
            );
            
            // Performance metrics
            metrics.insert(
                format!("{}.processing_rate", prefix),
                MetricValue::Float(node_metrics.message_processing_rate),
            );
            
            metrics.insert(
                format!("{}.processing_latency", prefix),
                MetricValue::Float(node_metrics.processing_latency_ms),
            );
            
            // Queue metrics
            metrics.insert(
                format!("{}.input_queue_size", prefix),
                MetricValue::Integer(node_metrics.input_queue_size as i64),
            );
            
            metrics.insert(
                format!("{}.output_queue_size", prefix),
                MetricValue::Integer(node_metrics.output_queue_size as i64),
            );
            
            // Health metrics
            metrics.insert(
                format!("{}.error_count", prefix),
                MetricValue::Integer(node_metrics.error_count as i64),
            );
            
            let health_score = match node_metrics.health_status {
                NodeHealthStatus::Healthy => 1.0,
                NodeHealthStatus::Warning => 0.7,
                NodeHealthStatus::Critical => 0.3,
                NodeHealthStatus::Offline => 0.0,
            };
            
            metrics.insert(
                format!("{}.health_score", prefix),
                MetricValue::Float(health_score),
            );
            
            // Heartbeat metrics
            let heartbeat_age = (Utc::now() - node_metrics.last_heartbeat).num_seconds();
            metrics.insert(
                format!("{}.heartbeat_age_seconds", prefix),
                MetricValue::Integer(heartbeat_age),
            );
        }
        
        self.last_collection = Some(Instant::now());
        Ok(metrics)
    }
    
    fn monitor_type(&self) -> ResourceType {
        ResourceType::Node
    }
    
    fn collection_interval(&self) -> Duration {
        Duration::from_secs(3) // Collect every 3 seconds for more granular node monitoring
    }
}

/// Custom resource monitor for extensibility
#[derive(Debug)]
pub struct CustomResourceMonitor {
    name: String,
    resource_type: ResourceType,
    collection_fn: Box<dyn Fn() -> HashMap<String, MetricValue> + Send + Sync>,
    interval: Duration,
}

impl CustomResourceMonitor {
    pub fn new<F>(
        name: String,
        collection_fn: F,
        interval: Duration,
    ) -> Self
    where
        F: Fn() -> HashMap<String, MetricValue> + Send + Sync + 'static,
    {
        Self {
            resource_type: ResourceType::Custom(name.clone()),
            name,
            collection_fn: Box::new(collection_fn),
            interval,
        }
    }
}

#[async_trait]
impl ResourceMonitor for CustomResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        Ok((self.collection_fn)())
    }
    
    fn monitor_type(&self) -> ResourceType {
        self.resource_type.clone()
    }
    
    fn collection_interval(&self) -> Duration {
        self.interval
    }
}

/// Monitor factory for creating different types of monitors
pub struct MonitorFactory;

impl MonitorFactory {
    pub fn create_system_monitor() -> Box<dyn ResourceMonitor> {
        Box::new(SystemResourceMonitor::new())
    }
    
    pub fn create_dataflow_monitor(daemon_client: DaemonClient) -> Box<dyn ResourceMonitor> {
        Box::new(DataflowResourceMonitor::new(daemon_client))
    }
    
    pub fn create_node_monitor(daemon_client: DaemonClient) -> Box<dyn ResourceMonitor> {
        Box::new(NodeResourceMonitor::new(daemon_client))
    }
    
    pub fn create_custom_monitor<F>(
        name: String,
        collection_fn: F,
        interval: Duration,
    ) -> Box<dyn ResourceMonitor>
    where
        F: Fn() -> HashMap<String, MetricValue> + Send + Sync + 'static,
    {
        Box::new(CustomResourceMonitor::new(name, collection_fn, interval))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::timeout;
    
    #[tokio::test]
    async fn test_system_monitor_collection() {
        let mut monitor = SystemResourceMonitor::new();
        
        let result = timeout(Duration::from_secs(5), monitor.collect_metrics()).await;
        assert!(result.is_ok());
        
        let metrics = result.unwrap().unwrap();
        
        // Check that basic system metrics are present
        assert!(metrics.contains_key("system.cpu_usage"));
        assert!(metrics.contains_key("system.memory_total"));
        assert!(metrics.contains_key("system.memory_used"));
        assert!(metrics.contains_key("system.memory_usage_percent"));
        
        // Validate metric types
        if let Some(MetricValue::Float(cpu)) = metrics.get("system.cpu_usage") {
            assert!(*cpu >= 0.0 && *cpu <= 100.0);
        } else {
            panic!("CPU usage metric should be a float");
        }
    }
    
    #[test]
    fn test_metric_value_conversions() {
        let int_metric = MetricValue::Integer(42);
        assert_eq!(int_metric.as_float(), 42.0);
        assert_eq!(int_metric.as_i64(), 42);
        
        let float_metric = MetricValue::Float(3.14);
        assert_eq!(float_metric.as_float(), 3.14);
        assert_eq!(float_metric.as_i64(), 3);
        
        let bool_metric = MetricValue::Boolean(true);
        assert_eq!(bool_metric.as_float(), 1.0);
        assert_eq!(bool_metric.as_i64(), 1);
    }
    
    #[test]
    fn test_monitor_types() {
        let system_monitor = SystemResourceMonitor::new();
        assert_eq!(system_monitor.monitor_type(), ResourceType::System);
        assert_eq!(system_monitor.collection_interval(), Duration::from_secs(10));
    }
}