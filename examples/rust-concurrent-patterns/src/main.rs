//! Rust Concurrent Patterns for Dora
//!
//! This example demonstrates the recommended pattern for handling concurrent operations
//! in Dora using threads and mpsc channels instead of callbacks.

use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;

/// Simulates sensor data
#[derive(Clone, Debug)]
pub struct SensorData {
    pub id: u32,
    pub value: f64,
    pub timestamp: u64,
}

/// Output from processing
#[derive(Clone, Debug)]
pub struct ProcessedOutput {
    pub input_id: u32,
    pub result: f64,
    pub processing_time_ms: u128,
}

/// Main event loop for handling concurrent operations
fn main() {
    env_logger::init();

    println!("\n=== Rust Concurrent Patterns for Dora ===");
    println!("Demonstrating thread-based concurrency with mpsc channels\n");

    // Create channels for communication
    let (output_tx, output_rx) = mpsc::channel::<ProcessedOutput>();
    let output_tx = Arc::new(Mutex::new(output_tx));

    // Spawn output handler thread
    let output_handler = thread::spawn(move || {
        output_handler_fn(output_rx);
    });

    // Simulate sensor data
    let sensor_data = vec![
        SensorData {
            id: 1,
            value: 42.5,
            timestamp: 1000,
        },
        SensorData {
            id: 2,
            value: 73.2,
            timestamp: 1001,
        },
        SensorData {
            id: 3,
            value: 91.8,
            timestamp: 1002,
        },
    ];

    // Process each sensor reading in a worker thread
    let mut handles = vec![];
    for sensor in sensor_data {
        let tx = Arc::clone(&output_tx);
        let handle = thread::spawn(move || {
            process_sensor_data(sensor, tx);
        });
        handles.push(handle);
    }

    // Wait for all processing threads to complete
    for handle in handles {
        handle.join().expect("Thread panicked");
    }

    // Drop the original sender so the receiver knows when to stop
    drop(output_tx);

    // Wait for output handler to finish
    output_handler.join().expect("Output handler panicked");

    println!("\n=== All processing complete ===");
}

/// Processes sensor data in a worker thread
fn process_sensor_data(sensor: SensorData, tx: Arc<Mutex<mpsc::Sender<ProcessedOutput>>>) {
    let start = std::time::Instant::now();

    // Simulate expensive computation
    println!("[Worker {}] Processing sensor data: {:?}", sensor.id, sensor.value);
    thread::sleep(Duration::from_millis(100 + (sensor.id as u64) * 50));

    // Perform calculation
    let result = sensor.value * 2.0 + 10.0;

    let elapsed = start.elapsed().as_millis();

    // Send result back via channel
    let output = ProcessedOutput {
        input_id: sensor.id,
        result,
        processing_time_ms: elapsed,
    };

    if let Ok(sender) = tx.lock() {
        sender.send(output).expect("Failed to send output");
    }
}

/// Handler thread that processes results as they arrive
fn output_handler_fn(rx: mpsc::Receiver<ProcessedOutput>) {
    println!("[Output Handler] Ready to receive results");

    for output in rx {
        println!(
            "[Output Handler] Received result: id={}, result={:.2}, time={}ms",
            output.input_id, output.result, output.processing_time_ms
        );
    }

    println!("[Output Handler] All results received, shutting down");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_data_creation() {
        let sensor = SensorData {
            id: 1,
            value: 42.5,
            timestamp: 1000,
        };
        assert_eq!(sensor.id, 1);
        assert_eq!(sensor.value, 42.5);
    }

    #[test]
    fn test_concurrent_processing() {
        let (tx, rx) = mpsc::channel::<ProcessedOutput>();

        let sensor = SensorData {
            id: 1,
            value: 50.0,
            timestamp: 1000,
        };

        let tx_arc = Arc::new(Mutex::new(tx));
        let sensor_clone = sensor.clone();
        let tx_clone = Arc::clone(&tx_arc);

        thread::spawn(move || {
            process_sensor_data(sensor_clone, tx_clone);
        });

        if let Ok(output) = rx.recv() {
            assert_eq!(output.input_id, 1);
            assert_eq!(output.result, 110.0); // 50.0 * 2.0 + 10.0
        }
    }
}
