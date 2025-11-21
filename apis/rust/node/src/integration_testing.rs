#![allow(clippy::test_attr_in_doctest)]

//! This document describes how to write [integration tests] for Dora nodes.
//!
//! [integration tests]: https://en.wikipedia.org/wiki/Integration_testing
//!
//! # Usage
//!
//! There are currently three ways to run integration tests for Dora nodes:
//!
//! - calling [`setup_integration_testing`] before invoking the node's `main` function
//! - testing the compiled executable by setting environment variables
//! - using [`DoraNode::init_testing`](crate::DoraNode::init_testing) tests independent
//!   of the `main` function
//!
//!
//! ## Using `setup_integration_testing`
//!
//! The most straightforward way to write integration tests for a Dora node is to call the
//! [`setup_integration_testing`] function in the test function and then invoke the node's `main`
//! function. This way, the full behavior of the node (including the `main` function) is tested.
//!
//! This approach requires that the node's `main` function uses the
//! [`DoraNode::init_from_env`](crate::DoraNode::init_from_env) function for initialization.
//!
//! See the [`setup_integration_testing`] function documentation for details.
//!
//! ### Example
//! ```rust, ignore
//! #[test]
//! fn test_main_function() -> eyre::Result<()> {
//!    // specify the events that should be sent to the node
//!    let events = vec![
//!        TimedIncomingEvent {
//!            time_offset_secs: 0.01,
//!            event: IncomingEvent::Input {
//!                id: "tick".into(),
//!                metadata: None,
//!                data: None,
//!            },
//!        },
//!        TimedIncomingEvent {
//!            time_offset_secs: 0.055,
//!            event: IncomingEvent::Stop,
//!        },
//!    ];
//!    let inputs = dora_node_api::integration_testing::TestingInput::Input(
//!        IntegrationTestInput::new("node_id".parse().unwrap(), events),
//!    );
//!
//!    // send the node's outputs to a channel so we can verify them later
//!    let (tx, rx) = flume::unbounded();
//!    let outputs = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
//!
//!    // don't include time offsets in the outputs to make them deterministic
//!    let options = TestingOptions {
//!        skip_output_time_offsets: true,
//!    };
//!
//!     // setup the integration testing environment -> this will make `DoraNode::init_from_env``
//!     // initialize the node in testing mode
//!     integration_testing::setup_integration_testing(inputs, outputs, options);
//!
//!     // call the node's main function
//!     crate::main()?;
//!
//!     // collect the nodes's outputs and compare them
//!     let outputs = rx.try_iter().collect::<Vec<_>>();
//!     assert_eq!(outputs, expected_outputs);
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Testing through Environment Variables
//!
//! Dora also supports testing nodes after compilation. This is the most comprehensive way to ensure
//! that a node behaves as expected, as the testing will be done on the exact same executable as
//! used in the dataflow.
//!
//! To enable this, the
//! [`DoraNode::init_from_env`](crate::DoraNode::init_from_env) function provides
//! built-in support for integration testing through environment variables.
//!
//! To start an integration test, run a node executable directly (i.e. don't go through the dora
//! cli) with the following environment variables set:
//!
//! - `DORA_TEST_WITH_INPUTS`: Path to a JSON file that contains the inputs that should be sent
//!   to the node.
//!   - The file format is defined through the [`IntegrationTestInput`] struct (encoded as JSON).
//! - `DORA_TEST_WRITE_OUTPUTS_TO` (optional): Path at which the output
//!   [JSONL](https://jsonltools.com/what-is-jsonl) file should be written.
//!   - defaults to a `outputs.jsonl` file next to the given inputs file
//!   - See the [`TestingOutput`] struct for details on the output file format.
//! - `DORA_TEST_NO_OUTPUT_TIME_OFFSET` (optional): If set to any value, the output JSONL file
//!   will not contain time offsets for the outputs.
//!   - This is useful to get deterministic outputs that can be compared against expected outputs.
//!     (The time offsets depend on your machine, system load, OS, etc and thus vary between runs.)
//!
//! In integration test mode, the node will not connect to any Dora daemon or other nodes. Instead,
//! it will read all incoming events from the given inputs file and write all outputs to the output
//! file. The output file can then be compared against expected outputs to verify that the node
//! behaves as intended.
//!
//! ### Input File Format
//!
//! The input file must be a JSON file that can be deserialized to a [`IntegrationTestInput`]
//! instance.
//!
//! ### Example
//!
//! ```bash
//! > DORA_TEST_WITH_INPUTS=path/to/inputs.json DORA_TEST_NO_OUTPUT_TIME_OFFSET=1 cargo r -p rust-dataflow-example-status-node
//! ```
//!
//! ## Using `DoraNode::init_testing`
//!
//! While we recommend to include the `main` function in integration tests (as described above),
//! there might also be cases where you want to run additional tests independent of the
//! `main` function. To make this easier, Dora provides a
//! [`DoraNode::init_testing`](crate::DoraNode::init_testing) initialization function to initialize
//! an integration testing environment directly.
//!
//! This function is roughly equivalent to calling `setup_integration_testing` followed by
//! `DoraNode::init_from_env`, but it does not modify any global state or thread-local state.
//! Thus, it can be even used to run multiple integration tests as part of a single test function.
//!
//! ### Example
//!
//! ```rust, ignore
//! #[test]
//! fn test_run_function() -> eyre::Result<()> {
//!     // specify the events that should be sent to the node
//!     let events = vec![...]; // see above for details
//!     let inputs = dora_node_api::integration_testing::TestingInput::Input(
//!          IntegrationTestInput::new("node_id".parse().unwrap(), events),
//!     );
//!
//!     let outputs = dora_node_api::integration_testing::TestingOutput::ToFile(
//!        std::path::PathBuf::from("path/to/outputs.jsonl"),
//!     );
//!
//!     let (node, events) = DoraNode::init_testing(inputs, outputs, Default::default())?;
//!     do_something_with_node_and_events(node, events)?;
//! }
//! ```
//!
//!
//! ## Generating Input Files
//!
//! While manually writing input files is possible, it is often more convenient to autogenerate them
//! by recording actual dataflow runs. This can be done by running a Dora dataflow with the
//! **`DORA_WRITE_EVENTS_TO`** environment variable set to a folder path. This will instruct all
//! nodes in the started dataflow to write out their received inputs to a `inputs-{node_id}.json`
//! file in the given folder.
//!
//! The file format used for these input files is identical to the format expected by the
//! `DORA_TEST_WITH_INPUTS` environment variable. Thus, the generated files can be directly used
//! as input files for integration tests.
//!
//! Note that the implementation of this feature is currently not optimized. All incoming events
//! are buffered in memory before being written out at the end of the node's execution. Thus, it is
//! not recommended to use this feature with long-running dataflows or dataflows that process
//! large amounts of data.

use std::cell::Cell;

pub use dora_message::integration_testing_format::{self, IntegrationTestInput};

thread_local! {
    static TESTING_ENV: Cell<Option<Box<TestingCommunication>>> = const { Cell::new(None) };
}

pub(crate) fn take_testing_communication() -> Option<Box<TestingCommunication>> {
    TESTING_ENV.with(|env| env.take())
}

/// Sets up an integration testing environment for the current thread.
///
/// This overrides the default behavior of
/// [`DoraNode::init_from_env`](crate::DoraNode::init_from_env) to initialize
/// a testing node with the given input, output, and options.
///
/// ## Implementation Details
///
/// This function sets up thread-local state that is read by the `DoraNode::init_from_env` function.
/// Thus, it only affects the current thread.
///
/// Calls to `DoraNode::init_from_env` will consume the testing environment set up by this function
/// and reset the thread-local state afterwards. Thus, subsequent calls to `DoraNode::init_from_env`
/// will _not_ run in testing mode unless this function is called again.
pub fn setup_integration_testing(
    input: TestingInput,
    output: TestingOutput,
    options: TestingOptions,
) {
    TESTING_ENV.with(|env| {
        env.set(Some(Box::new(TestingCommunication {
            input,
            output,
            options,
        })))
    });
}

pub(crate) struct TestingCommunication {
    pub input: TestingInput,
    pub output: TestingOutput,
    pub options: TestingOptions,
}

/// Provides input data for integration testing.
pub enum TestingInput {
    /// Loads the integration test input from the given JSON file.
    ///
    /// The given file must deserialize to an [`IntegrationTestInput`] instance.
    FromJsonFile(std::path::PathBuf),
    /// Directly provides the integration test input.
    Input(IntegrationTestInput),
}

/// Specifies where to write the output data of an integration test.
///
/// ## Output File Format
///
/// The output file is a [JSONL](https://jsonltools.com/what-is-jsonl) (_"JSON lines"_) file. Each
/// line in the file is a separate JSON object that represents one output sent by the node.
///
/// The following fields are present in each output object:
///
/// - `id` (string): The output identifier that was used when sending the output.
/// - `data`: The output data (if any), encoded as JSON.
///   - see below for details on the encoding
/// - `data_type` (string or object): The [arrow::datatypes::DataType] of the output data
///   - Serialized to JSON using the type's `serde::Serialize` implementation.
/// - `time_offset_secs` (float, optional): The time offset in seconds between the start of the
///   node and the time when the output was sent.
///   - This field is omitted if the `DORA_TEST_NO_OUTPUT_TIME_OFFSET` environment is set.
///
///
/// ## Output Data Encoding
///
/// We use the following steps to encode the `data` field of each output object:
///
/// - Convert the data to a [`RecordBatch`](arrow::array::RecordBatch) through:
///   ```rust,ignore
///   RecordBatch::try_from_iter([("inner", data)])
///   ```
/// - Convert the `RecordBatch` to a JSON object through the [`arrow_json::ArrayWriter`].
pub enum TestingOutput {
    /// Writes the output to the given JSONL file.
    ///
    /// The file will be created or overwritten.
    ToFile(std::path::PathBuf),
    /// Writes the output as JSONL file to the given writer.
    ToWriter(Box<dyn std::io::Write + Send>),
    /// Sends each output as a JSON object to the given [`flume::Receiver`].
    ///
    /// Note: When using a bounded channel, the node may block when the channel is full.
    ToChannel(flume::Sender<serde_json::Map<String, serde_json::Value>>),
}

/// Options for integration testing.
#[derive(Debug, Clone, Default)]
pub struct TestingOptions {
    /// Whether to skip including time offsets in the output.
    ///
    /// Skipping time offsets makes the outputs deterministic and easier to compare against
    /// expected outputs.
    pub skip_output_time_offsets: bool,
}
