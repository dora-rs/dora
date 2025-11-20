//! This document describes how to write [integration tests] for Dora nodes.
//!
//! [integration tests]: https://en.wikipedia.org/wiki/Integration_testing
//!
//! Node integration tests can verify that a node produces the expected outputs for a given set
//! of inputs. The [`DoraNode::init_from_env`](crate::DoraNode::init_from_env) function provides
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
//!   - The file format is documented below.
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
//! ## Input File Format
//!
//! The input file must be a JSON file that can be deserialized to a [`IntegrationTestInput`]
//! instance.
//!
//! ## Output File Format
//!
//! The output file is a [JSONL](https://jsonltools.com/what-is-jsonl) (_"JSON lines"_) file. Each
//! line in the file is a separate JSON object that represents one output sent by the node.
//!
//! The following fields are present in each output object:
//!
//! - `id` (string): The output identifier that was used when sending the output.
//! - `data`: The output data (if any), encoded as JSON.
//!   - see below for details on the encoding
//! - `data_type` (string or object): The [arrow::datatypes::DataType] of the output data
//!   - Serialized to JSON using the type's `serde::Serialize` implementation.
//! - `time_offset_secs` (float, optional): The time offset in seconds between the start of the
//!   node and the time when the output was sent.
//!   - This field is omitted if the `DORA_TEST_NO_OUTPUT_TIME_OFFSET` environment is set.
//!
//! ### Output Data Encoding
//!
//! We use the following steps to encode the `data` field of each output object:
//!
//! - Convert the data to a [`RecordBatch`](arrow::array::RecordBatch) through:
//!   ```rust,ignore
//!   RecordBatch::try_from_iter([("inner", data)])
//!   ```
//! - Convert the `RecordBatch` to a JSON object through the [`arrow_json::ArrayWriter`].
//!
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

pub use dora_message::integration_testing_format::IntegrationTestInput;
