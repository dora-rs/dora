use std::{io::stdout, path::Path, time::Duration};

use arrow::array::{Array, UInt8Array};
use colored::Colorize;
use dora_core::{metadata::ArrowTypeInfoExt, uhlc::HLC};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};
use eyre::{Context, ContextCompat};

use crate::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    daemon_connection::json_to_arrow::read_json_bytes_as_arrow,
    event_stream::data_to_arrow_array,
};

#[derive(Default)]
pub struct InteractiveEvents {
    stopped: bool,
}

impl InteractiveEvents {
    pub fn request(
        &mut self,
        request: &Timestamped<DaemonRequest>,
    ) -> Result<DaemonReply, eyre::Error> {
        let reply = match &request.inner {
            DaemonRequest::Register(_) => DaemonReply::Result(Ok(())),
            DaemonRequest::Subscribe => DaemonReply::Result(Ok(())),
            DaemonRequest::NextEvent => {
                let events = if let Some(event) = self.next_event()? {
                    let event = Timestamped {
                        inner: event,
                        timestamp: HLC::default().new_timestamp(),
                    };
                    vec![event]
                } else {
                    vec![]
                };
                DaemonReply::NextEvents(events)
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                let array = data_to_arrow_array(data.clone(), metadata);

                let array_display = match array {
                    Err(err) => format!("<error>: {err:?}"),
                    Ok(data) => format!("{data:?}"),
                };
                println!(
                    "{} {} with data: {array_display}",
                    "node sends output".yellow(),
                    output_id.bright_blue().bold()
                );
                DaemonReply::Empty
            }
            DaemonRequest::CloseOutputs(data_ids) => {
                println!("{} {data_ids:?}", "node reports closed outputs".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::OutputsDone => {
                println!("{}", "node reports OutputsDone".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::EventStreamDropped => {
                println!("{}", "node reports EventStreamDropped".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::NodeConfig { .. } => {
                eyre::bail!("unexpected NodeConfig in interactive mode")
            }
            DaemonRequest::RegisterDirectListener { .. } => DaemonReply::Result(Ok(())),
            DaemonRequest::QueryDirectRoutes => DaemonReply::DirectRoutes(vec![]),
        };
        Ok(reply)
    }

    fn next_event(&mut self) -> eyre::Result<Option<NodeEvent>> {
        std::thread::sleep(Duration::from_millis(500));
        println!("{}", "Node asks for next input".blue());
        if self.stopped {
            println!(
                "{}",
                "event channel was stopped -> returning empty event list".green()
            );
            return Ok(None);
        }
        let stdout_lock = stdout().lock();
        let id = inquire::Text::new("Input ID")
            .with_help_message("empty input ID to stop")
            .prompt()?;
        std::mem::drop(stdout_lock);
        let event = if id.is_empty() {
            println!("{}", "given input ID is empty -> stopping".blue());
            self.stopped = true;
            NodeEvent::Stop
        } else {
            let id = id.into();
            let (data, type_info) = loop {
                let stdout_lock = stdout().lock();
                let data = inquire::Text::new("Data")
                    .with_help_message(
                        "String/JSON, FILE:<path.arrow>, HEX:<hexbytes>, or esc to skip",
                    )
                    .prompt_skippable()?;
                std::mem::drop(stdout_lock);
                let typed_data = if let Some(data) = data {
                    let array_data = if let Some(path) = data.strip_prefix("FILE:") {
                        match read_arrow_ipc_file(Path::new(path.trim())) {
                            Ok(d) => d,
                            Err(err) => {
                                eprintln!("{}", format!("{err}").red());
                                continue;
                            }
                        }
                    } else if let Some(hex) = data.strip_prefix("HEX:") {
                        match decode_hex_as_uint8_array(hex.trim()) {
                            Ok(d) => d,
                            Err(err) => {
                                eprintln!("{}", format!("{err}").red());
                                continue;
                            }
                        }
                    } else {
                        // JSON or plain text
                        match read_json_bytes_as_arrow(data.as_bytes()) {
                            Ok(d) => d,
                            Err(err) => {
                                eprintln!("{}", format!("{err}").red());
                                continue;
                            }
                        }
                    };

                    let total_len = required_data_size(&array_data);
                    let mut buf = vec![0; total_len];
                    let type_info = copy_array_into_sample(buf.as_mut_slice(), &array_data);

                    (Some(buf), type_info)
                } else {
                    (None, ArrowTypeInfo::empty())
                };
                break typed_data;
            };

            NodeEvent::Input {
                id,
                metadata: std::sync::Arc::new(Metadata::new(
                    HLC::default().new_timestamp(),
                    type_info,
                )),
                data: data.map(|d| {
                    std::sync::Arc::new(DataMessage::Vec(aligned_vec::AVec::from_slice(1, &d)))
                }),
            }
        };
        Ok(Some(event))
    }
}

/// Read the first batch from an Arrow IPC file and return it as `ArrayData`.
///
/// This opens arbitrary user-provided file paths. It is intended for
/// interactive (REPL) testing only and should not be used in
/// non-interactive contexts without path validation.
fn read_arrow_ipc_file(path: &Path) -> eyre::Result<arrow::array::ArrayData> {
    eyre::ensure!(!path.as_os_str().is_empty(), "FILE: path is empty");
    let file =
        std::fs::File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
    let mut reader = arrow::ipc::reader::FileReader::try_new(file, None)
        .context("failed to read Arrow IPC file")?;
    if reader.num_batches() > 1 {
        tracing::warn!(
            "Arrow IPC file {} has {} batches, only the first will be used",
            path.display(),
            reader.num_batches()
        );
    }
    let batch = reader
        .next()
        .context("Arrow IPC file has no batches")?
        .context("failed to read batch from Arrow IPC file")?;
    eyre::ensure!(
        batch.num_columns() == 1,
        "Arrow IPC file must have exactly 1 column, got {}",
        batch.num_columns()
    );
    Ok(batch.column(0).to_data())
}

/// Decode a hex string into a `UInt8Array`.
fn decode_hex_as_uint8_array(hex: &str) -> eyre::Result<arrow::array::ArrayData> {
    eyre::ensure!(!hex.is_empty(), "HEX: input is empty");
    eyre::ensure!(hex.is_ascii(), "HEX: input contains non-ASCII characters");
    eyre::ensure!(
        hex.len().is_multiple_of(2),
        "HEX: odd number of characters ({}), hex bytes must come in pairs",
        hex.len()
    );
    let bytes: Vec<u8> = hex
        .as_bytes()
        .chunks(2)
        .enumerate()
        .map(|(i, chunk)| {
            let s = std::str::from_utf8(chunk)
                .with_context(|| format!("non-ASCII at byte {}", i * 2))?;
            u8::from_str_radix(s, 16).with_context(|| format!("invalid hex at position {}", i * 2))
        })
        .collect::<eyre::Result<Vec<u8>>>()?;
    let array = UInt8Array::from(bytes);
    Ok(array.to_data())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::ArrayRef;

    #[test]
    fn test_decode_hex_valid() {
        let data = decode_hex_as_uint8_array("deadbeef").unwrap();
        let arr = arrow::array::make_array(data);
        let u8arr = arr.as_any().downcast_ref::<UInt8Array>().unwrap();
        assert_eq!(u8arr.values().as_ref(), &[0xde, 0xad, 0xbe, 0xef]);
    }

    #[test]
    fn test_decode_hex_empty() {
        assert!(decode_hex_as_uint8_array("").is_err());
    }

    #[test]
    fn test_decode_hex_odd_length() {
        assert!(decode_hex_as_uint8_array("abc").is_err());
    }

    #[test]
    fn test_decode_hex_invalid() {
        assert!(decode_hex_as_uint8_array("zz").is_err());
    }

    #[test]
    fn test_read_arrow_ipc_file() {
        use arrow::array::Int32Array;
        use arrow::datatypes::{DataType, Field, Schema};
        use arrow::ipc::writer::FileWriter;
        use arrow::record_batch::RecordBatch;
        use std::sync::Arc;

        let tmpfile = std::env::temp_dir().join("test_interactive.arrow");

        let schema = Arc::new(Schema::new(vec![Field::new("col", DataType::Int32, false)]));
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![Arc::new(Int32Array::from(vec![1, 2, 3])) as ArrayRef],
        )
        .unwrap();

        let file = std::fs::File::create(&tmpfile).unwrap();
        let mut writer = FileWriter::try_new(file, &schema).unwrap();
        writer.write(&batch).unwrap();
        writer.finish().unwrap();

        let data = read_arrow_ipc_file(&tmpfile).unwrap();
        let arr = arrow::array::make_array(data);
        let i32arr = arr.as_any().downcast_ref::<Int32Array>().unwrap();
        assert_eq!(i32arr.values().as_ref(), &[1, 2, 3]);

        std::fs::remove_file(&tmpfile).ok();
    }

    #[test]
    fn test_read_arrow_ipc_file_multi_column_rejected() {
        use arrow::array::Int32Array;
        use arrow::datatypes::{DataType, Field, Schema};
        use arrow::ipc::writer::FileWriter;
        use arrow::record_batch::RecordBatch;
        use std::sync::Arc;

        let tmpfile = std::env::temp_dir().join("test_interactive_multi.arrow");

        let schema = Arc::new(Schema::new(vec![
            Field::new("a", DataType::Int32, false),
            Field::new("b", DataType::Int32, false),
        ]));
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![
                Arc::new(Int32Array::from(vec![1, 2])) as ArrayRef,
                Arc::new(Int32Array::from(vec![3, 4])) as ArrayRef,
            ],
        )
        .unwrap();

        let file = std::fs::File::create(&tmpfile).unwrap();
        let mut writer = FileWriter::try_new(file, &schema).unwrap();
        writer.write(&batch).unwrap();
        writer.finish().unwrap();

        let err = read_arrow_ipc_file(&tmpfile).unwrap_err();
        assert!(err.to_string().contains("exactly 1 column"));

        std::fs::remove_file(&tmpfile).ok();
    }

    #[test]
    fn test_read_arrow_ipc_file_empty_path() {
        assert!(read_arrow_ipc_file(Path::new("")).is_err());
    }

    #[test]
    fn test_decode_hex_utf8_input() {
        // Multi-byte UTF-8 should be rejected by the ASCII check
        assert!(decode_hex_as_uint8_array("\u{00e9}\u{00e9}").is_err());
    }
}
