//! Compare two dora recordings (`.drec`) output by output, for golden-file
//! regression tests: record a known-good run once, then check that a later
//! run still produces the same messages.
//!
//! For every `node_id/output_id` the two recordings must hold the same number
//! of messages, and the `n`-th message of one must carry the same data as the
//! `n`-th message of the other. Only the recorded Arrow payload bytes are
//! compared, exactly. Everything that changes from run to run anyway is
//! ignored: the recording's start time and dataflow id, each message's
//! timestamps, and the order in which different outputs' messages
//! interleave.
//!
//! # Limits
//!
//! - `metadata.parameters` is **not** compared. Services and actions put a
//!   fresh `request_id`/`goal_id` there on every run, so comparing it would
//!   make every such recording differ. A change that is only in the
//!   parameters (an action result's `goal_status`, an image's size) is not
//!   found.
//! - The same values encoded differently (for example by a dora version whose
//!   recorder writes the Arrow stream another way) count as different data.
//! - Only what was recorded is compared: messages the record node dropped or
//!   skipped as too large are not in the recording.
//! - A recording that was not finished cleanly (no footer) is compared as far
//!   as it goes, with a warning: it may be missing its last messages.
//! - Only a SHA-256 digest of each message is kept in memory, not the payload,
//!   so large recordings stay cheap to compare, but the report can only say
//!   which messages differ, not how.
use std::{
    collections::BTreeMap,
    fs::File,
    io::{Read, Write},
};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_recording::RecordingReader;
use eyre::{WrapErr, eyre};
use sha2::{Digest, Sha256};

#[derive(Debug, clap::Args)]
pub struct Diff {
    /// The recording to compare against, e.g. one of a known-good run
    #[clap(value_name = "OLD")]
    old: String,

    /// The recording to check
    #[clap(value_name = "NEW")]
    new: String,
}

impl crate::command::Executable for Diff {
    fn execute(self) -> eyre::Result<()> {
        let result = run_diff(&self.old, &self.new, &mut std::io::stdout().lock());
        match result {
            Ok(true) => Ok(()),
            Ok(false) => std::process::exit(1),
            Err(err) => {
                // `lib_main` exits with 1 on an error, but here 1 means "the
                // recordings differ", so report the error the same way and
                // exit with 2, as `diff` and `cmp` do.
                crate::print_error(&err);
                std::process::exit(2);
            }
        }
    }
}

/// Each recorded output's messages: one digest per message, in recorded order.
type Outputs = BTreeMap<(String, String), Vec<[u8; 32]>>;

/// Compare the recordings at `old` and `new`, write the result to `out`, and
/// return whether they match.
///
/// A failed write is an error, not a panic as with `println!`, so a closed
/// stdout still exits with 2 instead of Rust's panic status.
fn run_diff(old: &str, new: &str, out: &mut impl Write) -> eyre::Result<bool> {
    let old_outputs = read_file(old)?;
    let new_outputs = read_file(new)?;

    let differences = compare(&old_outputs, &new_outputs);
    if differences.is_empty() {
        let messages: usize = old_outputs.values().map(Vec::len).sum();
        writeln!(
            out,
            "Recordings match: {} on {}",
            count(messages, "message"),
            count(old_outputs.len(), "output")
        )
        .wrap_err("failed to write the result")?;
        return Ok(true);
    }
    for line in &differences {
        writeln!(out, "{line}").wrap_err("failed to write the result")?;
    }
    Ok(false)
}

fn read_file(path: &str) -> eyre::Result<Outputs> {
    let file = File::open(path).wrap_err_with(|| eyre!("failed to open recording `{path}`"))?;
    let mut reader = RecordingReader::open(file)
        .wrap_err_with(|| eyre!("failed to read the header of recording `{path}`"))?;
    let outputs =
        read_outputs(&mut reader).wrap_err_with(|| eyre!("failed to read recording `{path}`"))?;
    // The reader ends an unfinished recording early without an error, so its
    // missing last messages show up below only as a count difference; say why.
    if !reader.reached_footer() {
        eprintln!(
            "warning: recording `{path}` has no footer: it was not finished cleanly \
             (a crash or a second Ctrl-C) and may be missing its last messages"
        );
    }
    Ok(outputs)
}

/// Read every recorded output message, keeping one SHA-256 digest of its
/// payload.
///
/// A record that does not decode fails the whole comparison. `dora recording
/// export` skips such a record, but here a skipped message could hide a
/// difference, and the diff would report two recordings as matching without
/// having read them.
fn read_outputs<R: Read>(reader: &mut RecordingReader<R>) -> eyre::Result<Outputs> {
    let mut outputs = Outputs::new();
    while let Some(entry) = reader.next_entry()? {
        let event = Timestamped::deserialize_inter_daemon_event(&entry.event_bytes).wrap_err_with(
            || {
                eyre!(
                    "failed to decode a message of `{}/{}`",
                    entry.node_id,
                    entry.output_id
                )
            },
        )?;
        // Only `Output` events are messages; the recorder writes nothing else.
        let InterDaemonEvent::Output { data, .. } = event.inner else {
            continue;
        };
        let mut digest = Sha256::new();
        // A message without data hashes nothing; a byte in front of any data,
        // even empty data, keeps the two apart.
        if let Some(data) = &data {
            digest.update([1]);
            digest.update(data.as_slice());
        }
        outputs
            .entry((entry.node_id, entry.output_id))
            .or_default()
            .push(digest.finalize().into());
    }
    Ok(outputs)
}

/// The lines saying how each output differs between `old` and `new` (one or
/// two per output); empty when the recordings match.
fn compare(old: &Outputs, new: &Outputs) -> Vec<String> {
    let mut lines = Vec::new();
    for (key @ (node, output), old_messages) in old {
        let Some(new_messages) = new.get(key) else {
            lines.push(format!(
                "`{node}/{output}`: only in the old recording ({})",
                count(old_messages.len(), "message")
            ));
            continue;
        };
        if old_messages.len() != new_messages.len() {
            lines.push(format!(
                "`{node}/{output}`: {} in the old recording, {} in the new one",
                count(old_messages.len(), "message"),
                new_messages.len()
            ));
        }
        let mut changed = old_messages
            .iter()
            .zip(new_messages)
            .enumerate()
            .filter(|(_, (old, new))| old != new)
            .map(|(index, _)| index);
        if let Some(first) = changed.next() {
            let compared = old_messages.len().min(new_messages.len());
            lines.push(format!(
                "`{node}/{output}`: {}/{compared} messages differ (first: message {})",
                changed.count() + 1,
                first + 1
            ));
        }
    }
    for (key @ (node, output), new_messages) in new {
        if !old.contains_key(key) {
            lines.push(format!(
                "`{node}/{output}`: only in the new recording ({})",
                count(new_messages.len(), "message")
            ));
        }
    }
    lines
}

/// `n` and `noun`, with an `s` unless `n` is 1.
fn count(n: usize, noun: &str) -> String {
    if n == 1 {
        format!("{n} {noun}")
    } else {
        format!("{n} {noun}s")
    }
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::{Outputs, compare, read_outputs, run_diff};
    use aligned_vec::{AVec, ConstAlign};
    use dora_message::{
        common::Timestamped,
        daemon_to_daemon::InterDaemonEvent,
        id::{DataId, NodeId},
        metadata::{Metadata, MetadataParameters, Parameter},
        uhlc::{ID, NTP64, Timestamp},
    };
    use dora_recording::{
        FORMAT_VERSION, RecordEntry, RecordingHeader, RecordingReader, RecordingWriter,
    };
    use uuid::Uuid;

    fn timestamp(time: u64) -> Timestamp {
        Timestamp::new(NTP64(time), ID::try_from(&[7u8; 16]).expect("non-zero id"))
    }

    /// A recorded `Output` event, encoded the way the record node writes it.
    /// `run` stands for everything that changes from run to run: the
    /// timestamps and the dataflow id.
    fn output_event(
        node_id: &str,
        output_id: &str,
        data: Option<&[u8]>,
        parameters: MetadataParameters,
        run: u64,
    ) -> Vec<u8> {
        Timestamped {
            inner: InterDaemonEvent::Output {
                dataflow_id: Uuid::from_u128(run.into()),
                node_id: NodeId::from(node_id.to_string()),
                output_id: DataId::from(output_id.to_string()),
                metadata: Metadata::from_parameters(timestamp(run), parameters),
                data: data.map(|data| AVec::<u8, ConstAlign<128>>::from_slice(128, data)),
            },
            timestamp: timestamp(run + 1),
        }
        .serialize()
        .expect("serialize output event")
    }

    /// A recorded `Output` entry, as [`output_event`] encodes it.
    fn entry(
        node_id: &str,
        output_id: &str,
        data: Option<&[u8]>,
        parameters: MetadataParameters,
        run: u64,
    ) -> RecordEntry {
        RecordEntry {
            node_id: node_id.to_string(),
            output_id: output_id.to_string(),
            timestamp_offset_nanos: run,
            event_bytes: output_event(node_id, output_id, data, parameters, run),
        }
    }

    /// A recorded message with `data` and no metadata parameters.
    fn message(node_id: &str, output_id: &str, data: &[u8], run: u64) -> RecordEntry {
        entry(
            node_id,
            output_id,
            Some(data),
            MetadataParameters::new(),
            run,
        )
    }

    /// The bytes of a finished recording of run `run` holding `entries`.
    fn recording(entries: &[RecordEntry], run: u64) -> Vec<u8> {
        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: run,
            dataflow_id: Uuid::from_u128(run.into()),
            descriptor_yaml: format!("# run {run}").into_bytes(),
        };
        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).expect("init writer");
        for entry in entries {
            writer.write_entry(entry).expect("write entry");
        }
        writer.finish().expect("finish recording");
        buf
    }

    /// Write `entries` into an in-memory recording of run `run` and read it back.
    fn try_read(entries: &[RecordEntry], run: u64) -> eyre::Result<Outputs> {
        let buf = recording(entries, run);
        read_outputs(&mut RecordingReader::open(Cursor::new(buf)).expect("open recording"))
    }

    fn read(entries: &[RecordEntry], run: u64) -> Outputs {
        try_read(entries, run).expect("read recording")
    }

    #[test]
    fn two_runs_with_the_same_messages_match() {
        let old = read(
            &[
                message("camera", "image", b"a", 1),
                message("lidar", "points", b"x", 1),
                message("camera", "image", b"b", 1),
            ],
            1,
        );
        // Different start time, dataflow id, timestamps, descriptor and
        // interleaving of the two outputs.
        let new = read(
            &[
                message("camera", "image", b"a", 2),
                message("camera", "image", b"b", 2),
                message("lidar", "points", b"x", 2),
            ],
            2,
        );
        assert_eq!(compare(&old, &new), Vec::<String>::new());
    }

    #[test]
    fn changed_messages_are_counted_with_the_first_one() {
        let old = read(
            &[
                message("camera", "image", b"a", 1),
                message("camera", "image", b"b", 1),
                message("camera", "image", b"c", 1),
                message("camera", "image", b"d", 1),
            ],
            1,
        );
        let new = read(
            &[
                message("camera", "image", b"a", 2),
                message("camera", "image", b"B", 2),
                message("camera", "image", b"c", 2),
                message("camera", "image", b"D", 2),
            ],
            2,
        );
        assert_eq!(
            compare(&old, &new),
            ["`camera/image`: 2/4 messages differ (first: message 2)"]
        );
    }

    #[test]
    fn a_different_message_count_is_reported() {
        let old = read(
            &[
                message("camera", "image", b"a", 1),
                message("camera", "image", b"b", 1),
            ],
            1,
        );
        let new = read(&[message("camera", "image", b"a", 2)], 2);
        assert_eq!(
            compare(&old, &new),
            ["`camera/image`: 2 messages in the old recording, 1 in the new one"]
        );
        assert_eq!(
            compare(&new, &old),
            ["`camera/image`: 1 message in the old recording, 2 in the new one"]
        );
    }

    #[test]
    fn a_different_count_and_changed_messages_are_both_reported() {
        let old = read(
            &[
                message("camera", "image", b"a", 1),
                message("camera", "image", b"b", 1),
            ],
            1,
        );
        let new = read(&[message("camera", "image", b"x", 2)], 2);
        assert_eq!(
            compare(&old, &new),
            [
                "`camera/image`: 2 messages in the old recording, 1 in the new one",
                "`camera/image`: 1/1 messages differ (first: message 1)",
            ]
        );
    }

    #[test]
    fn an_output_in_only_one_recording_is_reported() {
        let old = read(
            &[
                message("camera", "image", b"a", 1),
                message("lidar", "points", b"x", 1),
            ],
            1,
        );
        let new = read(
            &[
                message("camera", "image", b"a", 2),
                message("imu", "accel", b"1", 2),
                message("imu", "accel", b"2", 2),
            ],
            2,
        );
        assert_eq!(
            compare(&old, &new),
            [
                "`lidar/points`: only in the old recording (1 message)",
                "`imu/accel`: only in the new recording (2 messages)",
            ]
        );
    }

    /// Outputs are keyed by node *and* output: the same output name on another
    /// node is another output.
    #[test]
    fn the_same_output_name_on_another_node_is_another_output() {
        let old = read(&[message("left", "image", b"a", 1)], 1);
        let new = read(&[message("right", "image", b"a", 2)], 2);
        assert_eq!(
            compare(&old, &new),
            [
                "`left/image`: only in the old recording (1 message)",
                "`right/image`: only in the new recording (1 message)",
            ]
        );
    }

    #[test]
    fn a_message_without_data_differs_from_one_with_empty_data() {
        let tick = |data: Option<&[u8]>, run| {
            entry("source", "tick", data, MetadataParameters::new(), run)
        };
        let none = read(&[tick(None, 1)], 1);
        let empty = read(&[tick(Some(&[]), 2)], 2);
        assert_eq!(
            compare(&none, &empty),
            ["`source/tick`: 1/1 messages differ (first: message 1)"]
        );
        let none_again = read(&[tick(None, 3)], 3);
        assert_eq!(compare(&none, &none_again), Vec::<String>::new());
    }

    /// `request_id`s and `goal_id`s are new on every run, so the metadata
    /// parameters are not part of a message's data.
    #[test]
    fn metadata_parameters_are_not_compared() {
        let request = |request_id: &str, run| {
            let parameters = MetadataParameters::from([(
                "request_id".to_string(),
                Parameter::String(request_id.to_string()),
            )]);
            entry("client", "request", Some(b"same"), parameters, run)
        };
        let old = read(&[request("first-run-id", 1)], 1);
        let new = read(&[request("second-run-id", 2)], 2);
        assert_eq!(compare(&old, &new), Vec::<String>::new());
    }

    /// The record node writes only `Output` events; anything else in a
    /// recording is not a message and must not count as one.
    #[test]
    fn events_other_than_outputs_are_not_messages() {
        let closed = RecordEntry {
            node_id: "camera".to_string(),
            output_id: "image".to_string(),
            timestamp_offset_nanos: 5,
            event_bytes: Timestamped {
                inner: InterDaemonEvent::OutputClosed {
                    dataflow_id: Uuid::nil(),
                    node_id: NodeId::from("camera".to_string()),
                    output_id: DataId::from("image".to_string()),
                },
                timestamp: timestamp(5),
            }
            .serialize()
            .expect("serialize output closed"),
        };
        let old = read(&[message("camera", "image", b"a", 1), closed], 1);
        let new = read(&[message("camera", "image", b"a", 2)], 2);
        assert_eq!(compare(&old, &new), Vec::<String>::new());
    }

    /// A record that does not decode could hide a difference, so it fails the
    /// comparison instead of being skipped.
    #[test]
    fn an_undecodable_record_is_an_error() {
        let garbage = RecordEntry {
            node_id: "camera".to_string(),
            output_id: "image".to_string(),
            timestamp_offset_nanos: 2,
            event_bytes: vec![0xff; 16],
        };
        let err = try_read(&[message("camera", "image", b"a", 1), garbage], 1)
            .expect_err("an undecodable record must fail the comparison");
        assert_eq!(
            err.to_string(),
            "failed to decode a message of `camera/image`"
        );
    }

    /// `run_diff` on two files: what it prints, and whether they match.
    fn diff_files(old: &[RecordEntry], new: &[RecordEntry]) -> (bool, String) {
        let dir = tempfile::tempdir().expect("tempdir");
        let old_path = dir.path().join("old.drec");
        let new_path = dir.path().join("new.drec");
        std::fs::write(&old_path, recording(old, 1)).expect("write old");
        std::fs::write(&new_path, recording(new, 2)).expect("write new");
        let mut out = Vec::new();
        let same = run_diff(
            old_path.to_str().expect("utf-8 path"),
            new_path.to_str().expect("utf-8 path"),
            &mut out,
        )
        .expect("diff");
        (same, String::from_utf8(out).expect("utf-8 output"))
    }

    #[test]
    fn matching_files_print_a_summary() {
        let entries = |run| {
            [
                message("camera", "image", b"a", run),
                message("camera", "image", b"b", run),
                message("lidar", "points", b"x", run),
            ]
        };
        assert_eq!(
            diff_files(&entries(1), &entries(2)),
            (
                true,
                "Recordings match: 3 messages on 2 outputs\n".to_string()
            )
        );
        assert_eq!(
            diff_files(
                &[message("imu", "accel", b"1", 1)],
                &[message("imu", "accel", b"1", 2)]
            ),
            (
                true,
                "Recordings match: 1 message on 1 output\n".to_string()
            )
        );
    }

    #[test]
    fn differing_files_print_one_line_per_difference() {
        assert_eq!(
            diff_files(
                &[message("camera", "image", b"a", 1)],
                &[message("lidar", "points", b"x", 2)],
            ),
            (
                false,
                "`camera/image`: only in the old recording (1 message)\n\
                 `lidar/points`: only in the new recording (1 message)\n"
                    .to_string()
            )
        );
    }

    /// A closed stdout must end in an error, so the command exits with 2, not
    /// with the status of a `println!` panic.
    #[test]
    fn a_failed_write_is_an_error() {
        struct Closed;
        impl std::io::Write for Closed {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::ErrorKind::BrokenPipe.into())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let dir = tempfile::tempdir().expect("tempdir");
        let file = |name: &str, data: &[u8]| {
            let path = dir.path().join(name);
            std::fs::write(&path, recording(&[message("camera", "image", data, 1)], 1))
                .expect("write recording");
            path.to_str().expect("utf-8 path").to_string()
        };
        let (a, b) = (file("a.drec", b"a"), file("b.drec", b"b"));
        // Both the summary of a match and the lines of a difference.
        for (old, new) in [(&a, &a), (&a, &b)] {
            let err = run_diff(old, new, &mut Closed).expect_err("a failed write must be an error");
            assert_eq!(err.to_string(), "failed to write the result");
        }
    }
}
